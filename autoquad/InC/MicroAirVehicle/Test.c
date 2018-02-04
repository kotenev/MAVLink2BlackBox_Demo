
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
            return e_MAV_CMD_MAV_CMD_AQ_NAV_LEG_ORBIT;
        case 1:
            return e_MAV_CMD_MAV_CMD_AQ_TELEMETRY;
        case 2:
            return e_MAV_CMD_MAV_CMD_AQ_REQUEST_VERSION;
        case 3:
            return e_MAV_CMD_MAV_CMD_NAV_WAYPOINT;
        case 4:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM;
        case 5:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS;
        case 6:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME;
        case 7:
            return e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH;
        case 8:
            return e_MAV_CMD_MAV_CMD_NAV_LAND;
        case 9:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF;
        case 10:
            return e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL;
        case 11:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL;
        case 12:
            return e_MAV_CMD_MAV_CMD_NAV_FOLLOW;
        case 13:
            return e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
        case 14:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT;
        case 15:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW;
        case 16:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION;
        case 17:
            return e_MAV_CMD_MAV_CMD_NAV_ROI;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 23:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 24:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 25:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 28:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 29:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 30:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 56:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 59:
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 63:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 64:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 65:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 68:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 69:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 70:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 71:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 72:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 73:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 74:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 75:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 76:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 77:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 80:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 81:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 83:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 84:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 85:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 86:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 87:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 88:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 89:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 90:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 93:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 94:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 95:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 96:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 97:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 98:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 99:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 100:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 101:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 102:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 103:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 104:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 105:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 109:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 110:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 111:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 112:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 113:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 114:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 117:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 118:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 119:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 122:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 123:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 124:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 127:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 128:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 129:
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
            return e_MAV_CMD_MAV_CMD_AQ_NAV_LEG_ORBIT;
        case 1:
            return e_MAV_CMD_MAV_CMD_AQ_TELEMETRY;
        case 2:
            return e_MAV_CMD_MAV_CMD_AQ_REQUEST_VERSION;
        case 3:
            return e_MAV_CMD_MAV_CMD_NAV_WAYPOINT;
        case 4:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM;
        case 5:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS;
        case 6:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME;
        case 7:
            return e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH;
        case 8:
            return e_MAV_CMD_MAV_CMD_NAV_LAND;
        case 9:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF;
        case 10:
            return e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL;
        case 11:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL;
        case 12:
            return e_MAV_CMD_MAV_CMD_NAV_FOLLOW;
        case 13:
            return e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
        case 14:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT;
        case 15:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW;
        case 16:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION;
        case 17:
            return e_MAV_CMD_MAV_CMD_NAV_ROI;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 23:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 24:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 25:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 28:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 29:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 30:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 56:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 59:
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 63:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 64:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 65:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 68:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 69:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 70:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 71:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 72:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 73:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 74:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 75:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 76:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 77:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 80:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 81:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 83:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 84:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 85:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 86:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 87:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 88:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 89:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 90:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 93:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 94:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 95:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 96:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 97:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 98:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 99:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 100:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 101:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 102:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 103:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 104:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 105:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 109:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 110:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 111:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 112:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 113:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 114:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 117:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 118:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 119:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 122:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 123:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 124:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 127:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 128:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 129:
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
INLINER void p139_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
/**
*Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
*	this field to difference between instances*/
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
*	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
*	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
*	mixer to repurpose them as generic outputs*/
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
*	this field to difference between instances*/
INLINER void p140_group_mlx_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
/**
*Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
*	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
*	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
*	mixer to repurpose them as generic outputs*/
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
*	local altitude change). The only guarantee on this field is that it will never be reset and is consistent
*	within a flight. The recommended value for this field is the uncorrected barometric altitude at boot
*	time. This altitude will also drift and vary between flights*/
INLINER void p141_altitude_monotonic_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
/**
*This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events
*	like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints
*	are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL
*	by default and not the WGS84 altitude*/
INLINER void p141_altitude_amsl_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
/**
*This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
*	to the coordinate origin (0, 0, 0). It is up-positive*/
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
*	than -1000 should be interpreted as unknown*/
INLINER void p141_altitude_terrain_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
/**
*This is not the altitude, but the clear space below the system according to the fused clearance estimate.
*	It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving
*	target. A negative value indicates no measurement available*/
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
*	on the URI type enum*/
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
*	has a storage associated (e.g. MAVLink FTP)*/
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
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_INVALID);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p0_custom_mode_GET(pack) == (uint32_t)2221666408L);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_PARAFOIL);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_STANDBY);
    assert(p0_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED);
};


void c_TEST_Channel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_onboard_control_sensors_health_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)5868);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t) -12000);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)47409);
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)3178);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)56723);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)16852);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t)113);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)55054);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)51574);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)3304);
    assert(p1_onboard_control_sensors_present_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION);
};


void c_TEST_Channel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)3533646915999010566L);
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)2303755343L);
};


void c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)37257);
    assert(p3_afx_GET(pack) == (float) -3.1696549E38F);
    assert(p3_afz_GET(pack) == (float)2.60396E38F);
    assert(p3_x_GET(pack) == (float)1.9504634E37F);
    assert(p3_yaw_GET(pack) == (float)2.2967965E38F);
    assert(p3_y_GET(pack) == (float) -1.8739448E38F);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)1354866517L);
    assert(p3_vy_GET(pack) == (float)3.2471353E38F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_INT);
    assert(p3_vz_GET(pack) == (float)2.6061847E36F);
    assert(p3_yaw_rate_GET(pack) == (float)2.4300759E38F);
    assert(p3_afy_GET(pack) == (float) -2.802668E38F);
    assert(p3_vx_GET(pack) == (float)1.9356047E37F);
    assert(p3_z_GET(pack) == (float)8.443878E35F);
};


void c_TEST_Channel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_seq_GET(pack) == (uint32_t)1143921337L);
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)250);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)112);
    assert(p4_time_usec_GET(pack) == (uint64_t)867110182192984774L);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)201);
    assert(p5_passkey_LEN(ph) == 5);
    {
        char16_t * exemplary = u"wpazG";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)146);
};


void c_TEST_Channel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 16);
    {
        char16_t * exemplary = u"bauzMlzizznhduwz";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)172);
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_AUTO_ARMED);
    assert(p11_custom_mode_GET(pack) == (uint32_t)2054899685L);
};


void c_TEST_Channel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)220);
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)2);
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t) -21103);
    assert(p20_param_id_LEN(ph) == 2);
    {
        char16_t * exemplary = u"by";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)51);
};


void c_TEST_Channel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_value_GET(pack) == (float) -7.657659E37F);
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)14461);
    assert(p22_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"q";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64);
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)64985);
};


void c_TEST_Channel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p23_param_id_LEN(ph) == 8);
    {
        char16_t * exemplary = u"kmKzBaia";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64);
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p23_param_value_GET(pack) == (float)2.1027516E38F);
};


void c_TEST_Channel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_v_acc_TRY(ph) == (uint32_t)3700695610L);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t)1786262293);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)132);
    assert(p24_alt_GET(pack) == (int32_t)1127467116);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC);
    assert(p24_lon_GET(pack) == (int32_t)598711829);
    assert(p24_h_acc_TRY(ph) == (uint32_t)56114960L);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)3104);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)1481388067L);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)41090);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)49693);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)34655);
    assert(p24_time_usec_GET(pack) == (uint64_t)1392793432039439373L);
    assert(p24_lat_GET(pack) == (int32_t)2142856508);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)3399672932L);
};


void c_TEST_Channel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)234, (uint8_t)185, (uint8_t)80, (uint8_t)105, (uint8_t)108, (uint8_t)128, (uint8_t)154, (uint8_t)87, (uint8_t)96, (uint8_t)131, (uint8_t)66, (uint8_t)205, (uint8_t)196, (uint8_t)184, (uint8_t)201, (uint8_t)74, (uint8_t)4, (uint8_t)132, (uint8_t)221, (uint8_t)127} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)93, (uint8_t)54, (uint8_t)37, (uint8_t)101, (uint8_t)194, (uint8_t)106, (uint8_t)85, (uint8_t)177, (uint8_t)185, (uint8_t)118, (uint8_t)91, (uint8_t)103, (uint8_t)237, (uint8_t)113, (uint8_t)185, (uint8_t)77, (uint8_t)243, (uint8_t)129, (uint8_t)221, (uint8_t)189} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)235, (uint8_t)161, (uint8_t)140, (uint8_t)217, (uint8_t)128, (uint8_t)127, (uint8_t)17, (uint8_t)242, (uint8_t)150, (uint8_t)181, (uint8_t)49, (uint8_t)2, (uint8_t)244, (uint8_t)162, (uint8_t)56, (uint8_t)141, (uint8_t)22, (uint8_t)175, (uint8_t)168, (uint8_t)131} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)125, (uint8_t)248, (uint8_t)201, (uint8_t)252, (uint8_t)185, (uint8_t)20, (uint8_t)224, (uint8_t)5, (uint8_t)93, (uint8_t)78, (uint8_t)97, (uint8_t)26, (uint8_t)154, (uint8_t)8, (uint8_t)177, (uint8_t)197, (uint8_t)50, (uint8_t)135, (uint8_t)38, (uint8_t)143} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)38, (uint8_t)232, (uint8_t)181, (uint8_t)14, (uint8_t)233, (uint8_t)30, (uint8_t)122, (uint8_t)133, (uint8_t)245, (uint8_t)1, (uint8_t)62, (uint8_t)141, (uint8_t)141, (uint8_t)60, (uint8_t)126, (uint8_t)98, (uint8_t)155, (uint8_t)214, (uint8_t)186, (uint8_t)107} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)16);
};


void c_TEST_Channel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t) -9569);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)1630607337L);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t) -23150);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t)28101);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t) -14826);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t) -9459);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t) -25948);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t) -987);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t)13130);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t)13209);
};


void c_TEST_Channel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t) -10507);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t) -27646);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t) -1);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t)13102);
    assert(p27_time_usec_GET(pack) == (uint64_t)3160921651594694711L);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t) -27246);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t) -32572);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t) -14116);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t)30834);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t) -26325);
};


void c_TEST_Channel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_time_usec_GET(pack) == (uint64_t)5019872799998961054L);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t)22104);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t)25917);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t)9778);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t) -5702);
};


void c_TEST_Channel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)1590722016L);
    assert(p29_press_abs_GET(pack) == (float) -2.4840997E38F);
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t)28587);
    assert(p29_press_diff_GET(pack) == (float)5.936642E37F);
};


void c_TEST_Channel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)2790532839L);
    assert(p30_roll_GET(pack) == (float) -2.4379874E38F);
    assert(p30_rollspeed_GET(pack) == (float)2.2957793E38F);
    assert(p30_pitchspeed_GET(pack) == (float)2.5860422E37F);
    assert(p30_yawspeed_GET(pack) == (float)1.11737E38F);
    assert(p30_yaw_GET(pack) == (float)3.1624132E38F);
    assert(p30_pitch_GET(pack) == (float)2.8465143E38F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_q2_GET(pack) == (float) -1.5882945E38F);
    assert(p31_rollspeed_GET(pack) == (float) -1.6666187E38F);
    assert(p31_yawspeed_GET(pack) == (float) -2.3087477E38F);
    assert(p31_q3_GET(pack) == (float) -2.8676433E38F);
    assert(p31_pitchspeed_GET(pack) == (float)1.1345294E38F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)1462098181L);
    assert(p31_q1_GET(pack) == (float)3.1284992E38F);
    assert(p31_q4_GET(pack) == (float) -2.526291E37F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_x_GET(pack) == (float) -1.5064892E38F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)1438173946L);
    assert(p32_y_GET(pack) == (float)1.9394906E38F);
    assert(p32_z_GET(pack) == (float)1.3732364E37F);
    assert(p32_vy_GET(pack) == (float)4.8915863E37F);
    assert(p32_vz_GET(pack) == (float)1.9013037E38F);
    assert(p32_vx_GET(pack) == (float) -2.9538447E37F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_lon_GET(pack) == (int32_t) -54814593);
    assert(p33_alt_GET(pack) == (int32_t) -1323332920);
    assert(p33_lat_GET(pack) == (int32_t)270299175);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t)15901);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)63253);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t) -910);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)2219127252L);
    assert(p33_relative_alt_GET(pack) == (int32_t)1593336398);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t) -6960);
};


void c_TEST_Channel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t) -18292);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t) -29592);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t)6371);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t) -19971);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t)3103);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)4240847923L);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t)29501);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t)32753);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)72);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t)24150);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)63);
};


void c_TEST_Channel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)159);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)310482535L);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)26541);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)33018);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)7917);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)41192);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)51915);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)33404);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)16017);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)35893);
};


void c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)4100);
    assert(p36_time_usec_GET(pack) == (uint32_t)219674282L);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)13947);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)11248);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)12866);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)19360);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)49800);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)13929);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)20716);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)18098);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)41092);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)18096);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)25907);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)47435);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)57584);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)64010);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)17700);
};


void c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t)3445);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)73);
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t) -9336);
};


void c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t) -11920);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)40);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t) -25300);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
};


void c_TEST_Channel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p39_z_GET(pack) == (float)6.3445923E37F);
    assert(p39_param3_GET(pack) == (float)2.6416642E38F);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p39_param4_GET(pack) == (float)2.7659544E38F);
    assert(p39_param2_GET(pack) == (float) -3.1104521E38F);
    assert(p39_x_GET(pack) == (float) -2.3200975E38F);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)181);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_STORAGE_FORMAT);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)0);
    assert(p39_param1_GET(pack) == (float)3.3293117E38F);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)124);
    assert(p39_y_GET(pack) == (float) -2.933072E37F);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)10393);
};


void c_TEST_Channel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)64477);
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)248);
};


void c_TEST_Channel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)159);
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)33224);
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)194);
};


void c_TEST_Channel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)29654);
};


void c_TEST_Channel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)159);
};


void c_TEST_Channel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)211);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)13739);
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)6);
};


void c_TEST_Channel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)220);
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)109);
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
};


void c_TEST_Channel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)6784);
};


void c_TEST_Channel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM3);
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)221);
};


void c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_latitude_GET(pack) == (int32_t) -1816248353);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)10);
    assert(p48_time_usec_TRY(ph) == (uint64_t)1724899279251753948L);
    assert(p48_longitude_GET(pack) == (int32_t) -1653045921);
    assert(p48_altitude_GET(pack) == (int32_t)729989718);
};


void c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_time_usec_TRY(ph) == (uint64_t)8488947208253689794L);
    assert(p49_latitude_GET(pack) == (int32_t) -1437309319);
    assert(p49_altitude_GET(pack) == (int32_t)430531497);
    assert(p49_longitude_GET(pack) == (int32_t)800932314);
};


void c_TEST_Channel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_param_value0_GET(pack) == (float) -6.0586093E37F);
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t)29735);
    assert(p50_param_id_LEN(ph) == 3);
    {
        char16_t * exemplary = u"vhz";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p50_param_value_max_GET(pack) == (float) -8.183716E37F);
    assert(p50_scale_GET(pack) == (float)4.0614843E37F);
    assert(p50_param_value_min_GET(pack) == (float)2.5909898E38F);
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)125);
};


void c_TEST_Channel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)57952);
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
};


void c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_p1z_GET(pack) == (float)1.1577633E38F);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p54_p1x_GET(pack) == (float) -6.402852E37F);
    assert(p54_p2z_GET(pack) == (float)3.3855867E37F);
    assert(p54_p2x_GET(pack) == (float)3.0362597E38F);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)132);
    assert(p54_p2y_GET(pack) == (float) -1.2204764E38F);
    assert(p54_p1y_GET(pack) == (float) -6.152469E37F);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
};


void c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p2z_GET(pack) == (float)2.9955683E38F);
    assert(p55_p1z_GET(pack) == (float) -2.024351E38F);
    assert(p55_p2x_GET(pack) == (float)2.470024E38F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_NED);
    assert(p55_p1y_GET(pack) == (float) -1.7954867E38F);
    assert(p55_p1x_GET(pack) == (float) -3.1577661E38F);
    assert(p55_p2y_GET(pack) == (float)3.155085E38F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    assert(p61_time_usec_GET(pack) == (uint64_t)6478808968165330614L);
    assert(p61_yawspeed_GET(pack) == (float) -2.4428547E37F);
    {
        float exemplary[] =  {-1.6737202E38F, 1.7702637E37F, -5.058709E37F, -2.8162312E38F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_pitchspeed_GET(pack) == (float)1.0662435E38F);
    {
        float exemplary[] =  {2.0060321E38F, -6.8210646E37F, -1.6508405E38F, -2.143125E38F, -1.4958156E38F, -2.058654E38F, 9.324558E37F, -3.1155345E37F, 2.7696539E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_rollspeed_GET(pack) == (float) -9.221051E37F);
};


void c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_alt_error_GET(pack) == (float)3.0493327E38F);
    assert(p62_nav_pitch_GET(pack) == (float)3.1180177E38F);
    assert(p62_aspd_error_GET(pack) == (float)1.2924985E38F);
    assert(p62_nav_roll_GET(pack) == (float)2.5074431E38F);
    assert(p62_xtrack_error_GET(pack) == (float)1.9417908E38F);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)574);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t) -24621);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t) -855);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_vx_GET(pack) == (float)7.609964E37F);
    assert(p63_lat_GET(pack) == (int32_t)2146201980);
    assert(p63_relative_alt_GET(pack) == (int32_t)1570996567);
    assert(p63_lon_GET(pack) == (int32_t) -1413123874);
    assert(p63_vz_GET(pack) == (float)2.1762072E38F);
    assert(p63_alt_GET(pack) == (int32_t) -857341023);
    {
        float exemplary[] =  {-2.577218E38F, -2.6615917E38F, -1.2660305E38F, -2.2077695E38F, 3.9712573E37F, 2.6837086E38F, 1.6574327E38F, 1.5092356E38F, 1.4018865E38F, -3.9068018E37F, 6.2782316E36F, -2.5717694E38F, 1.2485264E38F, -1.8522303E37F, -5.54099E37F, 3.0952582E38F, 2.2744522E38F, 9.932919E37F, 2.9236578E38F, 2.8878294E38F, 3.0720305E38F, -2.317474E38F, -6.341311E37F, -3.0346766E38F, 2.7237666E38F, 2.3980617E38F, 2.5199748E38F, -1.2824314E38F, 1.8069325E38F, -1.8560259E38F, -3.1268454E38F, 4.1166872E37F, 2.9160655E38F, -9.99869E37F, -3.1272737E38F, -7.126759E36F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE);
    assert(p63_time_usec_GET(pack) == (uint64_t)5739622895817135010L);
    assert(p63_vy_GET(pack) == (float) -5.563327E36F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_vy_GET(pack) == (float) -1.3358435E38F);
    assert(p64_ax_GET(pack) == (float) -2.0594071E38F);
    assert(p64_ay_GET(pack) == (float) -9.646949E37F);
    {
        float exemplary[] =  {3.027161E38F, -1.5583886E38F, 8.0492536E36F, -4.869038E37F, -2.5012006E38F, 1.3852462E38F, 2.8684294E38F, 3.0177938E38F, -3.055699E38F, -1.0883535E38F, 1.0534159E38F, -2.9112188E38F, 2.6012467E37F, -3.105432E38F, 3.3805925E38F, -3.184458E37F, 3.775455E37F, -2.752562E38F, 1.744714E38F, 2.3228693E38F, 1.0079246E38F, -5.2534874E37F, 2.3263247E37F, 2.509188E38F, 1.1847145E38F, 9.936895E37F, -1.6030006E38F, 2.0308779E38F, -2.0668889E38F, 1.2749677E38F, -5.056746E37F, -2.2441645E38F, 2.4775392E37F, 1.3275389E38F, 2.7463107E38F, 3.331919E38F, -6.017658E36F, -2.7375775E38F, -3.2139437E38F, 1.2643611E38F, 5.877602E37F, -1.5484435E38F, -5.8073797E37F, 2.805427E38F, -2.798528E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_time_usec_GET(pack) == (uint64_t)1750864055655260234L);
    assert(p64_y_GET(pack) == (float)2.3565984E38F);
    assert(p64_z_GET(pack) == (float)1.9320332E38F);
    assert(p64_vx_GET(pack) == (float) -8.743383E37F);
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS);
    assert(p64_x_GET(pack) == (float)3.1484636E38F);
    assert(p64_vz_GET(pack) == (float)9.234945E37F);
    assert(p64_az_GET(pack) == (float) -4.465342E37F);
};


void c_TEST_Channel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)30308);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)39245);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)63192);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)58911);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)39677);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)20505);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)29253);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)15878);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)27747);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)62491);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)2134229912L);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)16038);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)43217);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)5690);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)43660);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)24714);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)2157);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)27345);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)65434);
};


void c_TEST_Channel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)136);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)131);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)174);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)14020);
};


void c_TEST_Channel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)38574);
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)80);
};


void c_TEST_Channel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_x_GET(pack) == (int16_t)(int16_t)23847);
    assert(p69_r_GET(pack) == (int16_t)(int16_t) -30589);
    assert(p69_z_GET(pack) == (int16_t)(int16_t)1148);
    assert(p69_y_GET(pack) == (int16_t)(int16_t) -9226);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)147);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)36680);
};


void c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)15991);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)22398);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)43814);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)19879);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)51699);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)44348);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)37);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)38246);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)16);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)58350);
};


void c_TEST_Channel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_z_GET(pack) == (float) -2.8730786E38F);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_SPATIAL_USER_1);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)146);
    assert(p73_param3_GET(pack) == (float) -1.999876E38F);
    assert(p73_x_GET(pack) == (int32_t)1303066455);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)21);
    assert(p73_param4_GET(pack) == (float) -9.160898E37F);
    assert(p73_param2_GET(pack) == (float)3.1028336E38F);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)51791);
    assert(p73_param1_GET(pack) == (float)3.2669303E38F);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p73_y_GET(pack) == (int32_t)1548756174);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)25);
};


void c_CommunicationChannel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_groundspeed_GET(pack) == (float)2.4690623E38F);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)24230);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t) -24241);
    assert(p74_climb_GET(pack) == (float)1.8729682E38F);
    assert(p74_airspeed_GET(pack) == (float)3.2499574E38F);
    assert(p74_alt_GET(pack) == (float)3.897891E37F);
};


void c_CommunicationChannel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_param1_GET(pack) == (float) -2.6167545E38F);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p75_x_GET(pack) == (int32_t)2035452180);
    assert(p75_param2_GET(pack) == (float)2.7701027E38F);
    assert(p75_y_GET(pack) == (int32_t) -144001565);
    assert(p75_z_GET(pack) == (float)2.1351842E38F);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)111);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)76);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_START_RX_PAIR);
    assert(p75_param3_GET(pack) == (float) -3.3778355E38F);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)32);
    assert(p75_param4_GET(pack) == (float) -1.252562E38F);
};


void c_CommunicationChannel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_param5_GET(pack) == (float)2.0129981E38F);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p76_param2_GET(pack) == (float) -2.7809843E38F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p76_param3_GET(pack) == (float) -1.1456784E38F);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD);
    assert(p76_param1_GET(pack) == (float)7.4873193E36F);
    assert(p76_param6_GET(pack) == (float) -1.5695167E38F);
    assert(p76_param7_GET(pack) == (float)2.0970926E38F);
    assert(p76_param4_GET(pack) == (float)1.4496854E38F);
};


void c_CommunicationChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_result_param2_TRY(ph) == (int32_t)473233479);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)38);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)177);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)84);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_FAILED);
};


void c_CommunicationChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)2649196596L);
    assert(p81_thrust_GET(pack) == (float) -2.0326719E38F);
    assert(p81_roll_GET(pack) == (float)1.7505255E38F);
    assert(p81_pitch_GET(pack) == (float) -6.2668016E37F);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)227);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p81_yaw_GET(pack) == (float)4.1401243E37F);
};


void c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)243);
    assert(p82_thrust_GET(pack) == (float)2.0440803E38F);
    assert(p82_body_roll_rate_GET(pack) == (float)3.2194123E38F);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)1183948301L);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)73);
    {
        float exemplary[] =  {-7.974049E37F, -5.3335877E37F, -1.5337702E38F, 9.76182E37F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_body_yaw_rate_GET(pack) == (float) -1.7441718E38F);
    assert(p82_body_pitch_rate_GET(pack) == (float) -6.2256415E37F);
};


void c_CommunicationChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)2258915363L);
    assert(p83_thrust_GET(pack) == (float)2.4050034E38F);
    assert(p83_body_roll_rate_GET(pack) == (float)9.896418E37F);
    assert(p83_body_yaw_rate_GET(pack) == (float) -5.544431E37F);
    assert(p83_body_pitch_rate_GET(pack) == (float)1.4259926E37F);
    {
        float exemplary[] =  {3.2538451E38F, 2.5106944E38F, -3.3164577E38F, -3.0150173E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_yaw_GET(pack) == (float)5.4748126E37F);
    assert(p84_vy_GET(pack) == (float)1.614217E38F);
    assert(p84_yaw_rate_GET(pack) == (float) -5.5574444E35F);
    assert(p84_vz_GET(pack) == (float)3.7530188E37F);
    assert(p84_z_GET(pack) == (float)2.5880274E38F);
    assert(p84_afx_GET(pack) == (float) -1.6763401E38F);
    assert(p84_vx_GET(pack) == (float)2.7327547E38F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)2277559524L);
    assert(p84_afy_GET(pack) == (float) -2.8951262E38F);
    assert(p84_afz_GET(pack) == (float)2.9756994E38F);
    assert(p84_y_GET(pack) == (float)3.3038174E37F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)36235);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)63);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)101);
    assert(p84_x_GET(pack) == (float)5.483587E37F);
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_vz_GET(pack) == (float)9.221703E37F);
    assert(p86_afx_GET(pack) == (float) -2.3823445E38F);
    assert(p86_yaw_GET(pack) == (float) -9.403621E37F);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)1587837483L);
    assert(p86_vy_GET(pack) == (float)5.5336245E37F);
    assert(p86_afy_GET(pack) == (float) -6.1872104E37F);
    assert(p86_alt_GET(pack) == (float)1.6357319E38F);
    assert(p86_afz_GET(pack) == (float) -1.935852E38F);
    assert(p86_vx_GET(pack) == (float) -7.566171E37F);
    assert(p86_lat_int_GET(pack) == (int32_t)737468659);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)209);
    assert(p86_lon_int_GET(pack) == (int32_t) -2015101432);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)29620);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)255);
    assert(p86_yaw_rate_GET(pack) == (float) -1.7884633E38F);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_INT);
};


void c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_lat_int_GET(pack) == (int32_t)280732925);
    assert(p87_yaw_GET(pack) == (float)2.625362E37F);
    assert(p87_vy_GET(pack) == (float) -8.941055E37F);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)60862);
    assert(p87_alt_GET(pack) == (float)2.2490724E38F);
    assert(p87_afx_GET(pack) == (float)1.7293835E38F);
    assert(p87_lon_int_GET(pack) == (int32_t) -805140550);
    assert(p87_vz_GET(pack) == (float)2.6864021E38F);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p87_vx_GET(pack) == (float) -5.531325E37F);
    assert(p87_afy_GET(pack) == (float)2.4087607E38F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)1539783912L);
    assert(p87_afz_GET(pack) == (float)2.5208048E38F);
    assert(p87_yaw_rate_GET(pack) == (float) -1.5253487E38F);
};


void c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_roll_GET(pack) == (float)1.3810173E38F);
    assert(p89_yaw_GET(pack) == (float)2.6649792E38F);
    assert(p89_y_GET(pack) == (float) -1.4831896E38F);
    assert(p89_z_GET(pack) == (float) -1.1801731E37F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)2927559060L);
    assert(p89_pitch_GET(pack) == (float)1.173822E36F);
    assert(p89_x_GET(pack) == (float)2.1313748E38F);
};


void c_CommunicationChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_vy_GET(pack) == (int16_t)(int16_t)18084);
    assert(p90_rollspeed_GET(pack) == (float)2.533746E38F);
    assert(p90_alt_GET(pack) == (int32_t) -884847031);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t)3537);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t)24461);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t)24866);
    assert(p90_lon_GET(pack) == (int32_t) -671044021);
    assert(p90_yaw_GET(pack) == (float) -6.3059735E37F);
    assert(p90_pitch_GET(pack) == (float) -5.573744E37F);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t)12193);
    assert(p90_pitchspeed_GET(pack) == (float)6.057332E37F);
    assert(p90_lat_GET(pack) == (int32_t)46033092);
    assert(p90_yawspeed_GET(pack) == (float) -4.6084844E37F);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t)1227);
    assert(p90_time_usec_GET(pack) == (uint64_t)4761075264404981053L);
    assert(p90_roll_GET(pack) == (float) -1.2767851E38F);
};


void c_CommunicationChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_aux1_GET(pack) == (float) -1.8049278E38F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED);
    assert(p91_aux4_GET(pack) == (float) -1.0928701E38F);
    assert(p91_aux3_GET(pack) == (float)3.195682E38F);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)116);
    assert(p91_yaw_rudder_GET(pack) == (float) -1.3171051E37F);
    assert(p91_throttle_GET(pack) == (float) -1.0594665E38F);
    assert(p91_time_usec_GET(pack) == (uint64_t)2521045793685330362L);
    assert(p91_roll_ailerons_GET(pack) == (float)5.772481E37F);
    assert(p91_aux2_GET(pack) == (float)1.1556709E38F);
    assert(p91_pitch_elevator_GET(pack) == (float) -5.4908656E37F);
};


void c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_time_usec_GET(pack) == (uint64_t)2923960796231749404L);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)904);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)63526);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)27518);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)47594);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)52398);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)18176);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)106);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)5742);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)36732);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)994);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)24791);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)15229);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)24444);
};


void c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-2.4814603E38F, -9.052799E37F, -4.6621426E37F, 5.1576155E37F, 1.553543E37F, -1.898137E38F, -6.20488E37F, -7.151764E37F, 3.0546386E36F, 1.0833605E38F, 7.0779484E37F, -1.3312824E38F, 2.552502E38F, 1.8674159E38F, 2.2928092E38F, -2.068036E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_flags_GET(pack) == (uint64_t)8182773568483499392L);
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_MANUAL_ARMED);
    assert(p93_time_usec_GET(pack) == (uint64_t)2059834387021081645L);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)227);
    assert(p100_ground_distance_GET(pack) == (float) -2.5820777E38F);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p100_flow_comp_m_x_GET(pack) == (float) -6.016133E37F);
    assert(p100_flow_rate_x_TRY(ph) == (float) -1.7542165E38F);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t) -20913);
    assert(p100_time_usec_GET(pack) == (uint64_t)1785343245002181180L);
    assert(p100_flow_rate_y_TRY(ph) == (float)1.7968345E38F);
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t)23801);
    assert(p100_flow_comp_m_y_GET(pack) == (float) -2.0152485E38F);
};


void c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_z_GET(pack) == (float) -3.6809995E37F);
    assert(p101_roll_GET(pack) == (float)2.2978327E38F);
    assert(p101_yaw_GET(pack) == (float)2.0164232E38F);
    assert(p101_pitch_GET(pack) == (float) -2.0628685E38F);
    assert(p101_usec_GET(pack) == (uint64_t)8143982069132596138L);
    assert(p101_x_GET(pack) == (float)2.4416875E37F);
    assert(p101_y_GET(pack) == (float)3.0237047E38F);
};


void c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_y_GET(pack) == (float) -2.8366773E37F);
    assert(p102_roll_GET(pack) == (float) -7.945485E37F);
    assert(p102_usec_GET(pack) == (uint64_t)360562127219600640L);
    assert(p102_x_GET(pack) == (float) -2.624997E38F);
    assert(p102_yaw_GET(pack) == (float)2.3130597E38F);
    assert(p102_pitch_GET(pack) == (float)2.3732014E37F);
    assert(p102_z_GET(pack) == (float)1.4480473E38F);
};


void c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_y_GET(pack) == (float)1.6485353E38F);
    assert(p103_z_GET(pack) == (float)1.6859611E37F);
    assert(p103_x_GET(pack) == (float) -1.5815744E37F);
    assert(p103_usec_GET(pack) == (uint64_t)1211192891050447536L);
};


void c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_usec_GET(pack) == (uint64_t)1499602390536947725L);
    assert(p104_yaw_GET(pack) == (float)8.65666E37F);
    assert(p104_x_GET(pack) == (float)2.712264E38F);
    assert(p104_pitch_GET(pack) == (float)2.3223375E38F);
    assert(p104_roll_GET(pack) == (float) -2.2306844E38F);
    assert(p104_z_GET(pack) == (float)1.3048371E38F);
    assert(p104_y_GET(pack) == (float) -2.3945348E38F);
};


void c_CommunicationChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_pressure_alt_GET(pack) == (float) -3.3678343E38F);
    assert(p105_yacc_GET(pack) == (float) -1.305332E38F);
    assert(p105_xmag_GET(pack) == (float)2.971489E37F);
    assert(p105_abs_pressure_GET(pack) == (float) -5.769311E36F);
    assert(p105_temperature_GET(pack) == (float) -3.0959955E37F);
    assert(p105_zacc_GET(pack) == (float)2.261633E38F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)50039);
    assert(p105_zmag_GET(pack) == (float) -8.603375E37F);
    assert(p105_zgyro_GET(pack) == (float)3.2045856E38F);
    assert(p105_ygyro_GET(pack) == (float) -8.76715E37F);
    assert(p105_diff_pressure_GET(pack) == (float) -2.568845E38F);
    assert(p105_xacc_GET(pack) == (float) -2.806662E38F);
    assert(p105_xgyro_GET(pack) == (float)1.6890764E38F);
    assert(p105_time_usec_GET(pack) == (uint64_t)5105223883275752616L);
    assert(p105_ymag_GET(pack) == (float)1.3882769E38F);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_integrated_zgyro_GET(pack) == (float)3.0897326E38F);
    assert(p106_integrated_ygyro_GET(pack) == (float) -7.442806E37F);
    assert(p106_distance_GET(pack) == (float)2.426386E38F);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)2958049646L);
    assert(p106_time_usec_GET(pack) == (uint64_t)6071555478050683830L);
    assert(p106_integrated_x_GET(pack) == (float)5.7730157E37F);
    assert(p106_integrated_y_GET(pack) == (float)3.3956214E38F);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t) -21747);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)2304841550L);
    assert(p106_integrated_xgyro_GET(pack) == (float) -3.0740448E38F);
};


void c_CommunicationChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_xacc_GET(pack) == (float)3.18727E38F);
    assert(p107_pressure_alt_GET(pack) == (float)2.7836918E38F);
    assert(p107_ygyro_GET(pack) == (float) -1.6792121E38F);
    assert(p107_zmag_GET(pack) == (float)2.226621E38F);
    assert(p107_ymag_GET(pack) == (float)2.9680453E38F);
    assert(p107_xmag_GET(pack) == (float) -1.0839854E38F);
    assert(p107_diff_pressure_GET(pack) == (float) -4.536224E37F);
    assert(p107_temperature_GET(pack) == (float) -3.1369649E38F);
    assert(p107_zacc_GET(pack) == (float)3.3195234E38F);
    assert(p107_yacc_GET(pack) == (float) -1.6122638E38F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)2278989202L);
    assert(p107_abs_pressure_GET(pack) == (float)2.8082308E38F);
    assert(p107_time_usec_GET(pack) == (uint64_t)337593339332657511L);
    assert(p107_zgyro_GET(pack) == (float) -3.0461774E38F);
    assert(p107_xgyro_GET(pack) == (float)3.3849098E37F);
};


void c_CommunicationChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_yaw_GET(pack) == (float)2.9995607E37F);
    assert(p108_xacc_GET(pack) == (float) -1.9951344E38F);
    assert(p108_lat_GET(pack) == (float)1.5628176E38F);
    assert(p108_yacc_GET(pack) == (float)7.7047033E37F);
    assert(p108_zacc_GET(pack) == (float)2.058793E38F);
    assert(p108_pitch_GET(pack) == (float) -2.420902E37F);
    assert(p108_ve_GET(pack) == (float) -2.600696E38F);
    assert(p108_std_dev_horz_GET(pack) == (float) -1.8079332E38F);
    assert(p108_q4_GET(pack) == (float)1.531516E38F);
    assert(p108_vn_GET(pack) == (float)2.4430408E38F);
    assert(p108_std_dev_vert_GET(pack) == (float) -9.0092606E36F);
    assert(p108_roll_GET(pack) == (float)6.954744E37F);
    assert(p108_q3_GET(pack) == (float)3.320931E38F);
    assert(p108_alt_GET(pack) == (float) -1.1081859E38F);
    assert(p108_zgyro_GET(pack) == (float)1.952701E37F);
    assert(p108_ygyro_GET(pack) == (float)6.5743793E37F);
    assert(p108_vd_GET(pack) == (float)2.8705745E38F);
    assert(p108_q2_GET(pack) == (float)6.241178E35F);
    assert(p108_xgyro_GET(pack) == (float)2.4323345E38F);
    assert(p108_q1_GET(pack) == (float)2.144288E38F);
    assert(p108_lon_GET(pack) == (float)1.4365794E38F);
};


void c_CommunicationChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)57);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)46347);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)28018);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)236);
};


void c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)65);
    {
        uint8_t exemplary[] =  {(uint8_t)239, (uint8_t)211, (uint8_t)208, (uint8_t)110, (uint8_t)47, (uint8_t)118, (uint8_t)37, (uint8_t)202, (uint8_t)41, (uint8_t)154, (uint8_t)194, (uint8_t)152, (uint8_t)175, (uint8_t)39, (uint8_t)215, (uint8_t)76, (uint8_t)117, (uint8_t)140, (uint8_t)33, (uint8_t)44, (uint8_t)243, (uint8_t)128, (uint8_t)147, (uint8_t)209, (uint8_t)111, (uint8_t)121, (uint8_t)104, (uint8_t)202, (uint8_t)11, (uint8_t)31, (uint8_t)54, (uint8_t)164, (uint8_t)75, (uint8_t)106, (uint8_t)58, (uint8_t)208, (uint8_t)60, (uint8_t)130, (uint8_t)133, (uint8_t)170, (uint8_t)160, (uint8_t)24, (uint8_t)52, (uint8_t)73, (uint8_t)94, (uint8_t)21, (uint8_t)37, (uint8_t)113, (uint8_t)208, (uint8_t)102, (uint8_t)82, (uint8_t)185, (uint8_t)123, (uint8_t)107, (uint8_t)169, (uint8_t)120, (uint8_t)209, (uint8_t)22, (uint8_t)164, (uint8_t)143, (uint8_t)185, (uint8_t)64, (uint8_t)189, (uint8_t)128, (uint8_t)71, (uint8_t)236, (uint8_t)19, (uint8_t)139, (uint8_t)149, (uint8_t)131, (uint8_t)196, (uint8_t)20, (uint8_t)22, (uint8_t)205, (uint8_t)103, (uint8_t)35, (uint8_t)21, (uint8_t)80, (uint8_t)5, (uint8_t)78, (uint8_t)117, (uint8_t)132, (uint8_t)65, (uint8_t)43, (uint8_t)20, (uint8_t)77, (uint8_t)247, (uint8_t)173, (uint8_t)246, (uint8_t)227, (uint8_t)18, (uint8_t)201, (uint8_t)191, (uint8_t)185, (uint8_t)33, (uint8_t)54, (uint8_t)175, (uint8_t)79, (uint8_t)229, (uint8_t)132, (uint8_t)239, (uint8_t)248, (uint8_t)54, (uint8_t)123, (uint8_t)180, (uint8_t)208, (uint8_t)48, (uint8_t)208, (uint8_t)93, (uint8_t)230, (uint8_t)114, (uint8_t)233, (uint8_t)114, (uint8_t)220, (uint8_t)148, (uint8_t)42, (uint8_t)125, (uint8_t)125, (uint8_t)246, (uint8_t)81, (uint8_t)76, (uint8_t)26, (uint8_t)198, (uint8_t)124, (uint8_t)80, (uint8_t)38, (uint8_t)74, (uint8_t)128, (uint8_t)147, (uint8_t)127, (uint8_t)209, (uint8_t)178, (uint8_t)253, (uint8_t)48, (uint8_t)18, (uint8_t)137, (uint8_t)45, (uint8_t)178, (uint8_t)10, (uint8_t)150, (uint8_t)95, (uint8_t)80, (uint8_t)95, (uint8_t)91, (uint8_t)58, (uint8_t)140, (uint8_t)65, (uint8_t)221, (uint8_t)188, (uint8_t)244, (uint8_t)34, (uint8_t)140, (uint8_t)55, (uint8_t)32, (uint8_t)253, (uint8_t)149, (uint8_t)115, (uint8_t)117, (uint8_t)57, (uint8_t)243, (uint8_t)44, (uint8_t)170, (uint8_t)173, (uint8_t)13, (uint8_t)250, (uint8_t)62, (uint8_t)105, (uint8_t)160, (uint8_t)180, (uint8_t)164, (uint8_t)206, (uint8_t)248, (uint8_t)122, (uint8_t)10, (uint8_t)101, (uint8_t)226, (uint8_t)217, (uint8_t)29, (uint8_t)101, (uint8_t)241, (uint8_t)173, (uint8_t)44, (uint8_t)46, (uint8_t)114, (uint8_t)120, (uint8_t)84, (uint8_t)34, (uint8_t)232, (uint8_t)227, (uint8_t)70, (uint8_t)99, (uint8_t)86, (uint8_t)211, (uint8_t)115, (uint8_t)186, (uint8_t)32, (uint8_t)111, (uint8_t)237, (uint8_t)254, (uint8_t)136, (uint8_t)195, (uint8_t)63, (uint8_t)229, (uint8_t)64, (uint8_t)171, (uint8_t)37, (uint8_t)108, (uint8_t)160, (uint8_t)70, (uint8_t)82, (uint8_t)65, (uint8_t)169, (uint8_t)0, (uint8_t)165, (uint8_t)60, (uint8_t)76, (uint8_t)4, (uint8_t)0, (uint8_t)70, (uint8_t)235, (uint8_t)199, (uint8_t)144, (uint8_t)121, (uint8_t)54, (uint8_t)79, (uint8_t)237, (uint8_t)52, (uint8_t)28, (uint8_t)96, (uint8_t)91, (uint8_t)66, (uint8_t)250, (uint8_t)31, (uint8_t)201, (uint8_t)173, (uint8_t)90, (uint8_t)77, (uint8_t)232, (uint8_t)52, (uint8_t)168, (uint8_t)4, (uint8_t)28, (uint8_t)27, (uint8_t)225, (uint8_t)207, (uint8_t)243, (uint8_t)12, (uint8_t)166, (uint8_t)192, (uint8_t)85, (uint8_t)129} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)148);
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)186);
};


void c_CommunicationChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_ts1_GET(pack) == (int64_t) -4103656191883114175L);
    assert(p111_tc1_GET(pack) == (int64_t)633506756755885034L);
};


void c_CommunicationChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_seq_GET(pack) == (uint32_t)3603856836L);
    assert(p112_time_usec_GET(pack) == (uint64_t)6937714871197763264L);
};


void c_CommunicationChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)2034);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t) -1888);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p113_lat_GET(pack) == (int32_t) -2108285371);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)262);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)64427);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t)18960);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t)14623);
    assert(p113_lon_GET(pack) == (int32_t)1011628572);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)20374);
    assert(p113_alt_GET(pack) == (int32_t) -27467952);
    assert(p113_time_usec_GET(pack) == (uint64_t)7520355250268813190L);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)61);
};


void c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t)11305);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)62854891L);
    assert(p114_integrated_x_GET(pack) == (float) -1.647722E38F);
    assert(p114_integrated_xgyro_GET(pack) == (float)1.7939759E38F);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)3510100682L);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)9);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p114_time_usec_GET(pack) == (uint64_t)708509918534386171L);
    assert(p114_distance_GET(pack) == (float) -1.4745204E38F);
    assert(p114_integrated_zgyro_GET(pack) == (float)1.0852622E38F);
    assert(p114_integrated_y_GET(pack) == (float)8.771594E37F);
    assert(p114_integrated_ygyro_GET(pack) == (float) -1.2679214E38F);
};


void c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)794);
    assert(p115_lat_GET(pack) == (int32_t)10842282);
    {
        float exemplary[] =  {4.133478E37F, -2.346324E38F, -5.53602E37F, -2.524565E38F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_vy_GET(pack) == (int16_t)(int16_t) -4171);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t) -2850);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t) -26527);
    assert(p115_yawspeed_GET(pack) == (float)1.5710696E38F);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t)13713);
    assert(p115_alt_GET(pack) == (int32_t)1553581837);
    assert(p115_time_usec_GET(pack) == (uint64_t)2873181721335708150L);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t) -32730);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)34839);
    assert(p115_pitchspeed_GET(pack) == (float)1.8767898E38F);
    assert(p115_rollspeed_GET(pack) == (float)2.429347E36F);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t) -6450);
    assert(p115_lon_GET(pack) == (int32_t)495384339);
};


void c_CommunicationChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t)8407);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t) -16994);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t) -13226);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t)11269);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t)28160);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)4069989961L);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t) -18355);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t)6437);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t)4571);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t) -30997);
};


void c_CommunicationChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)29357);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)211);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)50);
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)55817);
};


void c_CommunicationChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)55689);
    assert(p118_time_utc_GET(pack) == (uint32_t)2340433430L);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)43733);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)39751);
    assert(p118_size_GET(pack) == (uint32_t)348735129L);
};


void c_CommunicationChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)225);
    assert(p119_ofs_GET(pack) == (uint32_t)2466055741L);
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)53383);
    assert(p119_count_GET(pack) == (uint32_t)3635092174L);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)176);
};


void c_CommunicationChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_ofs_GET(pack) == (uint32_t)3343964101L);
    {
        uint8_t exemplary[] =  {(uint8_t)23, (uint8_t)199, (uint8_t)46, (uint8_t)183, (uint8_t)152, (uint8_t)163, (uint8_t)157, (uint8_t)52, (uint8_t)29, (uint8_t)243, (uint8_t)121, (uint8_t)116, (uint8_t)47, (uint8_t)89, (uint8_t)42, (uint8_t)11, (uint8_t)46, (uint8_t)41, (uint8_t)166, (uint8_t)235, (uint8_t)104, (uint8_t)133, (uint8_t)214, (uint8_t)182, (uint8_t)200, (uint8_t)152, (uint8_t)156, (uint8_t)201, (uint8_t)11, (uint8_t)135, (uint8_t)229, (uint8_t)148, (uint8_t)255, (uint8_t)184, (uint8_t)61, (uint8_t)147, (uint8_t)48, (uint8_t)251, (uint8_t)88, (uint8_t)73, (uint8_t)227, (uint8_t)139, (uint8_t)17, (uint8_t)39, (uint8_t)168, (uint8_t)202, (uint8_t)217, (uint8_t)126, (uint8_t)146, (uint8_t)77, (uint8_t)61, (uint8_t)65, (uint8_t)22, (uint8_t)227, (uint8_t)96, (uint8_t)46, (uint8_t)120, (uint8_t)169, (uint8_t)234, (uint8_t)228, (uint8_t)177, (uint8_t)49, (uint8_t)173, (uint8_t)73, (uint8_t)45, (uint8_t)190, (uint8_t)106, (uint8_t)8, (uint8_t)187, (uint8_t)139, (uint8_t)238, (uint8_t)130, (uint8_t)213, (uint8_t)171, (uint8_t)127, (uint8_t)166, (uint8_t)187, (uint8_t)11, (uint8_t)196, (uint8_t)204, (uint8_t)61, (uint8_t)205, (uint8_t)177, (uint8_t)252, (uint8_t)63, (uint8_t)192, (uint8_t)6, (uint8_t)189, (uint8_t)99, (uint8_t)174} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)126);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)30462);
};


void c_CommunicationChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)1);
};


void c_CommunicationChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)109);
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)6);
};


void c_CommunicationChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)16);
    {
        uint8_t exemplary[] =  {(uint8_t)230, (uint8_t)134, (uint8_t)35, (uint8_t)246, (uint8_t)143, (uint8_t)197, (uint8_t)218, (uint8_t)27, (uint8_t)68, (uint8_t)243, (uint8_t)30, (uint8_t)64, (uint8_t)70, (uint8_t)249, (uint8_t)92, (uint8_t)68, (uint8_t)133, (uint8_t)221, (uint8_t)244, (uint8_t)0, (uint8_t)216, (uint8_t)33, (uint8_t)152, (uint8_t)226, (uint8_t)63, (uint8_t)203, (uint8_t)111, (uint8_t)214, (uint8_t)193, (uint8_t)10, (uint8_t)43, (uint8_t)33, (uint8_t)129, (uint8_t)69, (uint8_t)31, (uint8_t)86, (uint8_t)129, (uint8_t)252, (uint8_t)160, (uint8_t)2, (uint8_t)85, (uint8_t)59, (uint8_t)16, (uint8_t)53, (uint8_t)185, (uint8_t)153, (uint8_t)53, (uint8_t)58, (uint8_t)141, (uint8_t)166, (uint8_t)164, (uint8_t)52, (uint8_t)68, (uint8_t)167, (uint8_t)14, (uint8_t)238, (uint8_t)33, (uint8_t)103, (uint8_t)118, (uint8_t)201, (uint8_t)8, (uint8_t)226, (uint8_t)170, (uint8_t)139, (uint8_t)153, (uint8_t)223, (uint8_t)118, (uint8_t)178, (uint8_t)208, (uint8_t)232, (uint8_t)254, (uint8_t)169, (uint8_t)136, (uint8_t)164, (uint8_t)101, (uint8_t)89, (uint8_t)238, (uint8_t)163, (uint8_t)230, (uint8_t)61, (uint8_t)204, (uint8_t)247, (uint8_t)78, (uint8_t)222, (uint8_t)103, (uint8_t)171, (uint8_t)35, (uint8_t)245, (uint8_t)31, (uint8_t)200, (uint8_t)147, (uint8_t)89, (uint8_t)16, (uint8_t)135, (uint8_t)231, (uint8_t)205, (uint8_t)158, (uint8_t)93, (uint8_t)74, (uint8_t)121, (uint8_t)124, (uint8_t)117, (uint8_t)88, (uint8_t)27, (uint8_t)175, (uint8_t)38, (uint8_t)158, (uint8_t)172, (uint8_t)1, (uint8_t)129} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)11);
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)211);
};


void c_CommunicationChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)32529);
    assert(p124_alt_GET(pack) == (int32_t) -587427041);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p124_lat_GET(pack) == (int32_t)1558407623);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)38795);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)64025);
    assert(p124_time_usec_GET(pack) == (uint64_t)7638772046898576182L);
    assert(p124_lon_GET(pack) == (int32_t) -1114929944);
    assert(p124_dgps_age_GET(pack) == (uint32_t)425004508L);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)2595);
};


void c_CommunicationChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)53587);
    assert(p125_flags_GET(pack) == e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID);
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)60433);
};


void c_CommunicationChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)122, (uint8_t)200, (uint8_t)225, (uint8_t)174, (uint8_t)175, (uint8_t)54, (uint8_t)168, (uint8_t)160, (uint8_t)103, (uint8_t)195, (uint8_t)139, (uint8_t)60, (uint8_t)133, (uint8_t)171, (uint8_t)247, (uint8_t)75, (uint8_t)138, (uint8_t)153, (uint8_t)223, (uint8_t)18, (uint8_t)47, (uint8_t)131, (uint8_t)175, (uint8_t)24, (uint8_t)120, (uint8_t)126, (uint8_t)154, (uint8_t)222, (uint8_t)86, (uint8_t)17, (uint8_t)52, (uint8_t)25, (uint8_t)6, (uint8_t)121, (uint8_t)219, (uint8_t)124, (uint8_t)198, (uint8_t)8, (uint8_t)34, (uint8_t)139, (uint8_t)45, (uint8_t)13, (uint8_t)184, (uint8_t)237, (uint8_t)158, (uint8_t)105, (uint8_t)18, (uint8_t)87, (uint8_t)142, (uint8_t)91, (uint8_t)129, (uint8_t)176, (uint8_t)57, (uint8_t)8, (uint8_t)22, (uint8_t)113, (uint8_t)90, (uint8_t)203, (uint8_t)152, (uint8_t)33, (uint8_t)253, (uint8_t)128, (uint8_t)218, (uint8_t)189, (uint8_t)127, (uint8_t)113, (uint8_t)112, (uint8_t)239, (uint8_t)73, (uint8_t)100} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_flags_GET(pack) == e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING);
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)32584);
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1);
    assert(p126_baudrate_GET(pack) == (uint32_t)2602259593L);
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)70);
};


void c_CommunicationChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_baseline_b_mm_GET(pack) == (int32_t)1059834170);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)4290087520L);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)178);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t) -1327894295);
    assert(p127_tow_GET(pack) == (uint32_t)531733435L);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)165);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t) -1158400589);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)56979);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t)1016694760);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p127_accuracy_GET(pack) == (uint32_t)250166633L);
};


void c_CommunicationChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_tow_GET(pack) == (uint32_t)388488657L);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p128_accuracy_GET(pack) == (uint32_t)3528912347L);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)163);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t)442975909);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t)891757458);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)32899);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t)689320771);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)63);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t) -1214304203);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)2585521831L);
};


void c_CommunicationChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)2391013324L);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t) -21251);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t)22602);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t) -14297);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t) -27135);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)22311);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t) -29312);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t) -5876);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t)629);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t)12671);
};


void c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)4726);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)183);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p130_size_GET(pack) == (uint32_t)2768345946L);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)35872);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)34307);
};


void c_CommunicationChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)61541);
    {
        uint8_t exemplary[] =  {(uint8_t)188, (uint8_t)39, (uint8_t)173, (uint8_t)90, (uint8_t)44, (uint8_t)218, (uint8_t)225, (uint8_t)47, (uint8_t)69, (uint8_t)114, (uint8_t)75, (uint8_t)196, (uint8_t)151, (uint8_t)103, (uint8_t)180, (uint8_t)114, (uint8_t)39, (uint8_t)37, (uint8_t)46, (uint8_t)94, (uint8_t)226, (uint8_t)249, (uint8_t)159, (uint8_t)6, (uint8_t)245, (uint8_t)81, (uint8_t)114, (uint8_t)44, (uint8_t)206, (uint8_t)69, (uint8_t)137, (uint8_t)112, (uint8_t)164, (uint8_t)22, (uint8_t)119, (uint8_t)11, (uint8_t)79, (uint8_t)170, (uint8_t)79, (uint8_t)91, (uint8_t)129, (uint8_t)38, (uint8_t)6, (uint8_t)68, (uint8_t)29, (uint8_t)155, (uint8_t)128, (uint8_t)94, (uint8_t)196, (uint8_t)47, (uint8_t)54, (uint8_t)104, (uint8_t)40, (uint8_t)218, (uint8_t)110, (uint8_t)172, (uint8_t)250, (uint8_t)168, (uint8_t)8, (uint8_t)159, (uint8_t)50, (uint8_t)36, (uint8_t)205, (uint8_t)148, (uint8_t)238, (uint8_t)169, (uint8_t)13, (uint8_t)237, (uint8_t)155, (uint8_t)110, (uint8_t)6, (uint8_t)23, (uint8_t)221, (uint8_t)43, (uint8_t)137, (uint8_t)162, (uint8_t)189, (uint8_t)59, (uint8_t)199, (uint8_t)10, (uint8_t)1, (uint8_t)6, (uint8_t)60, (uint8_t)44, (uint8_t)9, (uint8_t)23, (uint8_t)91, (uint8_t)18, (uint8_t)222, (uint8_t)106, (uint8_t)61, (uint8_t)110, (uint8_t)21, (uint8_t)107, (uint8_t)197, (uint8_t)31, (uint8_t)91, (uint8_t)12, (uint8_t)116, (uint8_t)251, (uint8_t)87, (uint8_t)162, (uint8_t)133, (uint8_t)227, (uint8_t)176, (uint8_t)186, (uint8_t)21, (uint8_t)21, (uint8_t)51, (uint8_t)236, (uint8_t)75, (uint8_t)89, (uint8_t)181, (uint8_t)196, (uint8_t)9, (uint8_t)250, (uint8_t)84, (uint8_t)73, (uint8_t)21, (uint8_t)184, (uint8_t)28, (uint8_t)250, (uint8_t)233, (uint8_t)201, (uint8_t)206, (uint8_t)158, (uint8_t)62, (uint8_t)8, (uint8_t)81, (uint8_t)217, (uint8_t)41, (uint8_t)78, (uint8_t)70, (uint8_t)251, (uint8_t)214, (uint8_t)96, (uint8_t)70, (uint8_t)192, (uint8_t)166, (uint8_t)126, (uint8_t)136, (uint8_t)198, (uint8_t)149, (uint8_t)111, (uint8_t)212, (uint8_t)230, (uint8_t)190, (uint8_t)197, (uint8_t)135, (uint8_t)233, (uint8_t)25, (uint8_t)234, (uint8_t)46, (uint8_t)193, (uint8_t)25, (uint8_t)42, (uint8_t)3, (uint8_t)29, (uint8_t)8, (uint8_t)132, (uint8_t)73, (uint8_t)186, (uint8_t)200, (uint8_t)150, (uint8_t)143, (uint8_t)155, (uint8_t)77, (uint8_t)106, (uint8_t)82, (uint8_t)156, (uint8_t)54, (uint8_t)44, (uint8_t)76, (uint8_t)3, (uint8_t)96, (uint8_t)224, (uint8_t)128, (uint8_t)28, (uint8_t)144, (uint8_t)101, (uint8_t)59, (uint8_t)207, (uint8_t)225, (uint8_t)100, (uint8_t)43, (uint8_t)140, (uint8_t)247, (uint8_t)237, (uint8_t)22, (uint8_t)205, (uint8_t)164, (uint8_t)208, (uint8_t)203, (uint8_t)239, (uint8_t)237, (uint8_t)142, (uint8_t)58, (uint8_t)112, (uint8_t)89, (uint8_t)4, (uint8_t)175, (uint8_t)63, (uint8_t)140, (uint8_t)18, (uint8_t)39, (uint8_t)249, (uint8_t)82, (uint8_t)45, (uint8_t)118, (uint8_t)88, (uint8_t)147, (uint8_t)224, (uint8_t)174, (uint8_t)88, (uint8_t)31, (uint8_t)147, (uint8_t)172, (uint8_t)161, (uint8_t)1, (uint8_t)202, (uint8_t)168, (uint8_t)139, (uint8_t)86, (uint8_t)53, (uint8_t)152, (uint8_t)54, (uint8_t)48, (uint8_t)174, (uint8_t)27, (uint8_t)147, (uint8_t)94, (uint8_t)107, (uint8_t)8, (uint8_t)122, (uint8_t)48, (uint8_t)146, (uint8_t)64, (uint8_t)252, (uint8_t)44, (uint8_t)37, (uint8_t)33, (uint8_t)71, (uint8_t)72, (uint8_t)53, (uint8_t)150, (uint8_t)50, (uint8_t)197, (uint8_t)164, (uint8_t)134, (uint8_t)254, (uint8_t)98, (uint8_t)91, (uint8_t)122} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)250);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)44568);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_YAW_270);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)45575);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)3467384688L);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)60449);
};


void c_CommunicationChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_lat_GET(pack) == (int32_t)1253362199);
    assert(p133_mask_GET(pack) == (uint64_t)5772458444715259195L);
    assert(p133_lon_GET(pack) == (int32_t)218541196);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)13701);
};


void c_CommunicationChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_lon_GET(pack) == (int32_t) -959128891);
    assert(p134_lat_GET(pack) == (int32_t) -1817399396);
    {
        int16_t exemplary[] =  {(int16_t) -346, (int16_t) -10006, (int16_t) -27183, (int16_t) -15897, (int16_t)20839, (int16_t)15430, (int16_t)12285, (int16_t) -2986, (int16_t)2293, (int16_t)18172, (int16_t)29930, (int16_t) -11235, (int16_t) -29376, (int16_t) -1638, (int16_t) -14504, (int16_t) -31795} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)46201);
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)139);
};


void c_CommunicationChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lon_GET(pack) == (int32_t) -335060553);
    assert(p135_lat_GET(pack) == (int32_t) -1159619802);
};


void c_CommunicationChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_current_height_GET(pack) == (float) -2.1857905E38F);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)15972);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)28520);
    assert(p136_lat_GET(pack) == (int32_t)1843563009);
    assert(p136_lon_GET(pack) == (int32_t) -418547448);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)17331);
    assert(p136_terrain_height_GET(pack) == (float)1.2955662E38F);
};


void c_CommunicationChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t) -30594);
    assert(p137_press_diff_GET(pack) == (float)1.2614977E38F);
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)3599920670L);
    assert(p137_press_abs_GET(pack) == (float) -7.9668144E37F);
};


void c_CommunicationChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    assert(p138_x_GET(pack) == (float)3.3562202E38F);
    assert(p138_time_usec_GET(pack) == (uint64_t)7599245665255036759L);
    {
        float exemplary[] =  {1.0379074E38F, 5.0465928E36F, 1.6644815E38F, 9.733854E37F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_z_GET(pack) == (float) -1.1622727E38F);
    assert(p138_y_GET(pack) == (float) -1.3745644E38F);
};


void c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)183);
    {
        float exemplary[] =  {-2.73652E38F, -2.3299485E38F, 1.9436118E38F, -8.905948E37F, -7.9589615E37F, -2.260777E38F, 2.0189102E38F, -1.3604048E38F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p139_time_usec_GET(pack) == (uint64_t)5905742376266188652L);
};


void c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    assert(p140_time_usec_GET(pack) == (uint64_t)204805683154758408L);
    {
        float exemplary[] =  {-1.2877317E38F, 3.328549E38F, 1.3232384E38F, 3.2043696E38F, 3.172789E38F, -3.3842703E38F, -5.6490395E37F, -2.263283E37F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)88);
};


void c_CommunicationChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_altitude_local_GET(pack) == (float)1.678073E38F);
    assert(p141_altitude_amsl_GET(pack) == (float) -2.3213621E38F);
    assert(p141_bottom_clearance_GET(pack) == (float) -2.7229545E38F);
    assert(p141_altitude_terrain_GET(pack) == (float) -3.7763717E36F);
    assert(p141_altitude_relative_GET(pack) == (float) -8.540514E37F);
    assert(p141_time_usec_GET(pack) == (uint64_t)3473044262436307002L);
    assert(p141_altitude_monotonic_GET(pack) == (float)2.6429625E38F);
};


void c_CommunicationChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)111);
    {
        uint8_t exemplary[] =  {(uint8_t)152, (uint8_t)149, (uint8_t)159, (uint8_t)53, (uint8_t)230, (uint8_t)77, (uint8_t)50, (uint8_t)9, (uint8_t)73, (uint8_t)143, (uint8_t)206, (uint8_t)142, (uint8_t)241, (uint8_t)129, (uint8_t)218, (uint8_t)69, (uint8_t)153, (uint8_t)63, (uint8_t)81, (uint8_t)124, (uint8_t)48, (uint8_t)147, (uint8_t)247, (uint8_t)34, (uint8_t)138, (uint8_t)61, (uint8_t)110, (uint8_t)16, (uint8_t)169, (uint8_t)156, (uint8_t)245, (uint8_t)208, (uint8_t)189, (uint8_t)51, (uint8_t)183, (uint8_t)212, (uint8_t)90, (uint8_t)9, (uint8_t)6, (uint8_t)16, (uint8_t)165, (uint8_t)71, (uint8_t)146, (uint8_t)161, (uint8_t)167, (uint8_t)89, (uint8_t)0, (uint8_t)167, (uint8_t)84, (uint8_t)191, (uint8_t)184, (uint8_t)201, (uint8_t)92, (uint8_t)126, (uint8_t)48, (uint8_t)95, (uint8_t)237, (uint8_t)157, (uint8_t)89, (uint8_t)11, (uint8_t)18, (uint8_t)5, (uint8_t)103, (uint8_t)243, (uint8_t)166, (uint8_t)15, (uint8_t)135, (uint8_t)59, (uint8_t)225, (uint8_t)9, (uint8_t)210, (uint8_t)225, (uint8_t)111, (uint8_t)141, (uint8_t)36, (uint8_t)198, (uint8_t)100, (uint8_t)24, (uint8_t)66, (uint8_t)36, (uint8_t)103, (uint8_t)153, (uint8_t)104, (uint8_t)72, (uint8_t)58, (uint8_t)73, (uint8_t)43, (uint8_t)12, (uint8_t)193, (uint8_t)173, (uint8_t)210, (uint8_t)77, (uint8_t)22, (uint8_t)194, (uint8_t)132, (uint8_t)251, (uint8_t)134, (uint8_t)199, (uint8_t)10, (uint8_t)111, (uint8_t)11, (uint8_t)218, (uint8_t)61, (uint8_t)124, (uint8_t)92, (uint8_t)27, (uint8_t)1, (uint8_t)5, (uint8_t)200, (uint8_t)240, (uint8_t)141, (uint8_t)146, (uint8_t)97, (uint8_t)152, (uint8_t)207, (uint8_t)6, (uint8_t)210, (uint8_t)189, (uint8_t)194, (uint8_t)38} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)51, (uint8_t)109, (uint8_t)159, (uint8_t)208, (uint8_t)75, (uint8_t)156, (uint8_t)98, (uint8_t)58, (uint8_t)50, (uint8_t)15, (uint8_t)135, (uint8_t)222, (uint8_t)89, (uint8_t)232, (uint8_t)141, (uint8_t)26, (uint8_t)70, (uint8_t)133, (uint8_t)193, (uint8_t)164, (uint8_t)3, (uint8_t)239, (uint8_t)69, (uint8_t)134, (uint8_t)60, (uint8_t)176, (uint8_t)208, (uint8_t)86, (uint8_t)186, (uint8_t)135, (uint8_t)173, (uint8_t)115, (uint8_t)103, (uint8_t)188, (uint8_t)196, (uint8_t)41, (uint8_t)136, (uint8_t)36, (uint8_t)214, (uint8_t)37, (uint8_t)255, (uint8_t)53, (uint8_t)130, (uint8_t)185, (uint8_t)184, (uint8_t)99, (uint8_t)145, (uint8_t)132, (uint8_t)197, (uint8_t)116, (uint8_t)217, (uint8_t)181, (uint8_t)75, (uint8_t)216, (uint8_t)145, (uint8_t)69, (uint8_t)147, (uint8_t)178, (uint8_t)37, (uint8_t)193, (uint8_t)220, (uint8_t)40, (uint8_t)246, (uint8_t)136, (uint8_t)127, (uint8_t)71, (uint8_t)157, (uint8_t)77, (uint8_t)240, (uint8_t)248, (uint8_t)202, (uint8_t)227, (uint8_t)109, (uint8_t)155, (uint8_t)72, (uint8_t)187, (uint8_t)225, (uint8_t)132, (uint8_t)121, (uint8_t)102, (uint8_t)160, (uint8_t)206, (uint8_t)17, (uint8_t)176, (uint8_t)4, (uint8_t)234, (uint8_t)213, (uint8_t)179, (uint8_t)204, (uint8_t)36, (uint8_t)184, (uint8_t)242, (uint8_t)36, (uint8_t)153, (uint8_t)244, (uint8_t)89, (uint8_t)113, (uint8_t)244, (uint8_t)213, (uint8_t)14, (uint8_t)134, (uint8_t)142, (uint8_t)53, (uint8_t)163, (uint8_t)157, (uint8_t)254, (uint8_t)20, (uint8_t)12, (uint8_t)242, (uint8_t)139, (uint8_t)207, (uint8_t)201, (uint8_t)57, (uint8_t)209, (uint8_t)132, (uint8_t)84, (uint8_t)139, (uint8_t)51, (uint8_t)6, (uint8_t)211} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)113);
};


void c_CommunicationChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t) -19449);
    assert(p143_press_diff_GET(pack) == (float)1.0745996E38F);
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)1683663861L);
    assert(p143_press_abs_GET(pack) == (float)1.682078E38F);
};


void c_CommunicationChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-2.6654153E38F, -1.5465242E38F, 1.8622976E38F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {3.2508877E38F, 1.0253613E38F, 2.6060905E38F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {1.7509305E38F, -9.476816E37F, 2.7711655E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-1.7881498E38F, -2.938117E38F, -1.5812252E38F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_custom_state_GET(pack) == (uint64_t)8321112846915514256L);
    assert(p144_lat_GET(pack) == (int32_t) -1886024704);
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)184);
    assert(p144_lon_GET(pack) == (int32_t) -1757854381);
    assert(p144_alt_GET(pack) == (float) -3.1291182E38F);
    {
        float exemplary[] =  {2.2788255E38F, 1.8943898E38F, 1.1401667E38F, -2.7231514E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_timestamp_GET(pack) == (uint64_t)2543914614391503421L);
};


void c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_time_usec_GET(pack) == (uint64_t)4965253514208826191L);
    assert(p146_z_acc_GET(pack) == (float)9.562689E37F);
    assert(p146_x_pos_GET(pack) == (float) -1.8876307E38F);
    assert(p146_y_vel_GET(pack) == (float)2.0790853E38F);
    assert(p146_pitch_rate_GET(pack) == (float) -7.021863E37F);
    assert(p146_roll_rate_GET(pack) == (float) -2.7382644E38F);
    assert(p146_airspeed_GET(pack) == (float)2.1839727E38F);
    assert(p146_z_vel_GET(pack) == (float)2.2337535E38F);
    assert(p146_y_pos_GET(pack) == (float) -1.0669033E37F);
    assert(p146_x_vel_GET(pack) == (float)1.8287872E38F);
    {
        float exemplary[] =  {3.0289456E38F, 2.601582E38F, -1.9547789E38F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_y_acc_GET(pack) == (float)4.373306E37F);
    {
        float exemplary[] =  {2.180479E38F, -2.6406154E38F, 6.002052E37F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {1.0022256E37F, -2.4938442E38F, -1.2414346E38F, -1.7563465E38F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_x_acc_GET(pack) == (float)2.486412E38F);
    assert(p146_z_pos_GET(pack) == (float) -1.1242104E37F);
    assert(p146_yaw_rate_GET(pack) == (float) -3.1712809E38F);
};


void c_CommunicationChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)211);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIPO);
    assert(p147_current_consumed_GET(pack) == (int32_t)92688);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t) -29343);
    {
        uint16_t exemplary[] =  {(uint16_t)27784, (uint16_t)31419, (uint16_t)50578, (uint16_t)14451, (uint16_t)44876, (uint16_t)29297, (uint16_t)50929, (uint16_t)31983, (uint16_t)54754, (uint16_t)12909} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t)23683);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t) -85);
    assert(p147_energy_consumed_GET(pack) == (int32_t)1885054876);
};


void c_CommunicationChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_os_sw_version_GET(pack) == (uint32_t)575586744L);
    assert(p148_board_version_GET(pack) == (uint32_t)3629162258L);
    assert(p148_uid_GET(pack) == (uint64_t)4382466503664530744L);
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)1230);
    assert(p148_capabilities_GET(pack) == e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT);
    {
        uint8_t exemplary[] =  {(uint8_t)249, (uint8_t)138, (uint8_t)239, (uint8_t)173, (uint8_t)169, (uint8_t)213, (uint8_t)108, (uint8_t)184} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)1880126634L);
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)26214);
    {
        uint8_t exemplary[] =  {(uint8_t)21, (uint8_t)65, (uint8_t)80, (uint8_t)112, (uint8_t)182, (uint8_t)16, (uint8_t)85, (uint8_t)117} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)131, (uint8_t)82, (uint8_t)139, (uint8_t)26, (uint8_t)120, (uint8_t)46, (uint8_t)4, (uint8_t)117, (uint8_t)117, (uint8_t)95, (uint8_t)213, (uint8_t)128, (uint8_t)14, (uint8_t)90, (uint8_t)39, (uint8_t)67, (uint8_t)93, (uint8_t)241} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)178, (uint8_t)125, (uint8_t)58, (uint8_t)233, (uint8_t)232, (uint8_t)173, (uint8_t)57, (uint8_t)169} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)3678170855L);
};


void c_CommunicationChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_size_y_GET(pack) == (float) -3.8066353E37F);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_RADIO_BEACON);
    assert(p149_distance_GET(pack) == (float) -3.2185359E38F);
    assert(p149_time_usec_GET(pack) == (uint64_t)5399986078439850086L);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)119);
    {
        float exemplary[] =  {-2.9977225E38F, 2.2878037E38F, -1.3870347E38F, 1.2044326E38F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_x_TRY(ph) == (float) -2.6925596E38F);
    assert(p149_y_TRY(ph) == (float)5.2875365E37F);
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p149_z_TRY(ph) == (float) -9.053326E36F);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)78);
    assert(p149_angle_y_GET(pack) == (float)3.429606E37F);
    assert(p149_size_x_GET(pack) == (float) -1.8891844E38F);
    assert(p149_angle_x_GET(pack) == (float) -2.6137475E37F);
};


void c_CommunicationChannel_on_AQ_TELEMETRY_F_150(Bounds_Inside * ph, Pack * pack)
{
    assert(p150_value8_GET(pack) == (float) -9.555088E37F);
    assert(p150_value4_GET(pack) == (float)1.4889478E38F);
    assert(p150_value11_GET(pack) == (float)1.8255578E38F);
    assert(p150_value14_GET(pack) == (float) -9.13397E37F);
    assert(p150_value6_GET(pack) == (float) -1.5494409E38F);
    assert(p150_value10_GET(pack) == (float) -1.475662E38F);
    assert(p150_value7_GET(pack) == (float) -4.843846E37F);
    assert(p150_value20_GET(pack) == (float)5.839068E37F);
    assert(p150_value13_GET(pack) == (float) -7.255428E37F);
    assert(p150_value3_GET(pack) == (float)3.2535117E38F);
    assert(p150_Index_GET(pack) == (uint16_t)(uint16_t)57859);
    assert(p150_value5_GET(pack) == (float) -5.6799103E37F);
    assert(p150_value15_GET(pack) == (float) -2.7591467E36F);
    assert(p150_value18_GET(pack) == (float) -3.0462863E37F);
    assert(p150_value12_GET(pack) == (float)1.838679E38F);
    assert(p150_value19_GET(pack) == (float) -8.226431E37F);
    assert(p150_value17_GET(pack) == (float)2.1936428E38F);
    assert(p150_value1_GET(pack) == (float) -5.392704E37F);
    assert(p150_value9_GET(pack) == (float) -1.1004821E38F);
    assert(p150_value2_GET(pack) == (float)3.2828471E38F);
    assert(p150_value16_GET(pack) == (float)1.0499287E38F);
};


void c_CommunicationChannel_on_AQ_ESC_TELEMETRY_152(Bounds_Inside * ph, Pack * pack)
{
    assert(p152_seq_GET(pack) == (uint8_t)(uint8_t)171);
    {
        uint8_t exemplary[] =  {(uint8_t)66, (uint8_t)67, (uint8_t)234, (uint8_t)95} ;
        uint8_t*  sample = p152_data_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint16_t exemplary[] =  {(uint16_t)49859, (uint16_t)59786, (uint16_t)7124, (uint16_t)59668} ;
        uint16_t*  sample = p152_status_age_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)177, (uint8_t)148, (uint8_t)51, (uint8_t)29} ;
        uint8_t*  sample = p152_escid_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint32_t exemplary[] =  {2770028949L, 228202652L, 596172276L, 2267382589L} ;
        uint32_t*  sample = p152_data0_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p152_num_motors_GET(pack) == (uint8_t)(uint8_t)237);
    assert(p152_time_boot_ms_GET(pack) == (uint32_t)45143426L);
    {
        uint32_t exemplary[] =  {1049983750L, 578858551L, 4198537330L, 3975281948L} ;
        uint32_t*  sample = p152_data1_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p152_num_in_seq_GET(pack) == (uint8_t)(uint8_t)8);
};


void c_CommunicationChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_time_usec_GET(pack) == (uint64_t)3923058367190746169L);
    assert(p230_tas_ratio_GET(pack) == (float) -3.808844E37F);
    assert(p230_pos_horiz_ratio_GET(pack) == (float)3.5278617E37F);
    assert(p230_hagl_ratio_GET(pack) == (float) -1.2000671E38F);
    assert(p230_mag_ratio_GET(pack) == (float)2.4642596E38F);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float)2.2941434E38F);
    assert(p230_vel_ratio_GET(pack) == (float)2.9970215E38F);
    assert(p230_flags_GET(pack) == e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE);
    assert(p230_pos_vert_accuracy_GET(pack) == (float) -7.9149097E37F);
    assert(p230_pos_vert_ratio_GET(pack) == (float) -2.7398168E38F);
};


void c_CommunicationChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_horiz_accuracy_GET(pack) == (float)2.181051E38F);
    assert(p231_var_vert_GET(pack) == (float) -7.587624E37F);
    assert(p231_wind_x_GET(pack) == (float)2.5124622E38F);
    assert(p231_time_usec_GET(pack) == (uint64_t)1086841837524163310L);
    assert(p231_wind_alt_GET(pack) == (float) -1.8518599E38F);
    assert(p231_wind_y_GET(pack) == (float) -8.932963E37F);
    assert(p231_wind_z_GET(pack) == (float)1.8936844E38F);
    assert(p231_var_horiz_GET(pack) == (float)1.4352971E38F);
    assert(p231_vert_accuracy_GET(pack) == (float)2.4563278E38F);
};


void c_CommunicationChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)111);
    assert(p232_hdop_GET(pack) == (float) -1.6241342E38F);
    assert(p232_vn_GET(pack) == (float)7.926133E37F);
    assert(p232_vdop_GET(pack) == (float)2.9763834E38F);
    assert(p232_horiz_accuracy_GET(pack) == (float)3.3999766E38F);
    assert(p232_ve_GET(pack) == (float)5.5010093E37F);
    assert(p232_vd_GET(pack) == (float) -1.2679857E36F);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)40415);
    assert(p232_ignore_flags_GET(pack) == e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)2027246167L);
    assert(p232_lon_GET(pack) == (int32_t) -503845290);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)143);
    assert(p232_alt_GET(pack) == (float) -1.971394E38F);
    assert(p232_vert_accuracy_GET(pack) == (float)1.8148898E36F);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p232_time_usec_GET(pack) == (uint64_t)1149849318146977234L);
    assert(p232_speed_accuracy_GET(pack) == (float)9.827738E37F);
    assert(p232_lat_GET(pack) == (int32_t)1273375205);
};


void c_CommunicationChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)31);
    {
        uint8_t exemplary[] =  {(uint8_t)233, (uint8_t)67, (uint8_t)148, (uint8_t)192, (uint8_t)252, (uint8_t)230, (uint8_t)161, (uint8_t)250, (uint8_t)194, (uint8_t)232, (uint8_t)113, (uint8_t)192, (uint8_t)124, (uint8_t)214, (uint8_t)23, (uint8_t)235, (uint8_t)187, (uint8_t)247, (uint8_t)128, (uint8_t)216, (uint8_t)49, (uint8_t)89, (uint8_t)26, (uint8_t)212, (uint8_t)22, (uint8_t)68, (uint8_t)197, (uint8_t)115, (uint8_t)188, (uint8_t)34, (uint8_t)66, (uint8_t)229, (uint8_t)62, (uint8_t)119, (uint8_t)26, (uint8_t)88, (uint8_t)36, (uint8_t)23, (uint8_t)36, (uint8_t)20, (uint8_t)102, (uint8_t)17, (uint8_t)131, (uint8_t)223, (uint8_t)7, (uint8_t)129, (uint8_t)140, (uint8_t)153, (uint8_t)95, (uint8_t)134, (uint8_t)193, (uint8_t)23, (uint8_t)71, (uint8_t)34, (uint8_t)184, (uint8_t)185, (uint8_t)237, (uint8_t)157, (uint8_t)84, (uint8_t)67, (uint8_t)226, (uint8_t)249, (uint8_t)236, (uint8_t)208, (uint8_t)2, (uint8_t)203, (uint8_t)71, (uint8_t)12, (uint8_t)51, (uint8_t)153, (uint8_t)75, (uint8_t)172, (uint8_t)111, (uint8_t)64, (uint8_t)90, (uint8_t)97, (uint8_t)90, (uint8_t)133, (uint8_t)59, (uint8_t)31, (uint8_t)78, (uint8_t)131, (uint8_t)246, (uint8_t)113, (uint8_t)133, (uint8_t)33, (uint8_t)1, (uint8_t)43, (uint8_t)235, (uint8_t)203, (uint8_t)2, (uint8_t)93, (uint8_t)219, (uint8_t)210, (uint8_t)26, (uint8_t)246, (uint8_t)6, (uint8_t)142, (uint8_t)235, (uint8_t)33, (uint8_t)1, (uint8_t)140, (uint8_t)152, (uint8_t)249, (uint8_t)171, (uint8_t)65, (uint8_t)125, (uint8_t)185, (uint8_t)182, (uint8_t)145, (uint8_t)132, (uint8_t)78, (uint8_t)129, (uint8_t)115, (uint8_t)47, (uint8_t)29, (uint8_t)2, (uint8_t)91, (uint8_t)91, (uint8_t)155, (uint8_t)179, (uint8_t)23, (uint8_t)108, (uint8_t)104, (uint8_t)231, (uint8_t)43, (uint8_t)2, (uint8_t)42, (uint8_t)10, (uint8_t)21, (uint8_t)82, (uint8_t)192, (uint8_t)19, (uint8_t)84, (uint8_t)11, (uint8_t)27, (uint8_t)135, (uint8_t)231, (uint8_t)168, (uint8_t)169, (uint8_t)196, (uint8_t)28, (uint8_t)33, (uint8_t)218, (uint8_t)213, (uint8_t)252, (uint8_t)239, (uint8_t)153, (uint8_t)182, (uint8_t)160, (uint8_t)39, (uint8_t)118, (uint8_t)206, (uint8_t)38, (uint8_t)3, (uint8_t)26, (uint8_t)60, (uint8_t)207, (uint8_t)94, (uint8_t)146, (uint8_t)79, (uint8_t)154, (uint8_t)75, (uint8_t)182, (uint8_t)192, (uint8_t)221, (uint8_t)0, (uint8_t)44, (uint8_t)212, (uint8_t)63, (uint8_t)201, (uint8_t)138, (uint8_t)47, (uint8_t)121, (uint8_t)148, (uint8_t)99, (uint8_t)138, (uint8_t)95, (uint8_t)92, (uint8_t)36} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)157);
};


void c_CommunicationChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t) -3865);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t) -126);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t) -20);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)27159);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t)2765);
    assert(p234_custom_mode_GET(pack) == (uint32_t)3196392122L);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t) -35);
    assert(p234_latitude_GET(pack) == (int32_t)1388463869);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t) -1081);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t) -41);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)160);
    assert(p234_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t) -7446);
    assert(p234_longitude_GET(pack) == (int32_t)1973151624);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t)16053);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)55);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)15150);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)112);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)204);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP);
};


void c_CommunicationChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_time_usec_GET(pack) == (uint64_t)3233851370422934086L);
    assert(p241_vibration_z_GET(pack) == (float)2.9159333E38F);
    assert(p241_vibration_y_GET(pack) == (float)1.1444053E38F);
    assert(p241_vibration_x_GET(pack) == (float)1.6319823E38F);
    assert(p241_clipping_2_GET(pack) == (uint32_t)3212046500L);
    assert(p241_clipping_1_GET(pack) == (uint32_t)2213158821L);
    assert(p241_clipping_0_GET(pack) == (uint32_t)3443350254L);
};


void c_CommunicationChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_time_usec_TRY(ph) == (uint64_t)6635489893600274986L);
    assert(p242_longitude_GET(pack) == (int32_t) -636781737);
    assert(p242_x_GET(pack) == (float) -3.0082476E38F);
    assert(p242_latitude_GET(pack) == (int32_t) -1237058113);
    assert(p242_approach_y_GET(pack) == (float) -2.266127E38F);
    assert(p242_y_GET(pack) == (float) -3.105126E38F);
    assert(p242_altitude_GET(pack) == (int32_t)1593474894);
    assert(p242_approach_z_GET(pack) == (float) -7.754246E37F);
    {
        float exemplary[] =  {3.3340166E38F, -1.8847281E38F, 1.3844984E38F, -1.7381318E37F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_approach_x_GET(pack) == (float) -2.7708364E38F);
    assert(p242_z_GET(pack) == (float)2.667485E38F);
};


void c_CommunicationChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_z_GET(pack) == (float) -1.763316E38F);
    assert(p243_latitude_GET(pack) == (int32_t) -1456077207);
    assert(p243_x_GET(pack) == (float) -2.0874652E38F);
    assert(p243_longitude_GET(pack) == (int32_t)659151816);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p243_time_usec_TRY(ph) == (uint64_t)2649724085903844373L);
    assert(p243_altitude_GET(pack) == (int32_t) -1490177590);
    assert(p243_approach_y_GET(pack) == (float) -2.3611368E38F);
    {
        float exemplary[] =  {6.6474695E37F, 5.892577E37F, -9.488282E37F, -7.613717E37F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_approach_x_GET(pack) == (float) -1.7241693E38F);
    assert(p243_y_GET(pack) == (float) -2.2561718E38F);
    assert(p243_approach_z_GET(pack) == (float)7.815999E37F);
};


void c_CommunicationChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_interval_us_GET(pack) == (int32_t) -171054102);
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)17622);
};


void c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED);
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED);
};


void c_CommunicationChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_ICAO_address_GET(pack) == (uint32_t)2961331819L);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HEAVY);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC);
    assert(p246_altitude_GET(pack) == (int32_t) -1182839700);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)2672);
    assert(p246_flags_GET(pack) == e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t) -25334);
    assert(p246_lon_GET(pack) == (int32_t)2004600735);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)4066);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)27563);
    assert(p246_lat_GET(pack) == (int32_t)300559381);
    assert(p246_callsign_LEN(ph) == 7);
    {
        char16_t * exemplary = u"kkbuvyz";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)69);
};


void c_CommunicationChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_id_GET(pack) == (uint32_t)347296417L);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_HORIZONTALLY);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH);
    assert(p247_altitude_minimum_delta_GET(pack) == (float)3.2134809E38F);
    assert(p247_time_to_minimum_delta_GET(pack) == (float)5.747431E37F);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float) -1.9360588E38F);
};


void c_CommunicationChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)65);
    {
        uint8_t exemplary[] =  {(uint8_t)57, (uint8_t)213, (uint8_t)249, (uint8_t)214, (uint8_t)120, (uint8_t)244, (uint8_t)141, (uint8_t)122, (uint8_t)186, (uint8_t)74, (uint8_t)123, (uint8_t)179, (uint8_t)156, (uint8_t)138, (uint8_t)228, (uint8_t)233, (uint8_t)249, (uint8_t)241, (uint8_t)146, (uint8_t)235, (uint8_t)47, (uint8_t)252, (uint8_t)88, (uint8_t)216, (uint8_t)135, (uint8_t)178, (uint8_t)184, (uint8_t)184, (uint8_t)8, (uint8_t)88, (uint8_t)146, (uint8_t)99, (uint8_t)0, (uint8_t)19, (uint8_t)218, (uint8_t)102, (uint8_t)38, (uint8_t)197, (uint8_t)166, (uint8_t)224, (uint8_t)128, (uint8_t)108, (uint8_t)49, (uint8_t)115, (uint8_t)24, (uint8_t)177, (uint8_t)198, (uint8_t)74, (uint8_t)42, (uint8_t)154, (uint8_t)144, (uint8_t)255, (uint8_t)89, (uint8_t)15, (uint8_t)129, (uint8_t)64, (uint8_t)220, (uint8_t)131, (uint8_t)211, (uint8_t)203, (uint8_t)28, (uint8_t)199, (uint8_t)239, (uint8_t)81, (uint8_t)150, (uint8_t)221, (uint8_t)158, (uint8_t)162, (uint8_t)70, (uint8_t)250, (uint8_t)79, (uint8_t)28, (uint8_t)208, (uint8_t)112, (uint8_t)89, (uint8_t)159, (uint8_t)81, (uint8_t)211, (uint8_t)243, (uint8_t)80, (uint8_t)244, (uint8_t)251, (uint8_t)95, (uint8_t)164, (uint8_t)42, (uint8_t)124, (uint8_t)166, (uint8_t)217, (uint8_t)107, (uint8_t)196, (uint8_t)0, (uint8_t)222, (uint8_t)232, (uint8_t)38, (uint8_t)212, (uint8_t)207, (uint8_t)20, (uint8_t)29, (uint8_t)240, (uint8_t)44, (uint8_t)208, (uint8_t)16, (uint8_t)207, (uint8_t)6, (uint8_t)56, (uint8_t)162, (uint8_t)160, (uint8_t)200, (uint8_t)108, (uint8_t)223, (uint8_t)7, (uint8_t)33, (uint8_t)197, (uint8_t)18, (uint8_t)48, (uint8_t)120, (uint8_t)2, (uint8_t)106, (uint8_t)79, (uint8_t)189, (uint8_t)105, (uint8_t)200, (uint8_t)192, (uint8_t)4, (uint8_t)95, (uint8_t)103, (uint8_t)9, (uint8_t)148, (uint8_t)33, (uint8_t)183, (uint8_t)251, (uint8_t)89, (uint8_t)237, (uint8_t)77, (uint8_t)250, (uint8_t)139, (uint8_t)26, (uint8_t)3, (uint8_t)6, (uint8_t)29, (uint8_t)25, (uint8_t)143, (uint8_t)5, (uint8_t)92, (uint8_t)123, (uint8_t)8, (uint8_t)64, (uint8_t)115, (uint8_t)55, (uint8_t)137, (uint8_t)221, (uint8_t)244, (uint8_t)57, (uint8_t)225, (uint8_t)198, (uint8_t)118, (uint8_t)79, (uint8_t)251, (uint8_t)204, (uint8_t)248, (uint8_t)3, (uint8_t)163, (uint8_t)183, (uint8_t)48, (uint8_t)71, (uint8_t)141, (uint8_t)206, (uint8_t)120, (uint8_t)131, (uint8_t)157, (uint8_t)9, (uint8_t)18, (uint8_t)240, (uint8_t)11, (uint8_t)181, (uint8_t)192, (uint8_t)117, (uint8_t)221, (uint8_t)19, (uint8_t)175, (uint8_t)236, (uint8_t)87, (uint8_t)222, (uint8_t)153, (uint8_t)219, (uint8_t)235, (uint8_t)52, (uint8_t)61, (uint8_t)12, (uint8_t)20, (uint8_t)165, (uint8_t)177, (uint8_t)17, (uint8_t)206, (uint8_t)209, (uint8_t)203, (uint8_t)17, (uint8_t)96, (uint8_t)100, (uint8_t)196, (uint8_t)199, (uint8_t)165, (uint8_t)253, (uint8_t)196, (uint8_t)105, (uint8_t)244, (uint8_t)32, (uint8_t)102, (uint8_t)21, (uint8_t)237, (uint8_t)197, (uint8_t)249, (uint8_t)153, (uint8_t)161, (uint8_t)21, (uint8_t)29, (uint8_t)204, (uint8_t)167, (uint8_t)117, (uint8_t)88, (uint8_t)163, (uint8_t)222, (uint8_t)225, (uint8_t)247, (uint8_t)119, (uint8_t)174, (uint8_t)80, (uint8_t)141, (uint8_t)12, (uint8_t)225, (uint8_t)37, (uint8_t)97, (uint8_t)191, (uint8_t)155, (uint8_t)63, (uint8_t)222, (uint8_t)96, (uint8_t)241, (uint8_t)40, (uint8_t)14, (uint8_t)192, (uint8_t)235, (uint8_t)46, (uint8_t)169, (uint8_t)58, (uint8_t)65, (uint8_t)226, (uint8_t)111, (uint8_t)102} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)17028);
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)196);
};


void c_CommunicationChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)0);
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)4);
    {
        int8_t exemplary[] =  {(int8_t)42, (int8_t) -78, (int8_t) -46, (int8_t)95, (int8_t)125, (int8_t) -106, (int8_t)98, (int8_t) -59, (int8_t) -52, (int8_t) -92, (int8_t)64, (int8_t)40, (int8_t)20, (int8_t) -33, (int8_t)69, (int8_t) -86, (int8_t)19, (int8_t)31, (int8_t)97, (int8_t) -55, (int8_t) -110, (int8_t)84, (int8_t)88, (int8_t) -67, (int8_t) -82, (int8_t) -118, (int8_t)85, (int8_t)122, (int8_t) -122, (int8_t) -121, (int8_t) -61, (int8_t) -97} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)50617);
};


void c_CommunicationChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_x_GET(pack) == (float) -3.3999933E38F);
    assert(p250_y_GET(pack) == (float) -5.7876277E37F);
    assert(p250_time_usec_GET(pack) == (uint64_t)3486507564428539286L);
    assert(p250_z_GET(pack) == (float)2.7095328E38F);
    assert(p250_name_LEN(ph) == 3);
    {
        char16_t * exemplary = u"cMz";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_value_GET(pack) == (float) -1.9238748E38F);
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)1704974398L);
    assert(p251_name_LEN(ph) == 7);
    {
        char16_t * exemplary = u"ejxvzmn";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)1532238953L);
    assert(p252_name_LEN(ph) == 5);
    {
        char16_t * exemplary = u"ofsSe";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_value_GET(pack) == (int32_t) -233100281);
};


void c_CommunicationChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_INFO);
    assert(p253_text_LEN(ph) == 41);
    {
        char16_t * exemplary = u"ocpdhxIbemyzgnizlmwdwrfdmhciqnygufllYalwv";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 82);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_value_GET(pack) == (float) -1.752027E38F);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)615850665L);
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)2);
};


void c_CommunicationChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)183);
    {
        uint8_t exemplary[] =  {(uint8_t)18, (uint8_t)232, (uint8_t)108, (uint8_t)193, (uint8_t)24, (uint8_t)253, (uint8_t)139, (uint8_t)129, (uint8_t)211, (uint8_t)110, (uint8_t)137, (uint8_t)239, (uint8_t)7, (uint8_t)110, (uint8_t)208, (uint8_t)177, (uint8_t)149, (uint8_t)57, (uint8_t)75, (uint8_t)31, (uint8_t)182, (uint8_t)77, (uint8_t)57, (uint8_t)168, (uint8_t)41, (uint8_t)67, (uint8_t)54, (uint8_t)108, (uint8_t)146, (uint8_t)199, (uint8_t)76, (uint8_t)245} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)633515334548168862L);
};


void c_CommunicationChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)1501436732L);
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)152);
    assert(p257_last_change_ms_GET(pack) == (uint32_t)3588070996L);
};


void c_CommunicationChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_tune_LEN(ph) == 8);
    {
        char16_t * exemplary = u"zgslhvjo";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)172);
};


void c_CommunicationChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_sensor_size_h_GET(pack) == (float) -1.5899593E38F);
    assert(p259_cam_definition_uri_LEN(ph) == 5);
    {
        char16_t * exemplary = u"kWLdw";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)51246);
    assert(p259_focal_length_GET(pack) == (float) -2.2013982E38F);
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)19614);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)183);
    assert(p259_sensor_size_v_GET(pack) == (float)3.5142763E37F);
    assert(p259_firmware_version_GET(pack) == (uint32_t)3637224030L);
    assert(p259_flags_GET(pack) == e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES);
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)888913768L);
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)57923);
    {
        uint8_t exemplary[] =  {(uint8_t)212, (uint8_t)61, (uint8_t)200, (uint8_t)200, (uint8_t)2, (uint8_t)207, (uint8_t)180, (uint8_t)167, (uint8_t)217, (uint8_t)17, (uint8_t)182, (uint8_t)84, (uint8_t)30, (uint8_t)53, (uint8_t)144, (uint8_t)66, (uint8_t)120, (uint8_t)122, (uint8_t)247, (uint8_t)137, (uint8_t)20, (uint8_t)253, (uint8_t)123, (uint8_t)222, (uint8_t)14, (uint8_t)134, (uint8_t)67, (uint8_t)132, (uint8_t)168, (uint8_t)13, (uint8_t)167, (uint8_t)8} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)195, (uint8_t)30, (uint8_t)0, (uint8_t)214, (uint8_t)160, (uint8_t)51, (uint8_t)223, (uint8_t)215, (uint8_t)141, (uint8_t)180, (uint8_t)168, (uint8_t)230, (uint8_t)8, (uint8_t)245, (uint8_t)127, (uint8_t)110, (uint8_t)55, (uint8_t)175, (uint8_t)154, (uint8_t)240, (uint8_t)86, (uint8_t)100, (uint8_t)135, (uint8_t)213, (uint8_t)176, (uint8_t)234, (uint8_t)195, (uint8_t)249, (uint8_t)34, (uint8_t)225, (uint8_t)18, (uint8_t)16} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_VIDEO);
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)1953402280L);
};


void c_CommunicationChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)38);
    assert(p261_used_capacity_GET(pack) == (float)3.000668E38F);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p261_write_speed_GET(pack) == (float) -2.021595E38F);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)20);
    assert(p261_read_speed_GET(pack) == (float)2.6007199E38F);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)3284622734L);
    assert(p261_total_capacity_GET(pack) == (float)2.7226608E38F);
    assert(p261_available_capacity_GET(pack) == (float)5.7831245E37F);
};


void c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)222);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p262_available_capacity_GET(pack) == (float) -1.8972945E38F);
    assert(p262_image_interval_GET(pack) == (float)1.9767021E38F);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)3594837601L);
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)3683909764L);
};


void c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-3.4225953E37F, 1.7098319E38F, -2.3307513E38F, -5.4801565E37F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)3122380836L);
    assert(p263_image_index_GET(pack) == (int32_t) -1501649527);
    assert(p263_time_utc_GET(pack) == (uint64_t)4021644501310134824L);
    assert(p263_lat_GET(pack) == (int32_t) -1701311411);
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t) -105);
    assert(p263_file_url_LEN(ph) == 184);
    {
        char16_t * exemplary = u"gjwowvjhhaqexifsarseodwqpssRnnhpKhybidQkyamcwvkcPvqutdypnrtreTfczmabbfBjzudhNxcodvkktnyfKemazoayiMXjrpvukhceRhbcgtxuzhlcfJhwXoxhqygtjrqlvxpysgspoocrAfciYshZupwskCxxsmsgjhxivnklfbualruv";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 368);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_alt_GET(pack) == (int32_t)1879260172);
    assert(p263_lon_GET(pack) == (int32_t) -1172797255);
    assert(p263_relative_alt_GET(pack) == (int32_t)1982799590);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)186);
};


void c_CommunicationChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)3759068448L);
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)84913462640655291L);
    assert(p264_flight_uuid_GET(pack) == (uint64_t)5645365529008825778L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)6088455146710635728L);
};


void c_CommunicationChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_yaw_GET(pack) == (float)5.6599935E37F);
    assert(p265_pitch_GET(pack) == (float)2.3313096E38F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)3720156103L);
    assert(p265_roll_GET(pack) == (float) -9.563315E37F);
};


void c_CommunicationChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)134, (uint8_t)97, (uint8_t)165, (uint8_t)181, (uint8_t)135, (uint8_t)50, (uint8_t)173, (uint8_t)239, (uint8_t)161, (uint8_t)127, (uint8_t)141, (uint8_t)147, (uint8_t)190, (uint8_t)40, (uint8_t)175, (uint8_t)46, (uint8_t)1, (uint8_t)213, (uint8_t)167, (uint8_t)77, (uint8_t)48, (uint8_t)195, (uint8_t)73, (uint8_t)184, (uint8_t)68, (uint8_t)198, (uint8_t)161, (uint8_t)76, (uint8_t)97, (uint8_t)90, (uint8_t)13, (uint8_t)110, (uint8_t)190, (uint8_t)139, (uint8_t)175, (uint8_t)62, (uint8_t)135, (uint8_t)215, (uint8_t)112, (uint8_t)79, (uint8_t)130, (uint8_t)191, (uint8_t)252, (uint8_t)93, (uint8_t)246, (uint8_t)132, (uint8_t)227, (uint8_t)138, (uint8_t)105, (uint8_t)98, (uint8_t)61, (uint8_t)235, (uint8_t)70, (uint8_t)72, (uint8_t)245, (uint8_t)218, (uint8_t)195, (uint8_t)187, (uint8_t)210, (uint8_t)176, (uint8_t)241, (uint8_t)34, (uint8_t)211, (uint8_t)129, (uint8_t)121, (uint8_t)211, (uint8_t)236, (uint8_t)59, (uint8_t)22, (uint8_t)23, (uint8_t)58, (uint8_t)223, (uint8_t)154, (uint8_t)89, (uint8_t)71, (uint8_t)141, (uint8_t)192, (uint8_t)21, (uint8_t)41, (uint8_t)226, (uint8_t)194, (uint8_t)48, (uint8_t)2, (uint8_t)172, (uint8_t)0, (uint8_t)196, (uint8_t)180, (uint8_t)242, (uint8_t)133, (uint8_t)241, (uint8_t)81, (uint8_t)19, (uint8_t)165, (uint8_t)88, (uint8_t)242, (uint8_t)12, (uint8_t)13, (uint8_t)15, (uint8_t)34, (uint8_t)203, (uint8_t)249, (uint8_t)100, (uint8_t)226, (uint8_t)77, (uint8_t)93, (uint8_t)9, (uint8_t)24, (uint8_t)235, (uint8_t)151, (uint8_t)122, (uint8_t)79, (uint8_t)99, (uint8_t)152, (uint8_t)186, (uint8_t)245, (uint8_t)137, (uint8_t)97, (uint8_t)5, (uint8_t)143, (uint8_t)80, (uint8_t)29, (uint8_t)209, (uint8_t)158, (uint8_t)46, (uint8_t)136, (uint8_t)57, (uint8_t)4, (uint8_t)149, (uint8_t)103, (uint8_t)72, (uint8_t)42, (uint8_t)21, (uint8_t)225, (uint8_t)90, (uint8_t)12, (uint8_t)77, (uint8_t)192, (uint8_t)134, (uint8_t)11, (uint8_t)21, (uint8_t)85, (uint8_t)22, (uint8_t)244, (uint8_t)149, (uint8_t)128, (uint8_t)157, (uint8_t)91, (uint8_t)242, (uint8_t)161, (uint8_t)152, (uint8_t)86, (uint8_t)106, (uint8_t)29, (uint8_t)125, (uint8_t)124, (uint8_t)58, (uint8_t)178, (uint8_t)142, (uint8_t)202, (uint8_t)58, (uint8_t)70, (uint8_t)46, (uint8_t)240, (uint8_t)91, (uint8_t)116, (uint8_t)31, (uint8_t)165, (uint8_t)52, (uint8_t)6, (uint8_t)186, (uint8_t)58, (uint8_t)202, (uint8_t)160, (uint8_t)198, (uint8_t)103, (uint8_t)166, (uint8_t)144, (uint8_t)110, (uint8_t)53, (uint8_t)211, (uint8_t)138, (uint8_t)136, (uint8_t)43, (uint8_t)104, (uint8_t)50, (uint8_t)138, (uint8_t)163, (uint8_t)246, (uint8_t)149, (uint8_t)109, (uint8_t)48, (uint8_t)88, (uint8_t)94, (uint8_t)13, (uint8_t)234, (uint8_t)51, (uint8_t)5, (uint8_t)128, (uint8_t)135, (uint8_t)172, (uint8_t)31, (uint8_t)135, (uint8_t)183, (uint8_t)68, (uint8_t)104, (uint8_t)94, (uint8_t)185, (uint8_t)45, (uint8_t)6, (uint8_t)60, (uint8_t)126, (uint8_t)4, (uint8_t)224, (uint8_t)184, (uint8_t)154, (uint8_t)204, (uint8_t)121, (uint8_t)129, (uint8_t)106, (uint8_t)185, (uint8_t)83, (uint8_t)156, (uint8_t)254, (uint8_t)181, (uint8_t)164, (uint8_t)110, (uint8_t)115, (uint8_t)108, (uint8_t)245, (uint8_t)217, (uint8_t)216, (uint8_t)223, (uint8_t)40, (uint8_t)112, (uint8_t)250, (uint8_t)92, (uint8_t)13, (uint8_t)18, (uint8_t)240, (uint8_t)88, (uint8_t)123, (uint8_t)136, (uint8_t)34, (uint8_t)61, (uint8_t)82, (uint8_t)222, (uint8_t)81, (uint8_t)17, (uint8_t)249} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)65);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)143);
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)144);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)25046);
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)66);
};


void c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)8, (uint8_t)69, (uint8_t)122, (uint8_t)6, (uint8_t)22, (uint8_t)184, (uint8_t)105, (uint8_t)7, (uint8_t)85, (uint8_t)32, (uint8_t)119, (uint8_t)48, (uint8_t)73, (uint8_t)166, (uint8_t)67, (uint8_t)56, (uint8_t)120, (uint8_t)17, (uint8_t)50, (uint8_t)113, (uint8_t)110, (uint8_t)252, (uint8_t)203, (uint8_t)214, (uint8_t)94, (uint8_t)93, (uint8_t)254, (uint8_t)1, (uint8_t)188, (uint8_t)96, (uint8_t)254, (uint8_t)21, (uint8_t)49, (uint8_t)244, (uint8_t)20, (uint8_t)101, (uint8_t)222, (uint8_t)65, (uint8_t)140, (uint8_t)89, (uint8_t)212, (uint8_t)110, (uint8_t)2, (uint8_t)10, (uint8_t)81, (uint8_t)84, (uint8_t)207, (uint8_t)244, (uint8_t)31, (uint8_t)86, (uint8_t)92, (uint8_t)48, (uint8_t)102, (uint8_t)236, (uint8_t)88, (uint8_t)245, (uint8_t)24, (uint8_t)9, (uint8_t)21, (uint8_t)228, (uint8_t)146, (uint8_t)89, (uint8_t)206, (uint8_t)202, (uint8_t)254, (uint8_t)194, (uint8_t)162, (uint8_t)223, (uint8_t)146, (uint8_t)199, (uint8_t)204, (uint8_t)218, (uint8_t)69, (uint8_t)217, (uint8_t)5, (uint8_t)144, (uint8_t)230, (uint8_t)212, (uint8_t)240, (uint8_t)198, (uint8_t)141, (uint8_t)147, (uint8_t)233, (uint8_t)45, (uint8_t)58, (uint8_t)136, (uint8_t)67, (uint8_t)1, (uint8_t)115, (uint8_t)221, (uint8_t)90, (uint8_t)227, (uint8_t)86, (uint8_t)173, (uint8_t)41, (uint8_t)156, (uint8_t)223, (uint8_t)196, (uint8_t)103, (uint8_t)178, (uint8_t)114, (uint8_t)218, (uint8_t)137, (uint8_t)52, (uint8_t)35, (uint8_t)194, (uint8_t)216, (uint8_t)125, (uint8_t)188, (uint8_t)144, (uint8_t)190, (uint8_t)121, (uint8_t)153, (uint8_t)51, (uint8_t)91, (uint8_t)205, (uint8_t)239, (uint8_t)225, (uint8_t)6, (uint8_t)57, (uint8_t)143, (uint8_t)171, (uint8_t)242, (uint8_t)210, (uint8_t)21, (uint8_t)251, (uint8_t)197, (uint8_t)12, (uint8_t)249, (uint8_t)58, (uint8_t)196, (uint8_t)33, (uint8_t)162, (uint8_t)208, (uint8_t)157, (uint8_t)98, (uint8_t)149, (uint8_t)38, (uint8_t)37, (uint8_t)133, (uint8_t)89, (uint8_t)41, (uint8_t)155, (uint8_t)214, (uint8_t)178, (uint8_t)217, (uint8_t)218, (uint8_t)180, (uint8_t)16, (uint8_t)120, (uint8_t)164, (uint8_t)111, (uint8_t)151, (uint8_t)16, (uint8_t)77, (uint8_t)236, (uint8_t)34, (uint8_t)55, (uint8_t)93, (uint8_t)225, (uint8_t)125, (uint8_t)145, (uint8_t)13, (uint8_t)93, (uint8_t)236, (uint8_t)198, (uint8_t)64, (uint8_t)147, (uint8_t)4, (uint8_t)17, (uint8_t)188, (uint8_t)252, (uint8_t)149, (uint8_t)180, (uint8_t)14, (uint8_t)161, (uint8_t)123, (uint8_t)217, (uint8_t)118, (uint8_t)71, (uint8_t)188, (uint8_t)95, (uint8_t)166, (uint8_t)125, (uint8_t)61, (uint8_t)86, (uint8_t)190, (uint8_t)73, (uint8_t)11, (uint8_t)15, (uint8_t)157, (uint8_t)102, (uint8_t)116, (uint8_t)14, (uint8_t)80, (uint8_t)237, (uint8_t)243, (uint8_t)9, (uint8_t)32, (uint8_t)194, (uint8_t)141, (uint8_t)44, (uint8_t)87, (uint8_t)38, (uint8_t)146, (uint8_t)69, (uint8_t)91, (uint8_t)124, (uint8_t)87, (uint8_t)70, (uint8_t)239, (uint8_t)227, (uint8_t)176, (uint8_t)7, (uint8_t)21, (uint8_t)63, (uint8_t)93, (uint8_t)92, (uint8_t)107, (uint8_t)41, (uint8_t)253, (uint8_t)39, (uint8_t)246, (uint8_t)37, (uint8_t)88, (uint8_t)170, (uint8_t)237, (uint8_t)245, (uint8_t)246, (uint8_t)128, (uint8_t)52, (uint8_t)148, (uint8_t)139, (uint8_t)233, (uint8_t)138, (uint8_t)118, (uint8_t)78, (uint8_t)6, (uint8_t)153, (uint8_t)201, (uint8_t)162, (uint8_t)200, (uint8_t)216, (uint8_t)79, (uint8_t)40, (uint8_t)143, (uint8_t)147, (uint8_t)38, (uint8_t)108} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)30241);
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)143);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)196);
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)9);
};


void c_CommunicationChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)126);
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)23191);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)129);
};


void c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)15664);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)18096);
    assert(p269_uri_LEN(ph) == 113);
    {
        char16_t * exemplary = u"LnbijXefehcblnsiypjotzuamhtuoufDpoxphfcjgroyyzoqpgirfvobpdgikGttkmcyvmjcktzfvBykxkdknzdvfCBldkyidzHbkneInbuatmuyc";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 226);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_framerate_GET(pack) == (float) -2.9854421E38F);
    assert(p269_bitrate_GET(pack) == (uint32_t)1692809756L);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)9569);
};


void c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_framerate_GET(pack) == (float) -1.658811E38F);
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)41);
    assert(p270_uri_LEN(ph) == 110);
    {
        char16_t * exemplary = u"QgykjgfxsMzaiKuejhrcsiucbqkppbzvkdNiupkegVgalWvghofzrcopNuyfwynjttuoenlqxhfPtJcwOfbaoHefpzbyrzrvrzxumldPghfbDk";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 220);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_bitrate_GET(pack) == (uint32_t)290694674L);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)59110);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)60561);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)16634);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)161);
};


void c_CommunicationChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_ssid_LEN(ph) == 9);
    {
        char16_t * exemplary = u"wkuEdgbtD";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_password_LEN(ph) == 2);
    {
        char16_t * exemplary = u"vt";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)126, (uint8_t)20, (uint8_t)7, (uint8_t)108, (uint8_t)42, (uint8_t)250, (uint8_t)226, (uint8_t)196} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)199, (uint8_t)23, (uint8_t)44, (uint8_t)223, (uint8_t)75, (uint8_t)73, (uint8_t)208, (uint8_t)251} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)46514);
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)34609);
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)45046);
};


void c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL);
    assert(p310_uptime_sec_GET(pack) == (uint32_t)523861249L);
    assert(p310_time_usec_GET(pack) == (uint64_t)1183291904642880617L);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)23366);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_OK);
};


void c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_uptime_sec_GET(pack) == (uint32_t)2842300958L);
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)3549404889L);
    {
        uint8_t exemplary[] =  {(uint8_t)43, (uint8_t)186, (uint8_t)219, (uint8_t)190, (uint8_t)122, (uint8_t)218, (uint8_t)217, (uint8_t)176, (uint8_t)225, (uint8_t)224, (uint8_t)3, (uint8_t)56, (uint8_t)165, (uint8_t)224, (uint8_t)143, (uint8_t)173} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)232);
    assert(p311_time_usec_GET(pack) == (uint64_t)7398593798634825819L);
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p311_name_LEN(ph) == 35);
    {
        char16_t * exemplary = u"xbljknwMzwubvfqemuqqQueoewvfvsplhtw";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)216);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)30);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t) -27292);
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)111);
    assert(p320_param_id_LEN(ph) == 11);
    {
        char16_t * exemplary = u"znmprYxvkcr";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 22);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)26);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)192);
};


void c_CommunicationChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_value_LEN(ph) == 74);
    {
        char16_t * exemplary = u"ikxrXuRzterejufyoegbgcopvTdMhrjxrknsoEynvczoukNlrwfkzticdolKrtewcoljwJvvma";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 148);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16);
    assert(p322_param_id_LEN(ph) == 7);
    {
        char16_t * exemplary = u"Xprsnug";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)51381);
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)40376);
};


void c_CommunicationChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_param_value_LEN(ph) == 66);
    {
        char16_t * exemplary = u"gsivxGnwdldwlzjepwgrvvylvTBmnuownzmlgUhncttfeTepEwCmelzdabnbdzpvmk";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 132);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_id_LEN(ph) == 11);
    {
        char16_t * exemplary = u"Bkparfegalk";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 22);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_CUSTOM);
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)246);
};


void c_CommunicationChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_id_LEN(ph) == 2);
    {
        char16_t * exemplary = u"kz";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_value_LEN(ph) == 58);
    {
        char16_t * exemplary = u"pzcxXkjghxhoitiathuTtdtyPmynnnpfyoaavipfdrbkoymleiXxifhcXO";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 116);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT64);
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_VALUE_UNSUPPORTED);
};


void c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)65263);
    assert(p330_time_usec_GET(pack) == (uint64_t)4385629047192520414L);
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)33275);
    {
        uint16_t exemplary[] =  {(uint16_t)21443, (uint16_t)34606, (uint16_t)35393, (uint16_t)65145, (uint16_t)26338, (uint16_t)9089, (uint16_t)188, (uint16_t)55243, (uint16_t)35763, (uint16_t)64303, (uint16_t)41585, (uint16_t)56993, (uint16_t)56927, (uint16_t)28396, (uint16_t)46145, (uint16_t)47330, (uint16_t)51504, (uint16_t)5697, (uint16_t)36670, (uint16_t)23611, (uint16_t)21541, (uint16_t)63132, (uint16_t)37865, (uint16_t)38743, (uint16_t)9500, (uint16_t)22516, (uint16_t)60559, (uint16_t)35860, (uint16_t)20496, (uint16_t)46724, (uint16_t)14529, (uint16_t)16219, (uint16_t)16909, (uint16_t)28796, (uint16_t)15425, (uint16_t)45399, (uint16_t)788, (uint16_t)10256, (uint16_t)10273, (uint16_t)30856, (uint16_t)19733, (uint16_t)30072, (uint16_t)16282, (uint16_t)14692, (uint16_t)3539, (uint16_t)45549, (uint16_t)19918, (uint16_t)63568, (uint16_t)57312, (uint16_t)2081, (uint16_t)49972, (uint16_t)32833, (uint16_t)59649, (uint16_t)13689, (uint16_t)13871, (uint16_t)17736, (uint16_t)35725, (uint16_t)53197, (uint16_t)29896, (uint16_t)1616, (uint16_t)8496, (uint16_t)61918, (uint16_t)8413, (uint16_t)9009, (uint16_t)15637, (uint16_t)61412, (uint16_t)40067, (uint16_t)34381, (uint16_t)59009, (uint16_t)29900, (uint16_t)65288, (uint16_t)63080} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR);
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
        p0_mavlink_version_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_PARAFOIL, PH.base.pack) ;
        p0_custom_mode_SET((uint32_t)2221666408L, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_INVALID, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_STANDBY, PH.base.pack) ;
        p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED, PH.base.pack) ;
        c_TEST_Channel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
        p1_load_SET((uint16_t)(uint16_t)51574, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)3304, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)16852, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)47409, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t) -12000, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t)113, PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)56723, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)3178, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)5868, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)55054, PH.base.pack) ;
        c_TEST_Channel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_boot_ms_SET((uint32_t)2303755343L, PH.base.pack) ;
        p2_time_unix_usec_SET((uint64_t)3533646915999010566L, PH.base.pack) ;
        c_TEST_Channel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_vz_SET((float)2.6061847E36F, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)37257, PH.base.pack) ;
        p3_z_SET((float)8.443878E35F, PH.base.pack) ;
        p3_x_SET((float)1.9504634E37F, PH.base.pack) ;
        p3_vx_SET((float)1.9356047E37F, PH.base.pack) ;
        p3_afx_SET((float) -3.1696549E38F, PH.base.pack) ;
        p3_yaw_rate_SET((float)2.4300759E38F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)1354866517L, PH.base.pack) ;
        p3_vy_SET((float)3.2471353E38F, PH.base.pack) ;
        p3_afz_SET((float)2.60396E38F, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
        p3_afy_SET((float) -2.802668E38F, PH.base.pack) ;
        p3_yaw_SET((float)2.2967965E38F, PH.base.pack) ;
        p3_y_SET((float) -1.8739448E38F, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PING_4(), &PH);
        p4_target_component_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        p4_time_usec_SET((uint64_t)867110182192984774L, PH.base.pack) ;
        p4_target_system_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
        p4_seq_SET((uint32_t)1143921337L, PH.base.pack) ;
        c_TEST_Channel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        {
            char16_t* passkey = u"wpazG";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_version_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p5_target_system_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p5_control_request_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_ack_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p6_control_request_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"bauzMlzizznhduwz";
            p7_key_SET_(key, &PH) ;
        }
        c_TEST_Channel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
        p11_target_system_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_ARMED, PH.base.pack) ;
        p11_custom_mode_SET((uint32_t)2054899685L, PH.base.pack) ;
        c_TEST_Channel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
        p20_target_component_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        {
            char16_t* param_id = u"by";
            p20_param_id_SET_(param_id, &PH) ;
        }
        p20_target_system_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        p20_param_index_SET((int16_t)(int16_t) -21103, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_component_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        p21_target_system_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
        {
            char16_t* param_id = u"q";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_count_SET((uint16_t)(uint16_t)64985, PH.base.pack) ;
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64, PH.base.pack) ;
        p22_param_value_SET((float) -7.657659E37F, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)14461, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64, PH.base.pack) ;
        {
            char16_t* param_id = u"kmKzBaia";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_target_component_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p23_target_system_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        p23_param_value_SET((float)2.1027516E38F, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)3399672932L, &PH) ;
        p24_lon_SET((int32_t)598711829, PH.base.pack) ;
        p24_eph_SET((uint16_t)(uint16_t)34655, PH.base.pack) ;
        p24_epv_SET((uint16_t)(uint16_t)49693, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)41090, PH.base.pack) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p24_v_acc_SET((uint32_t)3700695610L, &PH) ;
        p24_lat_SET((int32_t)2142856508, PH.base.pack) ;
        p24_hdg_acc_SET((uint32_t)1481388067L, &PH) ;
        p24_alt_SET((int32_t)1127467116, PH.base.pack) ;
        p24_alt_ellipsoid_SET((int32_t)1786262293, &PH) ;
        p24_vel_SET((uint16_t)(uint16_t)3104, PH.base.pack) ;
        p24_h_acc_SET((uint32_t)56114960L, &PH) ;
        p24_time_usec_SET((uint64_t)1392793432039439373L, PH.base.pack) ;
        c_TEST_Channel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)93, (uint8_t)54, (uint8_t)37, (uint8_t)101, (uint8_t)194, (uint8_t)106, (uint8_t)85, (uint8_t)177, (uint8_t)185, (uint8_t)118, (uint8_t)91, (uint8_t)103, (uint8_t)237, (uint8_t)113, (uint8_t)185, (uint8_t)77, (uint8_t)243, (uint8_t)129, (uint8_t)221, (uint8_t)189};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)38, (uint8_t)232, (uint8_t)181, (uint8_t)14, (uint8_t)233, (uint8_t)30, (uint8_t)122, (uint8_t)133, (uint8_t)245, (uint8_t)1, (uint8_t)62, (uint8_t)141, (uint8_t)141, (uint8_t)60, (uint8_t)126, (uint8_t)98, (uint8_t)155, (uint8_t)214, (uint8_t)186, (uint8_t)107};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_prn[] =  {(uint8_t)234, (uint8_t)185, (uint8_t)80, (uint8_t)105, (uint8_t)108, (uint8_t)128, (uint8_t)154, (uint8_t)87, (uint8_t)96, (uint8_t)131, (uint8_t)66, (uint8_t)205, (uint8_t)196, (uint8_t)184, (uint8_t)201, (uint8_t)74, (uint8_t)4, (uint8_t)132, (uint8_t)221, (uint8_t)127};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_snr[] =  {(uint8_t)125, (uint8_t)248, (uint8_t)201, (uint8_t)252, (uint8_t)185, (uint8_t)20, (uint8_t)224, (uint8_t)5, (uint8_t)93, (uint8_t)78, (uint8_t)97, (uint8_t)26, (uint8_t)154, (uint8_t)8, (uint8_t)177, (uint8_t)197, (uint8_t)50, (uint8_t)135, (uint8_t)38, (uint8_t)143};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_used[] =  {(uint8_t)235, (uint8_t)161, (uint8_t)140, (uint8_t)217, (uint8_t)128, (uint8_t)127, (uint8_t)17, (uint8_t)242, (uint8_t)150, (uint8_t)181, (uint8_t)49, (uint8_t)2, (uint8_t)244, (uint8_t)162, (uint8_t)56, (uint8_t)141, (uint8_t)22, (uint8_t)175, (uint8_t)168, (uint8_t)131};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        c_TEST_Channel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
        p26_xmag_SET((int16_t)(int16_t)28101, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t) -9459, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t) -987, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t) -14826, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t) -25948, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t) -23150, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t)13130, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t) -9569, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)1630607337L, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t)13209, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
        p27_zgyro_SET((int16_t)(int16_t) -27246, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t)30834, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)3160921651594694711L, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t) -14116, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t) -1, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t) -32572, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t) -10507, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t) -27646, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t)13102, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t) -26325, PH.base.pack) ;
        c_TEST_Channel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
        p28_time_usec_SET((uint64_t)5019872799998961054L, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t)9778, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t)25917, PH.base.pack) ;
        p28_press_diff1_SET((int16_t)(int16_t) -5702, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t)22104, PH.base.pack) ;
        c_TEST_Channel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_temperature_SET((int16_t)(int16_t)28587, PH.base.pack) ;
        p29_press_diff_SET((float)5.936642E37F, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)1590722016L, PH.base.pack) ;
        p29_press_abs_SET((float) -2.4840997E38F, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
        p30_roll_SET((float) -2.4379874E38F, PH.base.pack) ;
        p30_yaw_SET((float)3.1624132E38F, PH.base.pack) ;
        p30_rollspeed_SET((float)2.2957793E38F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)2790532839L, PH.base.pack) ;
        p30_pitchspeed_SET((float)2.5860422E37F, PH.base.pack) ;
        p30_pitch_SET((float)2.8465143E38F, PH.base.pack) ;
        p30_yawspeed_SET((float)1.11737E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_pitchspeed_SET((float)1.1345294E38F, PH.base.pack) ;
        p31_q1_SET((float)3.1284992E38F, PH.base.pack) ;
        p31_q4_SET((float) -2.526291E37F, PH.base.pack) ;
        p31_yawspeed_SET((float) -2.3087477E38F, PH.base.pack) ;
        p31_q3_SET((float) -2.8676433E38F, PH.base.pack) ;
        p31_rollspeed_SET((float) -1.6666187E38F, PH.base.pack) ;
        p31_q2_SET((float) -1.5882945E38F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)1462098181L, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_x_SET((float) -1.5064892E38F, PH.base.pack) ;
        p32_vz_SET((float)1.9013037E38F, PH.base.pack) ;
        p32_y_SET((float)1.9394906E38F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)1438173946L, PH.base.pack) ;
        p32_vy_SET((float)4.8915863E37F, PH.base.pack) ;
        p32_z_SET((float)1.3732364E37F, PH.base.pack) ;
        p32_vx_SET((float) -2.9538447E37F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_hdg_SET((uint16_t)(uint16_t)63253, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)2219127252L, PH.base.pack) ;
        p33_lon_SET((int32_t) -54814593, PH.base.pack) ;
        p33_lat_SET((int32_t)270299175, PH.base.pack) ;
        p33_alt_SET((int32_t) -1323332920, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t) -910, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t) -6960, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t)15901, PH.base.pack) ;
        p33_relative_alt_SET((int32_t)1593336398, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan8_scaled_SET((int16_t)(int16_t)32753, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t) -19971, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t)24150, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t)3103, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)4240847923L, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t)6371, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t) -18292, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t)29501, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t) -29592, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_chan7_raw_SET((uint16_t)(uint16_t)16017, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)310482535L, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)51915, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)7917, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)35893, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)41192, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)33018, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)33404, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)26541, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo15_raw_SET((uint16_t)(uint16_t)57584, &PH) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)17700, PH.base.pack) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)41092, &PH) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)4100, &PH) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)64010, &PH) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)12866, PH.base.pack) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)13929, &PH) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)20716, &PH) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)18098, PH.base.pack) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)19360, PH.base.pack) ;
        p36_port_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)47435, PH.base.pack) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)11248, PH.base.pack) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)18096, &PH) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)13947, PH.base.pack) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)49800, &PH) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)25907, PH.base.pack) ;
        p36_time_usec_SET((uint32_t)219674282L, PH.base.pack) ;
        c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_target_system_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p37_start_index_SET((int16_t)(int16_t)3445, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t) -9336, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_target_component_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t) -25300, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t) -11920, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
        p39_param2_SET((float) -3.1104521E38F, PH.base.pack) ;
        p39_x_SET((float) -2.3200975E38F, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p39_param3_SET((float)2.6416642E38F, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        p39_z_SET((float)6.3445923E37F, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        p39_param4_SET((float)2.7659544E38F, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_STORAGE_FORMAT, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)10393, PH.base.pack) ;
        p39_y_SET((float) -2.933072E37F, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        p39_param1_SET((float)3.3293117E38F, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
        p40_target_component_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)64477, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_seq_SET((uint16_t)(uint16_t)33224, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        p41_target_system_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)29654, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p43_target_system_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p44_count_SET((uint16_t)(uint16_t)13739, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_target_system_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)6784, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM3, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_target_system_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p48_longitude_SET((int32_t) -1653045921, PH.base.pack) ;
        p48_altitude_SET((int32_t)729989718, PH.base.pack) ;
        p48_latitude_SET((int32_t) -1816248353, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)1724899279251753948L, &PH) ;
        c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_time_usec_SET((uint64_t)8488947208253689794L, &PH) ;
        p49_longitude_SET((int32_t)800932314, PH.base.pack) ;
        p49_latitude_SET((int32_t) -1437309319, PH.base.pack) ;
        p49_altitude_SET((int32_t)430531497, PH.base.pack) ;
        c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p50_scale_SET((float)4.0614843E37F, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t)29735, PH.base.pack) ;
        {
            char16_t* param_id = u"vhz";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_target_system_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p50_param_value0_SET((float) -6.0586093E37F, PH.base.pack) ;
        p50_param_value_max_SET((float) -8.183716E37F, PH.base.pack) ;
        p50_param_value_min_SET((float)2.5909898E38F, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_target_component_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)57952, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_p1z_SET((float)1.1577633E38F, PH.base.pack) ;
        p54_p1x_SET((float) -6.402852E37F, PH.base.pack) ;
        p54_p1y_SET((float) -6.152469E37F, PH.base.pack) ;
        p54_p2y_SET((float) -1.2204764E38F, PH.base.pack) ;
        p54_p2x_SET((float)3.0362597E38F, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p54_p2z_SET((float)3.3855867E37F, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
        p55_p2x_SET((float)2.470024E38F, PH.base.pack) ;
        p55_p1x_SET((float) -3.1577661E38F, PH.base.pack) ;
        p55_p2z_SET((float)2.9955683E38F, PH.base.pack) ;
        p55_p1y_SET((float) -1.7954867E38F, PH.base.pack) ;
        p55_p2y_SET((float)3.155085E38F, PH.base.pack) ;
        p55_p1z_SET((float) -2.024351E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_rollspeed_SET((float) -9.221051E37F, PH.base.pack) ;
        p61_pitchspeed_SET((float)1.0662435E38F, PH.base.pack) ;
        p61_yawspeed_SET((float) -2.4428547E37F, PH.base.pack) ;
        {
            float covariance[] =  {2.0060321E38F, -6.8210646E37F, -1.6508405E38F, -2.143125E38F, -1.4958156E38F, -2.058654E38F, 9.324558E37F, -3.1155345E37F, 2.7696539E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p61_time_usec_SET((uint64_t)6478808968165330614L, PH.base.pack) ;
        {
            float q[] =  {-1.6737202E38F, 1.7702637E37F, -5.058709E37F, -2.8162312E38F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_nav_bearing_SET((int16_t)(int16_t) -855, PH.base.pack) ;
        p62_xtrack_error_SET((float)1.9417908E38F, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t) -24621, PH.base.pack) ;
        p62_aspd_error_SET((float)1.2924985E38F, PH.base.pack) ;
        p62_nav_pitch_SET((float)3.1180177E38F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)574, PH.base.pack) ;
        p62_nav_roll_SET((float)2.5074431E38F, PH.base.pack) ;
        p62_alt_error_SET((float)3.0493327E38F, PH.base.pack) ;
        c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_lon_SET((int32_t) -1413123874, PH.base.pack) ;
        p63_alt_SET((int32_t) -857341023, PH.base.pack) ;
        p63_relative_alt_SET((int32_t)1570996567, PH.base.pack) ;
        p63_vx_SET((float)7.609964E37F, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)5739622895817135010L, PH.base.pack) ;
        p63_vy_SET((float) -5.563327E36F, PH.base.pack) ;
        {
            float covariance[] =  {-2.577218E38F, -2.6615917E38F, -1.2660305E38F, -2.2077695E38F, 3.9712573E37F, 2.6837086E38F, 1.6574327E38F, 1.5092356E38F, 1.4018865E38F, -3.9068018E37F, 6.2782316E36F, -2.5717694E38F, 1.2485264E38F, -1.8522303E37F, -5.54099E37F, 3.0952582E38F, 2.2744522E38F, 9.932919E37F, 2.9236578E38F, 2.8878294E38F, 3.0720305E38F, -2.317474E38F, -6.341311E37F, -3.0346766E38F, 2.7237666E38F, 2.3980617E38F, 2.5199748E38F, -1.2824314E38F, 1.8069325E38F, -1.8560259E38F, -3.1268454E38F, 4.1166872E37F, 2.9160655E38F, -9.99869E37F, -3.1272737E38F, -7.126759E36F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_vz_SET((float)2.1762072E38F, PH.base.pack) ;
        p63_lat_SET((int32_t)2146201980, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_vx_SET((float) -8.743383E37F, PH.base.pack) ;
        p64_y_SET((float)2.3565984E38F, PH.base.pack) ;
        p64_x_SET((float)3.1484636E38F, PH.base.pack) ;
        p64_vz_SET((float)9.234945E37F, PH.base.pack) ;
        p64_ay_SET((float) -9.646949E37F, PH.base.pack) ;
        p64_az_SET((float) -4.465342E37F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
        {
            float covariance[] =  {3.027161E38F, -1.5583886E38F, 8.0492536E36F, -4.869038E37F, -2.5012006E38F, 1.3852462E38F, 2.8684294E38F, 3.0177938E38F, -3.055699E38F, -1.0883535E38F, 1.0534159E38F, -2.9112188E38F, 2.6012467E37F, -3.105432E38F, 3.3805925E38F, -3.184458E37F, 3.775455E37F, -2.752562E38F, 1.744714E38F, 2.3228693E38F, 1.0079246E38F, -5.2534874E37F, 2.3263247E37F, 2.509188E38F, 1.1847145E38F, 9.936895E37F, -1.6030006E38F, 2.0308779E38F, -2.0668889E38F, 1.2749677E38F, -5.056746E37F, -2.2441645E38F, 2.4775392E37F, 1.3275389E38F, 2.7463107E38F, 3.331919E38F, -6.017658E36F, -2.7375775E38F, -3.2139437E38F, 1.2643611E38F, 5.877602E37F, -1.5484435E38F, -5.8073797E37F, 2.805427E38F, -2.798528E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_time_usec_SET((uint64_t)1750864055655260234L, PH.base.pack) ;
        p64_z_SET((float)1.9320332E38F, PH.base.pack) ;
        p64_vy_SET((float) -1.3358435E38F, PH.base.pack) ;
        p64_ax_SET((float) -2.0594071E38F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan9_raw_SET((uint16_t)(uint16_t)58911, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)39677, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)5690, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)15878, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)65434, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)62491, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)27345, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)2157, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)63192, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)16038, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)27747, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)30308, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)29253, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)43660, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)39245, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)2134229912L, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)24714, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)43217, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)20505, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_req_stream_id_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
        p66_req_message_rate_SET((uint16_t)(uint16_t)14020, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        c_TEST_Channel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
        p67_stream_id_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        p67_on_off_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        p67_message_rate_SET((uint16_t)(uint16_t)38574, PH.base.pack) ;
        c_TEST_Channel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_y_SET((int16_t)(int16_t) -9226, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t) -30589, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t)23847, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
        p69_buttons_SET((uint16_t)(uint16_t)36680, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t)1148, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan3_raw_SET((uint16_t)(uint16_t)44348, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)19879, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)51699, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)38246, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)22398, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)58350, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)15991, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)43814, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_param1_SET((float)3.2669303E38F, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_SPATIAL_USER_1, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)51791, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p73_param3_SET((float) -1.999876E38F, PH.base.pack) ;
        p73_y_SET((int32_t)1548756174, PH.base.pack) ;
        p73_param4_SET((float) -9.160898E37F, PH.base.pack) ;
        p73_param2_SET((float)3.1028336E38F, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p73_x_SET((int32_t)1303066455, PH.base.pack) ;
        p73_z_SET((float) -2.8730786E38F, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
        p74_airspeed_SET((float)3.2499574E38F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t) -24241, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)24230, PH.base.pack) ;
        p74_groundspeed_SET((float)2.4690623E38F, PH.base.pack) ;
        p74_climb_SET((float)1.8729682E38F, PH.base.pack) ;
        p74_alt_SET((float)3.897891E37F, PH.base.pack) ;
        c_CommunicationChannel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
        p75_param2_SET((float)2.7701027E38F, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        p75_x_SET((int32_t)2035452180, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        p75_param4_SET((float) -1.252562E38F, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p75_z_SET((float)2.1351842E38F, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        p75_y_SET((int32_t) -144001565, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_START_RX_PAIR, PH.base.pack) ;
        p75_param1_SET((float) -2.6167545E38F, PH.base.pack) ;
        p75_param3_SET((float) -3.3778355E38F, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        c_CommunicationChannel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
        p76_target_system_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        p76_param7_SET((float)2.0970926E38F, PH.base.pack) ;
        p76_param2_SET((float) -2.7809843E38F, PH.base.pack) ;
        p76_param5_SET((float)2.0129981E38F, PH.base.pack) ;
        p76_param3_SET((float) -1.1456784E38F, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p76_param1_SET((float)7.4873193E36F, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD, PH.base.pack) ;
        p76_param4_SET((float)1.4496854E38F, PH.base.pack) ;
        p76_param6_SET((float) -1.5695167E38F, PH.base.pack) ;
        c_CommunicationChannel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
        p77_progress_SET((uint8_t)(uint8_t)84, &PH) ;
        p77_target_system_SET((uint8_t)(uint8_t)38, &PH) ;
        p77_target_component_SET((uint8_t)(uint8_t)177, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_FAILED, PH.base.pack) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY, PH.base.pack) ;
        p77_result_param2_SET((int32_t)473233479, &PH) ;
        c_CommunicationChannel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_pitch_SET((float) -6.2668016E37F, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)2649196596L, PH.base.pack) ;
        p81_thrust_SET((float) -2.0326719E38F, PH.base.pack) ;
        p81_yaw_SET((float)4.1401243E37F, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p81_roll_SET((float)1.7505255E38F, PH.base.pack) ;
        c_CommunicationChannel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_body_roll_rate_SET((float)3.2194123E38F, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p82_thrust_SET((float)2.0440803E38F, PH.base.pack) ;
        p82_body_yaw_rate_SET((float) -1.7441718E38F, PH.base.pack) ;
        p82_time_boot_ms_SET((uint32_t)1183948301L, PH.base.pack) ;
        {
            float q[] =  {-7.974049E37F, -5.3335877E37F, -1.5337702E38F, 9.76182E37F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_body_pitch_rate_SET((float) -6.2256415E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_body_yaw_rate_SET((float) -5.544431E37F, PH.base.pack) ;
        {
            float q[] =  {3.2538451E38F, 2.5106944E38F, -3.3164577E38F, -3.0150173E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_thrust_SET((float)2.4050034E38F, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)2258915363L, PH.base.pack) ;
        p83_type_mask_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p83_body_roll_rate_SET((float)9.896418E37F, PH.base.pack) ;
        p83_body_pitch_rate_SET((float)1.4259926E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        p84_vx_SET((float)2.7327547E38F, PH.base.pack) ;
        p84_afz_SET((float)2.9756994E38F, PH.base.pack) ;
        p84_yaw_rate_SET((float) -5.5574444E35F, PH.base.pack) ;
        p84_x_SET((float)5.483587E37F, PH.base.pack) ;
        p84_afx_SET((float) -1.6763401E38F, PH.base.pack) ;
        p84_vz_SET((float)3.7530188E37F, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        p84_yaw_SET((float)5.4748126E37F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)2277559524L, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)36235, PH.base.pack) ;
        p84_afy_SET((float) -2.8951262E38F, PH.base.pack) ;
        p84_y_SET((float)3.3038174E37F, PH.base.pack) ;
        p84_z_SET((float)2.5880274E38F, PH.base.pack) ;
        p84_vy_SET((float)1.614217E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_vz_SET((float)9.221703E37F, PH.base.pack) ;
        p86_lat_int_SET((int32_t)737468659, PH.base.pack) ;
        p86_vy_SET((float)5.5336245E37F, PH.base.pack) ;
        p86_lon_int_SET((int32_t) -2015101432, PH.base.pack) ;
        p86_alt_SET((float)1.6357319E38F, PH.base.pack) ;
        p86_afy_SET((float) -6.1872104E37F, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)1587837483L, PH.base.pack) ;
        p86_afz_SET((float) -1.935852E38F, PH.base.pack) ;
        p86_vx_SET((float) -7.566171E37F, PH.base.pack) ;
        p86_yaw_SET((float) -9.403621E37F, PH.base.pack) ;
        p86_yaw_rate_SET((float) -1.7884633E38F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)29620, PH.base.pack) ;
        p86_afx_SET((float) -2.3823445E38F, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_yaw_rate_SET((float) -1.5253487E38F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)60862, PH.base.pack) ;
        p87_vx_SET((float) -5.531325E37F, PH.base.pack) ;
        p87_vy_SET((float) -8.941055E37F, PH.base.pack) ;
        p87_lat_int_SET((int32_t)280732925, PH.base.pack) ;
        p87_afx_SET((float)1.7293835E38F, PH.base.pack) ;
        p87_afz_SET((float)2.5208048E38F, PH.base.pack) ;
        p87_vz_SET((float)2.6864021E38F, PH.base.pack) ;
        p87_lon_int_SET((int32_t) -805140550, PH.base.pack) ;
        p87_afy_SET((float)2.4087607E38F, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)1539783912L, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p87_yaw_SET((float)2.625362E37F, PH.base.pack) ;
        p87_alt_SET((float)2.2490724E38F, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_time_boot_ms_SET((uint32_t)2927559060L, PH.base.pack) ;
        p89_y_SET((float) -1.4831896E38F, PH.base.pack) ;
        p89_z_SET((float) -1.1801731E37F, PH.base.pack) ;
        p89_pitch_SET((float)1.173822E36F, PH.base.pack) ;
        p89_x_SET((float)2.1313748E38F, PH.base.pack) ;
        p89_roll_SET((float)1.3810173E38F, PH.base.pack) ;
        p89_yaw_SET((float)2.6649792E38F, PH.base.pack) ;
        c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
        p90_alt_SET((int32_t) -884847031, PH.base.pack) ;
        p90_yaw_SET((float) -6.3059735E37F, PH.base.pack) ;
        p90_yawspeed_SET((float) -4.6084844E37F, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t)24461, PH.base.pack) ;
        p90_lat_SET((int32_t)46033092, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t)24866, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t)12193, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t)18084, PH.base.pack) ;
        p90_pitchspeed_SET((float)6.057332E37F, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t)1227, PH.base.pack) ;
        p90_rollspeed_SET((float)2.533746E38F, PH.base.pack) ;
        p90_roll_SET((float) -1.2767851E38F, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t)3537, PH.base.pack) ;
        p90_pitch_SET((float) -5.573744E37F, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)4761075264404981053L, PH.base.pack) ;
        p90_lon_SET((int32_t) -671044021, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
        p91_aux3_SET((float)3.195682E38F, PH.base.pack) ;
        p91_aux4_SET((float) -1.0928701E38F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)2521045793685330362L, PH.base.pack) ;
        p91_roll_ailerons_SET((float)5.772481E37F, PH.base.pack) ;
        p91_aux1_SET((float) -1.8049278E38F, PH.base.pack) ;
        p91_throttle_SET((float) -1.0594665E38F, PH.base.pack) ;
        p91_yaw_rudder_SET((float) -1.3171051E37F, PH.base.pack) ;
        p91_aux2_SET((float)1.1556709E38F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        p91_pitch_elevator_SET((float) -5.4908656E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan1_raw_SET((uint16_t)(uint16_t)52398, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)27518, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)47594, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)2923960796231749404L, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)63526, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)904, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)36732, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)24791, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)24444, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)994, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)5742, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)15229, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)18176, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_ARMED, PH.base.pack) ;
        p93_flags_SET((uint64_t)8182773568483499392L, PH.base.pack) ;
        p93_time_usec_SET((uint64_t)2059834387021081645L, PH.base.pack) ;
        {
            float controls[] =  {-2.4814603E38F, -9.052799E37F, -4.6621426E37F, 5.1576155E37F, 1.553543E37F, -1.898137E38F, -6.20488E37F, -7.151764E37F, 3.0546386E36F, 1.0833605E38F, 7.0779484E37F, -1.3312824E38F, 2.552502E38F, 1.8674159E38F, 2.2928092E38F, -2.068036E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_quality_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p100_ground_distance_SET((float) -2.5820777E38F, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)1785343245002181180L, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float) -6.016133E37F, PH.base.pack) ;
        p100_flow_rate_y_SET((float)1.7968345E38F, &PH) ;
        p100_flow_y_SET((int16_t)(int16_t)23801, PH.base.pack) ;
        p100_flow_rate_x_SET((float) -1.7542165E38F, &PH) ;
        p100_flow_x_SET((int16_t)(int16_t) -20913, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float) -2.0152485E38F, PH.base.pack) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_x_SET((float)2.4416875E37F, PH.base.pack) ;
        p101_yaw_SET((float)2.0164232E38F, PH.base.pack) ;
        p101_pitch_SET((float) -2.0628685E38F, PH.base.pack) ;
        p101_y_SET((float)3.0237047E38F, PH.base.pack) ;
        p101_z_SET((float) -3.6809995E37F, PH.base.pack) ;
        p101_roll_SET((float)2.2978327E38F, PH.base.pack) ;
        p101_usec_SET((uint64_t)8143982069132596138L, PH.base.pack) ;
        c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_y_SET((float) -2.8366773E37F, PH.base.pack) ;
        p102_usec_SET((uint64_t)360562127219600640L, PH.base.pack) ;
        p102_z_SET((float)1.4480473E38F, PH.base.pack) ;
        p102_x_SET((float) -2.624997E38F, PH.base.pack) ;
        p102_roll_SET((float) -7.945485E37F, PH.base.pack) ;
        p102_yaw_SET((float)2.3130597E38F, PH.base.pack) ;
        p102_pitch_SET((float)2.3732014E37F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_usec_SET((uint64_t)1211192891050447536L, PH.base.pack) ;
        p103_z_SET((float)1.6859611E37F, PH.base.pack) ;
        p103_x_SET((float) -1.5815744E37F, PH.base.pack) ;
        p103_y_SET((float)1.6485353E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_yaw_SET((float)8.65666E37F, PH.base.pack) ;
        p104_z_SET((float)1.3048371E38F, PH.base.pack) ;
        p104_usec_SET((uint64_t)1499602390536947725L, PH.base.pack) ;
        p104_roll_SET((float) -2.2306844E38F, PH.base.pack) ;
        p104_x_SET((float)2.712264E38F, PH.base.pack) ;
        p104_pitch_SET((float)2.3223375E38F, PH.base.pack) ;
        p104_y_SET((float) -2.3945348E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
        p105_diff_pressure_SET((float) -2.568845E38F, PH.base.pack) ;
        p105_pressure_alt_SET((float) -3.3678343E38F, PH.base.pack) ;
        p105_xmag_SET((float)2.971489E37F, PH.base.pack) ;
        p105_abs_pressure_SET((float) -5.769311E36F, PH.base.pack) ;
        p105_xacc_SET((float) -2.806662E38F, PH.base.pack) ;
        p105_ymag_SET((float)1.3882769E38F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)50039, PH.base.pack) ;
        p105_zgyro_SET((float)3.2045856E38F, PH.base.pack) ;
        p105_zacc_SET((float)2.261633E38F, PH.base.pack) ;
        p105_yacc_SET((float) -1.305332E38F, PH.base.pack) ;
        p105_ygyro_SET((float) -8.76715E37F, PH.base.pack) ;
        p105_xgyro_SET((float)1.6890764E38F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)5105223883275752616L, PH.base.pack) ;
        p105_zmag_SET((float) -8.603375E37F, PH.base.pack) ;
        p105_temperature_SET((float) -3.0959955E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_time_delta_distance_us_SET((uint32_t)2958049646L, PH.base.pack) ;
        p106_integrated_zgyro_SET((float)3.0897326E38F, PH.base.pack) ;
        p106_integrated_y_SET((float)3.3956214E38F, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)6071555478050683830L, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t) -21747, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        p106_distance_SET((float)2.426386E38F, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)2304841550L, PH.base.pack) ;
        p106_integrated_x_SET((float)5.7730157E37F, PH.base.pack) ;
        p106_integrated_xgyro_SET((float) -3.0740448E38F, PH.base.pack) ;
        p106_integrated_ygyro_SET((float) -7.442806E37F, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
        p107_fields_updated_SET((uint32_t)2278989202L, PH.base.pack) ;
        p107_abs_pressure_SET((float)2.8082308E38F, PH.base.pack) ;
        p107_zgyro_SET((float) -3.0461774E38F, PH.base.pack) ;
        p107_xgyro_SET((float)3.3849098E37F, PH.base.pack) ;
        p107_ymag_SET((float)2.9680453E38F, PH.base.pack) ;
        p107_diff_pressure_SET((float) -4.536224E37F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)337593339332657511L, PH.base.pack) ;
        p107_zmag_SET((float)2.226621E38F, PH.base.pack) ;
        p107_yacc_SET((float) -1.6122638E38F, PH.base.pack) ;
        p107_xacc_SET((float)3.18727E38F, PH.base.pack) ;
        p107_temperature_SET((float) -3.1369649E38F, PH.base.pack) ;
        p107_pressure_alt_SET((float)2.7836918E38F, PH.base.pack) ;
        p107_zacc_SET((float)3.3195234E38F, PH.base.pack) ;
        p107_xmag_SET((float) -1.0839854E38F, PH.base.pack) ;
        p107_ygyro_SET((float) -1.6792121E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
        p108_std_dev_horz_SET((float) -1.8079332E38F, PH.base.pack) ;
        p108_pitch_SET((float) -2.420902E37F, PH.base.pack) ;
        p108_yaw_SET((float)2.9995607E37F, PH.base.pack) ;
        p108_xacc_SET((float) -1.9951344E38F, PH.base.pack) ;
        p108_vd_SET((float)2.8705745E38F, PH.base.pack) ;
        p108_ve_SET((float) -2.600696E38F, PH.base.pack) ;
        p108_q3_SET((float)3.320931E38F, PH.base.pack) ;
        p108_lat_SET((float)1.5628176E38F, PH.base.pack) ;
        p108_q1_SET((float)2.144288E38F, PH.base.pack) ;
        p108_yacc_SET((float)7.7047033E37F, PH.base.pack) ;
        p108_vn_SET((float)2.4430408E38F, PH.base.pack) ;
        p108_lon_SET((float)1.4365794E38F, PH.base.pack) ;
        p108_zgyro_SET((float)1.952701E37F, PH.base.pack) ;
        p108_xgyro_SET((float)2.4323345E38F, PH.base.pack) ;
        p108_zacc_SET((float)2.058793E38F, PH.base.pack) ;
        p108_q2_SET((float)6.241178E35F, PH.base.pack) ;
        p108_roll_SET((float)6.954744E37F, PH.base.pack) ;
        p108_q4_SET((float)1.531516E38F, PH.base.pack) ;
        p108_std_dev_vert_SET((float) -9.0092606E36F, PH.base.pack) ;
        p108_ygyro_SET((float)6.5743793E37F, PH.base.pack) ;
        p108_alt_SET((float) -1.1081859E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
        p109_rxerrors_SET((uint16_t)(uint16_t)46347, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)28018, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        c_CommunicationChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        {
            uint8_t payload[] =  {(uint8_t)239, (uint8_t)211, (uint8_t)208, (uint8_t)110, (uint8_t)47, (uint8_t)118, (uint8_t)37, (uint8_t)202, (uint8_t)41, (uint8_t)154, (uint8_t)194, (uint8_t)152, (uint8_t)175, (uint8_t)39, (uint8_t)215, (uint8_t)76, (uint8_t)117, (uint8_t)140, (uint8_t)33, (uint8_t)44, (uint8_t)243, (uint8_t)128, (uint8_t)147, (uint8_t)209, (uint8_t)111, (uint8_t)121, (uint8_t)104, (uint8_t)202, (uint8_t)11, (uint8_t)31, (uint8_t)54, (uint8_t)164, (uint8_t)75, (uint8_t)106, (uint8_t)58, (uint8_t)208, (uint8_t)60, (uint8_t)130, (uint8_t)133, (uint8_t)170, (uint8_t)160, (uint8_t)24, (uint8_t)52, (uint8_t)73, (uint8_t)94, (uint8_t)21, (uint8_t)37, (uint8_t)113, (uint8_t)208, (uint8_t)102, (uint8_t)82, (uint8_t)185, (uint8_t)123, (uint8_t)107, (uint8_t)169, (uint8_t)120, (uint8_t)209, (uint8_t)22, (uint8_t)164, (uint8_t)143, (uint8_t)185, (uint8_t)64, (uint8_t)189, (uint8_t)128, (uint8_t)71, (uint8_t)236, (uint8_t)19, (uint8_t)139, (uint8_t)149, (uint8_t)131, (uint8_t)196, (uint8_t)20, (uint8_t)22, (uint8_t)205, (uint8_t)103, (uint8_t)35, (uint8_t)21, (uint8_t)80, (uint8_t)5, (uint8_t)78, (uint8_t)117, (uint8_t)132, (uint8_t)65, (uint8_t)43, (uint8_t)20, (uint8_t)77, (uint8_t)247, (uint8_t)173, (uint8_t)246, (uint8_t)227, (uint8_t)18, (uint8_t)201, (uint8_t)191, (uint8_t)185, (uint8_t)33, (uint8_t)54, (uint8_t)175, (uint8_t)79, (uint8_t)229, (uint8_t)132, (uint8_t)239, (uint8_t)248, (uint8_t)54, (uint8_t)123, (uint8_t)180, (uint8_t)208, (uint8_t)48, (uint8_t)208, (uint8_t)93, (uint8_t)230, (uint8_t)114, (uint8_t)233, (uint8_t)114, (uint8_t)220, (uint8_t)148, (uint8_t)42, (uint8_t)125, (uint8_t)125, (uint8_t)246, (uint8_t)81, (uint8_t)76, (uint8_t)26, (uint8_t)198, (uint8_t)124, (uint8_t)80, (uint8_t)38, (uint8_t)74, (uint8_t)128, (uint8_t)147, (uint8_t)127, (uint8_t)209, (uint8_t)178, (uint8_t)253, (uint8_t)48, (uint8_t)18, (uint8_t)137, (uint8_t)45, (uint8_t)178, (uint8_t)10, (uint8_t)150, (uint8_t)95, (uint8_t)80, (uint8_t)95, (uint8_t)91, (uint8_t)58, (uint8_t)140, (uint8_t)65, (uint8_t)221, (uint8_t)188, (uint8_t)244, (uint8_t)34, (uint8_t)140, (uint8_t)55, (uint8_t)32, (uint8_t)253, (uint8_t)149, (uint8_t)115, (uint8_t)117, (uint8_t)57, (uint8_t)243, (uint8_t)44, (uint8_t)170, (uint8_t)173, (uint8_t)13, (uint8_t)250, (uint8_t)62, (uint8_t)105, (uint8_t)160, (uint8_t)180, (uint8_t)164, (uint8_t)206, (uint8_t)248, (uint8_t)122, (uint8_t)10, (uint8_t)101, (uint8_t)226, (uint8_t)217, (uint8_t)29, (uint8_t)101, (uint8_t)241, (uint8_t)173, (uint8_t)44, (uint8_t)46, (uint8_t)114, (uint8_t)120, (uint8_t)84, (uint8_t)34, (uint8_t)232, (uint8_t)227, (uint8_t)70, (uint8_t)99, (uint8_t)86, (uint8_t)211, (uint8_t)115, (uint8_t)186, (uint8_t)32, (uint8_t)111, (uint8_t)237, (uint8_t)254, (uint8_t)136, (uint8_t)195, (uint8_t)63, (uint8_t)229, (uint8_t)64, (uint8_t)171, (uint8_t)37, (uint8_t)108, (uint8_t)160, (uint8_t)70, (uint8_t)82, (uint8_t)65, (uint8_t)169, (uint8_t)0, (uint8_t)165, (uint8_t)60, (uint8_t)76, (uint8_t)4, (uint8_t)0, (uint8_t)70, (uint8_t)235, (uint8_t)199, (uint8_t)144, (uint8_t)121, (uint8_t)54, (uint8_t)79, (uint8_t)237, (uint8_t)52, (uint8_t)28, (uint8_t)96, (uint8_t)91, (uint8_t)66, (uint8_t)250, (uint8_t)31, (uint8_t)201, (uint8_t)173, (uint8_t)90, (uint8_t)77, (uint8_t)232, (uint8_t)52, (uint8_t)168, (uint8_t)4, (uint8_t)28, (uint8_t)27, (uint8_t)225, (uint8_t)207, (uint8_t)243, (uint8_t)12, (uint8_t)166, (uint8_t)192, (uint8_t)85, (uint8_t)129};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_system_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p110_target_network_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        p110_target_component_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
        c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
        p111_ts1_SET((int64_t) -4103656191883114175L, PH.base.pack) ;
        p111_tc1_SET((int64_t)633506756755885034L, PH.base.pack) ;
        c_CommunicationChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_seq_SET((uint32_t)3603856836L, PH.base.pack) ;
        p112_time_usec_SET((uint64_t)6937714871197763264L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
        p113_alt_SET((int32_t) -27467952, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)64427, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)20374, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t) -1888, PH.base.pack) ;
        p113_lon_SET((int32_t)1011628572, PH.base.pack) ;
        p113_lat_SET((int32_t) -2108285371, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t)14623, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)2034, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t)18960, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)262, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)7520355250268813190L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_temperature_SET((int16_t)(int16_t)11305, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)62854891L, PH.base.pack) ;
        p114_integrated_y_SET((float)8.771594E37F, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)3510100682L, PH.base.pack) ;
        p114_distance_SET((float) -1.4745204E38F, PH.base.pack) ;
        p114_integrated_zgyro_SET((float)1.0852622E38F, PH.base.pack) ;
        p114_integrated_ygyro_SET((float) -1.2679214E38F, PH.base.pack) ;
        p114_integrated_x_SET((float) -1.647722E38F, PH.base.pack) ;
        p114_integrated_xgyro_SET((float)1.7939759E38F, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)708509918534386171L, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_lon_SET((int32_t)495384339, PH.base.pack) ;
        p115_pitchspeed_SET((float)1.8767898E38F, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t) -6450, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)34839, PH.base.pack) ;
        p115_rollspeed_SET((float)2.429347E36F, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t) -26527, PH.base.pack) ;
        p115_alt_SET((int32_t)1553581837, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t)13713, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t) -4171, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t) -2850, PH.base.pack) ;
        p115_lat_SET((int32_t)10842282, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)2873181721335708150L, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)794, PH.base.pack) ;
        p115_yawspeed_SET((float)1.5710696E38F, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t) -32730, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {4.133478E37F, -2.346324E38F, -5.53602E37F, -2.524565E38F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
        p116_zmag_SET((int16_t)(int16_t)4571, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t)11269, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t) -16994, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)4069989961L, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t)6437, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t) -13226, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t)28160, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t) -18355, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t)8407, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t) -30997, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_start_SET((uint16_t)(uint16_t)29357, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p117_end_SET((uint16_t)(uint16_t)55817, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
        p118_time_utc_SET((uint32_t)2340433430L, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)43733, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)55689, PH.base.pack) ;
        p118_size_SET((uint32_t)348735129L, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)39751, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_ofs_SET((uint32_t)2466055741L, PH.base.pack) ;
        p119_count_SET((uint32_t)3635092174L, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        p119_id_SET((uint16_t)(uint16_t)53383, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
        p120_count_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        p120_ofs_SET((uint32_t)3343964101L, PH.base.pack) ;
        p120_id_SET((uint16_t)(uint16_t)30462, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)23, (uint8_t)199, (uint8_t)46, (uint8_t)183, (uint8_t)152, (uint8_t)163, (uint8_t)157, (uint8_t)52, (uint8_t)29, (uint8_t)243, (uint8_t)121, (uint8_t)116, (uint8_t)47, (uint8_t)89, (uint8_t)42, (uint8_t)11, (uint8_t)46, (uint8_t)41, (uint8_t)166, (uint8_t)235, (uint8_t)104, (uint8_t)133, (uint8_t)214, (uint8_t)182, (uint8_t)200, (uint8_t)152, (uint8_t)156, (uint8_t)201, (uint8_t)11, (uint8_t)135, (uint8_t)229, (uint8_t)148, (uint8_t)255, (uint8_t)184, (uint8_t)61, (uint8_t)147, (uint8_t)48, (uint8_t)251, (uint8_t)88, (uint8_t)73, (uint8_t)227, (uint8_t)139, (uint8_t)17, (uint8_t)39, (uint8_t)168, (uint8_t)202, (uint8_t)217, (uint8_t)126, (uint8_t)146, (uint8_t)77, (uint8_t)61, (uint8_t)65, (uint8_t)22, (uint8_t)227, (uint8_t)96, (uint8_t)46, (uint8_t)120, (uint8_t)169, (uint8_t)234, (uint8_t)228, (uint8_t)177, (uint8_t)49, (uint8_t)173, (uint8_t)73, (uint8_t)45, (uint8_t)190, (uint8_t)106, (uint8_t)8, (uint8_t)187, (uint8_t)139, (uint8_t)238, (uint8_t)130, (uint8_t)213, (uint8_t)171, (uint8_t)127, (uint8_t)166, (uint8_t)187, (uint8_t)11, (uint8_t)196, (uint8_t)204, (uint8_t)61, (uint8_t)205, (uint8_t)177, (uint8_t)252, (uint8_t)63, (uint8_t)192, (uint8_t)6, (uint8_t)189, (uint8_t)99, (uint8_t)174};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
        p121_target_component_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p121_target_system_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_component_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        p122_target_system_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_target_system_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)230, (uint8_t)134, (uint8_t)35, (uint8_t)246, (uint8_t)143, (uint8_t)197, (uint8_t)218, (uint8_t)27, (uint8_t)68, (uint8_t)243, (uint8_t)30, (uint8_t)64, (uint8_t)70, (uint8_t)249, (uint8_t)92, (uint8_t)68, (uint8_t)133, (uint8_t)221, (uint8_t)244, (uint8_t)0, (uint8_t)216, (uint8_t)33, (uint8_t)152, (uint8_t)226, (uint8_t)63, (uint8_t)203, (uint8_t)111, (uint8_t)214, (uint8_t)193, (uint8_t)10, (uint8_t)43, (uint8_t)33, (uint8_t)129, (uint8_t)69, (uint8_t)31, (uint8_t)86, (uint8_t)129, (uint8_t)252, (uint8_t)160, (uint8_t)2, (uint8_t)85, (uint8_t)59, (uint8_t)16, (uint8_t)53, (uint8_t)185, (uint8_t)153, (uint8_t)53, (uint8_t)58, (uint8_t)141, (uint8_t)166, (uint8_t)164, (uint8_t)52, (uint8_t)68, (uint8_t)167, (uint8_t)14, (uint8_t)238, (uint8_t)33, (uint8_t)103, (uint8_t)118, (uint8_t)201, (uint8_t)8, (uint8_t)226, (uint8_t)170, (uint8_t)139, (uint8_t)153, (uint8_t)223, (uint8_t)118, (uint8_t)178, (uint8_t)208, (uint8_t)232, (uint8_t)254, (uint8_t)169, (uint8_t)136, (uint8_t)164, (uint8_t)101, (uint8_t)89, (uint8_t)238, (uint8_t)163, (uint8_t)230, (uint8_t)61, (uint8_t)204, (uint8_t)247, (uint8_t)78, (uint8_t)222, (uint8_t)103, (uint8_t)171, (uint8_t)35, (uint8_t)245, (uint8_t)31, (uint8_t)200, (uint8_t)147, (uint8_t)89, (uint8_t)16, (uint8_t)135, (uint8_t)231, (uint8_t)205, (uint8_t)158, (uint8_t)93, (uint8_t)74, (uint8_t)121, (uint8_t)124, (uint8_t)117, (uint8_t)88, (uint8_t)27, (uint8_t)175, (uint8_t)38, (uint8_t)158, (uint8_t)172, (uint8_t)1, (uint8_t)129};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_len_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        p123_target_component_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
        p124_epv_SET((uint16_t)(uint16_t)64025, PH.base.pack) ;
        p124_lon_SET((int32_t) -1114929944, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)425004508L, PH.base.pack) ;
        p124_lat_SET((int32_t)1558407623, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)2595, PH.base.pack) ;
        p124_alt_SET((int32_t) -587427041, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)38795, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)32529, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)7638772046898576182L, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
        p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID, PH.base.pack) ;
        p125_Vcc_SET((uint16_t)(uint16_t)60433, PH.base.pack) ;
        p125_Vservo_SET((uint16_t)(uint16_t)53587, PH.base.pack) ;
        c_CommunicationChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_baudrate_SET((uint32_t)2602259593L, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)122, (uint8_t)200, (uint8_t)225, (uint8_t)174, (uint8_t)175, (uint8_t)54, (uint8_t)168, (uint8_t)160, (uint8_t)103, (uint8_t)195, (uint8_t)139, (uint8_t)60, (uint8_t)133, (uint8_t)171, (uint8_t)247, (uint8_t)75, (uint8_t)138, (uint8_t)153, (uint8_t)223, (uint8_t)18, (uint8_t)47, (uint8_t)131, (uint8_t)175, (uint8_t)24, (uint8_t)120, (uint8_t)126, (uint8_t)154, (uint8_t)222, (uint8_t)86, (uint8_t)17, (uint8_t)52, (uint8_t)25, (uint8_t)6, (uint8_t)121, (uint8_t)219, (uint8_t)124, (uint8_t)198, (uint8_t)8, (uint8_t)34, (uint8_t)139, (uint8_t)45, (uint8_t)13, (uint8_t)184, (uint8_t)237, (uint8_t)158, (uint8_t)105, (uint8_t)18, (uint8_t)87, (uint8_t)142, (uint8_t)91, (uint8_t)129, (uint8_t)176, (uint8_t)57, (uint8_t)8, (uint8_t)22, (uint8_t)113, (uint8_t)90, (uint8_t)203, (uint8_t)152, (uint8_t)33, (uint8_t)253, (uint8_t)128, (uint8_t)218, (uint8_t)189, (uint8_t)127, (uint8_t)113, (uint8_t)112, (uint8_t)239, (uint8_t)73, (uint8_t)100};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING, PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)32584, PH.base.pack) ;
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
        p127_time_last_baseline_ms_SET((uint32_t)4290087520L, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t) -1327894295, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t) -1158400589, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t)1016694760, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)56979, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t)1059834170, PH.base.pack) ;
        p127_tow_SET((uint32_t)531733435L, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)250166633L, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
        p128_wn_SET((uint16_t)(uint16_t)32899, PH.base.pack) ;
        p128_tow_SET((uint32_t)388488657L, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t) -1214304203, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t)442975909, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t)689320771, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)3528912347L, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)2585521831L, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t)891757458, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
        p129_zmag_SET((int16_t)(int16_t) -21251, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t)12671, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t) -27135, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t) -29312, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t)22602, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t) -14297, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t) -5876, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t)629, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)2391013324L, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t)22311, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_height_SET((uint16_t)(uint16_t)34307, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)35872, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)4726, PH.base.pack) ;
        p130_size_SET((uint32_t)2768345946L, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)188, (uint8_t)39, (uint8_t)173, (uint8_t)90, (uint8_t)44, (uint8_t)218, (uint8_t)225, (uint8_t)47, (uint8_t)69, (uint8_t)114, (uint8_t)75, (uint8_t)196, (uint8_t)151, (uint8_t)103, (uint8_t)180, (uint8_t)114, (uint8_t)39, (uint8_t)37, (uint8_t)46, (uint8_t)94, (uint8_t)226, (uint8_t)249, (uint8_t)159, (uint8_t)6, (uint8_t)245, (uint8_t)81, (uint8_t)114, (uint8_t)44, (uint8_t)206, (uint8_t)69, (uint8_t)137, (uint8_t)112, (uint8_t)164, (uint8_t)22, (uint8_t)119, (uint8_t)11, (uint8_t)79, (uint8_t)170, (uint8_t)79, (uint8_t)91, (uint8_t)129, (uint8_t)38, (uint8_t)6, (uint8_t)68, (uint8_t)29, (uint8_t)155, (uint8_t)128, (uint8_t)94, (uint8_t)196, (uint8_t)47, (uint8_t)54, (uint8_t)104, (uint8_t)40, (uint8_t)218, (uint8_t)110, (uint8_t)172, (uint8_t)250, (uint8_t)168, (uint8_t)8, (uint8_t)159, (uint8_t)50, (uint8_t)36, (uint8_t)205, (uint8_t)148, (uint8_t)238, (uint8_t)169, (uint8_t)13, (uint8_t)237, (uint8_t)155, (uint8_t)110, (uint8_t)6, (uint8_t)23, (uint8_t)221, (uint8_t)43, (uint8_t)137, (uint8_t)162, (uint8_t)189, (uint8_t)59, (uint8_t)199, (uint8_t)10, (uint8_t)1, (uint8_t)6, (uint8_t)60, (uint8_t)44, (uint8_t)9, (uint8_t)23, (uint8_t)91, (uint8_t)18, (uint8_t)222, (uint8_t)106, (uint8_t)61, (uint8_t)110, (uint8_t)21, (uint8_t)107, (uint8_t)197, (uint8_t)31, (uint8_t)91, (uint8_t)12, (uint8_t)116, (uint8_t)251, (uint8_t)87, (uint8_t)162, (uint8_t)133, (uint8_t)227, (uint8_t)176, (uint8_t)186, (uint8_t)21, (uint8_t)21, (uint8_t)51, (uint8_t)236, (uint8_t)75, (uint8_t)89, (uint8_t)181, (uint8_t)196, (uint8_t)9, (uint8_t)250, (uint8_t)84, (uint8_t)73, (uint8_t)21, (uint8_t)184, (uint8_t)28, (uint8_t)250, (uint8_t)233, (uint8_t)201, (uint8_t)206, (uint8_t)158, (uint8_t)62, (uint8_t)8, (uint8_t)81, (uint8_t)217, (uint8_t)41, (uint8_t)78, (uint8_t)70, (uint8_t)251, (uint8_t)214, (uint8_t)96, (uint8_t)70, (uint8_t)192, (uint8_t)166, (uint8_t)126, (uint8_t)136, (uint8_t)198, (uint8_t)149, (uint8_t)111, (uint8_t)212, (uint8_t)230, (uint8_t)190, (uint8_t)197, (uint8_t)135, (uint8_t)233, (uint8_t)25, (uint8_t)234, (uint8_t)46, (uint8_t)193, (uint8_t)25, (uint8_t)42, (uint8_t)3, (uint8_t)29, (uint8_t)8, (uint8_t)132, (uint8_t)73, (uint8_t)186, (uint8_t)200, (uint8_t)150, (uint8_t)143, (uint8_t)155, (uint8_t)77, (uint8_t)106, (uint8_t)82, (uint8_t)156, (uint8_t)54, (uint8_t)44, (uint8_t)76, (uint8_t)3, (uint8_t)96, (uint8_t)224, (uint8_t)128, (uint8_t)28, (uint8_t)144, (uint8_t)101, (uint8_t)59, (uint8_t)207, (uint8_t)225, (uint8_t)100, (uint8_t)43, (uint8_t)140, (uint8_t)247, (uint8_t)237, (uint8_t)22, (uint8_t)205, (uint8_t)164, (uint8_t)208, (uint8_t)203, (uint8_t)239, (uint8_t)237, (uint8_t)142, (uint8_t)58, (uint8_t)112, (uint8_t)89, (uint8_t)4, (uint8_t)175, (uint8_t)63, (uint8_t)140, (uint8_t)18, (uint8_t)39, (uint8_t)249, (uint8_t)82, (uint8_t)45, (uint8_t)118, (uint8_t)88, (uint8_t)147, (uint8_t)224, (uint8_t)174, (uint8_t)88, (uint8_t)31, (uint8_t)147, (uint8_t)172, (uint8_t)161, (uint8_t)1, (uint8_t)202, (uint8_t)168, (uint8_t)139, (uint8_t)86, (uint8_t)53, (uint8_t)152, (uint8_t)54, (uint8_t)48, (uint8_t)174, (uint8_t)27, (uint8_t)147, (uint8_t)94, (uint8_t)107, (uint8_t)8, (uint8_t)122, (uint8_t)48, (uint8_t)146, (uint8_t)64, (uint8_t)252, (uint8_t)44, (uint8_t)37, (uint8_t)33, (uint8_t)71, (uint8_t)72, (uint8_t)53, (uint8_t)150, (uint8_t)50, (uint8_t)197, (uint8_t)164, (uint8_t)134, (uint8_t)254, (uint8_t)98, (uint8_t)91, (uint8_t)122};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        p131_seqnr_SET((uint16_t)(uint16_t)61541, PH.base.pack) ;
        c_CommunicationChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_current_distance_SET((uint16_t)(uint16_t)60449, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)45575, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)3467384688L, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)44568, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_YAW_270, PH.base.pack) ;
        c_CommunicationChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_mask_SET((uint64_t)5772458444715259195L, PH.base.pack) ;
        p133_lat_SET((int32_t)1253362199, PH.base.pack) ;
        p133_lon_SET((int32_t)218541196, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)13701, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
        p134_lat_SET((int32_t) -1817399396, PH.base.pack) ;
        p134_gridbit_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        p134_lon_SET((int32_t) -959128891, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t) -346, (int16_t) -10006, (int16_t) -27183, (int16_t) -15897, (int16_t)20839, (int16_t)15430, (int16_t)12285, (int16_t) -2986, (int16_t)2293, (int16_t)18172, (int16_t)29930, (int16_t) -11235, (int16_t) -29376, (int16_t) -1638, (int16_t) -14504, (int16_t) -31795};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        p134_grid_spacing_SET((uint16_t)(uint16_t)46201, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lon_SET((int32_t) -335060553, PH.base.pack) ;
        p135_lat_SET((int32_t) -1159619802, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_current_height_SET((float) -2.1857905E38F, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)17331, PH.base.pack) ;
        p136_lat_SET((int32_t)1843563009, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)28520, PH.base.pack) ;
        p136_lon_SET((int32_t) -418547448, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)15972, PH.base.pack) ;
        p136_terrain_height_SET((float)1.2955662E38F, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_press_diff_SET((float)1.2614977E38F, PH.base.pack) ;
        p137_press_abs_SET((float) -7.9668144E37F, PH.base.pack) ;
        p137_time_boot_ms_SET((uint32_t)3599920670L, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t) -30594, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
        p138_x_SET((float)3.3562202E38F, PH.base.pack) ;
        {
            float q[] =  {1.0379074E38F, 5.0465928E36F, 1.6644815E38F, 9.733854E37F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_y_SET((float) -1.3745644E38F, PH.base.pack) ;
        p138_z_SET((float) -1.1622727E38F, PH.base.pack) ;
        p138_time_usec_SET((uint64_t)7599245665255036759L, PH.base.pack) ;
        c_CommunicationChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_time_usec_SET((uint64_t)5905742376266188652L, PH.base.pack) ;
        {
            float controls[] =  {-2.73652E38F, -2.3299485E38F, 1.9436118E38F, -8.905948E37F, -7.9589615E37F, -2.260777E38F, 2.0189102E38F, -1.3604048E38F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_group_mlx_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p139_target_component_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p139_target_system_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        {
            float controls[] =  {-1.2877317E38F, 3.328549E38F, 1.3232384E38F, 3.2043696E38F, 3.172789E38F, -3.3842703E38F, -5.6490395E37F, -2.263283E37F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p140_group_mlx_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        p140_time_usec_SET((uint64_t)204805683154758408L, PH.base.pack) ;
        c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ALTITUDE_141(), &PH);
        p141_altitude_terrain_SET((float) -3.7763717E36F, PH.base.pack) ;
        p141_altitude_monotonic_SET((float)2.6429625E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)3473044262436307002L, PH.base.pack) ;
        p141_altitude_local_SET((float)1.678073E38F, PH.base.pack) ;
        p141_altitude_relative_SET((float) -8.540514E37F, PH.base.pack) ;
        p141_altitude_amsl_SET((float) -2.3213621E38F, PH.base.pack) ;
        p141_bottom_clearance_SET((float) -2.7229545E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_RESOURCE_REQUEST_142(), &PH);
        {
            uint8_t uri[] =  {(uint8_t)152, (uint8_t)149, (uint8_t)159, (uint8_t)53, (uint8_t)230, (uint8_t)77, (uint8_t)50, (uint8_t)9, (uint8_t)73, (uint8_t)143, (uint8_t)206, (uint8_t)142, (uint8_t)241, (uint8_t)129, (uint8_t)218, (uint8_t)69, (uint8_t)153, (uint8_t)63, (uint8_t)81, (uint8_t)124, (uint8_t)48, (uint8_t)147, (uint8_t)247, (uint8_t)34, (uint8_t)138, (uint8_t)61, (uint8_t)110, (uint8_t)16, (uint8_t)169, (uint8_t)156, (uint8_t)245, (uint8_t)208, (uint8_t)189, (uint8_t)51, (uint8_t)183, (uint8_t)212, (uint8_t)90, (uint8_t)9, (uint8_t)6, (uint8_t)16, (uint8_t)165, (uint8_t)71, (uint8_t)146, (uint8_t)161, (uint8_t)167, (uint8_t)89, (uint8_t)0, (uint8_t)167, (uint8_t)84, (uint8_t)191, (uint8_t)184, (uint8_t)201, (uint8_t)92, (uint8_t)126, (uint8_t)48, (uint8_t)95, (uint8_t)237, (uint8_t)157, (uint8_t)89, (uint8_t)11, (uint8_t)18, (uint8_t)5, (uint8_t)103, (uint8_t)243, (uint8_t)166, (uint8_t)15, (uint8_t)135, (uint8_t)59, (uint8_t)225, (uint8_t)9, (uint8_t)210, (uint8_t)225, (uint8_t)111, (uint8_t)141, (uint8_t)36, (uint8_t)198, (uint8_t)100, (uint8_t)24, (uint8_t)66, (uint8_t)36, (uint8_t)103, (uint8_t)153, (uint8_t)104, (uint8_t)72, (uint8_t)58, (uint8_t)73, (uint8_t)43, (uint8_t)12, (uint8_t)193, (uint8_t)173, (uint8_t)210, (uint8_t)77, (uint8_t)22, (uint8_t)194, (uint8_t)132, (uint8_t)251, (uint8_t)134, (uint8_t)199, (uint8_t)10, (uint8_t)111, (uint8_t)11, (uint8_t)218, (uint8_t)61, (uint8_t)124, (uint8_t)92, (uint8_t)27, (uint8_t)1, (uint8_t)5, (uint8_t)200, (uint8_t)240, (uint8_t)141, (uint8_t)146, (uint8_t)97, (uint8_t)152, (uint8_t)207, (uint8_t)6, (uint8_t)210, (uint8_t)189, (uint8_t)194, (uint8_t)38};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        p142_request_id_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p142_transfer_type_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        p142_uri_type_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        {
            uint8_t storage[] =  {(uint8_t)51, (uint8_t)109, (uint8_t)159, (uint8_t)208, (uint8_t)75, (uint8_t)156, (uint8_t)98, (uint8_t)58, (uint8_t)50, (uint8_t)15, (uint8_t)135, (uint8_t)222, (uint8_t)89, (uint8_t)232, (uint8_t)141, (uint8_t)26, (uint8_t)70, (uint8_t)133, (uint8_t)193, (uint8_t)164, (uint8_t)3, (uint8_t)239, (uint8_t)69, (uint8_t)134, (uint8_t)60, (uint8_t)176, (uint8_t)208, (uint8_t)86, (uint8_t)186, (uint8_t)135, (uint8_t)173, (uint8_t)115, (uint8_t)103, (uint8_t)188, (uint8_t)196, (uint8_t)41, (uint8_t)136, (uint8_t)36, (uint8_t)214, (uint8_t)37, (uint8_t)255, (uint8_t)53, (uint8_t)130, (uint8_t)185, (uint8_t)184, (uint8_t)99, (uint8_t)145, (uint8_t)132, (uint8_t)197, (uint8_t)116, (uint8_t)217, (uint8_t)181, (uint8_t)75, (uint8_t)216, (uint8_t)145, (uint8_t)69, (uint8_t)147, (uint8_t)178, (uint8_t)37, (uint8_t)193, (uint8_t)220, (uint8_t)40, (uint8_t)246, (uint8_t)136, (uint8_t)127, (uint8_t)71, (uint8_t)157, (uint8_t)77, (uint8_t)240, (uint8_t)248, (uint8_t)202, (uint8_t)227, (uint8_t)109, (uint8_t)155, (uint8_t)72, (uint8_t)187, (uint8_t)225, (uint8_t)132, (uint8_t)121, (uint8_t)102, (uint8_t)160, (uint8_t)206, (uint8_t)17, (uint8_t)176, (uint8_t)4, (uint8_t)234, (uint8_t)213, (uint8_t)179, (uint8_t)204, (uint8_t)36, (uint8_t)184, (uint8_t)242, (uint8_t)36, (uint8_t)153, (uint8_t)244, (uint8_t)89, (uint8_t)113, (uint8_t)244, (uint8_t)213, (uint8_t)14, (uint8_t)134, (uint8_t)142, (uint8_t)53, (uint8_t)163, (uint8_t)157, (uint8_t)254, (uint8_t)20, (uint8_t)12, (uint8_t)242, (uint8_t)139, (uint8_t)207, (uint8_t)201, (uint8_t)57, (uint8_t)209, (uint8_t)132, (uint8_t)84, (uint8_t)139, (uint8_t)51, (uint8_t)6, (uint8_t)211};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SCALED_PRESSURE3_143(), &PH);
        p143_press_abs_SET((float)1.682078E38F, PH.base.pack) ;
        p143_temperature_SET((int16_t)(int16_t) -19449, PH.base.pack) ;
        p143_press_diff_SET((float)1.0745996E38F, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)1683663861L, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FOLLOW_TARGET_144(), &PH);
        p144_lon_SET((int32_t) -1757854381, PH.base.pack) ;
        p144_lat_SET((int32_t) -1886024704, PH.base.pack) ;
        p144_alt_SET((float) -3.1291182E38F, PH.base.pack) ;
        p144_timestamp_SET((uint64_t)2543914614391503421L, PH.base.pack) ;
        {
            float rates[] =  {-2.6654153E38F, -1.5465242E38F, 1.8622976E38F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        {
            float position_cov[] =  {1.7509305E38F, -9.476816E37F, 2.7711655E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        {
            float acc[] =  {3.2508877E38F, 1.0253613E38F, 2.6060905E38F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        {
            float attitude_q[] =  {2.2788255E38F, 1.8943898E38F, 1.1401667E38F, -2.7231514E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        p144_custom_state_SET((uint64_t)8321112846915514256L, PH.base.pack) ;
        {
            float vel[] =  {-1.7881498E38F, -2.938117E38F, -1.5812252E38F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        p144_est_capabilities_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        c_CommunicationChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_y_vel_SET((float)2.0790853E38F, PH.base.pack) ;
        p146_z_pos_SET((float) -1.1242104E37F, PH.base.pack) ;
        p146_airspeed_SET((float)2.1839727E38F, PH.base.pack) ;
        p146_x_vel_SET((float)1.8287872E38F, PH.base.pack) ;
        {
            float q[] =  {1.0022256E37F, -2.4938442E38F, -1.2414346E38F, -1.7563465E38F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_time_usec_SET((uint64_t)4965253514208826191L, PH.base.pack) ;
        {
            float pos_variance[] =  {2.180479E38F, -2.6406154E38F, 6.002052E37F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_x_acc_SET((float)2.486412E38F, PH.base.pack) ;
        p146_pitch_rate_SET((float) -7.021863E37F, PH.base.pack) ;
        p146_z_acc_SET((float)9.562689E37F, PH.base.pack) ;
        p146_y_acc_SET((float)4.373306E37F, PH.base.pack) ;
        p146_y_pos_SET((float) -1.0669033E37F, PH.base.pack) ;
        p146_x_pos_SET((float) -1.8876307E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {3.0289456E38F, 2.601582E38F, -1.9547789E38F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_yaw_rate_SET((float) -3.1712809E38F, PH.base.pack) ;
        p146_z_vel_SET((float)2.2337535E38F, PH.base.pack) ;
        p146_roll_rate_SET((float) -2.7382644E38F, PH.base.pack) ;
        c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BATTERY_STATUS_147(), &PH);
        {
            uint16_t voltages[] =  {(uint16_t)27784, (uint16_t)31419, (uint16_t)50578, (uint16_t)14451, (uint16_t)44876, (uint16_t)29297, (uint16_t)50929, (uint16_t)31983, (uint16_t)54754, (uint16_t)12909};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_current_consumed_SET((int32_t)92688, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t) -29343, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t)23683, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIPO, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t)1885054876, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t) -85, PH.base.pack) ;
        c_CommunicationChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AUTOPILOT_VERSION_148(), &PH);
        p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT, PH.base.pack) ;
        {
            uint8_t flight_custom_version[] =  {(uint8_t)21, (uint8_t)65, (uint8_t)80, (uint8_t)112, (uint8_t)182, (uint8_t)16, (uint8_t)85, (uint8_t)117};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t uid2[] =  {(uint8_t)131, (uint8_t)82, (uint8_t)139, (uint8_t)26, (uint8_t)120, (uint8_t)46, (uint8_t)4, (uint8_t)117, (uint8_t)117, (uint8_t)95, (uint8_t)213, (uint8_t)128, (uint8_t)14, (uint8_t)90, (uint8_t)39, (uint8_t)67, (uint8_t)93, (uint8_t)241};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_board_version_SET((uint32_t)3629162258L, PH.base.pack) ;
        p148_uid_SET((uint64_t)4382466503664530744L, PH.base.pack) ;
        p148_middleware_sw_version_SET((uint32_t)3678170855L, PH.base.pack) ;
        p148_vendor_id_SET((uint16_t)(uint16_t)26214, PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)249, (uint8_t)138, (uint8_t)239, (uint8_t)173, (uint8_t)169, (uint8_t)213, (uint8_t)108, (uint8_t)184};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t os_custom_version[] =  {(uint8_t)178, (uint8_t)125, (uint8_t)58, (uint8_t)233, (uint8_t)232, (uint8_t)173, (uint8_t)57, (uint8_t)169};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        p148_product_id_SET((uint16_t)(uint16_t)1230, PH.base.pack) ;
        p148_flight_sw_version_SET((uint32_t)1880126634L, PH.base.pack) ;
        p148_os_sw_version_SET((uint32_t)575586744L, PH.base.pack) ;
        c_CommunicationChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LANDING_TARGET_149(), &PH);
        p149_size_y_SET((float) -3.8066353E37F, PH.base.pack) ;
        p149_angle_x_SET((float) -2.6137475E37F, PH.base.pack) ;
        p149_z_SET((float) -9.053326E36F, &PH) ;
        p149_x_SET((float) -2.6925596E38F, &PH) ;
        p149_y_SET((float)5.2875365E37F, &PH) ;
        p149_time_usec_SET((uint64_t)5399986078439850086L, PH.base.pack) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_RADIO_BEACON, PH.base.pack) ;
        {
            float q[] =  {-2.9977225E38F, 2.2878037E38F, -1.3870347E38F, 1.2044326E38F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_size_x_SET((float) -1.8891844E38F, PH.base.pack) ;
        p149_position_valid_SET((uint8_t)(uint8_t)78, &PH) ;
        p149_target_num_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p149_distance_SET((float) -3.2185359E38F, PH.base.pack) ;
        p149_angle_y_SET((float)3.429606E37F, PH.base.pack) ;
        c_CommunicationChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AQ_TELEMETRY_F_150(), &PH);
        p150_value10_SET((float) -1.475662E38F, PH.base.pack) ;
        p150_value5_SET((float) -5.6799103E37F, PH.base.pack) ;
        p150_value14_SET((float) -9.13397E37F, PH.base.pack) ;
        p150_value13_SET((float) -7.255428E37F, PH.base.pack) ;
        p150_value8_SET((float) -9.555088E37F, PH.base.pack) ;
        p150_value17_SET((float)2.1936428E38F, PH.base.pack) ;
        p150_value4_SET((float)1.4889478E38F, PH.base.pack) ;
        p150_value15_SET((float) -2.7591467E36F, PH.base.pack) ;
        p150_value3_SET((float)3.2535117E38F, PH.base.pack) ;
        p150_value20_SET((float)5.839068E37F, PH.base.pack) ;
        p150_value11_SET((float)1.8255578E38F, PH.base.pack) ;
        p150_value9_SET((float) -1.1004821E38F, PH.base.pack) ;
        p150_value16_SET((float)1.0499287E38F, PH.base.pack) ;
        p150_value19_SET((float) -8.226431E37F, PH.base.pack) ;
        p150_value12_SET((float)1.838679E38F, PH.base.pack) ;
        p150_value2_SET((float)3.2828471E38F, PH.base.pack) ;
        p150_value1_SET((float) -5.392704E37F, PH.base.pack) ;
        p150_value18_SET((float) -3.0462863E37F, PH.base.pack) ;
        p150_Index_SET((uint16_t)(uint16_t)57859, PH.base.pack) ;
        p150_value7_SET((float) -4.843846E37F, PH.base.pack) ;
        p150_value6_SET((float) -1.5494409E38F, PH.base.pack) ;
        c_CommunicationChannel_on_AQ_TELEMETRY_F_150(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AQ_ESC_TELEMETRY_152(), &PH);
        {
            uint32_t data1[] =  {1049983750L, 578858551L, 4198537330L, 3975281948L};
            p152_data1_SET(&data1, 0, PH.base.pack) ;
        }
        {
            uint8_t escid[] =  {(uint8_t)177, (uint8_t)148, (uint8_t)51, (uint8_t)29};
            p152_escid_SET(&escid, 0, PH.base.pack) ;
        }
        p152_num_motors_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        p152_time_boot_ms_SET((uint32_t)45143426L, PH.base.pack) ;
        p152_num_in_seq_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        {
            uint32_t data0[] =  {2770028949L, 228202652L, 596172276L, 2267382589L};
            p152_data0_SET(&data0, 0, PH.base.pack) ;
        }
        {
            uint16_t status_age[] =  {(uint16_t)49859, (uint16_t)59786, (uint16_t)7124, (uint16_t)59668};
            p152_status_age_SET(&status_age, 0, PH.base.pack) ;
        }
        p152_seq_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
        {
            uint8_t data_version[] =  {(uint8_t)66, (uint8_t)67, (uint8_t)234, (uint8_t)95};
            p152_data_version_SET(&data_version, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_AQ_ESC_TELEMETRY_152(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE, PH.base.pack) ;
        p230_tas_ratio_SET((float) -3.808844E37F, PH.base.pack) ;
        p230_mag_ratio_SET((float)2.4642596E38F, PH.base.pack) ;
        p230_hagl_ratio_SET((float) -1.2000671E38F, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float)2.2941434E38F, PH.base.pack) ;
        p230_vel_ratio_SET((float)2.9970215E38F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float) -2.7398168E38F, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float)3.5278617E37F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)3923058367190746169L, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float) -7.9149097E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIND_COV_231(), &PH);
        p231_horiz_accuracy_SET((float)2.181051E38F, PH.base.pack) ;
        p231_wind_z_SET((float)1.8936844E38F, PH.base.pack) ;
        p231_vert_accuracy_SET((float)2.4563278E38F, PH.base.pack) ;
        p231_var_horiz_SET((float)1.4352971E38F, PH.base.pack) ;
        p231_var_vert_SET((float) -7.587624E37F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)1086841837524163310L, PH.base.pack) ;
        p231_wind_x_SET((float)2.5124622E38F, PH.base.pack) ;
        p231_wind_y_SET((float) -8.932963E37F, PH.base.pack) ;
        p231_wind_alt_SET((float) -1.8518599E38F, PH.base.pack) ;
        c_CommunicationChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_INPUT_232(), &PH);
        p232_alt_SET((float) -1.971394E38F, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)2027246167L, PH.base.pack) ;
        p232_lat_SET((int32_t)1273375205, PH.base.pack) ;
        p232_vdop_SET((float)2.9763834E38F, PH.base.pack) ;
        p232_ve_SET((float)5.5010093E37F, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)1149849318146977234L, PH.base.pack) ;
        p232_speed_accuracy_SET((float)9.827738E37F, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)40415, PH.base.pack) ;
        p232_vert_accuracy_SET((float)1.8148898E36F, PH.base.pack) ;
        p232_vd_SET((float) -1.2679857E36F, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY, PH.base.pack) ;
        p232_hdop_SET((float) -1.6241342E38F, PH.base.pack) ;
        p232_vn_SET((float)7.926133E37F, PH.base.pack) ;
        p232_lon_SET((int32_t) -503845290, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p232_horiz_accuracy_SET((float)3.3999766E38F, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_RTCM_DATA_233(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)233, (uint8_t)67, (uint8_t)148, (uint8_t)192, (uint8_t)252, (uint8_t)230, (uint8_t)161, (uint8_t)250, (uint8_t)194, (uint8_t)232, (uint8_t)113, (uint8_t)192, (uint8_t)124, (uint8_t)214, (uint8_t)23, (uint8_t)235, (uint8_t)187, (uint8_t)247, (uint8_t)128, (uint8_t)216, (uint8_t)49, (uint8_t)89, (uint8_t)26, (uint8_t)212, (uint8_t)22, (uint8_t)68, (uint8_t)197, (uint8_t)115, (uint8_t)188, (uint8_t)34, (uint8_t)66, (uint8_t)229, (uint8_t)62, (uint8_t)119, (uint8_t)26, (uint8_t)88, (uint8_t)36, (uint8_t)23, (uint8_t)36, (uint8_t)20, (uint8_t)102, (uint8_t)17, (uint8_t)131, (uint8_t)223, (uint8_t)7, (uint8_t)129, (uint8_t)140, (uint8_t)153, (uint8_t)95, (uint8_t)134, (uint8_t)193, (uint8_t)23, (uint8_t)71, (uint8_t)34, (uint8_t)184, (uint8_t)185, (uint8_t)237, (uint8_t)157, (uint8_t)84, (uint8_t)67, (uint8_t)226, (uint8_t)249, (uint8_t)236, (uint8_t)208, (uint8_t)2, (uint8_t)203, (uint8_t)71, (uint8_t)12, (uint8_t)51, (uint8_t)153, (uint8_t)75, (uint8_t)172, (uint8_t)111, (uint8_t)64, (uint8_t)90, (uint8_t)97, (uint8_t)90, (uint8_t)133, (uint8_t)59, (uint8_t)31, (uint8_t)78, (uint8_t)131, (uint8_t)246, (uint8_t)113, (uint8_t)133, (uint8_t)33, (uint8_t)1, (uint8_t)43, (uint8_t)235, (uint8_t)203, (uint8_t)2, (uint8_t)93, (uint8_t)219, (uint8_t)210, (uint8_t)26, (uint8_t)246, (uint8_t)6, (uint8_t)142, (uint8_t)235, (uint8_t)33, (uint8_t)1, (uint8_t)140, (uint8_t)152, (uint8_t)249, (uint8_t)171, (uint8_t)65, (uint8_t)125, (uint8_t)185, (uint8_t)182, (uint8_t)145, (uint8_t)132, (uint8_t)78, (uint8_t)129, (uint8_t)115, (uint8_t)47, (uint8_t)29, (uint8_t)2, (uint8_t)91, (uint8_t)91, (uint8_t)155, (uint8_t)179, (uint8_t)23, (uint8_t)108, (uint8_t)104, (uint8_t)231, (uint8_t)43, (uint8_t)2, (uint8_t)42, (uint8_t)10, (uint8_t)21, (uint8_t)82, (uint8_t)192, (uint8_t)19, (uint8_t)84, (uint8_t)11, (uint8_t)27, (uint8_t)135, (uint8_t)231, (uint8_t)168, (uint8_t)169, (uint8_t)196, (uint8_t)28, (uint8_t)33, (uint8_t)218, (uint8_t)213, (uint8_t)252, (uint8_t)239, (uint8_t)153, (uint8_t)182, (uint8_t)160, (uint8_t)39, (uint8_t)118, (uint8_t)206, (uint8_t)38, (uint8_t)3, (uint8_t)26, (uint8_t)60, (uint8_t)207, (uint8_t)94, (uint8_t)146, (uint8_t)79, (uint8_t)154, (uint8_t)75, (uint8_t)182, (uint8_t)192, (uint8_t)221, (uint8_t)0, (uint8_t)44, (uint8_t)212, (uint8_t)63, (uint8_t)201, (uint8_t)138, (uint8_t)47, (uint8_t)121, (uint8_t)148, (uint8_t)99, (uint8_t)138, (uint8_t)95, (uint8_t)92, (uint8_t)36};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        p233_len_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        p233_flags_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HIGH_LATENCY_234(), &PH);
        p234_gps_nsat_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)3196392122L, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t) -20, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t) -7446, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)27159, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t)16053, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t) -1081, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING, PH.base.pack) ;
        p234_latitude_SET((int32_t)1388463869, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t)2765, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t) -41, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t) -126, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP, PH.base.pack) ;
        p234_longitude_SET((int32_t)1973151624, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)15150, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t) -3865, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t) -35, PH.base.pack) ;
        c_CommunicationChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIBRATION_241(), &PH);
        p241_vibration_y_SET((float)1.1444053E38F, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)2213158821L, PH.base.pack) ;
        p241_vibration_x_SET((float)1.6319823E38F, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)3212046500L, PH.base.pack) ;
        p241_vibration_z_SET((float)2.9159333E38F, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)3233851370422934086L, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)3443350254L, PH.base.pack) ;
        c_CommunicationChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HOME_POSITION_242(), &PH);
        p242_z_SET((float)2.667485E38F, PH.base.pack) ;
        p242_latitude_SET((int32_t) -1237058113, PH.base.pack) ;
        p242_longitude_SET((int32_t) -636781737, PH.base.pack) ;
        p242_approach_x_SET((float) -2.7708364E38F, PH.base.pack) ;
        p242_altitude_SET((int32_t)1593474894, PH.base.pack) ;
        {
            float q[] =  {3.3340166E38F, -1.8847281E38F, 1.3844984E38F, -1.7381318E37F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_approach_y_SET((float) -2.266127E38F, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)6635489893600274986L, &PH) ;
        p242_x_SET((float) -3.0082476E38F, PH.base.pack) ;
        p242_approach_z_SET((float) -7.754246E37F, PH.base.pack) ;
        p242_y_SET((float) -3.105126E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_HOME_POSITION_243(), &PH);
        p243_longitude_SET((int32_t)659151816, PH.base.pack) ;
        p243_altitude_SET((int32_t) -1490177590, PH.base.pack) ;
        {
            float q[] =  {6.6474695E37F, 5.892577E37F, -9.488282E37F, -7.613717E37F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_latitude_SET((int32_t) -1456077207, PH.base.pack) ;
        p243_approach_y_SET((float) -2.3611368E38F, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)2649724085903844373L, &PH) ;
        p243_approach_z_SET((float)7.815999E37F, PH.base.pack) ;
        p243_x_SET((float) -2.0874652E38F, PH.base.pack) ;
        p243_z_SET((float) -1.763316E38F, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p243_y_SET((float) -2.2561718E38F, PH.base.pack) ;
        p243_approach_x_SET((float) -1.7241693E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_message_id_SET((uint16_t)(uint16_t)17622, PH.base.pack) ;
        p244_interval_us_SET((int32_t) -171054102, PH.base.pack) ;
        c_CommunicationChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED, PH.base.pack) ;
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, PH.base.pack) ;
        c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ADSB_VEHICLE_246(), &PH);
        p246_squawk_SET((uint16_t)(uint16_t)27563, PH.base.pack) ;
        p246_altitude_SET((int32_t) -1182839700, PH.base.pack) ;
        p246_lon_SET((int32_t)2004600735, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)2961331819L, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HEAVY, PH.base.pack) ;
        p246_lat_SET((int32_t)300559381, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t) -25334, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
        {
            char16_t* callsign = u"kkbuvyz";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_heading_SET((uint16_t)(uint16_t)2672, PH.base.pack) ;
        p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)4066, PH.base.pack) ;
        c_CommunicationChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_COLLISION_247(), &PH);
        p247_altitude_minimum_delta_SET((float)3.2134809E38F, PH.base.pack) ;
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float) -1.9360588E38F, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_HORIZONTALLY, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, PH.base.pack) ;
        p247_id_SET((uint32_t)347296417L, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float)5.747431E37F, PH.base.pack) ;
        c_CommunicationChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_V2_EXTENSION_248(), &PH);
        p248_target_component_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p248_target_system_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p248_target_network_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
        p248_message_type_SET((uint16_t)(uint16_t)17028, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)57, (uint8_t)213, (uint8_t)249, (uint8_t)214, (uint8_t)120, (uint8_t)244, (uint8_t)141, (uint8_t)122, (uint8_t)186, (uint8_t)74, (uint8_t)123, (uint8_t)179, (uint8_t)156, (uint8_t)138, (uint8_t)228, (uint8_t)233, (uint8_t)249, (uint8_t)241, (uint8_t)146, (uint8_t)235, (uint8_t)47, (uint8_t)252, (uint8_t)88, (uint8_t)216, (uint8_t)135, (uint8_t)178, (uint8_t)184, (uint8_t)184, (uint8_t)8, (uint8_t)88, (uint8_t)146, (uint8_t)99, (uint8_t)0, (uint8_t)19, (uint8_t)218, (uint8_t)102, (uint8_t)38, (uint8_t)197, (uint8_t)166, (uint8_t)224, (uint8_t)128, (uint8_t)108, (uint8_t)49, (uint8_t)115, (uint8_t)24, (uint8_t)177, (uint8_t)198, (uint8_t)74, (uint8_t)42, (uint8_t)154, (uint8_t)144, (uint8_t)255, (uint8_t)89, (uint8_t)15, (uint8_t)129, (uint8_t)64, (uint8_t)220, (uint8_t)131, (uint8_t)211, (uint8_t)203, (uint8_t)28, (uint8_t)199, (uint8_t)239, (uint8_t)81, (uint8_t)150, (uint8_t)221, (uint8_t)158, (uint8_t)162, (uint8_t)70, (uint8_t)250, (uint8_t)79, (uint8_t)28, (uint8_t)208, (uint8_t)112, (uint8_t)89, (uint8_t)159, (uint8_t)81, (uint8_t)211, (uint8_t)243, (uint8_t)80, (uint8_t)244, (uint8_t)251, (uint8_t)95, (uint8_t)164, (uint8_t)42, (uint8_t)124, (uint8_t)166, (uint8_t)217, (uint8_t)107, (uint8_t)196, (uint8_t)0, (uint8_t)222, (uint8_t)232, (uint8_t)38, (uint8_t)212, (uint8_t)207, (uint8_t)20, (uint8_t)29, (uint8_t)240, (uint8_t)44, (uint8_t)208, (uint8_t)16, (uint8_t)207, (uint8_t)6, (uint8_t)56, (uint8_t)162, (uint8_t)160, (uint8_t)200, (uint8_t)108, (uint8_t)223, (uint8_t)7, (uint8_t)33, (uint8_t)197, (uint8_t)18, (uint8_t)48, (uint8_t)120, (uint8_t)2, (uint8_t)106, (uint8_t)79, (uint8_t)189, (uint8_t)105, (uint8_t)200, (uint8_t)192, (uint8_t)4, (uint8_t)95, (uint8_t)103, (uint8_t)9, (uint8_t)148, (uint8_t)33, (uint8_t)183, (uint8_t)251, (uint8_t)89, (uint8_t)237, (uint8_t)77, (uint8_t)250, (uint8_t)139, (uint8_t)26, (uint8_t)3, (uint8_t)6, (uint8_t)29, (uint8_t)25, (uint8_t)143, (uint8_t)5, (uint8_t)92, (uint8_t)123, (uint8_t)8, (uint8_t)64, (uint8_t)115, (uint8_t)55, (uint8_t)137, (uint8_t)221, (uint8_t)244, (uint8_t)57, (uint8_t)225, (uint8_t)198, (uint8_t)118, (uint8_t)79, (uint8_t)251, (uint8_t)204, (uint8_t)248, (uint8_t)3, (uint8_t)163, (uint8_t)183, (uint8_t)48, (uint8_t)71, (uint8_t)141, (uint8_t)206, (uint8_t)120, (uint8_t)131, (uint8_t)157, (uint8_t)9, (uint8_t)18, (uint8_t)240, (uint8_t)11, (uint8_t)181, (uint8_t)192, (uint8_t)117, (uint8_t)221, (uint8_t)19, (uint8_t)175, (uint8_t)236, (uint8_t)87, (uint8_t)222, (uint8_t)153, (uint8_t)219, (uint8_t)235, (uint8_t)52, (uint8_t)61, (uint8_t)12, (uint8_t)20, (uint8_t)165, (uint8_t)177, (uint8_t)17, (uint8_t)206, (uint8_t)209, (uint8_t)203, (uint8_t)17, (uint8_t)96, (uint8_t)100, (uint8_t)196, (uint8_t)199, (uint8_t)165, (uint8_t)253, (uint8_t)196, (uint8_t)105, (uint8_t)244, (uint8_t)32, (uint8_t)102, (uint8_t)21, (uint8_t)237, (uint8_t)197, (uint8_t)249, (uint8_t)153, (uint8_t)161, (uint8_t)21, (uint8_t)29, (uint8_t)204, (uint8_t)167, (uint8_t)117, (uint8_t)88, (uint8_t)163, (uint8_t)222, (uint8_t)225, (uint8_t)247, (uint8_t)119, (uint8_t)174, (uint8_t)80, (uint8_t)141, (uint8_t)12, (uint8_t)225, (uint8_t)37, (uint8_t)97, (uint8_t)191, (uint8_t)155, (uint8_t)63, (uint8_t)222, (uint8_t)96, (uint8_t)241, (uint8_t)40, (uint8_t)14, (uint8_t)192, (uint8_t)235, (uint8_t)46, (uint8_t)169, (uint8_t)58, (uint8_t)65, (uint8_t)226, (uint8_t)111, (uint8_t)102};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MEMORY_VECT_249(), &PH);
        p249_ver_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        p249_type_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t)42, (int8_t) -78, (int8_t) -46, (int8_t)95, (int8_t)125, (int8_t) -106, (int8_t)98, (int8_t) -59, (int8_t) -52, (int8_t) -92, (int8_t)64, (int8_t)40, (int8_t)20, (int8_t) -33, (int8_t)69, (int8_t) -86, (int8_t)19, (int8_t)31, (int8_t)97, (int8_t) -55, (int8_t) -110, (int8_t)84, (int8_t)88, (int8_t) -67, (int8_t) -82, (int8_t) -118, (int8_t)85, (int8_t)122, (int8_t) -122, (int8_t) -121, (int8_t) -61, (int8_t) -97};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_address_SET((uint16_t)(uint16_t)50617, PH.base.pack) ;
        c_CommunicationChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_VECT_250(), &PH);
        p250_z_SET((float)2.7095328E38F, PH.base.pack) ;
        {
            char16_t* name = u"cMz";
            p250_name_SET_(name, &PH) ;
        }
        p250_y_SET((float) -5.7876277E37F, PH.base.pack) ;
        p250_x_SET((float) -3.3999933E38F, PH.base.pack) ;
        p250_time_usec_SET((uint64_t)3486507564428539286L, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_FLOAT_251(), &PH);
        {
            char16_t* name = u"ejxvzmn";
            p251_name_SET_(name, &PH) ;
        }
        p251_time_boot_ms_SET((uint32_t)1704974398L, PH.base.pack) ;
        p251_value_SET((float) -1.9238748E38F, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_INT_252(), &PH);
        p252_time_boot_ms_SET((uint32_t)1532238953L, PH.base.pack) ;
        p252_value_SET((int32_t) -233100281, PH.base.pack) ;
        {
            char16_t* name = u"ofsSe";
            p252_name_SET_(name, &PH) ;
        }
        c_CommunicationChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STATUSTEXT_253(), &PH);
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_INFO, PH.base.pack) ;
        {
            char16_t* text = u"ocpdhxIbemyzgnizlmwdwrfdmhciqnygufllYalwv";
            p253_text_SET_(text, &PH) ;
        }
        c_CommunicationChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_254(), &PH);
        p254_value_SET((float) -1.752027E38F, PH.base.pack) ;
        p254_ind_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)615850665L, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SETUP_SIGNING_256(), &PH);
        {
            uint8_t secret_key[] =  {(uint8_t)18, (uint8_t)232, (uint8_t)108, (uint8_t)193, (uint8_t)24, (uint8_t)253, (uint8_t)139, (uint8_t)129, (uint8_t)211, (uint8_t)110, (uint8_t)137, (uint8_t)239, (uint8_t)7, (uint8_t)110, (uint8_t)208, (uint8_t)177, (uint8_t)149, (uint8_t)57, (uint8_t)75, (uint8_t)31, (uint8_t)182, (uint8_t)77, (uint8_t)57, (uint8_t)168, (uint8_t)41, (uint8_t)67, (uint8_t)54, (uint8_t)108, (uint8_t)146, (uint8_t)199, (uint8_t)76, (uint8_t)245};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        p256_initial_timestamp_SET((uint64_t)633515334548168862L, PH.base.pack) ;
        p256_target_component_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p256_target_system_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        c_CommunicationChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BUTTON_CHANGE_257(), &PH);
        p257_last_change_ms_SET((uint32_t)3588070996L, PH.base.pack) ;
        p257_state_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
        p257_time_boot_ms_SET((uint32_t)1501436732L, PH.base.pack) ;
        c_CommunicationChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PLAY_TUNE_258(), &PH);
        p258_target_component_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        p258_target_system_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        {
            char16_t* tune = u"zgslhvjo";
            p258_tune_SET_(tune, &PH) ;
        }
        c_CommunicationChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_INFORMATION_259(), &PH);
        p259_sensor_size_h_SET((float) -1.5899593E38F, PH.base.pack) ;
        p259_cam_definition_version_SET((uint16_t)(uint16_t)19614, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)212, (uint8_t)61, (uint8_t)200, (uint8_t)200, (uint8_t)2, (uint8_t)207, (uint8_t)180, (uint8_t)167, (uint8_t)217, (uint8_t)17, (uint8_t)182, (uint8_t)84, (uint8_t)30, (uint8_t)53, (uint8_t)144, (uint8_t)66, (uint8_t)120, (uint8_t)122, (uint8_t)247, (uint8_t)137, (uint8_t)20, (uint8_t)253, (uint8_t)123, (uint8_t)222, (uint8_t)14, (uint8_t)134, (uint8_t)67, (uint8_t)132, (uint8_t)168, (uint8_t)13, (uint8_t)167, (uint8_t)8};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_sensor_size_v_SET((float)3.5142763E37F, PH.base.pack) ;
        p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES, PH.base.pack) ;
        p259_time_boot_ms_SET((uint32_t)888913768L, PH.base.pack) ;
        p259_resolution_v_SET((uint16_t)(uint16_t)51246, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)57923, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"kWLdw";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_focal_length_SET((float) -2.2013982E38F, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)195, (uint8_t)30, (uint8_t)0, (uint8_t)214, (uint8_t)160, (uint8_t)51, (uint8_t)223, (uint8_t)215, (uint8_t)141, (uint8_t)180, (uint8_t)168, (uint8_t)230, (uint8_t)8, (uint8_t)245, (uint8_t)127, (uint8_t)110, (uint8_t)55, (uint8_t)175, (uint8_t)154, (uint8_t)240, (uint8_t)86, (uint8_t)100, (uint8_t)135, (uint8_t)213, (uint8_t)176, (uint8_t)234, (uint8_t)195, (uint8_t)249, (uint8_t)34, (uint8_t)225, (uint8_t)18, (uint8_t)16};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_firmware_version_SET((uint32_t)3637224030L, PH.base.pack) ;
        p259_lens_id_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_SETTINGS_260(), &PH);
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_VIDEO, PH.base.pack) ;
        p260_time_boot_ms_SET((uint32_t)1953402280L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STORAGE_INFORMATION_261(), &PH);
        p261_storage_id_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p261_available_capacity_SET((float)5.7831245E37F, PH.base.pack) ;
        p261_total_capacity_SET((float)2.7226608E38F, PH.base.pack) ;
        p261_used_capacity_SET((float)3.000668E38F, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        p261_write_speed_SET((float) -2.021595E38F, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)3284622734L, PH.base.pack) ;
        p261_read_speed_SET((float)2.6007199E38F, PH.base.pack) ;
        c_CommunicationChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_available_capacity_SET((float) -1.8972945E38F, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p262_recording_time_ms_SET((uint32_t)3594837601L, PH.base.pack) ;
        p262_image_interval_SET((float)1.9767021E38F, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)3683909764L, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_capture_result_SET((int8_t)(int8_t) -105, PH.base.pack) ;
        p263_camera_id_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        p263_lat_SET((int32_t) -1701311411, PH.base.pack) ;
        p263_image_index_SET((int32_t) -1501649527, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)3122380836L, PH.base.pack) ;
        {
            float q[] =  {-3.4225953E37F, 1.7098319E38F, -2.3307513E38F, -5.4801565E37F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_time_utc_SET((uint64_t)4021644501310134824L, PH.base.pack) ;
        {
            char16_t* file_url = u"gjwowvjhhaqexifsarseodwqpssRnnhpKhybidQkyamcwvkcPvqutdypnrtreTfczmabbfBjzudhNxcodvkktnyfKemazoayiMXjrpvukhceRhbcgtxuzhlcfJhwXoxhqygtjrqlvxpysgspoocrAfciYshZupwskCxxsmsgjhxivnklfbualruv";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_alt_SET((int32_t)1879260172, PH.base.pack) ;
        p263_lon_SET((int32_t) -1172797255, PH.base.pack) ;
        p263_relative_alt_SET((int32_t)1982799590, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_time_boot_ms_SET((uint32_t)3759068448L, PH.base.pack) ;
        p264_arming_time_utc_SET((uint64_t)84913462640655291L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)5645365529008825778L, PH.base.pack) ;
        p264_takeoff_time_utc_SET((uint64_t)6088455146710635728L, PH.base.pack) ;
        c_CommunicationChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_roll_SET((float) -9.563315E37F, PH.base.pack) ;
        p265_time_boot_ms_SET((uint32_t)3720156103L, PH.base.pack) ;
        p265_pitch_SET((float)2.3313096E38F, PH.base.pack) ;
        p265_yaw_SET((float)5.6599935E37F, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_266(), &PH);
        p266_sequence_SET((uint16_t)(uint16_t)25046, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p266_length_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)134, (uint8_t)97, (uint8_t)165, (uint8_t)181, (uint8_t)135, (uint8_t)50, (uint8_t)173, (uint8_t)239, (uint8_t)161, (uint8_t)127, (uint8_t)141, (uint8_t)147, (uint8_t)190, (uint8_t)40, (uint8_t)175, (uint8_t)46, (uint8_t)1, (uint8_t)213, (uint8_t)167, (uint8_t)77, (uint8_t)48, (uint8_t)195, (uint8_t)73, (uint8_t)184, (uint8_t)68, (uint8_t)198, (uint8_t)161, (uint8_t)76, (uint8_t)97, (uint8_t)90, (uint8_t)13, (uint8_t)110, (uint8_t)190, (uint8_t)139, (uint8_t)175, (uint8_t)62, (uint8_t)135, (uint8_t)215, (uint8_t)112, (uint8_t)79, (uint8_t)130, (uint8_t)191, (uint8_t)252, (uint8_t)93, (uint8_t)246, (uint8_t)132, (uint8_t)227, (uint8_t)138, (uint8_t)105, (uint8_t)98, (uint8_t)61, (uint8_t)235, (uint8_t)70, (uint8_t)72, (uint8_t)245, (uint8_t)218, (uint8_t)195, (uint8_t)187, (uint8_t)210, (uint8_t)176, (uint8_t)241, (uint8_t)34, (uint8_t)211, (uint8_t)129, (uint8_t)121, (uint8_t)211, (uint8_t)236, (uint8_t)59, (uint8_t)22, (uint8_t)23, (uint8_t)58, (uint8_t)223, (uint8_t)154, (uint8_t)89, (uint8_t)71, (uint8_t)141, (uint8_t)192, (uint8_t)21, (uint8_t)41, (uint8_t)226, (uint8_t)194, (uint8_t)48, (uint8_t)2, (uint8_t)172, (uint8_t)0, (uint8_t)196, (uint8_t)180, (uint8_t)242, (uint8_t)133, (uint8_t)241, (uint8_t)81, (uint8_t)19, (uint8_t)165, (uint8_t)88, (uint8_t)242, (uint8_t)12, (uint8_t)13, (uint8_t)15, (uint8_t)34, (uint8_t)203, (uint8_t)249, (uint8_t)100, (uint8_t)226, (uint8_t)77, (uint8_t)93, (uint8_t)9, (uint8_t)24, (uint8_t)235, (uint8_t)151, (uint8_t)122, (uint8_t)79, (uint8_t)99, (uint8_t)152, (uint8_t)186, (uint8_t)245, (uint8_t)137, (uint8_t)97, (uint8_t)5, (uint8_t)143, (uint8_t)80, (uint8_t)29, (uint8_t)209, (uint8_t)158, (uint8_t)46, (uint8_t)136, (uint8_t)57, (uint8_t)4, (uint8_t)149, (uint8_t)103, (uint8_t)72, (uint8_t)42, (uint8_t)21, (uint8_t)225, (uint8_t)90, (uint8_t)12, (uint8_t)77, (uint8_t)192, (uint8_t)134, (uint8_t)11, (uint8_t)21, (uint8_t)85, (uint8_t)22, (uint8_t)244, (uint8_t)149, (uint8_t)128, (uint8_t)157, (uint8_t)91, (uint8_t)242, (uint8_t)161, (uint8_t)152, (uint8_t)86, (uint8_t)106, (uint8_t)29, (uint8_t)125, (uint8_t)124, (uint8_t)58, (uint8_t)178, (uint8_t)142, (uint8_t)202, (uint8_t)58, (uint8_t)70, (uint8_t)46, (uint8_t)240, (uint8_t)91, (uint8_t)116, (uint8_t)31, (uint8_t)165, (uint8_t)52, (uint8_t)6, (uint8_t)186, (uint8_t)58, (uint8_t)202, (uint8_t)160, (uint8_t)198, (uint8_t)103, (uint8_t)166, (uint8_t)144, (uint8_t)110, (uint8_t)53, (uint8_t)211, (uint8_t)138, (uint8_t)136, (uint8_t)43, (uint8_t)104, (uint8_t)50, (uint8_t)138, (uint8_t)163, (uint8_t)246, (uint8_t)149, (uint8_t)109, (uint8_t)48, (uint8_t)88, (uint8_t)94, (uint8_t)13, (uint8_t)234, (uint8_t)51, (uint8_t)5, (uint8_t)128, (uint8_t)135, (uint8_t)172, (uint8_t)31, (uint8_t)135, (uint8_t)183, (uint8_t)68, (uint8_t)104, (uint8_t)94, (uint8_t)185, (uint8_t)45, (uint8_t)6, (uint8_t)60, (uint8_t)126, (uint8_t)4, (uint8_t)224, (uint8_t)184, (uint8_t)154, (uint8_t)204, (uint8_t)121, (uint8_t)129, (uint8_t)106, (uint8_t)185, (uint8_t)83, (uint8_t)156, (uint8_t)254, (uint8_t)181, (uint8_t)164, (uint8_t)110, (uint8_t)115, (uint8_t)108, (uint8_t)245, (uint8_t)217, (uint8_t)216, (uint8_t)223, (uint8_t)40, (uint8_t)112, (uint8_t)250, (uint8_t)92, (uint8_t)13, (uint8_t)18, (uint8_t)240, (uint8_t)88, (uint8_t)123, (uint8_t)136, (uint8_t)34, (uint8_t)61, (uint8_t)82, (uint8_t)222, (uint8_t)81, (uint8_t)17, (uint8_t)249};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_target_system_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p266_first_message_offset_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_ACKED_267(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)8, (uint8_t)69, (uint8_t)122, (uint8_t)6, (uint8_t)22, (uint8_t)184, (uint8_t)105, (uint8_t)7, (uint8_t)85, (uint8_t)32, (uint8_t)119, (uint8_t)48, (uint8_t)73, (uint8_t)166, (uint8_t)67, (uint8_t)56, (uint8_t)120, (uint8_t)17, (uint8_t)50, (uint8_t)113, (uint8_t)110, (uint8_t)252, (uint8_t)203, (uint8_t)214, (uint8_t)94, (uint8_t)93, (uint8_t)254, (uint8_t)1, (uint8_t)188, (uint8_t)96, (uint8_t)254, (uint8_t)21, (uint8_t)49, (uint8_t)244, (uint8_t)20, (uint8_t)101, (uint8_t)222, (uint8_t)65, (uint8_t)140, (uint8_t)89, (uint8_t)212, (uint8_t)110, (uint8_t)2, (uint8_t)10, (uint8_t)81, (uint8_t)84, (uint8_t)207, (uint8_t)244, (uint8_t)31, (uint8_t)86, (uint8_t)92, (uint8_t)48, (uint8_t)102, (uint8_t)236, (uint8_t)88, (uint8_t)245, (uint8_t)24, (uint8_t)9, (uint8_t)21, (uint8_t)228, (uint8_t)146, (uint8_t)89, (uint8_t)206, (uint8_t)202, (uint8_t)254, (uint8_t)194, (uint8_t)162, (uint8_t)223, (uint8_t)146, (uint8_t)199, (uint8_t)204, (uint8_t)218, (uint8_t)69, (uint8_t)217, (uint8_t)5, (uint8_t)144, (uint8_t)230, (uint8_t)212, (uint8_t)240, (uint8_t)198, (uint8_t)141, (uint8_t)147, (uint8_t)233, (uint8_t)45, (uint8_t)58, (uint8_t)136, (uint8_t)67, (uint8_t)1, (uint8_t)115, (uint8_t)221, (uint8_t)90, (uint8_t)227, (uint8_t)86, (uint8_t)173, (uint8_t)41, (uint8_t)156, (uint8_t)223, (uint8_t)196, (uint8_t)103, (uint8_t)178, (uint8_t)114, (uint8_t)218, (uint8_t)137, (uint8_t)52, (uint8_t)35, (uint8_t)194, (uint8_t)216, (uint8_t)125, (uint8_t)188, (uint8_t)144, (uint8_t)190, (uint8_t)121, (uint8_t)153, (uint8_t)51, (uint8_t)91, (uint8_t)205, (uint8_t)239, (uint8_t)225, (uint8_t)6, (uint8_t)57, (uint8_t)143, (uint8_t)171, (uint8_t)242, (uint8_t)210, (uint8_t)21, (uint8_t)251, (uint8_t)197, (uint8_t)12, (uint8_t)249, (uint8_t)58, (uint8_t)196, (uint8_t)33, (uint8_t)162, (uint8_t)208, (uint8_t)157, (uint8_t)98, (uint8_t)149, (uint8_t)38, (uint8_t)37, (uint8_t)133, (uint8_t)89, (uint8_t)41, (uint8_t)155, (uint8_t)214, (uint8_t)178, (uint8_t)217, (uint8_t)218, (uint8_t)180, (uint8_t)16, (uint8_t)120, (uint8_t)164, (uint8_t)111, (uint8_t)151, (uint8_t)16, (uint8_t)77, (uint8_t)236, (uint8_t)34, (uint8_t)55, (uint8_t)93, (uint8_t)225, (uint8_t)125, (uint8_t)145, (uint8_t)13, (uint8_t)93, (uint8_t)236, (uint8_t)198, (uint8_t)64, (uint8_t)147, (uint8_t)4, (uint8_t)17, (uint8_t)188, (uint8_t)252, (uint8_t)149, (uint8_t)180, (uint8_t)14, (uint8_t)161, (uint8_t)123, (uint8_t)217, (uint8_t)118, (uint8_t)71, (uint8_t)188, (uint8_t)95, (uint8_t)166, (uint8_t)125, (uint8_t)61, (uint8_t)86, (uint8_t)190, (uint8_t)73, (uint8_t)11, (uint8_t)15, (uint8_t)157, (uint8_t)102, (uint8_t)116, (uint8_t)14, (uint8_t)80, (uint8_t)237, (uint8_t)243, (uint8_t)9, (uint8_t)32, (uint8_t)194, (uint8_t)141, (uint8_t)44, (uint8_t)87, (uint8_t)38, (uint8_t)146, (uint8_t)69, (uint8_t)91, (uint8_t)124, (uint8_t)87, (uint8_t)70, (uint8_t)239, (uint8_t)227, (uint8_t)176, (uint8_t)7, (uint8_t)21, (uint8_t)63, (uint8_t)93, (uint8_t)92, (uint8_t)107, (uint8_t)41, (uint8_t)253, (uint8_t)39, (uint8_t)246, (uint8_t)37, (uint8_t)88, (uint8_t)170, (uint8_t)237, (uint8_t)245, (uint8_t)246, (uint8_t)128, (uint8_t)52, (uint8_t)148, (uint8_t)139, (uint8_t)233, (uint8_t)138, (uint8_t)118, (uint8_t)78, (uint8_t)6, (uint8_t)153, (uint8_t)201, (uint8_t)162, (uint8_t)200, (uint8_t)216, (uint8_t)79, (uint8_t)40, (uint8_t)143, (uint8_t)147, (uint8_t)38, (uint8_t)108};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_sequence_SET((uint16_t)(uint16_t)30241, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        p267_target_component_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_ACK_268(), &PH);
        p268_target_component_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        p268_target_system_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        p268_sequence_SET((uint16_t)(uint16_t)23191, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_camera_id_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)18096, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)15664, PH.base.pack) ;
        p269_bitrate_SET((uint32_t)1692809756L, PH.base.pack) ;
        {
            char16_t* uri = u"LnbijXefehcblnsiypjotzuamhtuoufDpoxphfcjgroyyzoqpgirfvobpdgikGttkmcyvmjcktzfvBykxkdknzdvfCBldkyidzHbkneInbuatmuyc";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_framerate_SET((float) -2.9854421E38F, PH.base.pack) ;
        p269_status_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p269_rotation_SET((uint16_t)(uint16_t)9569, PH.base.pack) ;
        c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_resolution_h_SET((uint16_t)(uint16_t)60561, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)59110, PH.base.pack) ;
        p270_framerate_SET((float) -1.658811E38F, PH.base.pack) ;
        p270_target_component_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        {
            char16_t* uri = u"QgykjgfxsMzaiKuejhrcsiucbqkppbzvkdNiupkegVgalWvghofzrcopNuyfwynjttuoenlqxhfPtJcwOfbaoHefpzbyrzrvrzxumldPghfbDk";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_bitrate_SET((uint32_t)290694674L, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)16634, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* ssid = u"wkuEdgbtD";
            p299_ssid_SET_(ssid, &PH) ;
        }
        {
            char16_t* password = u"vt";
            p299_password_SET_(password, &PH) ;
        }
        c_CommunicationChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PROTOCOL_VERSION_300(), &PH);
        p300_max_version_SET((uint16_t)(uint16_t)34609, PH.base.pack) ;
        {
            uint8_t spec_version_hash[] =  {(uint8_t)199, (uint8_t)23, (uint8_t)44, (uint8_t)223, (uint8_t)75, (uint8_t)73, (uint8_t)208, (uint8_t)251};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        {
            uint8_t library_version_hash[] =  {(uint8_t)126, (uint8_t)20, (uint8_t)7, (uint8_t)108, (uint8_t)42, (uint8_t)250, (uint8_t)226, (uint8_t)196};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        p300_version_SET((uint16_t)(uint16_t)45046, PH.base.pack) ;
        p300_min_version_SET((uint16_t)(uint16_t)46514, PH.base.pack) ;
        c_CommunicationChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_uptime_sec_SET((uint32_t)523861249L, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)1183291904642880617L, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)23366, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_OK, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_INFO_311(), &PH);
        {
            char16_t* name = u"xbljknwMzwubvfqemuqqQueoewvfvsplhtw";
            p311_name_SET_(name, &PH) ;
        }
        p311_hw_version_major_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)43, (uint8_t)186, (uint8_t)219, (uint8_t)190, (uint8_t)122, (uint8_t)218, (uint8_t)217, (uint8_t)176, (uint8_t)225, (uint8_t)224, (uint8_t)3, (uint8_t)56, (uint8_t)165, (uint8_t)224, (uint8_t)143, (uint8_t)173};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        p311_time_usec_SET((uint64_t)7398593798634825819L, PH.base.pack) ;
        p311_sw_version_major_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p311_hw_version_minor_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)3549404889L, PH.base.pack) ;
        p311_sw_version_minor_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)2842300958L, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        p320_target_system_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        p320_target_component_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        {
            char16_t* param_id = u"znmprYxvkcr";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_param_index_SET((int16_t)(int16_t) -27292, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_component_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p321_target_system_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_VALUE_322(), &PH);
        {
            char16_t* param_id = u"Xprsnug";
            p322_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"ikxrXuRzterejufyoegbgcopvTdMhrjxrknsoEynvczoukNlrwfkzticdolKrtewcoljwJvvma";
            p322_param_value_SET_(param_value, &PH) ;
        }
        p322_param_index_SET((uint16_t)(uint16_t)40376, PH.base.pack) ;
        p322_param_count_SET((uint16_t)(uint16_t)51381, PH.base.pack) ;
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_SET_323(), &PH);
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_CUSTOM, PH.base.pack) ;
        p323_target_system_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        {
            char16_t* param_value = u"gsivxGnwdldwlzjepwgrvvylvTBmnuownzmlgUhncttfeTepEwCmelzdabnbdzpvmk";
            p323_param_value_SET_(param_value, &PH) ;
        }
        p323_target_component_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        {
            char16_t* param_id = u"Bkparfegalk";
            p323_param_id_SET_(param_id, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_ACK_324(), &PH);
        {
            char16_t* param_id = u"kz";
            p324_param_id_SET_(param_id, &PH) ;
        }
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_VALUE_UNSUPPORTED, PH.base.pack) ;
        {
            char16_t* param_value = u"pzcxXkjghxhoitiathuTtdtyPmynnnpfyoaavipfdrbkoymleiXxifhcXO";
            p324_param_value_SET_(param_value, &PH) ;
        }
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT64, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_OBSTACLE_DISTANCE_330(), &PH);
        p330_min_distance_SET((uint16_t)(uint16_t)65263, PH.base.pack) ;
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
        p330_max_distance_SET((uint16_t)(uint16_t)33275, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)21443, (uint16_t)34606, (uint16_t)35393, (uint16_t)65145, (uint16_t)26338, (uint16_t)9089, (uint16_t)188, (uint16_t)55243, (uint16_t)35763, (uint16_t)64303, (uint16_t)41585, (uint16_t)56993, (uint16_t)56927, (uint16_t)28396, (uint16_t)46145, (uint16_t)47330, (uint16_t)51504, (uint16_t)5697, (uint16_t)36670, (uint16_t)23611, (uint16_t)21541, (uint16_t)63132, (uint16_t)37865, (uint16_t)38743, (uint16_t)9500, (uint16_t)22516, (uint16_t)60559, (uint16_t)35860, (uint16_t)20496, (uint16_t)46724, (uint16_t)14529, (uint16_t)16219, (uint16_t)16909, (uint16_t)28796, (uint16_t)15425, (uint16_t)45399, (uint16_t)788, (uint16_t)10256, (uint16_t)10273, (uint16_t)30856, (uint16_t)19733, (uint16_t)30072, (uint16_t)16282, (uint16_t)14692, (uint16_t)3539, (uint16_t)45549, (uint16_t)19918, (uint16_t)63568, (uint16_t)57312, (uint16_t)2081, (uint16_t)49972, (uint16_t)32833, (uint16_t)59649, (uint16_t)13689, (uint16_t)13871, (uint16_t)17736, (uint16_t)35725, (uint16_t)53197, (uint16_t)29896, (uint16_t)1616, (uint16_t)8496, (uint16_t)61918, (uint16_t)8413, (uint16_t)9009, (uint16_t)15637, (uint16_t)61412, (uint16_t)40067, (uint16_t)34381, (uint16_t)59009, (uint16_t)29900, (uint16_t)65288, (uint16_t)63080};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_increment_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        p330_time_usec_SET((uint64_t)4385629047192520414L, PH.base.pack) ;
        c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
}


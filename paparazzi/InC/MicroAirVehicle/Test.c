
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
INLINER void p180_seq_SET(uint16_t  src, Pack * dst)//Sequence
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p180_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p180_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p180_name_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //The name of the mission script, NULL terminated.
{
    if(dst->base.field_bit != 32 && insert_field(dst, 32, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p180_name_SET_(char16_t*  src, Bounds_Inside * dst) {p180_name_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_SCRIPT_ITEM_180()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 180));
};
INLINER void p181_seq_SET(uint16_t  src, Pack * dst)//Sequence
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p181_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p181_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
Pack * c_TEST_Channel_new_SCRIPT_REQUEST_181()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 181));
};
INLINER void p182_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p182_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
Pack * c_TEST_Channel_new_SCRIPT_REQUEST_LIST_182()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 182));
};
INLINER void p183_count_SET(uint16_t  src, Pack * dst)//Number of script items in the sequence
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p183_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p183_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
Pack * c_TEST_Channel_new_SCRIPT_COUNT_183()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 183));
};
INLINER void p184_seq_SET(uint16_t  src, Pack * dst)//Active Sequence
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
Pack * c_TEST_Channel_new_SCRIPT_CURRENT_184()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 184));
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
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)63);
    assert(p0_custom_mode_GET(pack) == (uint32_t)2244579788L);
    assert(p0_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED);
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_FP);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_VTOL_DUOROTOR);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_CRITICAL);
};


void c_TEST_Channel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)37960);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE);
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)20068);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)16611);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)63551);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)46445);
    assert(p1_onboard_control_sensors_present_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)12703);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)13248);
    assert(p1_onboard_control_sensors_health_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t)7979);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t) -84);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)3410);
};


void c_TEST_Channel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)3023421201L);
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)5496399966035986942L);
};


void c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_afz_GET(pack) == (float)1.206009E38F);
    assert(p3_vx_GET(pack) == (float) -1.6381922E38F);
    assert(p3_vy_GET(pack) == (float)2.8099625E38F);
    assert(p3_y_GET(pack) == (float)1.954934E38F);
    assert(p3_afy_GET(pack) == (float)3.5315595E37F);
    assert(p3_vz_GET(pack) == (float)2.0536041E38F);
    assert(p3_yaw_GET(pack) == (float)2.8201291E38F);
    assert(p3_yaw_rate_GET(pack) == (float)1.8658677E37F);
    assert(p3_afx_GET(pack) == (float)2.5358008E38F);
    assert(p3_x_GET(pack) == (float)3.394667E37F);
    assert(p3_z_GET(pack) == (float)9.522373E37F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)3436052476L);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)34546);
};


void c_TEST_Channel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)213);
    assert(p4_seq_GET(pack) == (uint32_t)2845956950L);
    assert(p4_time_usec_GET(pack) == (uint64_t)2167029319243596742L);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p5_passkey_LEN(ph) == 2);
    {
        char16_t * exemplary = u"iy";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)61);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)231);
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)85);
};


void c_TEST_Channel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 30);
    {
        char16_t * exemplary = u"fearnmljfrhwfxospkmgfBnffrdJdp";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 60);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p11_custom_mode_GET(pack) == (uint32_t)3126152854L);
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_MANUAL_DISARMED);
};


void c_TEST_Channel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t) -12756);
    assert(p20_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"i";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)254);
};


void c_TEST_Channel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)242);
};


void c_TEST_Channel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)51229);
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)9932);
    assert(p22_param_value_GET(pack) == (float) -1.4255161E38F);
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT16);
    assert(p22_param_id_LEN(ph) == 8);
    {
        char16_t * exemplary = u"tivuxeKn";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64);
    assert(p23_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"Q";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_param_value_GET(pack) == (float)2.147907E38F);
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)186);
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)225);
};


void c_TEST_Channel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_lon_GET(pack) == (int32_t)191648058);
    assert(p24_time_usec_GET(pack) == (uint64_t)489585060487090865L);
    assert(p24_alt_GET(pack) == (int32_t)1848542608);
    assert(p24_lat_GET(pack) == (int32_t)1150299977);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)1125);
    assert(p24_h_acc_TRY(ph) == (uint32_t)4013204295L);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t)397157427);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)3963574299L);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)53908);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)131);
    assert(p24_v_acc_TRY(ph) == (uint32_t)886351382L);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)1703123430L);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)7037);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)39716);
};


void c_TEST_Channel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)185, (uint8_t)143, (uint8_t)77, (uint8_t)205, (uint8_t)158, (uint8_t)0, (uint8_t)57, (uint8_t)254, (uint8_t)224, (uint8_t)128, (uint8_t)43, (uint8_t)159, (uint8_t)131, (uint8_t)185, (uint8_t)47, (uint8_t)137, (uint8_t)133, (uint8_t)7, (uint8_t)133, (uint8_t)187} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)123, (uint8_t)172, (uint8_t)249, (uint8_t)26, (uint8_t)194, (uint8_t)174, (uint8_t)72, (uint8_t)46, (uint8_t)83, (uint8_t)235, (uint8_t)142, (uint8_t)153, (uint8_t)4, (uint8_t)17, (uint8_t)166, (uint8_t)134, (uint8_t)198, (uint8_t)236, (uint8_t)38, (uint8_t)242} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)109, (uint8_t)23, (uint8_t)30, (uint8_t)55, (uint8_t)98, (uint8_t)75, (uint8_t)163, (uint8_t)233, (uint8_t)42, (uint8_t)95, (uint8_t)58, (uint8_t)27, (uint8_t)211, (uint8_t)193, (uint8_t)241, (uint8_t)100, (uint8_t)175, (uint8_t)210, (uint8_t)202, (uint8_t)214} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)89, (uint8_t)103, (uint8_t)245, (uint8_t)107, (uint8_t)141, (uint8_t)41, (uint8_t)102, (uint8_t)83, (uint8_t)76, (uint8_t)146, (uint8_t)173, (uint8_t)153, (uint8_t)139, (uint8_t)198, (uint8_t)246, (uint8_t)96, (uint8_t)60, (uint8_t)174, (uint8_t)48, (uint8_t)144} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)137);
    {
        uint8_t exemplary[] =  {(uint8_t)13, (uint8_t)95, (uint8_t)72, (uint8_t)253, (uint8_t)169, (uint8_t)115, (uint8_t)251, (uint8_t)9, (uint8_t)81, (uint8_t)73, (uint8_t)159, (uint8_t)152, (uint8_t)34, (uint8_t)145, (uint8_t)238, (uint8_t)147, (uint8_t)235, (uint8_t)96, (uint8_t)110, (uint8_t)171} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t) -30239);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)1107123120L);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t) -27429);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t)21570);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t)9882);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t) -14888);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t)26156);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t)9672);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t)20629);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t)9301);
};


void c_TEST_Channel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t)5838);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t) -31333);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t) -18437);
    assert(p27_time_usec_GET(pack) == (uint64_t)6558650088664402933L);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t)14143);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t) -31877);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t) -8271);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t)26488);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t)26616);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t)1157);
};


void c_TEST_Channel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t)4041);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t)11720);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t) -4560);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t) -15361);
    assert(p28_time_usec_GET(pack) == (uint64_t)2258844833674639837L);
};


void c_TEST_Channel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t) -5652);
    assert(p29_press_abs_GET(pack) == (float)1.6443614E38F);
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)782945827L);
    assert(p29_press_diff_GET(pack) == (float) -2.3917464E37F);
};


void c_TEST_Channel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_yawspeed_GET(pack) == (float)5.6336847E37F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)1029576882L);
    assert(p30_rollspeed_GET(pack) == (float) -1.3323691E38F);
    assert(p30_roll_GET(pack) == (float) -5.592275E37F);
    assert(p30_pitch_GET(pack) == (float)3.2918154E38F);
    assert(p30_yaw_GET(pack) == (float) -9.207099E37F);
    assert(p30_pitchspeed_GET(pack) == (float)2.9595143E38F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_pitchspeed_GET(pack) == (float)2.286827E38F);
    assert(p31_rollspeed_GET(pack) == (float)1.0797617E38F);
    assert(p31_q1_GET(pack) == (float)2.3842185E38F);
    assert(p31_yawspeed_GET(pack) == (float)3.1840554E38F);
    assert(p31_q2_GET(pack) == (float) -3.1337043E38F);
    assert(p31_q4_GET(pack) == (float)1.5370253E38F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)1459240154L);
    assert(p31_q3_GET(pack) == (float) -2.2432566E37F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)2040140604L);
    assert(p32_x_GET(pack) == (float)1.1443959E38F);
    assert(p32_vx_GET(pack) == (float) -6.4141604E37F);
    assert(p32_z_GET(pack) == (float) -2.8268264E38F);
    assert(p32_y_GET(pack) == (float)1.3788593E38F);
    assert(p32_vz_GET(pack) == (float) -3.0860017E38F);
    assert(p32_vy_GET(pack) == (float) -6.7851044E37F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_vz_GET(pack) == (int16_t)(int16_t) -19585);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t) -19679);
    assert(p33_lat_GET(pack) == (int32_t)1985202301);
    assert(p33_relative_alt_GET(pack) == (int32_t)559728038);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)14977);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)1867922209L);
    assert(p33_alt_GET(pack) == (int32_t) -1477720047);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t)2808);
    assert(p33_lon_GET(pack) == (int32_t)933980537);
};


void c_TEST_Channel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)109);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t)27810);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t) -29587);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t) -15276);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t)23434);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t) -8739);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t) -2433);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)3386437292L);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t)8031);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t)12468);
};


void c_TEST_Channel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)45764);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)8793);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)43629);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)164);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)18768);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)3215154141L);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)22695);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)7069);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)4182);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)35116);
};


void c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)59062);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)3178);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)5583);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)17626);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)58850);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)36886);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)36582);
    assert(p36_time_usec_GET(pack) == (uint32_t)826438336L);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)64190);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)63085);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)44098);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)37382);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)55867);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)28607);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)62708);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)3557);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)15953);
};


void c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t)6267);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t) -11171);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)50);
};


void c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t) -6812);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)237);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t) -8927);
};


void c_TEST_Channel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_y_GET(pack) == (float) -3.1151514E38F);
    assert(p39_param4_GET(pack) == (float) -5.6516295E37F);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p39_x_GET(pack) == (float)1.4855484E38F);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p39_z_GET(pack) == (float)2.5635363E38F);
    assert(p39_param2_GET(pack) == (float) -1.8720346E38F);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p39_param3_GET(pack) == (float) -6.5976904E36F);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)45784);
    assert(p39_param1_GET(pack) == (float) -3.4026229E38F);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)62);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)127);
};


void c_TEST_Channel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)82);
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)43625);
};


void c_TEST_Channel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)33708);
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)97);
};


void c_TEST_Channel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)3319);
};


void c_TEST_Channel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
};


void c_TEST_Channel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)56952);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
};


void c_TEST_Channel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)55);
};


void c_TEST_Channel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)6133);
};


void c_TEST_Channel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_INVALID);
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)180);
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)92);
};


void c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)11);
    assert(p48_altitude_GET(pack) == (int32_t) -439535236);
    assert(p48_time_usec_TRY(ph) == (uint64_t)1171705631826896181L);
    assert(p48_longitude_GET(pack) == (int32_t) -648766259);
    assert(p48_latitude_GET(pack) == (int32_t)1929650765);
};


void c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_time_usec_TRY(ph) == (uint64_t)7945965014788591334L);
    assert(p49_altitude_GET(pack) == (int32_t) -1965921632);
    assert(p49_longitude_GET(pack) == (int32_t) -1644094725);
    assert(p49_latitude_GET(pack) == (int32_t)1075444354);
};


void c_TEST_Channel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)68);
    assert(p50_param_id_LEN(ph) == 14);
    {
        char16_t * exemplary = u"gnwzJuKOlyOzfz";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t) -29043);
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p50_param_value_min_GET(pack) == (float) -3.3122585E38F);
    assert(p50_scale_GET(pack) == (float)3.2832978E38F);
    assert(p50_param_value0_GET(pack) == (float)2.9063934E38F);
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)10);
    assert(p50_param_value_max_GET(pack) == (float) -7.9620186E37F);
};


void c_TEST_Channel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)189);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)51695);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
};


void c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_p2z_GET(pack) == (float) -1.92587E38F);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p54_p2x_GET(pack) == (float) -1.9187896E37F);
    assert(p54_p2y_GET(pack) == (float) -1.5047473E38F);
    assert(p54_p1z_GET(pack) == (float)1.02081875E37F);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)153);
    assert(p54_p1y_GET(pack) == (float)7.597769E37F);
    assert(p54_p1x_GET(pack) == (float) -1.5841163E38F);
};


void c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p1x_GET(pack) == (float)1.9601818E37F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p55_p2z_GET(pack) == (float) -1.717403E38F);
    assert(p55_p2x_GET(pack) == (float) -2.1535074E38F);
    assert(p55_p1z_GET(pack) == (float) -8.39649E37F);
    assert(p55_p2y_GET(pack) == (float)4.9026E37F);
    assert(p55_p1y_GET(pack) == (float) -1.4951904E38F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    assert(p61_time_usec_GET(pack) == (uint64_t)5638221047954871755L);
    {
        float exemplary[] =  {-2.8318418E38F, 3.3205086E37F, 1.4015373E38F, 1.349833E38F, 1.9698682E38F, -1.7776913E38F, -3.199769E37F, 1.5409817E38F, -2.6331293E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_yawspeed_GET(pack) == (float) -1.9852698E38F);
    assert(p61_rollspeed_GET(pack) == (float)3.0542587E38F);
    {
        float exemplary[] =  {2.8786952E38F, 2.6343528E38F, 1.4033174E38F, -2.1673083E38F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_pitchspeed_GET(pack) == (float) -2.2131336E38F);
};


void c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_nav_roll_GET(pack) == (float)1.0540867E38F);
    assert(p62_nav_pitch_GET(pack) == (float) -1.7950261E38F);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t)27699);
    assert(p62_xtrack_error_GET(pack) == (float) -3.3232049E38F);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t) -26296);
    assert(p62_alt_error_GET(pack) == (float) -1.498783E38F);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)59091);
    assert(p62_aspd_error_GET(pack) == (float)5.1164924E37F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {2.3833388E37F, 1.3746884E38F, -1.4937081E38F, -1.4570432E38F, 1.60349E38F, 2.433372E38F, -3.2628846E38F, 1.252493E38F, 1.6574042E38F, -2.1655537E38F, -9.229247E37F, -8.801982E37F, -1.1243764E38F, -2.6780616E38F, -2.1078884E38F, -1.4997842E38F, -1.382466E38F, -1.3883401E38F, 1.2513695E37F, -1.5024032E38F, 1.3006159E37F, 2.207462E38F, 2.3234689E38F, 7.0669376E37F, 1.0887308E37F, -2.1577216E38F, 1.7797693E38F, 3.7498537E37F, 1.4815309E38F, -6.285194E36F, -1.7480086E38F, -8.1606503E37F, 2.6869477E38F, 2.782944E38F, -1.6194425E38F, 3.7219705E37F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_vx_GET(pack) == (float)2.3686826E38F);
    assert(p63_time_usec_GET(pack) == (uint64_t)5746617745371498607L);
    assert(p63_alt_GET(pack) == (int32_t)693072637);
    assert(p63_vz_GET(pack) == (float)1.788118E38F);
    assert(p63_lon_GET(pack) == (int32_t) -47995421);
    assert(p63_vy_GET(pack) == (float)2.0108425E38F);
    assert(p63_lat_GET(pack) == (int32_t)1505905311);
    assert(p63_relative_alt_GET(pack) == (int32_t) -689932774);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION);
    assert(p64_y_GET(pack) == (float) -1.2058434E38F);
    assert(p64_z_GET(pack) == (float) -3.2615638E38F);
    assert(p64_vz_GET(pack) == (float)1.4129925E38F);
    assert(p64_az_GET(pack) == (float) -2.6551828E38F);
    assert(p64_ay_GET(pack) == (float) -3.0639413E38F);
    assert(p64_time_usec_GET(pack) == (uint64_t)1112520916112944277L);
    assert(p64_vx_GET(pack) == (float) -2.3671337E38F);
    assert(p64_ax_GET(pack) == (float) -7.753714E36F);
    assert(p64_vy_GET(pack) == (float)2.2598227E38F);
    {
        float exemplary[] =  {3.2434792E38F, -2.805835E38F, -3.078012E38F, 6.1354355E37F, -2.9334632E38F, 1.4653085E38F, -2.7373886E38F, 2.4631744E36F, -2.532949E38F, 2.6701314E38F, 2.8592472E38F, 1.4941785E38F, 1.3369799E38F, 2.5635597E38F, -4.0605866E37F, -1.2658818E38F, -9.278254E37F, 2.9769243E38F, -1.5026973E38F, -1.0794794E38F, -1.9744715E38F, 1.138759E38F, -2.2117605E38F, -2.2187555E38F, -1.7519745E38F, -2.2417291E38F, -3.6966094E37F, -2.0648824E38F, -2.9834895E38F, -2.5459833E38F, -1.9474214E38F, 2.6842185E38F, -2.78365E38F, 1.6646806E38F, 5.276777E37F, 1.169241E38F, -1.3575242E38F, 1.0404515E38F, -1.770447E38F, 1.2668533E38F, 3.3800362E38F, -2.7985258E37F, -1.9271944E38F, -3.0171914E38F, -8.509902E37F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_x_GET(pack) == (float)1.2210894E38F);
};


void c_TEST_Channel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)20012);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)65196);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)62191);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)7773);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)52921);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)1337);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)20435);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)47474);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)57843);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)30844);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)23048);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)44673);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)3914136336L);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)21873);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)92);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)27636);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)52798);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)18884);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)21524);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)47716);
};


void c_TEST_Channel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)204);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)64785);
};


void c_TEST_Channel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)193);
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)39033);
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)140);
};


void c_TEST_Channel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)19918);
    assert(p69_z_GET(pack) == (int16_t)(int16_t) -23);
    assert(p69_x_GET(pack) == (int16_t)(int16_t)1217);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p69_y_GET(pack) == (int16_t)(int16_t) -12829);
    assert(p69_r_GET(pack) == (int16_t)(int16_t)17789);
};


void c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)14589);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)9736);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)60482);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)31513);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)47535);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)20552);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)132);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)14568);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)50017);
};


void c_TEST_Channel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_param4_GET(pack) == (float) -2.9718294E38F);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)59349);
    assert(p73_param1_GET(pack) == (float)4.3593155E37F);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p73_param2_GET(pack) == (float) -3.2344294E38F);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)188);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p73_y_GET(pack) == (int32_t)1173590344);
    assert(p73_param3_GET(pack) == (float) -6.257039E36F);
    assert(p73_x_GET(pack) == (int32_t) -174836914);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p73_z_GET(pack) == (float) -1.6869311E37F);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_PARACHUTE);
};


void c_TEST_Channel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_heading_GET(pack) == (int16_t)(int16_t) -203);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)1142);
    assert(p74_groundspeed_GET(pack) == (float)2.474558E38F);
    assert(p74_airspeed_GET(pack) == (float) -3.176772E38F);
    assert(p74_climb_GET(pack) == (float)3.0677372E38F);
    assert(p74_alt_GET(pack) == (float)5.2667285E36F);
};


void c_CommunicationChannel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_x_GET(pack) == (int32_t)2117381653);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)180);
    assert(p75_param2_GET(pack) == (float) -8.1507905E37F);
    assert(p75_y_GET(pack) == (int32_t) -146403611);
    assert(p75_param3_GET(pack) == (float)2.6540713E38F);
    assert(p75_param1_GET(pack) == (float)1.202205E38F);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)40);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)157);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_GO_AROUND);
    assert(p75_z_GET(pack) == (float) -1.6204021E38F);
    assert(p75_param4_GET(pack) == (float) -1.8541668E38F);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)115);
};


void c_CommunicationChannel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)70);
    assert(p76_param1_GET(pack) == (float)1.250562E38F);
    assert(p76_param4_GET(pack) == (float) -9.84939E37F);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_SET_REVERSE);
    assert(p76_param2_GET(pack) == (float) -1.517006E38F);
    assert(p76_param7_GET(pack) == (float) -5.4524365E36F);
    assert(p76_param6_GET(pack) == (float)1.724867E38F);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)109);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)143);
    assert(p76_param5_GET(pack) == (float) -8.455628E37F);
    assert(p76_param3_GET(pack) == (float)3.2648745E38F);
};


void c_CommunicationChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO);
    assert(p77_result_param2_TRY(ph) == (int32_t) -1949085523);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)62);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_FAILED);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)132);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)38);
};


void c_CommunicationChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)2534815440L);
    assert(p81_thrust_GET(pack) == (float) -2.0988093E38F);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)120);
    assert(p81_pitch_GET(pack) == (float)2.6586868E38F);
    assert(p81_yaw_GET(pack) == (float) -1.0925911E38F);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p81_roll_GET(pack) == (float) -1.5309617E38F);
};


void c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-2.2303243E38F, 1.6805944E38F, -2.7409383E38F, -2.2500259E38F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)35);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)2176837536L);
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)112);
    assert(p82_body_yaw_rate_GET(pack) == (float)2.1979374E38F);
    assert(p82_body_pitch_rate_GET(pack) == (float) -1.619438E38F);
    assert(p82_thrust_GET(pack) == (float)2.1963473E38F);
    assert(p82_body_roll_rate_GET(pack) == (float)1.3006595E38F);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)125);
};


void c_CommunicationChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)1519205346L);
    {
        float exemplary[] =  {-2.4863413E38F, 1.7783635E38F, 2.8923116E38F, -1.6423628E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p83_thrust_GET(pack) == (float) -1.8285475E38F);
    assert(p83_body_roll_rate_GET(pack) == (float) -1.8181422E37F);
    assert(p83_body_yaw_rate_GET(pack) == (float)6.027938E37F);
    assert(p83_body_pitch_rate_GET(pack) == (float)1.1433144E38F);
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)206);
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_afy_GET(pack) == (float) -1.2510701E38F);
    assert(p84_z_GET(pack) == (float) -1.7125112E38F);
    assert(p84_y_GET(pack) == (float) -2.2763354E38F);
    assert(p84_afx_GET(pack) == (float)7.7909273E37F);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)669295808L);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)96);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)14820);
    assert(p84_vz_GET(pack) == (float) -1.7885516E38F);
    assert(p84_vx_GET(pack) == (float) -1.3978898E38F);
    assert(p84_afz_GET(pack) == (float) -1.3341064E38F);
    assert(p84_x_GET(pack) == (float)1.0798439E38F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p84_yaw_rate_GET(pack) == (float) -3.278856E38F);
    assert(p84_vy_GET(pack) == (float) -1.6858857E38F);
    assert(p84_yaw_GET(pack) == (float) -7.390092E37F);
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_vz_GET(pack) == (float) -4.632008E37F);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)2183036605L);
    assert(p86_vy_GET(pack) == (float)8.364396E37F);
    assert(p86_yaw_GET(pack) == (float)3.177494E38F);
    assert(p86_afz_GET(pack) == (float)2.8863891E38F);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p86_alt_GET(pack) == (float)3.908946E37F);
    assert(p86_lon_int_GET(pack) == (int32_t) -1970480173);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)176);
    assert(p86_afy_GET(pack) == (float) -3.268534E38F);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p86_afx_GET(pack) == (float) -8.6996636E36F);
    assert(p86_lat_int_GET(pack) == (int32_t)2020618561);
    assert(p86_yaw_rate_GET(pack) == (float) -1.326441E38F);
    assert(p86_vx_GET(pack) == (float) -1.5663636E38F);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)57081);
};


void c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_afz_GET(pack) == (float) -2.9126804E38F);
    assert(p87_lat_int_GET(pack) == (int32_t) -974670224);
    assert(p87_vz_GET(pack) == (float)1.988964E38F);
    assert(p87_yaw_GET(pack) == (float) -1.3973286E38F);
    assert(p87_alt_GET(pack) == (float)2.0035461E38F);
    assert(p87_afx_GET(pack) == (float)1.5053882E38F);
    assert(p87_yaw_rate_GET(pack) == (float)2.5311378E38F);
    assert(p87_afy_GET(pack) == (float)4.751996E37F);
    assert(p87_lon_int_GET(pack) == (int32_t)494254310);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT);
    assert(p87_vy_GET(pack) == (float) -1.9743417E38F);
    assert(p87_vx_GET(pack) == (float) -2.540315E38F);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)41911);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)575635555L);
};


void c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_x_GET(pack) == (float)7.443984E37F);
    assert(p89_y_GET(pack) == (float) -4.512567E37F);
    assert(p89_pitch_GET(pack) == (float) -1.893223E38F);
    assert(p89_z_GET(pack) == (float)8.0096103E37F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)1609838539L);
    assert(p89_yaw_GET(pack) == (float) -1.4282341E38F);
    assert(p89_roll_GET(pack) == (float)2.1398119E38F);
};


void c_CommunicationChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_pitchspeed_GET(pack) == (float) -3.398433E38F);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t)21235);
    assert(p90_lat_GET(pack) == (int32_t) -317533746);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t)21881);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t) -26070);
    assert(p90_roll_GET(pack) == (float)1.6986081E38F);
    assert(p90_time_usec_GET(pack) == (uint64_t)1122552055549011998L);
    assert(p90_rollspeed_GET(pack) == (float)1.416251E38F);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t) -28133);
    assert(p90_yawspeed_GET(pack) == (float) -8.0690024E36F);
    assert(p90_pitch_GET(pack) == (float) -2.894036E38F);
    assert(p90_alt_GET(pack) == (int32_t)213257976);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t) -28841);
    assert(p90_yaw_GET(pack) == (float) -3.1588437E38F);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t)19685);
    assert(p90_lon_GET(pack) == (int32_t)1210999742);
};


void c_CommunicationChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_STABILIZE_ARMED);
    assert(p91_aux1_GET(pack) == (float) -5.7751525E37F);
    assert(p91_throttle_GET(pack) == (float) -9.922321E37F);
    assert(p91_roll_ailerons_GET(pack) == (float) -3.7481396E37F);
    assert(p91_aux2_GET(pack) == (float) -2.5787187E38F);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)207);
    assert(p91_time_usec_GET(pack) == (uint64_t)5606425515006191345L);
    assert(p91_aux4_GET(pack) == (float) -2.3528354E38F);
    assert(p91_pitch_elevator_GET(pack) == (float)1.2404847E38F);
    assert(p91_yaw_rudder_GET(pack) == (float)2.84424E38F);
    assert(p91_aux3_GET(pack) == (float)2.0063422E38F);
};


void c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_time_usec_GET(pack) == (uint64_t)5079969898288747337L);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)35204);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)4294);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)16187);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)134);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)34679);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)11724);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)9563);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)40013);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)57146);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)64207);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)7375);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)64543);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)34921);
};


void c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {8.036061E37F, -1.320447E38F, 1.4296999E38F, -2.7671354E38F, -6.1011106E37F, 2.4628303E38F, -1.7844267E38F, 3.2655418E38F, 4.928114E37F, -1.1716481E38F, 2.337485E38F, -5.8273847E37F, -2.3523557E38F, 9.353736E37F, 9.278536E37F, -2.8856119E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_AUTO_DISARMED);
    assert(p93_time_usec_GET(pack) == (uint64_t)4367998290382085827L);
    assert(p93_flags_GET(pack) == (uint64_t)7909580135777052058L);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_comp_m_x_GET(pack) == (float) -1.4183449E38F);
    assert(p100_flow_comp_m_y_GET(pack) == (float) -3.2208583E37F);
    assert(p100_ground_distance_GET(pack) == (float)2.576953E38F);
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t) -19810);
    assert(p100_flow_rate_y_TRY(ph) == (float)1.6789105E38F);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t) -9872);
    assert(p100_flow_rate_x_TRY(ph) == (float) -2.5508353E38F);
    assert(p100_time_usec_GET(pack) == (uint64_t)5921680326004748660L);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)249);
};


void c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_yaw_GET(pack) == (float)2.2818116E38F);
    assert(p101_z_GET(pack) == (float) -1.5236849E38F);
    assert(p101_roll_GET(pack) == (float) -3.060404E38F);
    assert(p101_x_GET(pack) == (float) -1.0687994E38F);
    assert(p101_y_GET(pack) == (float)9.431476E37F);
    assert(p101_usec_GET(pack) == (uint64_t)4179899387342405931L);
    assert(p101_pitch_GET(pack) == (float) -1.6527336E38F);
};


void c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_usec_GET(pack) == (uint64_t)2711917128324492808L);
    assert(p102_yaw_GET(pack) == (float) -2.4317139E38F);
    assert(p102_pitch_GET(pack) == (float) -2.6896386E38F);
    assert(p102_z_GET(pack) == (float) -2.4419198E38F);
    assert(p102_x_GET(pack) == (float)1.715313E38F);
    assert(p102_y_GET(pack) == (float)2.028872E38F);
    assert(p102_roll_GET(pack) == (float)2.273977E38F);
};


void c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_y_GET(pack) == (float)8.200668E37F);
    assert(p103_z_GET(pack) == (float) -4.447912E37F);
    assert(p103_usec_GET(pack) == (uint64_t)7769795264581813088L);
    assert(p103_x_GET(pack) == (float)1.1549646E38F);
};


void c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_pitch_GET(pack) == (float)6.0693894E36F);
    assert(p104_x_GET(pack) == (float) -6.1707994E37F);
    assert(p104_z_GET(pack) == (float)6.120884E37F);
    assert(p104_y_GET(pack) == (float) -5.677991E37F);
    assert(p104_usec_GET(pack) == (uint64_t)3539996105022319989L);
    assert(p104_yaw_GET(pack) == (float)4.287942E37F);
    assert(p104_roll_GET(pack) == (float) -2.3967188E38F);
};


void c_CommunicationChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_diff_pressure_GET(pack) == (float)1.8220478E38F);
    assert(p105_zmag_GET(pack) == (float)4.501989E37F);
    assert(p105_yacc_GET(pack) == (float)5.063806E37F);
    assert(p105_abs_pressure_GET(pack) == (float)1.4893191E38F);
    assert(p105_xacc_GET(pack) == (float) -3.2456219E37F);
    assert(p105_xgyro_GET(pack) == (float)5.0066195E37F);
    assert(p105_zgyro_GET(pack) == (float) -3.282926E38F);
    assert(p105_zacc_GET(pack) == (float)1.4645726E38F);
    assert(p105_xmag_GET(pack) == (float) -1.3381293E38F);
    assert(p105_ymag_GET(pack) == (float) -4.5442947E37F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)4928);
    assert(p105_temperature_GET(pack) == (float)3.2901676E37F);
    assert(p105_pressure_alt_GET(pack) == (float)3.348652E38F);
    assert(p105_time_usec_GET(pack) == (uint64_t)3255498571451490362L);
    assert(p105_ygyro_GET(pack) == (float) -3.0244103E38F);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p106_distance_GET(pack) == (float)6.2195025E37F);
    assert(p106_integrated_x_GET(pack) == (float)1.6637992E38F);
    assert(p106_time_usec_GET(pack) == (uint64_t)9011521324116247634L);
    assert(p106_integrated_y_GET(pack) == (float) -2.87381E37F);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)4056833644L);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t)15457);
    assert(p106_integrated_xgyro_GET(pack) == (float)1.2280298E38F);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)2815419925L);
    assert(p106_integrated_ygyro_GET(pack) == (float) -2.233021E38F);
    assert(p106_integrated_zgyro_GET(pack) == (float) -2.633513E38F);
};


void c_CommunicationChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_xmag_GET(pack) == (float) -2.3477372E38F);
    assert(p107_xgyro_GET(pack) == (float) -2.416658E38F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)1764122671L);
    assert(p107_ymag_GET(pack) == (float) -1.3913354E38F);
    assert(p107_temperature_GET(pack) == (float)2.3617793E38F);
    assert(p107_abs_pressure_GET(pack) == (float) -3.1643293E38F);
    assert(p107_diff_pressure_GET(pack) == (float) -1.5038215E38F);
    assert(p107_time_usec_GET(pack) == (uint64_t)5942525826219682035L);
    assert(p107_zgyro_GET(pack) == (float)2.7301812E37F);
    assert(p107_zacc_GET(pack) == (float) -2.4118485E38F);
    assert(p107_zmag_GET(pack) == (float)8.422731E37F);
    assert(p107_xacc_GET(pack) == (float)2.8090121E38F);
    assert(p107_yacc_GET(pack) == (float)2.882824E38F);
    assert(p107_ygyro_GET(pack) == (float)1.2787965E38F);
    assert(p107_pressure_alt_GET(pack) == (float)2.6091517E37F);
};


void c_CommunicationChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_roll_GET(pack) == (float) -2.3902777E37F);
    assert(p108_q3_GET(pack) == (float)7.1240934E37F);
    assert(p108_lon_GET(pack) == (float)6.7853843E37F);
    assert(p108_q4_GET(pack) == (float)2.0574332E38F);
    assert(p108_std_dev_vert_GET(pack) == (float)1.4130506E38F);
    assert(p108_lat_GET(pack) == (float) -2.0244494E38F);
    assert(p108_q1_GET(pack) == (float)2.0695345E38F);
    assert(p108_ve_GET(pack) == (float) -4.4151074E37F);
    assert(p108_std_dev_horz_GET(pack) == (float)1.7523162E38F);
    assert(p108_yacc_GET(pack) == (float)1.4486945E38F);
    assert(p108_alt_GET(pack) == (float)3.490629E37F);
    assert(p108_xacc_GET(pack) == (float)2.2422512E38F);
    assert(p108_vn_GET(pack) == (float) -1.6539874E38F);
    assert(p108_q2_GET(pack) == (float)3.0114851E38F);
    assert(p108_zacc_GET(pack) == (float) -1.8599929E38F);
    assert(p108_pitch_GET(pack) == (float)1.2409822E38F);
    assert(p108_vd_GET(pack) == (float) -1.6448991E38F);
    assert(p108_xgyro_GET(pack) == (float)1.2881845E36F);
    assert(p108_yaw_GET(pack) == (float) -2.0424504E38F);
    assert(p108_ygyro_GET(pack) == (float)6.5132603E37F);
    assert(p108_zgyro_GET(pack) == (float) -3.251642E38F);
};


void c_CommunicationChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)138);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)209);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)63417);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)12081);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)103);
};


void c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)181);
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)101);
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)105);
    {
        uint8_t exemplary[] =  {(uint8_t)27, (uint8_t)128, (uint8_t)6, (uint8_t)44, (uint8_t)200, (uint8_t)59, (uint8_t)201, (uint8_t)121, (uint8_t)46, (uint8_t)173, (uint8_t)60, (uint8_t)200, (uint8_t)177, (uint8_t)202, (uint8_t)232, (uint8_t)214, (uint8_t)30, (uint8_t)77, (uint8_t)133, (uint8_t)125, (uint8_t)225, (uint8_t)232, (uint8_t)0, (uint8_t)179, (uint8_t)178, (uint8_t)79, (uint8_t)183, (uint8_t)124, (uint8_t)218, (uint8_t)57, (uint8_t)250, (uint8_t)190, (uint8_t)202, (uint8_t)48, (uint8_t)178, (uint8_t)17, (uint8_t)56, (uint8_t)72, (uint8_t)9, (uint8_t)249, (uint8_t)173, (uint8_t)39, (uint8_t)134, (uint8_t)45, (uint8_t)255, (uint8_t)100, (uint8_t)254, (uint8_t)66, (uint8_t)105, (uint8_t)80, (uint8_t)112, (uint8_t)155, (uint8_t)79, (uint8_t)114, (uint8_t)108, (uint8_t)3, (uint8_t)165, (uint8_t)179, (uint8_t)73, (uint8_t)136, (uint8_t)220, (uint8_t)212, (uint8_t)28, (uint8_t)234, (uint8_t)147, (uint8_t)208, (uint8_t)190, (uint8_t)39, (uint8_t)158, (uint8_t)42, (uint8_t)157, (uint8_t)130, (uint8_t)184, (uint8_t)120, (uint8_t)104, (uint8_t)125, (uint8_t)122, (uint8_t)249, (uint8_t)14, (uint8_t)121, (uint8_t)222, (uint8_t)83, (uint8_t)229, (uint8_t)220, (uint8_t)146, (uint8_t)76, (uint8_t)86, (uint8_t)174, (uint8_t)144, (uint8_t)14, (uint8_t)78, (uint8_t)249, (uint8_t)247, (uint8_t)227, (uint8_t)8, (uint8_t)184, (uint8_t)131, (uint8_t)234, (uint8_t)49, (uint8_t)208, (uint8_t)171, (uint8_t)251, (uint8_t)42, (uint8_t)141, (uint8_t)232, (uint8_t)89, (uint8_t)45, (uint8_t)49, (uint8_t)5, (uint8_t)132, (uint8_t)2, (uint8_t)128, (uint8_t)84, (uint8_t)118, (uint8_t)252, (uint8_t)223, (uint8_t)249, (uint8_t)59, (uint8_t)136, (uint8_t)203, (uint8_t)52, (uint8_t)47, (uint8_t)173, (uint8_t)131, (uint8_t)178, (uint8_t)72, (uint8_t)4, (uint8_t)34, (uint8_t)93, (uint8_t)38, (uint8_t)233, (uint8_t)53, (uint8_t)91, (uint8_t)100, (uint8_t)244, (uint8_t)248, (uint8_t)8, (uint8_t)22, (uint8_t)6, (uint8_t)141, (uint8_t)232, (uint8_t)27, (uint8_t)24, (uint8_t)175, (uint8_t)11, (uint8_t)3, (uint8_t)44, (uint8_t)197, (uint8_t)217, (uint8_t)21, (uint8_t)106, (uint8_t)202, (uint8_t)24, (uint8_t)150, (uint8_t)255, (uint8_t)4, (uint8_t)50, (uint8_t)192, (uint8_t)174, (uint8_t)34, (uint8_t)182, (uint8_t)253, (uint8_t)173, (uint8_t)168, (uint8_t)232, (uint8_t)21, (uint8_t)92, (uint8_t)154, (uint8_t)105, (uint8_t)141, (uint8_t)227, (uint8_t)204, (uint8_t)210, (uint8_t)30, (uint8_t)81, (uint8_t)225, (uint8_t)112, (uint8_t)175, (uint8_t)115, (uint8_t)184, (uint8_t)219, (uint8_t)218, (uint8_t)82, (uint8_t)149, (uint8_t)214, (uint8_t)126, (uint8_t)112, (uint8_t)55, (uint8_t)214, (uint8_t)237, (uint8_t)71, (uint8_t)26, (uint8_t)214, (uint8_t)13, (uint8_t)255, (uint8_t)120, (uint8_t)199, (uint8_t)1, (uint8_t)8, (uint8_t)116, (uint8_t)57, (uint8_t)190, (uint8_t)233, (uint8_t)73, (uint8_t)105, (uint8_t)33, (uint8_t)224, (uint8_t)61, (uint8_t)62, (uint8_t)205, (uint8_t)1, (uint8_t)202, (uint8_t)235, (uint8_t)207, (uint8_t)56, (uint8_t)49, (uint8_t)169, (uint8_t)49, (uint8_t)193, (uint8_t)103, (uint8_t)116, (uint8_t)222, (uint8_t)150, (uint8_t)20, (uint8_t)240, (uint8_t)113, (uint8_t)78, (uint8_t)131, (uint8_t)126, (uint8_t)84, (uint8_t)183, (uint8_t)24, (uint8_t)17, (uint8_t)214, (uint8_t)103, (uint8_t)153, (uint8_t)38, (uint8_t)158, (uint8_t)111, (uint8_t)110, (uint8_t)101, (uint8_t)82, (uint8_t)165, (uint8_t)13, (uint8_t)220, (uint8_t)3, (uint8_t)62, (uint8_t)177, (uint8_t)212, (uint8_t)255, (uint8_t)132} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_ts1_GET(pack) == (int64_t)8779977823302329070L);
    assert(p111_tc1_GET(pack) == (int64_t) -2998649128465505638L);
};


void c_CommunicationChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_seq_GET(pack) == (uint32_t)3736564981L);
    assert(p112_time_usec_GET(pack) == (uint64_t)6730606133999373695L);
};


void c_CommunicationChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)6569);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t)11990);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)76);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t) -10949);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)193);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t) -6879);
    assert(p113_time_usec_GET(pack) == (uint64_t)238102105641123289L);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)20331);
    assert(p113_lat_GET(pack) == (int32_t) -1689269909);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)64929);
    assert(p113_alt_GET(pack) == (int32_t) -1553749874);
    assert(p113_lon_GET(pack) == (int32_t) -1615754538);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)48780);
};


void c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_time_usec_GET(pack) == (uint64_t)1944861652753895095L);
    assert(p114_integrated_ygyro_GET(pack) == (float)1.7690458E38F);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)109);
    assert(p114_integrated_y_GET(pack) == (float) -4.8975204E37F);
    assert(p114_integrated_x_GET(pack) == (float) -2.1167248E38F);
    assert(p114_distance_GET(pack) == (float) -3.8545467E37F);
    assert(p114_integrated_zgyro_GET(pack) == (float) -1.3715837E38F);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)21);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t)20952);
    assert(p114_integrated_xgyro_GET(pack) == (float)5.567848E37F);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)2844480677L);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)4253019998L);
};


void c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t)30197);
    assert(p115_alt_GET(pack) == (int32_t)1462838358);
    {
        float exemplary[] =  {-1.6560066E38F, -2.0447293E38F, 2.2073389E38F, 2.9812132E38F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)13785);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t) -7118);
    assert(p115_lon_GET(pack) == (int32_t)1514642759);
    assert(p115_pitchspeed_GET(pack) == (float) -2.1211555E38F);
    assert(p115_time_usec_GET(pack) == (uint64_t)5428906822465152735L);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t) -2239);
    assert(p115_yawspeed_GET(pack) == (float)2.2699738E38F);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t) -8933);
    assert(p115_lat_GET(pack) == (int32_t) -311211727);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)62338);
    assert(p115_rollspeed_GET(pack) == (float) -3.771207E37F);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t) -18673);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t) -25268);
};


void c_CommunicationChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)3036171229L);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t)32005);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t) -3308);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t) -20336);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t)6895);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t)14711);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t) -23237);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t) -16669);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t)160);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t) -25655);
};


void c_CommunicationChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)16818);
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)41069);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)30);
};


void c_CommunicationChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)33440);
    assert(p118_size_GET(pack) == (uint32_t)1453734412L);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)49209);
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)18142);
    assert(p118_time_utc_GET(pack) == (uint32_t)741824610L);
};


void c_CommunicationChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p119_count_GET(pack) == (uint32_t)1843728L);
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)24254);
    assert(p119_ofs_GET(pack) == (uint32_t)1647046926L);
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)147);
};


void c_CommunicationChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p120_ofs_GET(pack) == (uint32_t)3906611811L);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)53697);
    {
        uint8_t exemplary[] =  {(uint8_t)28, (uint8_t)239, (uint8_t)123, (uint8_t)0, (uint8_t)24, (uint8_t)81, (uint8_t)59, (uint8_t)225, (uint8_t)1, (uint8_t)155, (uint8_t)153, (uint8_t)71, (uint8_t)136, (uint8_t)38, (uint8_t)76, (uint8_t)201, (uint8_t)209, (uint8_t)184, (uint8_t)66, (uint8_t)88, (uint8_t)9, (uint8_t)128, (uint8_t)219, (uint8_t)32, (uint8_t)20, (uint8_t)246, (uint8_t)72, (uint8_t)137, (uint8_t)39, (uint8_t)44, (uint8_t)114, (uint8_t)41, (uint8_t)160, (uint8_t)146, (uint8_t)52, (uint8_t)167, (uint8_t)58, (uint8_t)216, (uint8_t)199, (uint8_t)52, (uint8_t)31, (uint8_t)102, (uint8_t)5, (uint8_t)231, (uint8_t)5, (uint8_t)2, (uint8_t)183, (uint8_t)63, (uint8_t)41, (uint8_t)93, (uint8_t)250, (uint8_t)253, (uint8_t)146, (uint8_t)83, (uint8_t)182, (uint8_t)203, (uint8_t)197, (uint8_t)222, (uint8_t)195, (uint8_t)92, (uint8_t)123, (uint8_t)191, (uint8_t)194, (uint8_t)3, (uint8_t)35, (uint8_t)178, (uint8_t)107, (uint8_t)156, (uint8_t)20, (uint8_t)7, (uint8_t)46, (uint8_t)201, (uint8_t)38, (uint8_t)249, (uint8_t)68, (uint8_t)84, (uint8_t)135, (uint8_t)164, (uint8_t)15, (uint8_t)72, (uint8_t)196, (uint8_t)211, (uint8_t)140, (uint8_t)12, (uint8_t)98, (uint8_t)87, (uint8_t)125, (uint8_t)40, (uint8_t)133, (uint8_t)58} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)21);
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)73);
};


void c_CommunicationChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)11);
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)12);
};


void c_CommunicationChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)115);
    {
        uint8_t exemplary[] =  {(uint8_t)122, (uint8_t)108, (uint8_t)231, (uint8_t)87, (uint8_t)232, (uint8_t)252, (uint8_t)186, (uint8_t)204, (uint8_t)94, (uint8_t)244, (uint8_t)48, (uint8_t)7, (uint8_t)151, (uint8_t)37, (uint8_t)117, (uint8_t)129, (uint8_t)235, (uint8_t)210, (uint8_t)189, (uint8_t)136, (uint8_t)154, (uint8_t)65, (uint8_t)140, (uint8_t)92, (uint8_t)68, (uint8_t)162, (uint8_t)166, (uint8_t)39, (uint8_t)93, (uint8_t)200, (uint8_t)208, (uint8_t)206, (uint8_t)220, (uint8_t)177, (uint8_t)110, (uint8_t)255, (uint8_t)18, (uint8_t)197, (uint8_t)113, (uint8_t)146, (uint8_t)220, (uint8_t)221, (uint8_t)109, (uint8_t)34, (uint8_t)232, (uint8_t)173, (uint8_t)37, (uint8_t)63, (uint8_t)171, (uint8_t)211, (uint8_t)77, (uint8_t)162, (uint8_t)16, (uint8_t)21, (uint8_t)232, (uint8_t)143, (uint8_t)216, (uint8_t)172, (uint8_t)174, (uint8_t)151, (uint8_t)49, (uint8_t)140, (uint8_t)209, (uint8_t)117, (uint8_t)187, (uint8_t)16, (uint8_t)69, (uint8_t)206, (uint8_t)140, (uint8_t)89, (uint8_t)142, (uint8_t)190, (uint8_t)120, (uint8_t)241, (uint8_t)143, (uint8_t)120, (uint8_t)117, (uint8_t)236, (uint8_t)197, (uint8_t)135, (uint8_t)227, (uint8_t)102, (uint8_t)33, (uint8_t)58, (uint8_t)53, (uint8_t)168, (uint8_t)167, (uint8_t)26, (uint8_t)192, (uint8_t)162, (uint8_t)25, (uint8_t)2, (uint8_t)85, (uint8_t)188, (uint8_t)226, (uint8_t)186, (uint8_t)61, (uint8_t)174, (uint8_t)20, (uint8_t)94, (uint8_t)37, (uint8_t)42, (uint8_t)191, (uint8_t)163, (uint8_t)192, (uint8_t)117, (uint8_t)219, (uint8_t)67, (uint8_t)45, (uint8_t)158} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)40);
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)82);
};


void c_CommunicationChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_lon_GET(pack) == (int32_t) -679625659);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)48738);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)45547);
    assert(p124_time_usec_GET(pack) == (uint64_t)3667789353428624123L);
    assert(p124_dgps_age_GET(pack) == (uint32_t)1947124945L);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)107);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)57590);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)19649);
    assert(p124_alt_GET(pack) == (int32_t) -661005024);
    assert(p124_lat_GET(pack) == (int32_t) -315932364);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)100);
};


void c_CommunicationChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_flags_GET(pack) == e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT);
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)60228);
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)21941);
};


void c_CommunicationChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    assert(p126_flags_GET(pack) == e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND);
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1);
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)163);
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)15707);
    assert(p126_baudrate_GET(pack) == (uint32_t)1153411085L);
    {
        uint8_t exemplary[] =  {(uint8_t)143, (uint8_t)85, (uint8_t)98, (uint8_t)249, (uint8_t)7, (uint8_t)11, (uint8_t)3, (uint8_t)34, (uint8_t)235, (uint8_t)34, (uint8_t)180, (uint8_t)185, (uint8_t)64, (uint8_t)66, (uint8_t)239, (uint8_t)178, (uint8_t)21, (uint8_t)14, (uint8_t)110, (uint8_t)194, (uint8_t)143, (uint8_t)105, (uint8_t)191, (uint8_t)160, (uint8_t)186, (uint8_t)35, (uint8_t)117, (uint8_t)100, (uint8_t)75, (uint8_t)30, (uint8_t)171, (uint8_t)29, (uint8_t)223, (uint8_t)193, (uint8_t)58, (uint8_t)162, (uint8_t)119, (uint8_t)109, (uint8_t)233, (uint8_t)205, (uint8_t)98, (uint8_t)244, (uint8_t)84, (uint8_t)153, (uint8_t)4, (uint8_t)237, (uint8_t)117, (uint8_t)117, (uint8_t)137, (uint8_t)155, (uint8_t)142, (uint8_t)100, (uint8_t)108, (uint8_t)44, (uint8_t)178, (uint8_t)159, (uint8_t)249, (uint8_t)193, (uint8_t)16, (uint8_t)11, (uint8_t)47, (uint8_t)163, (uint8_t)205, (uint8_t)209, (uint8_t)248, (uint8_t)210, (uint8_t)178, (uint8_t)182, (uint8_t)21, (uint8_t)204} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)156);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)1350706618L);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)18);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t)512175018);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t) -781356605);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t) -538181780);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)231);
    assert(p127_accuracy_GET(pack) == (uint32_t)1390611867L);
    assert(p127_tow_GET(pack) == (uint32_t)2487833546L);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)176);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)216);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)24993);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t)538821739);
};


void c_CommunicationChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)40);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t) -563723551);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)136);
    assert(p128_tow_GET(pack) == (uint32_t)3476305333L);
    assert(p128_accuracy_GET(pack) == (uint32_t)3604752308L);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)967870776L);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t) -1559378638);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)26);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t) -1410993893);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)18687);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t)1029384068);
};


void c_CommunicationChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t) -18507);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t) -30042);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t)6292);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)299080472L);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)4995);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t)31784);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t)14737);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t)22267);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t) -9544);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t) -12965);
};


void c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)208);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)45197);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)36519);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)226);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)16481);
    assert(p130_size_GET(pack) == (uint32_t)1681865327L);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)194);
};


void c_CommunicationChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)54, (uint8_t)86, (uint8_t)64, (uint8_t)143, (uint8_t)138, (uint8_t)168, (uint8_t)178, (uint8_t)171, (uint8_t)154, (uint8_t)13, (uint8_t)140, (uint8_t)7, (uint8_t)221, (uint8_t)31, (uint8_t)108, (uint8_t)173, (uint8_t)195, (uint8_t)157, (uint8_t)138, (uint8_t)113, (uint8_t)72, (uint8_t)82, (uint8_t)49, (uint8_t)117, (uint8_t)243, (uint8_t)181, (uint8_t)97, (uint8_t)56, (uint8_t)7, (uint8_t)169, (uint8_t)202, (uint8_t)40, (uint8_t)146, (uint8_t)198, (uint8_t)84, (uint8_t)187, (uint8_t)217, (uint8_t)137, (uint8_t)147, (uint8_t)133, (uint8_t)113, (uint8_t)106, (uint8_t)42, (uint8_t)239, (uint8_t)244, (uint8_t)30, (uint8_t)214, (uint8_t)108, (uint8_t)183, (uint8_t)148, (uint8_t)100, (uint8_t)35, (uint8_t)73, (uint8_t)49, (uint8_t)81, (uint8_t)115, (uint8_t)139, (uint8_t)75, (uint8_t)104, (uint8_t)107, (uint8_t)82, (uint8_t)240, (uint8_t)155, (uint8_t)183, (uint8_t)51, (uint8_t)208, (uint8_t)145, (uint8_t)73, (uint8_t)195, (uint8_t)62, (uint8_t)127, (uint8_t)224, (uint8_t)48, (uint8_t)216, (uint8_t)65, (uint8_t)29, (uint8_t)223, (uint8_t)110, (uint8_t)76, (uint8_t)245, (uint8_t)164, (uint8_t)83, (uint8_t)18, (uint8_t)105, (uint8_t)44, (uint8_t)217, (uint8_t)97, (uint8_t)167, (uint8_t)92, (uint8_t)23, (uint8_t)220, (uint8_t)128, (uint8_t)181, (uint8_t)154, (uint8_t)44, (uint8_t)117, (uint8_t)14, (uint8_t)149, (uint8_t)49, (uint8_t)78, (uint8_t)197, (uint8_t)189, (uint8_t)216, (uint8_t)39, (uint8_t)219, (uint8_t)180, (uint8_t)112, (uint8_t)182, (uint8_t)87, (uint8_t)7, (uint8_t)125, (uint8_t)239, (uint8_t)172, (uint8_t)104, (uint8_t)71, (uint8_t)165, (uint8_t)236, (uint8_t)135, (uint8_t)168, (uint8_t)78, (uint8_t)151, (uint8_t)173, (uint8_t)227, (uint8_t)245, (uint8_t)132, (uint8_t)146, (uint8_t)183, (uint8_t)5, (uint8_t)33, (uint8_t)66, (uint8_t)221, (uint8_t)151, (uint8_t)118, (uint8_t)60, (uint8_t)165, (uint8_t)143, (uint8_t)40, (uint8_t)44, (uint8_t)163, (uint8_t)24, (uint8_t)70, (uint8_t)208, (uint8_t)33, (uint8_t)73, (uint8_t)232, (uint8_t)60, (uint8_t)20, (uint8_t)243, (uint8_t)206, (uint8_t)186, (uint8_t)92, (uint8_t)22, (uint8_t)210, (uint8_t)28, (uint8_t)88, (uint8_t)52, (uint8_t)82, (uint8_t)108, (uint8_t)28, (uint8_t)34, (uint8_t)101, (uint8_t)175, (uint8_t)170, (uint8_t)138, (uint8_t)118, (uint8_t)109, (uint8_t)4, (uint8_t)52, (uint8_t)3, (uint8_t)127, (uint8_t)68, (uint8_t)226, (uint8_t)124, (uint8_t)193, (uint8_t)146, (uint8_t)21, (uint8_t)41, (uint8_t)168, (uint8_t)124, (uint8_t)209, (uint8_t)206, (uint8_t)133, (uint8_t)239, (uint8_t)191, (uint8_t)70, (uint8_t)213, (uint8_t)198, (uint8_t)105, (uint8_t)252, (uint8_t)16, (uint8_t)167, (uint8_t)176, (uint8_t)11, (uint8_t)43, (uint8_t)159, (uint8_t)21, (uint8_t)127, (uint8_t)35, (uint8_t)19, (uint8_t)89, (uint8_t)102, (uint8_t)191, (uint8_t)94, (uint8_t)208, (uint8_t)243, (uint8_t)62, (uint8_t)100, (uint8_t)141, (uint8_t)0, (uint8_t)132, (uint8_t)18, (uint8_t)220, (uint8_t)180, (uint8_t)170, (uint8_t)35, (uint8_t)12, (uint8_t)50, (uint8_t)208, (uint8_t)186, (uint8_t)167, (uint8_t)29, (uint8_t)32, (uint8_t)116, (uint8_t)248, (uint8_t)219, (uint8_t)100, (uint8_t)181, (uint8_t)50, (uint8_t)209, (uint8_t)52, (uint8_t)72, (uint8_t)108, (uint8_t)247, (uint8_t)239, (uint8_t)1, (uint8_t)236, (uint8_t)134, (uint8_t)152, (uint8_t)72, (uint8_t)98, (uint8_t)191, (uint8_t)7, (uint8_t)44, (uint8_t)185, (uint8_t)27, (uint8_t)74, (uint8_t)91, (uint8_t)198, (uint8_t)30, (uint8_t)102, (uint8_t)132, (uint8_t)206, (uint8_t)229} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)56336);
};


void c_CommunicationChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)1292646927L);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)58068);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)6641);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)79);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)11992);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)19);
};


void c_CommunicationChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_lat_GET(pack) == (int32_t)1254489976);
    assert(p133_mask_GET(pack) == (uint64_t)1680060074204372468L);
    assert(p133_lon_GET(pack) == (int32_t) -241388805);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)30851);
};


void c_CommunicationChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)18280);
    assert(p134_lon_GET(pack) == (int32_t) -1887547875);
    {
        int16_t exemplary[] =  {(int16_t) -2281, (int16_t) -22745, (int16_t)15663, (int16_t) -27843, (int16_t) -2280, (int16_t)32376, (int16_t)24789, (int16_t)12137, (int16_t)19510, (int16_t) -20119, (int16_t) -16228, (int16_t) -11941, (int16_t) -13998, (int16_t) -18204, (int16_t)28976, (int16_t)27832} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_lat_GET(pack) == (int32_t)1430054319);
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)92);
};


void c_CommunicationChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lon_GET(pack) == (int32_t)653779611);
    assert(p135_lat_GET(pack) == (int32_t)1463617961);
};


void c_CommunicationChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_lat_GET(pack) == (int32_t) -872154463);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)58301);
    assert(p136_terrain_height_GET(pack) == (float) -1.3678118E38F);
    assert(p136_current_height_GET(pack) == (float) -2.0599026E38F);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)13260);
    assert(p136_lon_GET(pack) == (int32_t) -1198683054);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)64532);
};


void c_CommunicationChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_press_abs_GET(pack) == (float)1.7846397E38F);
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)3781506397L);
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t) -32510);
    assert(p137_press_diff_GET(pack) == (float) -3.0184525E38F);
};


void c_CommunicationChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    assert(p138_time_usec_GET(pack) == (uint64_t)924248566902828115L);
    assert(p138_x_GET(pack) == (float)3.8712825E37F);
    assert(p138_z_GET(pack) == (float)7.1914675E37F);
    assert(p138_y_GET(pack) == (float)9.496752E37F);
    {
        float exemplary[] =  {-1.5407952E38F, 2.5466934E38F, 2.0858081E38F, -2.6889853E38F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_time_usec_GET(pack) == (uint64_t)5260666375115683793L);
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)30);
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)229);
    {
        float exemplary[] =  {-4.5218E37F, -3.2530721E38F, -2.097604E38F, -9.317569E36F, 2.551852E38F, 2.3549924E38F, -2.05851E38F, -2.2374964E38F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)164);
    assert(p140_time_usec_GET(pack) == (uint64_t)6412352912868954294L);
    {
        float exemplary[] =  {-1.1421533E38F, -2.6105454E37F, -1.648163E38F, 2.3211461E38F, -1.6707519E38F, -1.0223979E38F, -2.1904262E38F, -2.7077402E37F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_altitude_monotonic_GET(pack) == (float)3.1094185E38F);
    assert(p141_altitude_amsl_GET(pack) == (float) -1.1498933E38F);
    assert(p141_altitude_relative_GET(pack) == (float)1.0486672E38F);
    assert(p141_altitude_local_GET(pack) == (float) -1.7951156E38F);
    assert(p141_altitude_terrain_GET(pack) == (float)2.8901693E38F);
    assert(p141_bottom_clearance_GET(pack) == (float)9.393969E37F);
    assert(p141_time_usec_GET(pack) == (uint64_t)5814206301282074872L);
};


void c_CommunicationChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)70);
    {
        uint8_t exemplary[] =  {(uint8_t)203, (uint8_t)10, (uint8_t)173, (uint8_t)235, (uint8_t)179, (uint8_t)197, (uint8_t)115, (uint8_t)101, (uint8_t)192, (uint8_t)210, (uint8_t)196, (uint8_t)101, (uint8_t)178, (uint8_t)74, (uint8_t)74, (uint8_t)111, (uint8_t)245, (uint8_t)93, (uint8_t)222, (uint8_t)9, (uint8_t)169, (uint8_t)160, (uint8_t)143, (uint8_t)241, (uint8_t)156, (uint8_t)5, (uint8_t)75, (uint8_t)167, (uint8_t)234, (uint8_t)1, (uint8_t)89, (uint8_t)18, (uint8_t)12, (uint8_t)229, (uint8_t)54, (uint8_t)70, (uint8_t)130, (uint8_t)87, (uint8_t)216, (uint8_t)16, (uint8_t)163, (uint8_t)57, (uint8_t)14, (uint8_t)25, (uint8_t)174, (uint8_t)181, (uint8_t)99, (uint8_t)160, (uint8_t)193, (uint8_t)105, (uint8_t)106, (uint8_t)77, (uint8_t)236, (uint8_t)52, (uint8_t)192, (uint8_t)6, (uint8_t)76, (uint8_t)251, (uint8_t)87, (uint8_t)184, (uint8_t)102, (uint8_t)16, (uint8_t)82, (uint8_t)229, (uint8_t)132, (uint8_t)233, (uint8_t)245, (uint8_t)233, (uint8_t)89, (uint8_t)24, (uint8_t)137, (uint8_t)57, (uint8_t)112, (uint8_t)169, (uint8_t)91, (uint8_t)155, (uint8_t)89, (uint8_t)138, (uint8_t)198, (uint8_t)107, (uint8_t)160, (uint8_t)14, (uint8_t)235, (uint8_t)134, (uint8_t)115, (uint8_t)185, (uint8_t)8, (uint8_t)117, (uint8_t)177, (uint8_t)150, (uint8_t)151, (uint8_t)87, (uint8_t)75, (uint8_t)251, (uint8_t)30, (uint8_t)27, (uint8_t)185, (uint8_t)147, (uint8_t)225, (uint8_t)8, (uint8_t)228, (uint8_t)164, (uint8_t)143, (uint8_t)7, (uint8_t)142, (uint8_t)123, (uint8_t)2, (uint8_t)167, (uint8_t)129, (uint8_t)141, (uint8_t)94, (uint8_t)33, (uint8_t)166, (uint8_t)54, (uint8_t)115, (uint8_t)252, (uint8_t)187, (uint8_t)211, (uint8_t)76, (uint8_t)125} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)175);
    {
        uint8_t exemplary[] =  {(uint8_t)9, (uint8_t)233, (uint8_t)183, (uint8_t)185, (uint8_t)48, (uint8_t)221, (uint8_t)211, (uint8_t)72, (uint8_t)20, (uint8_t)209, (uint8_t)237, (uint8_t)87, (uint8_t)29, (uint8_t)79, (uint8_t)115, (uint8_t)214, (uint8_t)124, (uint8_t)160, (uint8_t)245, (uint8_t)102, (uint8_t)25, (uint8_t)77, (uint8_t)123, (uint8_t)218, (uint8_t)254, (uint8_t)211, (uint8_t)151, (uint8_t)117, (uint8_t)99, (uint8_t)25, (uint8_t)221, (uint8_t)219, (uint8_t)233, (uint8_t)88, (uint8_t)104, (uint8_t)223, (uint8_t)2, (uint8_t)94, (uint8_t)168, (uint8_t)3, (uint8_t)39, (uint8_t)102, (uint8_t)60, (uint8_t)98, (uint8_t)182, (uint8_t)89, (uint8_t)223, (uint8_t)248, (uint8_t)0, (uint8_t)18, (uint8_t)128, (uint8_t)38, (uint8_t)170, (uint8_t)174, (uint8_t)20, (uint8_t)33, (uint8_t)110, (uint8_t)73, (uint8_t)193, (uint8_t)245, (uint8_t)10, (uint8_t)126, (uint8_t)42, (uint8_t)91, (uint8_t)255, (uint8_t)73, (uint8_t)125, (uint8_t)208, (uint8_t)63, (uint8_t)116, (uint8_t)161, (uint8_t)47, (uint8_t)178, (uint8_t)38, (uint8_t)37, (uint8_t)92, (uint8_t)90, (uint8_t)193, (uint8_t)18, (uint8_t)53, (uint8_t)251, (uint8_t)240, (uint8_t)137, (uint8_t)41, (uint8_t)16, (uint8_t)108, (uint8_t)66, (uint8_t)150, (uint8_t)253, (uint8_t)42, (uint8_t)158, (uint8_t)40, (uint8_t)143, (uint8_t)114, (uint8_t)233, (uint8_t)4, (uint8_t)78, (uint8_t)70, (uint8_t)9, (uint8_t)217, (uint8_t)130, (uint8_t)121, (uint8_t)249, (uint8_t)188, (uint8_t)70, (uint8_t)74, (uint8_t)90, (uint8_t)85, (uint8_t)103, (uint8_t)230, (uint8_t)140, (uint8_t)109, (uint8_t)156, (uint8_t)249, (uint8_t)96, (uint8_t)155, (uint8_t)170, (uint8_t)88, (uint8_t)2, (uint8_t)92} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)122);
};


void c_CommunicationChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t)8192);
    assert(p143_press_abs_GET(pack) == (float) -1.946696E38F);
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)493567334L);
    assert(p143_press_diff_GET(pack) == (float) -8.207362E37F);
};


void c_CommunicationChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    assert(p144_custom_state_GET(pack) == (uint64_t)6870007098816702364L);
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)15);
    {
        float exemplary[] =  {-1.1486321E38F, 2.3676073E38F, -1.9738214E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-3.9906303E37F, 2.4230957E38F, 7.2651293E36F, -1.5446351E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-1.805299E38F, 1.413285E37F, 6.207582E37F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-2.2574354E38F, -2.2605794E38F, -3.2136835E38F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lon_GET(pack) == (int32_t)1840498473);
    assert(p144_alt_GET(pack) == (float) -4.290464E37F);
    {
        float exemplary[] =  {-3.283359E37F, -2.1221242E38F, -2.4118091E38F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lat_GET(pack) == (int32_t)1729480882);
    assert(p144_timestamp_GET(pack) == (uint64_t)4279504168482789703L);
};


void c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-1.1291554E38F, 5.4258235E37F, 2.6829949E38F, -3.062401E38F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_y_pos_GET(pack) == (float)3.3815742E38F);
    assert(p146_time_usec_GET(pack) == (uint64_t)2975577667978283410L);
    assert(p146_z_pos_GET(pack) == (float) -1.740992E38F);
    assert(p146_x_vel_GET(pack) == (float) -2.179621E38F);
    assert(p146_z_vel_GET(pack) == (float) -2.7671105E38F);
    {
        float exemplary[] =  {3.1021998E38F, 7.5249803E37F, 2.0008548E38F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-1.0952608E37F, -9.662304E37F, -4.8178623E37F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_x_acc_GET(pack) == (float) -1.0094178E38F);
    assert(p146_yaw_rate_GET(pack) == (float) -6.0608013E37F);
    assert(p146_roll_rate_GET(pack) == (float)1.3067257E38F);
    assert(p146_pitch_rate_GET(pack) == (float) -2.80269E38F);
    assert(p146_airspeed_GET(pack) == (float) -1.8774726E37F);
    assert(p146_y_acc_GET(pack) == (float) -1.895421E38F);
    assert(p146_z_acc_GET(pack) == (float) -1.0538645E38F);
    assert(p146_x_pos_GET(pack) == (float)1.2248886E38F);
    assert(p146_y_vel_GET(pack) == (float)2.5100336E38F);
};


void c_CommunicationChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)193);
    {
        uint16_t exemplary[] =  {(uint16_t)25863, (uint16_t)42194, (uint16_t)46087, (uint16_t)53530, (uint16_t)34041, (uint16_t)19457, (uint16_t)32125, (uint16_t)4534, (uint16_t)34168, (uint16_t)41846} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t)28211);
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t) -27466);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL);
    assert(p147_energy_consumed_GET(pack) == (int32_t)432518029);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t)115);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN);
    assert(p147_current_consumed_GET(pack) == (int32_t)1155217861);
};


void c_CommunicationChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)2654842194L);
    {
        uint8_t exemplary[] =  {(uint8_t)52, (uint8_t)54, (uint8_t)154, (uint8_t)116, (uint8_t)135, (uint8_t)246, (uint8_t)27, (uint8_t)192} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_board_version_GET(pack) == (uint32_t)1387443219L);
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)3675992424L);
    {
        uint8_t exemplary[] =  {(uint8_t)108, (uint8_t)125, (uint8_t)151, (uint8_t)108, (uint8_t)34, (uint8_t)139, (uint8_t)37, (uint8_t)27} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)28970);
    assert(p148_os_sw_version_GET(pack) == (uint32_t)3539602230L);
    {
        uint8_t exemplary[] =  {(uint8_t)91, (uint8_t)248, (uint8_t)118, (uint8_t)236, (uint8_t)147, (uint8_t)59, (uint8_t)202, (uint8_t)194, (uint8_t)69, (uint8_t)179, (uint8_t)78, (uint8_t)168, (uint8_t)153, (uint8_t)194, (uint8_t)37, (uint8_t)185, (uint8_t)236, (uint8_t)208} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)202, (uint8_t)13, (uint8_t)107, (uint8_t)182, (uint8_t)144, (uint8_t)175, (uint8_t)47, (uint8_t)122} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)26075);
    assert(p148_uid_GET(pack) == (uint64_t)3838784069892282189L);
    assert(p148_capabilities_GET(pack) == e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMMAND_INT);
};


void c_CommunicationChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER);
    assert(p149_time_usec_GET(pack) == (uint64_t)81914681621160817L);
    assert(p149_x_TRY(ph) == (float)1.962821E38F);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)238);
    {
        float exemplary[] =  {2.8488374E38F, 3.2065709E37F, -4.917466E37F, 1.264135E38F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p149_angle_x_GET(pack) == (float) -1.9459753E38F);
    assert(p149_y_TRY(ph) == (float)1.2388849E38F);
    assert(p149_distance_GET(pack) == (float) -7.473657E37F);
    assert(p149_size_x_GET(pack) == (float) -1.6699465E38F);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p149_size_y_GET(pack) == (float)4.0458113E37F);
    assert(p149_angle_y_GET(pack) == (float) -8.581042E37F);
    assert(p149_z_TRY(ph) == (float)3.1929468E38F);
};


void c_CommunicationChannel_on_SCRIPT_ITEM_180(Bounds_Inside * ph, Pack * pack)
{
    assert(p180_seq_GET(pack) == (uint16_t)(uint16_t)30861);
    assert(p180_target_system_GET(pack) == (uint8_t)(uint8_t)227);
    assert(p180_target_component_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p180_name_LEN(ph) == 25);
    {
        char16_t * exemplary = u"ktxnzwiixdvwyiwzojqrnldzo";
        char16_t * sample = p180_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 50);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SCRIPT_REQUEST_181(Bounds_Inside * ph, Pack * pack)
{
    assert(p181_target_system_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p181_seq_GET(pack) == (uint16_t)(uint16_t)19638);
    assert(p181_target_component_GET(pack) == (uint8_t)(uint8_t)199);
};


void c_CommunicationChannel_on_SCRIPT_REQUEST_LIST_182(Bounds_Inside * ph, Pack * pack)
{
    assert(p182_target_system_GET(pack) == (uint8_t)(uint8_t)0);
    assert(p182_target_component_GET(pack) == (uint8_t)(uint8_t)191);
};


void c_CommunicationChannel_on_SCRIPT_COUNT_183(Bounds_Inside * ph, Pack * pack)
{
    assert(p183_target_system_GET(pack) == (uint8_t)(uint8_t)9);
    assert(p183_target_component_GET(pack) == (uint8_t)(uint8_t)247);
    assert(p183_count_GET(pack) == (uint16_t)(uint16_t)19613);
};


void c_CommunicationChannel_on_SCRIPT_CURRENT_184(Bounds_Inside * ph, Pack * pack)
{
    assert(p184_seq_GET(pack) == (uint16_t)(uint16_t)26877);
};


void c_CommunicationChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_mag_ratio_GET(pack) == (float) -8.895444E37F);
    assert(p230_time_usec_GET(pack) == (uint64_t)8650523584487809958L);
    assert(p230_tas_ratio_GET(pack) == (float) -2.9689736E38F);
    assert(p230_vel_ratio_GET(pack) == (float)9.415936E37F);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float)3.3315174E38F);
    assert(p230_pos_vert_accuracy_GET(pack) == (float) -2.5573405E38F);
    assert(p230_pos_vert_ratio_GET(pack) == (float)1.887045E38F);
    assert(p230_flags_GET(pack) == e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS);
    assert(p230_pos_horiz_ratio_GET(pack) == (float)3.196326E38F);
    assert(p230_hagl_ratio_GET(pack) == (float) -7.621018E37F);
};


void c_CommunicationChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_var_horiz_GET(pack) == (float)6.471001E37F);
    assert(p231_horiz_accuracy_GET(pack) == (float)3.8009644E37F);
    assert(p231_wind_y_GET(pack) == (float) -5.663904E37F);
    assert(p231_time_usec_GET(pack) == (uint64_t)6922413796538833499L);
    assert(p231_vert_accuracy_GET(pack) == (float) -2.5281054E38F);
    assert(p231_wind_alt_GET(pack) == (float) -1.148085E38F);
    assert(p231_wind_z_GET(pack) == (float) -7.1157777E37F);
    assert(p231_wind_x_GET(pack) == (float)8.801804E37F);
    assert(p231_var_vert_GET(pack) == (float)3.2947284E38F);
};


void c_CommunicationChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)18086);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p232_alt_GET(pack) == (float) -2.277422E38F);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)182);
    assert(p232_speed_accuracy_GET(pack) == (float)2.6208283E38F);
    assert(p232_vd_GET(pack) == (float)1.1886941E38F);
    assert(p232_hdop_GET(pack) == (float) -2.0129638E38F);
    assert(p232_vn_GET(pack) == (float)9.279959E36F);
    assert(p232_vdop_GET(pack) == (float) -2.1962675E38F);
    assert(p232_lon_GET(pack) == (int32_t) -598491356);
    assert(p232_time_usec_GET(pack) == (uint64_t)5700286231885534053L);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)124);
    assert(p232_vert_accuracy_GET(pack) == (float) -3.8337164E37F);
    assert(p232_lat_GET(pack) == (int32_t)710176928);
    assert(p232_ve_GET(pack) == (float)2.4690015E38F);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)2327678466L);
    assert(p232_horiz_accuracy_GET(pack) == (float)1.1297521E38F);
    assert(p232_ignore_flags_GET(pack) == e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP);
};


void c_CommunicationChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)229);
    {
        uint8_t exemplary[] =  {(uint8_t)0, (uint8_t)254, (uint8_t)205, (uint8_t)30, (uint8_t)203, (uint8_t)69, (uint8_t)132, (uint8_t)197, (uint8_t)159, (uint8_t)176, (uint8_t)232, (uint8_t)55, (uint8_t)40, (uint8_t)52, (uint8_t)203, (uint8_t)144, (uint8_t)65, (uint8_t)158, (uint8_t)102, (uint8_t)18, (uint8_t)220, (uint8_t)237, (uint8_t)208, (uint8_t)251, (uint8_t)24, (uint8_t)138, (uint8_t)211, (uint8_t)245, (uint8_t)168, (uint8_t)127, (uint8_t)219, (uint8_t)79, (uint8_t)146, (uint8_t)89, (uint8_t)8, (uint8_t)116, (uint8_t)171, (uint8_t)115, (uint8_t)105, (uint8_t)147, (uint8_t)27, (uint8_t)63, (uint8_t)136, (uint8_t)55, (uint8_t)204, (uint8_t)144, (uint8_t)211, (uint8_t)7, (uint8_t)231, (uint8_t)238, (uint8_t)250, (uint8_t)51, (uint8_t)8, (uint8_t)207, (uint8_t)216, (uint8_t)234, (uint8_t)227, (uint8_t)115, (uint8_t)214, (uint8_t)107, (uint8_t)53, (uint8_t)55, (uint8_t)184, (uint8_t)20, (uint8_t)53, (uint8_t)162, (uint8_t)156, (uint8_t)123, (uint8_t)158, (uint8_t)153, (uint8_t)41, (uint8_t)113, (uint8_t)234, (uint8_t)53, (uint8_t)209, (uint8_t)38, (uint8_t)158, (uint8_t)174, (uint8_t)22, (uint8_t)69, (uint8_t)66, (uint8_t)232, (uint8_t)249, (uint8_t)251, (uint8_t)15, (uint8_t)93, (uint8_t)150, (uint8_t)218, (uint8_t)100, (uint8_t)14, (uint8_t)229, (uint8_t)206, (uint8_t)247, (uint8_t)177, (uint8_t)22, (uint8_t)93, (uint8_t)167, (uint8_t)227, (uint8_t)115, (uint8_t)73, (uint8_t)185, (uint8_t)173, (uint8_t)243, (uint8_t)182, (uint8_t)202, (uint8_t)52, (uint8_t)191, (uint8_t)134, (uint8_t)141, (uint8_t)144, (uint8_t)14, (uint8_t)159, (uint8_t)86, (uint8_t)230, (uint8_t)243, (uint8_t)240, (uint8_t)4, (uint8_t)104, (uint8_t)144, (uint8_t)46, (uint8_t)124, (uint8_t)184, (uint8_t)42, (uint8_t)115, (uint8_t)92, (uint8_t)124, (uint8_t)180, (uint8_t)58, (uint8_t)154, (uint8_t)229, (uint8_t)208, (uint8_t)45, (uint8_t)222, (uint8_t)206, (uint8_t)190, (uint8_t)91, (uint8_t)53, (uint8_t)27, (uint8_t)23, (uint8_t)207, (uint8_t)131, (uint8_t)175, (uint8_t)49, (uint8_t)70, (uint8_t)227, (uint8_t)229, (uint8_t)197, (uint8_t)139, (uint8_t)152, (uint8_t)166, (uint8_t)219, (uint8_t)69, (uint8_t)68, (uint8_t)88, (uint8_t)255, (uint8_t)4, (uint8_t)191, (uint8_t)162, (uint8_t)235, (uint8_t)150, (uint8_t)215, (uint8_t)106, (uint8_t)20, (uint8_t)187, (uint8_t)179, (uint8_t)35, (uint8_t)148, (uint8_t)98, (uint8_t)124, (uint8_t)54, (uint8_t)239, (uint8_t)234, (uint8_t)10, (uint8_t)106, (uint8_t)134, (uint8_t)88, (uint8_t)146, (uint8_t)229, (uint8_t)184, (uint8_t)40} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)7);
};


void c_CommunicationChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_latitude_GET(pack) == (int32_t)1655096094);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)68);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p234_custom_mode_GET(pack) == (uint32_t)3801128081L);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p234_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t)13306);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)146);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)50318);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)183);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t)125);
    assert(p234_longitude_GET(pack) == (int32_t) -1628746013);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t)3586);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t)1776);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)14490);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t)97);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t)38);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t) -15799);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t) -85);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t)27863);
};


void c_CommunicationChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_vibration_x_GET(pack) == (float)6.50424E36F);
    assert(p241_time_usec_GET(pack) == (uint64_t)8942639826998546540L);
    assert(p241_vibration_z_GET(pack) == (float) -9.297376E37F);
    assert(p241_clipping_2_GET(pack) == (uint32_t)328569783L);
    assert(p241_vibration_y_GET(pack) == (float)2.0011131E37F);
    assert(p241_clipping_0_GET(pack) == (uint32_t)4291047704L);
    assert(p241_clipping_1_GET(pack) == (uint32_t)3306060183L);
};


void c_CommunicationChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_time_usec_TRY(ph) == (uint64_t)4672639445532162590L);
    assert(p242_latitude_GET(pack) == (int32_t) -1678408284);
    {
        float exemplary[] =  {3.2809434E38F, 1.2299912E38F, 5.6277774E37F, 2.6870933E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_approach_z_GET(pack) == (float) -7.6773813E37F);
    assert(p242_longitude_GET(pack) == (int32_t)674765480);
    assert(p242_z_GET(pack) == (float)2.5536623E38F);
    assert(p242_y_GET(pack) == (float) -2.8301014E38F);
    assert(p242_approach_x_GET(pack) == (float) -2.3727286E37F);
    assert(p242_approach_y_GET(pack) == (float) -2.7345085E38F);
    assert(p242_altitude_GET(pack) == (int32_t) -1814616134);
    assert(p242_x_GET(pack) == (float)2.3649636E38F);
};


void c_CommunicationChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_approach_x_GET(pack) == (float)2.8701616E38F);
    assert(p243_time_usec_TRY(ph) == (uint64_t)2566154670001589697L);
    assert(p243_longitude_GET(pack) == (int32_t)1394529959);
    assert(p243_latitude_GET(pack) == (int32_t)1531838969);
    assert(p243_altitude_GET(pack) == (int32_t)1602230401);
    assert(p243_approach_y_GET(pack) == (float)9.02974E37F);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p243_y_GET(pack) == (float) -7.495438E37F);
    {
        float exemplary[] =  {2.24303E38F, 1.5621928E38F, 2.718189E38F, 3.195863E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_x_GET(pack) == (float) -2.4458416E38F);
    assert(p243_approach_z_GET(pack) == (float)2.0188031E38F);
    assert(p243_z_GET(pack) == (float) -1.5449559E38F);
};


void c_CommunicationChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_interval_us_GET(pack) == (int32_t)2056408475);
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)40890);
};


void c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND);
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_MC);
};


void c_CommunicationChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_callsign_LEN(ph) == 5);
    {
        char16_t * exemplary = u"tinwq";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_ICAO_address_GET(pack) == (uint32_t)1702756559L);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t) -10745);
    assert(p246_altitude_GET(pack) == (int32_t)1635815296);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)16960);
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)153);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)52133);
    assert(p246_flags_GET(pack) == e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UAV);
    assert(p246_lat_GET(pack) == (int32_t) -2007246302);
    assert(p246_lon_GET(pack) == (int32_t)1626159605);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)60621);
};


void c_CommunicationChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH);
    assert(p247_time_to_minimum_delta_GET(pack) == (float)2.8415275E38F);
    assert(p247_altitude_minimum_delta_GET(pack) == (float)1.7878277E38F);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float) -3.1439599E38F);
    assert(p247_id_GET(pack) == (uint32_t)2283758296L);
};


void c_CommunicationChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)12954);
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)49);
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)234);
    {
        uint8_t exemplary[] =  {(uint8_t)101, (uint8_t)230, (uint8_t)76, (uint8_t)218, (uint8_t)4, (uint8_t)235, (uint8_t)209, (uint8_t)128, (uint8_t)10, (uint8_t)38, (uint8_t)139, (uint8_t)201, (uint8_t)162, (uint8_t)189, (uint8_t)142, (uint8_t)253, (uint8_t)250, (uint8_t)78, (uint8_t)11, (uint8_t)90, (uint8_t)53, (uint8_t)252, (uint8_t)56, (uint8_t)74, (uint8_t)45, (uint8_t)113, (uint8_t)8, (uint8_t)235, (uint8_t)94, (uint8_t)238, (uint8_t)112, (uint8_t)96, (uint8_t)126, (uint8_t)252, (uint8_t)119, (uint8_t)119, (uint8_t)207, (uint8_t)158, (uint8_t)61, (uint8_t)50, (uint8_t)214, (uint8_t)157, (uint8_t)52, (uint8_t)80, (uint8_t)18, (uint8_t)202, (uint8_t)66, (uint8_t)92, (uint8_t)172, (uint8_t)189, (uint8_t)103, (uint8_t)231, (uint8_t)216, (uint8_t)26, (uint8_t)177, (uint8_t)5, (uint8_t)208, (uint8_t)225, (uint8_t)52, (uint8_t)189, (uint8_t)47, (uint8_t)109, (uint8_t)201, (uint8_t)47, (uint8_t)237, (uint8_t)84, (uint8_t)13, (uint8_t)125, (uint8_t)143, (uint8_t)236, (uint8_t)169, (uint8_t)156, (uint8_t)197, (uint8_t)206, (uint8_t)187, (uint8_t)41, (uint8_t)201, (uint8_t)17, (uint8_t)32, (uint8_t)3, (uint8_t)163, (uint8_t)98, (uint8_t)184, (uint8_t)252, (uint8_t)110, (uint8_t)156, (uint8_t)104, (uint8_t)196, (uint8_t)242, (uint8_t)248, (uint8_t)128, (uint8_t)154, (uint8_t)58, (uint8_t)88, (uint8_t)177, (uint8_t)107, (uint8_t)135, (uint8_t)91, (uint8_t)194, (uint8_t)77, (uint8_t)255, (uint8_t)79, (uint8_t)227, (uint8_t)29, (uint8_t)112, (uint8_t)207, (uint8_t)250, (uint8_t)0, (uint8_t)2, (uint8_t)249, (uint8_t)138, (uint8_t)249, (uint8_t)178, (uint8_t)240, (uint8_t)237, (uint8_t)204, (uint8_t)237, (uint8_t)89, (uint8_t)208, (uint8_t)176, (uint8_t)139, (uint8_t)227, (uint8_t)59, (uint8_t)118, (uint8_t)124, (uint8_t)118, (uint8_t)1, (uint8_t)85, (uint8_t)6, (uint8_t)171, (uint8_t)64, (uint8_t)31, (uint8_t)10, (uint8_t)116, (uint8_t)196, (uint8_t)130, (uint8_t)190, (uint8_t)27, (uint8_t)234, (uint8_t)28, (uint8_t)227, (uint8_t)209, (uint8_t)29, (uint8_t)214, (uint8_t)102, (uint8_t)11, (uint8_t)88, (uint8_t)20, (uint8_t)70, (uint8_t)131, (uint8_t)190, (uint8_t)247, (uint8_t)10, (uint8_t)250, (uint8_t)149, (uint8_t)255, (uint8_t)218, (uint8_t)97, (uint8_t)121, (uint8_t)245, (uint8_t)134, (uint8_t)73, (uint8_t)77, (uint8_t)26, (uint8_t)127, (uint8_t)234, (uint8_t)81, (uint8_t)138, (uint8_t)82, (uint8_t)87, (uint8_t)141, (uint8_t)1, (uint8_t)209, (uint8_t)244, (uint8_t)97, (uint8_t)168, (uint8_t)79, (uint8_t)55, (uint8_t)79, (uint8_t)188, (uint8_t)244, (uint8_t)103, (uint8_t)2, (uint8_t)69, (uint8_t)38, (uint8_t)180, (uint8_t)128, (uint8_t)73, (uint8_t)163, (uint8_t)17, (uint8_t)176, (uint8_t)105, (uint8_t)51, (uint8_t)123, (uint8_t)83, (uint8_t)52, (uint8_t)155, (uint8_t)227, (uint8_t)2, (uint8_t)42, (uint8_t)195, (uint8_t)140, (uint8_t)190, (uint8_t)91, (uint8_t)32, (uint8_t)131, (uint8_t)248, (uint8_t)149, (uint8_t)134, (uint8_t)65, (uint8_t)226, (uint8_t)94, (uint8_t)12, (uint8_t)71, (uint8_t)191, (uint8_t)64, (uint8_t)51, (uint8_t)120, (uint8_t)214, (uint8_t)247, (uint8_t)46, (uint8_t)200, (uint8_t)121, (uint8_t)168, (uint8_t)217, (uint8_t)79, (uint8_t)126, (uint8_t)61, (uint8_t)24, (uint8_t)74, (uint8_t)49, (uint8_t)124, (uint8_t)145, (uint8_t)43, (uint8_t)132, (uint8_t)47, (uint8_t)141, (uint8_t)215, (uint8_t)10, (uint8_t)28, (uint8_t)55, (uint8_t)88, (uint8_t)77, (uint8_t)115, (uint8_t)49, (uint8_t)219, (uint8_t)183, (uint8_t)197, (uint8_t)189} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)17);
};


void c_CommunicationChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)167);
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)131);
    {
        int8_t exemplary[] =  {(int8_t)19, (int8_t) -78, (int8_t)110, (int8_t) -48, (int8_t)125, (int8_t) -94, (int8_t)56, (int8_t) -28, (int8_t)76, (int8_t)101, (int8_t)90, (int8_t) -11, (int8_t)76, (int8_t) -51, (int8_t)51, (int8_t)111, (int8_t)90, (int8_t) -87, (int8_t)26, (int8_t)24, (int8_t) -35, (int8_t)34, (int8_t) -30, (int8_t) -50, (int8_t)11, (int8_t)49, (int8_t) -117, (int8_t) -111, (int8_t) -28, (int8_t)27, (int8_t) -118, (int8_t)15} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)56413);
};


void c_CommunicationChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_y_GET(pack) == (float) -1.2712121E38F);
    assert(p250_x_GET(pack) == (float)6.214597E37F);
    assert(p250_z_GET(pack) == (float) -1.8165853E38F);
    assert(p250_time_usec_GET(pack) == (uint64_t)5526748862680987300L);
    assert(p250_name_LEN(ph) == 4);
    {
        char16_t * exemplary = u"nnqt";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_name_LEN(ph) == 2);
    {
        char16_t * exemplary = u"Da";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)3667925539L);
    assert(p251_value_GET(pack) == (float) -2.340629E38F);
};


void c_CommunicationChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_name_LEN(ph) == 8);
    {
        char16_t * exemplary = u"ldoqkesk";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)827410548L);
    assert(p252_value_GET(pack) == (int32_t) -1123165763);
};


void c_CommunicationChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_ALERT);
    assert(p253_text_LEN(ph) == 24);
    {
        char16_t * exemplary = u"aeeowhlzobrptkgpnbainvqy";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 48);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_value_GET(pack) == (float)1.0263536E38F);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)3460879917L);
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)124);
};


void c_CommunicationChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)1752657446413388789L);
    {
        uint8_t exemplary[] =  {(uint8_t)43, (uint8_t)253, (uint8_t)152, (uint8_t)44, (uint8_t)32, (uint8_t)227, (uint8_t)41, (uint8_t)90, (uint8_t)95, (uint8_t)92, (uint8_t)121, (uint8_t)172, (uint8_t)181, (uint8_t)37, (uint8_t)91, (uint8_t)16, (uint8_t)103, (uint8_t)45, (uint8_t)200, (uint8_t)132, (uint8_t)58, (uint8_t)41, (uint8_t)16, (uint8_t)29, (uint8_t)32, (uint8_t)131, (uint8_t)171, (uint8_t)57, (uint8_t)98, (uint8_t)160, (uint8_t)169, (uint8_t)93} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)144);
};


void c_CommunicationChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_last_change_ms_GET(pack) == (uint32_t)3661985548L);
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)626888448L);
};


void c_CommunicationChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)95);
    assert(p258_tune_LEN(ph) == 2);
    {
        char16_t * exemplary = u"dj";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)131);
};


void c_CommunicationChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_firmware_version_GET(pack) == (uint32_t)595480510L);
    assert(p259_sensor_size_v_GET(pack) == (float) -2.0490965E37F);
    assert(p259_cam_definition_uri_LEN(ph) == 57);
    {
        char16_t * exemplary = u"aqdyvwooSPUpdfzedsirueLgnxfcfgbosewwBxoyIgnsniyWfctrinweh";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 114);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)83, (uint8_t)133, (uint8_t)210, (uint8_t)24, (uint8_t)78, (uint8_t)253, (uint8_t)52, (uint8_t)20, (uint8_t)146, (uint8_t)241, (uint8_t)37, (uint8_t)29, (uint8_t)130, (uint8_t)9, (uint8_t)249, (uint8_t)232, (uint8_t)64, (uint8_t)31, (uint8_t)65, (uint8_t)124, (uint8_t)25, (uint8_t)17, (uint8_t)147, (uint8_t)115, (uint8_t)36, (uint8_t)239, (uint8_t)224, (uint8_t)219, (uint8_t)152, (uint8_t)15, (uint8_t)183, (uint8_t)216} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_focal_length_GET(pack) == (float)5.554374E37F);
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)58080);
    {
        uint8_t exemplary[] =  {(uint8_t)117, (uint8_t)95, (uint8_t)50, (uint8_t)142, (uint8_t)135, (uint8_t)40, (uint8_t)72, (uint8_t)169, (uint8_t)222, (uint8_t)221, (uint8_t)84, (uint8_t)170, (uint8_t)125, (uint8_t)202, (uint8_t)130, (uint8_t)140, (uint8_t)8, (uint8_t)75, (uint8_t)109, (uint8_t)237, (uint8_t)190, (uint8_t)71, (uint8_t)196, (uint8_t)105, (uint8_t)4, (uint8_t)47, (uint8_t)228, (uint8_t)36, (uint8_t)95, (uint8_t)178, (uint8_t)205, (uint8_t)72} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_flags_GET(pack) == e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)52015);
    assert(p259_sensor_size_h_GET(pack) == (float)3.0338793E38F);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)65);
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)44724);
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)3150093291L);
};


void c_CommunicationChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)458067567L);
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_VIDEO);
};


void c_CommunicationChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_total_capacity_GET(pack) == (float) -6.6644885E37F);
    assert(p261_write_speed_GET(pack) == (float)1.6210977E38F);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)251);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p261_available_capacity_GET(pack) == (float) -1.1375118E38F);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)3855099934L);
    assert(p261_read_speed_GET(pack) == (float)1.4978025E38F);
    assert(p261_used_capacity_GET(pack) == (float)3.3304098E38F);
};


void c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p262_image_interval_GET(pack) == (float)2.9112975E38F);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)1044396119L);
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)1686141741L);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)49);
    assert(p262_available_capacity_GET(pack) == (float) -1.253692E38F);
};


void c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p263_image_index_GET(pack) == (int32_t) -377666724);
    assert(p263_relative_alt_GET(pack) == (int32_t) -532615088);
    assert(p263_time_utc_GET(pack) == (uint64_t)8303992998392865220L);
    assert(p263_file_url_LEN(ph) == 166);
    {
        char16_t * exemplary = u"XrtAclowbzlbbexMbllrkcehraJnewpjnwtfxjsTOqbZqxnuzcfixcUypspekovtbvzplacYmvftytBpfftjbhdsgajiiWdinFxfxxUmTinxzhiahcrrnizgbtkrvwwhovvxjqpctjppcylrlgbpuqkxjsmcabbypcbksc";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 332);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_alt_GET(pack) == (int32_t)1036762488);
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)2511759240L);
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t) -120);
    assert(p263_lon_GET(pack) == (int32_t) -710566914);
    {
        float exemplary[] =  {-1.928797E38F, 3.22647E38F, -2.5164941E38F, 2.1487838E38F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_lat_GET(pack) == (int32_t)1499872835);
};


void c_CommunicationChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)7778942620396854329L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)1680708644L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)2814884143861888998L);
    assert(p264_flight_uuid_GET(pack) == (uint64_t)1750588388826710228L);
};


void c_CommunicationChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_yaw_GET(pack) == (float)2.4555824E38F);
    assert(p265_roll_GET(pack) == (float)2.2871473E38F);
    assert(p265_pitch_GET(pack) == (float)1.0000598E38F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)611813947L);
};


void c_CommunicationChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)67);
    {
        uint8_t exemplary[] =  {(uint8_t)173, (uint8_t)223, (uint8_t)62, (uint8_t)195, (uint8_t)219, (uint8_t)228, (uint8_t)112, (uint8_t)243, (uint8_t)227, (uint8_t)232, (uint8_t)140, (uint8_t)123, (uint8_t)248, (uint8_t)234, (uint8_t)244, (uint8_t)31, (uint8_t)204, (uint8_t)109, (uint8_t)69, (uint8_t)160, (uint8_t)110, (uint8_t)101, (uint8_t)102, (uint8_t)1, (uint8_t)209, (uint8_t)11, (uint8_t)169, (uint8_t)113, (uint8_t)131, (uint8_t)50, (uint8_t)230, (uint8_t)143, (uint8_t)168, (uint8_t)104, (uint8_t)166, (uint8_t)28, (uint8_t)43, (uint8_t)45, (uint8_t)227, (uint8_t)142, (uint8_t)3, (uint8_t)17, (uint8_t)146, (uint8_t)98, (uint8_t)212, (uint8_t)223, (uint8_t)15, (uint8_t)70, (uint8_t)73, (uint8_t)100, (uint8_t)17, (uint8_t)33, (uint8_t)175, (uint8_t)240, (uint8_t)185, (uint8_t)236, (uint8_t)218, (uint8_t)79, (uint8_t)40, (uint8_t)205, (uint8_t)103, (uint8_t)168, (uint8_t)82, (uint8_t)158, (uint8_t)112, (uint8_t)130, (uint8_t)16, (uint8_t)248, (uint8_t)20, (uint8_t)193, (uint8_t)239, (uint8_t)141, (uint8_t)106, (uint8_t)39, (uint8_t)107, (uint8_t)124, (uint8_t)28, (uint8_t)53, (uint8_t)168, (uint8_t)94, (uint8_t)97, (uint8_t)156, (uint8_t)23, (uint8_t)28, (uint8_t)151, (uint8_t)190, (uint8_t)81, (uint8_t)233, (uint8_t)214, (uint8_t)168, (uint8_t)81, (uint8_t)139, (uint8_t)112, (uint8_t)212, (uint8_t)148, (uint8_t)138, (uint8_t)146, (uint8_t)189, (uint8_t)40, (uint8_t)138, (uint8_t)222, (uint8_t)60, (uint8_t)123, (uint8_t)90, (uint8_t)66, (uint8_t)82, (uint8_t)204, (uint8_t)139, (uint8_t)149, (uint8_t)94, (uint8_t)12, (uint8_t)137, (uint8_t)122, (uint8_t)139, (uint8_t)193, (uint8_t)58, (uint8_t)34, (uint8_t)94, (uint8_t)12, (uint8_t)245, (uint8_t)219, (uint8_t)102, (uint8_t)110, (uint8_t)192, (uint8_t)35, (uint8_t)208, (uint8_t)72, (uint8_t)145, (uint8_t)68, (uint8_t)223, (uint8_t)37, (uint8_t)20, (uint8_t)13, (uint8_t)118, (uint8_t)225, (uint8_t)193, (uint8_t)16, (uint8_t)142, (uint8_t)79, (uint8_t)24, (uint8_t)219, (uint8_t)126, (uint8_t)236, (uint8_t)167, (uint8_t)15, (uint8_t)14, (uint8_t)205, (uint8_t)155, (uint8_t)73, (uint8_t)115, (uint8_t)81, (uint8_t)115, (uint8_t)9, (uint8_t)111, (uint8_t)109, (uint8_t)241, (uint8_t)134, (uint8_t)21, (uint8_t)15, (uint8_t)246, (uint8_t)47, (uint8_t)163, (uint8_t)174, (uint8_t)188, (uint8_t)174, (uint8_t)25, (uint8_t)27, (uint8_t)179, (uint8_t)172, (uint8_t)30, (uint8_t)170, (uint8_t)31, (uint8_t)7, (uint8_t)97, (uint8_t)37, (uint8_t)254, (uint8_t)240, (uint8_t)247, (uint8_t)60, (uint8_t)151, (uint8_t)109, (uint8_t)142, (uint8_t)0, (uint8_t)199, (uint8_t)18, (uint8_t)164, (uint8_t)22, (uint8_t)162, (uint8_t)195, (uint8_t)127, (uint8_t)94, (uint8_t)146, (uint8_t)128, (uint8_t)240, (uint8_t)230, (uint8_t)174, (uint8_t)31, (uint8_t)158, (uint8_t)224, (uint8_t)23, (uint8_t)236, (uint8_t)183, (uint8_t)21, (uint8_t)52, (uint8_t)44, (uint8_t)76, (uint8_t)172, (uint8_t)68, (uint8_t)189, (uint8_t)37, (uint8_t)0, (uint8_t)214, (uint8_t)93, (uint8_t)164, (uint8_t)11, (uint8_t)11, (uint8_t)128, (uint8_t)53, (uint8_t)83, (uint8_t)11, (uint8_t)176, (uint8_t)137, (uint8_t)208, (uint8_t)15, (uint8_t)11, (uint8_t)82, (uint8_t)34, (uint8_t)215, (uint8_t)194, (uint8_t)103, (uint8_t)29, (uint8_t)2, (uint8_t)33, (uint8_t)79, (uint8_t)189, (uint8_t)18, (uint8_t)55, (uint8_t)225, (uint8_t)144, (uint8_t)135, (uint8_t)240, (uint8_t)115, (uint8_t)101, (uint8_t)47, (uint8_t)194, (uint8_t)234, (uint8_t)251, (uint8_t)1, (uint8_t)42} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)109);
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)178);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)19617);
};


void c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)90);
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)188);
    {
        uint8_t exemplary[] =  {(uint8_t)245, (uint8_t)11, (uint8_t)191, (uint8_t)112, (uint8_t)43, (uint8_t)136, (uint8_t)64, (uint8_t)114, (uint8_t)15, (uint8_t)163, (uint8_t)223, (uint8_t)106, (uint8_t)35, (uint8_t)23, (uint8_t)20, (uint8_t)180, (uint8_t)41, (uint8_t)173, (uint8_t)127, (uint8_t)231, (uint8_t)244, (uint8_t)20, (uint8_t)148, (uint8_t)173, (uint8_t)202, (uint8_t)10, (uint8_t)229, (uint8_t)234, (uint8_t)158, (uint8_t)254, (uint8_t)253, (uint8_t)24, (uint8_t)227, (uint8_t)153, (uint8_t)168, (uint8_t)9, (uint8_t)184, (uint8_t)6, (uint8_t)230, (uint8_t)9, (uint8_t)229, (uint8_t)25, (uint8_t)149, (uint8_t)182, (uint8_t)35, (uint8_t)67, (uint8_t)28, (uint8_t)240, (uint8_t)10, (uint8_t)183, (uint8_t)35, (uint8_t)212, (uint8_t)188, (uint8_t)235, (uint8_t)221, (uint8_t)12, (uint8_t)177, (uint8_t)34, (uint8_t)33, (uint8_t)246, (uint8_t)34, (uint8_t)173, (uint8_t)61, (uint8_t)15, (uint8_t)223, (uint8_t)172, (uint8_t)72, (uint8_t)243, (uint8_t)38, (uint8_t)144, (uint8_t)49, (uint8_t)28, (uint8_t)28, (uint8_t)128, (uint8_t)203, (uint8_t)221, (uint8_t)172, (uint8_t)3, (uint8_t)247, (uint8_t)205, (uint8_t)238, (uint8_t)48, (uint8_t)218, (uint8_t)63, (uint8_t)134, (uint8_t)207, (uint8_t)25, (uint8_t)169, (uint8_t)36, (uint8_t)193, (uint8_t)18, (uint8_t)134, (uint8_t)62, (uint8_t)14, (uint8_t)168, (uint8_t)49, (uint8_t)81, (uint8_t)76, (uint8_t)61, (uint8_t)52, (uint8_t)158, (uint8_t)134, (uint8_t)244, (uint8_t)36, (uint8_t)119, (uint8_t)240, (uint8_t)58, (uint8_t)12, (uint8_t)225, (uint8_t)86, (uint8_t)9, (uint8_t)144, (uint8_t)101, (uint8_t)57, (uint8_t)12, (uint8_t)239, (uint8_t)84, (uint8_t)210, (uint8_t)188, (uint8_t)39, (uint8_t)46, (uint8_t)59, (uint8_t)87, (uint8_t)188, (uint8_t)253, (uint8_t)156, (uint8_t)103, (uint8_t)215, (uint8_t)60, (uint8_t)3, (uint8_t)17, (uint8_t)130, (uint8_t)220, (uint8_t)134, (uint8_t)11, (uint8_t)147, (uint8_t)89, (uint8_t)91, (uint8_t)42, (uint8_t)91, (uint8_t)26, (uint8_t)191, (uint8_t)102, (uint8_t)137, (uint8_t)83, (uint8_t)137, (uint8_t)37, (uint8_t)141, (uint8_t)117, (uint8_t)251, (uint8_t)49, (uint8_t)98, (uint8_t)77, (uint8_t)232, (uint8_t)87, (uint8_t)58, (uint8_t)216, (uint8_t)69, (uint8_t)133, (uint8_t)41, (uint8_t)40, (uint8_t)64, (uint8_t)86, (uint8_t)60, (uint8_t)47, (uint8_t)46, (uint8_t)10, (uint8_t)96, (uint8_t)160, (uint8_t)180, (uint8_t)191, (uint8_t)252, (uint8_t)119, (uint8_t)209, (uint8_t)230, (uint8_t)127, (uint8_t)220, (uint8_t)125, (uint8_t)123, (uint8_t)224, (uint8_t)49, (uint8_t)162, (uint8_t)45, (uint8_t)10, (uint8_t)17, (uint8_t)109, (uint8_t)93, (uint8_t)225, (uint8_t)238, (uint8_t)57, (uint8_t)222, (uint8_t)152, (uint8_t)177, (uint8_t)231, (uint8_t)91, (uint8_t)68, (uint8_t)21, (uint8_t)135, (uint8_t)46, (uint8_t)112, (uint8_t)10, (uint8_t)189, (uint8_t)232, (uint8_t)145, (uint8_t)103, (uint8_t)2, (uint8_t)101, (uint8_t)234, (uint8_t)43, (uint8_t)131, (uint8_t)216, (uint8_t)66, (uint8_t)68, (uint8_t)109, (uint8_t)255, (uint8_t)235, (uint8_t)136, (uint8_t)3, (uint8_t)22, (uint8_t)32, (uint8_t)178, (uint8_t)255, (uint8_t)105, (uint8_t)116, (uint8_t)35, (uint8_t)122, (uint8_t)7, (uint8_t)106, (uint8_t)6, (uint8_t)202, (uint8_t)62, (uint8_t)114, (uint8_t)198, (uint8_t)242, (uint8_t)173, (uint8_t)235, (uint8_t)217, (uint8_t)252, (uint8_t)166, (uint8_t)152, (uint8_t)127, (uint8_t)146, (uint8_t)186, (uint8_t)228, (uint8_t)241, (uint8_t)72, (uint8_t)137, (uint8_t)116, (uint8_t)129} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)25159);
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)53);
};


void c_CommunicationChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)44443);
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)88);
};


void c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)10277);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)26043);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)51376);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p269_framerate_GET(pack) == (float)2.644777E38F);
    assert(p269_bitrate_GET(pack) == (uint32_t)2472526889L);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p269_uri_LEN(ph) == 39);
    {
        char16_t * exemplary = u"stcwsiiaplblcqkkIlivmzlnKYmSAjcukxHjrdq";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 78);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)153);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)36210);
    assert(p270_framerate_GET(pack) == (float)6.9973796E37F);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)14720);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)247);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)48651);
    assert(p270_bitrate_GET(pack) == (uint32_t)2007766081L);
    assert(p270_uri_LEN(ph) == 15);
    {
        char16_t * exemplary = u"uqlxOygORxoukql";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_ssid_LEN(ph) == 21);
    {
        char16_t * exemplary = u"lliWykjnhcqnwnscmwfkv";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 42);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_password_LEN(ph) == 8);
    {
        char16_t * exemplary = u"fnxgcuWa";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)12313);
    {
        uint8_t exemplary[] =  {(uint8_t)82, (uint8_t)100, (uint8_t)72, (uint8_t)232, (uint8_t)250, (uint8_t)129, (uint8_t)151, (uint8_t)118} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)202, (uint8_t)126, (uint8_t)108, (uint8_t)122, (uint8_t)7, (uint8_t)244, (uint8_t)175, (uint8_t)190} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)2370);
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)28237);
};


void c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_uptime_sec_GET(pack) == (uint32_t)513399332L);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_OK);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)55530);
    assert(p310_time_usec_GET(pack) == (uint64_t)1080011271449153016L);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)92);
};


void c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)1080322266L);
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)227);
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)225);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)91);
    {
        uint8_t exemplary[] =  {(uint8_t)75, (uint8_t)203, (uint8_t)156, (uint8_t)8, (uint8_t)200, (uint8_t)0, (uint8_t)2, (uint8_t)187, (uint8_t)249, (uint8_t)156, (uint8_t)29, (uint8_t)174, (uint8_t)45, (uint8_t)122, (uint8_t)171, (uint8_t)181} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)152);
    assert(p311_name_LEN(ph) == 61);
    {
        char16_t * exemplary = u"wGufjiifptipabAvvomjkjmdckOysuhhyfedkhhtsofwvNupiukGDghceenyi";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 122);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_uptime_sec_GET(pack) == (uint32_t)1295781270L);
    assert(p311_time_usec_GET(pack) == (uint64_t)3717425399426368881L);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"rbiqIyXsppdsu";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)224);
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t) -22867);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)63);
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)88);
};


void c_CommunicationChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_value_LEN(ph) == 43);
    {
        char16_t * exemplary = u"gtqnnylwfbpDmqicHebatevxfafyBpyQbdtgtpqfhwP";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 86);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)24701);
    assert(p322_param_id_LEN(ph) == 6);
    {
        char16_t * exemplary = u"upigdd";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64);
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)60745);
};


void c_CommunicationChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_param_value_LEN(ph) == 104);
    {
        char16_t * exemplary = u"MlyttvtspubjceuThjvnfnwzyrdkkhxoddktvFtjeeorQzzzhfhbhepkexUcTHgkgemmewfsnuwegivgkmnSmyykxajkIejkfzvaAdrh";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 208);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)120);
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32);
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p323_param_id_LEN(ph) == 16);
    {
        char16_t * exemplary = u"rdeognqbjpzwHovk";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_IN_PROGRESS);
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64);
    assert(p324_param_value_LEN(ph) == 80);
    {
        char16_t * exemplary = u"ketShqihmigLslrsmWyrbxkWrhkmahbxgvvhDzccyufcjkqXxYvqvbjZaJzDsudtpkiMwleuhsAykdjK";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 160);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_id_LEN(ph) == 11);
    {
        char16_t * exemplary = u"gylcfQWdstr";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 22);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)22469);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)39002);
    assert(p330_time_usec_GET(pack) == (uint64_t)5281984995944529875L);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN);
    {
        uint16_t exemplary[] =  {(uint16_t)53760, (uint16_t)16994, (uint16_t)38921, (uint16_t)42230, (uint16_t)13469, (uint16_t)42249, (uint16_t)40964, (uint16_t)48488, (uint16_t)38355, (uint16_t)38515, (uint16_t)23708, (uint16_t)13759, (uint16_t)3541, (uint16_t)59710, (uint16_t)23386, (uint16_t)19197, (uint16_t)42871, (uint16_t)25563, (uint16_t)59819, (uint16_t)1531, (uint16_t)34299, (uint16_t)55700, (uint16_t)13518, (uint16_t)39290, (uint16_t)22524, (uint16_t)22582, (uint16_t)59842, (uint16_t)720, (uint16_t)30784, (uint16_t)56439, (uint16_t)51002, (uint16_t)7385, (uint16_t)49201, (uint16_t)15965, (uint16_t)952, (uint16_t)15067, (uint16_t)61706, (uint16_t)39823, (uint16_t)33199, (uint16_t)59084, (uint16_t)16446, (uint16_t)22766, (uint16_t)61743, (uint16_t)36762, (uint16_t)55614, (uint16_t)20140, (uint16_t)54331, (uint16_t)38686, (uint16_t)13060, (uint16_t)32471, (uint16_t)40501, (uint16_t)22531, (uint16_t)47961, (uint16_t)52517, (uint16_t)31687, (uint16_t)27489, (uint16_t)63971, (uint16_t)1633, (uint16_t)11376, (uint16_t)3850, (uint16_t)55919, (uint16_t)1555, (uint16_t)25982, (uint16_t)51974, (uint16_t)12428, (uint16_t)55681, (uint16_t)20792, (uint16_t)22095, (uint16_t)47340, (uint16_t)54652, (uint16_t)41155, (uint16_t)60118} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)134);
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
        p0_mavlink_version_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        p0_custom_mode_SET((uint32_t)2244579788L, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_VTOL_DUOROTOR, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_FP, PH.base.pack) ;
        p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_CRITICAL, PH.base.pack) ;
        c_TEST_Channel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
        p1_voltage_battery_SET((uint16_t)(uint16_t)37960, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)20068, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)46445, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)13248, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)16611, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)3410, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t) -84, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)63551, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)12703, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t)7979, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN, PH.base.pack) ;
        c_TEST_Channel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_unix_usec_SET((uint64_t)5496399966035986942L, PH.base.pack) ;
        p2_time_boot_ms_SET((uint32_t)3023421201L, PH.base.pack) ;
        c_TEST_Channel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_vx_SET((float) -1.6381922E38F, PH.base.pack) ;
        p3_x_SET((float)3.394667E37F, PH.base.pack) ;
        p3_yaw_SET((float)2.8201291E38F, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)34546, PH.base.pack) ;
        p3_afz_SET((float)1.206009E38F, PH.base.pack) ;
        p3_yaw_rate_SET((float)1.8658677E37F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)3436052476L, PH.base.pack) ;
        p3_z_SET((float)9.522373E37F, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        p3_vz_SET((float)2.0536041E38F, PH.base.pack) ;
        p3_vy_SET((float)2.8099625E38F, PH.base.pack) ;
        p3_y_SET((float)1.954934E38F, PH.base.pack) ;
        p3_afy_SET((float)3.5315595E37F, PH.base.pack) ;
        p3_afx_SET((float)2.5358008E38F, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PING_4(), &PH);
        p4_target_component_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p4_target_system_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
        p4_time_usec_SET((uint64_t)2167029319243596742L, PH.base.pack) ;
        p4_seq_SET((uint32_t)2845956950L, PH.base.pack) ;
        c_TEST_Channel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_version_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p5_control_request_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        {
            char16_t* passkey = u"iy";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_target_system_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_ack_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        p6_control_request_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"fearnmljfrhwfxospkmgfBnffrdJdp";
            p7_key_SET_(key, &PH) ;
        }
        c_TEST_Channel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_DISARMED, PH.base.pack) ;
        p11_custom_mode_SET((uint32_t)3126152854L, PH.base.pack) ;
        p11_target_system_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        c_TEST_Channel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
        p20_param_index_SET((int16_t)(int16_t) -12756, PH.base.pack) ;
        p20_target_system_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        {
            char16_t* param_id = u"i";
            p20_param_id_SET_(param_id, &PH) ;
        }
        p20_target_component_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_component_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        p21_target_system_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT16, PH.base.pack) ;
        p22_param_value_SET((float) -1.4255161E38F, PH.base.pack) ;
        p22_param_count_SET((uint16_t)(uint16_t)51229, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)9932, PH.base.pack) ;
        {
            char16_t* param_id = u"tivuxeKn";
            p22_param_id_SET_(param_id, &PH) ;
        }
        c_TEST_Channel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
        p23_target_component_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64, PH.base.pack) ;
        {
            char16_t* param_id = u"Q";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_param_value_SET((float)2.147907E38F, PH.base.pack) ;
        p23_target_system_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
        p24_cog_SET((uint16_t)(uint16_t)53908, PH.base.pack) ;
        p24_epv_SET((uint16_t)(uint16_t)39716, PH.base.pack) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)3963574299L, &PH) ;
        p24_v_acc_SET((uint32_t)886351382L, &PH) ;
        p24_lat_SET((int32_t)1150299977, PH.base.pack) ;
        p24_alt_SET((int32_t)1848542608, PH.base.pack) ;
        p24_lon_SET((int32_t)191648058, PH.base.pack) ;
        p24_h_acc_SET((uint32_t)4013204295L, &PH) ;
        p24_vel_SET((uint16_t)(uint16_t)7037, PH.base.pack) ;
        p24_alt_ellipsoid_SET((int32_t)397157427, &PH) ;
        p24_eph_SET((uint16_t)(uint16_t)1125, PH.base.pack) ;
        p24_hdg_acc_SET((uint32_t)1703123430L, &PH) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
        p24_time_usec_SET((uint64_t)489585060487090865L, PH.base.pack) ;
        c_TEST_Channel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_prn[] =  {(uint8_t)13, (uint8_t)95, (uint8_t)72, (uint8_t)253, (uint8_t)169, (uint8_t)115, (uint8_t)251, (uint8_t)9, (uint8_t)81, (uint8_t)73, (uint8_t)159, (uint8_t)152, (uint8_t)34, (uint8_t)145, (uint8_t)238, (uint8_t)147, (uint8_t)235, (uint8_t)96, (uint8_t)110, (uint8_t)171};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_used[] =  {(uint8_t)109, (uint8_t)23, (uint8_t)30, (uint8_t)55, (uint8_t)98, (uint8_t)75, (uint8_t)163, (uint8_t)233, (uint8_t)42, (uint8_t)95, (uint8_t)58, (uint8_t)27, (uint8_t)211, (uint8_t)193, (uint8_t)241, (uint8_t)100, (uint8_t)175, (uint8_t)210, (uint8_t)202, (uint8_t)214};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)123, (uint8_t)172, (uint8_t)249, (uint8_t)26, (uint8_t)194, (uint8_t)174, (uint8_t)72, (uint8_t)46, (uint8_t)83, (uint8_t)235, (uint8_t)142, (uint8_t)153, (uint8_t)4, (uint8_t)17, (uint8_t)166, (uint8_t)134, (uint8_t)198, (uint8_t)236, (uint8_t)38, (uint8_t)242};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_snr[] =  {(uint8_t)89, (uint8_t)103, (uint8_t)245, (uint8_t)107, (uint8_t)141, (uint8_t)41, (uint8_t)102, (uint8_t)83, (uint8_t)76, (uint8_t)146, (uint8_t)173, (uint8_t)153, (uint8_t)139, (uint8_t)198, (uint8_t)246, (uint8_t)96, (uint8_t)60, (uint8_t)174, (uint8_t)48, (uint8_t)144};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)185, (uint8_t)143, (uint8_t)77, (uint8_t)205, (uint8_t)158, (uint8_t)0, (uint8_t)57, (uint8_t)254, (uint8_t)224, (uint8_t)128, (uint8_t)43, (uint8_t)159, (uint8_t)131, (uint8_t)185, (uint8_t)47, (uint8_t)137, (uint8_t)133, (uint8_t)7, (uint8_t)133, (uint8_t)187};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        c_TEST_Channel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
        p26_zgyro_SET((int16_t)(int16_t)21570, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t)9882, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t) -27429, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t) -30239, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t)9301, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)1107123120L, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t)20629, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t)26156, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t) -14888, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t)9672, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
        p27_yacc_SET((int16_t)(int16_t) -18437, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t)5838, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t)14143, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t)26488, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t) -8271, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t)1157, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t) -31333, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t) -31877, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)6558650088664402933L, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t)26616, PH.base.pack) ;
        c_TEST_Channel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
        p28_press_diff1_SET((int16_t)(int16_t) -4560, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t) -15361, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)2258844833674639837L, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t)11720, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t)4041, PH.base.pack) ;
        c_TEST_Channel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_temperature_SET((int16_t)(int16_t) -5652, PH.base.pack) ;
        p29_press_diff_SET((float) -2.3917464E37F, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)782945827L, PH.base.pack) ;
        p29_press_abs_SET((float)1.6443614E38F, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
        p30_pitchspeed_SET((float)2.9595143E38F, PH.base.pack) ;
        p30_yaw_SET((float) -9.207099E37F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)1029576882L, PH.base.pack) ;
        p30_rollspeed_SET((float) -1.3323691E38F, PH.base.pack) ;
        p30_yawspeed_SET((float)5.6336847E37F, PH.base.pack) ;
        p30_roll_SET((float) -5.592275E37F, PH.base.pack) ;
        p30_pitch_SET((float)3.2918154E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_rollspeed_SET((float)1.0797617E38F, PH.base.pack) ;
        p31_q3_SET((float) -2.2432566E37F, PH.base.pack) ;
        p31_yawspeed_SET((float)3.1840554E38F, PH.base.pack) ;
        p31_q1_SET((float)2.3842185E38F, PH.base.pack) ;
        p31_q2_SET((float) -3.1337043E38F, PH.base.pack) ;
        p31_q4_SET((float)1.5370253E38F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)1459240154L, PH.base.pack) ;
        p31_pitchspeed_SET((float)2.286827E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_vx_SET((float) -6.4141604E37F, PH.base.pack) ;
        p32_x_SET((float)1.1443959E38F, PH.base.pack) ;
        p32_y_SET((float)1.3788593E38F, PH.base.pack) ;
        p32_vz_SET((float) -3.0860017E38F, PH.base.pack) ;
        p32_z_SET((float) -2.8268264E38F, PH.base.pack) ;
        p32_vy_SET((float) -6.7851044E37F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)2040140604L, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_vy_SET((int16_t)(int16_t)2808, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t) -19585, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)1867922209L, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t) -19679, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)14977, PH.base.pack) ;
        p33_lon_SET((int32_t)933980537, PH.base.pack) ;
        p33_lat_SET((int32_t)1985202301, PH.base.pack) ;
        p33_relative_alt_SET((int32_t)559728038, PH.base.pack) ;
        p33_alt_SET((int32_t) -1477720047, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan3_scaled_SET((int16_t)(int16_t) -29587, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t)8031, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t)12468, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)3386437292L, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t)27810, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t) -15276, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t) -2433, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t) -8739, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t)23434, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_chan2_raw_SET((uint16_t)(uint16_t)18768, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)45764, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)4182, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)22695, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)8793, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)43629, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)35116, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)3215154141L, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)7069, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo16_raw_SET((uint16_t)(uint16_t)3178, &PH) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)64190, PH.base.pack) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)36582, &PH) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)62708, &PH) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)37382, PH.base.pack) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)28607, PH.base.pack) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)15953, &PH) ;
        p36_port_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        p36_time_usec_SET((uint32_t)826438336L, PH.base.pack) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)17626, &PH) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)44098, &PH) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)58850, PH.base.pack) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)36886, PH.base.pack) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)3557, PH.base.pack) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)55867, &PH) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)59062, PH.base.pack) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)63085, PH.base.pack) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)5583, &PH) ;
        c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_target_component_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t) -11171, PH.base.pack) ;
        p37_start_index_SET((int16_t)(int16_t)6267, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_end_index_SET((int16_t)(int16_t) -6812, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p38_target_component_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t) -8927, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
        p39_param3_SET((float) -6.5976904E36F, PH.base.pack) ;
        p39_param2_SET((float) -1.8720346E38F, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p39_param1_SET((float) -3.4026229E38F, PH.base.pack) ;
        p39_x_SET((float)1.4855484E38F, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)45784, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        p39_y_SET((float) -3.1151514E38F, PH.base.pack) ;
        p39_z_SET((float)2.5635363E38F, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE, PH.base.pack) ;
        p39_param4_SET((float) -5.6516295E37F, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
        p40_seq_SET((uint16_t)(uint16_t)43625, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p40_target_component_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_seq_SET((uint16_t)(uint16_t)33708, PH.base.pack) ;
        p41_target_system_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)3319, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p43_target_system_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        p44_count_SET((uint16_t)(uint16_t)56952, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p45_target_system_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)6133, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_target_system_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)1171705631826896181L, &PH) ;
        p48_longitude_SET((int32_t) -648766259, PH.base.pack) ;
        p48_latitude_SET((int32_t)1929650765, PH.base.pack) ;
        p48_altitude_SET((int32_t) -439535236, PH.base.pack) ;
        c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_altitude_SET((int32_t) -1965921632, PH.base.pack) ;
        p49_time_usec_SET((uint64_t)7945965014788591334L, &PH) ;
        p49_longitude_SET((int32_t) -1644094725, PH.base.pack) ;
        p49_latitude_SET((int32_t)1075444354, PH.base.pack) ;
        c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t) -29043, PH.base.pack) ;
        p50_param_value_min_SET((float) -3.3122585E38F, PH.base.pack) ;
        p50_param_value_max_SET((float) -7.9620186E37F, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p50_param_value0_SET((float)2.9063934E38F, PH.base.pack) ;
        p50_scale_SET((float)3.2832978E38F, PH.base.pack) ;
        {
            char16_t* param_id = u"gnwzJuKOlyOzfz";
            p50_param_id_SET_(param_id, &PH) ;
        }
        c_TEST_Channel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_target_system_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)51695, PH.base.pack) ;
        p51_target_component_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_p2z_SET((float) -1.92587E38F, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
        p54_p2x_SET((float) -1.9187896E37F, PH.base.pack) ;
        p54_p2y_SET((float) -1.5047473E38F, PH.base.pack) ;
        p54_p1x_SET((float) -1.5841163E38F, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        p54_p1y_SET((float)7.597769E37F, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p54_p1z_SET((float)1.02081875E37F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p2x_SET((float) -2.1535074E38F, PH.base.pack) ;
        p55_p2y_SET((float)4.9026E37F, PH.base.pack) ;
        p55_p2z_SET((float) -1.717403E38F, PH.base.pack) ;
        p55_p1x_SET((float)1.9601818E37F, PH.base.pack) ;
        p55_p1y_SET((float) -1.4951904E38F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p55_p1z_SET((float) -8.39649E37F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_pitchspeed_SET((float) -2.2131336E38F, PH.base.pack) ;
        {
            float covariance[] =  {-2.8318418E38F, 3.3205086E37F, 1.4015373E38F, 1.349833E38F, 1.9698682E38F, -1.7776913E38F, -3.199769E37F, 1.5409817E38F, -2.6331293E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p61_rollspeed_SET((float)3.0542587E38F, PH.base.pack) ;
        {
            float q[] =  {2.8786952E38F, 2.6343528E38F, 1.4033174E38F, -2.1673083E38F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        p61_time_usec_SET((uint64_t)5638221047954871755L, PH.base.pack) ;
        p61_yawspeed_SET((float) -1.9852698E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_xtrack_error_SET((float) -3.3232049E38F, PH.base.pack) ;
        p62_nav_pitch_SET((float) -1.7950261E38F, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t) -26296, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t)27699, PH.base.pack) ;
        p62_nav_roll_SET((float)1.0540867E38F, PH.base.pack) ;
        p62_alt_error_SET((float) -1.498783E38F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)59091, PH.base.pack) ;
        p62_aspd_error_SET((float)5.1164924E37F, PH.base.pack) ;
        c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_alt_SET((int32_t)693072637, PH.base.pack) ;
        p63_vx_SET((float)2.3686826E38F, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)5746617745371498607L, PH.base.pack) ;
        p63_vy_SET((float)2.0108425E38F, PH.base.pack) ;
        p63_relative_alt_SET((int32_t) -689932774, PH.base.pack) ;
        p63_lat_SET((int32_t)1505905311, PH.base.pack) ;
        p63_lon_SET((int32_t) -47995421, PH.base.pack) ;
        p63_vz_SET((float)1.788118E38F, PH.base.pack) ;
        {
            float covariance[] =  {2.3833388E37F, 1.3746884E38F, -1.4937081E38F, -1.4570432E38F, 1.60349E38F, 2.433372E38F, -3.2628846E38F, 1.252493E38F, 1.6574042E38F, -2.1655537E38F, -9.229247E37F, -8.801982E37F, -1.1243764E38F, -2.6780616E38F, -2.1078884E38F, -1.4997842E38F, -1.382466E38F, -1.3883401E38F, 1.2513695E37F, -1.5024032E38F, 1.3006159E37F, 2.207462E38F, 2.3234689E38F, 7.0669376E37F, 1.0887308E37F, -2.1577216E38F, 1.7797693E38F, 3.7498537E37F, 1.4815309E38F, -6.285194E36F, -1.7480086E38F, -8.1606503E37F, 2.6869477E38F, 2.782944E38F, -1.6194425E38F, 3.7219705E37F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_y_SET((float) -1.2058434E38F, PH.base.pack) ;
        {
            float covariance[] =  {3.2434792E38F, -2.805835E38F, -3.078012E38F, 6.1354355E37F, -2.9334632E38F, 1.4653085E38F, -2.7373886E38F, 2.4631744E36F, -2.532949E38F, 2.6701314E38F, 2.8592472E38F, 1.4941785E38F, 1.3369799E38F, 2.5635597E38F, -4.0605866E37F, -1.2658818E38F, -9.278254E37F, 2.9769243E38F, -1.5026973E38F, -1.0794794E38F, -1.9744715E38F, 1.138759E38F, -2.2117605E38F, -2.2187555E38F, -1.7519745E38F, -2.2417291E38F, -3.6966094E37F, -2.0648824E38F, -2.9834895E38F, -2.5459833E38F, -1.9474214E38F, 2.6842185E38F, -2.78365E38F, 1.6646806E38F, 5.276777E37F, 1.169241E38F, -1.3575242E38F, 1.0404515E38F, -1.770447E38F, 1.2668533E38F, 3.3800362E38F, -2.7985258E37F, -1.9271944E38F, -3.0171914E38F, -8.509902E37F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_ay_SET((float) -3.0639413E38F, PH.base.pack) ;
        p64_vy_SET((float)2.2598227E38F, PH.base.pack) ;
        p64_vz_SET((float)1.4129925E38F, PH.base.pack) ;
        p64_z_SET((float) -3.2615638E38F, PH.base.pack) ;
        p64_vx_SET((float) -2.3671337E38F, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)1112520916112944277L, PH.base.pack) ;
        p64_az_SET((float) -2.6551828E38F, PH.base.pack) ;
        p64_x_SET((float)1.2210894E38F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, PH.base.pack) ;
        p64_ax_SET((float) -7.753714E36F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan14_raw_SET((uint16_t)(uint16_t)7773, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)23048, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)62191, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)30844, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)21524, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)20435, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)57843, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)1337, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)52921, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)44673, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)18884, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)52798, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)3914136336L, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)47474, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)47716, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)27636, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)20012, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)21873, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)65196, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_req_stream_id_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        p66_req_message_rate_SET((uint16_t)(uint16_t)64785, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        c_TEST_Channel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
        p67_message_rate_SET((uint16_t)(uint16_t)39033, PH.base.pack) ;
        p67_on_off_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
        p67_stream_id_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        c_TEST_Channel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_buttons_SET((uint16_t)(uint16_t)19918, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t) -23, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t)1217, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t) -12829, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t)17789, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan4_raw_SET((uint16_t)(uint16_t)47535, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)60482, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)14568, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)14589, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)31513, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)50017, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)20552, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)9736, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_param1_SET((float)4.3593155E37F, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        p73_param3_SET((float) -6.257039E36F, PH.base.pack) ;
        p73_x_SET((int32_t) -174836914, PH.base.pack) ;
        p73_param4_SET((float) -2.9718294E38F, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
        p73_y_SET((int32_t)1173590344, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)59349, PH.base.pack) ;
        p73_z_SET((float) -1.6869311E37F, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_DO_PARACHUTE, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p73_param2_SET((float) -3.2344294E38F, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
        p74_climb_SET((float)3.0677372E38F, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)1142, PH.base.pack) ;
        p74_groundspeed_SET((float)2.474558E38F, PH.base.pack) ;
        p74_airspeed_SET((float) -3.176772E38F, PH.base.pack) ;
        p74_alt_SET((float)5.2667285E36F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t) -203, PH.base.pack) ;
        c_TEST_Channel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
        p75_current_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        p75_param3_SET((float)2.6540713E38F, PH.base.pack) ;
        p75_param2_SET((float) -8.1507905E37F, PH.base.pack) ;
        p75_param1_SET((float)1.202205E38F, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p75_y_SET((int32_t) -146403611, PH.base.pack) ;
        p75_z_SET((float) -1.6204021E38F, PH.base.pack) ;
        p75_param4_SET((float) -1.8541668E38F, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_DO_GO_AROUND, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p75_x_SET((int32_t)2117381653, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        c_CommunicationChannel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
        p76_target_component_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        p76_param6_SET((float)1.724867E38F, PH.base.pack) ;
        p76_param2_SET((float) -1.517006E38F, PH.base.pack) ;
        p76_param1_SET((float)1.250562E38F, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_REVERSE, PH.base.pack) ;
        p76_param7_SET((float) -5.4524365E36F, PH.base.pack) ;
        p76_param5_SET((float) -8.455628E37F, PH.base.pack) ;
        p76_param4_SET((float) -9.84939E37F, PH.base.pack) ;
        p76_param3_SET((float)3.2648745E38F, PH.base.pack) ;
        c_CommunicationChannel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
        p77_target_component_SET((uint8_t)(uint8_t)38, &PH) ;
        p77_progress_SET((uint8_t)(uint8_t)62, &PH) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO, PH.base.pack) ;
        p77_target_system_SET((uint8_t)(uint8_t)132, &PH) ;
        p77_result_param2_SET((int32_t) -1949085523, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_FAILED, PH.base.pack) ;
        c_CommunicationChannel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_time_boot_ms_SET((uint32_t)2534815440L, PH.base.pack) ;
        p81_roll_SET((float) -1.5309617E38F, PH.base.pack) ;
        p81_thrust_SET((float) -2.0988093E38F, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        p81_yaw_SET((float) -1.0925911E38F, PH.base.pack) ;
        p81_pitch_SET((float)2.6586868E38F, PH.base.pack) ;
        c_CommunicationChannel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_time_boot_ms_SET((uint32_t)2176837536L, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        {
            float q[] =  {-2.2303243E38F, 1.6805944E38F, -2.7409383E38F, -2.2500259E38F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_body_pitch_rate_SET((float) -1.619438E38F, PH.base.pack) ;
        p82_body_roll_rate_SET((float)1.3006595E38F, PH.base.pack) ;
        p82_thrust_SET((float)2.1963473E38F, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p82_body_yaw_rate_SET((float)2.1979374E38F, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_type_mask_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p83_thrust_SET((float) -1.8285475E38F, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)1519205346L, PH.base.pack) ;
        p83_body_yaw_rate_SET((float)6.027938E37F, PH.base.pack) ;
        p83_body_pitch_rate_SET((float)1.1433144E38F, PH.base.pack) ;
        {
            float q[] =  {-2.4863413E38F, 1.7783635E38F, 2.8923116E38F, -1.6423628E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_body_roll_rate_SET((float) -1.8181422E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_yaw_rate_SET((float) -3.278856E38F, PH.base.pack) ;
        p84_afz_SET((float) -1.3341064E38F, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)14820, PH.base.pack) ;
        p84_vx_SET((float) -1.3978898E38F, PH.base.pack) ;
        p84_x_SET((float)1.0798439E38F, PH.base.pack) ;
        p84_afy_SET((float) -1.2510701E38F, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p84_afx_SET((float)7.7909273E37F, PH.base.pack) ;
        p84_vy_SET((float) -1.6858857E38F, PH.base.pack) ;
        p84_yaw_SET((float) -7.390092E37F, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)669295808L, PH.base.pack) ;
        p84_y_SET((float) -2.2763354E38F, PH.base.pack) ;
        p84_z_SET((float) -1.7125112E38F, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
        p84_vz_SET((float) -1.7885516E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_yaw_rate_SET((float) -1.326441E38F, PH.base.pack) ;
        p86_lat_int_SET((int32_t)2020618561, PH.base.pack) ;
        p86_afz_SET((float)2.8863891E38F, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)57081, PH.base.pack) ;
        p86_afx_SET((float) -8.6996636E36F, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)2183036605L, PH.base.pack) ;
        p86_vy_SET((float)8.364396E37F, PH.base.pack) ;
        p86_vx_SET((float) -1.5663636E38F, PH.base.pack) ;
        p86_vz_SET((float) -4.632008E37F, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        p86_lon_int_SET((int32_t) -1970480173, PH.base.pack) ;
        p86_yaw_SET((float)3.177494E38F, PH.base.pack) ;
        p86_alt_SET((float)3.908946E37F, PH.base.pack) ;
        p86_afy_SET((float) -3.268534E38F, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_yaw_SET((float) -1.3973286E38F, PH.base.pack) ;
        p87_vx_SET((float) -2.540315E38F, PH.base.pack) ;
        p87_vy_SET((float) -1.9743417E38F, PH.base.pack) ;
        p87_afx_SET((float)1.5053882E38F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)41911, PH.base.pack) ;
        p87_lat_int_SET((int32_t) -974670224, PH.base.pack) ;
        p87_alt_SET((float)2.0035461E38F, PH.base.pack) ;
        p87_afy_SET((float)4.751996E37F, PH.base.pack) ;
        p87_afz_SET((float) -2.9126804E38F, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
        p87_yaw_rate_SET((float)2.5311378E38F, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)575635555L, PH.base.pack) ;
        p87_vz_SET((float)1.988964E38F, PH.base.pack) ;
        p87_lon_int_SET((int32_t)494254310, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_z_SET((float)8.0096103E37F, PH.base.pack) ;
        p89_x_SET((float)7.443984E37F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)1609838539L, PH.base.pack) ;
        p89_pitch_SET((float) -1.893223E38F, PH.base.pack) ;
        p89_roll_SET((float)2.1398119E38F, PH.base.pack) ;
        p89_yaw_SET((float) -1.4282341E38F, PH.base.pack) ;
        p89_y_SET((float) -4.512567E37F, PH.base.pack) ;
        c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
        p90_xacc_SET((int16_t)(int16_t)19685, PH.base.pack) ;
        p90_rollspeed_SET((float)1.416251E38F, PH.base.pack) ;
        p90_yawspeed_SET((float) -8.0690024E36F, PH.base.pack) ;
        p90_pitchspeed_SET((float) -3.398433E38F, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t)21235, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t)21881, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t) -28133, PH.base.pack) ;
        p90_lon_SET((int32_t)1210999742, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)1122552055549011998L, PH.base.pack) ;
        p90_yaw_SET((float) -3.1588437E38F, PH.base.pack) ;
        p90_alt_SET((int32_t)213257976, PH.base.pack) ;
        p90_pitch_SET((float) -2.894036E38F, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t) -28841, PH.base.pack) ;
        p90_roll_SET((float)1.6986081E38F, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t) -26070, PH.base.pack) ;
        p90_lat_SET((int32_t) -317533746, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
        p91_time_usec_SET((uint64_t)5606425515006191345L, PH.base.pack) ;
        p91_aux1_SET((float) -5.7751525E37F, PH.base.pack) ;
        p91_aux4_SET((float) -2.3528354E38F, PH.base.pack) ;
        p91_throttle_SET((float) -9.922321E37F, PH.base.pack) ;
        p91_aux3_SET((float)2.0063422E38F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
        p91_pitch_elevator_SET((float)1.2404847E38F, PH.base.pack) ;
        p91_roll_ailerons_SET((float) -3.7481396E37F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_ARMED, PH.base.pack) ;
        p91_aux2_SET((float) -2.5787187E38F, PH.base.pack) ;
        p91_yaw_rudder_SET((float)2.84424E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan3_raw_SET((uint16_t)(uint16_t)64207, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)57146, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)34679, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)4294, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)9563, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)5079969898288747337L, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)35204, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)34921, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)7375, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)40013, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)16187, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)64543, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)11724, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_time_usec_SET((uint64_t)4367998290382085827L, PH.base.pack) ;
        p93_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_DISARMED, PH.base.pack) ;
        p93_flags_SET((uint64_t)7909580135777052058L, PH.base.pack) ;
        {
            float controls[] =  {8.036061E37F, -1.320447E38F, 1.4296999E38F, -2.7671354E38F, -6.1011106E37F, 2.4628303E38F, -1.7844267E38F, 3.2655418E38F, 4.928114E37F, -1.1716481E38F, 2.337485E38F, -5.8273847E37F, -2.3523557E38F, 9.353736E37F, 9.278536E37F, -2.8856119E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_ground_distance_SET((float)2.576953E38F, PH.base.pack) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        p100_flow_x_SET((int16_t)(int16_t) -9872, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)5921680326004748660L, PH.base.pack) ;
        p100_flow_rate_x_SET((float) -2.5508353E38F, &PH) ;
        p100_flow_y_SET((int16_t)(int16_t) -19810, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float) -1.4183449E38F, PH.base.pack) ;
        p100_flow_rate_y_SET((float)1.6789105E38F, &PH) ;
        p100_flow_comp_m_y_SET((float) -3.2208583E37F, PH.base.pack) ;
        p100_quality_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_yaw_SET((float)2.2818116E38F, PH.base.pack) ;
        p101_usec_SET((uint64_t)4179899387342405931L, PH.base.pack) ;
        p101_y_SET((float)9.431476E37F, PH.base.pack) ;
        p101_roll_SET((float) -3.060404E38F, PH.base.pack) ;
        p101_x_SET((float) -1.0687994E38F, PH.base.pack) ;
        p101_z_SET((float) -1.5236849E38F, PH.base.pack) ;
        p101_pitch_SET((float) -1.6527336E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_x_SET((float)1.715313E38F, PH.base.pack) ;
        p102_z_SET((float) -2.4419198E38F, PH.base.pack) ;
        p102_yaw_SET((float) -2.4317139E38F, PH.base.pack) ;
        p102_usec_SET((uint64_t)2711917128324492808L, PH.base.pack) ;
        p102_roll_SET((float)2.273977E38F, PH.base.pack) ;
        p102_y_SET((float)2.028872E38F, PH.base.pack) ;
        p102_pitch_SET((float) -2.6896386E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_x_SET((float)1.1549646E38F, PH.base.pack) ;
        p103_usec_SET((uint64_t)7769795264581813088L, PH.base.pack) ;
        p103_z_SET((float) -4.447912E37F, PH.base.pack) ;
        p103_y_SET((float)8.200668E37F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_roll_SET((float) -2.3967188E38F, PH.base.pack) ;
        p104_z_SET((float)6.120884E37F, PH.base.pack) ;
        p104_y_SET((float) -5.677991E37F, PH.base.pack) ;
        p104_pitch_SET((float)6.0693894E36F, PH.base.pack) ;
        p104_x_SET((float) -6.1707994E37F, PH.base.pack) ;
        p104_yaw_SET((float)4.287942E37F, PH.base.pack) ;
        p104_usec_SET((uint64_t)3539996105022319989L, PH.base.pack) ;
        c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
        p105_zmag_SET((float)4.501989E37F, PH.base.pack) ;
        p105_ygyro_SET((float) -3.0244103E38F, PH.base.pack) ;
        p105_pressure_alt_SET((float)3.348652E38F, PH.base.pack) ;
        p105_abs_pressure_SET((float)1.4893191E38F, PH.base.pack) ;
        p105_temperature_SET((float)3.2901676E37F, PH.base.pack) ;
        p105_yacc_SET((float)5.063806E37F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)3255498571451490362L, PH.base.pack) ;
        p105_zacc_SET((float)1.4645726E38F, PH.base.pack) ;
        p105_zgyro_SET((float) -3.282926E38F, PH.base.pack) ;
        p105_xacc_SET((float) -3.2456219E37F, PH.base.pack) ;
        p105_diff_pressure_SET((float)1.8220478E38F, PH.base.pack) ;
        p105_xgyro_SET((float)5.0066195E37F, PH.base.pack) ;
        p105_xmag_SET((float) -1.3381293E38F, PH.base.pack) ;
        p105_ymag_SET((float) -4.5442947E37F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)4928, PH.base.pack) ;
        c_CommunicationChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_integrated_y_SET((float) -2.87381E37F, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t)15457, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)9011521324116247634L, PH.base.pack) ;
        p106_integrated_x_SET((float)1.6637992E38F, PH.base.pack) ;
        p106_distance_SET((float)6.2195025E37F, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        p106_integrated_ygyro_SET((float) -2.233021E38F, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)4056833644L, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)2815419925L, PH.base.pack) ;
        p106_integrated_zgyro_SET((float) -2.633513E38F, PH.base.pack) ;
        p106_integrated_xgyro_SET((float)1.2280298E38F, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
        p107_abs_pressure_SET((float) -3.1643293E38F, PH.base.pack) ;
        p107_diff_pressure_SET((float) -1.5038215E38F, PH.base.pack) ;
        p107_zmag_SET((float)8.422731E37F, PH.base.pack) ;
        p107_yacc_SET((float)2.882824E38F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)1764122671L, PH.base.pack) ;
        p107_xmag_SET((float) -2.3477372E38F, PH.base.pack) ;
        p107_zgyro_SET((float)2.7301812E37F, PH.base.pack) ;
        p107_temperature_SET((float)2.3617793E38F, PH.base.pack) ;
        p107_xacc_SET((float)2.8090121E38F, PH.base.pack) ;
        p107_pressure_alt_SET((float)2.6091517E37F, PH.base.pack) ;
        p107_ygyro_SET((float)1.2787965E38F, PH.base.pack) ;
        p107_zacc_SET((float) -2.4118485E38F, PH.base.pack) ;
        p107_xgyro_SET((float) -2.416658E38F, PH.base.pack) ;
        p107_ymag_SET((float) -1.3913354E38F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)5942525826219682035L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
        p108_lon_SET((float)6.7853843E37F, PH.base.pack) ;
        p108_zgyro_SET((float) -3.251642E38F, PH.base.pack) ;
        p108_ygyro_SET((float)6.5132603E37F, PH.base.pack) ;
        p108_xgyro_SET((float)1.2881845E36F, PH.base.pack) ;
        p108_pitch_SET((float)1.2409822E38F, PH.base.pack) ;
        p108_yacc_SET((float)1.4486945E38F, PH.base.pack) ;
        p108_std_dev_vert_SET((float)1.4130506E38F, PH.base.pack) ;
        p108_vn_SET((float) -1.6539874E38F, PH.base.pack) ;
        p108_yaw_SET((float) -2.0424504E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float)1.7523162E38F, PH.base.pack) ;
        p108_xacc_SET((float)2.2422512E38F, PH.base.pack) ;
        p108_vd_SET((float) -1.6448991E38F, PH.base.pack) ;
        p108_q1_SET((float)2.0695345E38F, PH.base.pack) ;
        p108_alt_SET((float)3.490629E37F, PH.base.pack) ;
        p108_zacc_SET((float) -1.8599929E38F, PH.base.pack) ;
        p108_q4_SET((float)2.0574332E38F, PH.base.pack) ;
        p108_q3_SET((float)7.1240934E37F, PH.base.pack) ;
        p108_q2_SET((float)3.0114851E38F, PH.base.pack) ;
        p108_ve_SET((float) -4.4151074E37F, PH.base.pack) ;
        p108_roll_SET((float) -2.3902777E37F, PH.base.pack) ;
        p108_lat_SET((float) -2.0244494E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
        p109_noise_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)63417, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)12081, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        c_CommunicationChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        {
            uint8_t payload[] =  {(uint8_t)27, (uint8_t)128, (uint8_t)6, (uint8_t)44, (uint8_t)200, (uint8_t)59, (uint8_t)201, (uint8_t)121, (uint8_t)46, (uint8_t)173, (uint8_t)60, (uint8_t)200, (uint8_t)177, (uint8_t)202, (uint8_t)232, (uint8_t)214, (uint8_t)30, (uint8_t)77, (uint8_t)133, (uint8_t)125, (uint8_t)225, (uint8_t)232, (uint8_t)0, (uint8_t)179, (uint8_t)178, (uint8_t)79, (uint8_t)183, (uint8_t)124, (uint8_t)218, (uint8_t)57, (uint8_t)250, (uint8_t)190, (uint8_t)202, (uint8_t)48, (uint8_t)178, (uint8_t)17, (uint8_t)56, (uint8_t)72, (uint8_t)9, (uint8_t)249, (uint8_t)173, (uint8_t)39, (uint8_t)134, (uint8_t)45, (uint8_t)255, (uint8_t)100, (uint8_t)254, (uint8_t)66, (uint8_t)105, (uint8_t)80, (uint8_t)112, (uint8_t)155, (uint8_t)79, (uint8_t)114, (uint8_t)108, (uint8_t)3, (uint8_t)165, (uint8_t)179, (uint8_t)73, (uint8_t)136, (uint8_t)220, (uint8_t)212, (uint8_t)28, (uint8_t)234, (uint8_t)147, (uint8_t)208, (uint8_t)190, (uint8_t)39, (uint8_t)158, (uint8_t)42, (uint8_t)157, (uint8_t)130, (uint8_t)184, (uint8_t)120, (uint8_t)104, (uint8_t)125, (uint8_t)122, (uint8_t)249, (uint8_t)14, (uint8_t)121, (uint8_t)222, (uint8_t)83, (uint8_t)229, (uint8_t)220, (uint8_t)146, (uint8_t)76, (uint8_t)86, (uint8_t)174, (uint8_t)144, (uint8_t)14, (uint8_t)78, (uint8_t)249, (uint8_t)247, (uint8_t)227, (uint8_t)8, (uint8_t)184, (uint8_t)131, (uint8_t)234, (uint8_t)49, (uint8_t)208, (uint8_t)171, (uint8_t)251, (uint8_t)42, (uint8_t)141, (uint8_t)232, (uint8_t)89, (uint8_t)45, (uint8_t)49, (uint8_t)5, (uint8_t)132, (uint8_t)2, (uint8_t)128, (uint8_t)84, (uint8_t)118, (uint8_t)252, (uint8_t)223, (uint8_t)249, (uint8_t)59, (uint8_t)136, (uint8_t)203, (uint8_t)52, (uint8_t)47, (uint8_t)173, (uint8_t)131, (uint8_t)178, (uint8_t)72, (uint8_t)4, (uint8_t)34, (uint8_t)93, (uint8_t)38, (uint8_t)233, (uint8_t)53, (uint8_t)91, (uint8_t)100, (uint8_t)244, (uint8_t)248, (uint8_t)8, (uint8_t)22, (uint8_t)6, (uint8_t)141, (uint8_t)232, (uint8_t)27, (uint8_t)24, (uint8_t)175, (uint8_t)11, (uint8_t)3, (uint8_t)44, (uint8_t)197, (uint8_t)217, (uint8_t)21, (uint8_t)106, (uint8_t)202, (uint8_t)24, (uint8_t)150, (uint8_t)255, (uint8_t)4, (uint8_t)50, (uint8_t)192, (uint8_t)174, (uint8_t)34, (uint8_t)182, (uint8_t)253, (uint8_t)173, (uint8_t)168, (uint8_t)232, (uint8_t)21, (uint8_t)92, (uint8_t)154, (uint8_t)105, (uint8_t)141, (uint8_t)227, (uint8_t)204, (uint8_t)210, (uint8_t)30, (uint8_t)81, (uint8_t)225, (uint8_t)112, (uint8_t)175, (uint8_t)115, (uint8_t)184, (uint8_t)219, (uint8_t)218, (uint8_t)82, (uint8_t)149, (uint8_t)214, (uint8_t)126, (uint8_t)112, (uint8_t)55, (uint8_t)214, (uint8_t)237, (uint8_t)71, (uint8_t)26, (uint8_t)214, (uint8_t)13, (uint8_t)255, (uint8_t)120, (uint8_t)199, (uint8_t)1, (uint8_t)8, (uint8_t)116, (uint8_t)57, (uint8_t)190, (uint8_t)233, (uint8_t)73, (uint8_t)105, (uint8_t)33, (uint8_t)224, (uint8_t)61, (uint8_t)62, (uint8_t)205, (uint8_t)1, (uint8_t)202, (uint8_t)235, (uint8_t)207, (uint8_t)56, (uint8_t)49, (uint8_t)169, (uint8_t)49, (uint8_t)193, (uint8_t)103, (uint8_t)116, (uint8_t)222, (uint8_t)150, (uint8_t)20, (uint8_t)240, (uint8_t)113, (uint8_t)78, (uint8_t)131, (uint8_t)126, (uint8_t)84, (uint8_t)183, (uint8_t)24, (uint8_t)17, (uint8_t)214, (uint8_t)103, (uint8_t)153, (uint8_t)38, (uint8_t)158, (uint8_t)111, (uint8_t)110, (uint8_t)101, (uint8_t)82, (uint8_t)165, (uint8_t)13, (uint8_t)220, (uint8_t)3, (uint8_t)62, (uint8_t)177, (uint8_t)212, (uint8_t)255, (uint8_t)132};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_network_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        p110_target_component_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
        p110_target_system_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
        p111_ts1_SET((int64_t)8779977823302329070L, PH.base.pack) ;
        p111_tc1_SET((int64_t) -2998649128465505638L, PH.base.pack) ;
        c_CommunicationChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_time_usec_SET((uint64_t)6730606133999373695L, PH.base.pack) ;
        p112_seq_SET((uint32_t)3736564981L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
        p113_ve_SET((int16_t)(int16_t) -10949, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)48780, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)238102105641123289L, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t)11990, PH.base.pack) ;
        p113_lat_SET((int32_t) -1689269909, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t) -6879, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)20331, PH.base.pack) ;
        p113_lon_SET((int32_t) -1615754538, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)64929, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)6569, PH.base.pack) ;
        p113_alt_SET((int32_t) -1553749874, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_integrated_y_SET((float) -4.8975204E37F, PH.base.pack) ;
        p114_integrated_xgyro_SET((float)5.567848E37F, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)2844480677L, PH.base.pack) ;
        p114_integrated_ygyro_SET((float)1.7690458E38F, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t)20952, PH.base.pack) ;
        p114_integrated_zgyro_SET((float) -1.3715837E38F, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)4253019998L, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)1944861652753895095L, PH.base.pack) ;
        p114_distance_SET((float) -3.8545467E37F, PH.base.pack) ;
        p114_integrated_x_SET((float) -2.1167248E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_xacc_SET((int16_t)(int16_t) -18673, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t) -8933, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t) -2239, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)13785, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {-1.6560066E38F, -2.0447293E38F, 2.2073389E38F, 2.9812132E38F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_lon_SET((int32_t)1514642759, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t) -7118, PH.base.pack) ;
        p115_rollspeed_SET((float) -3.771207E37F, PH.base.pack) ;
        p115_alt_SET((int32_t)1462838358, PH.base.pack) ;
        p115_lat_SET((int32_t) -311211727, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)62338, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t) -25268, PH.base.pack) ;
        p115_yawspeed_SET((float)2.2699738E38F, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t)30197, PH.base.pack) ;
        p115_pitchspeed_SET((float) -2.1211555E38F, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)5428906822465152735L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
        p116_time_boot_ms_SET((uint32_t)3036171229L, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t) -16669, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t) -25655, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t)6895, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t) -3308, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t)32005, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t)14711, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t)160, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t) -20336, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t) -23237, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_target_system_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)16818, PH.base.pack) ;
        p117_end_SET((uint16_t)(uint16_t)41069, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
        p118_last_log_num_SET((uint16_t)(uint16_t)49209, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)741824610L, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)33440, PH.base.pack) ;
        p118_size_SET((uint32_t)1453734412L, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)18142, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_id_SET((uint16_t)(uint16_t)24254, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p119_count_SET((uint32_t)1843728L, PH.base.pack) ;
        p119_ofs_SET((uint32_t)1647046926L, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
        p120_count_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)28, (uint8_t)239, (uint8_t)123, (uint8_t)0, (uint8_t)24, (uint8_t)81, (uint8_t)59, (uint8_t)225, (uint8_t)1, (uint8_t)155, (uint8_t)153, (uint8_t)71, (uint8_t)136, (uint8_t)38, (uint8_t)76, (uint8_t)201, (uint8_t)209, (uint8_t)184, (uint8_t)66, (uint8_t)88, (uint8_t)9, (uint8_t)128, (uint8_t)219, (uint8_t)32, (uint8_t)20, (uint8_t)246, (uint8_t)72, (uint8_t)137, (uint8_t)39, (uint8_t)44, (uint8_t)114, (uint8_t)41, (uint8_t)160, (uint8_t)146, (uint8_t)52, (uint8_t)167, (uint8_t)58, (uint8_t)216, (uint8_t)199, (uint8_t)52, (uint8_t)31, (uint8_t)102, (uint8_t)5, (uint8_t)231, (uint8_t)5, (uint8_t)2, (uint8_t)183, (uint8_t)63, (uint8_t)41, (uint8_t)93, (uint8_t)250, (uint8_t)253, (uint8_t)146, (uint8_t)83, (uint8_t)182, (uint8_t)203, (uint8_t)197, (uint8_t)222, (uint8_t)195, (uint8_t)92, (uint8_t)123, (uint8_t)191, (uint8_t)194, (uint8_t)3, (uint8_t)35, (uint8_t)178, (uint8_t)107, (uint8_t)156, (uint8_t)20, (uint8_t)7, (uint8_t)46, (uint8_t)201, (uint8_t)38, (uint8_t)249, (uint8_t)68, (uint8_t)84, (uint8_t)135, (uint8_t)164, (uint8_t)15, (uint8_t)72, (uint8_t)196, (uint8_t)211, (uint8_t)140, (uint8_t)12, (uint8_t)98, (uint8_t)87, (uint8_t)125, (uint8_t)40, (uint8_t)133, (uint8_t)58};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        p120_ofs_SET((uint32_t)3906611811L, PH.base.pack) ;
        p120_id_SET((uint16_t)(uint16_t)53697, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
        p121_target_component_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
        p121_target_system_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_component_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        p122_target_system_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_target_system_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        p123_len_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)122, (uint8_t)108, (uint8_t)231, (uint8_t)87, (uint8_t)232, (uint8_t)252, (uint8_t)186, (uint8_t)204, (uint8_t)94, (uint8_t)244, (uint8_t)48, (uint8_t)7, (uint8_t)151, (uint8_t)37, (uint8_t)117, (uint8_t)129, (uint8_t)235, (uint8_t)210, (uint8_t)189, (uint8_t)136, (uint8_t)154, (uint8_t)65, (uint8_t)140, (uint8_t)92, (uint8_t)68, (uint8_t)162, (uint8_t)166, (uint8_t)39, (uint8_t)93, (uint8_t)200, (uint8_t)208, (uint8_t)206, (uint8_t)220, (uint8_t)177, (uint8_t)110, (uint8_t)255, (uint8_t)18, (uint8_t)197, (uint8_t)113, (uint8_t)146, (uint8_t)220, (uint8_t)221, (uint8_t)109, (uint8_t)34, (uint8_t)232, (uint8_t)173, (uint8_t)37, (uint8_t)63, (uint8_t)171, (uint8_t)211, (uint8_t)77, (uint8_t)162, (uint8_t)16, (uint8_t)21, (uint8_t)232, (uint8_t)143, (uint8_t)216, (uint8_t)172, (uint8_t)174, (uint8_t)151, (uint8_t)49, (uint8_t)140, (uint8_t)209, (uint8_t)117, (uint8_t)187, (uint8_t)16, (uint8_t)69, (uint8_t)206, (uint8_t)140, (uint8_t)89, (uint8_t)142, (uint8_t)190, (uint8_t)120, (uint8_t)241, (uint8_t)143, (uint8_t)120, (uint8_t)117, (uint8_t)236, (uint8_t)197, (uint8_t)135, (uint8_t)227, (uint8_t)102, (uint8_t)33, (uint8_t)58, (uint8_t)53, (uint8_t)168, (uint8_t)167, (uint8_t)26, (uint8_t)192, (uint8_t)162, (uint8_t)25, (uint8_t)2, (uint8_t)85, (uint8_t)188, (uint8_t)226, (uint8_t)186, (uint8_t)61, (uint8_t)174, (uint8_t)20, (uint8_t)94, (uint8_t)37, (uint8_t)42, (uint8_t)191, (uint8_t)163, (uint8_t)192, (uint8_t)117, (uint8_t)219, (uint8_t)67, (uint8_t)45, (uint8_t)158};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_target_component_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
        p124_vel_SET((uint16_t)(uint16_t)57590, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)3667789353428624123L, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)19649, PH.base.pack) ;
        p124_lon_SET((int32_t) -679625659, PH.base.pack) ;
        p124_alt_SET((int32_t) -661005024, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)48738, PH.base.pack) ;
        p124_lat_SET((int32_t) -315932364, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)45547, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)1947124945L, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
        p125_Vcc_SET((uint16_t)(uint16_t)21941, PH.base.pack) ;
        p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT, PH.base.pack) ;
        p125_Vservo_SET((uint16_t)(uint16_t)60228, PH.base.pack) ;
        c_CommunicationChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_timeout_SET((uint16_t)(uint16_t)15707, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)143, (uint8_t)85, (uint8_t)98, (uint8_t)249, (uint8_t)7, (uint8_t)11, (uint8_t)3, (uint8_t)34, (uint8_t)235, (uint8_t)34, (uint8_t)180, (uint8_t)185, (uint8_t)64, (uint8_t)66, (uint8_t)239, (uint8_t)178, (uint8_t)21, (uint8_t)14, (uint8_t)110, (uint8_t)194, (uint8_t)143, (uint8_t)105, (uint8_t)191, (uint8_t)160, (uint8_t)186, (uint8_t)35, (uint8_t)117, (uint8_t)100, (uint8_t)75, (uint8_t)30, (uint8_t)171, (uint8_t)29, (uint8_t)223, (uint8_t)193, (uint8_t)58, (uint8_t)162, (uint8_t)119, (uint8_t)109, (uint8_t)233, (uint8_t)205, (uint8_t)98, (uint8_t)244, (uint8_t)84, (uint8_t)153, (uint8_t)4, (uint8_t)237, (uint8_t)117, (uint8_t)117, (uint8_t)137, (uint8_t)155, (uint8_t)142, (uint8_t)100, (uint8_t)108, (uint8_t)44, (uint8_t)178, (uint8_t)159, (uint8_t)249, (uint8_t)193, (uint8_t)16, (uint8_t)11, (uint8_t)47, (uint8_t)163, (uint8_t)205, (uint8_t)209, (uint8_t)248, (uint8_t)210, (uint8_t)178, (uint8_t)182, (uint8_t)21, (uint8_t)204};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_baudrate_SET((uint32_t)1153411085L, PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND, PH.base.pack) ;
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
        p127_nsats_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)1390611867L, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)24993, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t)538821739, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t)512175018, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t) -781356605, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)1350706618L, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t) -538181780, PH.base.pack) ;
        p127_tow_SET((uint32_t)2487833546L, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        p128_tow_SET((uint32_t)3476305333L, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)18687, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t) -563723551, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)967870776L, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t) -1559378638, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t)1029384068, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t) -1410993893, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)3604752308L, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
        p129_time_boot_ms_SET((uint32_t)299080472L, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t)22267, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t)14737, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t) -9544, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t) -30042, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t)6292, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t) -18507, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t) -12965, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t)4995, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t)31784, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_jpg_quality_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)36519, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)45197, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)16481, PH.base.pack) ;
        p130_size_SET((uint32_t)1681865327L, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
        p131_seqnr_SET((uint16_t)(uint16_t)56336, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)54, (uint8_t)86, (uint8_t)64, (uint8_t)143, (uint8_t)138, (uint8_t)168, (uint8_t)178, (uint8_t)171, (uint8_t)154, (uint8_t)13, (uint8_t)140, (uint8_t)7, (uint8_t)221, (uint8_t)31, (uint8_t)108, (uint8_t)173, (uint8_t)195, (uint8_t)157, (uint8_t)138, (uint8_t)113, (uint8_t)72, (uint8_t)82, (uint8_t)49, (uint8_t)117, (uint8_t)243, (uint8_t)181, (uint8_t)97, (uint8_t)56, (uint8_t)7, (uint8_t)169, (uint8_t)202, (uint8_t)40, (uint8_t)146, (uint8_t)198, (uint8_t)84, (uint8_t)187, (uint8_t)217, (uint8_t)137, (uint8_t)147, (uint8_t)133, (uint8_t)113, (uint8_t)106, (uint8_t)42, (uint8_t)239, (uint8_t)244, (uint8_t)30, (uint8_t)214, (uint8_t)108, (uint8_t)183, (uint8_t)148, (uint8_t)100, (uint8_t)35, (uint8_t)73, (uint8_t)49, (uint8_t)81, (uint8_t)115, (uint8_t)139, (uint8_t)75, (uint8_t)104, (uint8_t)107, (uint8_t)82, (uint8_t)240, (uint8_t)155, (uint8_t)183, (uint8_t)51, (uint8_t)208, (uint8_t)145, (uint8_t)73, (uint8_t)195, (uint8_t)62, (uint8_t)127, (uint8_t)224, (uint8_t)48, (uint8_t)216, (uint8_t)65, (uint8_t)29, (uint8_t)223, (uint8_t)110, (uint8_t)76, (uint8_t)245, (uint8_t)164, (uint8_t)83, (uint8_t)18, (uint8_t)105, (uint8_t)44, (uint8_t)217, (uint8_t)97, (uint8_t)167, (uint8_t)92, (uint8_t)23, (uint8_t)220, (uint8_t)128, (uint8_t)181, (uint8_t)154, (uint8_t)44, (uint8_t)117, (uint8_t)14, (uint8_t)149, (uint8_t)49, (uint8_t)78, (uint8_t)197, (uint8_t)189, (uint8_t)216, (uint8_t)39, (uint8_t)219, (uint8_t)180, (uint8_t)112, (uint8_t)182, (uint8_t)87, (uint8_t)7, (uint8_t)125, (uint8_t)239, (uint8_t)172, (uint8_t)104, (uint8_t)71, (uint8_t)165, (uint8_t)236, (uint8_t)135, (uint8_t)168, (uint8_t)78, (uint8_t)151, (uint8_t)173, (uint8_t)227, (uint8_t)245, (uint8_t)132, (uint8_t)146, (uint8_t)183, (uint8_t)5, (uint8_t)33, (uint8_t)66, (uint8_t)221, (uint8_t)151, (uint8_t)118, (uint8_t)60, (uint8_t)165, (uint8_t)143, (uint8_t)40, (uint8_t)44, (uint8_t)163, (uint8_t)24, (uint8_t)70, (uint8_t)208, (uint8_t)33, (uint8_t)73, (uint8_t)232, (uint8_t)60, (uint8_t)20, (uint8_t)243, (uint8_t)206, (uint8_t)186, (uint8_t)92, (uint8_t)22, (uint8_t)210, (uint8_t)28, (uint8_t)88, (uint8_t)52, (uint8_t)82, (uint8_t)108, (uint8_t)28, (uint8_t)34, (uint8_t)101, (uint8_t)175, (uint8_t)170, (uint8_t)138, (uint8_t)118, (uint8_t)109, (uint8_t)4, (uint8_t)52, (uint8_t)3, (uint8_t)127, (uint8_t)68, (uint8_t)226, (uint8_t)124, (uint8_t)193, (uint8_t)146, (uint8_t)21, (uint8_t)41, (uint8_t)168, (uint8_t)124, (uint8_t)209, (uint8_t)206, (uint8_t)133, (uint8_t)239, (uint8_t)191, (uint8_t)70, (uint8_t)213, (uint8_t)198, (uint8_t)105, (uint8_t)252, (uint8_t)16, (uint8_t)167, (uint8_t)176, (uint8_t)11, (uint8_t)43, (uint8_t)159, (uint8_t)21, (uint8_t)127, (uint8_t)35, (uint8_t)19, (uint8_t)89, (uint8_t)102, (uint8_t)191, (uint8_t)94, (uint8_t)208, (uint8_t)243, (uint8_t)62, (uint8_t)100, (uint8_t)141, (uint8_t)0, (uint8_t)132, (uint8_t)18, (uint8_t)220, (uint8_t)180, (uint8_t)170, (uint8_t)35, (uint8_t)12, (uint8_t)50, (uint8_t)208, (uint8_t)186, (uint8_t)167, (uint8_t)29, (uint8_t)32, (uint8_t)116, (uint8_t)248, (uint8_t)219, (uint8_t)100, (uint8_t)181, (uint8_t)50, (uint8_t)209, (uint8_t)52, (uint8_t)72, (uint8_t)108, (uint8_t)247, (uint8_t)239, (uint8_t)1, (uint8_t)236, (uint8_t)134, (uint8_t)152, (uint8_t)72, (uint8_t)98, (uint8_t)191, (uint8_t)7, (uint8_t)44, (uint8_t)185, (uint8_t)27, (uint8_t)74, (uint8_t)91, (uint8_t)198, (uint8_t)30, (uint8_t)102, (uint8_t)132, (uint8_t)206, (uint8_t)229};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)11992, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)58068, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)6641, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)1292646927L, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        c_CommunicationChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_lon_SET((int32_t) -241388805, PH.base.pack) ;
        p133_mask_SET((uint64_t)1680060074204372468L, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)30851, PH.base.pack) ;
        p133_lat_SET((int32_t)1254489976, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
        p134_lat_SET((int32_t)1430054319, PH.base.pack) ;
        p134_lon_SET((int32_t) -1887547875, PH.base.pack) ;
        p134_gridbit_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t) -2281, (int16_t) -22745, (int16_t)15663, (int16_t) -27843, (int16_t) -2280, (int16_t)32376, (int16_t)24789, (int16_t)12137, (int16_t)19510, (int16_t) -20119, (int16_t) -16228, (int16_t) -11941, (int16_t) -13998, (int16_t) -18204, (int16_t)28976, (int16_t)27832};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        p134_grid_spacing_SET((uint16_t)(uint16_t)18280, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lat_SET((int32_t)1463617961, PH.base.pack) ;
        p135_lon_SET((int32_t)653779611, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_spacing_SET((uint16_t)(uint16_t)64532, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)58301, PH.base.pack) ;
        p136_lat_SET((int32_t) -872154463, PH.base.pack) ;
        p136_lon_SET((int32_t) -1198683054, PH.base.pack) ;
        p136_terrain_height_SET((float) -1.3678118E38F, PH.base.pack) ;
        p136_current_height_SET((float) -2.0599026E38F, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)13260, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_temperature_SET((int16_t)(int16_t) -32510, PH.base.pack) ;
        p137_press_diff_SET((float) -3.0184525E38F, PH.base.pack) ;
        p137_time_boot_ms_SET((uint32_t)3781506397L, PH.base.pack) ;
        p137_press_abs_SET((float)1.7846397E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
        {
            float q[] =  {-1.5407952E38F, 2.5466934E38F, 2.0858081E38F, -2.6889853E38F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_y_SET((float)9.496752E37F, PH.base.pack) ;
        p138_z_SET((float)7.1914675E37F, PH.base.pack) ;
        p138_x_SET((float)3.8712825E37F, PH.base.pack) ;
        p138_time_usec_SET((uint64_t)924248566902828115L, PH.base.pack) ;
        c_CommunicationChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_target_component_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        p139_group_mlx_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        {
            float controls[] =  {-4.5218E37F, -3.2530721E38F, -2.097604E38F, -9.317569E36F, 2.551852E38F, 2.3549924E38F, -2.05851E38F, -2.2374964E38F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_time_usec_SET((uint64_t)5260666375115683793L, PH.base.pack) ;
        p139_target_system_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        p140_group_mlx_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        p140_time_usec_SET((uint64_t)6412352912868954294L, PH.base.pack) ;
        {
            float controls[] =  {-1.1421533E38F, -2.6105454E37F, -1.648163E38F, 2.3211461E38F, -1.6707519E38F, -1.0223979E38F, -2.1904262E38F, -2.7077402E37F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ALTITUDE_141(), &PH);
        p141_altitude_relative_SET((float)1.0486672E38F, PH.base.pack) ;
        p141_bottom_clearance_SET((float)9.393969E37F, PH.base.pack) ;
        p141_altitude_local_SET((float) -1.7951156E38F, PH.base.pack) ;
        p141_altitude_monotonic_SET((float)3.1094185E38F, PH.base.pack) ;
        p141_altitude_terrain_SET((float)2.8901693E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)5814206301282074872L, PH.base.pack) ;
        p141_altitude_amsl_SET((float) -1.1498933E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_RESOURCE_REQUEST_142(), &PH);
        p142_request_id_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        {
            uint8_t storage[] =  {(uint8_t)9, (uint8_t)233, (uint8_t)183, (uint8_t)185, (uint8_t)48, (uint8_t)221, (uint8_t)211, (uint8_t)72, (uint8_t)20, (uint8_t)209, (uint8_t)237, (uint8_t)87, (uint8_t)29, (uint8_t)79, (uint8_t)115, (uint8_t)214, (uint8_t)124, (uint8_t)160, (uint8_t)245, (uint8_t)102, (uint8_t)25, (uint8_t)77, (uint8_t)123, (uint8_t)218, (uint8_t)254, (uint8_t)211, (uint8_t)151, (uint8_t)117, (uint8_t)99, (uint8_t)25, (uint8_t)221, (uint8_t)219, (uint8_t)233, (uint8_t)88, (uint8_t)104, (uint8_t)223, (uint8_t)2, (uint8_t)94, (uint8_t)168, (uint8_t)3, (uint8_t)39, (uint8_t)102, (uint8_t)60, (uint8_t)98, (uint8_t)182, (uint8_t)89, (uint8_t)223, (uint8_t)248, (uint8_t)0, (uint8_t)18, (uint8_t)128, (uint8_t)38, (uint8_t)170, (uint8_t)174, (uint8_t)20, (uint8_t)33, (uint8_t)110, (uint8_t)73, (uint8_t)193, (uint8_t)245, (uint8_t)10, (uint8_t)126, (uint8_t)42, (uint8_t)91, (uint8_t)255, (uint8_t)73, (uint8_t)125, (uint8_t)208, (uint8_t)63, (uint8_t)116, (uint8_t)161, (uint8_t)47, (uint8_t)178, (uint8_t)38, (uint8_t)37, (uint8_t)92, (uint8_t)90, (uint8_t)193, (uint8_t)18, (uint8_t)53, (uint8_t)251, (uint8_t)240, (uint8_t)137, (uint8_t)41, (uint8_t)16, (uint8_t)108, (uint8_t)66, (uint8_t)150, (uint8_t)253, (uint8_t)42, (uint8_t)158, (uint8_t)40, (uint8_t)143, (uint8_t)114, (uint8_t)233, (uint8_t)4, (uint8_t)78, (uint8_t)70, (uint8_t)9, (uint8_t)217, (uint8_t)130, (uint8_t)121, (uint8_t)249, (uint8_t)188, (uint8_t)70, (uint8_t)74, (uint8_t)90, (uint8_t)85, (uint8_t)103, (uint8_t)230, (uint8_t)140, (uint8_t)109, (uint8_t)156, (uint8_t)249, (uint8_t)96, (uint8_t)155, (uint8_t)170, (uint8_t)88, (uint8_t)2, (uint8_t)92};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        p142_uri_type_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        {
            uint8_t uri[] =  {(uint8_t)203, (uint8_t)10, (uint8_t)173, (uint8_t)235, (uint8_t)179, (uint8_t)197, (uint8_t)115, (uint8_t)101, (uint8_t)192, (uint8_t)210, (uint8_t)196, (uint8_t)101, (uint8_t)178, (uint8_t)74, (uint8_t)74, (uint8_t)111, (uint8_t)245, (uint8_t)93, (uint8_t)222, (uint8_t)9, (uint8_t)169, (uint8_t)160, (uint8_t)143, (uint8_t)241, (uint8_t)156, (uint8_t)5, (uint8_t)75, (uint8_t)167, (uint8_t)234, (uint8_t)1, (uint8_t)89, (uint8_t)18, (uint8_t)12, (uint8_t)229, (uint8_t)54, (uint8_t)70, (uint8_t)130, (uint8_t)87, (uint8_t)216, (uint8_t)16, (uint8_t)163, (uint8_t)57, (uint8_t)14, (uint8_t)25, (uint8_t)174, (uint8_t)181, (uint8_t)99, (uint8_t)160, (uint8_t)193, (uint8_t)105, (uint8_t)106, (uint8_t)77, (uint8_t)236, (uint8_t)52, (uint8_t)192, (uint8_t)6, (uint8_t)76, (uint8_t)251, (uint8_t)87, (uint8_t)184, (uint8_t)102, (uint8_t)16, (uint8_t)82, (uint8_t)229, (uint8_t)132, (uint8_t)233, (uint8_t)245, (uint8_t)233, (uint8_t)89, (uint8_t)24, (uint8_t)137, (uint8_t)57, (uint8_t)112, (uint8_t)169, (uint8_t)91, (uint8_t)155, (uint8_t)89, (uint8_t)138, (uint8_t)198, (uint8_t)107, (uint8_t)160, (uint8_t)14, (uint8_t)235, (uint8_t)134, (uint8_t)115, (uint8_t)185, (uint8_t)8, (uint8_t)117, (uint8_t)177, (uint8_t)150, (uint8_t)151, (uint8_t)87, (uint8_t)75, (uint8_t)251, (uint8_t)30, (uint8_t)27, (uint8_t)185, (uint8_t)147, (uint8_t)225, (uint8_t)8, (uint8_t)228, (uint8_t)164, (uint8_t)143, (uint8_t)7, (uint8_t)142, (uint8_t)123, (uint8_t)2, (uint8_t)167, (uint8_t)129, (uint8_t)141, (uint8_t)94, (uint8_t)33, (uint8_t)166, (uint8_t)54, (uint8_t)115, (uint8_t)252, (uint8_t)187, (uint8_t)211, (uint8_t)76, (uint8_t)125};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        p142_transfer_type_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        c_CommunicationChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SCALED_PRESSURE3_143(), &PH);
        p143_press_abs_SET((float) -1.946696E38F, PH.base.pack) ;
        p143_temperature_SET((int16_t)(int16_t)8192, PH.base.pack) ;
        p143_press_diff_SET((float) -8.207362E37F, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)493567334L, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FOLLOW_TARGET_144(), &PH);
        p144_alt_SET((float) -4.290464E37F, PH.base.pack) ;
        {
            float attitude_q[] =  {-3.9906303E37F, 2.4230957E38F, 7.2651293E36F, -1.5446351E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        p144_lon_SET((int32_t)1840498473, PH.base.pack) ;
        p144_lat_SET((int32_t)1729480882, PH.base.pack) ;
        p144_custom_state_SET((uint64_t)6870007098816702364L, PH.base.pack) ;
        p144_timestamp_SET((uint64_t)4279504168482789703L, PH.base.pack) ;
        p144_est_capabilities_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        {
            float rates[] =  {-3.283359E37F, -2.1221242E38F, -2.4118091E38F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        {
            float acc[] =  {-2.2574354E38F, -2.2605794E38F, -3.2136835E38F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        {
            float vel[] =  {-1.805299E38F, 1.413285E37F, 6.207582E37F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        {
            float position_cov[] =  {-1.1486321E38F, 2.3676073E38F, -1.9738214E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        {
            float q[] =  {-1.1291554E38F, 5.4258235E37F, 2.6829949E38F, -3.062401E38F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_roll_rate_SET((float)1.3067257E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float) -6.0608013E37F, PH.base.pack) ;
        p146_x_pos_SET((float)1.2248886E38F, PH.base.pack) ;
        p146_y_acc_SET((float) -1.895421E38F, PH.base.pack) ;
        p146_z_acc_SET((float) -1.0538645E38F, PH.base.pack) ;
        p146_pitch_rate_SET((float) -2.80269E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {3.1021998E38F, 7.5249803E37F, 2.0008548E38F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_y_vel_SET((float)2.5100336E38F, PH.base.pack) ;
        p146_x_acc_SET((float) -1.0094178E38F, PH.base.pack) ;
        p146_z_vel_SET((float) -2.7671105E38F, PH.base.pack) ;
        p146_time_usec_SET((uint64_t)2975577667978283410L, PH.base.pack) ;
        {
            float pos_variance[] =  {-1.0952608E37F, -9.662304E37F, -4.8178623E37F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_y_pos_SET((float)3.3815742E38F, PH.base.pack) ;
        p146_x_vel_SET((float) -2.179621E38F, PH.base.pack) ;
        p146_airspeed_SET((float) -1.8774726E37F, PH.base.pack) ;
        p146_z_pos_SET((float) -1.740992E38F, PH.base.pack) ;
        c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BATTERY_STATUS_147(), &PH);
        p147_temperature_SET((int16_t)(int16_t)28211, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t)115, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)25863, (uint16_t)42194, (uint16_t)46087, (uint16_t)53530, (uint16_t)34041, (uint16_t)19457, (uint16_t)32125, (uint16_t)4534, (uint16_t)34168, (uint16_t)41846};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_current_battery_SET((int16_t)(int16_t) -27466, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t)432518029, PH.base.pack) ;
        p147_current_consumed_SET((int32_t)1155217861, PH.base.pack) ;
        c_CommunicationChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AUTOPILOT_VERSION_148(), &PH);
        p148_vendor_id_SET((uint16_t)(uint16_t)28970, PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)52, (uint8_t)54, (uint8_t)154, (uint8_t)116, (uint8_t)135, (uint8_t)246, (uint8_t)27, (uint8_t)192};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        p148_os_sw_version_SET((uint32_t)3539602230L, PH.base.pack) ;
        p148_board_version_SET((uint32_t)1387443219L, PH.base.pack) ;
        p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMMAND_INT, PH.base.pack) ;
        {
            uint8_t flight_custom_version[] =  {(uint8_t)202, (uint8_t)13, (uint8_t)107, (uint8_t)182, (uint8_t)144, (uint8_t)175, (uint8_t)47, (uint8_t)122};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t os_custom_version[] =  {(uint8_t)108, (uint8_t)125, (uint8_t)151, (uint8_t)108, (uint8_t)34, (uint8_t)139, (uint8_t)37, (uint8_t)27};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t uid2[] =  {(uint8_t)91, (uint8_t)248, (uint8_t)118, (uint8_t)236, (uint8_t)147, (uint8_t)59, (uint8_t)202, (uint8_t)194, (uint8_t)69, (uint8_t)179, (uint8_t)78, (uint8_t)168, (uint8_t)153, (uint8_t)194, (uint8_t)37, (uint8_t)185, (uint8_t)236, (uint8_t)208};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_product_id_SET((uint16_t)(uint16_t)26075, PH.base.pack) ;
        p148_uid_SET((uint64_t)3838784069892282189L, PH.base.pack) ;
        p148_middleware_sw_version_SET((uint32_t)2654842194L, PH.base.pack) ;
        p148_flight_sw_version_SET((uint32_t)3675992424L, PH.base.pack) ;
        c_CommunicationChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LANDING_TARGET_149(), &PH);
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, PH.base.pack) ;
        p149_distance_SET((float) -7.473657E37F, PH.base.pack) ;
        p149_y_SET((float)1.2388849E38F, &PH) ;
        p149_size_x_SET((float) -1.6699465E38F, PH.base.pack) ;
        p149_position_valid_SET((uint8_t)(uint8_t)238, &PH) ;
        p149_z_SET((float)3.1929468E38F, &PH) ;
        p149_size_y_SET((float)4.0458113E37F, PH.base.pack) ;
        p149_angle_x_SET((float) -1.9459753E38F, PH.base.pack) ;
        p149_x_SET((float)1.962821E38F, &PH) ;
        p149_target_num_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p149_time_usec_SET((uint64_t)81914681621160817L, PH.base.pack) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p149_angle_y_SET((float) -8.581042E37F, PH.base.pack) ;
        {
            float q[] =  {2.8488374E38F, 3.2065709E37F, -4.917466E37F, 1.264135E38F};
            p149_q_SET(&q, 0, &PH) ;
        }
        c_CommunicationChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SCRIPT_ITEM_180(), &PH);
        p180_seq_SET((uint16_t)(uint16_t)30861, PH.base.pack) ;
        {
            char16_t* name = u"ktxnzwiixdvwyiwzojqrnldzo";
            p180_name_SET_(name, &PH) ;
        }
        p180_target_component_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p180_target_system_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        c_CommunicationChannel_on_SCRIPT_ITEM_180(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SCRIPT_REQUEST_181(), &PH);
        p181_target_component_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
        p181_seq_SET((uint16_t)(uint16_t)19638, PH.base.pack) ;
        p181_target_system_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        c_CommunicationChannel_on_SCRIPT_REQUEST_181(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SCRIPT_REQUEST_LIST_182(), &PH);
        p182_target_system_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        p182_target_component_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        c_CommunicationChannel_on_SCRIPT_REQUEST_LIST_182(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SCRIPT_COUNT_183(), &PH);
        p183_target_system_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p183_count_SET((uint16_t)(uint16_t)19613, PH.base.pack) ;
        p183_target_component_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        c_CommunicationChannel_on_SCRIPT_COUNT_183(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SCRIPT_CURRENT_184(), &PH);
        p184_seq_SET((uint16_t)(uint16_t)26877, PH.base.pack) ;
        c_CommunicationChannel_on_SCRIPT_CURRENT_184(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_time_usec_SET((uint64_t)8650523584487809958L, PH.base.pack) ;
        p230_mag_ratio_SET((float) -8.895444E37F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float)1.887045E38F, PH.base.pack) ;
        p230_hagl_ratio_SET((float) -7.621018E37F, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float) -2.5573405E38F, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float)3.196326E38F, PH.base.pack) ;
        p230_vel_ratio_SET((float)9.415936E37F, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float)3.3315174E38F, PH.base.pack) ;
        p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS, PH.base.pack) ;
        p230_tas_ratio_SET((float) -2.9689736E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIND_COV_231(), &PH);
        p231_wind_z_SET((float) -7.1157777E37F, PH.base.pack) ;
        p231_vert_accuracy_SET((float) -2.5281054E38F, PH.base.pack) ;
        p231_wind_y_SET((float) -5.663904E37F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float)3.8009644E37F, PH.base.pack) ;
        p231_wind_alt_SET((float) -1.148085E38F, PH.base.pack) ;
        p231_var_vert_SET((float)3.2947284E38F, PH.base.pack) ;
        p231_var_horiz_SET((float)6.471001E37F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)6922413796538833499L, PH.base.pack) ;
        p231_wind_x_SET((float)8.801804E37F, PH.base.pack) ;
        c_CommunicationChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_INPUT_232(), &PH);
        p232_alt_SET((float) -2.277422E38F, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        p232_speed_accuracy_SET((float)2.6208283E38F, PH.base.pack) ;
        p232_horiz_accuracy_SET((float)1.1297521E38F, PH.base.pack) ;
        p232_hdop_SET((float) -2.0129638E38F, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)18086, PH.base.pack) ;
        p232_vd_SET((float)1.1886941E38F, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)5700286231885534053L, PH.base.pack) ;
        p232_vn_SET((float)9.279959E36F, PH.base.pack) ;
        p232_vert_accuracy_SET((float) -3.8337164E37F, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)2327678466L, PH.base.pack) ;
        p232_ve_SET((float)2.4690015E38F, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP, PH.base.pack) ;
        p232_lon_SET((int32_t) -598491356, PH.base.pack) ;
        p232_lat_SET((int32_t)710176928, PH.base.pack) ;
        p232_vdop_SET((float) -2.1962675E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_RTCM_DATA_233(), &PH);
        p233_len_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
        p233_flags_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)0, (uint8_t)254, (uint8_t)205, (uint8_t)30, (uint8_t)203, (uint8_t)69, (uint8_t)132, (uint8_t)197, (uint8_t)159, (uint8_t)176, (uint8_t)232, (uint8_t)55, (uint8_t)40, (uint8_t)52, (uint8_t)203, (uint8_t)144, (uint8_t)65, (uint8_t)158, (uint8_t)102, (uint8_t)18, (uint8_t)220, (uint8_t)237, (uint8_t)208, (uint8_t)251, (uint8_t)24, (uint8_t)138, (uint8_t)211, (uint8_t)245, (uint8_t)168, (uint8_t)127, (uint8_t)219, (uint8_t)79, (uint8_t)146, (uint8_t)89, (uint8_t)8, (uint8_t)116, (uint8_t)171, (uint8_t)115, (uint8_t)105, (uint8_t)147, (uint8_t)27, (uint8_t)63, (uint8_t)136, (uint8_t)55, (uint8_t)204, (uint8_t)144, (uint8_t)211, (uint8_t)7, (uint8_t)231, (uint8_t)238, (uint8_t)250, (uint8_t)51, (uint8_t)8, (uint8_t)207, (uint8_t)216, (uint8_t)234, (uint8_t)227, (uint8_t)115, (uint8_t)214, (uint8_t)107, (uint8_t)53, (uint8_t)55, (uint8_t)184, (uint8_t)20, (uint8_t)53, (uint8_t)162, (uint8_t)156, (uint8_t)123, (uint8_t)158, (uint8_t)153, (uint8_t)41, (uint8_t)113, (uint8_t)234, (uint8_t)53, (uint8_t)209, (uint8_t)38, (uint8_t)158, (uint8_t)174, (uint8_t)22, (uint8_t)69, (uint8_t)66, (uint8_t)232, (uint8_t)249, (uint8_t)251, (uint8_t)15, (uint8_t)93, (uint8_t)150, (uint8_t)218, (uint8_t)100, (uint8_t)14, (uint8_t)229, (uint8_t)206, (uint8_t)247, (uint8_t)177, (uint8_t)22, (uint8_t)93, (uint8_t)167, (uint8_t)227, (uint8_t)115, (uint8_t)73, (uint8_t)185, (uint8_t)173, (uint8_t)243, (uint8_t)182, (uint8_t)202, (uint8_t)52, (uint8_t)191, (uint8_t)134, (uint8_t)141, (uint8_t)144, (uint8_t)14, (uint8_t)159, (uint8_t)86, (uint8_t)230, (uint8_t)243, (uint8_t)240, (uint8_t)4, (uint8_t)104, (uint8_t)144, (uint8_t)46, (uint8_t)124, (uint8_t)184, (uint8_t)42, (uint8_t)115, (uint8_t)92, (uint8_t)124, (uint8_t)180, (uint8_t)58, (uint8_t)154, (uint8_t)229, (uint8_t)208, (uint8_t)45, (uint8_t)222, (uint8_t)206, (uint8_t)190, (uint8_t)91, (uint8_t)53, (uint8_t)27, (uint8_t)23, (uint8_t)207, (uint8_t)131, (uint8_t)175, (uint8_t)49, (uint8_t)70, (uint8_t)227, (uint8_t)229, (uint8_t)197, (uint8_t)139, (uint8_t)152, (uint8_t)166, (uint8_t)219, (uint8_t)69, (uint8_t)68, (uint8_t)88, (uint8_t)255, (uint8_t)4, (uint8_t)191, (uint8_t)162, (uint8_t)235, (uint8_t)150, (uint8_t)215, (uint8_t)106, (uint8_t)20, (uint8_t)187, (uint8_t)179, (uint8_t)35, (uint8_t)148, (uint8_t)98, (uint8_t)124, (uint8_t)54, (uint8_t)239, (uint8_t)234, (uint8_t)10, (uint8_t)106, (uint8_t)134, (uint8_t)88, (uint8_t)146, (uint8_t)229, (uint8_t)184, (uint8_t)40};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HIGH_LATENCY_234(), &PH);
        p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t)97, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        p234_latitude_SET((int32_t)1655096094, PH.base.pack) ;
        p234_longitude_SET((int32_t) -1628746013, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t)13306, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t)38, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t)1776, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)14490, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t)125, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t)27863, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)3801128081L, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t)3586, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)50318, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t) -15799, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t) -85, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        c_CommunicationChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIBRATION_241(), &PH);
        p241_vibration_y_SET((float)2.0011131E37F, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)8942639826998546540L, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)328569783L, PH.base.pack) ;
        p241_vibration_z_SET((float) -9.297376E37F, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)4291047704L, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)3306060183L, PH.base.pack) ;
        p241_vibration_x_SET((float)6.50424E36F, PH.base.pack) ;
        c_CommunicationChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HOME_POSITION_242(), &PH);
        p242_latitude_SET((int32_t) -1678408284, PH.base.pack) ;
        p242_y_SET((float) -2.8301014E38F, PH.base.pack) ;
        {
            float q[] =  {3.2809434E38F, 1.2299912E38F, 5.6277774E37F, 2.6870933E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_longitude_SET((int32_t)674765480, PH.base.pack) ;
        p242_z_SET((float)2.5536623E38F, PH.base.pack) ;
        p242_approach_z_SET((float) -7.6773813E37F, PH.base.pack) ;
        p242_approach_y_SET((float) -2.7345085E38F, PH.base.pack) ;
        p242_approach_x_SET((float) -2.3727286E37F, PH.base.pack) ;
        p242_x_SET((float)2.3649636E38F, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)4672639445532162590L, &PH) ;
        p242_altitude_SET((int32_t) -1814616134, PH.base.pack) ;
        c_CommunicationChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_HOME_POSITION_243(), &PH);
        p243_latitude_SET((int32_t)1531838969, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        {
            float q[] =  {2.24303E38F, 1.5621928E38F, 2.718189E38F, 3.195863E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_time_usec_SET((uint64_t)2566154670001589697L, &PH) ;
        p243_z_SET((float) -1.5449559E38F, PH.base.pack) ;
        p243_longitude_SET((int32_t)1394529959, PH.base.pack) ;
        p243_approach_x_SET((float)2.8701616E38F, PH.base.pack) ;
        p243_approach_y_SET((float)9.02974E37F, PH.base.pack) ;
        p243_approach_z_SET((float)2.0188031E38F, PH.base.pack) ;
        p243_x_SET((float) -2.4458416E38F, PH.base.pack) ;
        p243_altitude_SET((int32_t)1602230401, PH.base.pack) ;
        p243_y_SET((float) -7.495438E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_interval_us_SET((int32_t)2056408475, PH.base.pack) ;
        p244_message_id_SET((uint16_t)(uint16_t)40890, PH.base.pack) ;
        c_CommunicationChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_MC, PH.base.pack) ;
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, PH.base.pack) ;
        c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ADSB_VEHICLE_246(), &PH);
        p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)52133, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)1702756559L, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UAV, PH.base.pack) ;
        p246_lat_SET((int32_t) -2007246302, PH.base.pack) ;
        p246_altitude_SET((int32_t)1635815296, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t) -10745, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)60621, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)16960, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
        p246_lon_SET((int32_t)1626159605, PH.base.pack) ;
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
        {
            char16_t* callsign = u"tinwq";
            p246_callsign_SET_(callsign, &PH) ;
        }
        c_CommunicationChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_COLLISION_247(), &PH);
        p247_horizontal_minimum_delta_SET((float) -3.1439599E38F, PH.base.pack) ;
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float)1.7878277E38F, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float)2.8415275E38F, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, PH.base.pack) ;
        p247_id_SET((uint32_t)2283758296L, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL, PH.base.pack) ;
        c_CommunicationChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_V2_EXTENSION_248(), &PH);
        {
            uint8_t payload[] =  {(uint8_t)101, (uint8_t)230, (uint8_t)76, (uint8_t)218, (uint8_t)4, (uint8_t)235, (uint8_t)209, (uint8_t)128, (uint8_t)10, (uint8_t)38, (uint8_t)139, (uint8_t)201, (uint8_t)162, (uint8_t)189, (uint8_t)142, (uint8_t)253, (uint8_t)250, (uint8_t)78, (uint8_t)11, (uint8_t)90, (uint8_t)53, (uint8_t)252, (uint8_t)56, (uint8_t)74, (uint8_t)45, (uint8_t)113, (uint8_t)8, (uint8_t)235, (uint8_t)94, (uint8_t)238, (uint8_t)112, (uint8_t)96, (uint8_t)126, (uint8_t)252, (uint8_t)119, (uint8_t)119, (uint8_t)207, (uint8_t)158, (uint8_t)61, (uint8_t)50, (uint8_t)214, (uint8_t)157, (uint8_t)52, (uint8_t)80, (uint8_t)18, (uint8_t)202, (uint8_t)66, (uint8_t)92, (uint8_t)172, (uint8_t)189, (uint8_t)103, (uint8_t)231, (uint8_t)216, (uint8_t)26, (uint8_t)177, (uint8_t)5, (uint8_t)208, (uint8_t)225, (uint8_t)52, (uint8_t)189, (uint8_t)47, (uint8_t)109, (uint8_t)201, (uint8_t)47, (uint8_t)237, (uint8_t)84, (uint8_t)13, (uint8_t)125, (uint8_t)143, (uint8_t)236, (uint8_t)169, (uint8_t)156, (uint8_t)197, (uint8_t)206, (uint8_t)187, (uint8_t)41, (uint8_t)201, (uint8_t)17, (uint8_t)32, (uint8_t)3, (uint8_t)163, (uint8_t)98, (uint8_t)184, (uint8_t)252, (uint8_t)110, (uint8_t)156, (uint8_t)104, (uint8_t)196, (uint8_t)242, (uint8_t)248, (uint8_t)128, (uint8_t)154, (uint8_t)58, (uint8_t)88, (uint8_t)177, (uint8_t)107, (uint8_t)135, (uint8_t)91, (uint8_t)194, (uint8_t)77, (uint8_t)255, (uint8_t)79, (uint8_t)227, (uint8_t)29, (uint8_t)112, (uint8_t)207, (uint8_t)250, (uint8_t)0, (uint8_t)2, (uint8_t)249, (uint8_t)138, (uint8_t)249, (uint8_t)178, (uint8_t)240, (uint8_t)237, (uint8_t)204, (uint8_t)237, (uint8_t)89, (uint8_t)208, (uint8_t)176, (uint8_t)139, (uint8_t)227, (uint8_t)59, (uint8_t)118, (uint8_t)124, (uint8_t)118, (uint8_t)1, (uint8_t)85, (uint8_t)6, (uint8_t)171, (uint8_t)64, (uint8_t)31, (uint8_t)10, (uint8_t)116, (uint8_t)196, (uint8_t)130, (uint8_t)190, (uint8_t)27, (uint8_t)234, (uint8_t)28, (uint8_t)227, (uint8_t)209, (uint8_t)29, (uint8_t)214, (uint8_t)102, (uint8_t)11, (uint8_t)88, (uint8_t)20, (uint8_t)70, (uint8_t)131, (uint8_t)190, (uint8_t)247, (uint8_t)10, (uint8_t)250, (uint8_t)149, (uint8_t)255, (uint8_t)218, (uint8_t)97, (uint8_t)121, (uint8_t)245, (uint8_t)134, (uint8_t)73, (uint8_t)77, (uint8_t)26, (uint8_t)127, (uint8_t)234, (uint8_t)81, (uint8_t)138, (uint8_t)82, (uint8_t)87, (uint8_t)141, (uint8_t)1, (uint8_t)209, (uint8_t)244, (uint8_t)97, (uint8_t)168, (uint8_t)79, (uint8_t)55, (uint8_t)79, (uint8_t)188, (uint8_t)244, (uint8_t)103, (uint8_t)2, (uint8_t)69, (uint8_t)38, (uint8_t)180, (uint8_t)128, (uint8_t)73, (uint8_t)163, (uint8_t)17, (uint8_t)176, (uint8_t)105, (uint8_t)51, (uint8_t)123, (uint8_t)83, (uint8_t)52, (uint8_t)155, (uint8_t)227, (uint8_t)2, (uint8_t)42, (uint8_t)195, (uint8_t)140, (uint8_t)190, (uint8_t)91, (uint8_t)32, (uint8_t)131, (uint8_t)248, (uint8_t)149, (uint8_t)134, (uint8_t)65, (uint8_t)226, (uint8_t)94, (uint8_t)12, (uint8_t)71, (uint8_t)191, (uint8_t)64, (uint8_t)51, (uint8_t)120, (uint8_t)214, (uint8_t)247, (uint8_t)46, (uint8_t)200, (uint8_t)121, (uint8_t)168, (uint8_t)217, (uint8_t)79, (uint8_t)126, (uint8_t)61, (uint8_t)24, (uint8_t)74, (uint8_t)49, (uint8_t)124, (uint8_t)145, (uint8_t)43, (uint8_t)132, (uint8_t)47, (uint8_t)141, (uint8_t)215, (uint8_t)10, (uint8_t)28, (uint8_t)55, (uint8_t)88, (uint8_t)77, (uint8_t)115, (uint8_t)49, (uint8_t)219, (uint8_t)183, (uint8_t)197, (uint8_t)189};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p248_message_type_SET((uint16_t)(uint16_t)12954, PH.base.pack) ;
        p248_target_system_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p248_target_network_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
        p248_target_component_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        c_CommunicationChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MEMORY_VECT_249(), &PH);
        p249_ver_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        p249_type_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t)19, (int8_t) -78, (int8_t)110, (int8_t) -48, (int8_t)125, (int8_t) -94, (int8_t)56, (int8_t) -28, (int8_t)76, (int8_t)101, (int8_t)90, (int8_t) -11, (int8_t)76, (int8_t) -51, (int8_t)51, (int8_t)111, (int8_t)90, (int8_t) -87, (int8_t)26, (int8_t)24, (int8_t) -35, (int8_t)34, (int8_t) -30, (int8_t) -50, (int8_t)11, (int8_t)49, (int8_t) -117, (int8_t) -111, (int8_t) -28, (int8_t)27, (int8_t) -118, (int8_t)15};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_address_SET((uint16_t)(uint16_t)56413, PH.base.pack) ;
        c_CommunicationChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_VECT_250(), &PH);
        p250_time_usec_SET((uint64_t)5526748862680987300L, PH.base.pack) ;
        p250_x_SET((float)6.214597E37F, PH.base.pack) ;
        {
            char16_t* name = u"nnqt";
            p250_name_SET_(name, &PH) ;
        }
        p250_z_SET((float) -1.8165853E38F, PH.base.pack) ;
        p250_y_SET((float) -1.2712121E38F, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_FLOAT_251(), &PH);
        {
            char16_t* name = u"Da";
            p251_name_SET_(name, &PH) ;
        }
        p251_value_SET((float) -2.340629E38F, PH.base.pack) ;
        p251_time_boot_ms_SET((uint32_t)3667925539L, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_INT_252(), &PH);
        p252_value_SET((int32_t) -1123165763, PH.base.pack) ;
        p252_time_boot_ms_SET((uint32_t)827410548L, PH.base.pack) ;
        {
            char16_t* name = u"ldoqkesk";
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
            char16_t* text = u"aeeowhlzobrptkgpnbainvqy";
            p253_text_SET_(text, &PH) ;
        }
        c_CommunicationChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_254(), &PH);
        p254_time_boot_ms_SET((uint32_t)3460879917L, PH.base.pack) ;
        p254_value_SET((float)1.0263536E38F, PH.base.pack) ;
        p254_ind_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SETUP_SIGNING_256(), &PH);
        {
            uint8_t secret_key[] =  {(uint8_t)43, (uint8_t)253, (uint8_t)152, (uint8_t)44, (uint8_t)32, (uint8_t)227, (uint8_t)41, (uint8_t)90, (uint8_t)95, (uint8_t)92, (uint8_t)121, (uint8_t)172, (uint8_t)181, (uint8_t)37, (uint8_t)91, (uint8_t)16, (uint8_t)103, (uint8_t)45, (uint8_t)200, (uint8_t)132, (uint8_t)58, (uint8_t)41, (uint8_t)16, (uint8_t)29, (uint8_t)32, (uint8_t)131, (uint8_t)171, (uint8_t)57, (uint8_t)98, (uint8_t)160, (uint8_t)169, (uint8_t)93};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        p256_initial_timestamp_SET((uint64_t)1752657446413388789L, PH.base.pack) ;
        p256_target_system_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p256_target_component_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        c_CommunicationChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BUTTON_CHANGE_257(), &PH);
        p257_last_change_ms_SET((uint32_t)3661985548L, PH.base.pack) ;
        p257_state_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p257_time_boot_ms_SET((uint32_t)626888448L, PH.base.pack) ;
        c_CommunicationChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PLAY_TUNE_258(), &PH);
        p258_target_component_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
        p258_target_system_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        {
            char16_t* tune = u"dj";
            p258_tune_SET_(tune, &PH) ;
        }
        c_CommunicationChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_INFORMATION_259(), &PH);
        p259_sensor_size_h_SET((float)3.0338793E38F, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)117, (uint8_t)95, (uint8_t)50, (uint8_t)142, (uint8_t)135, (uint8_t)40, (uint8_t)72, (uint8_t)169, (uint8_t)222, (uint8_t)221, (uint8_t)84, (uint8_t)170, (uint8_t)125, (uint8_t)202, (uint8_t)130, (uint8_t)140, (uint8_t)8, (uint8_t)75, (uint8_t)109, (uint8_t)237, (uint8_t)190, (uint8_t)71, (uint8_t)196, (uint8_t)105, (uint8_t)4, (uint8_t)47, (uint8_t)228, (uint8_t)36, (uint8_t)95, (uint8_t)178, (uint8_t)205, (uint8_t)72};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_cam_definition_version_SET((uint16_t)(uint16_t)58080, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)52015, PH.base.pack) ;
        p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO, PH.base.pack) ;
        p259_firmware_version_SET((uint32_t)595480510L, PH.base.pack) ;
        p259_sensor_size_v_SET((float) -2.0490965E37F, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)83, (uint8_t)133, (uint8_t)210, (uint8_t)24, (uint8_t)78, (uint8_t)253, (uint8_t)52, (uint8_t)20, (uint8_t)146, (uint8_t)241, (uint8_t)37, (uint8_t)29, (uint8_t)130, (uint8_t)9, (uint8_t)249, (uint8_t)232, (uint8_t)64, (uint8_t)31, (uint8_t)65, (uint8_t)124, (uint8_t)25, (uint8_t)17, (uint8_t)147, (uint8_t)115, (uint8_t)36, (uint8_t)239, (uint8_t)224, (uint8_t)219, (uint8_t)152, (uint8_t)15, (uint8_t)183, (uint8_t)216};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        {
            char16_t* cam_definition_uri = u"aqdyvwooSPUpdfzedsirueLgnxfcfgbosewwBxoyIgnsniyWfctrinweh";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_time_boot_ms_SET((uint32_t)3150093291L, PH.base.pack) ;
        p259_lens_id_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p259_resolution_v_SET((uint16_t)(uint16_t)44724, PH.base.pack) ;
        p259_focal_length_SET((float)5.554374E37F, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_SETTINGS_260(), &PH);
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_VIDEO, PH.base.pack) ;
        p260_time_boot_ms_SET((uint32_t)458067567L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STORAGE_INFORMATION_261(), &PH);
        p261_available_capacity_SET((float) -1.1375118E38F, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        p261_total_capacity_SET((float) -6.6644885E37F, PH.base.pack) ;
        p261_read_speed_SET((float)1.4978025E38F, PH.base.pack) ;
        p261_used_capacity_SET((float)3.3304098E38F, PH.base.pack) ;
        p261_write_speed_SET((float)1.6210977E38F, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)3855099934L, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        c_CommunicationChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_recording_time_ms_SET((uint32_t)1044396119L, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        p262_available_capacity_SET((float) -1.253692E38F, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)1686141741L, PH.base.pack) ;
        p262_image_interval_SET((float)2.9112975E38F, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_lat_SET((int32_t)1499872835, PH.base.pack) ;
        p263_alt_SET((int32_t)1036762488, PH.base.pack) ;
        p263_relative_alt_SET((int32_t) -532615088, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)2511759240L, PH.base.pack) ;
        p263_lon_SET((int32_t) -710566914, PH.base.pack) ;
        {
            char16_t* file_url = u"XrtAclowbzlbbexMbllrkcehraJnewpjnwtfxjsTOqbZqxnuzcfixcUypspekovtbvzplacYmvftytBpfftjbhdsgajiiWdinFxfxxUmTinxzhiahcrrnizgbtkrvwwhovvxjqpctjppcylrlgbpuqkxjsmcabbypcbksc";
            p263_file_url_SET_(file_url, &PH) ;
        }
        {
            float q[] =  {-1.928797E38F, 3.22647E38F, -2.5164941E38F, 2.1487838E38F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_capture_result_SET((int8_t)(int8_t) -120, PH.base.pack) ;
        p263_time_utc_SET((uint64_t)8303992998392865220L, PH.base.pack) ;
        p263_camera_id_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p263_image_index_SET((int32_t) -377666724, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_takeoff_time_utc_SET((uint64_t)2814884143861888998L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)1750588388826710228L, PH.base.pack) ;
        p264_arming_time_utc_SET((uint64_t)7778942620396854329L, PH.base.pack) ;
        p264_time_boot_ms_SET((uint32_t)1680708644L, PH.base.pack) ;
        c_CommunicationChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_pitch_SET((float)1.0000598E38F, PH.base.pack) ;
        p265_yaw_SET((float)2.4555824E38F, PH.base.pack) ;
        p265_time_boot_ms_SET((uint32_t)611813947L, PH.base.pack) ;
        p265_roll_SET((float)2.2871473E38F, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_266(), &PH);
        p266_sequence_SET((uint16_t)(uint16_t)19617, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p266_length_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p266_target_system_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        p266_first_message_offset_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)173, (uint8_t)223, (uint8_t)62, (uint8_t)195, (uint8_t)219, (uint8_t)228, (uint8_t)112, (uint8_t)243, (uint8_t)227, (uint8_t)232, (uint8_t)140, (uint8_t)123, (uint8_t)248, (uint8_t)234, (uint8_t)244, (uint8_t)31, (uint8_t)204, (uint8_t)109, (uint8_t)69, (uint8_t)160, (uint8_t)110, (uint8_t)101, (uint8_t)102, (uint8_t)1, (uint8_t)209, (uint8_t)11, (uint8_t)169, (uint8_t)113, (uint8_t)131, (uint8_t)50, (uint8_t)230, (uint8_t)143, (uint8_t)168, (uint8_t)104, (uint8_t)166, (uint8_t)28, (uint8_t)43, (uint8_t)45, (uint8_t)227, (uint8_t)142, (uint8_t)3, (uint8_t)17, (uint8_t)146, (uint8_t)98, (uint8_t)212, (uint8_t)223, (uint8_t)15, (uint8_t)70, (uint8_t)73, (uint8_t)100, (uint8_t)17, (uint8_t)33, (uint8_t)175, (uint8_t)240, (uint8_t)185, (uint8_t)236, (uint8_t)218, (uint8_t)79, (uint8_t)40, (uint8_t)205, (uint8_t)103, (uint8_t)168, (uint8_t)82, (uint8_t)158, (uint8_t)112, (uint8_t)130, (uint8_t)16, (uint8_t)248, (uint8_t)20, (uint8_t)193, (uint8_t)239, (uint8_t)141, (uint8_t)106, (uint8_t)39, (uint8_t)107, (uint8_t)124, (uint8_t)28, (uint8_t)53, (uint8_t)168, (uint8_t)94, (uint8_t)97, (uint8_t)156, (uint8_t)23, (uint8_t)28, (uint8_t)151, (uint8_t)190, (uint8_t)81, (uint8_t)233, (uint8_t)214, (uint8_t)168, (uint8_t)81, (uint8_t)139, (uint8_t)112, (uint8_t)212, (uint8_t)148, (uint8_t)138, (uint8_t)146, (uint8_t)189, (uint8_t)40, (uint8_t)138, (uint8_t)222, (uint8_t)60, (uint8_t)123, (uint8_t)90, (uint8_t)66, (uint8_t)82, (uint8_t)204, (uint8_t)139, (uint8_t)149, (uint8_t)94, (uint8_t)12, (uint8_t)137, (uint8_t)122, (uint8_t)139, (uint8_t)193, (uint8_t)58, (uint8_t)34, (uint8_t)94, (uint8_t)12, (uint8_t)245, (uint8_t)219, (uint8_t)102, (uint8_t)110, (uint8_t)192, (uint8_t)35, (uint8_t)208, (uint8_t)72, (uint8_t)145, (uint8_t)68, (uint8_t)223, (uint8_t)37, (uint8_t)20, (uint8_t)13, (uint8_t)118, (uint8_t)225, (uint8_t)193, (uint8_t)16, (uint8_t)142, (uint8_t)79, (uint8_t)24, (uint8_t)219, (uint8_t)126, (uint8_t)236, (uint8_t)167, (uint8_t)15, (uint8_t)14, (uint8_t)205, (uint8_t)155, (uint8_t)73, (uint8_t)115, (uint8_t)81, (uint8_t)115, (uint8_t)9, (uint8_t)111, (uint8_t)109, (uint8_t)241, (uint8_t)134, (uint8_t)21, (uint8_t)15, (uint8_t)246, (uint8_t)47, (uint8_t)163, (uint8_t)174, (uint8_t)188, (uint8_t)174, (uint8_t)25, (uint8_t)27, (uint8_t)179, (uint8_t)172, (uint8_t)30, (uint8_t)170, (uint8_t)31, (uint8_t)7, (uint8_t)97, (uint8_t)37, (uint8_t)254, (uint8_t)240, (uint8_t)247, (uint8_t)60, (uint8_t)151, (uint8_t)109, (uint8_t)142, (uint8_t)0, (uint8_t)199, (uint8_t)18, (uint8_t)164, (uint8_t)22, (uint8_t)162, (uint8_t)195, (uint8_t)127, (uint8_t)94, (uint8_t)146, (uint8_t)128, (uint8_t)240, (uint8_t)230, (uint8_t)174, (uint8_t)31, (uint8_t)158, (uint8_t)224, (uint8_t)23, (uint8_t)236, (uint8_t)183, (uint8_t)21, (uint8_t)52, (uint8_t)44, (uint8_t)76, (uint8_t)172, (uint8_t)68, (uint8_t)189, (uint8_t)37, (uint8_t)0, (uint8_t)214, (uint8_t)93, (uint8_t)164, (uint8_t)11, (uint8_t)11, (uint8_t)128, (uint8_t)53, (uint8_t)83, (uint8_t)11, (uint8_t)176, (uint8_t)137, (uint8_t)208, (uint8_t)15, (uint8_t)11, (uint8_t)82, (uint8_t)34, (uint8_t)215, (uint8_t)194, (uint8_t)103, (uint8_t)29, (uint8_t)2, (uint8_t)33, (uint8_t)79, (uint8_t)189, (uint8_t)18, (uint8_t)55, (uint8_t)225, (uint8_t)144, (uint8_t)135, (uint8_t)240, (uint8_t)115, (uint8_t)101, (uint8_t)47, (uint8_t)194, (uint8_t)234, (uint8_t)251, (uint8_t)1, (uint8_t)42};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_ACKED_267(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)245, (uint8_t)11, (uint8_t)191, (uint8_t)112, (uint8_t)43, (uint8_t)136, (uint8_t)64, (uint8_t)114, (uint8_t)15, (uint8_t)163, (uint8_t)223, (uint8_t)106, (uint8_t)35, (uint8_t)23, (uint8_t)20, (uint8_t)180, (uint8_t)41, (uint8_t)173, (uint8_t)127, (uint8_t)231, (uint8_t)244, (uint8_t)20, (uint8_t)148, (uint8_t)173, (uint8_t)202, (uint8_t)10, (uint8_t)229, (uint8_t)234, (uint8_t)158, (uint8_t)254, (uint8_t)253, (uint8_t)24, (uint8_t)227, (uint8_t)153, (uint8_t)168, (uint8_t)9, (uint8_t)184, (uint8_t)6, (uint8_t)230, (uint8_t)9, (uint8_t)229, (uint8_t)25, (uint8_t)149, (uint8_t)182, (uint8_t)35, (uint8_t)67, (uint8_t)28, (uint8_t)240, (uint8_t)10, (uint8_t)183, (uint8_t)35, (uint8_t)212, (uint8_t)188, (uint8_t)235, (uint8_t)221, (uint8_t)12, (uint8_t)177, (uint8_t)34, (uint8_t)33, (uint8_t)246, (uint8_t)34, (uint8_t)173, (uint8_t)61, (uint8_t)15, (uint8_t)223, (uint8_t)172, (uint8_t)72, (uint8_t)243, (uint8_t)38, (uint8_t)144, (uint8_t)49, (uint8_t)28, (uint8_t)28, (uint8_t)128, (uint8_t)203, (uint8_t)221, (uint8_t)172, (uint8_t)3, (uint8_t)247, (uint8_t)205, (uint8_t)238, (uint8_t)48, (uint8_t)218, (uint8_t)63, (uint8_t)134, (uint8_t)207, (uint8_t)25, (uint8_t)169, (uint8_t)36, (uint8_t)193, (uint8_t)18, (uint8_t)134, (uint8_t)62, (uint8_t)14, (uint8_t)168, (uint8_t)49, (uint8_t)81, (uint8_t)76, (uint8_t)61, (uint8_t)52, (uint8_t)158, (uint8_t)134, (uint8_t)244, (uint8_t)36, (uint8_t)119, (uint8_t)240, (uint8_t)58, (uint8_t)12, (uint8_t)225, (uint8_t)86, (uint8_t)9, (uint8_t)144, (uint8_t)101, (uint8_t)57, (uint8_t)12, (uint8_t)239, (uint8_t)84, (uint8_t)210, (uint8_t)188, (uint8_t)39, (uint8_t)46, (uint8_t)59, (uint8_t)87, (uint8_t)188, (uint8_t)253, (uint8_t)156, (uint8_t)103, (uint8_t)215, (uint8_t)60, (uint8_t)3, (uint8_t)17, (uint8_t)130, (uint8_t)220, (uint8_t)134, (uint8_t)11, (uint8_t)147, (uint8_t)89, (uint8_t)91, (uint8_t)42, (uint8_t)91, (uint8_t)26, (uint8_t)191, (uint8_t)102, (uint8_t)137, (uint8_t)83, (uint8_t)137, (uint8_t)37, (uint8_t)141, (uint8_t)117, (uint8_t)251, (uint8_t)49, (uint8_t)98, (uint8_t)77, (uint8_t)232, (uint8_t)87, (uint8_t)58, (uint8_t)216, (uint8_t)69, (uint8_t)133, (uint8_t)41, (uint8_t)40, (uint8_t)64, (uint8_t)86, (uint8_t)60, (uint8_t)47, (uint8_t)46, (uint8_t)10, (uint8_t)96, (uint8_t)160, (uint8_t)180, (uint8_t)191, (uint8_t)252, (uint8_t)119, (uint8_t)209, (uint8_t)230, (uint8_t)127, (uint8_t)220, (uint8_t)125, (uint8_t)123, (uint8_t)224, (uint8_t)49, (uint8_t)162, (uint8_t)45, (uint8_t)10, (uint8_t)17, (uint8_t)109, (uint8_t)93, (uint8_t)225, (uint8_t)238, (uint8_t)57, (uint8_t)222, (uint8_t)152, (uint8_t)177, (uint8_t)231, (uint8_t)91, (uint8_t)68, (uint8_t)21, (uint8_t)135, (uint8_t)46, (uint8_t)112, (uint8_t)10, (uint8_t)189, (uint8_t)232, (uint8_t)145, (uint8_t)103, (uint8_t)2, (uint8_t)101, (uint8_t)234, (uint8_t)43, (uint8_t)131, (uint8_t)216, (uint8_t)66, (uint8_t)68, (uint8_t)109, (uint8_t)255, (uint8_t)235, (uint8_t)136, (uint8_t)3, (uint8_t)22, (uint8_t)32, (uint8_t)178, (uint8_t)255, (uint8_t)105, (uint8_t)116, (uint8_t)35, (uint8_t)122, (uint8_t)7, (uint8_t)106, (uint8_t)6, (uint8_t)202, (uint8_t)62, (uint8_t)114, (uint8_t)198, (uint8_t)242, (uint8_t)173, (uint8_t)235, (uint8_t)217, (uint8_t)252, (uint8_t)166, (uint8_t)152, (uint8_t)127, (uint8_t)146, (uint8_t)186, (uint8_t)228, (uint8_t)241, (uint8_t)72, (uint8_t)137, (uint8_t)116, (uint8_t)129};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_target_component_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p267_sequence_SET((uint16_t)(uint16_t)25159, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_ACK_268(), &PH);
        p268_sequence_SET((uint16_t)(uint16_t)44443, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        p268_target_system_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_bitrate_SET((uint32_t)2472526889L, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)10277, PH.base.pack) ;
        {
            char16_t* uri = u"stcwsiiaplblcqkkIlivmzlnKYmSAjcukxHjrdq";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_status_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p269_framerate_SET((float)2.644777E38F, PH.base.pack) ;
        p269_rotation_SET((uint16_t)(uint16_t)51376, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)26043, PH.base.pack) ;
        c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_framerate_SET((float)6.9973796E37F, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)36210, PH.base.pack) ;
        {
            char16_t* uri = u"uqlxOygORxoukql";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_camera_id_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)48651, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)14720, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)2007766081L, PH.base.pack) ;
        p270_target_component_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
        c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* ssid = u"lliWykjnhcqnwnscmwfkv";
            p299_ssid_SET_(ssid, &PH) ;
        }
        {
            char16_t* password = u"fnxgcuWa";
            p299_password_SET_(password, &PH) ;
        }
        c_CommunicationChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PROTOCOL_VERSION_300(), &PH);
        p300_max_version_SET((uint16_t)(uint16_t)12313, PH.base.pack) ;
        p300_version_SET((uint16_t)(uint16_t)28237, PH.base.pack) ;
        p300_min_version_SET((uint16_t)(uint16_t)2370, PH.base.pack) ;
        {
            uint8_t spec_version_hash[] =  {(uint8_t)82, (uint8_t)100, (uint8_t)72, (uint8_t)232, (uint8_t)250, (uint8_t)129, (uint8_t)151, (uint8_t)118};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        {
            uint8_t library_version_hash[] =  {(uint8_t)202, (uint8_t)126, (uint8_t)108, (uint8_t)122, (uint8_t)7, (uint8_t)244, (uint8_t)175, (uint8_t)190};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_uptime_sec_SET((uint32_t)513399332L, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)55530, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_OK, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)1080011271449153016L, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_INFO_311(), &PH);
        {
            char16_t* name = u"wGufjiifptipabAvvomjkjmdckOysuhhyfedkhhtsofwvNupiukGDghceenyi";
            p311_name_SET_(name, &PH) ;
        }
        p311_hw_version_major_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)1080322266L, PH.base.pack) ;
        p311_sw_version_minor_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p311_sw_version_major_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
        p311_hw_version_minor_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)1295781270L, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)75, (uint8_t)203, (uint8_t)156, (uint8_t)8, (uint8_t)200, (uint8_t)0, (uint8_t)2, (uint8_t)187, (uint8_t)249, (uint8_t)156, (uint8_t)29, (uint8_t)174, (uint8_t)45, (uint8_t)122, (uint8_t)171, (uint8_t)181};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        p311_time_usec_SET((uint64_t)3717425399426368881L, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        p320_target_component_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        p320_param_index_SET((int16_t)(int16_t) -22867, PH.base.pack) ;
        {
            char16_t* param_id = u"rbiqIyXsppdsu";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_target_system_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_component_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        p321_target_system_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_VALUE_322(), &PH);
        {
            char16_t* param_value = u"gtqnnylwfbpDmqicHebatevxfafyBpyQbdtgtpqfhwP";
            p322_param_value_SET_(param_value, &PH) ;
        }
        p322_param_count_SET((uint16_t)(uint16_t)60745, PH.base.pack) ;
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64, PH.base.pack) ;
        p322_param_index_SET((uint16_t)(uint16_t)24701, PH.base.pack) ;
        {
            char16_t* param_id = u"upigdd";
            p322_param_id_SET_(param_id, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_SET_323(), &PH);
        {
            char16_t* param_id = u"rdeognqbjpzwHovk";
            p323_param_id_SET_(param_id, &PH) ;
        }
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32, PH.base.pack) ;
        p323_target_component_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        {
            char16_t* param_value = u"MlyttvtspubjceuThjvnfnwzyrdkkhxoddktvFtjeeorQzzzhfhbhepkexUcTHgkgemmewfsnuwegivgkmnSmyykxajkIejkfzvaAdrh";
            p323_param_value_SET_(param_value, &PH) ;
        }
        p323_target_system_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_ACK_324(), &PH);
        {
            char16_t* param_id = u"gylcfQWdstr";
            p324_param_id_SET_(param_id, &PH) ;
        }
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_IN_PROGRESS, PH.base.pack) ;
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64, PH.base.pack) ;
        {
            char16_t* param_value = u"ketShqihmigLslrsmWyrbxkWrhkmahbxgvvhDzccyufcjkqXxYvqvbjZaJzDsudtpkiMwleuhsAykdjK";
            p324_param_value_SET_(param_value, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_OBSTACLE_DISTANCE_330(), &PH);
        p330_increment_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
        p330_max_distance_SET((uint16_t)(uint16_t)22469, PH.base.pack) ;
        p330_time_usec_SET((uint64_t)5281984995944529875L, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)53760, (uint16_t)16994, (uint16_t)38921, (uint16_t)42230, (uint16_t)13469, (uint16_t)42249, (uint16_t)40964, (uint16_t)48488, (uint16_t)38355, (uint16_t)38515, (uint16_t)23708, (uint16_t)13759, (uint16_t)3541, (uint16_t)59710, (uint16_t)23386, (uint16_t)19197, (uint16_t)42871, (uint16_t)25563, (uint16_t)59819, (uint16_t)1531, (uint16_t)34299, (uint16_t)55700, (uint16_t)13518, (uint16_t)39290, (uint16_t)22524, (uint16_t)22582, (uint16_t)59842, (uint16_t)720, (uint16_t)30784, (uint16_t)56439, (uint16_t)51002, (uint16_t)7385, (uint16_t)49201, (uint16_t)15965, (uint16_t)952, (uint16_t)15067, (uint16_t)61706, (uint16_t)39823, (uint16_t)33199, (uint16_t)59084, (uint16_t)16446, (uint16_t)22766, (uint16_t)61743, (uint16_t)36762, (uint16_t)55614, (uint16_t)20140, (uint16_t)54331, (uint16_t)38686, (uint16_t)13060, (uint16_t)32471, (uint16_t)40501, (uint16_t)22531, (uint16_t)47961, (uint16_t)52517, (uint16_t)31687, (uint16_t)27489, (uint16_t)63971, (uint16_t)1633, (uint16_t)11376, (uint16_t)3850, (uint16_t)55919, (uint16_t)1555, (uint16_t)25982, (uint16_t)51974, (uint16_t)12428, (uint16_t)55681, (uint16_t)20792, (uint16_t)22095, (uint16_t)47340, (uint16_t)54652, (uint16_t)41155, (uint16_t)60118};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)39002, PH.base.pack) ;
        c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
}


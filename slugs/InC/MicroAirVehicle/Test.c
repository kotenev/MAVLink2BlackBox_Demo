
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
    assert(p0_custom_mode_GET(pack) == (uint32_t)2588040869L);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_VTOL_TILTROTOR);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_CRITICAL);
    assert(p0_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED);
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_OPENPILOT);
};


void c_TEST_Channel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)25234);
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)64675);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)9572);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)19186);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)59846);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)57777);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t)75);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t)7037);
    assert(p1_onboard_control_sensors_health_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
    assert(p1_onboard_control_sensors_present_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)41647);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)32120);
};


void c_TEST_Channel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)3006717002084327655L);
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)2560823329L);
};


void c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_x_GET(pack) == (float)3.3664583E38F);
    assert(p3_afx_GET(pack) == (float) -1.9116828E38F);
    assert(p3_vx_GET(pack) == (float)9.649564E37F);
    assert(p3_z_GET(pack) == (float)1.3186221E38F);
    assert(p3_vy_GET(pack) == (float)2.9454864E38F);
    assert(p3_vz_GET(pack) == (float) -2.8771981E38F);
    assert(p3_afz_GET(pack) == (float)1.5039224E38F);
    assert(p3_y_GET(pack) == (float)2.837918E38F);
    assert(p3_yaw_GET(pack) == (float)2.7323831E38F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)26504);
    assert(p3_afy_GET(pack) == (float) -1.4177441E38F);
    assert(p3_yaw_rate_GET(pack) == (float)5.919883E37F);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)1389850673L);
};


void c_TEST_Channel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_seq_GET(pack) == (uint32_t)1041195585L);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)243);
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p4_time_usec_GET(pack) == (uint64_t)5696199367167570760L);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)126);
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p5_passkey_LEN(ph) == 3);
    {
        char16_t * exemplary = u"dji";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)132);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)105);
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)234);
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)221);
};


void c_TEST_Channel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 28);
    {
        char16_t * exemplary = u"jwbqFiqlgsblfveqbigbtPnnamxo";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 56);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_custom_mode_GET(pack) == (uint32_t)1277609598L);
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED);
};


void c_TEST_Channel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)40);
    assert(p20_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"zOTGmvdvhftzr";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)48);
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t)27704);
};


void c_TEST_Channel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)213);
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)187);
};


void c_TEST_Channel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_value_GET(pack) == (float)8.0050345E37F);
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)45996);
    assert(p22_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"z";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)10211);
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64);
};


void c_TEST_Channel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)120);
    assert(p23_param_id_LEN(ph) == 8);
    {
        char16_t * exemplary = u"hgdylsvp";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_param_value_GET(pack) == (float)2.2259001E38F);
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16);
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)185);
};


void c_TEST_Channel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)49312);
    assert(p24_v_acc_TRY(ph) == (uint32_t)256554595L);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)3710163861L);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED);
    assert(p24_lon_GET(pack) == (int32_t)290237011);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)5535);
    assert(p24_alt_GET(pack) == (int32_t) -1105092772);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)3642784836L);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t)2083780793);
    assert(p24_time_usec_GET(pack) == (uint64_t)4342790446797420163L);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)14879);
    assert(p24_h_acc_TRY(ph) == (uint32_t)1345742975L);
    assert(p24_lat_GET(pack) == (int32_t)2008758175);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)54651);
};


void c_TEST_Channel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)113, (uint8_t)228, (uint8_t)168, (uint8_t)93, (uint8_t)108, (uint8_t)35, (uint8_t)2, (uint8_t)49, (uint8_t)242, (uint8_t)127, (uint8_t)180, (uint8_t)182, (uint8_t)244, (uint8_t)118, (uint8_t)145, (uint8_t)32, (uint8_t)123, (uint8_t)42, (uint8_t)173, (uint8_t)18} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)200, (uint8_t)108, (uint8_t)203, (uint8_t)79, (uint8_t)149, (uint8_t)52, (uint8_t)3, (uint8_t)190, (uint8_t)45, (uint8_t)238, (uint8_t)88, (uint8_t)190, (uint8_t)246, (uint8_t)178, (uint8_t)33, (uint8_t)248, (uint8_t)16, (uint8_t)232, (uint8_t)88, (uint8_t)24} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)207);
    {
        uint8_t exemplary[] =  {(uint8_t)105, (uint8_t)17, (uint8_t)28, (uint8_t)106, (uint8_t)222, (uint8_t)215, (uint8_t)69, (uint8_t)255, (uint8_t)238, (uint8_t)182, (uint8_t)146, (uint8_t)144, (uint8_t)156, (uint8_t)163, (uint8_t)242, (uint8_t)221, (uint8_t)118, (uint8_t)70, (uint8_t)12, (uint8_t)222} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)142, (uint8_t)106, (uint8_t)73, (uint8_t)38, (uint8_t)7, (uint8_t)131, (uint8_t)55, (uint8_t)144, (uint8_t)122, (uint8_t)9, (uint8_t)99, (uint8_t)42, (uint8_t)11, (uint8_t)175, (uint8_t)175, (uint8_t)247, (uint8_t)134, (uint8_t)255, (uint8_t)151, (uint8_t)95} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)56, (uint8_t)12, (uint8_t)19, (uint8_t)116, (uint8_t)170, (uint8_t)216, (uint8_t)50, (uint8_t)196, (uint8_t)247, (uint8_t)190, (uint8_t)248, (uint8_t)120, (uint8_t)36, (uint8_t)158, (uint8_t)65, (uint8_t)132, (uint8_t)70, (uint8_t)7, (uint8_t)143, (uint8_t)186} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t) -31076);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t) -4857);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t) -21362);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t) -1104);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t)14671);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t) -1631);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)2580115909L);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t) -6624);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t)10065);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t)969);
};


void c_TEST_Channel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t)31942);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t)18309);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t) -9999);
    assert(p27_time_usec_GET(pack) == (uint64_t)2849045302744260408L);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t)27012);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t)31228);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t)14138);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t)21214);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t)20452);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t) -31941);
};


void c_TEST_Channel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t)354);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t) -13597);
    assert(p28_time_usec_GET(pack) == (uint64_t)2428858149517646322L);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t)24820);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t) -21351);
};


void c_TEST_Channel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)289928300L);
    assert(p29_press_diff_GET(pack) == (float)2.6633962E38F);
    assert(p29_press_abs_GET(pack) == (float) -2.000196E38F);
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t)7835);
};


void c_TEST_Channel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_pitch_GET(pack) == (float)2.0670073E38F);
    assert(p30_yawspeed_GET(pack) == (float)2.933856E38F);
    assert(p30_yaw_GET(pack) == (float)4.8542984E36F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)2222553745L);
    assert(p30_roll_GET(pack) == (float)1.7561372E38F);
    assert(p30_rollspeed_GET(pack) == (float) -1.4925807E38F);
    assert(p30_pitchspeed_GET(pack) == (float) -7.378402E37F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_q4_GET(pack) == (float)7.398097E37F);
    assert(p31_q2_GET(pack) == (float)2.017071E38F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)2302489818L);
    assert(p31_yawspeed_GET(pack) == (float)2.0195762E37F);
    assert(p31_q1_GET(pack) == (float)2.7724622E38F);
    assert(p31_pitchspeed_GET(pack) == (float) -9.633772E37F);
    assert(p31_q3_GET(pack) == (float) -7.6729304E37F);
    assert(p31_rollspeed_GET(pack) == (float)1.0238272E38F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)553880383L);
    assert(p32_vz_GET(pack) == (float)2.4119257E38F);
    assert(p32_vy_GET(pack) == (float) -2.366788E38F);
    assert(p32_z_GET(pack) == (float)3.2542475E38F);
    assert(p32_vx_GET(pack) == (float) -6.0139494E37F);
    assert(p32_x_GET(pack) == (float)1.0681679E38F);
    assert(p32_y_GET(pack) == (float)2.5022445E38F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_alt_GET(pack) == (int32_t) -1311474927);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t) -16767);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)3432);
    assert(p33_lat_GET(pack) == (int32_t) -741194677);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t) -16606);
    assert(p33_lon_GET(pack) == (int32_t)791322167);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t) -22008);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)1109798377L);
    assert(p33_relative_alt_GET(pack) == (int32_t) -528504218);
};


void c_TEST_Channel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)2963744756L);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t)14004);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)246);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t)16902);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t) -15316);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t) -4322);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t)27626);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t)31820);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t)27481);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t)24914);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)152);
};


void c_TEST_Channel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)57801);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)8472);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)1230914968L);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)28810);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)9822);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)47060);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)49);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)36500);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)43518);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)24041);
};


void c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)64573);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)15164);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)57737);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)35159);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)17731);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)42952);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)2257);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)58127);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)47778);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)43580);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)65159);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)52271);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)53469);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)28342);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)65205);
    assert(p36_time_usec_GET(pack) == (uint32_t)2554852049L);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)53989);
};


void c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)91);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t)26148);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t) -20157);
};


void c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)86);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t) -30792);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t)17174);
};


void c_TEST_Channel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_param1_GET(pack) == (float) -3.0556894E38F);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)91);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)246);
    assert(p39_param3_GET(pack) == (float) -2.0849007E38F);
    assert(p39_param2_GET(pack) == (float) -1.6986304E38F);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)12453);
    assert(p39_y_GET(pack) == (float) -1.887468E38F);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p39_z_GET(pack) == (float) -2.3284858E37F);
    assert(p39_param4_GET(pack) == (float) -2.9774281E38F);
    assert(p39_x_GET(pack) == (float)5.52412E37F);
};


void c_TEST_Channel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)62);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)5495);
};


void c_TEST_Channel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)89);
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)29478);
};


void c_TEST_Channel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)49332);
};


void c_TEST_Channel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)89);
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)126);
};


void c_TEST_Channel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)62);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)55790);
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)208);
};


void c_TEST_Channel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)216);
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)185);
};


void c_TEST_Channel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)18553);
};


void c_TEST_Channel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM1);
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)101);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)76);
};


void c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_latitude_GET(pack) == (int32_t)1233035359);
    assert(p48_longitude_GET(pack) == (int32_t)862802992);
    assert(p48_altitude_GET(pack) == (int32_t)1955629113);
    assert(p48_time_usec_TRY(ph) == (uint64_t)5894979835848281260L);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)169);
};


void c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_time_usec_TRY(ph) == (uint64_t)3884016250955752138L);
    assert(p49_altitude_GET(pack) == (int32_t) -1429838968);
    assert(p49_latitude_GET(pack) == (int32_t)1369347535);
    assert(p49_longitude_GET(pack) == (int32_t) -2072035595);
};


void c_TEST_Channel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_param_value0_GET(pack) == (float)3.3725787E38F);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t)12029);
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)208);
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p50_param_id_LEN(ph) == 16);
    {
        char16_t * exemplary = u"PxhskdXffrJeCrlw";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_param_value_max_GET(pack) == (float)2.728794E38F);
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)207);
    assert(p50_param_value_min_GET(pack) == (float) -2.4902438E37F);
    assert(p50_scale_GET(pack) == (float)2.2729497E38F);
};


void c_TEST_Channel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)24875);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)46);
};


void c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_p1z_GET(pack) == (float)1.9929335E38F);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p54_p1y_GET(pack) == (float)6.057524E37F);
    assert(p54_p2y_GET(pack) == (float)2.423597E38F);
    assert(p54_p2z_GET(pack) == (float) -3.2671917E38F);
    assert(p54_p2x_GET(pack) == (float) -1.749972E38F);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)183);
    assert(p54_p1x_GET(pack) == (float) -1.2124232E38F);
};


void c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p1z_GET(pack) == (float) -8.471325E37F);
    assert(p55_p1y_GET(pack) == (float)2.242803E38F);
    assert(p55_p2y_GET(pack) == (float) -1.7706823E38F);
    assert(p55_p1x_GET(pack) == (float) -1.91839E38F);
    assert(p55_p2x_GET(pack) == (float)3.1691131E38F);
    assert(p55_p2z_GET(pack) == (float)4.841926E37F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    assert(p61_yawspeed_GET(pack) == (float) -1.8946653E38F);
    assert(p61_pitchspeed_GET(pack) == (float)1.0350404E38F);
    {
        float exemplary[] =  {-3.2124688E38F, -2.0430964E38F, 2.4926877E38F, -3.1524112E38F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_time_usec_GET(pack) == (uint64_t)3651969912067895099L);
    {
        float exemplary[] =  {-1.9221485E38F, 2.4178993E38F, 1.8180485E38F, 2.9718068E38F, 1.7870898E38F, -6.0043194E37F, -1.8932656E38F, -1.0013715E38F, -1.6219966E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_rollspeed_GET(pack) == (float)6.1506316E37F);
};


void c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t) -32181);
    assert(p62_alt_error_GET(pack) == (float)4.7566723E37F);
    assert(p62_aspd_error_GET(pack) == (float) -2.4898078E38F);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t) -142);
    assert(p62_nav_roll_GET(pack) == (float)2.4081122E38F);
    assert(p62_xtrack_error_GET(pack) == (float) -1.4828465E38F);
    assert(p62_nav_pitch_GET(pack) == (float)9.653279E37F);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)59368);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_vx_GET(pack) == (float)2.4206626E38F);
    assert(p63_time_usec_GET(pack) == (uint64_t)7266695339762267279L);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS);
    assert(p63_lon_GET(pack) == (int32_t)1569097259);
    assert(p63_alt_GET(pack) == (int32_t)911946655);
    assert(p63_vy_GET(pack) == (float) -3.357298E37F);
    assert(p63_relative_alt_GET(pack) == (int32_t)445912849);
    {
        float exemplary[] =  {8.1721844E37F, 1.1451685E38F, -3.0626948E38F, -8.2992765E37F, 7.6783083E37F, -1.4441783E37F, -1.0034734E38F, -2.7122123E38F, -3.3119684E38F, -1.4214218E38F, 8.3957924E37F, -3.0075436E38F, 2.2809644E38F, -8.945184E37F, -2.230106E38F, -2.3184957E37F, 2.4246527E38F, 8.513528E37F, -1.3449332E38F, -1.6514884E37F, 4.893824E37F, -2.1918644E38F, 9.879083E37F, 2.8718975E38F, 3.1912627E38F, -1.3652356E38F, 1.551185E35F, 2.954596E38F, -1.218206E37F, 2.3888853E38F, -1.5947448E38F, -3.2555707E38F, 2.7754997E38F, -3.4108627E37F, 2.921796E37F, -3.402068E38F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_lat_GET(pack) == (int32_t) -1002884933);
    assert(p63_vz_GET(pack) == (float) -1.2741902E38F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_ax_GET(pack) == (float) -2.1060666E38F);
    assert(p64_z_GET(pack) == (float)5.355817E37F);
    assert(p64_vx_GET(pack) == (float) -2.8798892E37F);
    assert(p64_ay_GET(pack) == (float) -8.081211E37F);
    {
        float exemplary[] =  {1.9248606E37F, 3.1115695E38F, 6.052559E37F, -2.6584692E38F, 1.0833009E38F, 5.3149687E36F, -2.045917E38F, 7.6804704E37F, -7.8230653E37F, -9.7712315E36F, 8.251791E37F, 3.9670644E37F, 2.5207117E38F, 9.149471E37F, -1.2550986E38F, -1.8891422E38F, 2.9272945E38F, -3.028248E37F, -3.1424044E37F, 1.1996983E38F, 2.8548532E38F, -1.8029389E38F, -1.8067406E38F, 1.6401287E38F, -3.3340957E38F, -9.731827E37F, -3.0359488E38F, -2.3442192E38F, 2.4935324E38F, -1.4252892E38F, -2.1436012E38F, -1.6928125E38F, -2.2656176E38F, -7.470767E37F, 1.823955E38F, 2.3205295E38F, -2.6945208E38F, -2.6355945E38F, 3.0762012E38F, 6.5407607E37F, 1.2711362E38F, -1.0533232E38F, -2.023761E38F, -2.8617646E38F, 1.1298783E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_x_GET(pack) == (float) -1.219195E38F);
    assert(p64_time_usec_GET(pack) == (uint64_t)3238861729715010795L);
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION);
    assert(p64_y_GET(pack) == (float)4.1672644E37F);
    assert(p64_az_GET(pack) == (float)2.606813E38F);
    assert(p64_vz_GET(pack) == (float) -2.9973596E38F);
    assert(p64_vy_GET(pack) == (float)2.8617786E38F);
};


void c_TEST_Channel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)852732036L);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)34280);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)32348);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)14027);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)49241);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)36578);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)25079);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)16702);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)10981);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)33628);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)28345);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)5);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)21996);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)35412);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)59951);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)25752);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)65518);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)11539);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)4784);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)60139);
};


void c_TEST_Channel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)251);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)182);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)43801);
};


void c_TEST_Channel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)47033);
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)79);
};


void c_TEST_Channel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_r_GET(pack) == (int16_t)(int16_t)29326);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)29150);
    assert(p69_x_GET(pack) == (int16_t)(int16_t) -3706);
    assert(p69_y_GET(pack) == (int16_t)(int16_t) -9645);
    assert(p69_z_GET(pack) == (int16_t)(int16_t)29873);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)92);
};


void c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)52042);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)1745);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)29323);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)33226);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)91);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)30483);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)63723);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)802);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)224);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)41369);
};


void c_TEST_Channel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_param1_GET(pack) == (float)1.592505E38F);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)34632);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)134);
    assert(p73_param3_GET(pack) == (float)3.3955934E38F);
    assert(p73_param2_GET(pack) == (float) -4.351673E37F);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p73_y_GET(pack) == (int32_t) -1554072960);
    assert(p73_x_GET(pack) == (int32_t)1755720764);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)201);
    assert(p73_param4_GET(pack) == (float)1.5997715E38F);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p73_z_GET(pack) == (float) -1.4173052E38F);
};


void c_TEST_Channel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)32015);
    assert(p74_alt_GET(pack) == (float)7.902866E37F);
    assert(p74_climb_GET(pack) == (float)2.974927E38F);
    assert(p74_airspeed_GET(pack) == (float) -1.7381563E38F);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t)29312);
    assert(p74_groundspeed_GET(pack) == (float)1.1167466E38F);
};


void c_TEST_Channel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)111);
    assert(p75_y_GET(pack) == (int32_t)430182603);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE);
    assert(p75_param2_GET(pack) == (float)3.3342576E38F);
    assert(p75_x_GET(pack) == (int32_t)941072442);
    assert(p75_z_GET(pack) == (float) -8.410628E37F);
    assert(p75_param1_GET(pack) == (float) -2.035717E38F);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)90);
    assert(p75_param4_GET(pack) == (float) -2.8123031E38F);
    assert(p75_param3_GET(pack) == (float)2.2291092E38F);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)222);
};


void c_TEST_Channel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_param5_GET(pack) == (float) -2.1444863E37F);
    assert(p76_param3_GET(pack) == (float) -1.9368212E38F);
    assert(p76_param4_GET(pack) == (float)2.6685858E38F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)172);
    assert(p76_param2_GET(pack) == (float)2.082015E38F);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p76_param7_GET(pack) == (float)9.995622E37F);
    assert(p76_param1_GET(pack) == (float) -4.2524166E37F);
    assert(p76_param6_GET(pack) == (float) -1.1510447E38F);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)90);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_STOP_RETURN_TO_BASE);
};


void c_TEST_Channel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)232);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)76);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_IN_PROGRESS);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)96);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_FOLLOW);
    assert(p77_result_param2_TRY(ph) == (int32_t) -1340272510);
};


void c_TEST_Channel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)153);
    assert(p81_roll_GET(pack) == (float)3.2915217E38F);
    assert(p81_pitch_GET(pack) == (float) -3.2986232E38F);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)165);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)3944169920L);
    assert(p81_yaw_GET(pack) == (float)2.1937834E38F);
    assert(p81_thrust_GET(pack) == (float)2.434619E38F);
};


void c_TEST_Channel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_body_roll_rate_GET(pack) == (float) -6.9007816E37F);
    {
        float exemplary[] =  {-1.8710545E38F, -8.532836E37F, -1.7835856E38F, -2.555077E37F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)77);
    assert(p82_body_yaw_rate_GET(pack) == (float)2.700631E38F);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p82_body_pitch_rate_GET(pack) == (float) -9.678533E37F);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)3693377612L);
    assert(p82_thrust_GET(pack) == (float) -1.8610883E38F);
};


void c_CommunicationChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_body_pitch_rate_GET(pack) == (float) -1.5025551E38F);
    assert(p83_body_yaw_rate_GET(pack) == (float)1.1375035E38F);
    assert(p83_body_roll_rate_GET(pack) == (float)6.680417E37F);
    assert(p83_thrust_GET(pack) == (float) -3.0981172E38F);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)258674316L);
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)176);
    {
        float exemplary[] =  {1.2181909E38F, 1.1662056E36F, 1.9743857E38F, -1.937355E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_afx_GET(pack) == (float)7.364943E37F);
    assert(p84_yaw_GET(pack) == (float)1.1360697E38F);
    assert(p84_afz_GET(pack) == (float) -3.2674424E38F);
    assert(p84_yaw_rate_GET(pack) == (float)1.5120908E38F);
    assert(p84_vx_GET(pack) == (float) -2.7593993E38F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)20);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)1622336177L);
    assert(p84_vz_GET(pack) == (float) -3.3795123E38F);
    assert(p84_x_GET(pack) == (float) -2.8884202E38F);
    assert(p84_z_GET(pack) == (float) -2.6390472E37F);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p84_afy_GET(pack) == (float) -1.269506E38F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)62274);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT);
    assert(p84_y_GET(pack) == (float) -2.6781979E38F);
    assert(p84_vy_GET(pack) == (float)2.7181593E37F);
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_lat_int_GET(pack) == (int32_t)710505546);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)3183127443L);
    assert(p86_alt_GET(pack) == (float) -3.3546828E38F);
    assert(p86_yaw_rate_GET(pack) == (float)3.3854123E37F);
    assert(p86_afx_GET(pack) == (float)1.3708789E38F);
    assert(p86_yaw_GET(pack) == (float)2.3623576E38F);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)201);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)60180);
    assert(p86_vz_GET(pack) == (float) -3.0510212E38F);
    assert(p86_afz_GET(pack) == (float)2.000327E38F);
    assert(p86_vy_GET(pack) == (float)9.914073E37F);
    assert(p86_vx_GET(pack) == (float)1.4245735E38F);
    assert(p86_lon_int_GET(pack) == (int32_t) -1823943115);
    assert(p86_afy_GET(pack) == (float)1.0818347E38F);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)235);
};


void c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_vz_GET(pack) == (float) -3.096562E38F);
    assert(p87_afy_GET(pack) == (float)2.5851588E38F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)2433299078L);
    assert(p87_vy_GET(pack) == (float) -1.9735776E38F);
    assert(p87_yaw_rate_GET(pack) == (float)1.2561414E38F);
    assert(p87_afz_GET(pack) == (float)3.2768819E38F);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p87_afx_GET(pack) == (float) -1.814004E38F);
    assert(p87_alt_GET(pack) == (float)1.3030943E38F);
    assert(p87_yaw_GET(pack) == (float) -1.8895488E38F);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)24417);
    assert(p87_vx_GET(pack) == (float)3.1858317E38F);
    assert(p87_lon_int_GET(pack) == (int32_t)2118215883);
    assert(p87_lat_int_GET(pack) == (int32_t) -389642569);
};


void c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_pitch_GET(pack) == (float)1.3921978E38F);
    assert(p89_yaw_GET(pack) == (float) -1.6769405E38F);
    assert(p89_z_GET(pack) == (float) -2.6036261E38F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)1608700944L);
    assert(p89_roll_GET(pack) == (float) -5.158722E37F);
    assert(p89_x_GET(pack) == (float)2.5181733E38F);
    assert(p89_y_GET(pack) == (float) -2.1227453E37F);
};


void c_CommunicationChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t)24354);
    assert(p90_pitchspeed_GET(pack) == (float) -1.6897096E38F);
    assert(p90_alt_GET(pack) == (int32_t) -579698579);
    assert(p90_roll_GET(pack) == (float)6.3784096E37F);
    assert(p90_yaw_GET(pack) == (float) -3.09333E38F);
    assert(p90_pitch_GET(pack) == (float)2.6701226E38F);
    assert(p90_lon_GET(pack) == (int32_t) -2043320738);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t) -27403);
    assert(p90_lat_GET(pack) == (int32_t) -358195133);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t)27778);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t)20025);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t) -9911);
    assert(p90_rollspeed_GET(pack) == (float) -2.3489805E38F);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t) -32252);
    assert(p90_time_usec_GET(pack) == (uint64_t)387537790346673982L);
    assert(p90_yawspeed_GET(pack) == (float)2.8519364E38F);
};


void c_CommunicationChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p91_pitch_elevator_GET(pack) == (float) -6.5247107E37F);
    assert(p91_yaw_rudder_GET(pack) == (float) -2.5590838E36F);
    assert(p91_time_usec_GET(pack) == (uint64_t)9172628609464634319L);
    assert(p91_aux2_GET(pack) == (float) -4.478867E37F);
    assert(p91_aux3_GET(pack) == (float) -2.0561053E38F);
    assert(p91_throttle_GET(pack) == (float)2.363879E38F);
    assert(p91_aux1_GET(pack) == (float) -2.4742051E38F);
    assert(p91_aux4_GET(pack) == (float) -2.861018E38F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_AUTO_ARMED);
    assert(p91_roll_ailerons_GET(pack) == (float) -7.531352E37F);
};


void c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)42289);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)24963);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)41095);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)10696);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)11219);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)43043);
    assert(p92_time_usec_GET(pack) == (uint64_t)3319771575576279079L);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)10329);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)39410);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)21611);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)21177);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)45989);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)10373);
};


void c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    assert(p93_flags_GET(pack) == (uint64_t)985421710929194079L);
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_GUIDED_ARMED);
    {
        float exemplary[] =  {2.6321818E38F, 1.0498137E38F, -1.4393788E38F, 6.5344346E37F, 2.7724476E38F, -3.038045E38F, -7.094769E37F, 2.6067898E38F, 2.0182056E38F, 1.05245905E37F, 2.0568375E38F, -1.945764E38F, -8.834751E37F, 6.3372206E37F, -5.539322E37F, -1.2913188E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_time_usec_GET(pack) == (uint64_t)6979744782280549529L);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_time_usec_GET(pack) == (uint64_t)8646101139458236348L);
    assert(p100_flow_rate_y_TRY(ph) == (float)2.8538052E38F);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)181);
    assert(p100_flow_comp_m_y_GET(pack) == (float)1.4565572E38F);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)145);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t) -27540);
    assert(p100_flow_comp_m_x_GET(pack) == (float) -7.046211E37F);
    assert(p100_flow_rate_x_TRY(ph) == (float) -1.2886359E38F);
    assert(p100_ground_distance_GET(pack) == (float) -3.3031344E37F);
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t) -5667);
};


void c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_x_GET(pack) == (float) -4.8679963E37F);
    assert(p101_z_GET(pack) == (float)9.579718E37F);
    assert(p101_pitch_GET(pack) == (float) -3.612081E37F);
    assert(p101_yaw_GET(pack) == (float) -3.0983038E38F);
    assert(p101_usec_GET(pack) == (uint64_t)5945399769847346240L);
    assert(p101_y_GET(pack) == (float)8.211362E37F);
    assert(p101_roll_GET(pack) == (float) -2.1985465E38F);
};


void c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_y_GET(pack) == (float) -1.6518992E38F);
    assert(p102_yaw_GET(pack) == (float) -2.9647283E38F);
    assert(p102_roll_GET(pack) == (float)6.9460413E37F);
    assert(p102_z_GET(pack) == (float)3.1745484E38F);
    assert(p102_pitch_GET(pack) == (float)2.8884784E38F);
    assert(p102_usec_GET(pack) == (uint64_t)170013366537189483L);
    assert(p102_x_GET(pack) == (float)1.4888121E38F);
};


void c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_y_GET(pack) == (float)1.0391069E38F);
    assert(p103_x_GET(pack) == (float)2.1407868E38F);
    assert(p103_usec_GET(pack) == (uint64_t)8738493259555063444L);
    assert(p103_z_GET(pack) == (float)1.2438133E38F);
};


void c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_yaw_GET(pack) == (float) -4.341404E37F);
    assert(p104_z_GET(pack) == (float)1.1631432E38F);
    assert(p104_pitch_GET(pack) == (float) -2.8733495E38F);
    assert(p104_y_GET(pack) == (float) -8.09822E37F);
    assert(p104_usec_GET(pack) == (uint64_t)7948476928411481151L);
    assert(p104_x_GET(pack) == (float) -1.660804E38F);
    assert(p104_roll_GET(pack) == (float)5.4264243E37F);
};


void c_CommunicationChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_ygyro_GET(pack) == (float)3.3376605E38F);
    assert(p105_diff_pressure_GET(pack) == (float)2.8555049E38F);
    assert(p105_xacc_GET(pack) == (float)1.7553393E38F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)21081);
    assert(p105_xmag_GET(pack) == (float)2.0693717E37F);
    assert(p105_zgyro_GET(pack) == (float)2.6215091E38F);
    assert(p105_temperature_GET(pack) == (float) -1.4741927E38F);
    assert(p105_ymag_GET(pack) == (float) -5.040475E37F);
    assert(p105_time_usec_GET(pack) == (uint64_t)8148826324743800527L);
    assert(p105_zmag_GET(pack) == (float) -3.9094099E37F);
    assert(p105_yacc_GET(pack) == (float)1.9255643E38F);
    assert(p105_zacc_GET(pack) == (float)3.3230278E38F);
    assert(p105_pressure_alt_GET(pack) == (float)3.028618E38F);
    assert(p105_xgyro_GET(pack) == (float) -1.2961786E38F);
    assert(p105_abs_pressure_GET(pack) == (float) -1.7173844E38F);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)178);
    assert(p106_time_usec_GET(pack) == (uint64_t)6780159706924010568L);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)889709337L);
    assert(p106_integrated_xgyro_GET(pack) == (float)4.1873618E36F);
    assert(p106_integrated_zgyro_GET(pack) == (float) -2.6118916E38F);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)3449153362L);
    assert(p106_integrated_ygyro_GET(pack) == (float)2.8325784E38F);
    assert(p106_integrated_y_GET(pack) == (float)9.554746E37F);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t) -7578);
    assert(p106_distance_GET(pack) == (float)2.6345607E38F);
    assert(p106_integrated_x_GET(pack) == (float) -1.927174E38F);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)200);
};


void c_CommunicationChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_xmag_GET(pack) == (float) -1.882278E38F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)3067190894L);
    assert(p107_diff_pressure_GET(pack) == (float) -2.9666578E38F);
    assert(p107_pressure_alt_GET(pack) == (float) -1.8705387E38F);
    assert(p107_temperature_GET(pack) == (float) -4.833042E37F);
    assert(p107_xacc_GET(pack) == (float) -2.0044343E38F);
    assert(p107_time_usec_GET(pack) == (uint64_t)249636794737141773L);
    assert(p107_xgyro_GET(pack) == (float) -4.874964E37F);
    assert(p107_zmag_GET(pack) == (float) -9.149251E36F);
    assert(p107_abs_pressure_GET(pack) == (float) -3.2593644E38F);
    assert(p107_zacc_GET(pack) == (float)1.3329283E38F);
    assert(p107_yacc_GET(pack) == (float)1.920478E36F);
    assert(p107_ymag_GET(pack) == (float) -2.0957508E38F);
    assert(p107_zgyro_GET(pack) == (float) -2.1724796E37F);
    assert(p107_ygyro_GET(pack) == (float)2.7010493E38F);
};


void c_CommunicationChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_roll_GET(pack) == (float)3.517438E37F);
    assert(p108_q4_GET(pack) == (float)3.6965992E37F);
    assert(p108_std_dev_vert_GET(pack) == (float)2.4856965E37F);
    assert(p108_q2_GET(pack) == (float) -1.1580841E38F);
    assert(p108_ygyro_GET(pack) == (float)2.887184E37F);
    assert(p108_zgyro_GET(pack) == (float)5.028782E37F);
    assert(p108_q1_GET(pack) == (float)6.6372583E37F);
    assert(p108_vn_GET(pack) == (float) -1.9777714E38F);
    assert(p108_xacc_GET(pack) == (float) -1.962687E38F);
    assert(p108_pitch_GET(pack) == (float) -2.2860492E38F);
    assert(p108_vd_GET(pack) == (float) -2.1631543E38F);
    assert(p108_std_dev_horz_GET(pack) == (float) -3.0286612E37F);
    assert(p108_q3_GET(pack) == (float)2.3306018E38F);
    assert(p108_alt_GET(pack) == (float) -2.1091832E38F);
    assert(p108_ve_GET(pack) == (float) -4.0166026E37F);
    assert(p108_zacc_GET(pack) == (float)2.3991888E38F);
    assert(p108_yacc_GET(pack) == (float)1.2276604E37F);
    assert(p108_lat_GET(pack) == (float)4.379369E37F);
    assert(p108_lon_GET(pack) == (float)4.0281608E37F);
    assert(p108_yaw_GET(pack) == (float) -2.416675E37F);
    assert(p108_xgyro_GET(pack) == (float)3.1419513E38F);
};


void c_CommunicationChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)232);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)42033);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)95);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)56495);
};


void c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)37);
    {
        uint8_t exemplary[] =  {(uint8_t)44, (uint8_t)60, (uint8_t)247, (uint8_t)99, (uint8_t)5, (uint8_t)172, (uint8_t)60, (uint8_t)174, (uint8_t)134, (uint8_t)201, (uint8_t)160, (uint8_t)84, (uint8_t)159, (uint8_t)126, (uint8_t)129, (uint8_t)101, (uint8_t)187, (uint8_t)176, (uint8_t)210, (uint8_t)226, (uint8_t)53, (uint8_t)10, (uint8_t)236, (uint8_t)181, (uint8_t)29, (uint8_t)25, (uint8_t)146, (uint8_t)92, (uint8_t)68, (uint8_t)88, (uint8_t)103, (uint8_t)139, (uint8_t)209, (uint8_t)124, (uint8_t)70, (uint8_t)20, (uint8_t)209, (uint8_t)16, (uint8_t)211, (uint8_t)7, (uint8_t)201, (uint8_t)4, (uint8_t)150, (uint8_t)38, (uint8_t)166, (uint8_t)15, (uint8_t)81, (uint8_t)124, (uint8_t)253, (uint8_t)110, (uint8_t)166, (uint8_t)240, (uint8_t)164, (uint8_t)237, (uint8_t)129, (uint8_t)104, (uint8_t)182, (uint8_t)22, (uint8_t)40, (uint8_t)114, (uint8_t)140, (uint8_t)66, (uint8_t)92, (uint8_t)9, (uint8_t)205, (uint8_t)46, (uint8_t)189, (uint8_t)19, (uint8_t)114, (uint8_t)90, (uint8_t)10, (uint8_t)145, (uint8_t)211, (uint8_t)211, (uint8_t)45, (uint8_t)219, (uint8_t)25, (uint8_t)167, (uint8_t)11, (uint8_t)243, (uint8_t)110, (uint8_t)239, (uint8_t)101, (uint8_t)98, (uint8_t)43, (uint8_t)142, (uint8_t)119, (uint8_t)30, (uint8_t)69, (uint8_t)47, (uint8_t)4, (uint8_t)2, (uint8_t)24, (uint8_t)109, (uint8_t)209, (uint8_t)97, (uint8_t)20, (uint8_t)97, (uint8_t)172, (uint8_t)82, (uint8_t)129, (uint8_t)162, (uint8_t)21, (uint8_t)0, (uint8_t)93, (uint8_t)206, (uint8_t)166, (uint8_t)124, (uint8_t)191, (uint8_t)60, (uint8_t)168, (uint8_t)138, (uint8_t)75, (uint8_t)166, (uint8_t)136, (uint8_t)19, (uint8_t)99, (uint8_t)41, (uint8_t)175, (uint8_t)125, (uint8_t)81, (uint8_t)2, (uint8_t)158, (uint8_t)194, (uint8_t)53, (uint8_t)238, (uint8_t)240, (uint8_t)181, (uint8_t)182, (uint8_t)40, (uint8_t)139, (uint8_t)206, (uint8_t)74, (uint8_t)196, (uint8_t)112, (uint8_t)36, (uint8_t)91, (uint8_t)243, (uint8_t)32, (uint8_t)96, (uint8_t)79, (uint8_t)45, (uint8_t)115, (uint8_t)60, (uint8_t)163, (uint8_t)57, (uint8_t)82, (uint8_t)56, (uint8_t)181, (uint8_t)9, (uint8_t)104, (uint8_t)101, (uint8_t)231, (uint8_t)139, (uint8_t)58, (uint8_t)164, (uint8_t)207, (uint8_t)59, (uint8_t)8, (uint8_t)227, (uint8_t)69, (uint8_t)228, (uint8_t)28, (uint8_t)168, (uint8_t)205, (uint8_t)46, (uint8_t)113, (uint8_t)66, (uint8_t)225, (uint8_t)21, (uint8_t)128, (uint8_t)71, (uint8_t)172, (uint8_t)187, (uint8_t)189, (uint8_t)57, (uint8_t)122, (uint8_t)32, (uint8_t)102, (uint8_t)117, (uint8_t)187, (uint8_t)190, (uint8_t)186, (uint8_t)172, (uint8_t)83, (uint8_t)175, (uint8_t)228, (uint8_t)98, (uint8_t)71, (uint8_t)74, (uint8_t)249, (uint8_t)228, (uint8_t)117, (uint8_t)163, (uint8_t)232, (uint8_t)189, (uint8_t)53, (uint8_t)12, (uint8_t)184, (uint8_t)251, (uint8_t)72, (uint8_t)217, (uint8_t)59, (uint8_t)207, (uint8_t)238, (uint8_t)122, (uint8_t)32, (uint8_t)140, (uint8_t)1, (uint8_t)34, (uint8_t)185, (uint8_t)125, (uint8_t)109, (uint8_t)196, (uint8_t)62, (uint8_t)15, (uint8_t)112, (uint8_t)215, (uint8_t)20, (uint8_t)13, (uint8_t)140, (uint8_t)86, (uint8_t)53, (uint8_t)19, (uint8_t)110, (uint8_t)249, (uint8_t)235, (uint8_t)103, (uint8_t)10, (uint8_t)174, (uint8_t)25, (uint8_t)192, (uint8_t)32, (uint8_t)23, (uint8_t)171, (uint8_t)153, (uint8_t)192, (uint8_t)138, (uint8_t)4, (uint8_t)147, (uint8_t)30, (uint8_t)251, (uint8_t)171, (uint8_t)37, (uint8_t)180, (uint8_t)45, (uint8_t)38, (uint8_t)174, (uint8_t)102, (uint8_t)70, (uint8_t)40} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)20);
};


void c_CommunicationChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_tc1_GET(pack) == (int64_t) -128638384845514173L);
    assert(p111_ts1_GET(pack) == (int64_t)4916714548945593517L);
};


void c_CommunicationChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_seq_GET(pack) == (uint32_t)3668542228L);
    assert(p112_time_usec_GET(pack) == (uint64_t)1059027319243958914L);
};


void c_CommunicationChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_lat_GET(pack) == (int32_t) -270971975);
    assert(p113_lon_GET(pack) == (int32_t) -1059693012);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)56873);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)160);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t) -7078);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t) -14880);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t) -4987);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)41213);
    assert(p113_time_usec_GET(pack) == (uint64_t)8635047198574112312L);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)1346);
    assert(p113_alt_GET(pack) == (int32_t)976425434);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)45103);
};


void c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)2119972653L);
    assert(p114_distance_GET(pack) == (float) -2.2260273E38F);
    assert(p114_time_usec_GET(pack) == (uint64_t)6609154870585588291L);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)3304139886L);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t) -32442);
    assert(p114_integrated_zgyro_GET(pack) == (float) -5.6332015E37F);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)171);
    assert(p114_integrated_y_GET(pack) == (float)8.1138446E37F);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)165);
    assert(p114_integrated_ygyro_GET(pack) == (float)1.404539E38F);
    assert(p114_integrated_x_GET(pack) == (float)4.0774902E37F);
    assert(p114_integrated_xgyro_GET(pack) == (float) -2.6060503E38F);
};


void c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_time_usec_GET(pack) == (uint64_t)2045411262067085335L);
    assert(p115_rollspeed_GET(pack) == (float)3.34847E38F);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t)12356);
    assert(p115_yawspeed_GET(pack) == (float) -3.0217728E38F);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t)19552);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)8059);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t)6839);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t) -14415);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t) -32561);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)46088);
    assert(p115_lon_GET(pack) == (int32_t)1203235247);
    assert(p115_alt_GET(pack) == (int32_t) -1827078104);
    assert(p115_pitchspeed_GET(pack) == (float) -8.056619E37F);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t) -14535);
    {
        float exemplary[] =  {1.1940948E38F, 1.4256272E37F, -1.5610965E38F, -1.6565432E38F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_lat_GET(pack) == (int32_t)1844279122);
};


void c_CommunicationChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t) -7191);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)3937980805L);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t)31448);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t)28886);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t)31840);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t)17132);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t) -1406);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t)12729);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t)11941);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t)10171);
};


void c_CommunicationChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)32321);
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)13291);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)160);
};


void c_CommunicationChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_size_GET(pack) == (uint32_t)3653099038L);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)50304);
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)36076);
    assert(p118_time_utc_GET(pack) == (uint32_t)1912975291L);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)34169);
};


void c_CommunicationChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)144);
    assert(p119_count_GET(pack) == (uint32_t)411844098L);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)2669);
    assert(p119_ofs_GET(pack) == (uint32_t)2760488690L);
};


void c_CommunicationChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_ofs_GET(pack) == (uint32_t)2320009828L);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)37970);
    {
        uint8_t exemplary[] =  {(uint8_t)76, (uint8_t)41, (uint8_t)139, (uint8_t)224, (uint8_t)103, (uint8_t)5, (uint8_t)197, (uint8_t)74, (uint8_t)197, (uint8_t)28, (uint8_t)239, (uint8_t)187, (uint8_t)183, (uint8_t)224, (uint8_t)115, (uint8_t)94, (uint8_t)179, (uint8_t)24, (uint8_t)15, (uint8_t)123, (uint8_t)69, (uint8_t)178, (uint8_t)147, (uint8_t)31, (uint8_t)40, (uint8_t)113, (uint8_t)154, (uint8_t)164, (uint8_t)127, (uint8_t)251, (uint8_t)148, (uint8_t)246, (uint8_t)83, (uint8_t)174, (uint8_t)93, (uint8_t)65, (uint8_t)25, (uint8_t)39, (uint8_t)153, (uint8_t)255, (uint8_t)141, (uint8_t)255, (uint8_t)199, (uint8_t)244, (uint8_t)249, (uint8_t)62, (uint8_t)178, (uint8_t)220, (uint8_t)128, (uint8_t)25, (uint8_t)120, (uint8_t)235, (uint8_t)163, (uint8_t)45, (uint8_t)80, (uint8_t)249, (uint8_t)52, (uint8_t)6, (uint8_t)168, (uint8_t)155, (uint8_t)167, (uint8_t)20, (uint8_t)92, (uint8_t)137, (uint8_t)192, (uint8_t)151, (uint8_t)144, (uint8_t)73, (uint8_t)151, (uint8_t)232, (uint8_t)169, (uint8_t)60, (uint8_t)139, (uint8_t)194, (uint8_t)109, (uint8_t)218, (uint8_t)60, (uint8_t)111, (uint8_t)253, (uint8_t)168, (uint8_t)239, (uint8_t)162, (uint8_t)24, (uint8_t)5, (uint8_t)238, (uint8_t)8, (uint8_t)182, (uint8_t)221, (uint8_t)138, (uint8_t)158} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)45);
};


void c_CommunicationChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)220);
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)4);
};


void c_CommunicationChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)41);
};


void c_CommunicationChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)111, (uint8_t)132, (uint8_t)162, (uint8_t)56, (uint8_t)136, (uint8_t)238, (uint8_t)137, (uint8_t)247, (uint8_t)154, (uint8_t)6, (uint8_t)215, (uint8_t)12, (uint8_t)30, (uint8_t)171, (uint8_t)88, (uint8_t)60, (uint8_t)134, (uint8_t)219, (uint8_t)17, (uint8_t)0, (uint8_t)179, (uint8_t)127, (uint8_t)33, (uint8_t)129, (uint8_t)132, (uint8_t)210, (uint8_t)26, (uint8_t)199, (uint8_t)6, (uint8_t)242, (uint8_t)135, (uint8_t)125, (uint8_t)183, (uint8_t)123, (uint8_t)81, (uint8_t)72, (uint8_t)246, (uint8_t)237, (uint8_t)196, (uint8_t)107, (uint8_t)244, (uint8_t)72, (uint8_t)133, (uint8_t)188, (uint8_t)123, (uint8_t)39, (uint8_t)240, (uint8_t)159, (uint8_t)215, (uint8_t)122, (uint8_t)124, (uint8_t)156, (uint8_t)149, (uint8_t)171, (uint8_t)150, (uint8_t)71, (uint8_t)50, (uint8_t)246, (uint8_t)241, (uint8_t)115, (uint8_t)126, (uint8_t)71, (uint8_t)81, (uint8_t)104, (uint8_t)71, (uint8_t)209, (uint8_t)197, (uint8_t)106, (uint8_t)240, (uint8_t)3, (uint8_t)157, (uint8_t)220, (uint8_t)64, (uint8_t)184, (uint8_t)133, (uint8_t)62, (uint8_t)42, (uint8_t)203, (uint8_t)7, (uint8_t)154, (uint8_t)174, (uint8_t)36, (uint8_t)190, (uint8_t)123, (uint8_t)40, (uint8_t)180, (uint8_t)175, (uint8_t)173, (uint8_t)7, (uint8_t)19, (uint8_t)211, (uint8_t)69, (uint8_t)103, (uint8_t)41, (uint8_t)120, (uint8_t)117, (uint8_t)171, (uint8_t)22, (uint8_t)213, (uint8_t)228, (uint8_t)21, (uint8_t)7, (uint8_t)20, (uint8_t)30, (uint8_t)26, (uint8_t)74, (uint8_t)224, (uint8_t)246, (uint8_t)191, (uint8_t)98} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)57);
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)147);
};


void c_CommunicationChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_lat_GET(pack) == (int32_t)1100312878);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)24757);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT);
    assert(p124_lon_GET(pack) == (int32_t)725718205);
    assert(p124_time_usec_GET(pack) == (uint64_t)4947885165335678590L);
    assert(p124_alt_GET(pack) == (int32_t)1930187677);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)50742);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)172);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)37953);
    assert(p124_dgps_age_GET(pack) == (uint32_t)2543482038L);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)22895);
};


void c_CommunicationChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)40050);
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)59699);
    assert(p125_flags_GET(pack) == e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID);
};


void c_CommunicationChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)185);
    {
        uint8_t exemplary[] =  {(uint8_t)30, (uint8_t)70, (uint8_t)228, (uint8_t)9, (uint8_t)212, (uint8_t)208, (uint8_t)160, (uint8_t)237, (uint8_t)144, (uint8_t)58, (uint8_t)191, (uint8_t)164, (uint8_t)212, (uint8_t)176, (uint8_t)74, (uint8_t)80, (uint8_t)46, (uint8_t)96, (uint8_t)48, (uint8_t)162, (uint8_t)191, (uint8_t)242, (uint8_t)69, (uint8_t)9, (uint8_t)231, (uint8_t)60, (uint8_t)172, (uint8_t)176, (uint8_t)186, (uint8_t)202, (uint8_t)139, (uint8_t)231, (uint8_t)143, (uint8_t)255, (uint8_t)59, (uint8_t)217, (uint8_t)147, (uint8_t)232, (uint8_t)237, (uint8_t)192, (uint8_t)173, (uint8_t)131, (uint8_t)240, (uint8_t)243, (uint8_t)246, (uint8_t)9, (uint8_t)15, (uint8_t)20, (uint8_t)86, (uint8_t)194, (uint8_t)212, (uint8_t)37, (uint8_t)229, (uint8_t)202, (uint8_t)247, (uint8_t)220, (uint8_t)225, (uint8_t)18, (uint8_t)31, (uint8_t)154, (uint8_t)236, (uint8_t)200, (uint8_t)160, (uint8_t)42, (uint8_t)79, (uint8_t)6, (uint8_t)194, (uint8_t)157, (uint8_t)132, (uint8_t)174} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_baudrate_GET(pack) == (uint32_t)3933747905L);
    assert(p126_flags_GET(pack) == e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE);
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)13211);
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1);
};


void c_CommunicationChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_baseline_c_mm_GET(pack) == (int32_t)1510465751);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t) -1092074213);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p127_accuracy_GET(pack) == (uint32_t)3255365147L);
    assert(p127_tow_GET(pack) == (uint32_t)2363528971L);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)134);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)1006098950L);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t)1244810690);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)12141);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t) -1591778888);
};


void c_CommunicationChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t) -2053284568);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)710285677L);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)167);
    assert(p128_accuracy_GET(pack) == (uint32_t)2049169745L);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t)991984414);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t)2080907122);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -1690391046);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p128_tow_GET(pack) == (uint32_t)1649351279L);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)8130);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)177);
};


void c_CommunicationChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t) -3048);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t)4905);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t)2178);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t) -24696);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t)10568);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)754529265L);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t)4006);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)25623);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t)22430);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t)17746);
};


void c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)65193);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p130_size_GET(pack) == (uint32_t)3070567859L);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)49468);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)204);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)38528);
};


void c_CommunicationChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)36, (uint8_t)209, (uint8_t)163, (uint8_t)74, (uint8_t)242, (uint8_t)89, (uint8_t)65, (uint8_t)91, (uint8_t)192, (uint8_t)182, (uint8_t)229, (uint8_t)241, (uint8_t)233, (uint8_t)197, (uint8_t)237, (uint8_t)17, (uint8_t)139, (uint8_t)169, (uint8_t)71, (uint8_t)228, (uint8_t)159, (uint8_t)72, (uint8_t)80, (uint8_t)225, (uint8_t)140, (uint8_t)189, (uint8_t)167, (uint8_t)160, (uint8_t)73, (uint8_t)74, (uint8_t)66, (uint8_t)31, (uint8_t)183, (uint8_t)244, (uint8_t)149, (uint8_t)17, (uint8_t)182, (uint8_t)177, (uint8_t)196, (uint8_t)91, (uint8_t)125, (uint8_t)231, (uint8_t)160, (uint8_t)227, (uint8_t)118, (uint8_t)46, (uint8_t)47, (uint8_t)167, (uint8_t)194, (uint8_t)255, (uint8_t)232, (uint8_t)55, (uint8_t)155, (uint8_t)101, (uint8_t)25, (uint8_t)139, (uint8_t)97, (uint8_t)35, (uint8_t)118, (uint8_t)110, (uint8_t)116, (uint8_t)163, (uint8_t)211, (uint8_t)32, (uint8_t)41, (uint8_t)188, (uint8_t)32, (uint8_t)197, (uint8_t)34, (uint8_t)213, (uint8_t)1, (uint8_t)175, (uint8_t)237, (uint8_t)110, (uint8_t)194, (uint8_t)229, (uint8_t)63, (uint8_t)97, (uint8_t)40, (uint8_t)136, (uint8_t)86, (uint8_t)121, (uint8_t)113, (uint8_t)200, (uint8_t)0, (uint8_t)146, (uint8_t)142, (uint8_t)183, (uint8_t)19, (uint8_t)250, (uint8_t)19, (uint8_t)210, (uint8_t)5, (uint8_t)91, (uint8_t)175, (uint8_t)193, (uint8_t)218, (uint8_t)92, (uint8_t)23, (uint8_t)111, (uint8_t)63, (uint8_t)169, (uint8_t)244, (uint8_t)15, (uint8_t)184, (uint8_t)238, (uint8_t)182, (uint8_t)170, (uint8_t)9, (uint8_t)86, (uint8_t)152, (uint8_t)93, (uint8_t)65, (uint8_t)242, (uint8_t)224, (uint8_t)239, (uint8_t)130, (uint8_t)27, (uint8_t)201, (uint8_t)27, (uint8_t)232, (uint8_t)218, (uint8_t)129, (uint8_t)77, (uint8_t)169, (uint8_t)119, (uint8_t)136, (uint8_t)61, (uint8_t)188, (uint8_t)102, (uint8_t)135, (uint8_t)129, (uint8_t)198, (uint8_t)22, (uint8_t)212, (uint8_t)0, (uint8_t)110, (uint8_t)239, (uint8_t)155, (uint8_t)71, (uint8_t)23, (uint8_t)1, (uint8_t)166, (uint8_t)24, (uint8_t)119, (uint8_t)212, (uint8_t)181, (uint8_t)228, (uint8_t)10, (uint8_t)105, (uint8_t)153, (uint8_t)198, (uint8_t)212, (uint8_t)80, (uint8_t)0, (uint8_t)162, (uint8_t)183, (uint8_t)231, (uint8_t)209, (uint8_t)185, (uint8_t)43, (uint8_t)155, (uint8_t)254, (uint8_t)102, (uint8_t)13, (uint8_t)102, (uint8_t)250, (uint8_t)237, (uint8_t)75, (uint8_t)242, (uint8_t)95, (uint8_t)250, (uint8_t)162, (uint8_t)168, (uint8_t)175, (uint8_t)6, (uint8_t)238, (uint8_t)78, (uint8_t)38, (uint8_t)187, (uint8_t)21, (uint8_t)229, (uint8_t)43, (uint8_t)165, (uint8_t)145, (uint8_t)146, (uint8_t)9, (uint8_t)17, (uint8_t)58, (uint8_t)152, (uint8_t)75, (uint8_t)202, (uint8_t)217, (uint8_t)123, (uint8_t)159, (uint8_t)240, (uint8_t)110, (uint8_t)237, (uint8_t)188, (uint8_t)136, (uint8_t)172, (uint8_t)234, (uint8_t)32, (uint8_t)188, (uint8_t)190, (uint8_t)69, (uint8_t)144, (uint8_t)212, (uint8_t)152, (uint8_t)97, (uint8_t)24, (uint8_t)73, (uint8_t)65, (uint8_t)152, (uint8_t)124, (uint8_t)217, (uint8_t)171, (uint8_t)19, (uint8_t)240, (uint8_t)248, (uint8_t)73, (uint8_t)117, (uint8_t)23, (uint8_t)233, (uint8_t)238, (uint8_t)27, (uint8_t)133, (uint8_t)164, (uint8_t)197, (uint8_t)45, (uint8_t)37, (uint8_t)182, (uint8_t)185, (uint8_t)157, (uint8_t)47, (uint8_t)17, (uint8_t)54, (uint8_t)35, (uint8_t)243, (uint8_t)18, (uint8_t)161, (uint8_t)176, (uint8_t)146, (uint8_t)119, (uint8_t)191, (uint8_t)132, (uint8_t)254, (uint8_t)251, (uint8_t)56, (uint8_t)21, (uint8_t)189, (uint8_t)206, (uint8_t)34} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)31268);
};


void c_CommunicationChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)115);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)11615);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)5559);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)3226853906L);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)56555);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_PITCH_270);
};


void c_CommunicationChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)44027);
    assert(p133_lon_GET(pack) == (int32_t)1752231476);
    assert(p133_lat_GET(pack) == (int32_t) -428463151);
    assert(p133_mask_GET(pack) == (uint64_t)4011220607899331739L);
};


void c_CommunicationChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    {
        int16_t exemplary[] =  {(int16_t)9795, (int16_t) -30248, (int16_t) -26482, (int16_t)8290, (int16_t) -7581, (int16_t)8004, (int16_t)27971, (int16_t) -24591, (int16_t) -22073, (int16_t) -25459, (int16_t)27073, (int16_t)23779, (int16_t)29598, (int16_t)3495, (int16_t)6136, (int16_t)16923} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)42959);
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)3);
    assert(p134_lat_GET(pack) == (int32_t)341130128);
    assert(p134_lon_GET(pack) == (int32_t)1005565218);
};


void c_CommunicationChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lat_GET(pack) == (int32_t)853036083);
    assert(p135_lon_GET(pack) == (int32_t) -1036666439);
};


void c_CommunicationChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_lat_GET(pack) == (int32_t)1881058133);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)16538);
    assert(p136_terrain_height_GET(pack) == (float)3.5356735E37F);
    assert(p136_lon_GET(pack) == (int32_t)910160607);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)3936);
    assert(p136_current_height_GET(pack) == (float) -1.9331692E38F);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)19064);
};


void c_CommunicationChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_press_abs_GET(pack) == (float) -1.4614652E38F);
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)3786747938L);
    assert(p137_press_diff_GET(pack) == (float)3.2573836E38F);
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t) -6902);
};


void c_CommunicationChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    assert(p138_time_usec_GET(pack) == (uint64_t)5675129796668863682L);
    assert(p138_x_GET(pack) == (float)2.6010629E38F);
    assert(p138_y_GET(pack) == (float) -3.1576596E37F);
    {
        float exemplary[] =  {-7.11112E37F, -3.462129E37F, 2.66039E38F, -1.2464332E37F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_z_GET(pack) == (float) -6.9965475E37F);
};


void c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)43);
    {
        float exemplary[] =  {-1.9381364E38F, 1.1734986E38F, 3.0235964E38F, -2.8284563E38F, 6.218146E37F, 8.898832E37F, 2.683303E37F, 9.552813E37F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)227);
    assert(p139_time_usec_GET(pack) == (uint64_t)4604093745287409442L);
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)4);
};


void c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-1.802765E38F, -2.134553E38F, 1.5008257E38F, 1.0754278E38F, -1.1695672E38F, 3.3259298E38F, 8.938811E37F, -1.936508E38F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p140_time_usec_GET(pack) == (uint64_t)1919067092603424797L);
};


void c_CommunicationChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_altitude_monotonic_GET(pack) == (float) -6.0291496E37F);
    assert(p141_altitude_relative_GET(pack) == (float) -9.653118E37F);
    assert(p141_bottom_clearance_GET(pack) == (float) -1.9085622E38F);
    assert(p141_altitude_amsl_GET(pack) == (float) -3.3964455E38F);
    assert(p141_altitude_local_GET(pack) == (float)2.3903715E37F);
    assert(p141_time_usec_GET(pack) == (uint64_t)5589314639587325708L);
    assert(p141_altitude_terrain_GET(pack) == (float) -1.6526165E38F);
};


void c_CommunicationChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)72, (uint8_t)33, (uint8_t)247, (uint8_t)157, (uint8_t)253, (uint8_t)9, (uint8_t)84, (uint8_t)210, (uint8_t)149, (uint8_t)101, (uint8_t)45, (uint8_t)214, (uint8_t)99, (uint8_t)112, (uint8_t)138, (uint8_t)87, (uint8_t)227, (uint8_t)127, (uint8_t)55, (uint8_t)155, (uint8_t)41, (uint8_t)41, (uint8_t)255, (uint8_t)128, (uint8_t)119, (uint8_t)143, (uint8_t)106, (uint8_t)172, (uint8_t)67, (uint8_t)148, (uint8_t)223, (uint8_t)191, (uint8_t)241, (uint8_t)216, (uint8_t)191, (uint8_t)30, (uint8_t)226, (uint8_t)6, (uint8_t)35, (uint8_t)165, (uint8_t)47, (uint8_t)106, (uint8_t)19, (uint8_t)98, (uint8_t)203, (uint8_t)86, (uint8_t)229, (uint8_t)216, (uint8_t)223, (uint8_t)222, (uint8_t)200, (uint8_t)97, (uint8_t)137, (uint8_t)184, (uint8_t)55, (uint8_t)58, (uint8_t)210, (uint8_t)15, (uint8_t)217, (uint8_t)233, (uint8_t)65, (uint8_t)21, (uint8_t)15, (uint8_t)29, (uint8_t)31, (uint8_t)21, (uint8_t)152, (uint8_t)97, (uint8_t)209, (uint8_t)243, (uint8_t)227, (uint8_t)185, (uint8_t)242, (uint8_t)110, (uint8_t)242, (uint8_t)118, (uint8_t)196, (uint8_t)251, (uint8_t)96, (uint8_t)54, (uint8_t)142, (uint8_t)41, (uint8_t)61, (uint8_t)30, (uint8_t)7, (uint8_t)28, (uint8_t)164, (uint8_t)14, (uint8_t)120, (uint8_t)135, (uint8_t)27, (uint8_t)191, (uint8_t)41, (uint8_t)124, (uint8_t)26, (uint8_t)182, (uint8_t)253, (uint8_t)233, (uint8_t)128, (uint8_t)185, (uint8_t)193, (uint8_t)30, (uint8_t)214, (uint8_t)246, (uint8_t)37, (uint8_t)57, (uint8_t)220, (uint8_t)47, (uint8_t)219, (uint8_t)121, (uint8_t)13, (uint8_t)151, (uint8_t)84, (uint8_t)90, (uint8_t)97, (uint8_t)39, (uint8_t)142, (uint8_t)128, (uint8_t)105, (uint8_t)162} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)222);
    {
        uint8_t exemplary[] =  {(uint8_t)68, (uint8_t)27, (uint8_t)112, (uint8_t)141, (uint8_t)29, (uint8_t)63, (uint8_t)228, (uint8_t)217, (uint8_t)140, (uint8_t)128, (uint8_t)56, (uint8_t)186, (uint8_t)230, (uint8_t)243, (uint8_t)2, (uint8_t)44, (uint8_t)249, (uint8_t)120, (uint8_t)159, (uint8_t)103, (uint8_t)134, (uint8_t)15, (uint8_t)234, (uint8_t)20, (uint8_t)24, (uint8_t)239, (uint8_t)201, (uint8_t)110, (uint8_t)14, (uint8_t)42, (uint8_t)106, (uint8_t)65, (uint8_t)2, (uint8_t)157, (uint8_t)134, (uint8_t)80, (uint8_t)99, (uint8_t)7, (uint8_t)176, (uint8_t)102, (uint8_t)164, (uint8_t)204, (uint8_t)114, (uint8_t)132, (uint8_t)153, (uint8_t)255, (uint8_t)34, (uint8_t)130, (uint8_t)188, (uint8_t)251, (uint8_t)15, (uint8_t)185, (uint8_t)67, (uint8_t)235, (uint8_t)206, (uint8_t)48, (uint8_t)97, (uint8_t)32, (uint8_t)109, (uint8_t)220, (uint8_t)255, (uint8_t)234, (uint8_t)101, (uint8_t)157, (uint8_t)110, (uint8_t)210, (uint8_t)185, (uint8_t)13, (uint8_t)236, (uint8_t)73, (uint8_t)142, (uint8_t)75, (uint8_t)110, (uint8_t)27, (uint8_t)88, (uint8_t)243, (uint8_t)26, (uint8_t)124, (uint8_t)83, (uint8_t)14, (uint8_t)184, (uint8_t)75, (uint8_t)233, (uint8_t)80, (uint8_t)88, (uint8_t)86, (uint8_t)76, (uint8_t)218, (uint8_t)51, (uint8_t)115, (uint8_t)28, (uint8_t)136, (uint8_t)89, (uint8_t)36, (uint8_t)74, (uint8_t)39, (uint8_t)179, (uint8_t)109, (uint8_t)185, (uint8_t)10, (uint8_t)51, (uint8_t)115, (uint8_t)84, (uint8_t)24, (uint8_t)119, (uint8_t)154, (uint8_t)206, (uint8_t)52, (uint8_t)16, (uint8_t)25, (uint8_t)181, (uint8_t)71, (uint8_t)207, (uint8_t)79, (uint8_t)84, (uint8_t)241, (uint8_t)108, (uint8_t)65, (uint8_t)133, (uint8_t)65} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)101);
};


void c_CommunicationChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t) -15956);
    assert(p143_press_diff_GET(pack) == (float)8.69019E37F);
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)3281673497L);
    assert(p143_press_abs_GET(pack) == (float)1.9353986E38F);
};


void c_CommunicationChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    assert(p144_timestamp_GET(pack) == (uint64_t)8576132673122251980L);
    {
        float exemplary[] =  {-2.917955E38F, -1.922365E38F, 2.893489E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-2.196119E38F, -2.4075096E38F, -3.1900952E37F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_custom_state_GET(pack) == (uint64_t)7383851715539542503L);
    assert(p144_alt_GET(pack) == (float) -1.8723309E38F);
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)67);
    {
        float exemplary[] =  {-2.8366473E38F, 6.8715054E37F, 7.979309E37F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-2.9177646E38F, -1.1377091E38F, 1.0073689E38F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lat_GET(pack) == (int32_t)2007215741);
    {
        float exemplary[] =  {-7.5925963E37F, -1.6584429E36F, 1.1097432E38F, 2.7858218E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lon_GET(pack) == (int32_t) -1559690753);
};


void c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_x_acc_GET(pack) == (float)1.402545E38F);
    assert(p146_x_pos_GET(pack) == (float)4.6894564E37F);
    assert(p146_y_pos_GET(pack) == (float)1.7460863E38F);
    assert(p146_time_usec_GET(pack) == (uint64_t)5714096563057014722L);
    assert(p146_x_vel_GET(pack) == (float)2.4062012E38F);
    assert(p146_pitch_rate_GET(pack) == (float) -1.2987006E38F);
    {
        float exemplary[] =  {9.801902E37F, 1.3628309E38F, 1.3779298E38F, 1.4211097E38F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-1.4510498E38F, 3.3597102E38F, -8.3140655E37F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_yaw_rate_GET(pack) == (float)1.1404376E38F);
    assert(p146_z_acc_GET(pack) == (float)4.7014898E36F);
    assert(p146_roll_rate_GET(pack) == (float) -1.727649E38F);
    assert(p146_y_acc_GET(pack) == (float) -4.33816E37F);
    assert(p146_airspeed_GET(pack) == (float) -3.2526973E38F);
    assert(p146_z_pos_GET(pack) == (float) -1.0180164E38F);
    assert(p146_y_vel_GET(pack) == (float) -4.0349073E37F);
    assert(p146_z_vel_GET(pack) == (float)4.027063E35F);
    {
        float exemplary[] =  {-2.2624754E38F, 1.3501348E38F, -1.7365069E38F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)235);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t)34);
    assert(p147_energy_consumed_GET(pack) == (int32_t)1171068235);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_NIMH);
    {
        uint16_t exemplary[] =  {(uint16_t)26262, (uint16_t)42414, (uint16_t)1470, (uint16_t)37269, (uint16_t)14944, (uint16_t)30427, (uint16_t)32132, (uint16_t)44142, (uint16_t)9703, (uint16_t)55967} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_AVIONICS);
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t)10893);
    assert(p147_current_consumed_GET(pack) == (int32_t)1627149141);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t) -29449);
};


void c_CommunicationChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_capabilities_GET(pack) == e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMMAND_INT);
    {
        uint8_t exemplary[] =  {(uint8_t)210, (uint8_t)254, (uint8_t)167, (uint8_t)169, (uint8_t)10, (uint8_t)239, (uint8_t)190, (uint8_t)4} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)17176);
    assert(p148_uid_GET(pack) == (uint64_t)419737349904552244L);
    assert(p148_board_version_GET(pack) == (uint32_t)2156716582L);
    assert(p148_os_sw_version_GET(pack) == (uint32_t)999026390L);
    {
        uint8_t exemplary[] =  {(uint8_t)82, (uint8_t)150, (uint8_t)222, (uint8_t)169, (uint8_t)33, (uint8_t)82, (uint8_t)26, (uint8_t)140} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)1244599946L);
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)2036651191L);
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)18578);
    {
        uint8_t exemplary[] =  {(uint8_t)86, (uint8_t)108, (uint8_t)85, (uint8_t)134, (uint8_t)17, (uint8_t)176, (uint8_t)145, (uint8_t)243, (uint8_t)67, (uint8_t)176, (uint8_t)251, (uint8_t)39, (uint8_t)147, (uint8_t)71, (uint8_t)19, (uint8_t)139, (uint8_t)22, (uint8_t)19} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)231, (uint8_t)40, (uint8_t)197, (uint8_t)223, (uint8_t)172, (uint8_t)218, (uint8_t)187, (uint8_t)172} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)61);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)154);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON);
    assert(p149_y_TRY(ph) == (float)1.838419E38F);
    assert(p149_x_TRY(ph) == (float) -2.81798E38F);
    assert(p149_angle_x_GET(pack) == (float)2.3728807E38F);
    assert(p149_distance_GET(pack) == (float)1.1510844E38F);
    assert(p149_time_usec_GET(pack) == (uint64_t)8955228839679415102L);
    assert(p149_size_x_GET(pack) == (float)1.3411605E37F);
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p149_size_y_GET(pack) == (float) -1.7657908E38F);
    {
        float exemplary[] =  {-7.116993E37F, -3.1898282E37F, 1.2596789E38F, -8.2611953E37F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_angle_y_GET(pack) == (float) -1.023486E38F);
    assert(p149_z_TRY(ph) == (float)2.9882285E38F);
};


void c_CommunicationChannel_on_CPU_LOAD_170(Bounds_Inside * ph, Pack * pack)
{
    assert(p170_batVolt_GET(pack) == (uint16_t)(uint16_t)55805);
    assert(p170_sensLoad_GET(pack) == (uint8_t)(uint8_t)50);
    assert(p170_ctrlLoad_GET(pack) == (uint8_t)(uint8_t)127);
};


void c_CommunicationChannel_on_SENSOR_BIAS_172(Bounds_Inside * ph, Pack * pack)
{
    assert(p172_axBias_GET(pack) == (float) -3.0555962E37F);
    assert(p172_gyBias_GET(pack) == (float)1.7848987E37F);
    assert(p172_gzBias_GET(pack) == (float)2.8582787E38F);
    assert(p172_ayBias_GET(pack) == (float) -1.5665718E38F);
    assert(p172_azBias_GET(pack) == (float) -2.4359853E38F);
    assert(p172_gxBias_GET(pack) == (float) -7.0239754E37F);
};


void c_CommunicationChannel_on_DIAGNOSTIC_173(Bounds_Inside * ph, Pack * pack)
{
    assert(p173_diagSh2_GET(pack) == (int16_t)(int16_t)17434);
    assert(p173_diagSh1_GET(pack) == (int16_t)(int16_t)11008);
    assert(p173_diagFl3_GET(pack) == (float) -2.7263477E38F);
    assert(p173_diagFl1_GET(pack) == (float)2.9546222E38F);
    assert(p173_diagSh3_GET(pack) == (int16_t)(int16_t)11064);
    assert(p173_diagFl2_GET(pack) == (float)1.0211169E38F);
};


void c_CommunicationChannel_on_SLUGS_NAVIGATION_176(Bounds_Inside * ph, Pack * pack)
{
    assert(p176_fromWP_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p176_totalDist_GET(pack) == (float) -8.187116E37F);
    assert(p176_u_m_GET(pack) == (float)3.3250893E38F);
    assert(p176_toWP_GET(pack) == (uint8_t)(uint8_t)134);
    assert(p176_ay_body_GET(pack) == (float)1.3837325E38F);
    assert(p176_phi_c_GET(pack) == (float)6.130505E37F);
    assert(p176_dist2Go_GET(pack) == (float) -1.985682E38F);
    assert(p176_theta_c_GET(pack) == (float) -9.433098E37F);
    assert(p176_psiDot_c_GET(pack) == (float) -2.0685062E38F);
    assert(p176_h_c_GET(pack) == (uint16_t)(uint16_t)21183);
};


void c_CommunicationChannel_on_DATA_LOG_177(Bounds_Inside * ph, Pack * pack)
{
    assert(p177_fl_5_GET(pack) == (float)3.01179E38F);
    assert(p177_fl_6_GET(pack) == (float)2.0990894E38F);
    assert(p177_fl_3_GET(pack) == (float)6.603759E37F);
    assert(p177_fl_4_GET(pack) == (float) -3.6326934E37F);
    assert(p177_fl_2_GET(pack) == (float) -2.9747788E38F);
    assert(p177_fl_1_GET(pack) == (float)1.4793822E37F);
};


void c_CommunicationChannel_on_GPS_DATE_TIME_179(Bounds_Inside * ph, Pack * pack)
{
    assert(p179_year_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p179_month_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p179_useSat_GET(pack) == (uint8_t)(uint8_t)12);
    assert(p179_GppGl_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p179_min_GET(pack) == (uint8_t)(uint8_t)221);
    assert(p179_visSat_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p179_sigUsedMask_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p179_percentUsed_GET(pack) == (uint8_t)(uint8_t)204);
    assert(p179_day_GET(pack) == (uint8_t)(uint8_t)21);
    assert(p179_sec_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p179_hour_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p179_clockStat_GET(pack) == (uint8_t)(uint8_t)227);
};


void c_CommunicationChannel_on_MID_LVL_CMDS_180(Bounds_Inside * ph, Pack * pack)
{
    assert(p180_uCommand_GET(pack) == (float)3.2971724E38F);
    assert(p180_hCommand_GET(pack) == (float) -2.2030191E38F);
    assert(p180_rCommand_GET(pack) == (float) -1.6351938E38F);
    assert(p180_target_GET(pack) == (uint8_t)(uint8_t)83);
};


void c_CommunicationChannel_on_CTRL_SRFC_PT_181(Bounds_Inside * ph, Pack * pack)
{
    assert(p181_bitfieldPt_GET(pack) == (uint16_t)(uint16_t)11598);
    assert(p181_target_GET(pack) == (uint8_t)(uint8_t)193);
};


void c_CommunicationChannel_on_SLUGS_CAMERA_ORDER_184(Bounds_Inside * ph, Pack * pack)
{
    assert(p184_pan_GET(pack) == (int8_t)(int8_t) -117);
    assert(p184_moveHome_GET(pack) == (int8_t)(int8_t)59);
    assert(p184_target_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p184_tilt_GET(pack) == (int8_t)(int8_t) -46);
    assert(p184_zoom_GET(pack) == (int8_t)(int8_t)116);
};


void c_CommunicationChannel_on_CONTROL_SURFACE_185(Bounds_Inside * ph, Pack * pack)
{
    assert(p185_bControl_GET(pack) == (float) -1.1569067E38F);
    assert(p185_target_GET(pack) == (uint8_t)(uint8_t)232);
    assert(p185_mControl_GET(pack) == (float) -2.5702912E38F);
    assert(p185_idSurface_GET(pack) == (uint8_t)(uint8_t)92);
};


void c_CommunicationChannel_on_SLUGS_MOBILE_LOCATION_186(Bounds_Inside * ph, Pack * pack)
{
    assert(p186_target_GET(pack) == (uint8_t)(uint8_t)91);
    assert(p186_latitude_GET(pack) == (float)1.9845827E38F);
    assert(p186_longitude_GET(pack) == (float)2.88424E38F);
};


void c_CommunicationChannel_on_SLUGS_CONFIGURATION_CAMERA_188(Bounds_Inside * ph, Pack * pack)
{
    assert(p188_order_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p188_idOrder_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p188_target_GET(pack) == (uint8_t)(uint8_t)241);
};


void c_CommunicationChannel_on_ISR_LOCATION_189(Bounds_Inside * ph, Pack * pack)
{
    assert(p189_option3_GET(pack) == (uint8_t)(uint8_t)65);
    assert(p189_height_GET(pack) == (float) -1.614351E38F);
    assert(p189_option2_GET(pack) == (uint8_t)(uint8_t)124);
    assert(p189_target_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p189_option1_GET(pack) == (uint8_t)(uint8_t)90);
    assert(p189_longitude_GET(pack) == (float) -6.488452E37F);
    assert(p189_latitude_GET(pack) == (float) -2.1722061E38F);
};


void c_CommunicationChannel_on_VOLT_SENSOR_191(Bounds_Inside * ph, Pack * pack)
{
    assert(p191_reading2_GET(pack) == (uint16_t)(uint16_t)55310);
    assert(p191_voltage_GET(pack) == (uint16_t)(uint16_t)31618);
    assert(p191_r2Type_GET(pack) == (uint8_t)(uint8_t)192);
};


void c_CommunicationChannel_on_PTZ_STATUS_192(Bounds_Inside * ph, Pack * pack)
{
    assert(p192_zoom_GET(pack) == (uint8_t)(uint8_t)232);
    assert(p192_pan_GET(pack) == (int16_t)(int16_t)5611);
    assert(p192_tilt_GET(pack) == (int16_t)(int16_t) -210);
};


void c_CommunicationChannel_on_UAV_STATUS_193(Bounds_Inside * ph, Pack * pack)
{
    assert(p193_target_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p193_course_GET(pack) == (float) -2.7996264E38F);
    assert(p193_speed_GET(pack) == (float)7.871292E37F);
    assert(p193_longitude_GET(pack) == (float)2.8295134E38F);
    assert(p193_altitude_GET(pack) == (float)2.1865462E37F);
    assert(p193_latitude_GET(pack) == (float)1.6850917E38F);
};


void c_CommunicationChannel_on_STATUS_GPS_194(Bounds_Inside * ph, Pack * pack)
{
    assert(p194_magDir_GET(pack) == (int8_t)(int8_t)49);
    assert(p194_magVar_GET(pack) == (float)1.3370784E38F);
    assert(p194_csFails_GET(pack) == (uint16_t)(uint16_t)35257);
    assert(p194_msgsType_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p194_modeInd_GET(pack) == (uint8_t)(uint8_t)11);
    assert(p194_gpsQuality_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p194_posStatus_GET(pack) == (uint8_t)(uint8_t)81);
};


void c_CommunicationChannel_on_NOVATEL_DIAG_195(Bounds_Inside * ph, Pack * pack)
{
    assert(p195_solStatus_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p195_timeStatus_GET(pack) == (uint8_t)(uint8_t)224);
    assert(p195_posSolAge_GET(pack) == (float) -3.3470906E38F);
    assert(p195_velType_GET(pack) == (uint8_t)(uint8_t)189);
    assert(p195_receiverStatus_GET(pack) == (uint32_t)3950238385L);
    assert(p195_posType_GET(pack) == (uint8_t)(uint8_t)157);
    assert(p195_csFails_GET(pack) == (uint16_t)(uint16_t)48948);
};


void c_CommunicationChannel_on_SENSOR_DIAG_196(Bounds_Inside * ph, Pack * pack)
{
    assert(p196_int1_GET(pack) == (int16_t)(int16_t)9716);
    assert(p196_float2_GET(pack) == (float) -1.826724E38F);
    assert(p196_float1_GET(pack) == (float) -2.521057E38F);
    assert(p196_char1_GET(pack) == (int8_t)(int8_t) -114);
};


void c_CommunicationChannel_on_BOOT_197(Bounds_Inside * ph, Pack * pack)
{
    assert(p197_version_GET(pack) == (uint32_t)1924896412L);
};


void c_CommunicationChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_tas_ratio_GET(pack) == (float)2.6487085E38F);
    assert(p230_hagl_ratio_GET(pack) == (float) -7.068972E37F);
    assert(p230_vel_ratio_GET(pack) == (float) -2.6040074E37F);
    assert(p230_flags_GET(pack) == e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL);
    assert(p230_time_usec_GET(pack) == (uint64_t)9016035822336908008L);
    assert(p230_pos_vert_accuracy_GET(pack) == (float)3.293159E38F);
    assert(p230_mag_ratio_GET(pack) == (float) -1.1688565E37F);
    assert(p230_pos_vert_ratio_GET(pack) == (float) -1.3994759E38F);
    assert(p230_pos_horiz_ratio_GET(pack) == (float)2.1806057E38F);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float)9.397331E37F);
};


void c_CommunicationChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_var_vert_GET(pack) == (float) -1.4892463E38F);
    assert(p231_horiz_accuracy_GET(pack) == (float) -5.756607E37F);
    assert(p231_wind_z_GET(pack) == (float)2.645964E38F);
    assert(p231_wind_alt_GET(pack) == (float)1.534516E37F);
    assert(p231_vert_accuracy_GET(pack) == (float)2.2368912E38F);
    assert(p231_time_usec_GET(pack) == (uint64_t)1860902660178408629L);
    assert(p231_wind_x_GET(pack) == (float)1.1410627E38F);
    assert(p231_wind_y_GET(pack) == (float)2.2023145E38F);
    assert(p231_var_horiz_GET(pack) == (float)2.835734E38F);
};


void c_CommunicationChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)36343);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p232_lon_GET(pack) == (int32_t) -293375177);
    assert(p232_lat_GET(pack) == (int32_t) -867140556);
    assert(p232_speed_accuracy_GET(pack) == (float) -7.332238E37F);
    assert(p232_time_usec_GET(pack) == (uint64_t)1842183281547945249L);
    assert(p232_vdop_GET(pack) == (float)8.879837E37F);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)225);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)2098069297L);
    assert(p232_ve_GET(pack) == (float) -1.0462465E37F);
    assert(p232_hdop_GET(pack) == (float) -1.7353015E38F);
    assert(p232_horiz_accuracy_GET(pack) == (float)2.7029637E38F);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)189);
    assert(p232_alt_GET(pack) == (float) -2.0068278E38F);
    assert(p232_vn_GET(pack) == (float)8.91452E37F);
    assert(p232_vd_GET(pack) == (float) -7.3433554E37F);
    assert(p232_ignore_flags_GET(pack) == e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY);
    assert(p232_vert_accuracy_GET(pack) == (float)1.0167076E38F);
};


void c_CommunicationChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)40, (uint8_t)119, (uint8_t)153, (uint8_t)249, (uint8_t)158, (uint8_t)252, (uint8_t)185, (uint8_t)238, (uint8_t)201, (uint8_t)50, (uint8_t)168, (uint8_t)213, (uint8_t)140, (uint8_t)7, (uint8_t)65, (uint8_t)187, (uint8_t)78, (uint8_t)98, (uint8_t)227, (uint8_t)208, (uint8_t)135, (uint8_t)251, (uint8_t)121, (uint8_t)139, (uint8_t)249, (uint8_t)101, (uint8_t)11, (uint8_t)59, (uint8_t)188, (uint8_t)22, (uint8_t)157, (uint8_t)67, (uint8_t)240, (uint8_t)86, (uint8_t)229, (uint8_t)234, (uint8_t)32, (uint8_t)98, (uint8_t)19, (uint8_t)177, (uint8_t)136, (uint8_t)71, (uint8_t)177, (uint8_t)211, (uint8_t)193, (uint8_t)104, (uint8_t)64, (uint8_t)124, (uint8_t)223, (uint8_t)255, (uint8_t)7, (uint8_t)129, (uint8_t)18, (uint8_t)162, (uint8_t)185, (uint8_t)193, (uint8_t)70, (uint8_t)122, (uint8_t)219, (uint8_t)57, (uint8_t)96, (uint8_t)64, (uint8_t)43, (uint8_t)171, (uint8_t)226, (uint8_t)211, (uint8_t)62, (uint8_t)84, (uint8_t)101, (uint8_t)193, (uint8_t)169, (uint8_t)115, (uint8_t)18, (uint8_t)236, (uint8_t)209, (uint8_t)53, (uint8_t)186, (uint8_t)254, (uint8_t)171, (uint8_t)21, (uint8_t)11, (uint8_t)235, (uint8_t)92, (uint8_t)187, (uint8_t)60, (uint8_t)119, (uint8_t)128, (uint8_t)39, (uint8_t)28, (uint8_t)106, (uint8_t)147, (uint8_t)196, (uint8_t)108, (uint8_t)83, (uint8_t)192, (uint8_t)155, (uint8_t)29, (uint8_t)37, (uint8_t)116, (uint8_t)3, (uint8_t)156, (uint8_t)235, (uint8_t)144, (uint8_t)197, (uint8_t)160, (uint8_t)87, (uint8_t)59, (uint8_t)74, (uint8_t)13, (uint8_t)180, (uint8_t)192, (uint8_t)25, (uint8_t)254, (uint8_t)107, (uint8_t)253, (uint8_t)214, (uint8_t)126, (uint8_t)10, (uint8_t)122, (uint8_t)147, (uint8_t)169, (uint8_t)135, (uint8_t)190, (uint8_t)37, (uint8_t)144, (uint8_t)79, (uint8_t)249, (uint8_t)239, (uint8_t)180, (uint8_t)182, (uint8_t)49, (uint8_t)232, (uint8_t)48, (uint8_t)2, (uint8_t)145, (uint8_t)250, (uint8_t)142, (uint8_t)44, (uint8_t)160, (uint8_t)36, (uint8_t)173, (uint8_t)190, (uint8_t)167, (uint8_t)111, (uint8_t)252, (uint8_t)161, (uint8_t)152, (uint8_t)112, (uint8_t)219, (uint8_t)123, (uint8_t)254, (uint8_t)2, (uint8_t)39, (uint8_t)135, (uint8_t)191, (uint8_t)170, (uint8_t)134, (uint8_t)228, (uint8_t)151, (uint8_t)2, (uint8_t)33, (uint8_t)174, (uint8_t)219, (uint8_t)196, (uint8_t)208, (uint8_t)101, (uint8_t)114, (uint8_t)37, (uint8_t)239, (uint8_t)238, (uint8_t)151, (uint8_t)22, (uint8_t)191, (uint8_t)185, (uint8_t)134, (uint8_t)25, (uint8_t)119, (uint8_t)252, (uint8_t)243, (uint8_t)93} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)237);
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)48);
};


void c_CommunicationChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t)21);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)152);
    assert(p234_latitude_GET(pack) == (int32_t)1528444106);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)63951);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t) -19666);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t) -26663);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)57);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)12798);
    assert(p234_custom_mode_GET(pack) == (uint32_t)3860322367L);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t) -19584);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t) -21071);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t)18563);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t) -86);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t) -11);
    assert(p234_longitude_GET(pack) == (int32_t)931826897);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t)38);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)101);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p234_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED);
};


void c_CommunicationChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_clipping_1_GET(pack) == (uint32_t)2781430278L);
    assert(p241_clipping_2_GET(pack) == (uint32_t)1583377295L);
    assert(p241_vibration_z_GET(pack) == (float)2.0423806E37F);
    assert(p241_clipping_0_GET(pack) == (uint32_t)1008385182L);
    assert(p241_vibration_x_GET(pack) == (float)4.6654734E37F);
    assert(p241_time_usec_GET(pack) == (uint64_t)3593046151023791440L);
    assert(p241_vibration_y_GET(pack) == (float)7.753509E37F);
};


void c_CommunicationChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_z_GET(pack) == (float) -3.4373223E37F);
    {
        float exemplary[] =  {-1.920426E38F, 6.608992E37F, -1.4367643E37F, -2.3323621E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_latitude_GET(pack) == (int32_t)632377346);
    assert(p242_approach_y_GET(pack) == (float)1.2632916E38F);
    assert(p242_approach_x_GET(pack) == (float)2.2309111E38F);
    assert(p242_approach_z_GET(pack) == (float)2.6984574E38F);
    assert(p242_longitude_GET(pack) == (int32_t)963449330);
    assert(p242_x_GET(pack) == (float)9.265387E37F);
    assert(p242_altitude_GET(pack) == (int32_t)822957352);
    assert(p242_y_GET(pack) == (float) -3.1015451E38F);
    assert(p242_time_usec_TRY(ph) == (uint64_t)5467615960283793491L);
};


void c_CommunicationChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_y_GET(pack) == (float) -7.735065E37F);
    assert(p243_z_GET(pack) == (float) -1.7924746E38F);
    assert(p243_time_usec_TRY(ph) == (uint64_t)5893772108496882268L);
    assert(p243_altitude_GET(pack) == (int32_t) -1135089362);
    assert(p243_longitude_GET(pack) == (int32_t) -802307195);
    assert(p243_approach_z_GET(pack) == (float) -1.2722111E38F);
    assert(p243_x_GET(pack) == (float)3.1633691E38F);
    assert(p243_latitude_GET(pack) == (int32_t)588737200);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)110);
    assert(p243_approach_x_GET(pack) == (float)1.0280655E38F);
    {
        float exemplary[] =  {-5.201974E37F, -1.4145055E36F, 1.8726978E38F, 1.5689935E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_approach_y_GET(pack) == (float)6.9206147E37F);
};


void c_CommunicationChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)59372);
    assert(p244_interval_us_GET(pack) == (int32_t) -1844457700);
};


void c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR);
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_FW);
};


void c_CommunicationChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)10450);
    assert(p246_lat_GET(pack) == (int32_t)1069719980);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)2973159695L);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)46756);
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)222);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)20202);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
    assert(p246_altitude_GET(pack) == (int32_t)9255270);
    assert(p246_callsign_LEN(ph) == 8);
    {
        char16_t * exemplary = u"aimJlrmj";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_lon_GET(pack) == (int32_t) -1560081905);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LIGHT);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t) -8629);
    assert(p246_flags_GET(pack) == e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING);
};


void c_CommunicationChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_id_GET(pack) == (uint32_t)1713720901L);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float)3.0246385E38F);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_HOVER);
    assert(p247_time_to_minimum_delta_GET(pack) == (float)2.5633309E38F);
    assert(p247_altitude_minimum_delta_GET(pack) == (float)1.7954364E38F);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB);
};


void c_CommunicationChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)44);
    {
        uint8_t exemplary[] =  {(uint8_t)39, (uint8_t)180, (uint8_t)244, (uint8_t)193, (uint8_t)255, (uint8_t)103, (uint8_t)46, (uint8_t)255, (uint8_t)27, (uint8_t)236, (uint8_t)227, (uint8_t)171, (uint8_t)175, (uint8_t)67, (uint8_t)203, (uint8_t)43, (uint8_t)58, (uint8_t)105, (uint8_t)103, (uint8_t)226, (uint8_t)45, (uint8_t)172, (uint8_t)14, (uint8_t)234, (uint8_t)27, (uint8_t)78, (uint8_t)12, (uint8_t)148, (uint8_t)71, (uint8_t)171, (uint8_t)238, (uint8_t)59, (uint8_t)48, (uint8_t)87, (uint8_t)87, (uint8_t)82, (uint8_t)51, (uint8_t)4, (uint8_t)170, (uint8_t)199, (uint8_t)222, (uint8_t)128, (uint8_t)8, (uint8_t)13, (uint8_t)253, (uint8_t)51, (uint8_t)187, (uint8_t)82, (uint8_t)124, (uint8_t)186, (uint8_t)206, (uint8_t)216, (uint8_t)244, (uint8_t)227, (uint8_t)248, (uint8_t)197, (uint8_t)179, (uint8_t)171, (uint8_t)242, (uint8_t)13, (uint8_t)35, (uint8_t)104, (uint8_t)212, (uint8_t)138, (uint8_t)114, (uint8_t)157, (uint8_t)217, (uint8_t)133, (uint8_t)172, (uint8_t)106, (uint8_t)43, (uint8_t)143, (uint8_t)233, (uint8_t)208, (uint8_t)52, (uint8_t)155, (uint8_t)52, (uint8_t)183, (uint8_t)201, (uint8_t)62, (uint8_t)39, (uint8_t)4, (uint8_t)114, (uint8_t)124, (uint8_t)60, (uint8_t)164, (uint8_t)208, (uint8_t)245, (uint8_t)135, (uint8_t)49, (uint8_t)188, (uint8_t)15, (uint8_t)182, (uint8_t)78, (uint8_t)141, (uint8_t)16, (uint8_t)14, (uint8_t)37, (uint8_t)16, (uint8_t)245, (uint8_t)77, (uint8_t)19, (uint8_t)211, (uint8_t)87, (uint8_t)225, (uint8_t)117, (uint8_t)231, (uint8_t)198, (uint8_t)1, (uint8_t)127, (uint8_t)175, (uint8_t)98, (uint8_t)212, (uint8_t)0, (uint8_t)185, (uint8_t)100, (uint8_t)54, (uint8_t)214, (uint8_t)166, (uint8_t)125, (uint8_t)140, (uint8_t)148, (uint8_t)221, (uint8_t)141, (uint8_t)86, (uint8_t)149, (uint8_t)207, (uint8_t)241, (uint8_t)101, (uint8_t)228, (uint8_t)61, (uint8_t)60, (uint8_t)19, (uint8_t)152, (uint8_t)252, (uint8_t)116, (uint8_t)244, (uint8_t)203, (uint8_t)173, (uint8_t)248, (uint8_t)3, (uint8_t)157, (uint8_t)110, (uint8_t)94, (uint8_t)139, (uint8_t)116, (uint8_t)215, (uint8_t)7, (uint8_t)196, (uint8_t)47, (uint8_t)97, (uint8_t)172, (uint8_t)163, (uint8_t)99, (uint8_t)29, (uint8_t)211, (uint8_t)9, (uint8_t)218, (uint8_t)40, (uint8_t)220, (uint8_t)63, (uint8_t)245, (uint8_t)10, (uint8_t)59, (uint8_t)215, (uint8_t)101, (uint8_t)223, (uint8_t)106, (uint8_t)36, (uint8_t)227, (uint8_t)83, (uint8_t)255, (uint8_t)254, (uint8_t)87, (uint8_t)225, (uint8_t)71, (uint8_t)65, (uint8_t)210, (uint8_t)204, (uint8_t)124, (uint8_t)16, (uint8_t)174, (uint8_t)47, (uint8_t)254, (uint8_t)73, (uint8_t)93, (uint8_t)166, (uint8_t)209, (uint8_t)19, (uint8_t)121, (uint8_t)190, (uint8_t)85, (uint8_t)21, (uint8_t)171, (uint8_t)127, (uint8_t)201, (uint8_t)248, (uint8_t)147, (uint8_t)155, (uint8_t)100, (uint8_t)217, (uint8_t)109, (uint8_t)164, (uint8_t)244, (uint8_t)41, (uint8_t)78, (uint8_t)87, (uint8_t)116, (uint8_t)241, (uint8_t)138, (uint8_t)208, (uint8_t)226, (uint8_t)42, (uint8_t)81, (uint8_t)105, (uint8_t)41, (uint8_t)146, (uint8_t)86, (uint8_t)60, (uint8_t)144, (uint8_t)212, (uint8_t)150, (uint8_t)164, (uint8_t)226, (uint8_t)200, (uint8_t)190, (uint8_t)55, (uint8_t)65, (uint8_t)250, (uint8_t)115, (uint8_t)35, (uint8_t)161, (uint8_t)200, (uint8_t)44, (uint8_t)158, (uint8_t)51, (uint8_t)53, (uint8_t)189, (uint8_t)230, (uint8_t)231, (uint8_t)138, (uint8_t)179, (uint8_t)95, (uint8_t)235, (uint8_t)15, (uint8_t)186, (uint8_t)206, (uint8_t)209, (uint8_t)72} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)18);
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)178);
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)55048);
};


void c_CommunicationChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)79);
    {
        int8_t exemplary[] =  {(int8_t)4, (int8_t)93, (int8_t) -122, (int8_t) -32, (int8_t) -25, (int8_t)4, (int8_t) -97, (int8_t) -127, (int8_t)75, (int8_t)21, (int8_t)11, (int8_t)46, (int8_t)52, (int8_t) -61, (int8_t) -98, (int8_t)97, (int8_t) -49, (int8_t) -83, (int8_t) -72, (int8_t)122, (int8_t)43, (int8_t) -114, (int8_t) -72, (int8_t)11, (int8_t)32, (int8_t) -100, (int8_t) -103, (int8_t) -106, (int8_t)64, (int8_t)117, (int8_t)45, (int8_t) -2} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)50113);
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)40);
};


void c_CommunicationChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_y_GET(pack) == (float) -1.4080476E38F);
    assert(p250_z_GET(pack) == (float) -1.8162097E38F);
    assert(p250_name_LEN(ph) == 5);
    {
        char16_t * exemplary = u"dwqzh";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_time_usec_GET(pack) == (uint64_t)5470191345128341017L);
    assert(p250_x_GET(pack) == (float)2.7926489E38F);
};


void c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_value_GET(pack) == (float) -2.7340597E38F);
    assert(p251_name_LEN(ph) == 4);
    {
        char16_t * exemplary = u"mbtd";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)2889016376L);
};


void c_CommunicationChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_value_GET(pack) == (int32_t) -18282385);
    assert(p252_name_LEN(ph) == 6);
    {
        char16_t * exemplary = u"qjnojn";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)2685674959L);
};


void c_CommunicationChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_ERROR);
    assert(p253_text_LEN(ph) == 30);
    {
        char16_t * exemplary = u"laogFtzudckrlykqlBgyNdXarSjxac";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 60);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)157);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)1831354327L);
    assert(p254_value_GET(pack) == (float)8.0432273E37F);
};


void c_CommunicationChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)1516055848792011493L);
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)63);
    {
        uint8_t exemplary[] =  {(uint8_t)85, (uint8_t)135, (uint8_t)197, (uint8_t)113, (uint8_t)104, (uint8_t)142, (uint8_t)17, (uint8_t)174, (uint8_t)93, (uint8_t)145, (uint8_t)229, (uint8_t)195, (uint8_t)70, (uint8_t)32, (uint8_t)39, (uint8_t)236, (uint8_t)154, (uint8_t)251, (uint8_t)255, (uint8_t)156, (uint8_t)31, (uint8_t)109, (uint8_t)42, (uint8_t)87, (uint8_t)138, (uint8_t)165, (uint8_t)204, (uint8_t)179, (uint8_t)141, (uint8_t)172, (uint8_t)23, (uint8_t)112} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)174);
};


void c_CommunicationChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_last_change_ms_GET(pack) == (uint32_t)449390083L);
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)1893394770L);
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)151);
};


void c_CommunicationChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p258_tune_LEN(ph) == 19);
    {
        char16_t * exemplary = u"ykayaQwhTfqmmvdcjre";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 38);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_flags_GET(pack) == e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
    assert(p259_cam_definition_uri_LEN(ph) == 88);
    {
        char16_t * exemplary = u"lhprWhrfjgtSckihdumtfdvHongIsyjgfzrkYgooadkfrxvyimsuujcejnyjgseIwuzuvsuqilptnuneVfvfsvbs";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 176);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_sensor_size_h_GET(pack) == (float)5.6627737E37F);
    {
        uint8_t exemplary[] =  {(uint8_t)6, (uint8_t)80, (uint8_t)56, (uint8_t)234, (uint8_t)1, (uint8_t)12, (uint8_t)212, (uint8_t)47, (uint8_t)176, (uint8_t)116, (uint8_t)135, (uint8_t)240, (uint8_t)158, (uint8_t)161, (uint8_t)47, (uint8_t)170, (uint8_t)157, (uint8_t)34, (uint8_t)221, (uint8_t)56, (uint8_t)193, (uint8_t)79, (uint8_t)176, (uint8_t)18, (uint8_t)224, (uint8_t)200, (uint8_t)121, (uint8_t)219, (uint8_t)132, (uint8_t)101, (uint8_t)35, (uint8_t)37} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)64568);
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)3098976481L);
    assert(p259_firmware_version_GET(pack) == (uint32_t)2371953817L);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)28);
    {
        uint8_t exemplary[] =  {(uint8_t)154, (uint8_t)247, (uint8_t)177, (uint8_t)33, (uint8_t)32, (uint8_t)182, (uint8_t)176, (uint8_t)172, (uint8_t)94, (uint8_t)198, (uint8_t)28, (uint8_t)87, (uint8_t)242, (uint8_t)255, (uint8_t)62, (uint8_t)243, (uint8_t)216, (uint8_t)215, (uint8_t)64, (uint8_t)150, (uint8_t)223, (uint8_t)171, (uint8_t)79, (uint8_t)143, (uint8_t)112, (uint8_t)187, (uint8_t)114, (uint8_t)165, (uint8_t)182, (uint8_t)25, (uint8_t)5, (uint8_t)26} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_sensor_size_v_GET(pack) == (float) -1.8291476E38F);
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)644);
    assert(p259_focal_length_GET(pack) == (float)2.7059716E38F);
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)11847);
};


void c_CommunicationChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)1861889313L);
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY);
};


void c_CommunicationChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)3583965930L);
    assert(p261_used_capacity_GET(pack) == (float)3.377777E38F);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p261_available_capacity_GET(pack) == (float) -1.6362485E38F);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)177);
    assert(p261_write_speed_GET(pack) == (float) -3.0930514E38F);
    assert(p261_read_speed_GET(pack) == (float)2.1197546E38F);
    assert(p261_total_capacity_GET(pack) == (float) -3.139671E38F);
};


void c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)409385180L);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)1799417840L);
    assert(p262_image_interval_GET(pack) == (float)8.285761E37F);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p262_available_capacity_GET(pack) == (float)2.8452116E38F);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)254);
};


void c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {1.3887822E38F, 2.5962746E38F, -2.4482775E38F, -1.372586E38F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t) -93);
    assert(p263_lon_GET(pack) == (int32_t) -261760497);
    assert(p263_lat_GET(pack) == (int32_t)1305750309);
    assert(p263_time_utc_GET(pack) == (uint64_t)5966241922990225294L);
    assert(p263_file_url_LEN(ph) == 9);
    {
        char16_t * exemplary = u"orjlnnmal";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_alt_GET(pack) == (int32_t) -2070384364);
    assert(p263_relative_alt_GET(pack) == (int32_t)477841269);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)2107903128L);
    assert(p263_image_index_GET(pack) == (int32_t) -1976671550);
};


void c_CommunicationChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)2457793033L);
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)9006858912816301412L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)620601023238093248L);
    assert(p264_flight_uuid_GET(pack) == (uint64_t)4532303900450413308L);
};


void c_CommunicationChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_pitch_GET(pack) == (float) -2.6541054E37F);
    assert(p265_yaw_GET(pack) == (float)1.7026499E38F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)426983620L);
    assert(p265_roll_GET(pack) == (float) -1.3431301E38F);
};


void c_CommunicationChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)145, (uint8_t)169, (uint8_t)184, (uint8_t)153, (uint8_t)136, (uint8_t)230, (uint8_t)177, (uint8_t)169, (uint8_t)136, (uint8_t)120, (uint8_t)220, (uint8_t)65, (uint8_t)132, (uint8_t)6, (uint8_t)22, (uint8_t)5, (uint8_t)226, (uint8_t)220, (uint8_t)174, (uint8_t)211, (uint8_t)252, (uint8_t)233, (uint8_t)195, (uint8_t)205, (uint8_t)25, (uint8_t)141, (uint8_t)24, (uint8_t)62, (uint8_t)83, (uint8_t)10, (uint8_t)95, (uint8_t)54, (uint8_t)44, (uint8_t)147, (uint8_t)74, (uint8_t)209, (uint8_t)240, (uint8_t)8, (uint8_t)51, (uint8_t)249, (uint8_t)136, (uint8_t)136, (uint8_t)208, (uint8_t)124, (uint8_t)50, (uint8_t)89, (uint8_t)1, (uint8_t)96, (uint8_t)159, (uint8_t)138, (uint8_t)110, (uint8_t)46, (uint8_t)71, (uint8_t)55, (uint8_t)51, (uint8_t)97, (uint8_t)117, (uint8_t)23, (uint8_t)133, (uint8_t)223, (uint8_t)244, (uint8_t)47, (uint8_t)150, (uint8_t)115, (uint8_t)243, (uint8_t)194, (uint8_t)66, (uint8_t)149, (uint8_t)16, (uint8_t)251, (uint8_t)203, (uint8_t)219, (uint8_t)35, (uint8_t)50, (uint8_t)227, (uint8_t)44, (uint8_t)48, (uint8_t)5, (uint8_t)34, (uint8_t)72, (uint8_t)116, (uint8_t)125, (uint8_t)3, (uint8_t)55, (uint8_t)193, (uint8_t)79, (uint8_t)55, (uint8_t)32, (uint8_t)8, (uint8_t)153, (uint8_t)217, (uint8_t)234, (uint8_t)177, (uint8_t)78, (uint8_t)46, (uint8_t)163, (uint8_t)224, (uint8_t)9, (uint8_t)52, (uint8_t)1, (uint8_t)226, (uint8_t)225, (uint8_t)59, (uint8_t)243, (uint8_t)220, (uint8_t)154, (uint8_t)241, (uint8_t)6, (uint8_t)129, (uint8_t)10, (uint8_t)210, (uint8_t)39, (uint8_t)246, (uint8_t)34, (uint8_t)177, (uint8_t)243, (uint8_t)113, (uint8_t)235, (uint8_t)64, (uint8_t)123, (uint8_t)133, (uint8_t)218, (uint8_t)248, (uint8_t)17, (uint8_t)135, (uint8_t)219, (uint8_t)232, (uint8_t)223, (uint8_t)50, (uint8_t)197, (uint8_t)114, (uint8_t)102, (uint8_t)225, (uint8_t)103, (uint8_t)100, (uint8_t)150, (uint8_t)221, (uint8_t)42, (uint8_t)229, (uint8_t)169, (uint8_t)10, (uint8_t)106, (uint8_t)166, (uint8_t)186, (uint8_t)1, (uint8_t)245, (uint8_t)147, (uint8_t)78, (uint8_t)47, (uint8_t)196, (uint8_t)49, (uint8_t)52, (uint8_t)6, (uint8_t)17, (uint8_t)87, (uint8_t)180, (uint8_t)28, (uint8_t)44, (uint8_t)220, (uint8_t)129, (uint8_t)134, (uint8_t)157, (uint8_t)236, (uint8_t)177, (uint8_t)166, (uint8_t)182, (uint8_t)157, (uint8_t)187, (uint8_t)31, (uint8_t)135, (uint8_t)141, (uint8_t)3, (uint8_t)209, (uint8_t)149, (uint8_t)45, (uint8_t)104, (uint8_t)2, (uint8_t)37, (uint8_t)148, (uint8_t)39, (uint8_t)216, (uint8_t)228, (uint8_t)57, (uint8_t)125, (uint8_t)91, (uint8_t)104, (uint8_t)229, (uint8_t)217, (uint8_t)167, (uint8_t)237, (uint8_t)227, (uint8_t)155, (uint8_t)188, (uint8_t)35, (uint8_t)143, (uint8_t)243, (uint8_t)0, (uint8_t)230, (uint8_t)172, (uint8_t)79, (uint8_t)143, (uint8_t)73, (uint8_t)48, (uint8_t)144, (uint8_t)18, (uint8_t)23, (uint8_t)169, (uint8_t)32, (uint8_t)183, (uint8_t)246, (uint8_t)208, (uint8_t)105, (uint8_t)10, (uint8_t)22, (uint8_t)147, (uint8_t)187, (uint8_t)234, (uint8_t)98, (uint8_t)111, (uint8_t)71, (uint8_t)206, (uint8_t)113, (uint8_t)34, (uint8_t)202, (uint8_t)120, (uint8_t)214, (uint8_t)58, (uint8_t)27, (uint8_t)66, (uint8_t)221, (uint8_t)235, (uint8_t)14, (uint8_t)158, (uint8_t)119, (uint8_t)91, (uint8_t)207, (uint8_t)10, (uint8_t)84, (uint8_t)151, (uint8_t)55, (uint8_t)50, (uint8_t)232, (uint8_t)127, (uint8_t)66, (uint8_t)23, (uint8_t)122, (uint8_t)43, (uint8_t)25, (uint8_t)20} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)30);
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)157);
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)38);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)31509);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)11);
};


void c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)237);
    {
        uint8_t exemplary[] =  {(uint8_t)205, (uint8_t)244, (uint8_t)217, (uint8_t)189, (uint8_t)156, (uint8_t)143, (uint8_t)250, (uint8_t)93, (uint8_t)71, (uint8_t)108, (uint8_t)66, (uint8_t)76, (uint8_t)240, (uint8_t)246, (uint8_t)165, (uint8_t)112, (uint8_t)152, (uint8_t)249, (uint8_t)10, (uint8_t)44, (uint8_t)89, (uint8_t)245, (uint8_t)49, (uint8_t)108, (uint8_t)16, (uint8_t)243, (uint8_t)221, (uint8_t)9, (uint8_t)245, (uint8_t)13, (uint8_t)154, (uint8_t)193, (uint8_t)182, (uint8_t)105, (uint8_t)105, (uint8_t)230, (uint8_t)243, (uint8_t)90, (uint8_t)159, (uint8_t)250, (uint8_t)231, (uint8_t)243, (uint8_t)64, (uint8_t)252, (uint8_t)159, (uint8_t)33, (uint8_t)0, (uint8_t)40, (uint8_t)101, (uint8_t)27, (uint8_t)83, (uint8_t)143, (uint8_t)191, (uint8_t)55, (uint8_t)49, (uint8_t)204, (uint8_t)200, (uint8_t)205, (uint8_t)13, (uint8_t)74, (uint8_t)233, (uint8_t)199, (uint8_t)34, (uint8_t)80, (uint8_t)242, (uint8_t)61, (uint8_t)66, (uint8_t)185, (uint8_t)132, (uint8_t)145, (uint8_t)244, (uint8_t)26, (uint8_t)171, (uint8_t)92, (uint8_t)9, (uint8_t)9, (uint8_t)171, (uint8_t)202, (uint8_t)150, (uint8_t)116, (uint8_t)242, (uint8_t)177, (uint8_t)34, (uint8_t)76, (uint8_t)6, (uint8_t)96, (uint8_t)98, (uint8_t)135, (uint8_t)1, (uint8_t)98, (uint8_t)247, (uint8_t)222, (uint8_t)10, (uint8_t)78, (uint8_t)197, (uint8_t)5, (uint8_t)210, (uint8_t)216, (uint8_t)214, (uint8_t)138, (uint8_t)137, (uint8_t)190, (uint8_t)62, (uint8_t)145, (uint8_t)159, (uint8_t)223, (uint8_t)103, (uint8_t)249, (uint8_t)55, (uint8_t)204, (uint8_t)12, (uint8_t)107, (uint8_t)22, (uint8_t)126, (uint8_t)198, (uint8_t)186, (uint8_t)207, (uint8_t)235, (uint8_t)110, (uint8_t)118, (uint8_t)80, (uint8_t)23, (uint8_t)122, (uint8_t)163, (uint8_t)3, (uint8_t)182, (uint8_t)176, (uint8_t)179, (uint8_t)52, (uint8_t)189, (uint8_t)65, (uint8_t)221, (uint8_t)114, (uint8_t)245, (uint8_t)245, (uint8_t)200, (uint8_t)5, (uint8_t)46, (uint8_t)75, (uint8_t)186, (uint8_t)253, (uint8_t)38, (uint8_t)71, (uint8_t)196, (uint8_t)199, (uint8_t)131, (uint8_t)113, (uint8_t)67, (uint8_t)28, (uint8_t)127, (uint8_t)200, (uint8_t)117, (uint8_t)104, (uint8_t)184, (uint8_t)177, (uint8_t)28, (uint8_t)164, (uint8_t)228, (uint8_t)232, (uint8_t)58, (uint8_t)69, (uint8_t)43, (uint8_t)172, (uint8_t)94, (uint8_t)159, (uint8_t)237, (uint8_t)152, (uint8_t)173, (uint8_t)159, (uint8_t)95, (uint8_t)112, (uint8_t)89, (uint8_t)53, (uint8_t)21, (uint8_t)195, (uint8_t)27, (uint8_t)192, (uint8_t)128, (uint8_t)17, (uint8_t)145, (uint8_t)177, (uint8_t)170, (uint8_t)151, (uint8_t)45, (uint8_t)7, (uint8_t)2, (uint8_t)239, (uint8_t)73, (uint8_t)42, (uint8_t)10, (uint8_t)54, (uint8_t)123, (uint8_t)159, (uint8_t)89, (uint8_t)158, (uint8_t)160, (uint8_t)13, (uint8_t)153, (uint8_t)243, (uint8_t)66, (uint8_t)163, (uint8_t)154, (uint8_t)211, (uint8_t)33, (uint8_t)197, (uint8_t)22, (uint8_t)217, (uint8_t)3, (uint8_t)245, (uint8_t)224, (uint8_t)91, (uint8_t)107, (uint8_t)49, (uint8_t)237, (uint8_t)180, (uint8_t)78, (uint8_t)196, (uint8_t)67, (uint8_t)228, (uint8_t)81, (uint8_t)200, (uint8_t)26, (uint8_t)82, (uint8_t)166, (uint8_t)226, (uint8_t)214, (uint8_t)239, (uint8_t)194, (uint8_t)27, (uint8_t)202, (uint8_t)114, (uint8_t)124, (uint8_t)189, (uint8_t)239, (uint8_t)220, (uint8_t)239, (uint8_t)77, (uint8_t)150, (uint8_t)223, (uint8_t)8, (uint8_t)248, (uint8_t)176, (uint8_t)150, (uint8_t)57, (uint8_t)107, (uint8_t)240, (uint8_t)86, (uint8_t)151, (uint8_t)175} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)79);
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)52175);
};


void c_CommunicationChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)17486);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)207);
};


void c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_uri_LEN(ph) == 19);
    {
        char16_t * exemplary = u"nFoxfbzzmlqtppfwjle";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 38);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_bitrate_GET(pack) == (uint32_t)3410945479L);
    assert(p269_framerate_GET(pack) == (float)2.2096894E38F);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)31083);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)14492);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)53829);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)38);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)180);
};


void c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)30547);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)1433);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)35057);
    assert(p270_framerate_GET(pack) == (float)1.8216636E38F);
    assert(p270_uri_LEN(ph) == 224);
    {
        char16_t * exemplary = u"poxmjYuxydmrkusxfbrqeEsqrfmnxfcnrpbnjosegRasozbhzxoranyszpqempTuiuioEkrejtgpqighrRnddvemykzuqdYuhumcgstumpjtvdrgioywoynhJtexmUlocqwmtbngwrgDuyeoqoycqKZlsslhogshqviyryjrdzjacsyOYqxbsxfhhashiyduupcXfhcymglzVfwvdshtafwGbzgsbded";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 448);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_bitrate_GET(pack) == (uint32_t)165600205L);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)122);
};


void c_CommunicationChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_ssid_LEN(ph) == 22);
    {
        char16_t * exemplary = u"lrhzctJwezxhwgtgtulgjz";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 44);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_password_LEN(ph) == 58);
    {
        char16_t * exemplary = u"ceJGgzmmhwHxsieszZhnbqijhwfzfcdvxprdwltrtoqsxIsyxAovdbuqlt";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 116);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)59115);
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)29829);
    {
        uint8_t exemplary[] =  {(uint8_t)25, (uint8_t)126, (uint8_t)32, (uint8_t)4, (uint8_t)162, (uint8_t)34, (uint8_t)49, (uint8_t)82} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)12066);
    {
        uint8_t exemplary[] =  {(uint8_t)73, (uint8_t)34, (uint8_t)194, (uint8_t)251, (uint8_t)156, (uint8_t)38, (uint8_t)190, (uint8_t)139} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)18460);
    assert(p310_time_usec_GET(pack) == (uint64_t)125744112668726778L);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR);
    assert(p310_uptime_sec_GET(pack) == (uint32_t)3873453746L);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)22);
};


void c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p311_uptime_sec_GET(pack) == (uint32_t)1583425061L);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)107);
    {
        uint8_t exemplary[] =  {(uint8_t)78, (uint8_t)155, (uint8_t)94, (uint8_t)34, (uint8_t)243, (uint8_t)77, (uint8_t)115, (uint8_t)177, (uint8_t)208, (uint8_t)173, (uint8_t)115, (uint8_t)190, (uint8_t)54, (uint8_t)221, (uint8_t)31, (uint8_t)130} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p311_time_usec_GET(pack) == (uint64_t)2945280521365090318L);
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)169);
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)3025369553L);
    assert(p311_name_LEN(ph) == 40);
    {
        char16_t * exemplary = u"flheyojocxfxzthnvvaabfOiHbBaccshytqtrozt";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 80);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t) -11335);
    assert(p320_param_id_LEN(ph) == 6);
    {
        char16_t * exemplary = u"sullyy";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)173);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)65);
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)209);
};


void c_CommunicationChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32);
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)25089);
    assert(p322_param_id_LEN(ph) == 4);
    {
        char16_t * exemplary = u"pzjb";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_value_LEN(ph) == 107);
    {
        char16_t * exemplary = u"iohfYsWvgYuxxfcQmjyWVlfohhwwrwldbjboncoXuqfrbhnuXunyraiiqfsphOpufzmkomsrxdjqwynbyIitdmlatLwlWbcBXmmtlbZstyy";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 214);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)38717);
};


void c_CommunicationChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)115);
    assert(p323_param_value_LEN(ph) == 28);
    {
        char16_t * exemplary = u"bDgiuorzrbuecfjZjcecazitfxWo";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 56);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32);
    assert(p323_param_id_LEN(ph) == 4);
    {
        char16_t * exemplary = u"pcbd";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)222);
};


void c_CommunicationChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_id_LEN(ph) == 2);
    {
        char16_t * exemplary = u"lb";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_ACCEPTED);
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8);
    assert(p324_param_value_LEN(ph) == 29);
    {
        char16_t * exemplary = u"pijwGVnqdlngukyQuioqmdfmgDeno";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 58);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)47733);
    {
        uint16_t exemplary[] =  {(uint16_t)36849, (uint16_t)44779, (uint16_t)25423, (uint16_t)17511, (uint16_t)26650, (uint16_t)3475, (uint16_t)61505, (uint16_t)32221, (uint16_t)44780, (uint16_t)39975, (uint16_t)30588, (uint16_t)56288, (uint16_t)51494, (uint16_t)17063, (uint16_t)62676, (uint16_t)23797, (uint16_t)23515, (uint16_t)49672, (uint16_t)32695, (uint16_t)16576, (uint16_t)52166, (uint16_t)13718, (uint16_t)4233, (uint16_t)6496, (uint16_t)14052, (uint16_t)43255, (uint16_t)31354, (uint16_t)5956, (uint16_t)22143, (uint16_t)15894, (uint16_t)18681, (uint16_t)32984, (uint16_t)51455, (uint16_t)62374, (uint16_t)35219, (uint16_t)497, (uint16_t)43159, (uint16_t)49203, (uint16_t)27878, (uint16_t)22191, (uint16_t)2474, (uint16_t)18834, (uint16_t)56488, (uint16_t)63261, (uint16_t)8160, (uint16_t)12219, (uint16_t)56499, (uint16_t)14254, (uint16_t)17889, (uint16_t)51149, (uint16_t)41184, (uint16_t)19428, (uint16_t)1227, (uint16_t)58779, (uint16_t)19092, (uint16_t)6596, (uint16_t)49198, (uint16_t)15279, (uint16_t)19026, (uint16_t)12885, (uint16_t)60199, (uint16_t)26894, (uint16_t)64479, (uint16_t)54343, (uint16_t)31218, (uint16_t)42574, (uint16_t)14484, (uint16_t)55927, (uint16_t)48709, (uint16_t)6151, (uint16_t)44540, (uint16_t)21434} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)10331);
    assert(p330_time_usec_GET(pack) == (uint64_t)957320193579351525L);
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
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_VTOL_TILTROTOR, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_CRITICAL, PH.base.pack) ;
        p0_custom_mode_SET((uint32_t)2588040869L, PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_OPENPILOT, PH.base.pack) ;
        p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED, PH.base.pack) ;
        c_TEST_Channel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
        p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t)7037, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)19186, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t)75, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)25234, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE, PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)57777, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)9572, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)41647, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)32120, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)64675, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)59846, PH.base.pack) ;
        c_TEST_Channel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_boot_ms_SET((uint32_t)2560823329L, PH.base.pack) ;
        p2_time_unix_usec_SET((uint64_t)3006717002084327655L, PH.base.pack) ;
        c_TEST_Channel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_type_mask_SET((uint16_t)(uint16_t)26504, PH.base.pack) ;
        p3_afx_SET((float) -1.9116828E38F, PH.base.pack) ;
        p3_afy_SET((float) -1.4177441E38F, PH.base.pack) ;
        p3_vx_SET((float)9.649564E37F, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p3_vz_SET((float) -2.8771981E38F, PH.base.pack) ;
        p3_vy_SET((float)2.9454864E38F, PH.base.pack) ;
        p3_y_SET((float)2.837918E38F, PH.base.pack) ;
        p3_afz_SET((float)1.5039224E38F, PH.base.pack) ;
        p3_x_SET((float)3.3664583E38F, PH.base.pack) ;
        p3_yaw_SET((float)2.7323831E38F, PH.base.pack) ;
        p3_yaw_rate_SET((float)5.919883E37F, PH.base.pack) ;
        p3_z_SET((float)1.3186221E38F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)1389850673L, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PING_4(), &PH);
        p4_time_usec_SET((uint64_t)5696199367167570760L, PH.base.pack) ;
        p4_target_component_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        p4_target_system_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p4_seq_SET((uint32_t)1041195585L, PH.base.pack) ;
        c_TEST_Channel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_control_request_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        {
            char16_t* passkey = u"dji";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_target_system_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p5_version_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_ack_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        p6_control_request_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"jwbqFiqlgsblfveqbigbtPnnamxo";
            p7_key_SET_(key, &PH) ;
        }
        c_TEST_Channel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
        p11_custom_mode_SET((uint32_t)1277609598L, PH.base.pack) ;
        p11_target_system_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED, PH.base.pack) ;
        c_TEST_Channel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
        p20_target_system_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
        p20_target_component_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        p20_param_index_SET((int16_t)(int16_t)27704, PH.base.pack) ;
        {
            char16_t* param_id = u"zOTGmvdvhftzr";
            p20_param_id_SET_(param_id, &PH) ;
        }
        c_TEST_Channel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_system_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
        p21_target_component_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
        p22_param_count_SET((uint16_t)(uint16_t)10211, PH.base.pack) ;
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64, PH.base.pack) ;
        {
            char16_t* param_id = u"z";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_value_SET((float)8.0050345E37F, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)45996, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
        p23_param_value_SET((float)2.2259001E38F, PH.base.pack) ;
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16, PH.base.pack) ;
        p23_target_system_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        p23_target_component_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        {
            char16_t* param_id = u"hgdylsvp";
            p23_param_id_SET_(param_id, &PH) ;
        }
        c_TEST_Channel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
        p24_v_acc_SET((uint32_t)256554595L, &PH) ;
        p24_vel_SET((uint16_t)(uint16_t)5535, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)14879, PH.base.pack) ;
        p24_eph_SET((uint16_t)(uint16_t)49312, PH.base.pack) ;
        p24_alt_ellipsoid_SET((int32_t)2083780793, &PH) ;
        p24_epv_SET((uint16_t)(uint16_t)54651, PH.base.pack) ;
        p24_lon_SET((int32_t)290237011, PH.base.pack) ;
        p24_h_acc_SET((uint32_t)1345742975L, &PH) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, PH.base.pack) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p24_lat_SET((int32_t)2008758175, PH.base.pack) ;
        p24_hdg_acc_SET((uint32_t)3710163861L, &PH) ;
        p24_alt_SET((int32_t) -1105092772, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)3642784836L, &PH) ;
        p24_time_usec_SET((uint64_t)4342790446797420163L, PH.base.pack) ;
        c_TEST_Channel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_used[] =  {(uint8_t)113, (uint8_t)228, (uint8_t)168, (uint8_t)93, (uint8_t)108, (uint8_t)35, (uint8_t)2, (uint8_t)49, (uint8_t)242, (uint8_t)127, (uint8_t)180, (uint8_t)182, (uint8_t)244, (uint8_t)118, (uint8_t)145, (uint8_t)32, (uint8_t)123, (uint8_t)42, (uint8_t)173, (uint8_t)18};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_snr[] =  {(uint8_t)105, (uint8_t)17, (uint8_t)28, (uint8_t)106, (uint8_t)222, (uint8_t)215, (uint8_t)69, (uint8_t)255, (uint8_t)238, (uint8_t)182, (uint8_t)146, (uint8_t)144, (uint8_t)156, (uint8_t)163, (uint8_t)242, (uint8_t)221, (uint8_t)118, (uint8_t)70, (uint8_t)12, (uint8_t)222};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)200, (uint8_t)108, (uint8_t)203, (uint8_t)79, (uint8_t)149, (uint8_t)52, (uint8_t)3, (uint8_t)190, (uint8_t)45, (uint8_t)238, (uint8_t)88, (uint8_t)190, (uint8_t)246, (uint8_t)178, (uint8_t)33, (uint8_t)248, (uint8_t)16, (uint8_t)232, (uint8_t)88, (uint8_t)24};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_prn[] =  {(uint8_t)56, (uint8_t)12, (uint8_t)19, (uint8_t)116, (uint8_t)170, (uint8_t)216, (uint8_t)50, (uint8_t)196, (uint8_t)247, (uint8_t)190, (uint8_t)248, (uint8_t)120, (uint8_t)36, (uint8_t)158, (uint8_t)65, (uint8_t)132, (uint8_t)70, (uint8_t)7, (uint8_t)143, (uint8_t)186};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)142, (uint8_t)106, (uint8_t)73, (uint8_t)38, (uint8_t)7, (uint8_t)131, (uint8_t)55, (uint8_t)144, (uint8_t)122, (uint8_t)9, (uint8_t)99, (uint8_t)42, (uint8_t)11, (uint8_t)175, (uint8_t)175, (uint8_t)247, (uint8_t)134, (uint8_t)255, (uint8_t)151, (uint8_t)95};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
        c_TEST_Channel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
        p26_xgyro_SET((int16_t)(int16_t) -1104, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t)14671, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t) -21362, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t) -1631, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)2580115909L, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t)10065, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t)969, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t) -31076, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t) -4857, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t) -6624, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
        p27_xgyro_SET((int16_t)(int16_t)27012, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t)18309, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t) -9999, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t)31228, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)2849045302744260408L, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t)14138, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t) -31941, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t)20452, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t)31942, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t)21214, PH.base.pack) ;
        c_TEST_Channel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
        p28_time_usec_SET((uint64_t)2428858149517646322L, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t)24820, PH.base.pack) ;
        p28_press_diff1_SET((int16_t)(int16_t)354, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t) -21351, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t) -13597, PH.base.pack) ;
        c_TEST_Channel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_press_abs_SET((float) -2.000196E38F, PH.base.pack) ;
        p29_press_diff_SET((float)2.6633962E38F, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)289928300L, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t)7835, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
        p30_yaw_SET((float)4.8542984E36F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)2222553745L, PH.base.pack) ;
        p30_roll_SET((float)1.7561372E38F, PH.base.pack) ;
        p30_rollspeed_SET((float) -1.4925807E38F, PH.base.pack) ;
        p30_yawspeed_SET((float)2.933856E38F, PH.base.pack) ;
        p30_pitch_SET((float)2.0670073E38F, PH.base.pack) ;
        p30_pitchspeed_SET((float) -7.378402E37F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_q2_SET((float)2.017071E38F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)2302489818L, PH.base.pack) ;
        p31_pitchspeed_SET((float) -9.633772E37F, PH.base.pack) ;
        p31_yawspeed_SET((float)2.0195762E37F, PH.base.pack) ;
        p31_rollspeed_SET((float)1.0238272E38F, PH.base.pack) ;
        p31_q4_SET((float)7.398097E37F, PH.base.pack) ;
        p31_q3_SET((float) -7.6729304E37F, PH.base.pack) ;
        p31_q1_SET((float)2.7724622E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_y_SET((float)2.5022445E38F, PH.base.pack) ;
        p32_vy_SET((float) -2.366788E38F, PH.base.pack) ;
        p32_vz_SET((float)2.4119257E38F, PH.base.pack) ;
        p32_vx_SET((float) -6.0139494E37F, PH.base.pack) ;
        p32_z_SET((float)3.2542475E38F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)553880383L, PH.base.pack) ;
        p32_x_SET((float)1.0681679E38F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_relative_alt_SET((int32_t) -528504218, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)1109798377L, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)3432, PH.base.pack) ;
        p33_lat_SET((int32_t) -741194677, PH.base.pack) ;
        p33_alt_SET((int32_t) -1311474927, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t) -16606, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t) -22008, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t) -16767, PH.base.pack) ;
        p33_lon_SET((int32_t)791322167, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan4_scaled_SET((int16_t)(int16_t)16902, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)2963744756L, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t)27626, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t) -15316, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t)24914, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t)27481, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t) -4322, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t)31820, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t)14004, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_chan6_raw_SET((uint16_t)(uint16_t)8472, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)28810, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)47060, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)9822, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)1230914968L, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)36500, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)57801, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)24041, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)43518, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo4_raw_SET((uint16_t)(uint16_t)28342, PH.base.pack) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)64573, PH.base.pack) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)53989, PH.base.pack) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)53469, PH.base.pack) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)2257, &PH) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)17731, PH.base.pack) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)47778, &PH) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)15164, PH.base.pack) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)57737, &PH) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)35159, &PH) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)65159, PH.base.pack) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)42952, PH.base.pack) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)52271, &PH) ;
        p36_port_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)43580, &PH) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)65205, &PH) ;
        p36_time_usec_SET((uint32_t)2554852049L, PH.base.pack) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)58127, &PH) ;
        c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_start_index_SET((int16_t)(int16_t)26148, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t) -20157, PH.base.pack) ;
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_target_component_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t) -30792, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t)17174, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
        p39_z_SET((float) -2.3284858E37F, PH.base.pack) ;
        p39_param3_SET((float) -2.0849007E38F, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p39_param4_SET((float) -2.9774281E38F, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
        p39_x_SET((float)5.52412E37F, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)12453, PH.base.pack) ;
        p39_y_SET((float) -1.887468E38F, PH.base.pack) ;
        p39_param2_SET((float) -1.6986304E38F, PH.base.pack) ;
        p39_param1_SET((float) -3.0556894E38F, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p40_target_component_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)5495, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_target_system_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        p41_seq_SET((uint16_t)(uint16_t)29478, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)49332, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_target_component_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        p43_target_system_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
        p44_count_SET((uint16_t)(uint16_t)55790, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_target_system_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)18553, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
        p47_target_system_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM1, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_time_usec_SET((uint64_t)5894979835848281260L, &PH) ;
        p48_longitude_SET((int32_t)862802992, PH.base.pack) ;
        p48_target_system_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        p48_altitude_SET((int32_t)1955629113, PH.base.pack) ;
        p48_latitude_SET((int32_t)1233035359, PH.base.pack) ;
        c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_altitude_SET((int32_t) -1429838968, PH.base.pack) ;
        p49_latitude_SET((int32_t)1369347535, PH.base.pack) ;
        p49_longitude_SET((int32_t) -2072035595, PH.base.pack) ;
        p49_time_usec_SET((uint64_t)3884016250955752138L, &PH) ;
        c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_param_value_max_SET((float)2.728794E38F, PH.base.pack) ;
        p50_param_value0_SET((float)3.3725787E38F, PH.base.pack) ;
        {
            char16_t* param_id = u"PxhskdXffrJeCrlw";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        p50_param_value_min_SET((float) -2.4902438E37F, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p50_scale_SET((float)2.2729497E38F, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t)12029, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p51_target_component_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)24875, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p54_p1y_SET((float)6.057524E37F, PH.base.pack) ;
        p54_p2y_SET((float)2.423597E38F, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p54_p2z_SET((float) -3.2671917E38F, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p54_p1x_SET((float) -1.2124232E38F, PH.base.pack) ;
        p54_p1z_SET((float)1.9929335E38F, PH.base.pack) ;
        p54_p2x_SET((float) -1.749972E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p2x_SET((float)3.1691131E38F, PH.base.pack) ;
        p55_p1x_SET((float) -1.91839E38F, PH.base.pack) ;
        p55_p1y_SET((float)2.242803E38F, PH.base.pack) ;
        p55_p2z_SET((float)4.841926E37F, PH.base.pack) ;
        p55_p2y_SET((float) -1.7706823E38F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p55_p1z_SET((float) -8.471325E37F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_rollspeed_SET((float)6.1506316E37F, PH.base.pack) ;
        {
            float covariance[] =  {-1.9221485E38F, 2.4178993E38F, 1.8180485E38F, 2.9718068E38F, 1.7870898E38F, -6.0043194E37F, -1.8932656E38F, -1.0013715E38F, -1.6219966E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p61_time_usec_SET((uint64_t)3651969912067895099L, PH.base.pack) ;
        p61_yawspeed_SET((float) -1.8946653E38F, PH.base.pack) ;
        p61_pitchspeed_SET((float)1.0350404E38F, PH.base.pack) ;
        {
            float q[] =  {-3.2124688E38F, -2.0430964E38F, 2.4926877E38F, -3.1524112E38F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_nav_pitch_SET((float)9.653279E37F, PH.base.pack) ;
        p62_nav_roll_SET((float)2.4081122E38F, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t) -32181, PH.base.pack) ;
        p62_aspd_error_SET((float) -2.4898078E38F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)59368, PH.base.pack) ;
        p62_alt_error_SET((float)4.7566723E37F, PH.base.pack) ;
        p62_xtrack_error_SET((float) -1.4828465E38F, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t) -142, PH.base.pack) ;
        c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_vy_SET((float) -3.357298E37F, PH.base.pack) ;
        p63_lon_SET((int32_t)1569097259, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)7266695339762267279L, PH.base.pack) ;
        p63_alt_SET((int32_t)911946655, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
        p63_vx_SET((float)2.4206626E38F, PH.base.pack) ;
        p63_vz_SET((float) -1.2741902E38F, PH.base.pack) ;
        {
            float covariance[] =  {8.1721844E37F, 1.1451685E38F, -3.0626948E38F, -8.2992765E37F, 7.6783083E37F, -1.4441783E37F, -1.0034734E38F, -2.7122123E38F, -3.3119684E38F, -1.4214218E38F, 8.3957924E37F, -3.0075436E38F, 2.2809644E38F, -8.945184E37F, -2.230106E38F, -2.3184957E37F, 2.4246527E38F, 8.513528E37F, -1.3449332E38F, -1.6514884E37F, 4.893824E37F, -2.1918644E38F, 9.879083E37F, 2.8718975E38F, 3.1912627E38F, -1.3652356E38F, 1.551185E35F, 2.954596E38F, -1.218206E37F, 2.3888853E38F, -1.5947448E38F, -3.2555707E38F, 2.7754997E38F, -3.4108627E37F, 2.921796E37F, -3.402068E38F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_lat_SET((int32_t) -1002884933, PH.base.pack) ;
        p63_relative_alt_SET((int32_t)445912849, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_time_usec_SET((uint64_t)3238861729715010795L, PH.base.pack) ;
        p64_z_SET((float)5.355817E37F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, PH.base.pack) ;
        p64_ay_SET((float) -8.081211E37F, PH.base.pack) ;
        p64_vy_SET((float)2.8617786E38F, PH.base.pack) ;
        p64_x_SET((float) -1.219195E38F, PH.base.pack) ;
        p64_vz_SET((float) -2.9973596E38F, PH.base.pack) ;
        p64_ax_SET((float) -2.1060666E38F, PH.base.pack) ;
        p64_az_SET((float)2.606813E38F, PH.base.pack) ;
        p64_y_SET((float)4.1672644E37F, PH.base.pack) ;
        {
            float covariance[] =  {1.9248606E37F, 3.1115695E38F, 6.052559E37F, -2.6584692E38F, 1.0833009E38F, 5.3149687E36F, -2.045917E38F, 7.6804704E37F, -7.8230653E37F, -9.7712315E36F, 8.251791E37F, 3.9670644E37F, 2.5207117E38F, 9.149471E37F, -1.2550986E38F, -1.8891422E38F, 2.9272945E38F, -3.028248E37F, -3.1424044E37F, 1.1996983E38F, 2.8548532E38F, -1.8029389E38F, -1.8067406E38F, 1.6401287E38F, -3.3340957E38F, -9.731827E37F, -3.0359488E38F, -2.3442192E38F, 2.4935324E38F, -1.4252892E38F, -2.1436012E38F, -1.6928125E38F, -2.2656176E38F, -7.470767E37F, 1.823955E38F, 2.3205295E38F, -2.6945208E38F, -2.6355945E38F, 3.0762012E38F, 6.5407607E37F, 1.2711362E38F, -1.0533232E38F, -2.023761E38F, -2.8617646E38F, 1.1298783E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_vx_SET((float) -2.8798892E37F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan12_raw_SET((uint16_t)(uint16_t)10981, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)16702, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)59951, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)33628, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)65518, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)11539, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)25079, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)28345, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)49241, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)4784, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)14027, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)852732036L, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)60139, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)21996, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)36578, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)35412, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)32348, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)25752, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)34280, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_req_message_rate_SET((uint16_t)(uint16_t)43801, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        c_TEST_Channel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
        p67_on_off_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p67_message_rate_SET((uint16_t)(uint16_t)47033, PH.base.pack) ;
        p67_stream_id_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        c_TEST_Channel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_r_SET((int16_t)(int16_t)29326, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t) -9645, PH.base.pack) ;
        p69_buttons_SET((uint16_t)(uint16_t)29150, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t)29873, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t) -3706, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan1_raw_SET((uint16_t)(uint16_t)1745, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)30483, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)33226, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)29323, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)41369, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)63723, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)52042, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)802, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_param2_SET((float) -4.351673E37F, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)34632, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        p73_param3_SET((float)3.3955934E38F, PH.base.pack) ;
        p73_param1_SET((float)1.592505E38F, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p73_y_SET((int32_t) -1554072960, PH.base.pack) ;
        p73_x_SET((int32_t)1755720764, PH.base.pack) ;
        p73_z_SET((float) -1.4173052E38F, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p73_param4_SET((float)1.5997715E38F, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
        p74_airspeed_SET((float) -1.7381563E38F, PH.base.pack) ;
        p74_alt_SET((float)7.902866E37F, PH.base.pack) ;
        p74_climb_SET((float)2.974927E38F, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)32015, PH.base.pack) ;
        p74_groundspeed_SET((float)1.1167466E38F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t)29312, PH.base.pack) ;
        c_TEST_Channel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
        p75_z_SET((float) -8.410628E37F, PH.base.pack) ;
        p75_param4_SET((float) -2.8123031E38F, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p75_param3_SET((float)2.2291092E38F, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        p75_y_SET((int32_t)430182603, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE, PH.base.pack) ;
        p75_param2_SET((float)3.3342576E38F, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        p75_param1_SET((float) -2.035717E38F, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p75_x_SET((int32_t)941072442, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
        p76_target_component_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p76_param2_SET((float)2.082015E38F, PH.base.pack) ;
        p76_param5_SET((float) -2.1444863E37F, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        p76_param1_SET((float) -4.2524166E37F, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_STOP_RETURN_TO_BASE, PH.base.pack) ;
        p76_param3_SET((float) -1.9368212E38F, PH.base.pack) ;
        p76_param6_SET((float) -1.1510447E38F, PH.base.pack) ;
        p76_param4_SET((float)2.6685858E38F, PH.base.pack) ;
        p76_param7_SET((float)9.995622E37F, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
        p77_progress_SET((uint8_t)(uint8_t)76, &PH) ;
        p77_result_param2_SET((int32_t) -1340272510, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_IN_PROGRESS, PH.base.pack) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_NAV_FOLLOW, PH.base.pack) ;
        p77_target_component_SET((uint8_t)(uint8_t)232, &PH) ;
        p77_target_system_SET((uint8_t)(uint8_t)96, &PH) ;
        c_TEST_Channel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_thrust_SET((float)2.434619E38F, PH.base.pack) ;
        p81_roll_SET((float)3.2915217E38F, PH.base.pack) ;
        p81_yaw_SET((float)2.1937834E38F, PH.base.pack) ;
        p81_pitch_SET((float) -3.2986232E38F, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)3944169920L, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_body_pitch_rate_SET((float) -9.678533E37F, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        {
            float q[] =  {-1.8710545E38F, -8.532836E37F, -1.7835856E38F, -2.555077E37F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_target_system_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        p82_body_roll_rate_SET((float) -6.9007816E37F, PH.base.pack) ;
        p82_time_boot_ms_SET((uint32_t)3693377612L, PH.base.pack) ;
        p82_thrust_SET((float) -1.8610883E38F, PH.base.pack) ;
        p82_body_yaw_rate_SET((float)2.700631E38F, PH.base.pack) ;
        c_TEST_Channel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_thrust_SET((float) -3.0981172E38F, PH.base.pack) ;
        p83_body_yaw_rate_SET((float)1.1375035E38F, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)258674316L, PH.base.pack) ;
        p83_body_roll_rate_SET((float)6.680417E37F, PH.base.pack) ;
        {
            float q[] =  {1.2181909E38F, 1.1662056E36F, 1.9743857E38F, -1.937355E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_body_pitch_rate_SET((float) -1.5025551E38F, PH.base.pack) ;
        p83_type_mask_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        c_CommunicationChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_yaw_SET((float)1.1360697E38F, PH.base.pack) ;
        p84_yaw_rate_SET((float)1.5120908E38F, PH.base.pack) ;
        p84_afx_SET((float)7.364943E37F, PH.base.pack) ;
        p84_vy_SET((float)2.7181593E37F, PH.base.pack) ;
        p84_vz_SET((float) -3.3795123E38F, PH.base.pack) ;
        p84_x_SET((float) -2.8884202E38F, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)1622336177L, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)62274, PH.base.pack) ;
        p84_vx_SET((float) -2.7593993E38F, PH.base.pack) ;
        p84_afz_SET((float) -3.2674424E38F, PH.base.pack) ;
        p84_y_SET((float) -2.6781979E38F, PH.base.pack) ;
        p84_afy_SET((float) -1.269506E38F, PH.base.pack) ;
        p84_z_SET((float) -2.6390472E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p86_lat_int_SET((int32_t)710505546, PH.base.pack) ;
        p86_vz_SET((float) -3.0510212E38F, PH.base.pack) ;
        p86_afz_SET((float)2.000327E38F, PH.base.pack) ;
        p86_yaw_SET((float)2.3623576E38F, PH.base.pack) ;
        p86_vy_SET((float)9.914073E37F, PH.base.pack) ;
        p86_yaw_rate_SET((float)3.3854123E37F, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)3183127443L, PH.base.pack) ;
        p86_vx_SET((float)1.4245735E38F, PH.base.pack) ;
        p86_alt_SET((float) -3.3546828E38F, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)60180, PH.base.pack) ;
        p86_afy_SET((float)1.0818347E38F, PH.base.pack) ;
        p86_lon_int_SET((int32_t) -1823943115, PH.base.pack) ;
        p86_afx_SET((float)1.3708789E38F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_lat_int_SET((int32_t) -389642569, PH.base.pack) ;
        p87_afy_SET((float)2.5851588E38F, PH.base.pack) ;
        p87_vx_SET((float)3.1858317E38F, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)2433299078L, PH.base.pack) ;
        p87_vy_SET((float) -1.9735776E38F, PH.base.pack) ;
        p87_yaw_rate_SET((float)1.2561414E38F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)24417, PH.base.pack) ;
        p87_vz_SET((float) -3.096562E38F, PH.base.pack) ;
        p87_yaw_SET((float) -1.8895488E38F, PH.base.pack) ;
        p87_afz_SET((float)3.2768819E38F, PH.base.pack) ;
        p87_afx_SET((float) -1.814004E38F, PH.base.pack) ;
        p87_lon_int_SET((int32_t)2118215883, PH.base.pack) ;
        p87_alt_SET((float)1.3030943E38F, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_roll_SET((float) -5.158722E37F, PH.base.pack) ;
        p89_y_SET((float) -2.1227453E37F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)1608700944L, PH.base.pack) ;
        p89_yaw_SET((float) -1.6769405E38F, PH.base.pack) ;
        p89_z_SET((float) -2.6036261E38F, PH.base.pack) ;
        p89_pitch_SET((float)1.3921978E38F, PH.base.pack) ;
        p89_x_SET((float)2.5181733E38F, PH.base.pack) ;
        c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
        p90_zacc_SET((int16_t)(int16_t) -9911, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t) -27403, PH.base.pack) ;
        p90_lat_SET((int32_t) -358195133, PH.base.pack) ;
        p90_rollspeed_SET((float) -2.3489805E38F, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t)24354, PH.base.pack) ;
        p90_yawspeed_SET((float)2.8519364E38F, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t)27778, PH.base.pack) ;
        p90_roll_SET((float)6.3784096E37F, PH.base.pack) ;
        p90_alt_SET((int32_t) -579698579, PH.base.pack) ;
        p90_pitchspeed_SET((float) -1.6897096E38F, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t) -32252, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)387537790346673982L, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t)20025, PH.base.pack) ;
        p90_lon_SET((int32_t) -2043320738, PH.base.pack) ;
        p90_pitch_SET((float)2.6701226E38F, PH.base.pack) ;
        p90_yaw_SET((float) -3.09333E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
        p91_yaw_rudder_SET((float) -2.5590838E36F, PH.base.pack) ;
        p91_aux2_SET((float) -4.478867E37F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_ARMED, PH.base.pack) ;
        p91_pitch_elevator_SET((float) -6.5247107E37F, PH.base.pack) ;
        p91_roll_ailerons_SET((float) -7.531352E37F, PH.base.pack) ;
        p91_aux1_SET((float) -2.4742051E38F, PH.base.pack) ;
        p91_throttle_SET((float)2.363879E38F, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)9172628609464634319L, PH.base.pack) ;
        p91_aux4_SET((float) -2.861018E38F, PH.base.pack) ;
        p91_aux3_SET((float) -2.0561053E38F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan5_raw_SET((uint16_t)(uint16_t)10696, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)45989, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)41095, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)42289, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)3319771575576279079L, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)21611, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)10329, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)10373, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)21177, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)39410, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)11219, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)24963, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)43043, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        {
            float controls[] =  {2.6321818E38F, 1.0498137E38F, -1.4393788E38F, 6.5344346E37F, 2.7724476E38F, -3.038045E38F, -7.094769E37F, 2.6067898E38F, 2.0182056E38F, 1.05245905E37F, 2.0568375E38F, -1.945764E38F, -8.834751E37F, 6.3372206E37F, -5.539322E37F, -1.2913188E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p93_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_ARMED, PH.base.pack) ;
        p93_flags_SET((uint64_t)985421710929194079L, PH.base.pack) ;
        p93_time_usec_SET((uint64_t)6979744782280549529L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_sensor_id_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        p100_flow_y_SET((int16_t)(int16_t) -5667, PH.base.pack) ;
        p100_ground_distance_SET((float) -3.3031344E37F, PH.base.pack) ;
        p100_flow_rate_x_SET((float) -1.2886359E38F, &PH) ;
        p100_flow_comp_m_x_SET((float) -7.046211E37F, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float)1.4565572E38F, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)8646101139458236348L, PH.base.pack) ;
        p100_quality_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        p100_flow_x_SET((int16_t)(int16_t) -27540, PH.base.pack) ;
        p100_flow_rate_y_SET((float)2.8538052E38F, &PH) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_y_SET((float)8.211362E37F, PH.base.pack) ;
        p101_yaw_SET((float) -3.0983038E38F, PH.base.pack) ;
        p101_z_SET((float)9.579718E37F, PH.base.pack) ;
        p101_roll_SET((float) -2.1985465E38F, PH.base.pack) ;
        p101_pitch_SET((float) -3.612081E37F, PH.base.pack) ;
        p101_x_SET((float) -4.8679963E37F, PH.base.pack) ;
        p101_usec_SET((uint64_t)5945399769847346240L, PH.base.pack) ;
        c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_yaw_SET((float) -2.9647283E38F, PH.base.pack) ;
        p102_usec_SET((uint64_t)170013366537189483L, PH.base.pack) ;
        p102_y_SET((float) -1.6518992E38F, PH.base.pack) ;
        p102_roll_SET((float)6.9460413E37F, PH.base.pack) ;
        p102_x_SET((float)1.4888121E38F, PH.base.pack) ;
        p102_z_SET((float)3.1745484E38F, PH.base.pack) ;
        p102_pitch_SET((float)2.8884784E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_z_SET((float)1.2438133E38F, PH.base.pack) ;
        p103_usec_SET((uint64_t)8738493259555063444L, PH.base.pack) ;
        p103_y_SET((float)1.0391069E38F, PH.base.pack) ;
        p103_x_SET((float)2.1407868E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_usec_SET((uint64_t)7948476928411481151L, PH.base.pack) ;
        p104_y_SET((float) -8.09822E37F, PH.base.pack) ;
        p104_yaw_SET((float) -4.341404E37F, PH.base.pack) ;
        p104_roll_SET((float)5.4264243E37F, PH.base.pack) ;
        p104_z_SET((float)1.1631432E38F, PH.base.pack) ;
        p104_pitch_SET((float) -2.8733495E38F, PH.base.pack) ;
        p104_x_SET((float) -1.660804E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
        p105_abs_pressure_SET((float) -1.7173844E38F, PH.base.pack) ;
        p105_zgyro_SET((float)2.6215091E38F, PH.base.pack) ;
        p105_zmag_SET((float) -3.9094099E37F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)21081, PH.base.pack) ;
        p105_yacc_SET((float)1.9255643E38F, PH.base.pack) ;
        p105_pressure_alt_SET((float)3.028618E38F, PH.base.pack) ;
        p105_ymag_SET((float) -5.040475E37F, PH.base.pack) ;
        p105_xgyro_SET((float) -1.2961786E38F, PH.base.pack) ;
        p105_xmag_SET((float)2.0693717E37F, PH.base.pack) ;
        p105_zacc_SET((float)3.3230278E38F, PH.base.pack) ;
        p105_ygyro_SET((float)3.3376605E38F, PH.base.pack) ;
        p105_diff_pressure_SET((float)2.8555049E38F, PH.base.pack) ;
        p105_temperature_SET((float) -1.4741927E38F, PH.base.pack) ;
        p105_xacc_SET((float)1.7553393E38F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)8148826324743800527L, PH.base.pack) ;
        c_CommunicationChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_integrated_ygyro_SET((float)2.8325784E38F, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p106_integrated_y_SET((float)9.554746E37F, PH.base.pack) ;
        p106_integrated_xgyro_SET((float)4.1873618E36F, PH.base.pack) ;
        p106_integrated_zgyro_SET((float) -2.6118916E38F, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)6780159706924010568L, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t) -7578, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)889709337L, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)3449153362L, PH.base.pack) ;
        p106_distance_SET((float)2.6345607E38F, PH.base.pack) ;
        p106_integrated_x_SET((float) -1.927174E38F, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
        p107_zacc_SET((float)1.3329283E38F, PH.base.pack) ;
        p107_abs_pressure_SET((float) -3.2593644E38F, PH.base.pack) ;
        p107_zmag_SET((float) -9.149251E36F, PH.base.pack) ;
        p107_pressure_alt_SET((float) -1.8705387E38F, PH.base.pack) ;
        p107_xgyro_SET((float) -4.874964E37F, PH.base.pack) ;
        p107_diff_pressure_SET((float) -2.9666578E38F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)3067190894L, PH.base.pack) ;
        p107_xacc_SET((float) -2.0044343E38F, PH.base.pack) ;
        p107_ymag_SET((float) -2.0957508E38F, PH.base.pack) ;
        p107_xmag_SET((float) -1.882278E38F, PH.base.pack) ;
        p107_zgyro_SET((float) -2.1724796E37F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)249636794737141773L, PH.base.pack) ;
        p107_ygyro_SET((float)2.7010493E38F, PH.base.pack) ;
        p107_temperature_SET((float) -4.833042E37F, PH.base.pack) ;
        p107_yacc_SET((float)1.920478E36F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
        p108_ygyro_SET((float)2.887184E37F, PH.base.pack) ;
        p108_alt_SET((float) -2.1091832E38F, PH.base.pack) ;
        p108_yacc_SET((float)1.2276604E37F, PH.base.pack) ;
        p108_q1_SET((float)6.6372583E37F, PH.base.pack) ;
        p108_yaw_SET((float) -2.416675E37F, PH.base.pack) ;
        p108_lat_SET((float)4.379369E37F, PH.base.pack) ;
        p108_q4_SET((float)3.6965992E37F, PH.base.pack) ;
        p108_xacc_SET((float) -1.962687E38F, PH.base.pack) ;
        p108_vn_SET((float) -1.9777714E38F, PH.base.pack) ;
        p108_lon_SET((float)4.0281608E37F, PH.base.pack) ;
        p108_q3_SET((float)2.3306018E38F, PH.base.pack) ;
        p108_zacc_SET((float)2.3991888E38F, PH.base.pack) ;
        p108_xgyro_SET((float)3.1419513E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float) -3.0286612E37F, PH.base.pack) ;
        p108_pitch_SET((float) -2.2860492E38F, PH.base.pack) ;
        p108_ve_SET((float) -4.0166026E37F, PH.base.pack) ;
        p108_std_dev_vert_SET((float)2.4856965E37F, PH.base.pack) ;
        p108_q2_SET((float) -1.1580841E38F, PH.base.pack) ;
        p108_vd_SET((float) -2.1631543E38F, PH.base.pack) ;
        p108_roll_SET((float)3.517438E37F, PH.base.pack) ;
        p108_zgyro_SET((float)5.028782E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
        p109_noise_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)56495, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)42033, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        c_CommunicationChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        {
            uint8_t payload[] =  {(uint8_t)44, (uint8_t)60, (uint8_t)247, (uint8_t)99, (uint8_t)5, (uint8_t)172, (uint8_t)60, (uint8_t)174, (uint8_t)134, (uint8_t)201, (uint8_t)160, (uint8_t)84, (uint8_t)159, (uint8_t)126, (uint8_t)129, (uint8_t)101, (uint8_t)187, (uint8_t)176, (uint8_t)210, (uint8_t)226, (uint8_t)53, (uint8_t)10, (uint8_t)236, (uint8_t)181, (uint8_t)29, (uint8_t)25, (uint8_t)146, (uint8_t)92, (uint8_t)68, (uint8_t)88, (uint8_t)103, (uint8_t)139, (uint8_t)209, (uint8_t)124, (uint8_t)70, (uint8_t)20, (uint8_t)209, (uint8_t)16, (uint8_t)211, (uint8_t)7, (uint8_t)201, (uint8_t)4, (uint8_t)150, (uint8_t)38, (uint8_t)166, (uint8_t)15, (uint8_t)81, (uint8_t)124, (uint8_t)253, (uint8_t)110, (uint8_t)166, (uint8_t)240, (uint8_t)164, (uint8_t)237, (uint8_t)129, (uint8_t)104, (uint8_t)182, (uint8_t)22, (uint8_t)40, (uint8_t)114, (uint8_t)140, (uint8_t)66, (uint8_t)92, (uint8_t)9, (uint8_t)205, (uint8_t)46, (uint8_t)189, (uint8_t)19, (uint8_t)114, (uint8_t)90, (uint8_t)10, (uint8_t)145, (uint8_t)211, (uint8_t)211, (uint8_t)45, (uint8_t)219, (uint8_t)25, (uint8_t)167, (uint8_t)11, (uint8_t)243, (uint8_t)110, (uint8_t)239, (uint8_t)101, (uint8_t)98, (uint8_t)43, (uint8_t)142, (uint8_t)119, (uint8_t)30, (uint8_t)69, (uint8_t)47, (uint8_t)4, (uint8_t)2, (uint8_t)24, (uint8_t)109, (uint8_t)209, (uint8_t)97, (uint8_t)20, (uint8_t)97, (uint8_t)172, (uint8_t)82, (uint8_t)129, (uint8_t)162, (uint8_t)21, (uint8_t)0, (uint8_t)93, (uint8_t)206, (uint8_t)166, (uint8_t)124, (uint8_t)191, (uint8_t)60, (uint8_t)168, (uint8_t)138, (uint8_t)75, (uint8_t)166, (uint8_t)136, (uint8_t)19, (uint8_t)99, (uint8_t)41, (uint8_t)175, (uint8_t)125, (uint8_t)81, (uint8_t)2, (uint8_t)158, (uint8_t)194, (uint8_t)53, (uint8_t)238, (uint8_t)240, (uint8_t)181, (uint8_t)182, (uint8_t)40, (uint8_t)139, (uint8_t)206, (uint8_t)74, (uint8_t)196, (uint8_t)112, (uint8_t)36, (uint8_t)91, (uint8_t)243, (uint8_t)32, (uint8_t)96, (uint8_t)79, (uint8_t)45, (uint8_t)115, (uint8_t)60, (uint8_t)163, (uint8_t)57, (uint8_t)82, (uint8_t)56, (uint8_t)181, (uint8_t)9, (uint8_t)104, (uint8_t)101, (uint8_t)231, (uint8_t)139, (uint8_t)58, (uint8_t)164, (uint8_t)207, (uint8_t)59, (uint8_t)8, (uint8_t)227, (uint8_t)69, (uint8_t)228, (uint8_t)28, (uint8_t)168, (uint8_t)205, (uint8_t)46, (uint8_t)113, (uint8_t)66, (uint8_t)225, (uint8_t)21, (uint8_t)128, (uint8_t)71, (uint8_t)172, (uint8_t)187, (uint8_t)189, (uint8_t)57, (uint8_t)122, (uint8_t)32, (uint8_t)102, (uint8_t)117, (uint8_t)187, (uint8_t)190, (uint8_t)186, (uint8_t)172, (uint8_t)83, (uint8_t)175, (uint8_t)228, (uint8_t)98, (uint8_t)71, (uint8_t)74, (uint8_t)249, (uint8_t)228, (uint8_t)117, (uint8_t)163, (uint8_t)232, (uint8_t)189, (uint8_t)53, (uint8_t)12, (uint8_t)184, (uint8_t)251, (uint8_t)72, (uint8_t)217, (uint8_t)59, (uint8_t)207, (uint8_t)238, (uint8_t)122, (uint8_t)32, (uint8_t)140, (uint8_t)1, (uint8_t)34, (uint8_t)185, (uint8_t)125, (uint8_t)109, (uint8_t)196, (uint8_t)62, (uint8_t)15, (uint8_t)112, (uint8_t)215, (uint8_t)20, (uint8_t)13, (uint8_t)140, (uint8_t)86, (uint8_t)53, (uint8_t)19, (uint8_t)110, (uint8_t)249, (uint8_t)235, (uint8_t)103, (uint8_t)10, (uint8_t)174, (uint8_t)25, (uint8_t)192, (uint8_t)32, (uint8_t)23, (uint8_t)171, (uint8_t)153, (uint8_t)192, (uint8_t)138, (uint8_t)4, (uint8_t)147, (uint8_t)30, (uint8_t)251, (uint8_t)171, (uint8_t)37, (uint8_t)180, (uint8_t)45, (uint8_t)38, (uint8_t)174, (uint8_t)102, (uint8_t)70, (uint8_t)40};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_system_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
        p110_target_component_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p110_target_network_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
        p111_ts1_SET((int64_t)4916714548945593517L, PH.base.pack) ;
        p111_tc1_SET((int64_t) -128638384845514173L, PH.base.pack) ;
        c_CommunicationChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_time_usec_SET((uint64_t)1059027319243958914L, PH.base.pack) ;
        p112_seq_SET((uint32_t)3668542228L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
        p113_time_usec_SET((uint64_t)8635047198574112312L, PH.base.pack) ;
        p113_lat_SET((int32_t) -270971975, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)45103, PH.base.pack) ;
        p113_lon_SET((int32_t) -1059693012, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)56873, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t) -7078, PH.base.pack) ;
        p113_alt_SET((int32_t)976425434, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t) -14880, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)1346, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t) -4987, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)41213, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_integrated_xgyro_SET((float) -2.6060503E38F, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)3304139886L, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
        p114_integrated_ygyro_SET((float)1.404539E38F, PH.base.pack) ;
        p114_distance_SET((float) -2.2260273E38F, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)2119972653L, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t) -32442, PH.base.pack) ;
        p114_integrated_zgyro_SET((float) -5.6332015E37F, PH.base.pack) ;
        p114_integrated_x_SET((float)4.0774902E37F, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        p114_integrated_y_SET((float)8.1138446E37F, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)6609154870585588291L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_rollspeed_SET((float)3.34847E38F, PH.base.pack) ;
        p115_lat_SET((int32_t)1844279122, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t)6839, PH.base.pack) ;
        p115_yawspeed_SET((float) -3.0217728E38F, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {1.1940948E38F, 1.4256272E37F, -1.5610965E38F, -1.6565432E38F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_time_usec_SET((uint64_t)2045411262067085335L, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t)19552, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t) -32561, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t) -14415, PH.base.pack) ;
        p115_lon_SET((int32_t)1203235247, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)46088, PH.base.pack) ;
        p115_alt_SET((int32_t) -1827078104, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t)12356, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)8059, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t) -14535, PH.base.pack) ;
        p115_pitchspeed_SET((float) -8.056619E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
        p116_ymag_SET((int16_t)(int16_t)28886, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t)11941, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)3937980805L, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t) -1406, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t)12729, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t) -7191, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t)10171, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t)31448, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t)17132, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t)31840, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_target_component_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        p117_end_SET((uint16_t)(uint16_t)13291, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)32321, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
        p118_num_logs_SET((uint16_t)(uint16_t)36076, PH.base.pack) ;
        p118_size_SET((uint32_t)3653099038L, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)50304, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)1912975291L, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)34169, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_target_component_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        p119_ofs_SET((uint32_t)2760488690L, PH.base.pack) ;
        p119_id_SET((uint16_t)(uint16_t)2669, PH.base.pack) ;
        p119_count_SET((uint32_t)411844098L, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
        p120_ofs_SET((uint32_t)2320009828L, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)76, (uint8_t)41, (uint8_t)139, (uint8_t)224, (uint8_t)103, (uint8_t)5, (uint8_t)197, (uint8_t)74, (uint8_t)197, (uint8_t)28, (uint8_t)239, (uint8_t)187, (uint8_t)183, (uint8_t)224, (uint8_t)115, (uint8_t)94, (uint8_t)179, (uint8_t)24, (uint8_t)15, (uint8_t)123, (uint8_t)69, (uint8_t)178, (uint8_t)147, (uint8_t)31, (uint8_t)40, (uint8_t)113, (uint8_t)154, (uint8_t)164, (uint8_t)127, (uint8_t)251, (uint8_t)148, (uint8_t)246, (uint8_t)83, (uint8_t)174, (uint8_t)93, (uint8_t)65, (uint8_t)25, (uint8_t)39, (uint8_t)153, (uint8_t)255, (uint8_t)141, (uint8_t)255, (uint8_t)199, (uint8_t)244, (uint8_t)249, (uint8_t)62, (uint8_t)178, (uint8_t)220, (uint8_t)128, (uint8_t)25, (uint8_t)120, (uint8_t)235, (uint8_t)163, (uint8_t)45, (uint8_t)80, (uint8_t)249, (uint8_t)52, (uint8_t)6, (uint8_t)168, (uint8_t)155, (uint8_t)167, (uint8_t)20, (uint8_t)92, (uint8_t)137, (uint8_t)192, (uint8_t)151, (uint8_t)144, (uint8_t)73, (uint8_t)151, (uint8_t)232, (uint8_t)169, (uint8_t)60, (uint8_t)139, (uint8_t)194, (uint8_t)109, (uint8_t)218, (uint8_t)60, (uint8_t)111, (uint8_t)253, (uint8_t)168, (uint8_t)239, (uint8_t)162, (uint8_t)24, (uint8_t)5, (uint8_t)238, (uint8_t)8, (uint8_t)182, (uint8_t)221, (uint8_t)138, (uint8_t)158};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        p120_count_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        p120_id_SET((uint16_t)(uint16_t)37970, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
        p121_target_system_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        p121_target_component_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_system_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p122_target_component_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_len_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p123_target_component_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)111, (uint8_t)132, (uint8_t)162, (uint8_t)56, (uint8_t)136, (uint8_t)238, (uint8_t)137, (uint8_t)247, (uint8_t)154, (uint8_t)6, (uint8_t)215, (uint8_t)12, (uint8_t)30, (uint8_t)171, (uint8_t)88, (uint8_t)60, (uint8_t)134, (uint8_t)219, (uint8_t)17, (uint8_t)0, (uint8_t)179, (uint8_t)127, (uint8_t)33, (uint8_t)129, (uint8_t)132, (uint8_t)210, (uint8_t)26, (uint8_t)199, (uint8_t)6, (uint8_t)242, (uint8_t)135, (uint8_t)125, (uint8_t)183, (uint8_t)123, (uint8_t)81, (uint8_t)72, (uint8_t)246, (uint8_t)237, (uint8_t)196, (uint8_t)107, (uint8_t)244, (uint8_t)72, (uint8_t)133, (uint8_t)188, (uint8_t)123, (uint8_t)39, (uint8_t)240, (uint8_t)159, (uint8_t)215, (uint8_t)122, (uint8_t)124, (uint8_t)156, (uint8_t)149, (uint8_t)171, (uint8_t)150, (uint8_t)71, (uint8_t)50, (uint8_t)246, (uint8_t)241, (uint8_t)115, (uint8_t)126, (uint8_t)71, (uint8_t)81, (uint8_t)104, (uint8_t)71, (uint8_t)209, (uint8_t)197, (uint8_t)106, (uint8_t)240, (uint8_t)3, (uint8_t)157, (uint8_t)220, (uint8_t)64, (uint8_t)184, (uint8_t)133, (uint8_t)62, (uint8_t)42, (uint8_t)203, (uint8_t)7, (uint8_t)154, (uint8_t)174, (uint8_t)36, (uint8_t)190, (uint8_t)123, (uint8_t)40, (uint8_t)180, (uint8_t)175, (uint8_t)173, (uint8_t)7, (uint8_t)19, (uint8_t)211, (uint8_t)69, (uint8_t)103, (uint8_t)41, (uint8_t)120, (uint8_t)117, (uint8_t)171, (uint8_t)22, (uint8_t)213, (uint8_t)228, (uint8_t)21, (uint8_t)7, (uint8_t)20, (uint8_t)30, (uint8_t)26, (uint8_t)74, (uint8_t)224, (uint8_t)246, (uint8_t)191, (uint8_t)98};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_target_system_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
        p124_cog_SET((uint16_t)(uint16_t)37953, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)22895, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)4947885165335678590L, PH.base.pack) ;
        p124_lat_SET((int32_t)1100312878, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)2543482038L, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)24757, PH.base.pack) ;
        p124_lon_SET((int32_t)725718205, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, PH.base.pack) ;
        p124_alt_SET((int32_t)1930187677, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)50742, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
        p125_Vservo_SET((uint16_t)(uint16_t)40050, PH.base.pack) ;
        p125_Vcc_SET((uint16_t)(uint16_t)59699, PH.base.pack) ;
        p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID, PH.base.pack) ;
        c_CommunicationChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_baudrate_SET((uint32_t)3933747905L, PH.base.pack) ;
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1, PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)13211, PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)30, (uint8_t)70, (uint8_t)228, (uint8_t)9, (uint8_t)212, (uint8_t)208, (uint8_t)160, (uint8_t)237, (uint8_t)144, (uint8_t)58, (uint8_t)191, (uint8_t)164, (uint8_t)212, (uint8_t)176, (uint8_t)74, (uint8_t)80, (uint8_t)46, (uint8_t)96, (uint8_t)48, (uint8_t)162, (uint8_t)191, (uint8_t)242, (uint8_t)69, (uint8_t)9, (uint8_t)231, (uint8_t)60, (uint8_t)172, (uint8_t)176, (uint8_t)186, (uint8_t)202, (uint8_t)139, (uint8_t)231, (uint8_t)143, (uint8_t)255, (uint8_t)59, (uint8_t)217, (uint8_t)147, (uint8_t)232, (uint8_t)237, (uint8_t)192, (uint8_t)173, (uint8_t)131, (uint8_t)240, (uint8_t)243, (uint8_t)246, (uint8_t)9, (uint8_t)15, (uint8_t)20, (uint8_t)86, (uint8_t)194, (uint8_t)212, (uint8_t)37, (uint8_t)229, (uint8_t)202, (uint8_t)247, (uint8_t)220, (uint8_t)225, (uint8_t)18, (uint8_t)31, (uint8_t)154, (uint8_t)236, (uint8_t)200, (uint8_t)160, (uint8_t)42, (uint8_t)79, (uint8_t)6, (uint8_t)194, (uint8_t)157, (uint8_t)132, (uint8_t)174};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
        p127_rtk_health_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p127_tow_SET((uint32_t)2363528971L, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t) -1092074213, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t)1510465751, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)12141, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)1006098950L, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t) -1591778888, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t)1244810690, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)3255365147L, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
        p128_wn_SET((uint16_t)(uint16_t)8130, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t)2080907122, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p128_tow_SET((uint32_t)1649351279L, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t) -2053284568, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t) -1690391046, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t)991984414, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)2049169745L, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)710285677L, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
        p129_ygyro_SET((int16_t)(int16_t)10568, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t)17746, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t)22430, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t) -24696, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t) -3048, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t)25623, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t)4905, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t)2178, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)754529265L, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t)4006, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_width_SET((uint16_t)(uint16_t)65193, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)49468, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p130_size_SET((uint32_t)3070567859L, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)38528, PH.base.pack) ;
        c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
        p131_seqnr_SET((uint16_t)(uint16_t)31268, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)36, (uint8_t)209, (uint8_t)163, (uint8_t)74, (uint8_t)242, (uint8_t)89, (uint8_t)65, (uint8_t)91, (uint8_t)192, (uint8_t)182, (uint8_t)229, (uint8_t)241, (uint8_t)233, (uint8_t)197, (uint8_t)237, (uint8_t)17, (uint8_t)139, (uint8_t)169, (uint8_t)71, (uint8_t)228, (uint8_t)159, (uint8_t)72, (uint8_t)80, (uint8_t)225, (uint8_t)140, (uint8_t)189, (uint8_t)167, (uint8_t)160, (uint8_t)73, (uint8_t)74, (uint8_t)66, (uint8_t)31, (uint8_t)183, (uint8_t)244, (uint8_t)149, (uint8_t)17, (uint8_t)182, (uint8_t)177, (uint8_t)196, (uint8_t)91, (uint8_t)125, (uint8_t)231, (uint8_t)160, (uint8_t)227, (uint8_t)118, (uint8_t)46, (uint8_t)47, (uint8_t)167, (uint8_t)194, (uint8_t)255, (uint8_t)232, (uint8_t)55, (uint8_t)155, (uint8_t)101, (uint8_t)25, (uint8_t)139, (uint8_t)97, (uint8_t)35, (uint8_t)118, (uint8_t)110, (uint8_t)116, (uint8_t)163, (uint8_t)211, (uint8_t)32, (uint8_t)41, (uint8_t)188, (uint8_t)32, (uint8_t)197, (uint8_t)34, (uint8_t)213, (uint8_t)1, (uint8_t)175, (uint8_t)237, (uint8_t)110, (uint8_t)194, (uint8_t)229, (uint8_t)63, (uint8_t)97, (uint8_t)40, (uint8_t)136, (uint8_t)86, (uint8_t)121, (uint8_t)113, (uint8_t)200, (uint8_t)0, (uint8_t)146, (uint8_t)142, (uint8_t)183, (uint8_t)19, (uint8_t)250, (uint8_t)19, (uint8_t)210, (uint8_t)5, (uint8_t)91, (uint8_t)175, (uint8_t)193, (uint8_t)218, (uint8_t)92, (uint8_t)23, (uint8_t)111, (uint8_t)63, (uint8_t)169, (uint8_t)244, (uint8_t)15, (uint8_t)184, (uint8_t)238, (uint8_t)182, (uint8_t)170, (uint8_t)9, (uint8_t)86, (uint8_t)152, (uint8_t)93, (uint8_t)65, (uint8_t)242, (uint8_t)224, (uint8_t)239, (uint8_t)130, (uint8_t)27, (uint8_t)201, (uint8_t)27, (uint8_t)232, (uint8_t)218, (uint8_t)129, (uint8_t)77, (uint8_t)169, (uint8_t)119, (uint8_t)136, (uint8_t)61, (uint8_t)188, (uint8_t)102, (uint8_t)135, (uint8_t)129, (uint8_t)198, (uint8_t)22, (uint8_t)212, (uint8_t)0, (uint8_t)110, (uint8_t)239, (uint8_t)155, (uint8_t)71, (uint8_t)23, (uint8_t)1, (uint8_t)166, (uint8_t)24, (uint8_t)119, (uint8_t)212, (uint8_t)181, (uint8_t)228, (uint8_t)10, (uint8_t)105, (uint8_t)153, (uint8_t)198, (uint8_t)212, (uint8_t)80, (uint8_t)0, (uint8_t)162, (uint8_t)183, (uint8_t)231, (uint8_t)209, (uint8_t)185, (uint8_t)43, (uint8_t)155, (uint8_t)254, (uint8_t)102, (uint8_t)13, (uint8_t)102, (uint8_t)250, (uint8_t)237, (uint8_t)75, (uint8_t)242, (uint8_t)95, (uint8_t)250, (uint8_t)162, (uint8_t)168, (uint8_t)175, (uint8_t)6, (uint8_t)238, (uint8_t)78, (uint8_t)38, (uint8_t)187, (uint8_t)21, (uint8_t)229, (uint8_t)43, (uint8_t)165, (uint8_t)145, (uint8_t)146, (uint8_t)9, (uint8_t)17, (uint8_t)58, (uint8_t)152, (uint8_t)75, (uint8_t)202, (uint8_t)217, (uint8_t)123, (uint8_t)159, (uint8_t)240, (uint8_t)110, (uint8_t)237, (uint8_t)188, (uint8_t)136, (uint8_t)172, (uint8_t)234, (uint8_t)32, (uint8_t)188, (uint8_t)190, (uint8_t)69, (uint8_t)144, (uint8_t)212, (uint8_t)152, (uint8_t)97, (uint8_t)24, (uint8_t)73, (uint8_t)65, (uint8_t)152, (uint8_t)124, (uint8_t)217, (uint8_t)171, (uint8_t)19, (uint8_t)240, (uint8_t)248, (uint8_t)73, (uint8_t)117, (uint8_t)23, (uint8_t)233, (uint8_t)238, (uint8_t)27, (uint8_t)133, (uint8_t)164, (uint8_t)197, (uint8_t)45, (uint8_t)37, (uint8_t)182, (uint8_t)185, (uint8_t)157, (uint8_t)47, (uint8_t)17, (uint8_t)54, (uint8_t)35, (uint8_t)243, (uint8_t)18, (uint8_t)161, (uint8_t)176, (uint8_t)146, (uint8_t)119, (uint8_t)191, (uint8_t)132, (uint8_t)254, (uint8_t)251, (uint8_t)56, (uint8_t)21, (uint8_t)189, (uint8_t)206, (uint8_t)34};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_id_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)3226853906L, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)56555, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)5559, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)11615, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_PITCH_270, PH.base.pack) ;
        c_CommunicationChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_grid_spacing_SET((uint16_t)(uint16_t)44027, PH.base.pack) ;
        p133_mask_SET((uint64_t)4011220607899331739L, PH.base.pack) ;
        p133_lon_SET((int32_t)1752231476, PH.base.pack) ;
        p133_lat_SET((int32_t) -428463151, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
        p134_lat_SET((int32_t)341130128, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t)9795, (int16_t) -30248, (int16_t) -26482, (int16_t)8290, (int16_t) -7581, (int16_t)8004, (int16_t)27971, (int16_t) -24591, (int16_t) -22073, (int16_t) -25459, (int16_t)27073, (int16_t)23779, (int16_t)29598, (int16_t)3495, (int16_t)6136, (int16_t)16923};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        p134_lon_SET((int32_t)1005565218, PH.base.pack) ;
        p134_grid_spacing_SET((uint16_t)(uint16_t)42959, PH.base.pack) ;
        p134_gridbit_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lat_SET((int32_t)853036083, PH.base.pack) ;
        p135_lon_SET((int32_t) -1036666439, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_spacing_SET((uint16_t)(uint16_t)16538, PH.base.pack) ;
        p136_terrain_height_SET((float)3.5356735E37F, PH.base.pack) ;
        p136_lon_SET((int32_t)910160607, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)19064, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)3936, PH.base.pack) ;
        p136_current_height_SET((float) -1.9331692E38F, PH.base.pack) ;
        p136_lat_SET((int32_t)1881058133, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_time_boot_ms_SET((uint32_t)3786747938L, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t) -6902, PH.base.pack) ;
        p137_press_diff_SET((float)3.2573836E38F, PH.base.pack) ;
        p137_press_abs_SET((float) -1.4614652E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
        p138_time_usec_SET((uint64_t)5675129796668863682L, PH.base.pack) ;
        p138_z_SET((float) -6.9965475E37F, PH.base.pack) ;
        {
            float q[] =  {-7.11112E37F, -3.462129E37F, 2.66039E38F, -1.2464332E37F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_x_SET((float)2.6010629E38F, PH.base.pack) ;
        p138_y_SET((float) -3.1576596E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        {
            float controls[] =  {-1.9381364E38F, 1.1734986E38F, 3.0235964E38F, -2.8284563E38F, 6.218146E37F, 8.898832E37F, 2.683303E37F, 9.552813E37F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_target_component_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        p139_group_mlx_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p139_target_system_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        p139_time_usec_SET((uint64_t)4604093745287409442L, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        p140_time_usec_SET((uint64_t)1919067092603424797L, PH.base.pack) ;
        {
            float controls[] =  {-1.802765E38F, -2.134553E38F, 1.5008257E38F, 1.0754278E38F, -1.1695672E38F, 3.3259298E38F, 8.938811E37F, -1.936508E38F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p140_group_mlx_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ALTITUDE_141(), &PH);
        p141_altitude_local_SET((float)2.3903715E37F, PH.base.pack) ;
        p141_altitude_amsl_SET((float) -3.3964455E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)5589314639587325708L, PH.base.pack) ;
        p141_altitude_monotonic_SET((float) -6.0291496E37F, PH.base.pack) ;
        p141_altitude_terrain_SET((float) -1.6526165E38F, PH.base.pack) ;
        p141_altitude_relative_SET((float) -9.653118E37F, PH.base.pack) ;
        p141_bottom_clearance_SET((float) -1.9085622E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RESOURCE_REQUEST_142(), &PH);
        {
            uint8_t storage[] =  {(uint8_t)68, (uint8_t)27, (uint8_t)112, (uint8_t)141, (uint8_t)29, (uint8_t)63, (uint8_t)228, (uint8_t)217, (uint8_t)140, (uint8_t)128, (uint8_t)56, (uint8_t)186, (uint8_t)230, (uint8_t)243, (uint8_t)2, (uint8_t)44, (uint8_t)249, (uint8_t)120, (uint8_t)159, (uint8_t)103, (uint8_t)134, (uint8_t)15, (uint8_t)234, (uint8_t)20, (uint8_t)24, (uint8_t)239, (uint8_t)201, (uint8_t)110, (uint8_t)14, (uint8_t)42, (uint8_t)106, (uint8_t)65, (uint8_t)2, (uint8_t)157, (uint8_t)134, (uint8_t)80, (uint8_t)99, (uint8_t)7, (uint8_t)176, (uint8_t)102, (uint8_t)164, (uint8_t)204, (uint8_t)114, (uint8_t)132, (uint8_t)153, (uint8_t)255, (uint8_t)34, (uint8_t)130, (uint8_t)188, (uint8_t)251, (uint8_t)15, (uint8_t)185, (uint8_t)67, (uint8_t)235, (uint8_t)206, (uint8_t)48, (uint8_t)97, (uint8_t)32, (uint8_t)109, (uint8_t)220, (uint8_t)255, (uint8_t)234, (uint8_t)101, (uint8_t)157, (uint8_t)110, (uint8_t)210, (uint8_t)185, (uint8_t)13, (uint8_t)236, (uint8_t)73, (uint8_t)142, (uint8_t)75, (uint8_t)110, (uint8_t)27, (uint8_t)88, (uint8_t)243, (uint8_t)26, (uint8_t)124, (uint8_t)83, (uint8_t)14, (uint8_t)184, (uint8_t)75, (uint8_t)233, (uint8_t)80, (uint8_t)88, (uint8_t)86, (uint8_t)76, (uint8_t)218, (uint8_t)51, (uint8_t)115, (uint8_t)28, (uint8_t)136, (uint8_t)89, (uint8_t)36, (uint8_t)74, (uint8_t)39, (uint8_t)179, (uint8_t)109, (uint8_t)185, (uint8_t)10, (uint8_t)51, (uint8_t)115, (uint8_t)84, (uint8_t)24, (uint8_t)119, (uint8_t)154, (uint8_t)206, (uint8_t)52, (uint8_t)16, (uint8_t)25, (uint8_t)181, (uint8_t)71, (uint8_t)207, (uint8_t)79, (uint8_t)84, (uint8_t)241, (uint8_t)108, (uint8_t)65, (uint8_t)133, (uint8_t)65};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        p142_uri_type_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        {
            uint8_t uri[] =  {(uint8_t)72, (uint8_t)33, (uint8_t)247, (uint8_t)157, (uint8_t)253, (uint8_t)9, (uint8_t)84, (uint8_t)210, (uint8_t)149, (uint8_t)101, (uint8_t)45, (uint8_t)214, (uint8_t)99, (uint8_t)112, (uint8_t)138, (uint8_t)87, (uint8_t)227, (uint8_t)127, (uint8_t)55, (uint8_t)155, (uint8_t)41, (uint8_t)41, (uint8_t)255, (uint8_t)128, (uint8_t)119, (uint8_t)143, (uint8_t)106, (uint8_t)172, (uint8_t)67, (uint8_t)148, (uint8_t)223, (uint8_t)191, (uint8_t)241, (uint8_t)216, (uint8_t)191, (uint8_t)30, (uint8_t)226, (uint8_t)6, (uint8_t)35, (uint8_t)165, (uint8_t)47, (uint8_t)106, (uint8_t)19, (uint8_t)98, (uint8_t)203, (uint8_t)86, (uint8_t)229, (uint8_t)216, (uint8_t)223, (uint8_t)222, (uint8_t)200, (uint8_t)97, (uint8_t)137, (uint8_t)184, (uint8_t)55, (uint8_t)58, (uint8_t)210, (uint8_t)15, (uint8_t)217, (uint8_t)233, (uint8_t)65, (uint8_t)21, (uint8_t)15, (uint8_t)29, (uint8_t)31, (uint8_t)21, (uint8_t)152, (uint8_t)97, (uint8_t)209, (uint8_t)243, (uint8_t)227, (uint8_t)185, (uint8_t)242, (uint8_t)110, (uint8_t)242, (uint8_t)118, (uint8_t)196, (uint8_t)251, (uint8_t)96, (uint8_t)54, (uint8_t)142, (uint8_t)41, (uint8_t)61, (uint8_t)30, (uint8_t)7, (uint8_t)28, (uint8_t)164, (uint8_t)14, (uint8_t)120, (uint8_t)135, (uint8_t)27, (uint8_t)191, (uint8_t)41, (uint8_t)124, (uint8_t)26, (uint8_t)182, (uint8_t)253, (uint8_t)233, (uint8_t)128, (uint8_t)185, (uint8_t)193, (uint8_t)30, (uint8_t)214, (uint8_t)246, (uint8_t)37, (uint8_t)57, (uint8_t)220, (uint8_t)47, (uint8_t)219, (uint8_t)121, (uint8_t)13, (uint8_t)151, (uint8_t)84, (uint8_t)90, (uint8_t)97, (uint8_t)39, (uint8_t)142, (uint8_t)128, (uint8_t)105, (uint8_t)162};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        p142_transfer_type_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p142_request_id_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        c_CommunicationChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_press_diff_SET((float)8.69019E37F, PH.base.pack) ;
        p143_temperature_SET((int16_t)(int16_t) -15956, PH.base.pack) ;
        p143_press_abs_SET((float)1.9353986E38F, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)3281673497L, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FOLLOW_TARGET_144(), &PH);
        p144_lat_SET((int32_t)2007215741, PH.base.pack) ;
        {
            float vel[] =  {-2.8366473E38F, 6.8715054E37F, 7.979309E37F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        {
            float position_cov[] =  {-2.917955E38F, -1.922365E38F, 2.893489E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        p144_alt_SET((float) -1.8723309E38F, PH.base.pack) ;
        p144_lon_SET((int32_t) -1559690753, PH.base.pack) ;
        {
            float rates[] =  {-2.9177646E38F, -1.1377091E38F, 1.0073689E38F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        p144_timestamp_SET((uint64_t)8576132673122251980L, PH.base.pack) ;
        p144_custom_state_SET((uint64_t)7383851715539542503L, PH.base.pack) ;
        {
            float attitude_q[] =  {-7.5925963E37F, -1.6584429E36F, 1.1097432E38F, 2.7858218E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        p144_est_capabilities_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        {
            float acc[] =  {-2.196119E38F, -2.4075096E38F, -3.1900952E37F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_y_pos_SET((float)1.7460863E38F, PH.base.pack) ;
        p146_z_vel_SET((float)4.027063E35F, PH.base.pack) ;
        p146_x_vel_SET((float)2.4062012E38F, PH.base.pack) ;
        p146_x_acc_SET((float)1.402545E38F, PH.base.pack) ;
        p146_x_pos_SET((float)4.6894564E37F, PH.base.pack) ;
        p146_pitch_rate_SET((float) -1.2987006E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float)1.1404376E38F, PH.base.pack) ;
        p146_z_pos_SET((float) -1.0180164E38F, PH.base.pack) ;
        p146_time_usec_SET((uint64_t)5714096563057014722L, PH.base.pack) ;
        p146_y_acc_SET((float) -4.33816E37F, PH.base.pack) ;
        p146_airspeed_SET((float) -3.2526973E38F, PH.base.pack) ;
        p146_roll_rate_SET((float) -1.727649E38F, PH.base.pack) ;
        {
            float q[] =  {9.801902E37F, 1.3628309E38F, 1.3779298E38F, 1.4211097E38F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_y_vel_SET((float) -4.0349073E37F, PH.base.pack) ;
        {
            float pos_variance[] =  {-1.4510498E38F, 3.3597102E38F, -8.3140655E37F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        {
            float vel_variance[] =  {-2.2624754E38F, 1.3501348E38F, -1.7365069E38F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_z_acc_SET((float)4.7014898E36F, PH.base.pack) ;
        c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_BATTERY_STATUS_147(), &PH);
        p147_temperature_SET((int16_t)(int16_t) -29449, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_NIMH, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t)1171068235, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t)34, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_AVIONICS, PH.base.pack) ;
        p147_current_consumed_SET((int32_t)1627149141, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t)10893, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)26262, (uint16_t)42414, (uint16_t)1470, (uint16_t)37269, (uint16_t)14944, (uint16_t)30427, (uint16_t)32132, (uint16_t)44142, (uint16_t)9703, (uint16_t)55967};
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
            uint8_t uid2[] =  {(uint8_t)86, (uint8_t)108, (uint8_t)85, (uint8_t)134, (uint8_t)17, (uint8_t)176, (uint8_t)145, (uint8_t)243, (uint8_t)67, (uint8_t)176, (uint8_t)251, (uint8_t)39, (uint8_t)147, (uint8_t)71, (uint8_t)19, (uint8_t)139, (uint8_t)22, (uint8_t)19};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_product_id_SET((uint16_t)(uint16_t)17176, PH.base.pack) ;
        p148_uid_SET((uint64_t)419737349904552244L, PH.base.pack) ;
        p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMMAND_INT, PH.base.pack) ;
        p148_middleware_sw_version_SET((uint32_t)2036651191L, PH.base.pack) ;
        p148_os_sw_version_SET((uint32_t)999026390L, PH.base.pack) ;
        p148_board_version_SET((uint32_t)2156716582L, PH.base.pack) ;
        p148_vendor_id_SET((uint16_t)(uint16_t)18578, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)82, (uint8_t)150, (uint8_t)222, (uint8_t)169, (uint8_t)33, (uint8_t)82, (uint8_t)26, (uint8_t)140};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)210, (uint8_t)254, (uint8_t)167, (uint8_t)169, (uint8_t)10, (uint8_t)239, (uint8_t)190, (uint8_t)4};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t flight_custom_version[] =  {(uint8_t)231, (uint8_t)40, (uint8_t)197, (uint8_t)223, (uint8_t)172, (uint8_t)218, (uint8_t)187, (uint8_t)172};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        p148_flight_sw_version_SET((uint32_t)1244599946L, PH.base.pack) ;
        c_CommunicationChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LANDING_TARGET_149(), &PH);
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON, PH.base.pack) ;
        p149_target_num_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        p149_size_y_SET((float) -1.7657908E38F, PH.base.pack) ;
        p149_position_valid_SET((uint8_t)(uint8_t)61, &PH) ;
        p149_angle_y_SET((float) -1.023486E38F, PH.base.pack) ;
        p149_size_x_SET((float)1.3411605E37F, PH.base.pack) ;
        {
            float q[] =  {-7.116993E37F, -3.1898282E37F, 1.2596789E38F, -8.2611953E37F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_distance_SET((float)1.1510844E38F, PH.base.pack) ;
        p149_z_SET((float)2.9882285E38F, &PH) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p149_time_usec_SET((uint64_t)8955228839679415102L, PH.base.pack) ;
        p149_angle_x_SET((float)2.3728807E38F, PH.base.pack) ;
        p149_x_SET((float) -2.81798E38F, &PH) ;
        p149_y_SET((float)1.838419E38F, &PH) ;
        c_CommunicationChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CPU_LOAD_170(), &PH);
        p170_batVolt_SET((uint16_t)(uint16_t)55805, PH.base.pack) ;
        p170_ctrlLoad_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
        p170_sensLoad_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        c_CommunicationChannel_on_CPU_LOAD_170(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENSOR_BIAS_172(), &PH);
        p172_axBias_SET((float) -3.0555962E37F, PH.base.pack) ;
        p172_gyBias_SET((float)1.7848987E37F, PH.base.pack) ;
        p172_gzBias_SET((float)2.8582787E38F, PH.base.pack) ;
        p172_gxBias_SET((float) -7.0239754E37F, PH.base.pack) ;
        p172_ayBias_SET((float) -1.5665718E38F, PH.base.pack) ;
        p172_azBias_SET((float) -2.4359853E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SENSOR_BIAS_172(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DIAGNOSTIC_173(), &PH);
        p173_diagSh1_SET((int16_t)(int16_t)11008, PH.base.pack) ;
        p173_diagSh2_SET((int16_t)(int16_t)17434, PH.base.pack) ;
        p173_diagFl2_SET((float)1.0211169E38F, PH.base.pack) ;
        p173_diagFl1_SET((float)2.9546222E38F, PH.base.pack) ;
        p173_diagSh3_SET((int16_t)(int16_t)11064, PH.base.pack) ;
        p173_diagFl3_SET((float) -2.7263477E38F, PH.base.pack) ;
        c_CommunicationChannel_on_DIAGNOSTIC_173(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SLUGS_NAVIGATION_176(), &PH);
        p176_theta_c_SET((float) -9.433098E37F, PH.base.pack) ;
        p176_ay_body_SET((float)1.3837325E38F, PH.base.pack) ;
        p176_phi_c_SET((float)6.130505E37F, PH.base.pack) ;
        p176_psiDot_c_SET((float) -2.0685062E38F, PH.base.pack) ;
        p176_fromWP_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p176_h_c_SET((uint16_t)(uint16_t)21183, PH.base.pack) ;
        p176_dist2Go_SET((float) -1.985682E38F, PH.base.pack) ;
        p176_u_m_SET((float)3.3250893E38F, PH.base.pack) ;
        p176_totalDist_SET((float) -8.187116E37F, PH.base.pack) ;
        p176_toWP_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
        c_CommunicationChannel_on_SLUGS_NAVIGATION_176(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DATA_LOG_177(), &PH);
        p177_fl_5_SET((float)3.01179E38F, PH.base.pack) ;
        p177_fl_1_SET((float)1.4793822E37F, PH.base.pack) ;
        p177_fl_4_SET((float) -3.6326934E37F, PH.base.pack) ;
        p177_fl_3_SET((float)6.603759E37F, PH.base.pack) ;
        p177_fl_2_SET((float) -2.9747788E38F, PH.base.pack) ;
        p177_fl_6_SET((float)2.0990894E38F, PH.base.pack) ;
        c_CommunicationChannel_on_DATA_LOG_177(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_DATE_TIME_179(), &PH);
        p179_GppGl_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p179_hour_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        p179_min_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        p179_visSat_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p179_useSat_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        p179_sec_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p179_sigUsedMask_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p179_clockStat_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p179_month_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        p179_year_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p179_day_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        p179_percentUsed_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_DATE_TIME_179(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MID_LVL_CMDS_180(), &PH);
        p180_hCommand_SET((float) -2.2030191E38F, PH.base.pack) ;
        p180_target_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p180_rCommand_SET((float) -1.6351938E38F, PH.base.pack) ;
        p180_uCommand_SET((float)3.2971724E38F, PH.base.pack) ;
        c_CommunicationChannel_on_MID_LVL_CMDS_180(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CTRL_SRFC_PT_181(), &PH);
        p181_bitfieldPt_SET((uint16_t)(uint16_t)11598, PH.base.pack) ;
        p181_target_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
        c_CommunicationChannel_on_CTRL_SRFC_PT_181(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SLUGS_CAMERA_ORDER_184(), &PH);
        p184_zoom_SET((int8_t)(int8_t)116, PH.base.pack) ;
        p184_moveHome_SET((int8_t)(int8_t)59, PH.base.pack) ;
        p184_target_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p184_pan_SET((int8_t)(int8_t) -117, PH.base.pack) ;
        p184_tilt_SET((int8_t)(int8_t) -46, PH.base.pack) ;
        c_CommunicationChannel_on_SLUGS_CAMERA_ORDER_184(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CONTROL_SURFACE_185(), &PH);
        p185_bControl_SET((float) -1.1569067E38F, PH.base.pack) ;
        p185_idSurface_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p185_target_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        p185_mControl_SET((float) -2.5702912E38F, PH.base.pack) ;
        c_CommunicationChannel_on_CONTROL_SURFACE_185(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SLUGS_MOBILE_LOCATION_186(), &PH);
        p186_target_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
        p186_latitude_SET((float)1.9845827E38F, PH.base.pack) ;
        p186_longitude_SET((float)2.88424E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SLUGS_MOBILE_LOCATION_186(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SLUGS_CONFIGURATION_CAMERA_188(), &PH);
        p188_order_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p188_target_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p188_idOrder_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        c_CommunicationChannel_on_SLUGS_CONFIGURATION_CAMERA_188(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ISR_LOCATION_189(), &PH);
        p189_longitude_SET((float) -6.488452E37F, PH.base.pack) ;
        p189_option2_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        p189_target_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p189_height_SET((float) -1.614351E38F, PH.base.pack) ;
        p189_option3_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p189_option1_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        p189_latitude_SET((float) -2.1722061E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ISR_LOCATION_189(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VOLT_SENSOR_191(), &PH);
        p191_r2Type_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p191_reading2_SET((uint16_t)(uint16_t)55310, PH.base.pack) ;
        p191_voltage_SET((uint16_t)(uint16_t)31618, PH.base.pack) ;
        c_CommunicationChannel_on_VOLT_SENSOR_191(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PTZ_STATUS_192(), &PH);
        p192_pan_SET((int16_t)(int16_t)5611, PH.base.pack) ;
        p192_zoom_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        p192_tilt_SET((int16_t)(int16_t) -210, PH.base.pack) ;
        c_CommunicationChannel_on_PTZ_STATUS_192(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAV_STATUS_193(), &PH);
        p193_altitude_SET((float)2.1865462E37F, PH.base.pack) ;
        p193_latitude_SET((float)1.6850917E38F, PH.base.pack) ;
        p193_course_SET((float) -2.7996264E38F, PH.base.pack) ;
        p193_longitude_SET((float)2.8295134E38F, PH.base.pack) ;
        p193_speed_SET((float)7.871292E37F, PH.base.pack) ;
        p193_target_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        c_CommunicationChannel_on_UAV_STATUS_193(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STATUS_GPS_194(), &PH);
        p194_csFails_SET((uint16_t)(uint16_t)35257, PH.base.pack) ;
        p194_modeInd_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        p194_msgsType_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p194_gpsQuality_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        p194_magVar_SET((float)1.3370784E38F, PH.base.pack) ;
        p194_magDir_SET((int8_t)(int8_t)49, PH.base.pack) ;
        p194_posStatus_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        c_CommunicationChannel_on_STATUS_GPS_194(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NOVATEL_DIAG_195(), &PH);
        p195_timeStatus_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        p195_posType_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        p195_receiverStatus_SET((uint32_t)3950238385L, PH.base.pack) ;
        p195_velType_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        p195_csFails_SET((uint16_t)(uint16_t)48948, PH.base.pack) ;
        p195_posSolAge_SET((float) -3.3470906E38F, PH.base.pack) ;
        p195_solStatus_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        c_CommunicationChannel_on_NOVATEL_DIAG_195(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENSOR_DIAG_196(), &PH);
        p196_float2_SET((float) -1.826724E38F, PH.base.pack) ;
        p196_char1_SET((int8_t)(int8_t) -114, PH.base.pack) ;
        p196_int1_SET((int16_t)(int16_t)9716, PH.base.pack) ;
        p196_float1_SET((float) -2.521057E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SENSOR_DIAG_196(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BOOT_197(), &PH);
        p197_version_SET((uint32_t)1924896412L, PH.base.pack) ;
        c_CommunicationChannel_on_BOOT_197(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_tas_ratio_SET((float)2.6487085E38F, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float)2.1806057E38F, PH.base.pack) ;
        p230_vel_ratio_SET((float) -2.6040074E37F, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float)9.397331E37F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)9016035822336908008L, PH.base.pack) ;
        p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float) -1.3994759E38F, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float)3.293159E38F, PH.base.pack) ;
        p230_hagl_ratio_SET((float) -7.068972E37F, PH.base.pack) ;
        p230_mag_ratio_SET((float) -1.1688565E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_WIND_COV_231(), &PH);
        p231_wind_z_SET((float)2.645964E38F, PH.base.pack) ;
        p231_wind_x_SET((float)1.1410627E38F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)1860902660178408629L, PH.base.pack) ;
        p231_horiz_accuracy_SET((float) -5.756607E37F, PH.base.pack) ;
        p231_var_horiz_SET((float)2.835734E38F, PH.base.pack) ;
        p231_wind_y_SET((float)2.2023145E38F, PH.base.pack) ;
        p231_vert_accuracy_SET((float)2.2368912E38F, PH.base.pack) ;
        p231_var_vert_SET((float) -1.4892463E38F, PH.base.pack) ;
        p231_wind_alt_SET((float)1.534516E37F, PH.base.pack) ;
        c_CommunicationChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_INPUT_232(), &PH);
        p232_lat_SET((int32_t) -867140556, PH.base.pack) ;
        p232_lon_SET((int32_t) -293375177, PH.base.pack) ;
        p232_vd_SET((float) -7.3433554E37F, PH.base.pack) ;
        p232_vert_accuracy_SET((float)1.0167076E38F, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)2098069297L, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)1842183281547945249L, PH.base.pack) ;
        p232_vn_SET((float)8.91452E37F, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        p232_speed_accuracy_SET((float) -7.332238E37F, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        p232_horiz_accuracy_SET((float)2.7029637E38F, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)36343, PH.base.pack) ;
        p232_hdop_SET((float) -1.7353015E38F, PH.base.pack) ;
        p232_ve_SET((float) -1.0462465E37F, PH.base.pack) ;
        p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY, PH.base.pack) ;
        p232_vdop_SET((float)8.879837E37F, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        p232_alt_SET((float) -2.0068278E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_RTCM_DATA_233(), &PH);
        p233_len_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)40, (uint8_t)119, (uint8_t)153, (uint8_t)249, (uint8_t)158, (uint8_t)252, (uint8_t)185, (uint8_t)238, (uint8_t)201, (uint8_t)50, (uint8_t)168, (uint8_t)213, (uint8_t)140, (uint8_t)7, (uint8_t)65, (uint8_t)187, (uint8_t)78, (uint8_t)98, (uint8_t)227, (uint8_t)208, (uint8_t)135, (uint8_t)251, (uint8_t)121, (uint8_t)139, (uint8_t)249, (uint8_t)101, (uint8_t)11, (uint8_t)59, (uint8_t)188, (uint8_t)22, (uint8_t)157, (uint8_t)67, (uint8_t)240, (uint8_t)86, (uint8_t)229, (uint8_t)234, (uint8_t)32, (uint8_t)98, (uint8_t)19, (uint8_t)177, (uint8_t)136, (uint8_t)71, (uint8_t)177, (uint8_t)211, (uint8_t)193, (uint8_t)104, (uint8_t)64, (uint8_t)124, (uint8_t)223, (uint8_t)255, (uint8_t)7, (uint8_t)129, (uint8_t)18, (uint8_t)162, (uint8_t)185, (uint8_t)193, (uint8_t)70, (uint8_t)122, (uint8_t)219, (uint8_t)57, (uint8_t)96, (uint8_t)64, (uint8_t)43, (uint8_t)171, (uint8_t)226, (uint8_t)211, (uint8_t)62, (uint8_t)84, (uint8_t)101, (uint8_t)193, (uint8_t)169, (uint8_t)115, (uint8_t)18, (uint8_t)236, (uint8_t)209, (uint8_t)53, (uint8_t)186, (uint8_t)254, (uint8_t)171, (uint8_t)21, (uint8_t)11, (uint8_t)235, (uint8_t)92, (uint8_t)187, (uint8_t)60, (uint8_t)119, (uint8_t)128, (uint8_t)39, (uint8_t)28, (uint8_t)106, (uint8_t)147, (uint8_t)196, (uint8_t)108, (uint8_t)83, (uint8_t)192, (uint8_t)155, (uint8_t)29, (uint8_t)37, (uint8_t)116, (uint8_t)3, (uint8_t)156, (uint8_t)235, (uint8_t)144, (uint8_t)197, (uint8_t)160, (uint8_t)87, (uint8_t)59, (uint8_t)74, (uint8_t)13, (uint8_t)180, (uint8_t)192, (uint8_t)25, (uint8_t)254, (uint8_t)107, (uint8_t)253, (uint8_t)214, (uint8_t)126, (uint8_t)10, (uint8_t)122, (uint8_t)147, (uint8_t)169, (uint8_t)135, (uint8_t)190, (uint8_t)37, (uint8_t)144, (uint8_t)79, (uint8_t)249, (uint8_t)239, (uint8_t)180, (uint8_t)182, (uint8_t)49, (uint8_t)232, (uint8_t)48, (uint8_t)2, (uint8_t)145, (uint8_t)250, (uint8_t)142, (uint8_t)44, (uint8_t)160, (uint8_t)36, (uint8_t)173, (uint8_t)190, (uint8_t)167, (uint8_t)111, (uint8_t)252, (uint8_t)161, (uint8_t)152, (uint8_t)112, (uint8_t)219, (uint8_t)123, (uint8_t)254, (uint8_t)2, (uint8_t)39, (uint8_t)135, (uint8_t)191, (uint8_t)170, (uint8_t)134, (uint8_t)228, (uint8_t)151, (uint8_t)2, (uint8_t)33, (uint8_t)174, (uint8_t)219, (uint8_t)196, (uint8_t)208, (uint8_t)101, (uint8_t)114, (uint8_t)37, (uint8_t)239, (uint8_t)238, (uint8_t)151, (uint8_t)22, (uint8_t)191, (uint8_t)185, (uint8_t)134, (uint8_t)25, (uint8_t)119, (uint8_t)252, (uint8_t)243, (uint8_t)93};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        p233_flags_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HIGH_LATENCY_234(), &PH);
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t) -19584, PH.base.pack) ;
        p234_longitude_SET((int32_t)931826897, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)3860322367L, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)63951, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t) -11, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t)21, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t) -86, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t) -26663, PH.base.pack) ;
        p234_latitude_SET((int32_t)1528444106, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)12798, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t)18563, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t) -21071, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t) -19666, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t)38, PH.base.pack) ;
        c_CommunicationChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIBRATION_241(), &PH);
        p241_time_usec_SET((uint64_t)3593046151023791440L, PH.base.pack) ;
        p241_vibration_x_SET((float)4.6654734E37F, PH.base.pack) ;
        p241_vibration_y_SET((float)7.753509E37F, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)1583377295L, PH.base.pack) ;
        p241_vibration_z_SET((float)2.0423806E37F, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)2781430278L, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)1008385182L, PH.base.pack) ;
        c_CommunicationChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HOME_POSITION_242(), &PH);
        p242_x_SET((float)9.265387E37F, PH.base.pack) ;
        p242_y_SET((float) -3.1015451E38F, PH.base.pack) ;
        p242_longitude_SET((int32_t)963449330, PH.base.pack) ;
        p242_approach_y_SET((float)1.2632916E38F, PH.base.pack) ;
        p242_z_SET((float) -3.4373223E37F, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)5467615960283793491L, &PH) ;
        p242_latitude_SET((int32_t)632377346, PH.base.pack) ;
        p242_approach_z_SET((float)2.6984574E38F, PH.base.pack) ;
        p242_altitude_SET((int32_t)822957352, PH.base.pack) ;
        {
            float q[] =  {-1.920426E38F, 6.608992E37F, -1.4367643E37F, -2.3323621E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_approach_x_SET((float)2.2309111E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_HOME_POSITION_243(), &PH);
        p243_time_usec_SET((uint64_t)5893772108496882268L, &PH) ;
        p243_target_system_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
        p243_longitude_SET((int32_t) -802307195, PH.base.pack) ;
        {
            float q[] =  {-5.201974E37F, -1.4145055E36F, 1.8726978E38F, 1.5689935E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_z_SET((float) -1.7924746E38F, PH.base.pack) ;
        p243_approach_y_SET((float)6.9206147E37F, PH.base.pack) ;
        p243_approach_z_SET((float) -1.2722111E38F, PH.base.pack) ;
        p243_y_SET((float) -7.735065E37F, PH.base.pack) ;
        p243_latitude_SET((int32_t)588737200, PH.base.pack) ;
        p243_altitude_SET((int32_t) -1135089362, PH.base.pack) ;
        p243_approach_x_SET((float)1.0280655E38F, PH.base.pack) ;
        p243_x_SET((float)3.1633691E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_interval_us_SET((int32_t) -1844457700, PH.base.pack) ;
        p244_message_id_SET((uint16_t)(uint16_t)59372, PH.base.pack) ;
        c_CommunicationChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, PH.base.pack) ;
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_FW, PH.base.pack) ;
        c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ADSB_VEHICLE_246(), &PH);
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)46756, PH.base.pack) ;
        p246_lat_SET((int32_t)1069719980, PH.base.pack) ;
        p246_lon_SET((int32_t) -1560081905, PH.base.pack) ;
        p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t) -8629, PH.base.pack) ;
        {
            char16_t* callsign = u"aimJlrmj";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LIGHT, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)2973159695L, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p246_altitude_SET((int32_t)9255270, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)10450, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)20202, PH.base.pack) ;
        c_CommunicationChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_COLLISION_247(), &PH);
        p247_horizontal_minimum_delta_SET((float)3.0246385E38F, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_HOVER, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, PH.base.pack) ;
        p247_id_SET((uint32_t)1713720901L, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float)1.7954364E38F, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float)2.5633309E38F, PH.base.pack) ;
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, PH.base.pack) ;
        c_CommunicationChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_V2_EXTENSION_248(), &PH);
        p248_target_system_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        p248_target_network_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
        p248_target_component_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
        p248_message_type_SET((uint16_t)(uint16_t)55048, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)39, (uint8_t)180, (uint8_t)244, (uint8_t)193, (uint8_t)255, (uint8_t)103, (uint8_t)46, (uint8_t)255, (uint8_t)27, (uint8_t)236, (uint8_t)227, (uint8_t)171, (uint8_t)175, (uint8_t)67, (uint8_t)203, (uint8_t)43, (uint8_t)58, (uint8_t)105, (uint8_t)103, (uint8_t)226, (uint8_t)45, (uint8_t)172, (uint8_t)14, (uint8_t)234, (uint8_t)27, (uint8_t)78, (uint8_t)12, (uint8_t)148, (uint8_t)71, (uint8_t)171, (uint8_t)238, (uint8_t)59, (uint8_t)48, (uint8_t)87, (uint8_t)87, (uint8_t)82, (uint8_t)51, (uint8_t)4, (uint8_t)170, (uint8_t)199, (uint8_t)222, (uint8_t)128, (uint8_t)8, (uint8_t)13, (uint8_t)253, (uint8_t)51, (uint8_t)187, (uint8_t)82, (uint8_t)124, (uint8_t)186, (uint8_t)206, (uint8_t)216, (uint8_t)244, (uint8_t)227, (uint8_t)248, (uint8_t)197, (uint8_t)179, (uint8_t)171, (uint8_t)242, (uint8_t)13, (uint8_t)35, (uint8_t)104, (uint8_t)212, (uint8_t)138, (uint8_t)114, (uint8_t)157, (uint8_t)217, (uint8_t)133, (uint8_t)172, (uint8_t)106, (uint8_t)43, (uint8_t)143, (uint8_t)233, (uint8_t)208, (uint8_t)52, (uint8_t)155, (uint8_t)52, (uint8_t)183, (uint8_t)201, (uint8_t)62, (uint8_t)39, (uint8_t)4, (uint8_t)114, (uint8_t)124, (uint8_t)60, (uint8_t)164, (uint8_t)208, (uint8_t)245, (uint8_t)135, (uint8_t)49, (uint8_t)188, (uint8_t)15, (uint8_t)182, (uint8_t)78, (uint8_t)141, (uint8_t)16, (uint8_t)14, (uint8_t)37, (uint8_t)16, (uint8_t)245, (uint8_t)77, (uint8_t)19, (uint8_t)211, (uint8_t)87, (uint8_t)225, (uint8_t)117, (uint8_t)231, (uint8_t)198, (uint8_t)1, (uint8_t)127, (uint8_t)175, (uint8_t)98, (uint8_t)212, (uint8_t)0, (uint8_t)185, (uint8_t)100, (uint8_t)54, (uint8_t)214, (uint8_t)166, (uint8_t)125, (uint8_t)140, (uint8_t)148, (uint8_t)221, (uint8_t)141, (uint8_t)86, (uint8_t)149, (uint8_t)207, (uint8_t)241, (uint8_t)101, (uint8_t)228, (uint8_t)61, (uint8_t)60, (uint8_t)19, (uint8_t)152, (uint8_t)252, (uint8_t)116, (uint8_t)244, (uint8_t)203, (uint8_t)173, (uint8_t)248, (uint8_t)3, (uint8_t)157, (uint8_t)110, (uint8_t)94, (uint8_t)139, (uint8_t)116, (uint8_t)215, (uint8_t)7, (uint8_t)196, (uint8_t)47, (uint8_t)97, (uint8_t)172, (uint8_t)163, (uint8_t)99, (uint8_t)29, (uint8_t)211, (uint8_t)9, (uint8_t)218, (uint8_t)40, (uint8_t)220, (uint8_t)63, (uint8_t)245, (uint8_t)10, (uint8_t)59, (uint8_t)215, (uint8_t)101, (uint8_t)223, (uint8_t)106, (uint8_t)36, (uint8_t)227, (uint8_t)83, (uint8_t)255, (uint8_t)254, (uint8_t)87, (uint8_t)225, (uint8_t)71, (uint8_t)65, (uint8_t)210, (uint8_t)204, (uint8_t)124, (uint8_t)16, (uint8_t)174, (uint8_t)47, (uint8_t)254, (uint8_t)73, (uint8_t)93, (uint8_t)166, (uint8_t)209, (uint8_t)19, (uint8_t)121, (uint8_t)190, (uint8_t)85, (uint8_t)21, (uint8_t)171, (uint8_t)127, (uint8_t)201, (uint8_t)248, (uint8_t)147, (uint8_t)155, (uint8_t)100, (uint8_t)217, (uint8_t)109, (uint8_t)164, (uint8_t)244, (uint8_t)41, (uint8_t)78, (uint8_t)87, (uint8_t)116, (uint8_t)241, (uint8_t)138, (uint8_t)208, (uint8_t)226, (uint8_t)42, (uint8_t)81, (uint8_t)105, (uint8_t)41, (uint8_t)146, (uint8_t)86, (uint8_t)60, (uint8_t)144, (uint8_t)212, (uint8_t)150, (uint8_t)164, (uint8_t)226, (uint8_t)200, (uint8_t)190, (uint8_t)55, (uint8_t)65, (uint8_t)250, (uint8_t)115, (uint8_t)35, (uint8_t)161, (uint8_t)200, (uint8_t)44, (uint8_t)158, (uint8_t)51, (uint8_t)53, (uint8_t)189, (uint8_t)230, (uint8_t)231, (uint8_t)138, (uint8_t)179, (uint8_t)95, (uint8_t)235, (uint8_t)15, (uint8_t)186, (uint8_t)206, (uint8_t)209, (uint8_t)72};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MEMORY_VECT_249(), &PH);
        p249_ver_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        p249_address_SET((uint16_t)(uint16_t)50113, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t)4, (int8_t)93, (int8_t) -122, (int8_t) -32, (int8_t) -25, (int8_t)4, (int8_t) -97, (int8_t) -127, (int8_t)75, (int8_t)21, (int8_t)11, (int8_t)46, (int8_t)52, (int8_t) -61, (int8_t) -98, (int8_t)97, (int8_t) -49, (int8_t) -83, (int8_t) -72, (int8_t)122, (int8_t)43, (int8_t) -114, (int8_t) -72, (int8_t)11, (int8_t)32, (int8_t) -100, (int8_t) -103, (int8_t) -106, (int8_t)64, (int8_t)117, (int8_t)45, (int8_t) -2};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_type_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        c_CommunicationChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_VECT_250(), &PH);
        p250_time_usec_SET((uint64_t)5470191345128341017L, PH.base.pack) ;
        p250_x_SET((float)2.7926489E38F, PH.base.pack) ;
        p250_y_SET((float) -1.4080476E38F, PH.base.pack) ;
        p250_z_SET((float) -1.8162097E38F, PH.base.pack) ;
        {
            char16_t* name = u"dwqzh";
            p250_name_SET_(name, &PH) ;
        }
        c_CommunicationChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_FLOAT_251(), &PH);
        p251_value_SET((float) -2.7340597E38F, PH.base.pack) ;
        {
            char16_t* name = u"mbtd";
            p251_name_SET_(name, &PH) ;
        }
        p251_time_boot_ms_SET((uint32_t)2889016376L, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_INT_252(), &PH);
        p252_value_SET((int32_t) -18282385, PH.base.pack) ;
        p252_time_boot_ms_SET((uint32_t)2685674959L, PH.base.pack) ;
        {
            char16_t* name = u"qjnojn";
            p252_name_SET_(name, &PH) ;
        }
        c_CommunicationChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STATUSTEXT_253(), &PH);
        {
            char16_t* text = u"laogFtzudckrlykqlBgyNdXarSjxac";
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
        p254_time_boot_ms_SET((uint32_t)1831354327L, PH.base.pack) ;
        p254_ind_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        p254_value_SET((float)8.0432273E37F, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SETUP_SIGNING_256(), &PH);
        p256_initial_timestamp_SET((uint64_t)1516055848792011493L, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)85, (uint8_t)135, (uint8_t)197, (uint8_t)113, (uint8_t)104, (uint8_t)142, (uint8_t)17, (uint8_t)174, (uint8_t)93, (uint8_t)145, (uint8_t)229, (uint8_t)195, (uint8_t)70, (uint8_t)32, (uint8_t)39, (uint8_t)236, (uint8_t)154, (uint8_t)251, (uint8_t)255, (uint8_t)156, (uint8_t)31, (uint8_t)109, (uint8_t)42, (uint8_t)87, (uint8_t)138, (uint8_t)165, (uint8_t)204, (uint8_t)179, (uint8_t)141, (uint8_t)172, (uint8_t)23, (uint8_t)112};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        p256_target_component_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        p256_target_system_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        c_CommunicationChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BUTTON_CHANGE_257(), &PH);
        p257_last_change_ms_SET((uint32_t)449390083L, PH.base.pack) ;
        p257_state_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        p257_time_boot_ms_SET((uint32_t)1893394770L, PH.base.pack) ;
        c_CommunicationChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PLAY_TUNE_258(), &PH);
        {
            char16_t* tune = u"ykayaQwhTfqmmvdcjre";
            p258_tune_SET_(tune, &PH) ;
        }
        p258_target_component_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        p258_target_system_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        c_CommunicationChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_INFORMATION_259(), &PH);
        p259_sensor_size_h_SET((float)5.6627737E37F, PH.base.pack) ;
        p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)154, (uint8_t)247, (uint8_t)177, (uint8_t)33, (uint8_t)32, (uint8_t)182, (uint8_t)176, (uint8_t)172, (uint8_t)94, (uint8_t)198, (uint8_t)28, (uint8_t)87, (uint8_t)242, (uint8_t)255, (uint8_t)62, (uint8_t)243, (uint8_t)216, (uint8_t)215, (uint8_t)64, (uint8_t)150, (uint8_t)223, (uint8_t)171, (uint8_t)79, (uint8_t)143, (uint8_t)112, (uint8_t)187, (uint8_t)114, (uint8_t)165, (uint8_t)182, (uint8_t)25, (uint8_t)5, (uint8_t)26};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_firmware_version_SET((uint32_t)2371953817L, PH.base.pack) ;
        p259_focal_length_SET((float)2.7059716E38F, PH.base.pack) ;
        p259_sensor_size_v_SET((float) -1.8291476E38F, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"lhprWhrfjgtSckihdumtfdvHongIsyjgfzrkYgooadkfrxvyimsuujcejnyjgseIwuzuvsuqilptnuneVfvfsvbs";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_lens_id_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        p259_time_boot_ms_SET((uint32_t)3098976481L, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)644, PH.base.pack) ;
        p259_resolution_v_SET((uint16_t)(uint16_t)11847, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)6, (uint8_t)80, (uint8_t)56, (uint8_t)234, (uint8_t)1, (uint8_t)12, (uint8_t)212, (uint8_t)47, (uint8_t)176, (uint8_t)116, (uint8_t)135, (uint8_t)240, (uint8_t)158, (uint8_t)161, (uint8_t)47, (uint8_t)170, (uint8_t)157, (uint8_t)34, (uint8_t)221, (uint8_t)56, (uint8_t)193, (uint8_t)79, (uint8_t)176, (uint8_t)18, (uint8_t)224, (uint8_t)200, (uint8_t)121, (uint8_t)219, (uint8_t)132, (uint8_t)101, (uint8_t)35, (uint8_t)37};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_cam_definition_version_SET((uint16_t)(uint16_t)64568, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_SETTINGS_260(), &PH);
        p260_time_boot_ms_SET((uint32_t)1861889313L, PH.base.pack) ;
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STORAGE_INFORMATION_261(), &PH);
        p261_used_capacity_SET((float)3.377777E38F, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p261_total_capacity_SET((float) -3.139671E38F, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)3583965930L, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        p261_write_speed_SET((float) -3.0930514E38F, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p261_read_speed_SET((float)2.1197546E38F, PH.base.pack) ;
        p261_available_capacity_SET((float) -1.6362485E38F, PH.base.pack) ;
        c_CommunicationChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_recording_time_ms_SET((uint32_t)1799417840L, PH.base.pack) ;
        p262_available_capacity_SET((float)2.8452116E38F, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)409385180L, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
        p262_image_interval_SET((float)8.285761E37F, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_alt_SET((int32_t) -2070384364, PH.base.pack) ;
        p263_image_index_SET((int32_t) -1976671550, PH.base.pack) ;
        p263_camera_id_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        {
            char16_t* file_url = u"orjlnnmal";
            p263_file_url_SET_(file_url, &PH) ;
        }
        {
            float q[] =  {1.3887822E38F, 2.5962746E38F, -2.4482775E38F, -1.372586E38F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_time_utc_SET((uint64_t)5966241922990225294L, PH.base.pack) ;
        p263_lat_SET((int32_t)1305750309, PH.base.pack) ;
        p263_lon_SET((int32_t) -261760497, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t) -93, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)2107903128L, PH.base.pack) ;
        p263_relative_alt_SET((int32_t)477841269, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_time_boot_ms_SET((uint32_t)2457793033L, PH.base.pack) ;
        p264_takeoff_time_utc_SET((uint64_t)620601023238093248L, PH.base.pack) ;
        p264_arming_time_utc_SET((uint64_t)9006858912816301412L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)4532303900450413308L, PH.base.pack) ;
        c_CommunicationChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_yaw_SET((float)1.7026499E38F, PH.base.pack) ;
        p265_roll_SET((float) -1.3431301E38F, PH.base.pack) ;
        p265_pitch_SET((float) -2.6541054E37F, PH.base.pack) ;
        p265_time_boot_ms_SET((uint32_t)426983620L, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_266(), &PH);
        p266_target_system_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        p266_length_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)145, (uint8_t)169, (uint8_t)184, (uint8_t)153, (uint8_t)136, (uint8_t)230, (uint8_t)177, (uint8_t)169, (uint8_t)136, (uint8_t)120, (uint8_t)220, (uint8_t)65, (uint8_t)132, (uint8_t)6, (uint8_t)22, (uint8_t)5, (uint8_t)226, (uint8_t)220, (uint8_t)174, (uint8_t)211, (uint8_t)252, (uint8_t)233, (uint8_t)195, (uint8_t)205, (uint8_t)25, (uint8_t)141, (uint8_t)24, (uint8_t)62, (uint8_t)83, (uint8_t)10, (uint8_t)95, (uint8_t)54, (uint8_t)44, (uint8_t)147, (uint8_t)74, (uint8_t)209, (uint8_t)240, (uint8_t)8, (uint8_t)51, (uint8_t)249, (uint8_t)136, (uint8_t)136, (uint8_t)208, (uint8_t)124, (uint8_t)50, (uint8_t)89, (uint8_t)1, (uint8_t)96, (uint8_t)159, (uint8_t)138, (uint8_t)110, (uint8_t)46, (uint8_t)71, (uint8_t)55, (uint8_t)51, (uint8_t)97, (uint8_t)117, (uint8_t)23, (uint8_t)133, (uint8_t)223, (uint8_t)244, (uint8_t)47, (uint8_t)150, (uint8_t)115, (uint8_t)243, (uint8_t)194, (uint8_t)66, (uint8_t)149, (uint8_t)16, (uint8_t)251, (uint8_t)203, (uint8_t)219, (uint8_t)35, (uint8_t)50, (uint8_t)227, (uint8_t)44, (uint8_t)48, (uint8_t)5, (uint8_t)34, (uint8_t)72, (uint8_t)116, (uint8_t)125, (uint8_t)3, (uint8_t)55, (uint8_t)193, (uint8_t)79, (uint8_t)55, (uint8_t)32, (uint8_t)8, (uint8_t)153, (uint8_t)217, (uint8_t)234, (uint8_t)177, (uint8_t)78, (uint8_t)46, (uint8_t)163, (uint8_t)224, (uint8_t)9, (uint8_t)52, (uint8_t)1, (uint8_t)226, (uint8_t)225, (uint8_t)59, (uint8_t)243, (uint8_t)220, (uint8_t)154, (uint8_t)241, (uint8_t)6, (uint8_t)129, (uint8_t)10, (uint8_t)210, (uint8_t)39, (uint8_t)246, (uint8_t)34, (uint8_t)177, (uint8_t)243, (uint8_t)113, (uint8_t)235, (uint8_t)64, (uint8_t)123, (uint8_t)133, (uint8_t)218, (uint8_t)248, (uint8_t)17, (uint8_t)135, (uint8_t)219, (uint8_t)232, (uint8_t)223, (uint8_t)50, (uint8_t)197, (uint8_t)114, (uint8_t)102, (uint8_t)225, (uint8_t)103, (uint8_t)100, (uint8_t)150, (uint8_t)221, (uint8_t)42, (uint8_t)229, (uint8_t)169, (uint8_t)10, (uint8_t)106, (uint8_t)166, (uint8_t)186, (uint8_t)1, (uint8_t)245, (uint8_t)147, (uint8_t)78, (uint8_t)47, (uint8_t)196, (uint8_t)49, (uint8_t)52, (uint8_t)6, (uint8_t)17, (uint8_t)87, (uint8_t)180, (uint8_t)28, (uint8_t)44, (uint8_t)220, (uint8_t)129, (uint8_t)134, (uint8_t)157, (uint8_t)236, (uint8_t)177, (uint8_t)166, (uint8_t)182, (uint8_t)157, (uint8_t)187, (uint8_t)31, (uint8_t)135, (uint8_t)141, (uint8_t)3, (uint8_t)209, (uint8_t)149, (uint8_t)45, (uint8_t)104, (uint8_t)2, (uint8_t)37, (uint8_t)148, (uint8_t)39, (uint8_t)216, (uint8_t)228, (uint8_t)57, (uint8_t)125, (uint8_t)91, (uint8_t)104, (uint8_t)229, (uint8_t)217, (uint8_t)167, (uint8_t)237, (uint8_t)227, (uint8_t)155, (uint8_t)188, (uint8_t)35, (uint8_t)143, (uint8_t)243, (uint8_t)0, (uint8_t)230, (uint8_t)172, (uint8_t)79, (uint8_t)143, (uint8_t)73, (uint8_t)48, (uint8_t)144, (uint8_t)18, (uint8_t)23, (uint8_t)169, (uint8_t)32, (uint8_t)183, (uint8_t)246, (uint8_t)208, (uint8_t)105, (uint8_t)10, (uint8_t)22, (uint8_t)147, (uint8_t)187, (uint8_t)234, (uint8_t)98, (uint8_t)111, (uint8_t)71, (uint8_t)206, (uint8_t)113, (uint8_t)34, (uint8_t)202, (uint8_t)120, (uint8_t)214, (uint8_t)58, (uint8_t)27, (uint8_t)66, (uint8_t)221, (uint8_t)235, (uint8_t)14, (uint8_t)158, (uint8_t)119, (uint8_t)91, (uint8_t)207, (uint8_t)10, (uint8_t)84, (uint8_t)151, (uint8_t)55, (uint8_t)50, (uint8_t)232, (uint8_t)127, (uint8_t)66, (uint8_t)23, (uint8_t)122, (uint8_t)43, (uint8_t)25, (uint8_t)20};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_sequence_SET((uint16_t)(uint16_t)31509, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        p266_first_message_offset_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_ACKED_267(), &PH);
        p267_first_message_offset_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        p267_target_component_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        p267_sequence_SET((uint16_t)(uint16_t)52175, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)205, (uint8_t)244, (uint8_t)217, (uint8_t)189, (uint8_t)156, (uint8_t)143, (uint8_t)250, (uint8_t)93, (uint8_t)71, (uint8_t)108, (uint8_t)66, (uint8_t)76, (uint8_t)240, (uint8_t)246, (uint8_t)165, (uint8_t)112, (uint8_t)152, (uint8_t)249, (uint8_t)10, (uint8_t)44, (uint8_t)89, (uint8_t)245, (uint8_t)49, (uint8_t)108, (uint8_t)16, (uint8_t)243, (uint8_t)221, (uint8_t)9, (uint8_t)245, (uint8_t)13, (uint8_t)154, (uint8_t)193, (uint8_t)182, (uint8_t)105, (uint8_t)105, (uint8_t)230, (uint8_t)243, (uint8_t)90, (uint8_t)159, (uint8_t)250, (uint8_t)231, (uint8_t)243, (uint8_t)64, (uint8_t)252, (uint8_t)159, (uint8_t)33, (uint8_t)0, (uint8_t)40, (uint8_t)101, (uint8_t)27, (uint8_t)83, (uint8_t)143, (uint8_t)191, (uint8_t)55, (uint8_t)49, (uint8_t)204, (uint8_t)200, (uint8_t)205, (uint8_t)13, (uint8_t)74, (uint8_t)233, (uint8_t)199, (uint8_t)34, (uint8_t)80, (uint8_t)242, (uint8_t)61, (uint8_t)66, (uint8_t)185, (uint8_t)132, (uint8_t)145, (uint8_t)244, (uint8_t)26, (uint8_t)171, (uint8_t)92, (uint8_t)9, (uint8_t)9, (uint8_t)171, (uint8_t)202, (uint8_t)150, (uint8_t)116, (uint8_t)242, (uint8_t)177, (uint8_t)34, (uint8_t)76, (uint8_t)6, (uint8_t)96, (uint8_t)98, (uint8_t)135, (uint8_t)1, (uint8_t)98, (uint8_t)247, (uint8_t)222, (uint8_t)10, (uint8_t)78, (uint8_t)197, (uint8_t)5, (uint8_t)210, (uint8_t)216, (uint8_t)214, (uint8_t)138, (uint8_t)137, (uint8_t)190, (uint8_t)62, (uint8_t)145, (uint8_t)159, (uint8_t)223, (uint8_t)103, (uint8_t)249, (uint8_t)55, (uint8_t)204, (uint8_t)12, (uint8_t)107, (uint8_t)22, (uint8_t)126, (uint8_t)198, (uint8_t)186, (uint8_t)207, (uint8_t)235, (uint8_t)110, (uint8_t)118, (uint8_t)80, (uint8_t)23, (uint8_t)122, (uint8_t)163, (uint8_t)3, (uint8_t)182, (uint8_t)176, (uint8_t)179, (uint8_t)52, (uint8_t)189, (uint8_t)65, (uint8_t)221, (uint8_t)114, (uint8_t)245, (uint8_t)245, (uint8_t)200, (uint8_t)5, (uint8_t)46, (uint8_t)75, (uint8_t)186, (uint8_t)253, (uint8_t)38, (uint8_t)71, (uint8_t)196, (uint8_t)199, (uint8_t)131, (uint8_t)113, (uint8_t)67, (uint8_t)28, (uint8_t)127, (uint8_t)200, (uint8_t)117, (uint8_t)104, (uint8_t)184, (uint8_t)177, (uint8_t)28, (uint8_t)164, (uint8_t)228, (uint8_t)232, (uint8_t)58, (uint8_t)69, (uint8_t)43, (uint8_t)172, (uint8_t)94, (uint8_t)159, (uint8_t)237, (uint8_t)152, (uint8_t)173, (uint8_t)159, (uint8_t)95, (uint8_t)112, (uint8_t)89, (uint8_t)53, (uint8_t)21, (uint8_t)195, (uint8_t)27, (uint8_t)192, (uint8_t)128, (uint8_t)17, (uint8_t)145, (uint8_t)177, (uint8_t)170, (uint8_t)151, (uint8_t)45, (uint8_t)7, (uint8_t)2, (uint8_t)239, (uint8_t)73, (uint8_t)42, (uint8_t)10, (uint8_t)54, (uint8_t)123, (uint8_t)159, (uint8_t)89, (uint8_t)158, (uint8_t)160, (uint8_t)13, (uint8_t)153, (uint8_t)243, (uint8_t)66, (uint8_t)163, (uint8_t)154, (uint8_t)211, (uint8_t)33, (uint8_t)197, (uint8_t)22, (uint8_t)217, (uint8_t)3, (uint8_t)245, (uint8_t)224, (uint8_t)91, (uint8_t)107, (uint8_t)49, (uint8_t)237, (uint8_t)180, (uint8_t)78, (uint8_t)196, (uint8_t)67, (uint8_t)228, (uint8_t)81, (uint8_t)200, (uint8_t)26, (uint8_t)82, (uint8_t)166, (uint8_t)226, (uint8_t)214, (uint8_t)239, (uint8_t)194, (uint8_t)27, (uint8_t)202, (uint8_t)114, (uint8_t)124, (uint8_t)189, (uint8_t)239, (uint8_t)220, (uint8_t)239, (uint8_t)77, (uint8_t)150, (uint8_t)223, (uint8_t)8, (uint8_t)248, (uint8_t)176, (uint8_t)150, (uint8_t)57, (uint8_t)107, (uint8_t)240, (uint8_t)86, (uint8_t)151, (uint8_t)175};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_ACK_268(), &PH);
        p268_sequence_SET((uint16_t)(uint16_t)17486, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
        p268_target_system_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_framerate_SET((float)2.2096894E38F, PH.base.pack) ;
        p269_rotation_SET((uint16_t)(uint16_t)53829, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)14492, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)31083, PH.base.pack) ;
        p269_bitrate_SET((uint32_t)3410945479L, PH.base.pack) ;
        {
            char16_t* uri = u"nFoxfbzzmlqtppfwjle";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_status_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_camera_id_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)165600205L, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)1433, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)30547, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)35057, PH.base.pack) ;
        p270_target_component_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        {
            char16_t* uri = u"poxmjYuxydmrkusxfbrqeEsqrfmnxfcnrpbnjosegRasozbhzxoranyszpqempTuiuioEkrejtgpqighrRnddvemykzuqdYuhumcgstumpjtvdrgioywoynhJtexmUlocqwmtbngwrgDuyeoqoycqKZlsslhogshqviyryjrdzjacsyOYqxbsxfhhashiyduupcXfhcymglzVfwvdshtafwGbzgsbded";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_target_system_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        p270_framerate_SET((float)1.8216636E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* ssid = u"lrhzctJwezxhwgtgtulgjz";
            p299_ssid_SET_(ssid, &PH) ;
        }
        {
            char16_t* password = u"ceJGgzmmhwHxsieszZhnbqijhwfzfcdvxprdwltrtoqsxIsyxAovdbuqlt";
            p299_password_SET_(password, &PH) ;
        }
        c_CommunicationChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PROTOCOL_VERSION_300(), &PH);
        p300_version_SET((uint16_t)(uint16_t)12066, PH.base.pack) ;
        {
            uint8_t library_version_hash[] =  {(uint8_t)25, (uint8_t)126, (uint8_t)32, (uint8_t)4, (uint8_t)162, (uint8_t)34, (uint8_t)49, (uint8_t)82};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        p300_min_version_SET((uint16_t)(uint16_t)59115, PH.base.pack) ;
        {
            uint8_t spec_version_hash[] =  {(uint8_t)73, (uint8_t)34, (uint8_t)194, (uint8_t)251, (uint8_t)156, (uint8_t)38, (uint8_t)190, (uint8_t)139};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        p300_max_version_SET((uint16_t)(uint16_t)29829, PH.base.pack) ;
        c_CommunicationChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_sub_mode_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)3873453746L, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)18460, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)125744112668726778L, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_hw_version_minor_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        p311_hw_version_major_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)3025369553L, PH.base.pack) ;
        p311_sw_version_minor_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)78, (uint8_t)155, (uint8_t)94, (uint8_t)34, (uint8_t)243, (uint8_t)77, (uint8_t)115, (uint8_t)177, (uint8_t)208, (uint8_t)173, (uint8_t)115, (uint8_t)190, (uint8_t)54, (uint8_t)221, (uint8_t)31, (uint8_t)130};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        p311_time_usec_SET((uint64_t)2945280521365090318L, PH.base.pack) ;
        p311_sw_version_major_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)1583425061L, PH.base.pack) ;
        {
            char16_t* name = u"flheyojocxfxzthnvvaabfOiHbBaccshytqtrozt";
            p311_name_SET_(name, &PH) ;
        }
        c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        p320_target_system_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p320_target_component_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p320_param_index_SET((int16_t)(int16_t) -11335, PH.base.pack) ;
        {
            char16_t* param_id = u"sullyy";
            p320_param_id_SET_(param_id, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_component_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p321_target_system_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_VALUE_322(), &PH);
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32, PH.base.pack) ;
        p322_param_count_SET((uint16_t)(uint16_t)38717, PH.base.pack) ;
        {
            char16_t* param_id = u"pzjb";
            p322_param_id_SET_(param_id, &PH) ;
        }
        p322_param_index_SET((uint16_t)(uint16_t)25089, PH.base.pack) ;
        {
            char16_t* param_value = u"iohfYsWvgYuxxfcQmjyWVlfohhwwrwldbjboncoXuqfrbhnuXunyraiiqfsphOpufzmkomsrxdjqwynbyIitdmlatLwlWbcBXmmtlbZstyy";
            p322_param_value_SET_(param_value, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_SET_323(), &PH);
        p323_target_system_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p323_target_component_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32, PH.base.pack) ;
        {
            char16_t* param_id = u"pcbd";
            p323_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"bDgiuorzrbuecfjZjcecazitfxWo";
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
            char16_t* param_id = u"lb";
            p324_param_id_SET_(param_id, &PH) ;
        }
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_ACCEPTED, PH.base.pack) ;
        {
            char16_t* param_value = u"pijwGVnqdlngukyQuioqmdfmgDeno";
            p324_param_value_SET_(param_value, &PH) ;
        }
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_OBSTACLE_DISTANCE_330(), &PH);
        p330_min_distance_SET((uint16_t)(uint16_t)47733, PH.base.pack) ;
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, PH.base.pack) ;
        p330_max_distance_SET((uint16_t)(uint16_t)10331, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)36849, (uint16_t)44779, (uint16_t)25423, (uint16_t)17511, (uint16_t)26650, (uint16_t)3475, (uint16_t)61505, (uint16_t)32221, (uint16_t)44780, (uint16_t)39975, (uint16_t)30588, (uint16_t)56288, (uint16_t)51494, (uint16_t)17063, (uint16_t)62676, (uint16_t)23797, (uint16_t)23515, (uint16_t)49672, (uint16_t)32695, (uint16_t)16576, (uint16_t)52166, (uint16_t)13718, (uint16_t)4233, (uint16_t)6496, (uint16_t)14052, (uint16_t)43255, (uint16_t)31354, (uint16_t)5956, (uint16_t)22143, (uint16_t)15894, (uint16_t)18681, (uint16_t)32984, (uint16_t)51455, (uint16_t)62374, (uint16_t)35219, (uint16_t)497, (uint16_t)43159, (uint16_t)49203, (uint16_t)27878, (uint16_t)22191, (uint16_t)2474, (uint16_t)18834, (uint16_t)56488, (uint16_t)63261, (uint16_t)8160, (uint16_t)12219, (uint16_t)56499, (uint16_t)14254, (uint16_t)17889, (uint16_t)51149, (uint16_t)41184, (uint16_t)19428, (uint16_t)1227, (uint16_t)58779, (uint16_t)19092, (uint16_t)6596, (uint16_t)49198, (uint16_t)15279, (uint16_t)19026, (uint16_t)12885, (uint16_t)60199, (uint16_t)26894, (uint16_t)64479, (uint16_t)54343, (uint16_t)31218, (uint16_t)42574, (uint16_t)14484, (uint16_t)55927, (uint16_t)48709, (uint16_t)6151, (uint16_t)44540, (uint16_t)21434};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_increment_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p330_time_usec_SET((uint64_t)957320193579351525L, PH.base.pack) ;
        c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
}


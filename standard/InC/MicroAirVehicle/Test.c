
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
INLINER void p138_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER void p138_q_SET(float*  src, int32_t pos, Pack * dst) //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  8, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER void p138_x_SET(float  src, Pack * dst)//X position in meters (NED)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p138_y_SET(float  src, Pack * dst)//Y position in meters (NED)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p138_z_SET(float  src, Pack * dst)//Z position in meters (NED)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
Pack * c_TEST_Channel_new_ATT_POS_MOCAP_138()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 138));
};
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
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p0_custom_mode_GET(pack) == (uint32_t)3963663799L);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_EMERGENCY);
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_OPENPILOT);
    assert(p0_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_OCTOROTOR);
};


void c_TEST_Channel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t)18349);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)9645);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)28182);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)16391);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2);
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)48032);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)54312);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)10946);
    assert(p1_onboard_control_sensors_health_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)12376);
    assert(p1_onboard_control_sensors_present_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)50816);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t) -107);
};


void c_TEST_Channel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)1998796706L);
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)5623477479509228515L);
};


void c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_yaw_GET(pack) == (float) -9.358542E37F);
    assert(p3_afz_GET(pack) == (float)1.0809529E38F);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)38035);
    assert(p3_vx_GET(pack) == (float)3.0141145E38F);
    assert(p3_yaw_rate_GET(pack) == (float)2.8133365E38F);
    assert(p3_vy_GET(pack) == (float)1.1828985E38F);
    assert(p3_vz_GET(pack) == (float) -8.472486E37F);
    assert(p3_y_GET(pack) == (float) -1.0579017E38F);
    assert(p3_afy_GET(pack) == (float)1.5097453E38F);
    assert(p3_z_GET(pack) == (float)1.7846626E38F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)2678205790L);
    assert(p3_afx_GET(pack) == (float)1.4099021E38F);
    assert(p3_x_GET(pack) == (float)5.0584654E37F);
};


void c_TEST_Channel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_seq_GET(pack) == (uint32_t)1192581163L);
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)216);
    assert(p4_time_usec_GET(pack) == (uint64_t)738642279151904811L);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)241);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)245);
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)132);
    assert(p5_passkey_LEN(ph) == 13);
    {
        char16_t * exemplary = u"fVgMmmbvndkgs";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)244);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)26);
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)184);
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)205);
};


void c_TEST_Channel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 17);
    {
        char16_t * exemplary = u"ffxwzmbgkswmfEnht";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 34);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)255);
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_STABILIZE_ARMED);
    assert(p11_custom_mode_GET(pack) == (uint32_t)4109090955L);
};


void c_TEST_Channel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t)20512);
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)157);
    assert(p20_param_id_LEN(ph) == 2);
    {
        char16_t * exemplary = u"Oa";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)105);
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)37);
};


void c_TEST_Channel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)57557);
    assert(p22_param_value_GET(pack) == (float) -2.4499788E38F);
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)44890);
    assert(p22_param_id_LEN(ph) == 15);
    {
        char16_t * exemplary = u"dzbjmdywoRTdseq";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16);
};


void c_TEST_Channel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_param_value_GET(pack) == (float)3.0301916E38F);
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)213);
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32);
    assert(p23_param_id_LEN(ph) == 7);
    {
        char16_t * exemplary = u"uwfnppf";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)49715);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)68);
    assert(p24_h_acc_TRY(ph) == (uint32_t)2891619359L);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)733765542L);
    assert(p24_time_usec_GET(pack) == (uint64_t)5082700707253431056L);
    assert(p24_v_acc_TRY(ph) == (uint32_t)2163748263L);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)45889);
    assert(p24_alt_GET(pack) == (int32_t)832601250);
    assert(p24_lon_GET(pack) == (int32_t)1314997383);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)4277435625L);
    assert(p24_lat_GET(pack) == (int32_t)562793228);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t) -59889805);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)1722);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)40342);
};


void c_TEST_Channel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)27, (uint8_t)138, (uint8_t)40, (uint8_t)65, (uint8_t)149, (uint8_t)130, (uint8_t)184, (uint8_t)234, (uint8_t)211, (uint8_t)85, (uint8_t)7, (uint8_t)247, (uint8_t)16, (uint8_t)14, (uint8_t)16, (uint8_t)84, (uint8_t)154, (uint8_t)234, (uint8_t)175, (uint8_t)44} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)210, (uint8_t)193, (uint8_t)242, (uint8_t)194, (uint8_t)180, (uint8_t)245, (uint8_t)49, (uint8_t)139, (uint8_t)15, (uint8_t)26, (uint8_t)106, (uint8_t)163, (uint8_t)67, (uint8_t)29, (uint8_t)213, (uint8_t)238, (uint8_t)102, (uint8_t)171, (uint8_t)167, (uint8_t)185} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)35, (uint8_t)186, (uint8_t)147, (uint8_t)4, (uint8_t)157, (uint8_t)222, (uint8_t)42, (uint8_t)179, (uint8_t)162, (uint8_t)110, (uint8_t)17, (uint8_t)39, (uint8_t)213, (uint8_t)233, (uint8_t)237, (uint8_t)108, (uint8_t)108, (uint8_t)174, (uint8_t)235, (uint8_t)54} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)153, (uint8_t)156, (uint8_t)131, (uint8_t)22, (uint8_t)74, (uint8_t)111, (uint8_t)34, (uint8_t)228, (uint8_t)171, (uint8_t)213, (uint8_t)110, (uint8_t)238, (uint8_t)26, (uint8_t)72, (uint8_t)92, (uint8_t)135, (uint8_t)52, (uint8_t)244, (uint8_t)80, (uint8_t)143} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)198, (uint8_t)252, (uint8_t)146, (uint8_t)180, (uint8_t)245, (uint8_t)192, (uint8_t)249, (uint8_t)177, (uint8_t)245, (uint8_t)244, (uint8_t)210, (uint8_t)240, (uint8_t)181, (uint8_t)224, (uint8_t)0, (uint8_t)213, (uint8_t)145, (uint8_t)207, (uint8_t)50, (uint8_t)151} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)38);
};


void c_TEST_Channel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)1827985144L);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t)3266);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t)5131);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t) -28977);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t)12048);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t)6931);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t) -5486);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t)6632);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t)8315);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t)276);
};


void c_TEST_Channel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t)32072);
    assert(p27_time_usec_GET(pack) == (uint64_t)5784070630332877928L);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t)5190);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t) -8698);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t) -12455);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t) -17597);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t)25247);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t)28838);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t) -3783);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t)6156);
};


void c_TEST_Channel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t) -15821);
    assert(p28_time_usec_GET(pack) == (uint64_t)2420029976750290474L);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t) -7550);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t) -21831);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t)28967);
};


void c_TEST_Channel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)2777000147L);
    assert(p29_press_abs_GET(pack) == (float) -3.1806921E38F);
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t) -8705);
    assert(p29_press_diff_GET(pack) == (float)2.8552947E38F);
};


void c_TEST_Channel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_yaw_GET(pack) == (float) -2.0927818E38F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)3860618836L);
    assert(p30_roll_GET(pack) == (float) -2.995289E38F);
    assert(p30_pitch_GET(pack) == (float) -1.5504259E38F);
    assert(p30_rollspeed_GET(pack) == (float)2.5599526E38F);
    assert(p30_pitchspeed_GET(pack) == (float)1.2905179E38F);
    assert(p30_yawspeed_GET(pack) == (float)8.713658E36F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_q3_GET(pack) == (float) -1.3151464E38F);
    assert(p31_pitchspeed_GET(pack) == (float)1.1325669E38F);
    assert(p31_yawspeed_GET(pack) == (float) -2.0044965E38F);
    assert(p31_rollspeed_GET(pack) == (float)9.103879E37F);
    assert(p31_q4_GET(pack) == (float)3.3235552E37F);
    assert(p31_q2_GET(pack) == (float) -2.7354363E38F);
    assert(p31_q1_GET(pack) == (float) -2.268714E38F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)3228766487L);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)2718907227L);
    assert(p32_x_GET(pack) == (float) -3.3733418E38F);
    assert(p32_vx_GET(pack) == (float)2.3754338E38F);
    assert(p32_vz_GET(pack) == (float)2.4881673E38F);
    assert(p32_y_GET(pack) == (float) -1.4839423E38F);
    assert(p32_z_GET(pack) == (float)2.529464E37F);
    assert(p32_vy_GET(pack) == (float)6.943568E37F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_lat_GET(pack) == (int32_t)1183614468);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t) -21489);
    assert(p33_alt_GET(pack) == (int32_t) -591297719);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)52699);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t) -13302);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t)11561);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)1430183752L);
    assert(p33_relative_alt_GET(pack) == (int32_t)468401063);
    assert(p33_lon_GET(pack) == (int32_t)1140831066);
};


void c_TEST_Channel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t)3613);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t) -18107);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t) -5432);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t)29720);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t) -20710);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t) -27298);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t) -4952);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)822528977L);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t)30618);
};


void c_TEST_Channel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)55586);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)38234);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)32281);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)40);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)1953193067L);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)40018);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)35288);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)16903);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)11521);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)69);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)5711);
};


void c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)51332);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)52404);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)43501);
    assert(p36_time_usec_GET(pack) == (uint32_t)3297173575L);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)12990);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)7090);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)38193);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)54903);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)47062);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)16237);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)63544);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)57045);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)47721);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)1946);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)16444);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)32063);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)12685);
};


void c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)117);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)225);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t) -30347);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t)9576);
};


void c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t) -22335);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)107);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t)5405);
};


void c_TEST_Channel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)218);
    assert(p39_param3_GET(pack) == (float)3.18499E38F);
    assert(p39_param1_GET(pack) == (float)9.668622E37F);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)22888);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p39_param4_GET(pack) == (float)1.536996E38F);
    assert(p39_y_GET(pack) == (float) -1.6101427E38F);
    assert(p39_param2_GET(pack) == (float)2.723084E38F);
    assert(p39_x_GET(pack) == (float)4.0038699E37F);
    assert(p39_z_GET(pack) == (float)6.281886E37F);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
};


void c_TEST_Channel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)27947);
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)32);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)10);
};


void c_TEST_Channel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)25120);
};


void c_TEST_Channel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)18853);
};


void c_TEST_Channel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)83);
};


void c_TEST_Channel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)26207);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)226);
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)143);
};


void c_TEST_Channel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)69);
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)243);
};


void c_TEST_Channel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)49543);
};


void c_TEST_Channel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM5_X);
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)131);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)47);
};


void c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_latitude_GET(pack) == (int32_t) -821663400);
    assert(p48_time_usec_TRY(ph) == (uint64_t)661057399724568439L);
    assert(p48_longitude_GET(pack) == (int32_t)713842092);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p48_altitude_GET(pack) == (int32_t)388170029);
};


void c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_latitude_GET(pack) == (int32_t)1628743409);
    assert(p49_time_usec_TRY(ph) == (uint64_t)6190259897561767168L);
    assert(p49_longitude_GET(pack) == (int32_t)1567055870);
    assert(p49_altitude_GET(pack) == (int32_t) -1726678611);
};


void c_TEST_Channel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_param_value_min_GET(pack) == (float) -2.0911673E38F);
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)35);
    assert(p50_param_value0_GET(pack) == (float) -3.075087E38F);
    assert(p50_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"z";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t) -21049);
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)169);
    assert(p50_scale_GET(pack) == (float)1.4425711E38F);
    assert(p50_param_value_max_GET(pack) == (float)1.5479767E38F);
};


void c_TEST_Channel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)62694);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)72);
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)143);
};


void c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)211);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)154);
    assert(p54_p1y_GET(pack) == (float) -1.4393933E38F);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p54_p2y_GET(pack) == (float) -2.8972657E38F);
    assert(p54_p1z_GET(pack) == (float)2.7912086E38F);
    assert(p54_p2x_GET(pack) == (float) -2.2860515E38F);
    assert(p54_p1x_GET(pack) == (float) -1.7209253E38F);
    assert(p54_p2z_GET(pack) == (float)1.7046106E38F);
};


void c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p2z_GET(pack) == (float)3.8682135E37F);
    assert(p55_p2y_GET(pack) == (float)1.6674953E38F);
    assert(p55_p1x_GET(pack) == (float) -2.7042214E38F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_NED);
    assert(p55_p2x_GET(pack) == (float) -3.113374E38F);
    assert(p55_p1z_GET(pack) == (float)1.3492177E38F);
    assert(p55_p1y_GET(pack) == (float)1.769379E38F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-2.4627713E38F, -8.0559216E37F, 2.3250278E38F, -1.4721795E38F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {8.624855E37F, -1.773088E38F, -1.197097E38F, 2.1944395E38F, -4.567355E36F, -1.9617086E38F, 2.8283467E38F, -6.2815966E37F, 2.4682034E37F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_pitchspeed_GET(pack) == (float)1.8103148E38F);
    assert(p61_yawspeed_GET(pack) == (float)6.803884E37F);
    assert(p61_rollspeed_GET(pack) == (float) -3.2285225E37F);
    assert(p61_time_usec_GET(pack) == (uint64_t)382978356164063664L);
};


void c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t) -4287);
    assert(p62_aspd_error_GET(pack) == (float)2.982964E38F);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t)114);
    assert(p62_nav_roll_GET(pack) == (float) -2.750247E38F);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)38387);
    assert(p62_xtrack_error_GET(pack) == (float) -1.9268052E38F);
    assert(p62_nav_pitch_GET(pack) == (float) -1.6226565E38F);
    assert(p62_alt_error_GET(pack) == (float) -2.9013537E38F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_relative_alt_GET(pack) == (int32_t)230021988);
    assert(p63_lat_GET(pack) == (int32_t) -1810875865);
    assert(p63_alt_GET(pack) == (int32_t) -1387717025);
    assert(p63_vz_GET(pack) == (float)1.89144E38F);
    assert(p63_vy_GET(pack) == (float)3.6964844E37F);
    assert(p63_vx_GET(pack) == (float) -1.1796876E38F);
    assert(p63_time_usec_GET(pack) == (uint64_t)9130898990363855839L);
    assert(p63_lon_GET(pack) == (int32_t)294424678);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS);
    {
        float exemplary[] =  {-1.2102523E38F, 1.2810792E38F, -2.478801E38F, -3.0821705E38F, 2.5466693E38F, 1.4737472E38F, 2.5109718E38F, 1.4144409E38F, -2.4192935E38F, 1.0986593E38F, -2.5032651E38F, 2.4538034E38F, 1.655048E38F, -1.0743504E38F, -1.4499836E38F, -1.4807932E38F, 7.048733E37F, -3.353283E38F, 2.4203087E38F, -1.0236553E38F, -1.2965495E38F, 2.9330956E38F, -3.9446115E37F, -2.695256E38F, -2.7039368E37F, 2.5325744E38F, 3.206408E38F, -1.0343767E38F, -1.9201432E37F, -3.1005742E38F, 1.1987241E37F, 2.6079538E38F, 1.9328248E38F, -5.296355E37F, 3.0762897E38F, -2.6730794E38F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_vx_GET(pack) == (float)3.0937522E38F);
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO);
    {
        float exemplary[] =  {-2.6489693E38F, -1.9294758E38F, -1.3704618E37F, 5.776104E37F, 3.0120267E38F, -1.990804E38F, 8.834275E37F, 2.175768E38F, 1.2487679E38F, 2.6288542E38F, -2.7963875E38F, -6.4975374E37F, 1.7071205E38F, 2.0140946E38F, 2.4151905E37F, -1.8348847E38F, -1.0900902E38F, 8.378776E37F, 2.4877515E38F, 2.9725713E38F, 2.7205796E38F, 2.9173559E38F, -2.7126455E38F, 3.3631018E38F, 1.6427293E38F, -1.8831493E38F, 1.0701349E37F, -3.2151889E37F, -4.65126E37F, 8.709773E37F, -7.6544217E37F, 3.0473412E38F, 7.6192383E37F, 2.8677946E38F, -3.0411216E38F, 1.6844173E38F, 1.180784E37F, -1.8547267E37F, 1.951001E38F, 2.3083512E38F, 2.4104569E38F, 2.664469E38F, -3.1089399E38F, 3.136372E38F, -1.3566426E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_az_GET(pack) == (float)2.2904497E38F);
    assert(p64_vy_GET(pack) == (float) -1.5391697E38F);
    assert(p64_x_GET(pack) == (float)2.700082E37F);
    assert(p64_time_usec_GET(pack) == (uint64_t)4167268189364019142L);
    assert(p64_vz_GET(pack) == (float) -1.4765127E38F);
    assert(p64_z_GET(pack) == (float) -2.9070905E38F);
    assert(p64_ay_GET(pack) == (float) -1.9223717E38F);
    assert(p64_y_GET(pack) == (float) -1.1379941E38F);
    assert(p64_ax_GET(pack) == (float)4.2152412E36F);
};


void c_TEST_Channel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)59123);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)23774);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)46561);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)4123658133L);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)39236);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)30459);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)58578);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)52238);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)101);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)4386);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)55267);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)13857);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)50218);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)21522);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)6656);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)57366);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)65129);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)22958);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)51915);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)34087);
};


void c_TEST_Channel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)226);
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)148);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)57379);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)235);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)202);
};


void c_TEST_Channel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)221);
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)29115);
};


void c_TEST_Channel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)6801);
    assert(p69_r_GET(pack) == (int16_t)(int16_t) -3318);
    assert(p69_y_GET(pack) == (int16_t)(int16_t) -23729);
    assert(p69_z_GET(pack) == (int16_t)(int16_t) -27292);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p69_x_GET(pack) == (int16_t)(int16_t) -10170);
};


void c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)17707);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)59344);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)65171);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)44397);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)895);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)62168);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)225);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)60529);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)30459);
};


void c_TEST_Channel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)251);
    assert(p73_param2_GET(pack) == (float)1.5434218E38F);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)48163);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p73_y_GET(pack) == (int32_t) -507590316);
    assert(p73_x_GET(pack) == (int32_t) -578738493);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)227);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p73_param1_GET(pack) == (float)1.2158014E38F);
    assert(p73_param3_GET(pack) == (float) -7.732285E37F);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p73_param4_GET(pack) == (float)2.8287092E38F);
    assert(p73_z_GET(pack) == (float)2.1211845E38F);
};


void c_CommunicationChannel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_alt_GET(pack) == (float) -2.2534467E38F);
    assert(p74_groundspeed_GET(pack) == (float) -2.4065619E38F);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t) -22262);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)4200);
    assert(p74_airspeed_GET(pack) == (float)1.1652333E38F);
    assert(p74_climb_GET(pack) == (float) -4.802045E37F);
};


void c_CommunicationChannel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)171);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION);
    assert(p75_param4_GET(pack) == (float) -1.0336779E38F);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)227);
    assert(p75_z_GET(pack) == (float) -4.7665807E37F);
    assert(p75_y_GET(pack) == (int32_t) -1103037049);
    assert(p75_x_GET(pack) == (int32_t) -1773459716);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p75_param3_GET(pack) == (float) -2.1822763E38F);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p75_param2_GET(pack) == (float)8.3944193E37F);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p75_param1_GET(pack) == (float) -2.5065485E38F);
};


void c_CommunicationChannel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_param7_GET(pack) == (float) -2.0492371E38F);
    assert(p76_param3_GET(pack) == (float) -2.9838885E38F);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p76_param5_GET(pack) == (float) -3.5261978E37F);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)146);
    assert(p76_param6_GET(pack) == (float)2.8246103E37F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p76_param2_GET(pack) == (float) -7.122422E37F);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_LAST);
    assert(p76_param4_GET(pack) == (float) -1.4108692E38F);
    assert(p76_param1_GET(pack) == (float) -1.4657895E38F);
};


void c_CommunicationChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)160);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_ACCEPTED);
    assert(p77_result_param2_TRY(ph) == (int32_t)1915796705);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)43);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)50);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY);
};


void c_CommunicationChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_yaw_GET(pack) == (float) -2.5711321E38F);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)3704288031L);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)204);
    assert(p81_roll_GET(pack) == (float)1.7423118E37F);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p81_thrust_GET(pack) == (float) -1.0517644E38F);
    assert(p81_pitch_GET(pack) == (float) -7.899273E37F);
};


void c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)187);
    {
        float exemplary[] =  {3.0965447E38F, 2.134614E38F, 1.9944095E38F, -1.839429E38F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_thrust_GET(pack) == (float) -2.8882338E38F);
    assert(p82_body_pitch_rate_GET(pack) == (float)3.1095765E38F);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p82_body_roll_rate_GET(pack) == (float)1.671824E36F);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)4033694872L);
    assert(p82_body_yaw_rate_GET(pack) == (float) -8.528031E37F);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)62);
};


void c_CommunicationChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_thrust_GET(pack) == (float) -7.6001064E37F);
    assert(p83_body_roll_rate_GET(pack) == (float) -1.8965446E38F);
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)181);
    assert(p83_body_pitch_rate_GET(pack) == (float)2.7429525E37F);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)836184338L);
    {
        float exemplary[] =  {2.467757E38F, 1.1419551E38F, -4.2553218E36F, -3.0326632E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p83_body_yaw_rate_GET(pack) == (float)1.2020263E38F);
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_vy_GET(pack) == (float) -1.843436E38F);
    assert(p84_afy_GET(pack) == (float) -1.5762462E38F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)2840657028L);
    assert(p84_yaw_GET(pack) == (float)1.8024951E38F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)91);
    assert(p84_afz_GET(pack) == (float) -3.0432421E38F);
    assert(p84_y_GET(pack) == (float)2.1458006E38F);
    assert(p84_afx_GET(pack) == (float)2.5455708E38F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)37687);
    assert(p84_z_GET(pack) == (float) -2.757985E38F);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p84_vx_GET(pack) == (float) -2.6647164E38F);
    assert(p84_x_GET(pack) == (float)2.9109059E38F);
    assert(p84_yaw_rate_GET(pack) == (float) -1.6824368E38F);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p84_vz_GET(pack) == (float) -3.2925784E38F);
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)25973);
    assert(p86_vy_GET(pack) == (float) -1.9320902E38F);
    assert(p86_vz_GET(pack) == (float) -2.1358592E37F);
    assert(p86_vx_GET(pack) == (float) -2.6102897E38F);
    assert(p86_yaw_rate_GET(pack) == (float)1.2809817E38F);
    assert(p86_afz_GET(pack) == (float)2.9970862E38F);
    assert(p86_lat_int_GET(pack) == (int32_t)22567464);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)105);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p86_afy_GET(pack) == (float) -7.5840376E37F);
    assert(p86_alt_GET(pack) == (float) -2.3043766E38F);
    assert(p86_yaw_GET(pack) == (float) -1.8041025E38F);
    assert(p86_afx_GET(pack) == (float) -7.030669E37F);
    assert(p86_lon_int_GET(pack) == (int32_t)1005439647);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)3837076562L);
};


void c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_yaw_GET(pack) == (float) -1.302302E38F);
    assert(p87_vz_GET(pack) == (float) -1.236594E38F);
    assert(p87_afy_GET(pack) == (float) -2.9323085E38F);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)15341);
    assert(p87_alt_GET(pack) == (float) -1.4282283E37F);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p87_lat_int_GET(pack) == (int32_t) -1750558671);
    assert(p87_yaw_rate_GET(pack) == (float) -1.1988723E37F);
    assert(p87_afx_GET(pack) == (float) -1.4652337E38F);
    assert(p87_lon_int_GET(pack) == (int32_t) -1656171705);
    assert(p87_vx_GET(pack) == (float)1.6505227E38F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)25800711L);
    assert(p87_afz_GET(pack) == (float)1.8256937E38F);
    assert(p87_vy_GET(pack) == (float) -2.1458333E38F);
};


void c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_y_GET(pack) == (float) -3.1492727E38F);
    assert(p89_x_GET(pack) == (float) -8.664972E37F);
    assert(p89_pitch_GET(pack) == (float) -9.471885E37F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)2199127090L);
    assert(p89_roll_GET(pack) == (float)3.6291902E36F);
    assert(p89_z_GET(pack) == (float) -1.1691921E38F);
    assert(p89_yaw_GET(pack) == (float)1.2517426E38F);
};


void c_CommunicationChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t) -28403);
    assert(p90_pitchspeed_GET(pack) == (float)9.205762E37F);
    assert(p90_roll_GET(pack) == (float)2.3408758E37F);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t)11177);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t) -13396);
    assert(p90_yawspeed_GET(pack) == (float)2.7040912E37F);
    assert(p90_lon_GET(pack) == (int32_t) -1634073786);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t)8161);
    assert(p90_yaw_GET(pack) == (float) -1.3418471E38F);
    assert(p90_lat_GET(pack) == (int32_t)516288825);
    assert(p90_rollspeed_GET(pack) == (float)1.5731412E37F);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t) -9448);
    assert(p90_time_usec_GET(pack) == (uint64_t)2460617860524724550L);
    assert(p90_pitch_GET(pack) == (float) -2.9559448E38F);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t) -7812);
    assert(p90_alt_GET(pack) == (int32_t)364606664);
};


void c_CommunicationChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_aux1_GET(pack) == (float) -1.1018966E38F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_GUIDED_DISARMED);
    assert(p91_yaw_rudder_GET(pack) == (float) -1.762535E38F);
    assert(p91_time_usec_GET(pack) == (uint64_t)2954836293117905875L);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p91_roll_ailerons_GET(pack) == (float) -1.0650526E38F);
    assert(p91_aux3_GET(pack) == (float) -1.6665127E38F);
    assert(p91_aux2_GET(pack) == (float) -1.0273079E38F);
    assert(p91_throttle_GET(pack) == (float)2.1153253E38F);
    assert(p91_pitch_elevator_GET(pack) == (float)2.7982E38F);
    assert(p91_aux4_GET(pack) == (float) -3.3422626E38F);
};


void c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)58919);
    assert(p92_time_usec_GET(pack) == (uint64_t)1006035897317075436L);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)16634);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)32800);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)16738);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)38298);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)25279);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)2330);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)7332);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)174);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)34350);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)28694);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)36250);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)5780);
};


void c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    assert(p93_time_usec_GET(pack) == (uint64_t)4833137921624074869L);
    assert(p93_flags_GET(pack) == (uint64_t)698646595076618121L);
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_GUIDED_ARMED);
    {
        float exemplary[] =  {-8.247331E36F, 5.944022E37F, -2.2770088E38F, 5.466109E37F, -2.5152311E38F, -8.905611E37F, 2.9195898E38F, 2.6872844E38F, -9.164789E37F, -2.9454563E38F, 1.3820099E38F, 1.0713685E38F, 9.517887E37F, -8.658811E37F, 2.365475E38F, 2.004287E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t) -31355);
    assert(p100_flow_rate_y_TRY(ph) == (float) -3.2569836E38F);
    assert(p100_flow_rate_x_TRY(ph) == (float)2.64496E38F);
    assert(p100_flow_comp_m_x_GET(pack) == (float)1.9649876E38F);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)93);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t) -877);
    assert(p100_ground_distance_GET(pack) == (float) -2.5976975E37F);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)26);
    assert(p100_time_usec_GET(pack) == (uint64_t)818274337112929359L);
    assert(p100_flow_comp_m_y_GET(pack) == (float)1.6527811E38F);
};


void c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_usec_GET(pack) == (uint64_t)7205380170764564865L);
    assert(p101_z_GET(pack) == (float) -1.8679635E38F);
    assert(p101_pitch_GET(pack) == (float) -1.696389E38F);
    assert(p101_roll_GET(pack) == (float) -3.0782637E38F);
    assert(p101_yaw_GET(pack) == (float) -3.0323395E38F);
    assert(p101_y_GET(pack) == (float) -8.864521E37F);
    assert(p101_x_GET(pack) == (float)2.797536E38F);
};


void c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_roll_GET(pack) == (float)3.39434E38F);
    assert(p102_yaw_GET(pack) == (float) -2.3510135E37F);
    assert(p102_y_GET(pack) == (float)9.750819E37F);
    assert(p102_z_GET(pack) == (float)1.808906E38F);
    assert(p102_x_GET(pack) == (float)1.5388927E38F);
    assert(p102_usec_GET(pack) == (uint64_t)8711455863589439307L);
    assert(p102_pitch_GET(pack) == (float) -2.7004002E38F);
};


void c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_usec_GET(pack) == (uint64_t)8762700995146816756L);
    assert(p103_z_GET(pack) == (float)1.7251512E38F);
    assert(p103_x_GET(pack) == (float) -2.6531325E38F);
    assert(p103_y_GET(pack) == (float)1.860354E38F);
};


void c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_pitch_GET(pack) == (float) -2.8189438E38F);
    assert(p104_y_GET(pack) == (float) -2.0960584E38F);
    assert(p104_roll_GET(pack) == (float)3.2028093E38F);
    assert(p104_z_GET(pack) == (float) -1.543986E38F);
    assert(p104_x_GET(pack) == (float)2.7743866E38F);
    assert(p104_yaw_GET(pack) == (float)3.7907867E37F);
    assert(p104_usec_GET(pack) == (uint64_t)6386520075828062971L);
};


void c_CommunicationChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_diff_pressure_GET(pack) == (float) -2.1398766E38F);
    assert(p105_yacc_GET(pack) == (float)3.3336205E38F);
    assert(p105_zmag_GET(pack) == (float) -1.9904948E38F);
    assert(p105_xgyro_GET(pack) == (float)9.977335E37F);
    assert(p105_xmag_GET(pack) == (float) -3.1917793E38F);
    assert(p105_zgyro_GET(pack) == (float)3.0557217E38F);
    assert(p105_ymag_GET(pack) == (float)1.1042816E38F);
    assert(p105_ygyro_GET(pack) == (float) -5.1140727E37F);
    assert(p105_xacc_GET(pack) == (float)3.239239E38F);
    assert(p105_pressure_alt_GET(pack) == (float) -3.3457723E38F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)55066);
    assert(p105_abs_pressure_GET(pack) == (float)3.992781E37F);
    assert(p105_time_usec_GET(pack) == (uint64_t)3183386818622065744L);
    assert(p105_temperature_GET(pack) == (float) -2.5841709E38F);
    assert(p105_zacc_GET(pack) == (float) -1.3396574E38F);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_integrated_ygyro_GET(pack) == (float) -2.4838656E38F);
    assert(p106_integrated_x_GET(pack) == (float) -1.7039473E38F);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)3588890225L);
    assert(p106_integrated_xgyro_GET(pack) == (float)2.4804219E38F);
    assert(p106_integrated_zgyro_GET(pack) == (float) -1.8573195E38F);
    assert(p106_time_usec_GET(pack) == (uint64_t)8085116288809715674L);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t) -18754);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p106_distance_GET(pack) == (float)2.0195664E38F);
    assert(p106_integrated_y_GET(pack) == (float) -2.9072106E36F);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)772180620L);
};


void c_CommunicationChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_abs_pressure_GET(pack) == (float) -1.983397E38F);
    assert(p107_xmag_GET(pack) == (float) -2.0344338E38F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)1709510610L);
    assert(p107_ymag_GET(pack) == (float)2.7447507E38F);
    assert(p107_xacc_GET(pack) == (float)5.2220746E36F);
    assert(p107_zmag_GET(pack) == (float) -1.4442925E38F);
    assert(p107_xgyro_GET(pack) == (float)2.6556436E38F);
    assert(p107_yacc_GET(pack) == (float)3.0703047E38F);
    assert(p107_zacc_GET(pack) == (float)1.0605947E38F);
    assert(p107_pressure_alt_GET(pack) == (float) -2.9468203E38F);
    assert(p107_temperature_GET(pack) == (float) -2.8445232E38F);
    assert(p107_diff_pressure_GET(pack) == (float)2.6270548E37F);
    assert(p107_time_usec_GET(pack) == (uint64_t)4746961554163647405L);
    assert(p107_zgyro_GET(pack) == (float) -9.527326E37F);
    assert(p107_ygyro_GET(pack) == (float) -8.873014E37F);
};


void c_CommunicationChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_zgyro_GET(pack) == (float)6.907673E37F);
    assert(p108_xacc_GET(pack) == (float) -3.3355366E38F);
    assert(p108_yacc_GET(pack) == (float) -1.0527436E38F);
    assert(p108_q1_GET(pack) == (float)2.446726E38F);
    assert(p108_std_dev_horz_GET(pack) == (float)2.3794005E38F);
    assert(p108_q4_GET(pack) == (float)2.2343956E38F);
    assert(p108_alt_GET(pack) == (float) -6.5912994E37F);
    assert(p108_ygyro_GET(pack) == (float)8.873777E37F);
    assert(p108_vd_GET(pack) == (float)1.1967096E38F);
    assert(p108_lat_GET(pack) == (float) -2.7190274E38F);
    assert(p108_zacc_GET(pack) == (float)1.3799184E38F);
    assert(p108_roll_GET(pack) == (float)2.8086282E38F);
    assert(p108_q3_GET(pack) == (float)1.9513299E38F);
    assert(p108_lon_GET(pack) == (float)2.7550702E38F);
    assert(p108_ve_GET(pack) == (float) -3.1299468E38F);
    assert(p108_std_dev_vert_GET(pack) == (float)1.6895107E38F);
    assert(p108_vn_GET(pack) == (float) -2.1820956E38F);
    assert(p108_pitch_GET(pack) == (float)2.3347925E38F);
    assert(p108_yaw_GET(pack) == (float)1.6333923E38F);
    assert(p108_xgyro_GET(pack) == (float)1.7617273E38F);
    assert(p108_q2_GET(pack) == (float)1.122196E38F);
};


void c_CommunicationChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)64799);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)38247);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)49);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)76);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)103);
};


void c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)103);
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)2);
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)195);
    {
        uint8_t exemplary[] =  {(uint8_t)240, (uint8_t)61, (uint8_t)166, (uint8_t)177, (uint8_t)10, (uint8_t)65, (uint8_t)101, (uint8_t)84, (uint8_t)164, (uint8_t)108, (uint8_t)28, (uint8_t)142, (uint8_t)251, (uint8_t)69, (uint8_t)143, (uint8_t)110, (uint8_t)248, (uint8_t)187, (uint8_t)169, (uint8_t)238, (uint8_t)34, (uint8_t)152, (uint8_t)116, (uint8_t)135, (uint8_t)139, (uint8_t)144, (uint8_t)139, (uint8_t)16, (uint8_t)137, (uint8_t)19, (uint8_t)187, (uint8_t)81, (uint8_t)241, (uint8_t)63, (uint8_t)39, (uint8_t)28, (uint8_t)118, (uint8_t)21, (uint8_t)254, (uint8_t)227, (uint8_t)212, (uint8_t)237, (uint8_t)23, (uint8_t)122, (uint8_t)26, (uint8_t)86, (uint8_t)157, (uint8_t)234, (uint8_t)133, (uint8_t)156, (uint8_t)118, (uint8_t)112, (uint8_t)216, (uint8_t)230, (uint8_t)182, (uint8_t)223, (uint8_t)242, (uint8_t)118, (uint8_t)24, (uint8_t)109, (uint8_t)99, (uint8_t)239, (uint8_t)145, (uint8_t)150, (uint8_t)65, (uint8_t)47, (uint8_t)174, (uint8_t)106, (uint8_t)64, (uint8_t)119, (uint8_t)114, (uint8_t)20, (uint8_t)187, (uint8_t)128, (uint8_t)129, (uint8_t)18, (uint8_t)177, (uint8_t)191, (uint8_t)237, (uint8_t)223, (uint8_t)121, (uint8_t)106, (uint8_t)99, (uint8_t)170, (uint8_t)25, (uint8_t)159, (uint8_t)73, (uint8_t)147, (uint8_t)55, (uint8_t)86, (uint8_t)22, (uint8_t)211, (uint8_t)7, (uint8_t)94, (uint8_t)117, (uint8_t)144, (uint8_t)108, (uint8_t)26, (uint8_t)107, (uint8_t)91, (uint8_t)228, (uint8_t)48, (uint8_t)125, (uint8_t)184, (uint8_t)0, (uint8_t)142, (uint8_t)26, (uint8_t)148, (uint8_t)171, (uint8_t)10, (uint8_t)186, (uint8_t)110, (uint8_t)57, (uint8_t)65, (uint8_t)80, (uint8_t)66, (uint8_t)79, (uint8_t)189, (uint8_t)12, (uint8_t)114, (uint8_t)203, (uint8_t)139, (uint8_t)188, (uint8_t)133, (uint8_t)72, (uint8_t)248, (uint8_t)43, (uint8_t)71, (uint8_t)193, (uint8_t)179, (uint8_t)209, (uint8_t)18, (uint8_t)31, (uint8_t)170, (uint8_t)161, (uint8_t)210, (uint8_t)143, (uint8_t)244, (uint8_t)91, (uint8_t)14, (uint8_t)139, (uint8_t)16, (uint8_t)107, (uint8_t)109, (uint8_t)227, (uint8_t)133, (uint8_t)207, (uint8_t)21, (uint8_t)184, (uint8_t)17, (uint8_t)201, (uint8_t)88, (uint8_t)1, (uint8_t)200, (uint8_t)93, (uint8_t)178, (uint8_t)32, (uint8_t)153, (uint8_t)190, (uint8_t)228, (uint8_t)117, (uint8_t)198, (uint8_t)71, (uint8_t)80, (uint8_t)96, (uint8_t)224, (uint8_t)248, (uint8_t)138, (uint8_t)254, (uint8_t)241, (uint8_t)72, (uint8_t)194, (uint8_t)110, (uint8_t)199, (uint8_t)251, (uint8_t)239, (uint8_t)219, (uint8_t)101, (uint8_t)26, (uint8_t)102, (uint8_t)97, (uint8_t)91, (uint8_t)65, (uint8_t)83, (uint8_t)20, (uint8_t)173, (uint8_t)88, (uint8_t)32, (uint8_t)195, (uint8_t)235, (uint8_t)153, (uint8_t)126, (uint8_t)237, (uint8_t)186, (uint8_t)234, (uint8_t)55, (uint8_t)131, (uint8_t)103, (uint8_t)97, (uint8_t)5, (uint8_t)4, (uint8_t)66, (uint8_t)164, (uint8_t)247, (uint8_t)66, (uint8_t)172, (uint8_t)178, (uint8_t)94, (uint8_t)224, (uint8_t)33, (uint8_t)152, (uint8_t)125, (uint8_t)248, (uint8_t)237, (uint8_t)15, (uint8_t)243, (uint8_t)169, (uint8_t)64, (uint8_t)248, (uint8_t)177, (uint8_t)197, (uint8_t)185, (uint8_t)128, (uint8_t)207, (uint8_t)143, (uint8_t)20, (uint8_t)41, (uint8_t)28, (uint8_t)230, (uint8_t)101, (uint8_t)83, (uint8_t)199, (uint8_t)23, (uint8_t)102, (uint8_t)75, (uint8_t)220, (uint8_t)66, (uint8_t)213, (uint8_t)74, (uint8_t)119, (uint8_t)106, (uint8_t)253, (uint8_t)85, (uint8_t)136, (uint8_t)94, (uint8_t)126, (uint8_t)75, (uint8_t)8, (uint8_t)13, (uint8_t)29, (uint8_t)193} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_tc1_GET(pack) == (int64_t)1802500019957033528L);
    assert(p111_ts1_GET(pack) == (int64_t) -2156372053642851945L);
};


void c_CommunicationChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_seq_GET(pack) == (uint32_t)2247674867L);
    assert(p112_time_usec_GET(pack) == (uint64_t)5420808730898669976L);
};


void c_CommunicationChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_vd_GET(pack) == (int16_t)(int16_t) -31136);
    assert(p113_time_usec_GET(pack) == (uint64_t)8040578514204565075L);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)22882);
    assert(p113_alt_GET(pack) == (int32_t)1097516311);
    assert(p113_lat_GET(pack) == (int32_t) -834441838);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)49274);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)41191);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t) -7121);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)16513);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)220);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)146);
    assert(p113_lon_GET(pack) == (int32_t)1708280858);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t) -7051);
};


void c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_integrated_zgyro_GET(pack) == (float) -3.3287698E38F);
    assert(p114_integrated_xgyro_GET(pack) == (float) -1.291841E38F);
    assert(p114_integrated_x_GET(pack) == (float) -3.839793E36F);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t)14931);
    assert(p114_integrated_y_GET(pack) == (float) -4.219411E37F);
    assert(p114_distance_GET(pack) == (float)3.2335023E38F);
    assert(p114_integrated_ygyro_GET(pack) == (float)1.7266328E38F);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)558565300L);
    assert(p114_time_usec_GET(pack) == (uint64_t)8140735110475398491L);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)3683950146L);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)164);
};


void c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)22105);
    assert(p115_time_usec_GET(pack) == (uint64_t)2603694701253781558L);
    assert(p115_pitchspeed_GET(pack) == (float)2.6683459E38F);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t)12606);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t) -13007);
    assert(p115_lon_GET(pack) == (int32_t)227522083);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)38056);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t) -9622);
    assert(p115_yawspeed_GET(pack) == (float) -2.4799467E38F);
    assert(p115_alt_GET(pack) == (int32_t)2072380285);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t) -22783);
    assert(p115_lat_GET(pack) == (int32_t) -1083544770);
    assert(p115_rollspeed_GET(pack) == (float)2.3352075E38F);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t)27926);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t)17409);
    {
        float exemplary[] =  {7.223162E37F, 8.250534E37F, -3.3024533E38F, -3.9507474E37F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t) -24532);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)2695585558L);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t)17563);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t)294);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t)24254);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t)25436);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t)3454);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t) -12694);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t) -10476);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t)15777);
};


void c_CommunicationChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)6458);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)31377);
};


void c_CommunicationChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)36446);
    assert(p118_size_GET(pack) == (uint32_t)531705031L);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)59194);
    assert(p118_time_utc_GET(pack) == (uint32_t)3445078965L);
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)23138);
};


void c_CommunicationChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_ofs_GET(pack) == (uint32_t)2399604853L);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)48);
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)33017);
    assert(p119_count_GET(pack) == (uint32_t)2440630713L);
};


void c_CommunicationChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)97);
    {
        uint8_t exemplary[] =  {(uint8_t)145, (uint8_t)160, (uint8_t)204, (uint8_t)139, (uint8_t)69, (uint8_t)134, (uint8_t)113, (uint8_t)67, (uint8_t)230, (uint8_t)86, (uint8_t)221, (uint8_t)188, (uint8_t)217, (uint8_t)11, (uint8_t)65, (uint8_t)116, (uint8_t)212, (uint8_t)52, (uint8_t)28, (uint8_t)66, (uint8_t)141, (uint8_t)20, (uint8_t)28, (uint8_t)154, (uint8_t)88, (uint8_t)98, (uint8_t)106, (uint8_t)100, (uint8_t)15, (uint8_t)247, (uint8_t)84, (uint8_t)201, (uint8_t)98, (uint8_t)19, (uint8_t)171, (uint8_t)17, (uint8_t)1, (uint8_t)101, (uint8_t)96, (uint8_t)169, (uint8_t)213, (uint8_t)214, (uint8_t)8, (uint8_t)218, (uint8_t)226, (uint8_t)179, (uint8_t)55, (uint8_t)167, (uint8_t)210, (uint8_t)230, (uint8_t)5, (uint8_t)251, (uint8_t)67, (uint8_t)233, (uint8_t)161, (uint8_t)223, (uint8_t)178, (uint8_t)53, (uint8_t)176, (uint8_t)25, (uint8_t)217, (uint8_t)80, (uint8_t)41, (uint8_t)180, (uint8_t)219, (uint8_t)44, (uint8_t)89, (uint8_t)7, (uint8_t)182, (uint8_t)142, (uint8_t)139, (uint8_t)25, (uint8_t)70, (uint8_t)115, (uint8_t)144, (uint8_t)174, (uint8_t)160, (uint8_t)138, (uint8_t)106, (uint8_t)148, (uint8_t)213, (uint8_t)170, (uint8_t)152, (uint8_t)48, (uint8_t)91, (uint8_t)12, (uint8_t)56, (uint8_t)17, (uint8_t)115, (uint8_t)208} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_ofs_GET(pack) == (uint32_t)2470467810L);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)11662);
};


void c_CommunicationChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)226);
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)192);
};


void c_CommunicationChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)95);
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)145);
};


void c_CommunicationChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)89);
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)120);
    {
        uint8_t exemplary[] =  {(uint8_t)150, (uint8_t)125, (uint8_t)223, (uint8_t)133, (uint8_t)85, (uint8_t)103, (uint8_t)250, (uint8_t)210, (uint8_t)245, (uint8_t)126, (uint8_t)243, (uint8_t)185, (uint8_t)154, (uint8_t)108, (uint8_t)2, (uint8_t)99, (uint8_t)250, (uint8_t)202, (uint8_t)204, (uint8_t)229, (uint8_t)10, (uint8_t)56, (uint8_t)175, (uint8_t)110, (uint8_t)47, (uint8_t)61, (uint8_t)62, (uint8_t)155, (uint8_t)1, (uint8_t)205, (uint8_t)40, (uint8_t)19, (uint8_t)241, (uint8_t)192, (uint8_t)223, (uint8_t)15, (uint8_t)250, (uint8_t)196, (uint8_t)187, (uint8_t)45, (uint8_t)194, (uint8_t)216, (uint8_t)114, (uint8_t)223, (uint8_t)84, (uint8_t)61, (uint8_t)120, (uint8_t)143, (uint8_t)101, (uint8_t)124, (uint8_t)81, (uint8_t)107, (uint8_t)34, (uint8_t)248, (uint8_t)131, (uint8_t)58, (uint8_t)223, (uint8_t)101, (uint8_t)33, (uint8_t)25, (uint8_t)151, (uint8_t)111, (uint8_t)165, (uint8_t)91, (uint8_t)117, (uint8_t)82, (uint8_t)22, (uint8_t)213, (uint8_t)183, (uint8_t)56, (uint8_t)177, (uint8_t)11, (uint8_t)88, (uint8_t)243, (uint8_t)18, (uint8_t)89, (uint8_t)177, (uint8_t)232, (uint8_t)13, (uint8_t)243, (uint8_t)218, (uint8_t)115, (uint8_t)191, (uint8_t)235, (uint8_t)121, (uint8_t)104, (uint8_t)248, (uint8_t)60, (uint8_t)53, (uint8_t)103, (uint8_t)36, (uint8_t)222, (uint8_t)218, (uint8_t)117, (uint8_t)140, (uint8_t)82, (uint8_t)177, (uint8_t)181, (uint8_t)138, (uint8_t)183, (uint8_t)191, (uint8_t)174, (uint8_t)130, (uint8_t)254, (uint8_t)151, (uint8_t)9, (uint8_t)167, (uint8_t)117, (uint8_t)180, (uint8_t)220} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_lat_GET(pack) == (int32_t) -392042254);
    assert(p124_lon_GET(pack) == (int32_t) -434981377);
    assert(p124_alt_GET(pack) == (int32_t)160942686);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)57414);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)174);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)39461);
    assert(p124_dgps_age_GET(pack) == (uint32_t)196371693L);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)38478);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)221);
    assert(p124_time_usec_GET(pack) == (uint64_t)7043316742595647009L);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)3066);
};


void c_CommunicationChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)42621);
    assert(p125_flags_GET(pack) == e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID);
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)34480);
};


void c_CommunicationChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)41905);
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL);
    assert(p126_baudrate_GET(pack) == (uint32_t)921433821L);
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)19);
    {
        uint8_t exemplary[] =  {(uint8_t)66, (uint8_t)15, (uint8_t)31, (uint8_t)165, (uint8_t)48, (uint8_t)126, (uint8_t)189, (uint8_t)44, (uint8_t)34, (uint8_t)70, (uint8_t)217, (uint8_t)73, (uint8_t)25, (uint8_t)80, (uint8_t)239, (uint8_t)180, (uint8_t)80, (uint8_t)200, (uint8_t)136, (uint8_t)247, (uint8_t)194, (uint8_t)36, (uint8_t)80, (uint8_t)208, (uint8_t)87, (uint8_t)255, (uint8_t)36, (uint8_t)75, (uint8_t)166, (uint8_t)104, (uint8_t)122, (uint8_t)236, (uint8_t)56, (uint8_t)204, (uint8_t)234, (uint8_t)102, (uint8_t)227, (uint8_t)203, (uint8_t)74, (uint8_t)132, (uint8_t)211, (uint8_t)30, (uint8_t)72, (uint8_t)206, (uint8_t)158, (uint8_t)64, (uint8_t)213, (uint8_t)193, (uint8_t)180, (uint8_t)51, (uint8_t)158, (uint8_t)89, (uint8_t)70, (uint8_t)216, (uint8_t)207, (uint8_t)66, (uint8_t)217, (uint8_t)9, (uint8_t)235, (uint8_t)91, (uint8_t)197, (uint8_t)143, (uint8_t)72, (uint8_t)96, (uint8_t)11, (uint8_t)12, (uint8_t)32, (uint8_t)189, (uint8_t)74, (uint8_t)221} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_flags_GET(pack) == e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY);
};


void c_CommunicationChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)31);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t)1294952083);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)157);
    assert(p127_tow_GET(pack) == (uint32_t)981767684L);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t) -1270752438);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t) -205967155);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)56590);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)2663193355L);
    assert(p127_accuracy_GET(pack) == (uint32_t)2844378272L);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t)531524515);
};


void c_CommunicationChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)3864587519L);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t)1351038531);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)61325);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -168354658);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t) -1587884937);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t)1542463182);
    assert(p128_tow_GET(pack) == (uint32_t)686691887L);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)88);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p128_accuracy_GET(pack) == (uint32_t)217863905L);
};


void c_CommunicationChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t) -4829);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t)24318);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t) -3370);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t) -10017);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)7034);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t) -29353);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t)10573);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)3671968546L);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t)20955);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t) -10897);
};


void c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)50);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)35042);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)15921);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)27498);
    assert(p130_size_GET(pack) == (uint32_t)3089642794L);
};


void c_CommunicationChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)36259);
    {
        uint8_t exemplary[] =  {(uint8_t)188, (uint8_t)71, (uint8_t)31, (uint8_t)26, (uint8_t)155, (uint8_t)207, (uint8_t)21, (uint8_t)68, (uint8_t)137, (uint8_t)226, (uint8_t)143, (uint8_t)217, (uint8_t)86, (uint8_t)121, (uint8_t)157, (uint8_t)189, (uint8_t)157, (uint8_t)228, (uint8_t)229, (uint8_t)14, (uint8_t)117, (uint8_t)124, (uint8_t)105, (uint8_t)242, (uint8_t)59, (uint8_t)229, (uint8_t)60, (uint8_t)29, (uint8_t)253, (uint8_t)66, (uint8_t)206, (uint8_t)23, (uint8_t)110, (uint8_t)206, (uint8_t)91, (uint8_t)171, (uint8_t)5, (uint8_t)231, (uint8_t)153, (uint8_t)152, (uint8_t)140, (uint8_t)160, (uint8_t)8, (uint8_t)89, (uint8_t)60, (uint8_t)116, (uint8_t)115, (uint8_t)247, (uint8_t)46, (uint8_t)109, (uint8_t)195, (uint8_t)83, (uint8_t)220, (uint8_t)21, (uint8_t)98, (uint8_t)115, (uint8_t)15, (uint8_t)26, (uint8_t)192, (uint8_t)166, (uint8_t)245, (uint8_t)223, (uint8_t)178, (uint8_t)3, (uint8_t)147, (uint8_t)51, (uint8_t)187, (uint8_t)9, (uint8_t)137, (uint8_t)126, (uint8_t)55, (uint8_t)42, (uint8_t)32, (uint8_t)86, (uint8_t)118, (uint8_t)58, (uint8_t)155, (uint8_t)172, (uint8_t)137, (uint8_t)165, (uint8_t)46, (uint8_t)75, (uint8_t)68, (uint8_t)234, (uint8_t)21, (uint8_t)229, (uint8_t)241, (uint8_t)130, (uint8_t)230, (uint8_t)110, (uint8_t)148, (uint8_t)4, (uint8_t)216, (uint8_t)149, (uint8_t)56, (uint8_t)25, (uint8_t)126, (uint8_t)80, (uint8_t)105, (uint8_t)68, (uint8_t)134, (uint8_t)202, (uint8_t)38, (uint8_t)28, (uint8_t)3, (uint8_t)78, (uint8_t)51, (uint8_t)152, (uint8_t)76, (uint8_t)157, (uint8_t)2, (uint8_t)123, (uint8_t)57, (uint8_t)92, (uint8_t)228, (uint8_t)248, (uint8_t)191, (uint8_t)80, (uint8_t)193, (uint8_t)206, (uint8_t)47, (uint8_t)56, (uint8_t)73, (uint8_t)61, (uint8_t)86, (uint8_t)200, (uint8_t)69, (uint8_t)96, (uint8_t)142, (uint8_t)0, (uint8_t)0, (uint8_t)167, (uint8_t)182, (uint8_t)223, (uint8_t)124, (uint8_t)174, (uint8_t)249, (uint8_t)43, (uint8_t)26, (uint8_t)202, (uint8_t)136, (uint8_t)97, (uint8_t)246, (uint8_t)98, (uint8_t)19, (uint8_t)115, (uint8_t)39, (uint8_t)15, (uint8_t)27, (uint8_t)5, (uint8_t)37, (uint8_t)196, (uint8_t)136, (uint8_t)154, (uint8_t)77, (uint8_t)56, (uint8_t)46, (uint8_t)206, (uint8_t)171, (uint8_t)123, (uint8_t)144, (uint8_t)190, (uint8_t)237, (uint8_t)111, (uint8_t)172, (uint8_t)25, (uint8_t)210, (uint8_t)215, (uint8_t)84, (uint8_t)221, (uint8_t)141, (uint8_t)15, (uint8_t)235, (uint8_t)31, (uint8_t)60, (uint8_t)237, (uint8_t)77, (uint8_t)66, (uint8_t)169, (uint8_t)68, (uint8_t)91, (uint8_t)66, (uint8_t)176, (uint8_t)162, (uint8_t)203, (uint8_t)215, (uint8_t)97, (uint8_t)164, (uint8_t)85, (uint8_t)71, (uint8_t)200, (uint8_t)104, (uint8_t)60, (uint8_t)216, (uint8_t)55, (uint8_t)103, (uint8_t)61, (uint8_t)4, (uint8_t)117, (uint8_t)242, (uint8_t)159, (uint8_t)161, (uint8_t)132, (uint8_t)4, (uint8_t)245, (uint8_t)148, (uint8_t)137, (uint8_t)210, (uint8_t)57, (uint8_t)112, (uint8_t)48, (uint8_t)39, (uint8_t)77, (uint8_t)121, (uint8_t)131, (uint8_t)155, (uint8_t)5, (uint8_t)235, (uint8_t)145, (uint8_t)206, (uint8_t)116, (uint8_t)91, (uint8_t)231, (uint8_t)247, (uint8_t)196, (uint8_t)155, (uint8_t)160, (uint8_t)98, (uint8_t)135, (uint8_t)242, (uint8_t)253, (uint8_t)56, (uint8_t)24, (uint8_t)98, (uint8_t)156, (uint8_t)157, (uint8_t)50, (uint8_t)233, (uint8_t)204, (uint8_t)113, (uint8_t)24, (uint8_t)224, (uint8_t)241, (uint8_t)18, (uint8_t)68, (uint8_t)150, (uint8_t)126, (uint8_t)36, (uint8_t)135, (uint8_t)42, (uint8_t)212, (uint8_t)67, (uint8_t)122} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_YAW_90);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)160);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)2242400218L);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)31401);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)211);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)7799);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)3223);
};


void c_CommunicationChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_lat_GET(pack) == (int32_t)1674825285);
    assert(p133_mask_GET(pack) == (uint64_t)724014654130113026L);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)60677);
    assert(p133_lon_GET(pack) == (int32_t) -740375538);
};


void c_CommunicationChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    {
        int16_t exemplary[] =  {(int16_t)10168, (int16_t)24181, (int16_t) -20530, (int16_t)4664, (int16_t)7164, (int16_t) -11506, (int16_t)12032, (int16_t)11300, (int16_t)8235, (int16_t)15854, (int16_t) -28226, (int16_t)18530, (int16_t) -14937, (int16_t) -30199, (int16_t)7233, (int16_t) -15838} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)58542);
    assert(p134_lon_GET(pack) == (int32_t) -268865400);
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p134_lat_GET(pack) == (int32_t)1764363521);
};


void c_CommunicationChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lon_GET(pack) == (int32_t) -828604579);
    assert(p135_lat_GET(pack) == (int32_t)110618588);
};


void c_CommunicationChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_lon_GET(pack) == (int32_t)1208102702);
    assert(p136_current_height_GET(pack) == (float)2.0727344E37F);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)15338);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)135);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)35571);
    assert(p136_lat_GET(pack) == (int32_t) -267791488);
    assert(p136_terrain_height_GET(pack) == (float) -5.1448715E37F);
};


void c_CommunicationChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_press_abs_GET(pack) == (float)3.3252278E38F);
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)3894017802L);
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t)9799);
    assert(p137_press_diff_GET(pack) == (float) -7.167319E37F);
};


void c_CommunicationChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {5.098307E37F, 1.5198215E38F, 7.2430863E37F, -1.4739468E38F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_z_GET(pack) == (float) -1.6967688E38F);
    assert(p138_x_GET(pack) == (float)1.6030427E38F);
    assert(p138_time_usec_GET(pack) == (uint64_t)5490821407814724854L);
    assert(p138_y_GET(pack) == (float)1.6723194E38F);
};


void c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)136);
    {
        float exemplary[] =  {-3.3818807E38F, -2.9335617E38F, 1.9238835E38F, 1.856661E38F, -9.109852E37F, 2.7458614E38F, -8.921846E37F, 9.478225E37F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_time_usec_GET(pack) == (uint64_t)7058463238121930420L);
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)228);
};


void c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {6.623479E37F, -3.151085E38F, -1.1793655E38F, 1.2202488E38F, -1.2448248E38F, -2.504331E38F, -2.2064822E38F, -1.8405727E38F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p140_time_usec_GET(pack) == (uint64_t)1952369112243036347L);
};


void c_CommunicationChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_altitude_relative_GET(pack) == (float) -2.086277E38F);
    assert(p141_altitude_local_GET(pack) == (float)1.8667983E38F);
    assert(p141_altitude_amsl_GET(pack) == (float)9.430941E37F);
    assert(p141_time_usec_GET(pack) == (uint64_t)4007552051117726338L);
    assert(p141_bottom_clearance_GET(pack) == (float) -1.6151378E38F);
    assert(p141_altitude_terrain_GET(pack) == (float)1.3030429E38F);
    assert(p141_altitude_monotonic_GET(pack) == (float)1.293918E38F);
};


void c_CommunicationChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)228);
    {
        uint8_t exemplary[] =  {(uint8_t)147, (uint8_t)149, (uint8_t)209, (uint8_t)81, (uint8_t)16, (uint8_t)33, (uint8_t)182, (uint8_t)10, (uint8_t)231, (uint8_t)21, (uint8_t)42, (uint8_t)212, (uint8_t)3, (uint8_t)8, (uint8_t)85, (uint8_t)171, (uint8_t)63, (uint8_t)77, (uint8_t)255, (uint8_t)163, (uint8_t)3, (uint8_t)5, (uint8_t)57, (uint8_t)107, (uint8_t)188, (uint8_t)22, (uint8_t)72, (uint8_t)60, (uint8_t)124, (uint8_t)238, (uint8_t)114, (uint8_t)195, (uint8_t)24, (uint8_t)204, (uint8_t)160, (uint8_t)147, (uint8_t)196, (uint8_t)120, (uint8_t)222, (uint8_t)12, (uint8_t)242, (uint8_t)138, (uint8_t)153, (uint8_t)113, (uint8_t)206, (uint8_t)48, (uint8_t)182, (uint8_t)55, (uint8_t)50, (uint8_t)72, (uint8_t)245, (uint8_t)88, (uint8_t)129, (uint8_t)183, (uint8_t)154, (uint8_t)116, (uint8_t)48, (uint8_t)106, (uint8_t)156, (uint8_t)161, (uint8_t)229, (uint8_t)136, (uint8_t)240, (uint8_t)82, (uint8_t)242, (uint8_t)51, (uint8_t)89, (uint8_t)83, (uint8_t)228, (uint8_t)201, (uint8_t)84, (uint8_t)111, (uint8_t)170, (uint8_t)118, (uint8_t)3, (uint8_t)202, (uint8_t)139, (uint8_t)79, (uint8_t)103, (uint8_t)61, (uint8_t)84, (uint8_t)196, (uint8_t)190, (uint8_t)110, (uint8_t)217, (uint8_t)94, (uint8_t)216, (uint8_t)154, (uint8_t)4, (uint8_t)2, (uint8_t)100, (uint8_t)141, (uint8_t)83, (uint8_t)237, (uint8_t)137, (uint8_t)69, (uint8_t)255, (uint8_t)245, (uint8_t)22, (uint8_t)113, (uint8_t)96, (uint8_t)110, (uint8_t)84, (uint8_t)187, (uint8_t)238, (uint8_t)134, (uint8_t)67, (uint8_t)232, (uint8_t)169, (uint8_t)158, (uint8_t)3, (uint8_t)241, (uint8_t)102, (uint8_t)53, (uint8_t)163, (uint8_t)161, (uint8_t)122, (uint8_t)25, (uint8_t)122, (uint8_t)204} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)74, (uint8_t)80, (uint8_t)69, (uint8_t)21, (uint8_t)115, (uint8_t)87, (uint8_t)108, (uint8_t)111, (uint8_t)84, (uint8_t)89, (uint8_t)151, (uint8_t)156, (uint8_t)176, (uint8_t)235, (uint8_t)23, (uint8_t)147, (uint8_t)100, (uint8_t)191, (uint8_t)20, (uint8_t)79, (uint8_t)194, (uint8_t)148, (uint8_t)206, (uint8_t)34, (uint8_t)237, (uint8_t)93, (uint8_t)11, (uint8_t)249, (uint8_t)114, (uint8_t)254, (uint8_t)35, (uint8_t)241, (uint8_t)208, (uint8_t)254, (uint8_t)171, (uint8_t)218, (uint8_t)136, (uint8_t)156, (uint8_t)17, (uint8_t)181, (uint8_t)150, (uint8_t)216, (uint8_t)74, (uint8_t)76, (uint8_t)225, (uint8_t)191, (uint8_t)213, (uint8_t)84, (uint8_t)74, (uint8_t)76, (uint8_t)250, (uint8_t)131, (uint8_t)112, (uint8_t)234, (uint8_t)97, (uint8_t)98, (uint8_t)61, (uint8_t)103, (uint8_t)8, (uint8_t)96, (uint8_t)53, (uint8_t)41, (uint8_t)42, (uint8_t)226, (uint8_t)151, (uint8_t)180, (uint8_t)211, (uint8_t)64, (uint8_t)238, (uint8_t)137, (uint8_t)46, (uint8_t)242, (uint8_t)55, (uint8_t)188, (uint8_t)207, (uint8_t)215, (uint8_t)148, (uint8_t)175, (uint8_t)111, (uint8_t)44, (uint8_t)166, (uint8_t)198, (uint8_t)246, (uint8_t)174, (uint8_t)115, (uint8_t)42, (uint8_t)7, (uint8_t)57, (uint8_t)82, (uint8_t)228, (uint8_t)218, (uint8_t)147, (uint8_t)161, (uint8_t)15, (uint8_t)124, (uint8_t)165, (uint8_t)152, (uint8_t)75, (uint8_t)176, (uint8_t)124, (uint8_t)94, (uint8_t)59, (uint8_t)78, (uint8_t)234, (uint8_t)22, (uint8_t)211, (uint8_t)39, (uint8_t)249, (uint8_t)99, (uint8_t)248, (uint8_t)125, (uint8_t)121, (uint8_t)84, (uint8_t)43, (uint8_t)143, (uint8_t)201, (uint8_t)86, (uint8_t)85, (uint8_t)155, (uint8_t)252} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)200);
};


void c_CommunicationChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)139150093L);
    assert(p143_press_diff_GET(pack) == (float) -1.2129372E38F);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t) -840);
    assert(p143_press_abs_GET(pack) == (float) -1.092815E38F);
};


void c_CommunicationChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {7.9504373E37F, 1.025579E36F, -3.108935E38F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)20);
    {
        float exemplary[] =  {-7.327044E37F, -2.1559611E38F, 3.2146299E38F, 5.4868694E37F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_timestamp_GET(pack) == (uint64_t)1662993579140645087L);
    {
        float exemplary[] =  {-2.4871958E38F, 9.138984E37F, 1.353687E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_custom_state_GET(pack) == (uint64_t)5296345303237363771L);
    assert(p144_lon_GET(pack) == (int32_t)218545717);
    {
        float exemplary[] =  {-1.5771143E37F, -1.5511651E38F, -2.5844528E38F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_alt_GET(pack) == (float) -1.4831561E38F);
    assert(p144_lat_GET(pack) == (int32_t)1920781709);
    {
        float exemplary[] =  {-2.2650533E38F, -1.1005457E38F, -1.1568884E38F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_pitch_rate_GET(pack) == (float)2.8002883E37F);
    assert(p146_airspeed_GET(pack) == (float)2.613459E37F);
    assert(p146_y_vel_GET(pack) == (float) -3.1724181E38F);
    assert(p146_x_acc_GET(pack) == (float) -1.1053693E38F);
    assert(p146_roll_rate_GET(pack) == (float) -2.5388368E38F);
    assert(p146_y_acc_GET(pack) == (float) -1.6723242E38F);
    assert(p146_z_pos_GET(pack) == (float) -2.8726934E38F);
    assert(p146_z_vel_GET(pack) == (float) -9.789147E37F);
    {
        float exemplary[] =  {-1.6913545E38F, 1.9884469E38F, -1.9107016E38F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_x_vel_GET(pack) == (float)9.955363E37F);
    assert(p146_yaw_rate_GET(pack) == (float) -2.4519535E38F);
    assert(p146_time_usec_GET(pack) == (uint64_t)8509353078048676886L);
    {
        float exemplary[] =  {3.326445E38F, -1.5882006E38F, 6.9287475E37F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_x_pos_GET(pack) == (float) -6.7378256E37F);
    assert(p146_z_acc_GET(pack) == (float)3.2074994E38F);
    assert(p146_y_pos_GET(pack) == (float) -1.4153027E38F);
    {
        float exemplary[] =  {3.3645337E38F, -3.158734E38F, 1.2938342E38F, -3.176179E38F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t)16112);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t)25);
    {
        uint16_t exemplary[] =  {(uint16_t)60504, (uint16_t)64551, (uint16_t)30296, (uint16_t)51308, (uint16_t)42299, (uint16_t)56612, (uint16_t)13666, (uint16_t)47894, (uint16_t)14475, (uint16_t)63758} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION);
    assert(p147_current_consumed_GET(pack) == (int32_t) -1854145163);
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p147_energy_consumed_GET(pack) == (int32_t)1645962199);
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t) -28272);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_NIMH);
};


void c_CommunicationChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_capabilities_GET(pack) == e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET);
    {
        uint8_t exemplary[] =  {(uint8_t)88, (uint8_t)82, (uint8_t)114, (uint8_t)66, (uint8_t)55, (uint8_t)209, (uint8_t)151, (uint8_t)170, (uint8_t)181, (uint8_t)253, (uint8_t)209, (uint8_t)137, (uint8_t)25, (uint8_t)84, (uint8_t)136, (uint8_t)12, (uint8_t)160, (uint8_t)80} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_board_version_GET(pack) == (uint32_t)3960252267L);
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)53413);
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)4218438231L);
    {
        uint8_t exemplary[] =  {(uint8_t)50, (uint8_t)190, (uint8_t)65, (uint8_t)243, (uint8_t)115, (uint8_t)38, (uint8_t)95, (uint8_t)86} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)230, (uint8_t)166, (uint8_t)101, (uint8_t)136, (uint8_t)149, (uint8_t)249, (uint8_t)67, (uint8_t)48} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_uid_GET(pack) == (uint64_t)2594835786036186779L);
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)62707);
    {
        uint8_t exemplary[] =  {(uint8_t)15, (uint8_t)82, (uint8_t)224, (uint8_t)123, (uint8_t)227, (uint8_t)53, (uint8_t)175, (uint8_t)156} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)359373337L);
    assert(p148_os_sw_version_GET(pack) == (uint32_t)3540612850L);
};


void c_CommunicationChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_x_TRY(ph) == (float) -1.2791094E38F);
    {
        float exemplary[] =  {1.6074327E38F, 1.587068E38F, -2.6074135E38F, -1.9125173E37F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)112);
    assert(p149_size_y_GET(pack) == (float) -1.0287314E38F);
    assert(p149_angle_x_GET(pack) == (float) -1.5790047E38F);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)32);
    assert(p149_distance_GET(pack) == (float)3.1731703E37F);
    assert(p149_time_usec_GET(pack) == (uint64_t)3524217692185537446L);
    assert(p149_z_TRY(ph) == (float) -4.625304E35F);
    assert(p149_y_TRY(ph) == (float)3.2696847E38F);
    assert(p149_angle_y_GET(pack) == (float)3.1041047E38F);
    assert(p149_size_x_GET(pack) == (float) -1.4439333E38F);
};


void c_CommunicationChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_pos_horiz_ratio_GET(pack) == (float)9.035926E37F);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float)1.1981848E38F);
    assert(p230_tas_ratio_GET(pack) == (float) -2.8322243E37F);
    assert(p230_vel_ratio_GET(pack) == (float) -3.2219419E38F);
    assert(p230_flags_GET(pack) == e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE);
    assert(p230_hagl_ratio_GET(pack) == (float) -1.0016293E38F);
    assert(p230_pos_vert_ratio_GET(pack) == (float) -6.073017E37F);
    assert(p230_time_usec_GET(pack) == (uint64_t)458122051073468469L);
    assert(p230_pos_vert_accuracy_GET(pack) == (float) -1.920556E38F);
    assert(p230_mag_ratio_GET(pack) == (float)1.3411761E38F);
};


void c_CommunicationChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_wind_alt_GET(pack) == (float) -3.2568875E38F);
    assert(p231_vert_accuracy_GET(pack) == (float)8.0174656E37F);
    assert(p231_var_vert_GET(pack) == (float)2.0974182E38F);
    assert(p231_wind_x_GET(pack) == (float) -3.1513508E38F);
    assert(p231_time_usec_GET(pack) == (uint64_t)2336586632304887778L);
    assert(p231_wind_z_GET(pack) == (float)1.3620373E38F);
    assert(p231_var_horiz_GET(pack) == (float) -2.9981192E38F);
    assert(p231_horiz_accuracy_GET(pack) == (float) -2.640934E38F);
    assert(p231_wind_y_GET(pack) == (float)3.0845192E38F);
};


void c_CommunicationChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_vn_GET(pack) == (float)1.900894E38F);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p232_ignore_flags_GET(pack) == e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)116);
    assert(p232_alt_GET(pack) == (float) -3.1577773E38F);
    assert(p232_time_usec_GET(pack) == (uint64_t)9157232551296201146L);
    assert(p232_lon_GET(pack) == (int32_t)1559763295);
    assert(p232_vert_accuracy_GET(pack) == (float)1.1758327E38F);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)619800937L);
    assert(p232_hdop_GET(pack) == (float) -1.2214809E38F);
    assert(p232_vdop_GET(pack) == (float) -2.4410793E38F);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)106);
    assert(p232_speed_accuracy_GET(pack) == (float)2.0534031E38F);
    assert(p232_ve_GET(pack) == (float)2.172752E38F);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)33874);
    assert(p232_vd_GET(pack) == (float) -5.312828E37F);
    assert(p232_horiz_accuracy_GET(pack) == (float) -2.2124544E37F);
    assert(p232_lat_GET(pack) == (int32_t)1243583318);
};


void c_CommunicationChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)247);
    {
        uint8_t exemplary[] =  {(uint8_t)96, (uint8_t)12, (uint8_t)210, (uint8_t)25, (uint8_t)37, (uint8_t)30, (uint8_t)151, (uint8_t)60, (uint8_t)151, (uint8_t)138, (uint8_t)199, (uint8_t)196, (uint8_t)43, (uint8_t)247, (uint8_t)110, (uint8_t)48, (uint8_t)114, (uint8_t)165, (uint8_t)127, (uint8_t)227, (uint8_t)131, (uint8_t)254, (uint8_t)201, (uint8_t)144, (uint8_t)16, (uint8_t)74, (uint8_t)119, (uint8_t)33, (uint8_t)147, (uint8_t)234, (uint8_t)254, (uint8_t)116, (uint8_t)137, (uint8_t)136, (uint8_t)152, (uint8_t)13, (uint8_t)146, (uint8_t)103, (uint8_t)95, (uint8_t)114, (uint8_t)100, (uint8_t)254, (uint8_t)59, (uint8_t)191, (uint8_t)232, (uint8_t)230, (uint8_t)229, (uint8_t)248, (uint8_t)64, (uint8_t)14, (uint8_t)245, (uint8_t)167, (uint8_t)197, (uint8_t)97, (uint8_t)44, (uint8_t)103, (uint8_t)191, (uint8_t)223, (uint8_t)141, (uint8_t)253, (uint8_t)140, (uint8_t)77, (uint8_t)168, (uint8_t)187, (uint8_t)137, (uint8_t)81, (uint8_t)176, (uint8_t)66, (uint8_t)170, (uint8_t)47, (uint8_t)58, (uint8_t)191, (uint8_t)128, (uint8_t)201, (uint8_t)101, (uint8_t)236, (uint8_t)35, (uint8_t)136, (uint8_t)170, (uint8_t)249, (uint8_t)159, (uint8_t)130, (uint8_t)108, (uint8_t)240, (uint8_t)225, (uint8_t)24, (uint8_t)131, (uint8_t)207, (uint8_t)156, (uint8_t)211, (uint8_t)87, (uint8_t)106, (uint8_t)62, (uint8_t)242, (uint8_t)7, (uint8_t)192, (uint8_t)197, (uint8_t)64, (uint8_t)100, (uint8_t)210, (uint8_t)82, (uint8_t)1, (uint8_t)21, (uint8_t)222, (uint8_t)33, (uint8_t)123, (uint8_t)90, (uint8_t)159, (uint8_t)106, (uint8_t)114, (uint8_t)217, (uint8_t)49, (uint8_t)36, (uint8_t)101, (uint8_t)176, (uint8_t)186, (uint8_t)0, (uint8_t)44, (uint8_t)168, (uint8_t)207, (uint8_t)99, (uint8_t)207, (uint8_t)106, (uint8_t)35, (uint8_t)171, (uint8_t)37, (uint8_t)249, (uint8_t)187, (uint8_t)35, (uint8_t)56, (uint8_t)252, (uint8_t)149, (uint8_t)191, (uint8_t)166, (uint8_t)212, (uint8_t)180, (uint8_t)82, (uint8_t)221, (uint8_t)138, (uint8_t)35, (uint8_t)91, (uint8_t)181, (uint8_t)144, (uint8_t)242, (uint8_t)36, (uint8_t)100, (uint8_t)101, (uint8_t)64, (uint8_t)148, (uint8_t)243, (uint8_t)54, (uint8_t)147, (uint8_t)242, (uint8_t)214, (uint8_t)92, (uint8_t)120, (uint8_t)211, (uint8_t)60, (uint8_t)19, (uint8_t)182, (uint8_t)108, (uint8_t)241, (uint8_t)134, (uint8_t)240, (uint8_t)219, (uint8_t)248, (uint8_t)245, (uint8_t)231, (uint8_t)253, (uint8_t)155, (uint8_t)127, (uint8_t)192, (uint8_t)55, (uint8_t)8, (uint8_t)58, (uint8_t)19, (uint8_t)71, (uint8_t)220, (uint8_t)172, (uint8_t)20} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)249);
};


void c_CommunicationChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)7103);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t) -10882);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)154);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t)8480);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)92);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)210);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)42910);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t) -27036);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)176);
    assert(p234_longitude_GET(pack) == (int32_t) -1585980856);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t) -101);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t) -5561);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p234_latitude_GET(pack) == (int32_t) -755283619);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t)54);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t)86);
    assert(p234_custom_mode_GET(pack) == (uint32_t)670235846L);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)214);
    assert(p234_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t)27);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t)7085);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED);
};


void c_CommunicationChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_time_usec_GET(pack) == (uint64_t)8970279177692227870L);
    assert(p241_vibration_y_GET(pack) == (float) -1.4091136E38F);
    assert(p241_vibration_z_GET(pack) == (float)1.9286985E38F);
    assert(p241_vibration_x_GET(pack) == (float) -2.0242742E37F);
    assert(p241_clipping_0_GET(pack) == (uint32_t)3631332488L);
    assert(p241_clipping_1_GET(pack) == (uint32_t)399335996L);
    assert(p241_clipping_2_GET(pack) == (uint32_t)1000508911L);
};


void c_CommunicationChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_approach_y_GET(pack) == (float) -2.6353497E38F);
    assert(p242_approach_z_GET(pack) == (float) -2.9492849E38F);
    {
        float exemplary[] =  {2.739841E38F, -3.1571601E37F, -3.0655292E38F, 1.358867E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_x_GET(pack) == (float) -3.1355708E37F);
    assert(p242_approach_x_GET(pack) == (float) -2.8365126E38F);
    assert(p242_z_GET(pack) == (float) -2.4530142E37F);
    assert(p242_altitude_GET(pack) == (int32_t) -1797860752);
    assert(p242_time_usec_TRY(ph) == (uint64_t)1171379104338470627L);
    assert(p242_latitude_GET(pack) == (int32_t)1630036899);
    assert(p242_y_GET(pack) == (float) -1.8440994E38F);
    assert(p242_longitude_GET(pack) == (int32_t) -879890244);
};


void c_CommunicationChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {1.053879E38F, -4.2280338E37F, -5.177782E37F, 2.2434531E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)157);
    assert(p243_time_usec_TRY(ph) == (uint64_t)4842925762552016569L);
    assert(p243_z_GET(pack) == (float) -9.575267E37F);
    assert(p243_approach_z_GET(pack) == (float)1.1586706E38F);
    assert(p243_altitude_GET(pack) == (int32_t)1978935974);
    assert(p243_longitude_GET(pack) == (int32_t)810225029);
    assert(p243_latitude_GET(pack) == (int32_t) -783338466);
    assert(p243_y_GET(pack) == (float)2.5845518E38F);
    assert(p243_approach_x_GET(pack) == (float)6.3864706E36F);
    assert(p243_approach_y_GET(pack) == (float) -3.1858033E38F);
    assert(p243_x_GET(pack) == (float) -1.5966577E37F);
};


void c_CommunicationChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)46746);
    assert(p244_interval_us_GET(pack) == (int32_t) -1544289579);
};


void c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW);
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED);
};


void c_CommunicationChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_ULTRA_LIGHT);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)1750075344L);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)7550);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t)12555);
    assert(p246_callsign_LEN(ph) == 3);
    {
        char16_t * exemplary = u"fgf";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_lon_GET(pack) == (int32_t)254333280);
    assert(p246_flags_GET(pack) == e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)56235);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)49319);
    assert(p246_altitude_GET(pack) == (int32_t)601224448);
    assert(p246_lat_GET(pack) == (int32_t) -1397242436);
};


void c_CommunicationChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_time_to_minimum_delta_GET(pack) == (float)2.74995E38F);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_HORIZONTALLY);
    assert(p247_altitude_minimum_delta_GET(pack) == (float) -2.820458E37F);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
    assert(p247_id_GET(pack) == (uint32_t)155619091L);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float) -1.9374388E38F);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE);
};


void c_CommunicationChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)216);
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)57662);
    {
        uint8_t exemplary[] =  {(uint8_t)189, (uint8_t)221, (uint8_t)70, (uint8_t)231, (uint8_t)172, (uint8_t)58, (uint8_t)242, (uint8_t)141, (uint8_t)109, (uint8_t)109, (uint8_t)90, (uint8_t)5, (uint8_t)246, (uint8_t)125, (uint8_t)188, (uint8_t)197, (uint8_t)40, (uint8_t)87, (uint8_t)242, (uint8_t)161, (uint8_t)106, (uint8_t)177, (uint8_t)76, (uint8_t)15, (uint8_t)130, (uint8_t)155, (uint8_t)155, (uint8_t)120, (uint8_t)155, (uint8_t)249, (uint8_t)120, (uint8_t)98, (uint8_t)142, (uint8_t)129, (uint8_t)2, (uint8_t)90, (uint8_t)106, (uint8_t)105, (uint8_t)233, (uint8_t)123, (uint8_t)110, (uint8_t)145, (uint8_t)179, (uint8_t)226, (uint8_t)89, (uint8_t)216, (uint8_t)137, (uint8_t)196, (uint8_t)243, (uint8_t)51, (uint8_t)239, (uint8_t)137, (uint8_t)13, (uint8_t)220, (uint8_t)8, (uint8_t)143, (uint8_t)135, (uint8_t)149, (uint8_t)236, (uint8_t)155, (uint8_t)128, (uint8_t)199, (uint8_t)147, (uint8_t)165, (uint8_t)129, (uint8_t)128, (uint8_t)137, (uint8_t)35, (uint8_t)101, (uint8_t)133, (uint8_t)233, (uint8_t)104, (uint8_t)170, (uint8_t)2, (uint8_t)153, (uint8_t)60, (uint8_t)201, (uint8_t)30, (uint8_t)231, (uint8_t)89, (uint8_t)92, (uint8_t)125, (uint8_t)81, (uint8_t)186, (uint8_t)43, (uint8_t)250, (uint8_t)25, (uint8_t)25, (uint8_t)90, (uint8_t)161, (uint8_t)74, (uint8_t)244, (uint8_t)54, (uint8_t)56, (uint8_t)84, (uint8_t)229, (uint8_t)152, (uint8_t)128, (uint8_t)149, (uint8_t)115, (uint8_t)28, (uint8_t)39, (uint8_t)59, (uint8_t)123, (uint8_t)12, (uint8_t)44, (uint8_t)95, (uint8_t)180, (uint8_t)87, (uint8_t)242, (uint8_t)217, (uint8_t)205, (uint8_t)44, (uint8_t)94, (uint8_t)123, (uint8_t)52, (uint8_t)214, (uint8_t)224, (uint8_t)122, (uint8_t)45, (uint8_t)2, (uint8_t)16, (uint8_t)181, (uint8_t)79, (uint8_t)17, (uint8_t)136, (uint8_t)51, (uint8_t)202, (uint8_t)182, (uint8_t)3, (uint8_t)86, (uint8_t)68, (uint8_t)200, (uint8_t)53, (uint8_t)160, (uint8_t)85, (uint8_t)157, (uint8_t)165, (uint8_t)167, (uint8_t)47, (uint8_t)192, (uint8_t)204, (uint8_t)217, (uint8_t)70, (uint8_t)97, (uint8_t)179, (uint8_t)158, (uint8_t)180, (uint8_t)20, (uint8_t)3, (uint8_t)169, (uint8_t)10, (uint8_t)41, (uint8_t)242, (uint8_t)175, (uint8_t)142, (uint8_t)78, (uint8_t)105, (uint8_t)38, (uint8_t)63, (uint8_t)134, (uint8_t)254, (uint8_t)207, (uint8_t)190, (uint8_t)216, (uint8_t)69, (uint8_t)194, (uint8_t)77, (uint8_t)146, (uint8_t)95, (uint8_t)178, (uint8_t)36, (uint8_t)73, (uint8_t)13, (uint8_t)210, (uint8_t)31, (uint8_t)8, (uint8_t)240, (uint8_t)168, (uint8_t)15, (uint8_t)230, (uint8_t)157, (uint8_t)158, (uint8_t)3, (uint8_t)37, (uint8_t)146, (uint8_t)6, (uint8_t)253, (uint8_t)185, (uint8_t)57, (uint8_t)199, (uint8_t)99, (uint8_t)108, (uint8_t)45, (uint8_t)141, (uint8_t)134, (uint8_t)86, (uint8_t)205, (uint8_t)24, (uint8_t)104, (uint8_t)201, (uint8_t)248, (uint8_t)202, (uint8_t)86, (uint8_t)248, (uint8_t)61, (uint8_t)217, (uint8_t)104, (uint8_t)249, (uint8_t)71, (uint8_t)253, (uint8_t)142, (uint8_t)143, (uint8_t)141, (uint8_t)200, (uint8_t)20, (uint8_t)181, (uint8_t)72, (uint8_t)220, (uint8_t)192, (uint8_t)90, (uint8_t)217, (uint8_t)52, (uint8_t)155, (uint8_t)33, (uint8_t)72, (uint8_t)12, (uint8_t)185, (uint8_t)208, (uint8_t)242, (uint8_t)83, (uint8_t)164, (uint8_t)44, (uint8_t)38, (uint8_t)159, (uint8_t)226, (uint8_t)95, (uint8_t)64, (uint8_t)236, (uint8_t)151, (uint8_t)154, (uint8_t)147, (uint8_t)37, (uint8_t)119, (uint8_t)137, (uint8_t)154, (uint8_t)37, (uint8_t)60, (uint8_t)188} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)156);
};


void c_CommunicationChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    {
        int8_t exemplary[] =  {(int8_t) -17, (int8_t)59, (int8_t)93, (int8_t) -9, (int8_t) -119, (int8_t)116, (int8_t)127, (int8_t) -4, (int8_t) -51, (int8_t) -115, (int8_t)124, (int8_t) -49, (int8_t)94, (int8_t) -124, (int8_t)88, (int8_t) -70, (int8_t)124, (int8_t) -52, (int8_t)126, (int8_t) -81, (int8_t) -36, (int8_t)30, (int8_t) -70, (int8_t) -93, (int8_t) -23, (int8_t) -45, (int8_t) -60, (int8_t) -111, (int8_t) -43, (int8_t)72, (int8_t)90, (int8_t) -2} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)189);
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)49165);
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)220);
};


void c_CommunicationChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_y_GET(pack) == (float)1.8717147E38F);
    assert(p250_x_GET(pack) == (float)2.0048541E38F);
    assert(p250_z_GET(pack) == (float)9.525829E37F);
    assert(p250_time_usec_GET(pack) == (uint64_t)3699949712977014533L);
    assert(p250_name_LEN(ph) == 8);
    {
        char16_t * exemplary = u"sbjcoueh";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_name_LEN(ph) == 2);
    {
        char16_t * exemplary = u"Ge";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_value_GET(pack) == (float) -3.1888672E38F);
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)3383671205L);
};


void c_CommunicationChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_value_GET(pack) == (int32_t)532688143);
    assert(p252_name_LEN(ph) == 10);
    {
        char16_t * exemplary = u"vayzknshbf";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)3439804464L);
};


void c_CommunicationChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_text_LEN(ph) == 32);
    {
        char16_t * exemplary = u"uvBgkrItoetvqddomOqwourdsugjxmvc";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_ERROR);
};


void c_CommunicationChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_value_GET(pack) == (float)9.326201E37F);
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)76);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)2074052379L);
};


void c_CommunicationChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)4217961436245929757L);
    {
        uint8_t exemplary[] =  {(uint8_t)54, (uint8_t)57, (uint8_t)112, (uint8_t)233, (uint8_t)128, (uint8_t)109, (uint8_t)154, (uint8_t)214, (uint8_t)5, (uint8_t)195, (uint8_t)186, (uint8_t)27, (uint8_t)181, (uint8_t)226, (uint8_t)107, (uint8_t)39, (uint8_t)214, (uint8_t)37, (uint8_t)118, (uint8_t)30, (uint8_t)113, (uint8_t)65, (uint8_t)192, (uint8_t)97, (uint8_t)164, (uint8_t)84, (uint8_t)22, (uint8_t)110, (uint8_t)203, (uint8_t)166, (uint8_t)195, (uint8_t)156} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)3440392118L);
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)217);
    assert(p257_last_change_ms_GET(pack) == (uint32_t)1255025544L);
};


void c_CommunicationChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)171);
    assert(p258_tune_LEN(ph) == 1);
    {
        char16_t * exemplary = u"r";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)247);
};


void c_CommunicationChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_cam_definition_uri_LEN(ph) == 115);
    {
        char16_t * exemplary = u"mdefrpyjoohupuFqyuwokrulvdgwxlpjbmMkGnmpGuonvleezXkrSllejpjaayuSzkTauDLpjeBatlEpgszmluCsmtjwibhzjymiGuaftiXksvwdkwe";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 230);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)27347);
    assert(p259_flags_GET(pack) == e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)186);
    {
        uint8_t exemplary[] =  {(uint8_t)48, (uint8_t)170, (uint8_t)129, (uint8_t)135, (uint8_t)120, (uint8_t)107, (uint8_t)253, (uint8_t)209, (uint8_t)165, (uint8_t)31, (uint8_t)136, (uint8_t)54, (uint8_t)138, (uint8_t)98, (uint8_t)81, (uint8_t)73, (uint8_t)73, (uint8_t)61, (uint8_t)105, (uint8_t)57, (uint8_t)50, (uint8_t)8, (uint8_t)213, (uint8_t)195, (uint8_t)126, (uint8_t)17, (uint8_t)64, (uint8_t)78, (uint8_t)159, (uint8_t)174, (uint8_t)117, (uint8_t)88} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)196, (uint8_t)145, (uint8_t)37, (uint8_t)159, (uint8_t)249, (uint8_t)110, (uint8_t)212, (uint8_t)31, (uint8_t)206, (uint8_t)57, (uint8_t)102, (uint8_t)96, (uint8_t)167, (uint8_t)109, (uint8_t)86, (uint8_t)25, (uint8_t)209, (uint8_t)191, (uint8_t)185, (uint8_t)180, (uint8_t)168, (uint8_t)149, (uint8_t)84, (uint8_t)84, (uint8_t)49, (uint8_t)247, (uint8_t)190, (uint8_t)153, (uint8_t)156, (uint8_t)5, (uint8_t)221, (uint8_t)179} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)13013);
    assert(p259_sensor_size_v_GET(pack) == (float)2.1928816E38F);
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)13913);
    assert(p259_sensor_size_h_GET(pack) == (float) -1.0950048E38F);
    assert(p259_focal_length_GET(pack) == (float)2.3028774E38F);
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)1908702952L);
    assert(p259_firmware_version_GET(pack) == (uint32_t)3414453684L);
};


void c_CommunicationChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY);
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)3844437543L);
};


void c_CommunicationChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_read_speed_GET(pack) == (float) -3.0214024E38F);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)2);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)369446823L);
    assert(p261_used_capacity_GET(pack) == (float) -2.1259401E38F);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)248);
    assert(p261_available_capacity_GET(pack) == (float)2.9172966E37F);
    assert(p261_write_speed_GET(pack) == (float)2.4142434E38F);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p261_total_capacity_GET(pack) == (float) -2.9027678E38F);
};


void c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)2179558520L);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)177);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)448837371L);
    assert(p262_available_capacity_GET(pack) == (float) -2.5149407E38F);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)9);
    assert(p262_image_interval_GET(pack) == (float) -3.6851128E37F);
};


void c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_file_url_LEN(ph) == 24);
    {
        char16_t * exemplary = u"bluvbymxvqpzqteMewiJlduj";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 48);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)2260959201L);
    assert(p263_lat_GET(pack) == (int32_t)1226509761);
    assert(p263_time_utc_GET(pack) == (uint64_t)3494969922375431417L);
    assert(p263_image_index_GET(pack) == (int32_t) -1858746548);
    assert(p263_relative_alt_GET(pack) == (int32_t)188459201);
    assert(p263_lon_GET(pack) == (int32_t) -2129276720);
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t)123);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p263_alt_GET(pack) == (int32_t)788043192);
    {
        float exemplary[] =  {-2.1644392E38F, 4.3834475E37F, -7.081031E37F, 2.0900936E38F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_flight_uuid_GET(pack) == (uint64_t)1747591698458444238L);
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)5346752962439253047L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)7042662703160545242L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)55443782L);
};


void c_CommunicationChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_roll_GET(pack) == (float) -4.428499E37F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)969844230L);
    assert(p265_pitch_GET(pack) == (float)3.2605223E38F);
    assert(p265_yaw_GET(pack) == (float) -2.1447638E38F);
};


void c_CommunicationChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)228);
    {
        uint8_t exemplary[] =  {(uint8_t)108, (uint8_t)187, (uint8_t)178, (uint8_t)254, (uint8_t)138, (uint8_t)250, (uint8_t)121, (uint8_t)106, (uint8_t)221, (uint8_t)220, (uint8_t)245, (uint8_t)160, (uint8_t)33, (uint8_t)163, (uint8_t)117, (uint8_t)56, (uint8_t)58, (uint8_t)202, (uint8_t)10, (uint8_t)173, (uint8_t)2, (uint8_t)213, (uint8_t)244, (uint8_t)144, (uint8_t)139, (uint8_t)178, (uint8_t)147, (uint8_t)181, (uint8_t)240, (uint8_t)167, (uint8_t)230, (uint8_t)26, (uint8_t)88, (uint8_t)18, (uint8_t)133, (uint8_t)250, (uint8_t)198, (uint8_t)33, (uint8_t)197, (uint8_t)177, (uint8_t)153, (uint8_t)126, (uint8_t)241, (uint8_t)115, (uint8_t)81, (uint8_t)60, (uint8_t)43, (uint8_t)142, (uint8_t)12, (uint8_t)242, (uint8_t)169, (uint8_t)11, (uint8_t)175, (uint8_t)61, (uint8_t)115, (uint8_t)67, (uint8_t)72, (uint8_t)63, (uint8_t)9, (uint8_t)24, (uint8_t)198, (uint8_t)108, (uint8_t)26, (uint8_t)205, (uint8_t)188, (uint8_t)81, (uint8_t)86, (uint8_t)208, (uint8_t)208, (uint8_t)63, (uint8_t)175, (uint8_t)63, (uint8_t)25, (uint8_t)185, (uint8_t)142, (uint8_t)68, (uint8_t)222, (uint8_t)42, (uint8_t)30, (uint8_t)183, (uint8_t)34, (uint8_t)233, (uint8_t)222, (uint8_t)79, (uint8_t)64, (uint8_t)130, (uint8_t)198, (uint8_t)252, (uint8_t)26, (uint8_t)159, (uint8_t)109, (uint8_t)127, (uint8_t)215, (uint8_t)139, (uint8_t)11, (uint8_t)116, (uint8_t)191, (uint8_t)173, (uint8_t)96, (uint8_t)135, (uint8_t)33, (uint8_t)111, (uint8_t)23, (uint8_t)199, (uint8_t)5, (uint8_t)97, (uint8_t)192, (uint8_t)0, (uint8_t)252, (uint8_t)114, (uint8_t)234, (uint8_t)200, (uint8_t)108, (uint8_t)214, (uint8_t)6, (uint8_t)114, (uint8_t)212, (uint8_t)60, (uint8_t)81, (uint8_t)86, (uint8_t)10, (uint8_t)143, (uint8_t)90, (uint8_t)124, (uint8_t)101, (uint8_t)184, (uint8_t)32, (uint8_t)84, (uint8_t)153, (uint8_t)180, (uint8_t)205, (uint8_t)69, (uint8_t)216, (uint8_t)95, (uint8_t)109, (uint8_t)116, (uint8_t)244, (uint8_t)160, (uint8_t)73, (uint8_t)16, (uint8_t)56, (uint8_t)28, (uint8_t)154, (uint8_t)32, (uint8_t)7, (uint8_t)8, (uint8_t)243, (uint8_t)129, (uint8_t)0, (uint8_t)11, (uint8_t)168, (uint8_t)67, (uint8_t)25, (uint8_t)219, (uint8_t)191, (uint8_t)140, (uint8_t)75, (uint8_t)89, (uint8_t)60, (uint8_t)151, (uint8_t)191, (uint8_t)175, (uint8_t)26, (uint8_t)213, (uint8_t)99, (uint8_t)110, (uint8_t)227, (uint8_t)55, (uint8_t)133, (uint8_t)103, (uint8_t)201, (uint8_t)114, (uint8_t)45, (uint8_t)253, (uint8_t)116, (uint8_t)240, (uint8_t)183, (uint8_t)244, (uint8_t)164, (uint8_t)62, (uint8_t)25, (uint8_t)57, (uint8_t)249, (uint8_t)131, (uint8_t)60, (uint8_t)250, (uint8_t)234, (uint8_t)209, (uint8_t)242, (uint8_t)229, (uint8_t)254, (uint8_t)111, (uint8_t)53, (uint8_t)63, (uint8_t)146, (uint8_t)246, (uint8_t)199, (uint8_t)201, (uint8_t)211, (uint8_t)220, (uint8_t)184, (uint8_t)215, (uint8_t)249, (uint8_t)113, (uint8_t)103, (uint8_t)246, (uint8_t)116, (uint8_t)55, (uint8_t)66, (uint8_t)150, (uint8_t)200, (uint8_t)139, (uint8_t)40, (uint8_t)118, (uint8_t)179, (uint8_t)134, (uint8_t)172, (uint8_t)42, (uint8_t)195, (uint8_t)70, (uint8_t)233, (uint8_t)110, (uint8_t)39, (uint8_t)145, (uint8_t)45, (uint8_t)15, (uint8_t)60, (uint8_t)42, (uint8_t)243, (uint8_t)254, (uint8_t)147, (uint8_t)145, (uint8_t)176, (uint8_t)71, (uint8_t)15, (uint8_t)195, (uint8_t)238, (uint8_t)200, (uint8_t)241, (uint8_t)184, (uint8_t)200, (uint8_t)167, (uint8_t)179, (uint8_t)54, (uint8_t)64, (uint8_t)122, (uint8_t)150, (uint8_t)138, (uint8_t)64} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)28672);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)251);
};


void c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)14249);
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)107);
    {
        uint8_t exemplary[] =  {(uint8_t)120, (uint8_t)50, (uint8_t)187, (uint8_t)51, (uint8_t)249, (uint8_t)105, (uint8_t)124, (uint8_t)155, (uint8_t)202, (uint8_t)143, (uint8_t)82, (uint8_t)124, (uint8_t)144, (uint8_t)3, (uint8_t)139, (uint8_t)218, (uint8_t)4, (uint8_t)69, (uint8_t)97, (uint8_t)11, (uint8_t)45, (uint8_t)249, (uint8_t)64, (uint8_t)97, (uint8_t)100, (uint8_t)190, (uint8_t)48, (uint8_t)192, (uint8_t)119, (uint8_t)208, (uint8_t)229, (uint8_t)75, (uint8_t)116, (uint8_t)12, (uint8_t)219, (uint8_t)16, (uint8_t)13, (uint8_t)255, (uint8_t)36, (uint8_t)113, (uint8_t)126, (uint8_t)1, (uint8_t)143, (uint8_t)15, (uint8_t)234, (uint8_t)125, (uint8_t)136, (uint8_t)74, (uint8_t)230, (uint8_t)27, (uint8_t)233, (uint8_t)146, (uint8_t)52, (uint8_t)152, (uint8_t)75, (uint8_t)89, (uint8_t)147, (uint8_t)91, (uint8_t)72, (uint8_t)38, (uint8_t)169, (uint8_t)12, (uint8_t)197, (uint8_t)86, (uint8_t)39, (uint8_t)157, (uint8_t)181, (uint8_t)110, (uint8_t)131, (uint8_t)223, (uint8_t)231, (uint8_t)88, (uint8_t)253, (uint8_t)99, (uint8_t)208, (uint8_t)47, (uint8_t)145, (uint8_t)206, (uint8_t)209, (uint8_t)158, (uint8_t)210, (uint8_t)0, (uint8_t)198, (uint8_t)7, (uint8_t)213, (uint8_t)32, (uint8_t)30, (uint8_t)43, (uint8_t)29, (uint8_t)110, (uint8_t)4, (uint8_t)80, (uint8_t)42, (uint8_t)248, (uint8_t)51, (uint8_t)95, (uint8_t)5, (uint8_t)217, (uint8_t)157, (uint8_t)23, (uint8_t)21, (uint8_t)119, (uint8_t)57, (uint8_t)131, (uint8_t)182, (uint8_t)109, (uint8_t)187, (uint8_t)104, (uint8_t)128, (uint8_t)186, (uint8_t)225, (uint8_t)44, (uint8_t)236, (uint8_t)157, (uint8_t)124, (uint8_t)224, (uint8_t)25, (uint8_t)68, (uint8_t)105, (uint8_t)206, (uint8_t)18, (uint8_t)186, (uint8_t)69, (uint8_t)39, (uint8_t)179, (uint8_t)204, (uint8_t)166, (uint8_t)205, (uint8_t)141, (uint8_t)6, (uint8_t)120, (uint8_t)118, (uint8_t)17, (uint8_t)147, (uint8_t)153, (uint8_t)90, (uint8_t)251, (uint8_t)174, (uint8_t)110, (uint8_t)199, (uint8_t)29, (uint8_t)157, (uint8_t)96, (uint8_t)226, (uint8_t)189, (uint8_t)59, (uint8_t)145, (uint8_t)119, (uint8_t)103, (uint8_t)243, (uint8_t)23, (uint8_t)192, (uint8_t)25, (uint8_t)22, (uint8_t)126, (uint8_t)19, (uint8_t)171, (uint8_t)112, (uint8_t)10, (uint8_t)85, (uint8_t)4, (uint8_t)179, (uint8_t)192, (uint8_t)93, (uint8_t)217, (uint8_t)208, (uint8_t)207, (uint8_t)100, (uint8_t)151, (uint8_t)131, (uint8_t)229, (uint8_t)5, (uint8_t)40, (uint8_t)42, (uint8_t)186, (uint8_t)144, (uint8_t)252, (uint8_t)145, (uint8_t)232, (uint8_t)144, (uint8_t)14, (uint8_t)67, (uint8_t)221, (uint8_t)188, (uint8_t)108, (uint8_t)52, (uint8_t)104, (uint8_t)13, (uint8_t)18, (uint8_t)227, (uint8_t)181, (uint8_t)147, (uint8_t)215, (uint8_t)164, (uint8_t)213, (uint8_t)86, (uint8_t)44, (uint8_t)139, (uint8_t)88, (uint8_t)61, (uint8_t)218, (uint8_t)126, (uint8_t)243, (uint8_t)95, (uint8_t)146, (uint8_t)153, (uint8_t)172, (uint8_t)62, (uint8_t)188, (uint8_t)170, (uint8_t)14, (uint8_t)252, (uint8_t)203, (uint8_t)207, (uint8_t)76, (uint8_t)107, (uint8_t)33, (uint8_t)33, (uint8_t)205, (uint8_t)12, (uint8_t)66, (uint8_t)19, (uint8_t)209, (uint8_t)181, (uint8_t)151, (uint8_t)201, (uint8_t)60, (uint8_t)89, (uint8_t)122, (uint8_t)113, (uint8_t)214, (uint8_t)1, (uint8_t)186, (uint8_t)113, (uint8_t)225, (uint8_t)193, (uint8_t)154, (uint8_t)249, (uint8_t)123, (uint8_t)141, (uint8_t)51, (uint8_t)148, (uint8_t)201, (uint8_t)173, (uint8_t)14, (uint8_t)3, (uint8_t)144, (uint8_t)187, (uint8_t)146} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)93);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)248);
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)32098);
};


void c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_uri_LEN(ph) == 157);
    {
        char16_t * exemplary = u"PhlVCtqgkwFyxaezcikuqpygkatajKvuaizeBOszosltRjbnynpxtiujlruaZacjucrkfqltodohvvdhhodXwrsnmRknmdhFgcxjksuwMiqszurlcvkmRnftsbbvgyzhqnehttFyiumNegfgicqxutmdrfpao";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 314);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p269_bitrate_GET(pack) == (uint32_t)2475350522L);
    assert(p269_framerate_GET(pack) == (float) -9.563977E37F);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)93);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)26421);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)33494);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)4305);
};


void c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)52540);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)49948);
    assert(p270_bitrate_GET(pack) == (uint32_t)2235839958L);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)163);
    assert(p270_framerate_GET(pack) == (float) -3.143124E36F);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)51394);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)103);
    assert(p270_uri_LEN(ph) == 3);
    {
        char16_t * exemplary = u"rwa";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)246);
};


void c_CommunicationChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_password_LEN(ph) == 32);
    {
        char16_t * exemplary = u"srrmwlxsuAxAydpuxGanPfcmLjrbnsmo";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_ssid_LEN(ph) == 29);
    {
        char16_t * exemplary = u"hjiiCfoFhtdfyhKzHospevqrriqxe";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 58);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)235, (uint8_t)204, (uint8_t)126, (uint8_t)95, (uint8_t)16, (uint8_t)50, (uint8_t)187, (uint8_t)31} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)18937);
    {
        uint8_t exemplary[] =  {(uint8_t)156, (uint8_t)152, (uint8_t)245, (uint8_t)129, (uint8_t)117, (uint8_t)171, (uint8_t)144, (uint8_t)173} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)56447);
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)56162);
};


void c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)64593);
    assert(p310_uptime_sec_GET(pack) == (uint32_t)1963039189L);
    assert(p310_time_usec_GET(pack) == (uint64_t)63184929590684970L);
};


void c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_uptime_sec_GET(pack) == (uint32_t)3551104389L);
    {
        uint8_t exemplary[] =  {(uint8_t)29, (uint8_t)85, (uint8_t)104, (uint8_t)74, (uint8_t)15, (uint8_t)227, (uint8_t)197, (uint8_t)51, (uint8_t)176, (uint8_t)16, (uint8_t)246, (uint8_t)177, (uint8_t)217, (uint8_t)103, (uint8_t)120, (uint8_t)154} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p311_time_usec_GET(pack) == (uint64_t)2283214408316546566L);
    assert(p311_name_LEN(ph) == 69);
    {
        char16_t * exemplary = u"umxabCydFrbClZmjKlqkaTlmlcoficlfgygbpfxvytqzvtrKhnfAuavhopWmtkzowrdqq";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 138);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)4275976630L);
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)89);
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)115);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"kdnhtxbocxdrk";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t)14063);
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)124);
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)4);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)40);
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)168);
};


void c_CommunicationChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)34217);
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16);
    assert(p322_param_value_LEN(ph) == 127);
    {
        char16_t * exemplary = u"zrmHptfcnLdcuztPmrewvysquceluFyQaarWptmhbqapTeefxmmjlpiivsudozkyzlyjNviecaksjszsbbjbihnsojgebukmhdbcoqixxnZjJgiairkvzfxfuasmwai";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 254);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)21368);
    assert(p322_param_id_LEN(ph) == 8);
    {
        char16_t * exemplary = u"kyhvygei";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_param_value_LEN(ph) == 47);
    {
        char16_t * exemplary = u"uweywjnyqEcuvszsolqefpaldnsklqfduhyjbjYrwtmbbxl";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 94);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"tquvrogglfosb";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16);
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)66);
};


void c_CommunicationChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_FAILED);
    assert(p324_param_value_LEN(ph) == 89);
    {
        char16_t * exemplary = u"csGcswcnFcuktghfprlskXXsjgGSkxqrxelzAiwmuoxrxbwxrhmxlyEticwmynEGisceaUuqtervsxkxfrqjsiyac";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 178);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32);
    assert(p324_param_id_LEN(ph) == 8);
    {
        char16_t * exemplary = u"alerjxqn";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    {
        uint16_t exemplary[] =  {(uint16_t)48338, (uint16_t)41453, (uint16_t)204, (uint16_t)3262, (uint16_t)63208, (uint16_t)31564, (uint16_t)52364, (uint16_t)14614, (uint16_t)9421, (uint16_t)25658, (uint16_t)52815, (uint16_t)2739, (uint16_t)60154, (uint16_t)7059, (uint16_t)22026, (uint16_t)30192, (uint16_t)18286, (uint16_t)55210, (uint16_t)31923, (uint16_t)26647, (uint16_t)6048, (uint16_t)11687, (uint16_t)12371, (uint16_t)5036, (uint16_t)39164, (uint16_t)9655, (uint16_t)61350, (uint16_t)32016, (uint16_t)23997, (uint16_t)54990, (uint16_t)79, (uint16_t)26926, (uint16_t)47170, (uint16_t)64320, (uint16_t)2815, (uint16_t)52653, (uint16_t)38351, (uint16_t)2387, (uint16_t)32453, (uint16_t)45805, (uint16_t)47179, (uint16_t)41261, (uint16_t)3002, (uint16_t)35159, (uint16_t)40148, (uint16_t)2258, (uint16_t)63676, (uint16_t)3539, (uint16_t)37164, (uint16_t)55421, (uint16_t)36815, (uint16_t)8118, (uint16_t)30171, (uint16_t)8596, (uint16_t)63435, (uint16_t)24008, (uint16_t)19694, (uint16_t)56647, (uint16_t)46717, (uint16_t)21273, (uint16_t)63002, (uint16_t)60743, (uint16_t)62874, (uint16_t)52053, (uint16_t)56619, (uint16_t)53251, (uint16_t)60366, (uint16_t)47190, (uint16_t)59409, (uint16_t)12000, (uint16_t)9942, (uint16_t)19973} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR);
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)43236);
    assert(p330_time_usec_GET(pack) == (uint64_t)5741823915613362407L);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)49923);
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)106);
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
        p0_custom_mode_SET((uint32_t)3963663799L, PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_OPENPILOT, PH.base.pack) ;
        p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_EMERGENCY, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_OCTOROTOR, PH.base.pack) ;
        c_TEST_Channel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
        p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)54312, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)28182, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)48032, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t) -107, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t)18349, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)9645, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)10946, PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)12376, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)16391, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)50816, PH.base.pack) ;
        c_TEST_Channel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_unix_usec_SET((uint64_t)5623477479509228515L, PH.base.pack) ;
        p2_time_boot_ms_SET((uint32_t)1998796706L, PH.base.pack) ;
        c_TEST_Channel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_type_mask_SET((uint16_t)(uint16_t)38035, PH.base.pack) ;
        p3_z_SET((float)1.7846626E38F, PH.base.pack) ;
        p3_afy_SET((float)1.5097453E38F, PH.base.pack) ;
        p3_vx_SET((float)3.0141145E38F, PH.base.pack) ;
        p3_yaw_SET((float) -9.358542E37F, PH.base.pack) ;
        p3_afz_SET((float)1.0809529E38F, PH.base.pack) ;
        p3_afx_SET((float)1.4099021E38F, PH.base.pack) ;
        p3_y_SET((float) -1.0579017E38F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)2678205790L, PH.base.pack) ;
        p3_yaw_rate_SET((float)2.8133365E38F, PH.base.pack) ;
        p3_x_SET((float)5.0584654E37F, PH.base.pack) ;
        p3_vz_SET((float) -8.472486E37F, PH.base.pack) ;
        p3_vy_SET((float)1.1828985E38F, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PING_4(), &PH);
        p4_time_usec_SET((uint64_t)738642279151904811L, PH.base.pack) ;
        p4_target_system_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p4_seq_SET((uint32_t)1192581163L, PH.base.pack) ;
        p4_target_component_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        c_TEST_Channel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_target_system_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p5_version_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
        {
            char16_t* passkey = u"fVgMmmbvndkgs";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_control_request_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_control_request_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        p6_ack_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"ffxwzmbgkswmfEnht";
            p7_key_SET_(key, &PH) ;
        }
        c_TEST_Channel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
        p11_custom_mode_SET((uint32_t)4109090955L, PH.base.pack) ;
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_ARMED, PH.base.pack) ;
        p11_target_system_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        c_TEST_Channel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
        {
            char16_t* param_id = u"Oa";
            p20_param_id_SET_(param_id, &PH) ;
        }
        p20_param_index_SET((int16_t)(int16_t)20512, PH.base.pack) ;
        p20_target_component_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        p20_target_system_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_component_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p21_target_system_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
        {
            char16_t* param_id = u"dzbjmdywoRTdseq";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)57557, PH.base.pack) ;
        p22_param_value_SET((float) -2.4499788E38F, PH.base.pack) ;
        p22_param_count_SET((uint16_t)(uint16_t)44890, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
        p23_target_system_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32, PH.base.pack) ;
        p23_param_value_SET((float)3.0301916E38F, PH.base.pack) ;
        p23_target_component_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        {
            char16_t* param_id = u"uwfnppf";
            p23_param_id_SET_(param_id, &PH) ;
        }
        c_TEST_Channel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
        p24_lat_SET((int32_t)562793228, PH.base.pack) ;
        p24_hdg_acc_SET((uint32_t)733765542L, &PH) ;
        p24_alt_ellipsoid_SET((int32_t) -59889805, &PH) ;
        p24_vel_SET((uint16_t)(uint16_t)1722, PH.base.pack) ;
        p24_alt_SET((int32_t)832601250, PH.base.pack) ;
        p24_eph_SET((uint16_t)(uint16_t)45889, PH.base.pack) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP, PH.base.pack) ;
        p24_v_acc_SET((uint32_t)2163748263L, &PH) ;
        p24_h_acc_SET((uint32_t)2891619359L, &PH) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
        p24_time_usec_SET((uint64_t)5082700707253431056L, PH.base.pack) ;
        p24_lon_SET((int32_t)1314997383, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)4277435625L, &PH) ;
        p24_epv_SET((uint16_t)(uint16_t)49715, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)40342, PH.base.pack) ;
        c_TEST_Channel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_snr[] =  {(uint8_t)198, (uint8_t)252, (uint8_t)146, (uint8_t)180, (uint8_t)245, (uint8_t)192, (uint8_t)249, (uint8_t)177, (uint8_t)245, (uint8_t)244, (uint8_t)210, (uint8_t)240, (uint8_t)181, (uint8_t)224, (uint8_t)0, (uint8_t)213, (uint8_t)145, (uint8_t)207, (uint8_t)50, (uint8_t)151};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)153, (uint8_t)156, (uint8_t)131, (uint8_t)22, (uint8_t)74, (uint8_t)111, (uint8_t)34, (uint8_t)228, (uint8_t)171, (uint8_t)213, (uint8_t)110, (uint8_t)238, (uint8_t)26, (uint8_t)72, (uint8_t)92, (uint8_t)135, (uint8_t)52, (uint8_t)244, (uint8_t)80, (uint8_t)143};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_used[] =  {(uint8_t)27, (uint8_t)138, (uint8_t)40, (uint8_t)65, (uint8_t)149, (uint8_t)130, (uint8_t)184, (uint8_t)234, (uint8_t)211, (uint8_t)85, (uint8_t)7, (uint8_t)247, (uint8_t)16, (uint8_t)14, (uint8_t)16, (uint8_t)84, (uint8_t)154, (uint8_t)234, (uint8_t)175, (uint8_t)44};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)35, (uint8_t)186, (uint8_t)147, (uint8_t)4, (uint8_t)157, (uint8_t)222, (uint8_t)42, (uint8_t)179, (uint8_t)162, (uint8_t)110, (uint8_t)17, (uint8_t)39, (uint8_t)213, (uint8_t)233, (uint8_t)237, (uint8_t)108, (uint8_t)108, (uint8_t)174, (uint8_t)235, (uint8_t)54};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        {
            uint8_t satellite_prn[] =  {(uint8_t)210, (uint8_t)193, (uint8_t)242, (uint8_t)194, (uint8_t)180, (uint8_t)245, (uint8_t)49, (uint8_t)139, (uint8_t)15, (uint8_t)26, (uint8_t)106, (uint8_t)163, (uint8_t)67, (uint8_t)29, (uint8_t)213, (uint8_t)238, (uint8_t)102, (uint8_t)171, (uint8_t)167, (uint8_t)185};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
        p26_time_boot_ms_SET((uint32_t)1827985144L, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t) -5486, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t)12048, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t)6931, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t) -28977, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t)276, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t)3266, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t)6632, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t)8315, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t)5131, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
        p27_zacc_SET((int16_t)(int16_t)6156, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t)32072, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t)28838, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t) -3783, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t)25247, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t)5190, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)5784070630332877928L, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t) -12455, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t) -17597, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t) -8698, PH.base.pack) ;
        c_TEST_Channel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
        p28_time_usec_SET((uint64_t)2420029976750290474L, PH.base.pack) ;
        p28_press_diff1_SET((int16_t)(int16_t)28967, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t) -21831, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t) -15821, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t) -7550, PH.base.pack) ;
        c_TEST_Channel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_press_diff_SET((float)2.8552947E38F, PH.base.pack) ;
        p29_press_abs_SET((float) -3.1806921E38F, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)2777000147L, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t) -8705, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
        p30_pitchspeed_SET((float)1.2905179E38F, PH.base.pack) ;
        p30_yaw_SET((float) -2.0927818E38F, PH.base.pack) ;
        p30_yawspeed_SET((float)8.713658E36F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)3860618836L, PH.base.pack) ;
        p30_roll_SET((float) -2.995289E38F, PH.base.pack) ;
        p30_pitch_SET((float) -1.5504259E38F, PH.base.pack) ;
        p30_rollspeed_SET((float)2.5599526E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_yawspeed_SET((float) -2.0044965E38F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)3228766487L, PH.base.pack) ;
        p31_q3_SET((float) -1.3151464E38F, PH.base.pack) ;
        p31_pitchspeed_SET((float)1.1325669E38F, PH.base.pack) ;
        p31_q1_SET((float) -2.268714E38F, PH.base.pack) ;
        p31_rollspeed_SET((float)9.103879E37F, PH.base.pack) ;
        p31_q4_SET((float)3.3235552E37F, PH.base.pack) ;
        p31_q2_SET((float) -2.7354363E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_vz_SET((float)2.4881673E38F, PH.base.pack) ;
        p32_vx_SET((float)2.3754338E38F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)2718907227L, PH.base.pack) ;
        p32_vy_SET((float)6.943568E37F, PH.base.pack) ;
        p32_y_SET((float) -1.4839423E38F, PH.base.pack) ;
        p32_z_SET((float)2.529464E37F, PH.base.pack) ;
        p32_x_SET((float) -3.3733418E38F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_lat_SET((int32_t)1183614468, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)1430183752L, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)52699, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t)11561, PH.base.pack) ;
        p33_relative_alt_SET((int32_t)468401063, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t) -13302, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t) -21489, PH.base.pack) ;
        p33_alt_SET((int32_t) -591297719, PH.base.pack) ;
        p33_lon_SET((int32_t)1140831066, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan5_scaled_SET((int16_t)(int16_t) -27298, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t)3613, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t) -5432, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t) -20710, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t) -18107, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t)30618, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)822528977L, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t)29720, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t) -4952, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_rssi_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)40018, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)16903, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)5711, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)11521, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)1953193067L, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)38234, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)35288, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)32281, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)55586, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo5_raw_SET((uint16_t)(uint16_t)7090, PH.base.pack) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)1946, PH.base.pack) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)12685, PH.base.pack) ;
        p36_port_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)51332, PH.base.pack) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)52404, PH.base.pack) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)57045, PH.base.pack) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)47721, PH.base.pack) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)16237, PH.base.pack) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)47062, &PH) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)32063, &PH) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)63544, &PH) ;
        p36_time_usec_SET((uint32_t)3297173575L, PH.base.pack) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)12990, &PH) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)54903, &PH) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)38193, &PH) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)16444, &PH) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)43501, &PH) ;
        c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_target_component_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
        p37_start_index_SET((int16_t)(int16_t)9576, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t) -30347, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_target_component_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t) -22335, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t)5405, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
        p39_x_SET((float)4.0038699E37F, PH.base.pack) ;
        p39_z_SET((float)6.281886E37F, PH.base.pack) ;
        p39_param3_SET((float)3.18499E38F, PH.base.pack) ;
        p39_param4_SET((float)1.536996E38F, PH.base.pack) ;
        p39_y_SET((float) -1.6101427E38F, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p39_param1_SET((float)9.668622E37F, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)22888, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p39_param2_SET((float)2.723084E38F, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
        p40_target_component_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)27947, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_target_component_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p41_seq_SET((uint16_t)(uint16_t)25120, PH.base.pack) ;
        p41_target_system_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)18853, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p43_target_system_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
        p44_target_system_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
        p44_count_SET((uint16_t)(uint16_t)26207, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p45_target_system_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)49543, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
        p47_target_component_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM5_X, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_target_system_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p48_altitude_SET((int32_t)388170029, PH.base.pack) ;
        p48_longitude_SET((int32_t)713842092, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)661057399724568439L, &PH) ;
        p48_latitude_SET((int32_t) -821663400, PH.base.pack) ;
        c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_altitude_SET((int32_t) -1726678611, PH.base.pack) ;
        p49_latitude_SET((int32_t)1628743409, PH.base.pack) ;
        p49_longitude_SET((int32_t)1567055870, PH.base.pack) ;
        p49_time_usec_SET((uint64_t)6190259897561767168L, &PH) ;
        c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_param_value0_SET((float) -3.075087E38F, PH.base.pack) ;
        {
            char16_t* param_id = u"z";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_param_value_min_SET((float) -2.0911673E38F, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t) -21049, PH.base.pack) ;
        p50_param_value_max_SET((float)1.5479767E38F, PH.base.pack) ;
        p50_scale_SET((float)1.4425711E38F, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_target_component_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)62694, PH.base.pack) ;
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_p1z_SET((float)2.7912086E38F, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p54_p1x_SET((float) -1.7209253E38F, PH.base.pack) ;
        p54_p2z_SET((float)1.7046106E38F, PH.base.pack) ;
        p54_p2x_SET((float) -2.2860515E38F, PH.base.pack) ;
        p54_p2y_SET((float) -2.8972657E38F, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        p54_p1y_SET((float) -1.4393933E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p2y_SET((float)1.6674953E38F, PH.base.pack) ;
        p55_p1x_SET((float) -2.7042214E38F, PH.base.pack) ;
        p55_p2z_SET((float)3.8682135E37F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
        p55_p2x_SET((float) -3.113374E38F, PH.base.pack) ;
        p55_p1y_SET((float)1.769379E38F, PH.base.pack) ;
        p55_p1z_SET((float)1.3492177E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        {
            float covariance[] =  {8.624855E37F, -1.773088E38F, -1.197097E38F, 2.1944395E38F, -4.567355E36F, -1.9617086E38F, 2.8283467E38F, -6.2815966E37F, 2.4682034E37F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p61_yawspeed_SET((float)6.803884E37F, PH.base.pack) ;
        p61_rollspeed_SET((float) -3.2285225E37F, PH.base.pack) ;
        {
            float q[] =  {-2.4627713E38F, -8.0559216E37F, 2.3250278E38F, -1.4721795E38F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        p61_pitchspeed_SET((float)1.8103148E38F, PH.base.pack) ;
        p61_time_usec_SET((uint64_t)382978356164063664L, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_target_bearing_SET((int16_t)(int16_t)114, PH.base.pack) ;
        p62_alt_error_SET((float) -2.9013537E38F, PH.base.pack) ;
        p62_aspd_error_SET((float)2.982964E38F, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t) -4287, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)38387, PH.base.pack) ;
        p62_xtrack_error_SET((float) -1.9268052E38F, PH.base.pack) ;
        p62_nav_pitch_SET((float) -1.6226565E38F, PH.base.pack) ;
        p62_nav_roll_SET((float) -2.750247E38F, PH.base.pack) ;
        c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_vz_SET((float)1.89144E38F, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)9130898990363855839L, PH.base.pack) ;
        p63_relative_alt_SET((int32_t)230021988, PH.base.pack) ;
        p63_lat_SET((int32_t) -1810875865, PH.base.pack) ;
        p63_vy_SET((float)3.6964844E37F, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
        p63_lon_SET((int32_t)294424678, PH.base.pack) ;
        p63_alt_SET((int32_t) -1387717025, PH.base.pack) ;
        {
            float covariance[] =  {-1.2102523E38F, 1.2810792E38F, -2.478801E38F, -3.0821705E38F, 2.5466693E38F, 1.4737472E38F, 2.5109718E38F, 1.4144409E38F, -2.4192935E38F, 1.0986593E38F, -2.5032651E38F, 2.4538034E38F, 1.655048E38F, -1.0743504E38F, -1.4499836E38F, -1.4807932E38F, 7.048733E37F, -3.353283E38F, 2.4203087E38F, -1.0236553E38F, -1.2965495E38F, 2.9330956E38F, -3.9446115E37F, -2.695256E38F, -2.7039368E37F, 2.5325744E38F, 3.206408E38F, -1.0343767E38F, -1.9201432E37F, -3.1005742E38F, 1.1987241E37F, 2.6079538E38F, 1.9328248E38F, -5.296355E37F, 3.0762897E38F, -2.6730794E38F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_vx_SET((float) -1.1796876E38F, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_ax_SET((float)4.2152412E36F, PH.base.pack) ;
        p64_vx_SET((float)3.0937522E38F, PH.base.pack) ;
        {
            float covariance[] =  {-2.6489693E38F, -1.9294758E38F, -1.3704618E37F, 5.776104E37F, 3.0120267E38F, -1.990804E38F, 8.834275E37F, 2.175768E38F, 1.2487679E38F, 2.6288542E38F, -2.7963875E38F, -6.4975374E37F, 1.7071205E38F, 2.0140946E38F, 2.4151905E37F, -1.8348847E38F, -1.0900902E38F, 8.378776E37F, 2.4877515E38F, 2.9725713E38F, 2.7205796E38F, 2.9173559E38F, -2.7126455E38F, 3.3631018E38F, 1.6427293E38F, -1.8831493E38F, 1.0701349E37F, -3.2151889E37F, -4.65126E37F, 8.709773E37F, -7.6544217E37F, 3.0473412E38F, 7.6192383E37F, 2.8677946E38F, -3.0411216E38F, 1.6844173E38F, 1.180784E37F, -1.8547267E37F, 1.951001E38F, 2.3083512E38F, 2.4104569E38F, 2.664469E38F, -3.1089399E38F, 3.136372E38F, -1.3566426E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_y_SET((float) -1.1379941E38F, PH.base.pack) ;
        p64_z_SET((float) -2.9070905E38F, PH.base.pack) ;
        p64_x_SET((float)2.700082E37F, PH.base.pack) ;
        p64_vy_SET((float) -1.5391697E38F, PH.base.pack) ;
        p64_vz_SET((float) -1.4765127E38F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)4167268189364019142L, PH.base.pack) ;
        p64_az_SET((float)2.2904497E38F, PH.base.pack) ;
        p64_ay_SET((float) -1.9223717E38F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan3_raw_SET((uint16_t)(uint16_t)39236, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)52238, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)13857, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)4386, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)58578, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)59123, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)30459, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)22958, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)4123658133L, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)57366, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)6656, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)51915, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)55267, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)50218, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)46561, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)34087, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)65129, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)23774, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)21522, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_req_message_rate_SET((uint16_t)(uint16_t)57379, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
        c_TEST_Channel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
        p67_message_rate_SET((uint16_t)(uint16_t)29115, PH.base.pack) ;
        p67_stream_id_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        p67_on_off_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        c_TEST_Channel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_z_SET((int16_t)(int16_t) -27292, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t) -23729, PH.base.pack) ;
        p69_buttons_SET((uint16_t)(uint16_t)6801, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t) -3318, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t) -10170, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan3_raw_SET((uint16_t)(uint16_t)62168, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)17707, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)30459, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)895, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)60529, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)65171, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)44397, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)59344, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_param1_SET((float)1.2158014E38F, PH.base.pack) ;
        p73_param4_SET((float)2.8287092E38F, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        p73_z_SET((float)2.1211845E38F, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS, PH.base.pack) ;
        p73_x_SET((int32_t) -578738493, PH.base.pack) ;
        p73_y_SET((int32_t) -507590316, PH.base.pack) ;
        p73_param2_SET((float)1.5434218E38F, PH.base.pack) ;
        p73_param3_SET((float) -7.732285E37F, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)48163, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
        p74_airspeed_SET((float)1.1652333E38F, PH.base.pack) ;
        p74_climb_SET((float) -4.802045E37F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t) -22262, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)4200, PH.base.pack) ;
        p74_alt_SET((float) -2.2534467E38F, PH.base.pack) ;
        p74_groundspeed_SET((float) -2.4065619E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
        p75_z_SET((float) -4.7665807E37F, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p75_param3_SET((float) -2.1822763E38F, PH.base.pack) ;
        p75_y_SET((int32_t) -1103037049, PH.base.pack) ;
        p75_x_SET((int32_t) -1773459716, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION, PH.base.pack) ;
        p75_param4_SET((float) -1.0336779E38F, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p75_param2_SET((float)8.3944193E37F, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p75_param1_SET((float) -2.5065485E38F, PH.base.pack) ;
        c_CommunicationChannel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
        p76_param6_SET((float)2.8246103E37F, PH.base.pack) ;
        p76_param7_SET((float) -2.0492371E38F, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_DO_LAST, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p76_param3_SET((float) -2.9838885E38F, PH.base.pack) ;
        p76_param2_SET((float) -7.122422E37F, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p76_param1_SET((float) -1.4657895E38F, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p76_param5_SET((float) -3.5261978E37F, PH.base.pack) ;
        p76_param4_SET((float) -1.4108692E38F, PH.base.pack) ;
        c_CommunicationChannel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
        p77_target_component_SET((uint8_t)(uint8_t)160, &PH) ;
        p77_result_param2_SET((int32_t)1915796705, &PH) ;
        p77_progress_SET((uint8_t)(uint8_t)43, &PH) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY, PH.base.pack) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_ACCEPTED, PH.base.pack) ;
        p77_target_system_SET((uint8_t)(uint8_t)50, &PH) ;
        c_CommunicationChannel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_pitch_SET((float) -7.899273E37F, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)3704288031L, PH.base.pack) ;
        p81_thrust_SET((float) -1.0517644E38F, PH.base.pack) ;
        p81_roll_SET((float)1.7423118E37F, PH.base.pack) ;
        p81_yaw_SET((float) -2.5711321E38F, PH.base.pack) ;
        c_CommunicationChannel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_body_yaw_rate_SET((float) -8.528031E37F, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        p82_body_roll_rate_SET((float)1.671824E36F, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        p82_body_pitch_rate_SET((float)3.1095765E38F, PH.base.pack) ;
        {
            float q[] =  {3.0965447E38F, 2.134614E38F, 1.9944095E38F, -1.839429E38F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_type_mask_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p82_thrust_SET((float) -2.8882338E38F, PH.base.pack) ;
        p82_time_boot_ms_SET((uint32_t)4033694872L, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_type_mask_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        p83_body_pitch_rate_SET((float)2.7429525E37F, PH.base.pack) ;
        {
            float q[] =  {2.467757E38F, 1.1419551E38F, -4.2553218E36F, -3.0326632E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_body_roll_rate_SET((float) -1.8965446E38F, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)836184338L, PH.base.pack) ;
        p83_body_yaw_rate_SET((float)1.2020263E38F, PH.base.pack) ;
        p83_thrust_SET((float) -7.6001064E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_afy_SET((float) -1.5762462E38F, PH.base.pack) ;
        p84_yaw_SET((float)1.8024951E38F, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
        p84_vx_SET((float) -2.6647164E38F, PH.base.pack) ;
        p84_z_SET((float) -2.757985E38F, PH.base.pack) ;
        p84_x_SET((float)2.9109059E38F, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p84_vz_SET((float) -3.2925784E38F, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)37687, PH.base.pack) ;
        p84_yaw_rate_SET((float) -1.6824368E38F, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p84_vy_SET((float) -1.843436E38F, PH.base.pack) ;
        p84_y_SET((float)2.1458006E38F, PH.base.pack) ;
        p84_afx_SET((float)2.5455708E38F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)2840657028L, PH.base.pack) ;
        p84_afz_SET((float) -3.0432421E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_type_mask_SET((uint16_t)(uint16_t)25973, PH.base.pack) ;
        p86_yaw_SET((float) -1.8041025E38F, PH.base.pack) ;
        p86_lat_int_SET((int32_t)22567464, PH.base.pack) ;
        p86_yaw_rate_SET((float)1.2809817E38F, PH.base.pack) ;
        p86_afz_SET((float)2.9970862E38F, PH.base.pack) ;
        p86_vx_SET((float) -2.6102897E38F, PH.base.pack) ;
        p86_lon_int_SET((int32_t)1005439647, PH.base.pack) ;
        p86_afx_SET((float) -7.030669E37F, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)3837076562L, PH.base.pack) ;
        p86_vy_SET((float) -1.9320902E38F, PH.base.pack) ;
        p86_vz_SET((float) -2.1358592E37F, PH.base.pack) ;
        p86_alt_SET((float) -2.3043766E38F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        p86_afy_SET((float) -7.5840376E37F, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_lon_int_SET((int32_t) -1656171705, PH.base.pack) ;
        p87_vx_SET((float)1.6505227E38F, PH.base.pack) ;
        p87_afy_SET((float) -2.9323085E38F, PH.base.pack) ;
        p87_lat_int_SET((int32_t) -1750558671, PH.base.pack) ;
        p87_alt_SET((float) -1.4282283E37F, PH.base.pack) ;
        p87_afx_SET((float) -1.4652337E38F, PH.base.pack) ;
        p87_yaw_SET((float) -1.302302E38F, PH.base.pack) ;
        p87_vz_SET((float) -1.236594E38F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)15341, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        p87_yaw_rate_SET((float) -1.1988723E37F, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)25800711L, PH.base.pack) ;
        p87_vy_SET((float) -2.1458333E38F, PH.base.pack) ;
        p87_afz_SET((float)1.8256937E38F, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_y_SET((float) -3.1492727E38F, PH.base.pack) ;
        p89_pitch_SET((float) -9.471885E37F, PH.base.pack) ;
        p89_roll_SET((float)3.6291902E36F, PH.base.pack) ;
        p89_x_SET((float) -8.664972E37F, PH.base.pack) ;
        p89_z_SET((float) -1.1691921E38F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)2199127090L, PH.base.pack) ;
        p89_yaw_SET((float)1.2517426E38F, PH.base.pack) ;
        c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
        p90_vz_SET((int16_t)(int16_t)8161, PH.base.pack) ;
        p90_lat_SET((int32_t)516288825, PH.base.pack) ;
        p90_pitchspeed_SET((float)9.205762E37F, PH.base.pack) ;
        p90_yawspeed_SET((float)2.7040912E37F, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t) -13396, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t) -7812, PH.base.pack) ;
        p90_pitch_SET((float) -2.9559448E38F, PH.base.pack) ;
        p90_yaw_SET((float) -1.3418471E38F, PH.base.pack) ;
        p90_rollspeed_SET((float)1.5731412E37F, PH.base.pack) ;
        p90_alt_SET((int32_t)364606664, PH.base.pack) ;
        p90_lon_SET((int32_t) -1634073786, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t) -9448, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)2460617860524724550L, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t) -28403, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t)11177, PH.base.pack) ;
        p90_roll_SET((float)2.3408758E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
        p91_nav_mode_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p91_aux3_SET((float) -1.6665127E38F, PH.base.pack) ;
        p91_aux4_SET((float) -3.3422626E38F, PH.base.pack) ;
        p91_aux2_SET((float) -1.0273079E38F, PH.base.pack) ;
        p91_roll_ailerons_SET((float) -1.0650526E38F, PH.base.pack) ;
        p91_throttle_SET((float)2.1153253E38F, PH.base.pack) ;
        p91_aux1_SET((float) -1.1018966E38F, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)2954836293117905875L, PH.base.pack) ;
        p91_pitch_elevator_SET((float)2.7982E38F, PH.base.pack) ;
        p91_yaw_rudder_SET((float) -1.762535E38F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_DISARMED, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan8_raw_SET((uint16_t)(uint16_t)36250, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)32800, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)16634, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)34350, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)38298, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)2330, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)16738, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)1006035897317075436L, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)5780, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)7332, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)28694, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)25279, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)58919, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_flags_SET((uint64_t)698646595076618121L, PH.base.pack) ;
        {
            float controls[] =  {-8.247331E36F, 5.944022E37F, -2.2770088E38F, 5.466109E37F, -2.5152311E38F, -8.905611E37F, 2.9195898E38F, 2.6872844E38F, -9.164789E37F, -2.9454563E38F, 1.3820099E38F, 1.0713685E38F, 9.517887E37F, -8.658811E37F, 2.365475E38F, 2.004287E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p93_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_ARMED, PH.base.pack) ;
        p93_time_usec_SET((uint64_t)4833137921624074869L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_sensor_id_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        p100_flow_x_SET((int16_t)(int16_t) -877, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float)1.6527811E38F, PH.base.pack) ;
        p100_flow_y_SET((int16_t)(int16_t) -31355, PH.base.pack) ;
        p100_ground_distance_SET((float) -2.5976975E37F, PH.base.pack) ;
        p100_flow_rate_y_SET((float) -3.2569836E38F, &PH) ;
        p100_quality_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)818274337112929359L, PH.base.pack) ;
        p100_flow_rate_x_SET((float)2.64496E38F, &PH) ;
        p100_flow_comp_m_x_SET((float)1.9649876E38F, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_y_SET((float) -8.864521E37F, PH.base.pack) ;
        p101_z_SET((float) -1.8679635E38F, PH.base.pack) ;
        p101_roll_SET((float) -3.0782637E38F, PH.base.pack) ;
        p101_pitch_SET((float) -1.696389E38F, PH.base.pack) ;
        p101_yaw_SET((float) -3.0323395E38F, PH.base.pack) ;
        p101_x_SET((float)2.797536E38F, PH.base.pack) ;
        p101_usec_SET((uint64_t)7205380170764564865L, PH.base.pack) ;
        c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_pitch_SET((float) -2.7004002E38F, PH.base.pack) ;
        p102_roll_SET((float)3.39434E38F, PH.base.pack) ;
        p102_z_SET((float)1.808906E38F, PH.base.pack) ;
        p102_x_SET((float)1.5388927E38F, PH.base.pack) ;
        p102_yaw_SET((float) -2.3510135E37F, PH.base.pack) ;
        p102_y_SET((float)9.750819E37F, PH.base.pack) ;
        p102_usec_SET((uint64_t)8711455863589439307L, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_x_SET((float) -2.6531325E38F, PH.base.pack) ;
        p103_y_SET((float)1.860354E38F, PH.base.pack) ;
        p103_usec_SET((uint64_t)8762700995146816756L, PH.base.pack) ;
        p103_z_SET((float)1.7251512E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_yaw_SET((float)3.7907867E37F, PH.base.pack) ;
        p104_x_SET((float)2.7743866E38F, PH.base.pack) ;
        p104_y_SET((float) -2.0960584E38F, PH.base.pack) ;
        p104_pitch_SET((float) -2.8189438E38F, PH.base.pack) ;
        p104_usec_SET((uint64_t)6386520075828062971L, PH.base.pack) ;
        p104_roll_SET((float)3.2028093E38F, PH.base.pack) ;
        p104_z_SET((float) -1.543986E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
        p105_xacc_SET((float)3.239239E38F, PH.base.pack) ;
        p105_zmag_SET((float) -1.9904948E38F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)55066, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)3183386818622065744L, PH.base.pack) ;
        p105_ygyro_SET((float) -5.1140727E37F, PH.base.pack) ;
        p105_yacc_SET((float)3.3336205E38F, PH.base.pack) ;
        p105_zacc_SET((float) -1.3396574E38F, PH.base.pack) ;
        p105_xmag_SET((float) -3.1917793E38F, PH.base.pack) ;
        p105_temperature_SET((float) -2.5841709E38F, PH.base.pack) ;
        p105_ymag_SET((float)1.1042816E38F, PH.base.pack) ;
        p105_abs_pressure_SET((float)3.992781E37F, PH.base.pack) ;
        p105_diff_pressure_SET((float) -2.1398766E38F, PH.base.pack) ;
        p105_zgyro_SET((float)3.0557217E38F, PH.base.pack) ;
        p105_xgyro_SET((float)9.977335E37F, PH.base.pack) ;
        p105_pressure_alt_SET((float) -3.3457723E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_integration_time_us_SET((uint32_t)3588890225L, PH.base.pack) ;
        p106_integrated_y_SET((float) -2.9072106E36F, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t) -18754, PH.base.pack) ;
        p106_integrated_xgyro_SET((float)2.4804219E38F, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p106_distance_SET((float)2.0195664E38F, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)772180620L, PH.base.pack) ;
        p106_integrated_zgyro_SET((float) -1.8573195E38F, PH.base.pack) ;
        p106_integrated_ygyro_SET((float) -2.4838656E38F, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)8085116288809715674L, PH.base.pack) ;
        p106_integrated_x_SET((float) -1.7039473E38F, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
        p107_fields_updated_SET((uint32_t)1709510610L, PH.base.pack) ;
        p107_xmag_SET((float) -2.0344338E38F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)4746961554163647405L, PH.base.pack) ;
        p107_xacc_SET((float)5.2220746E36F, PH.base.pack) ;
        p107_zacc_SET((float)1.0605947E38F, PH.base.pack) ;
        p107_diff_pressure_SET((float)2.6270548E37F, PH.base.pack) ;
        p107_abs_pressure_SET((float) -1.983397E38F, PH.base.pack) ;
        p107_zgyro_SET((float) -9.527326E37F, PH.base.pack) ;
        p107_ygyro_SET((float) -8.873014E37F, PH.base.pack) ;
        p107_zmag_SET((float) -1.4442925E38F, PH.base.pack) ;
        p107_pressure_alt_SET((float) -2.9468203E38F, PH.base.pack) ;
        p107_temperature_SET((float) -2.8445232E38F, PH.base.pack) ;
        p107_yacc_SET((float)3.0703047E38F, PH.base.pack) ;
        p107_xgyro_SET((float)2.6556436E38F, PH.base.pack) ;
        p107_ymag_SET((float)2.7447507E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
        p108_vd_SET((float)1.1967096E38F, PH.base.pack) ;
        p108_alt_SET((float) -6.5912994E37F, PH.base.pack) ;
        p108_zacc_SET((float)1.3799184E38F, PH.base.pack) ;
        p108_q3_SET((float)1.9513299E38F, PH.base.pack) ;
        p108_zgyro_SET((float)6.907673E37F, PH.base.pack) ;
        p108_yaw_SET((float)1.6333923E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float)2.3794005E38F, PH.base.pack) ;
        p108_lon_SET((float)2.7550702E38F, PH.base.pack) ;
        p108_xacc_SET((float) -3.3355366E38F, PH.base.pack) ;
        p108_roll_SET((float)2.8086282E38F, PH.base.pack) ;
        p108_pitch_SET((float)2.3347925E38F, PH.base.pack) ;
        p108_vn_SET((float) -2.1820956E38F, PH.base.pack) ;
        p108_xgyro_SET((float)1.7617273E38F, PH.base.pack) ;
        p108_ve_SET((float) -3.1299468E38F, PH.base.pack) ;
        p108_yacc_SET((float) -1.0527436E38F, PH.base.pack) ;
        p108_ygyro_SET((float)8.873777E37F, PH.base.pack) ;
        p108_q1_SET((float)2.446726E38F, PH.base.pack) ;
        p108_q2_SET((float)1.122196E38F, PH.base.pack) ;
        p108_std_dev_vert_SET((float)1.6895107E38F, PH.base.pack) ;
        p108_q4_SET((float)2.2343956E38F, PH.base.pack) ;
        p108_lat_SET((float) -2.7190274E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
        p109_rssi_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)64799, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)38247, PH.base.pack) ;
        c_CommunicationChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        p110_target_component_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        p110_target_system_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)240, (uint8_t)61, (uint8_t)166, (uint8_t)177, (uint8_t)10, (uint8_t)65, (uint8_t)101, (uint8_t)84, (uint8_t)164, (uint8_t)108, (uint8_t)28, (uint8_t)142, (uint8_t)251, (uint8_t)69, (uint8_t)143, (uint8_t)110, (uint8_t)248, (uint8_t)187, (uint8_t)169, (uint8_t)238, (uint8_t)34, (uint8_t)152, (uint8_t)116, (uint8_t)135, (uint8_t)139, (uint8_t)144, (uint8_t)139, (uint8_t)16, (uint8_t)137, (uint8_t)19, (uint8_t)187, (uint8_t)81, (uint8_t)241, (uint8_t)63, (uint8_t)39, (uint8_t)28, (uint8_t)118, (uint8_t)21, (uint8_t)254, (uint8_t)227, (uint8_t)212, (uint8_t)237, (uint8_t)23, (uint8_t)122, (uint8_t)26, (uint8_t)86, (uint8_t)157, (uint8_t)234, (uint8_t)133, (uint8_t)156, (uint8_t)118, (uint8_t)112, (uint8_t)216, (uint8_t)230, (uint8_t)182, (uint8_t)223, (uint8_t)242, (uint8_t)118, (uint8_t)24, (uint8_t)109, (uint8_t)99, (uint8_t)239, (uint8_t)145, (uint8_t)150, (uint8_t)65, (uint8_t)47, (uint8_t)174, (uint8_t)106, (uint8_t)64, (uint8_t)119, (uint8_t)114, (uint8_t)20, (uint8_t)187, (uint8_t)128, (uint8_t)129, (uint8_t)18, (uint8_t)177, (uint8_t)191, (uint8_t)237, (uint8_t)223, (uint8_t)121, (uint8_t)106, (uint8_t)99, (uint8_t)170, (uint8_t)25, (uint8_t)159, (uint8_t)73, (uint8_t)147, (uint8_t)55, (uint8_t)86, (uint8_t)22, (uint8_t)211, (uint8_t)7, (uint8_t)94, (uint8_t)117, (uint8_t)144, (uint8_t)108, (uint8_t)26, (uint8_t)107, (uint8_t)91, (uint8_t)228, (uint8_t)48, (uint8_t)125, (uint8_t)184, (uint8_t)0, (uint8_t)142, (uint8_t)26, (uint8_t)148, (uint8_t)171, (uint8_t)10, (uint8_t)186, (uint8_t)110, (uint8_t)57, (uint8_t)65, (uint8_t)80, (uint8_t)66, (uint8_t)79, (uint8_t)189, (uint8_t)12, (uint8_t)114, (uint8_t)203, (uint8_t)139, (uint8_t)188, (uint8_t)133, (uint8_t)72, (uint8_t)248, (uint8_t)43, (uint8_t)71, (uint8_t)193, (uint8_t)179, (uint8_t)209, (uint8_t)18, (uint8_t)31, (uint8_t)170, (uint8_t)161, (uint8_t)210, (uint8_t)143, (uint8_t)244, (uint8_t)91, (uint8_t)14, (uint8_t)139, (uint8_t)16, (uint8_t)107, (uint8_t)109, (uint8_t)227, (uint8_t)133, (uint8_t)207, (uint8_t)21, (uint8_t)184, (uint8_t)17, (uint8_t)201, (uint8_t)88, (uint8_t)1, (uint8_t)200, (uint8_t)93, (uint8_t)178, (uint8_t)32, (uint8_t)153, (uint8_t)190, (uint8_t)228, (uint8_t)117, (uint8_t)198, (uint8_t)71, (uint8_t)80, (uint8_t)96, (uint8_t)224, (uint8_t)248, (uint8_t)138, (uint8_t)254, (uint8_t)241, (uint8_t)72, (uint8_t)194, (uint8_t)110, (uint8_t)199, (uint8_t)251, (uint8_t)239, (uint8_t)219, (uint8_t)101, (uint8_t)26, (uint8_t)102, (uint8_t)97, (uint8_t)91, (uint8_t)65, (uint8_t)83, (uint8_t)20, (uint8_t)173, (uint8_t)88, (uint8_t)32, (uint8_t)195, (uint8_t)235, (uint8_t)153, (uint8_t)126, (uint8_t)237, (uint8_t)186, (uint8_t)234, (uint8_t)55, (uint8_t)131, (uint8_t)103, (uint8_t)97, (uint8_t)5, (uint8_t)4, (uint8_t)66, (uint8_t)164, (uint8_t)247, (uint8_t)66, (uint8_t)172, (uint8_t)178, (uint8_t)94, (uint8_t)224, (uint8_t)33, (uint8_t)152, (uint8_t)125, (uint8_t)248, (uint8_t)237, (uint8_t)15, (uint8_t)243, (uint8_t)169, (uint8_t)64, (uint8_t)248, (uint8_t)177, (uint8_t)197, (uint8_t)185, (uint8_t)128, (uint8_t)207, (uint8_t)143, (uint8_t)20, (uint8_t)41, (uint8_t)28, (uint8_t)230, (uint8_t)101, (uint8_t)83, (uint8_t)199, (uint8_t)23, (uint8_t)102, (uint8_t)75, (uint8_t)220, (uint8_t)66, (uint8_t)213, (uint8_t)74, (uint8_t)119, (uint8_t)106, (uint8_t)253, (uint8_t)85, (uint8_t)136, (uint8_t)94, (uint8_t)126, (uint8_t)75, (uint8_t)8, (uint8_t)13, (uint8_t)29, (uint8_t)193};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_network_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
        p111_tc1_SET((int64_t)1802500019957033528L, PH.base.pack) ;
        p111_ts1_SET((int64_t) -2156372053642851945L, PH.base.pack) ;
        c_CommunicationChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_time_usec_SET((uint64_t)5420808730898669976L, PH.base.pack) ;
        p112_seq_SET((uint32_t)2247674867L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
        p113_satellites_visible_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        p113_lon_SET((int32_t)1708280858, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t) -7121, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)49274, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)8040578514204565075L, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)22882, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t) -31136, PH.base.pack) ;
        p113_alt_SET((int32_t)1097516311, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t) -7051, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)16513, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)41191, PH.base.pack) ;
        p113_lat_SET((int32_t) -834441838, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_quality_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p114_integrated_y_SET((float) -4.219411E37F, PH.base.pack) ;
        p114_integrated_zgyro_SET((float) -3.3287698E38F, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)3683950146L, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t)14931, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)8140735110475398491L, PH.base.pack) ;
        p114_integrated_x_SET((float) -3.839793E36F, PH.base.pack) ;
        p114_integrated_xgyro_SET((float) -1.291841E38F, PH.base.pack) ;
        p114_distance_SET((float)3.2335023E38F, PH.base.pack) ;
        p114_integrated_ygyro_SET((float)1.7266328E38F, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)558565300L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_lat_SET((int32_t) -1083544770, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t) -9622, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)22105, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {7.223162E37F, 8.250534E37F, -3.3024533E38F, -3.9507474E37F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_rollspeed_SET((float)2.3352075E38F, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)38056, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t)12606, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t) -13007, PH.base.pack) ;
        p115_pitchspeed_SET((float)2.6683459E38F, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t)17409, PH.base.pack) ;
        p115_alt_SET((int32_t)2072380285, PH.base.pack) ;
        p115_yawspeed_SET((float) -2.4799467E38F, PH.base.pack) ;
        p115_lon_SET((int32_t)227522083, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t)27926, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t) -22783, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)2603694701253781558L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
        p116_time_boot_ms_SET((uint32_t)2695585558L, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t)25436, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t) -12694, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t)294, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t)15777, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t) -10476, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t)17563, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t)24254, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t)3454, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t) -24532, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_target_component_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p117_end_SET((uint16_t)(uint16_t)31377, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)6458, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
        p118_last_log_num_SET((uint16_t)(uint16_t)36446, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)3445078965L, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)59194, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)23138, PH.base.pack) ;
        p118_size_SET((uint32_t)531705031L, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_target_system_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p119_count_SET((uint32_t)2440630713L, PH.base.pack) ;
        p119_ofs_SET((uint32_t)2399604853L, PH.base.pack) ;
        p119_id_SET((uint16_t)(uint16_t)33017, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
        p120_id_SET((uint16_t)(uint16_t)11662, PH.base.pack) ;
        p120_count_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p120_ofs_SET((uint32_t)2470467810L, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)145, (uint8_t)160, (uint8_t)204, (uint8_t)139, (uint8_t)69, (uint8_t)134, (uint8_t)113, (uint8_t)67, (uint8_t)230, (uint8_t)86, (uint8_t)221, (uint8_t)188, (uint8_t)217, (uint8_t)11, (uint8_t)65, (uint8_t)116, (uint8_t)212, (uint8_t)52, (uint8_t)28, (uint8_t)66, (uint8_t)141, (uint8_t)20, (uint8_t)28, (uint8_t)154, (uint8_t)88, (uint8_t)98, (uint8_t)106, (uint8_t)100, (uint8_t)15, (uint8_t)247, (uint8_t)84, (uint8_t)201, (uint8_t)98, (uint8_t)19, (uint8_t)171, (uint8_t)17, (uint8_t)1, (uint8_t)101, (uint8_t)96, (uint8_t)169, (uint8_t)213, (uint8_t)214, (uint8_t)8, (uint8_t)218, (uint8_t)226, (uint8_t)179, (uint8_t)55, (uint8_t)167, (uint8_t)210, (uint8_t)230, (uint8_t)5, (uint8_t)251, (uint8_t)67, (uint8_t)233, (uint8_t)161, (uint8_t)223, (uint8_t)178, (uint8_t)53, (uint8_t)176, (uint8_t)25, (uint8_t)217, (uint8_t)80, (uint8_t)41, (uint8_t)180, (uint8_t)219, (uint8_t)44, (uint8_t)89, (uint8_t)7, (uint8_t)182, (uint8_t)142, (uint8_t)139, (uint8_t)25, (uint8_t)70, (uint8_t)115, (uint8_t)144, (uint8_t)174, (uint8_t)160, (uint8_t)138, (uint8_t)106, (uint8_t)148, (uint8_t)213, (uint8_t)170, (uint8_t)152, (uint8_t)48, (uint8_t)91, (uint8_t)12, (uint8_t)56, (uint8_t)17, (uint8_t)115, (uint8_t)208};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
        p121_target_system_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p121_target_component_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_system_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        p122_target_component_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_len_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)150, (uint8_t)125, (uint8_t)223, (uint8_t)133, (uint8_t)85, (uint8_t)103, (uint8_t)250, (uint8_t)210, (uint8_t)245, (uint8_t)126, (uint8_t)243, (uint8_t)185, (uint8_t)154, (uint8_t)108, (uint8_t)2, (uint8_t)99, (uint8_t)250, (uint8_t)202, (uint8_t)204, (uint8_t)229, (uint8_t)10, (uint8_t)56, (uint8_t)175, (uint8_t)110, (uint8_t)47, (uint8_t)61, (uint8_t)62, (uint8_t)155, (uint8_t)1, (uint8_t)205, (uint8_t)40, (uint8_t)19, (uint8_t)241, (uint8_t)192, (uint8_t)223, (uint8_t)15, (uint8_t)250, (uint8_t)196, (uint8_t)187, (uint8_t)45, (uint8_t)194, (uint8_t)216, (uint8_t)114, (uint8_t)223, (uint8_t)84, (uint8_t)61, (uint8_t)120, (uint8_t)143, (uint8_t)101, (uint8_t)124, (uint8_t)81, (uint8_t)107, (uint8_t)34, (uint8_t)248, (uint8_t)131, (uint8_t)58, (uint8_t)223, (uint8_t)101, (uint8_t)33, (uint8_t)25, (uint8_t)151, (uint8_t)111, (uint8_t)165, (uint8_t)91, (uint8_t)117, (uint8_t)82, (uint8_t)22, (uint8_t)213, (uint8_t)183, (uint8_t)56, (uint8_t)177, (uint8_t)11, (uint8_t)88, (uint8_t)243, (uint8_t)18, (uint8_t)89, (uint8_t)177, (uint8_t)232, (uint8_t)13, (uint8_t)243, (uint8_t)218, (uint8_t)115, (uint8_t)191, (uint8_t)235, (uint8_t)121, (uint8_t)104, (uint8_t)248, (uint8_t)60, (uint8_t)53, (uint8_t)103, (uint8_t)36, (uint8_t)222, (uint8_t)218, (uint8_t)117, (uint8_t)140, (uint8_t)82, (uint8_t)177, (uint8_t)181, (uint8_t)138, (uint8_t)183, (uint8_t)191, (uint8_t)174, (uint8_t)130, (uint8_t)254, (uint8_t)151, (uint8_t)9, (uint8_t)167, (uint8_t)117, (uint8_t)180, (uint8_t)220};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_target_system_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        p123_target_component_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
        p124_lat_SET((int32_t) -392042254, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)196371693L, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        p124_lon_SET((int32_t) -434981377, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)38478, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)57414, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)7043316742595647009L, PH.base.pack) ;
        p124_alt_SET((int32_t)160942686, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)3066, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)39461, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
        p125_Vcc_SET((uint16_t)(uint16_t)34480, PH.base.pack) ;
        p125_Vservo_SET((uint16_t)(uint16_t)42621, PH.base.pack) ;
        p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID, PH.base.pack) ;
        c_CommunicationChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_timeout_SET((uint16_t)(uint16_t)41905, PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)66, (uint8_t)15, (uint8_t)31, (uint8_t)165, (uint8_t)48, (uint8_t)126, (uint8_t)189, (uint8_t)44, (uint8_t)34, (uint8_t)70, (uint8_t)217, (uint8_t)73, (uint8_t)25, (uint8_t)80, (uint8_t)239, (uint8_t)180, (uint8_t)80, (uint8_t)200, (uint8_t)136, (uint8_t)247, (uint8_t)194, (uint8_t)36, (uint8_t)80, (uint8_t)208, (uint8_t)87, (uint8_t)255, (uint8_t)36, (uint8_t)75, (uint8_t)166, (uint8_t)104, (uint8_t)122, (uint8_t)236, (uint8_t)56, (uint8_t)204, (uint8_t)234, (uint8_t)102, (uint8_t)227, (uint8_t)203, (uint8_t)74, (uint8_t)132, (uint8_t)211, (uint8_t)30, (uint8_t)72, (uint8_t)206, (uint8_t)158, (uint8_t)64, (uint8_t)213, (uint8_t)193, (uint8_t)180, (uint8_t)51, (uint8_t)158, (uint8_t)89, (uint8_t)70, (uint8_t)216, (uint8_t)207, (uint8_t)66, (uint8_t)217, (uint8_t)9, (uint8_t)235, (uint8_t)91, (uint8_t)197, (uint8_t)143, (uint8_t)72, (uint8_t)96, (uint8_t)11, (uint8_t)12, (uint8_t)32, (uint8_t)189, (uint8_t)74, (uint8_t)221};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY, PH.base.pack) ;
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL, PH.base.pack) ;
        p126_baudrate_SET((uint32_t)921433821L, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
        p127_tow_SET((uint32_t)981767684L, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t)1294952083, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)2844378272L, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)2663193355L, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t) -205967155, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t)531524515, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t) -1270752438, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)56590, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
        p128_nsats_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p128_tow_SET((uint32_t)686691887L, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)217863905L, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t) -168354658, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t)1351038531, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)3864587519L, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t) -1587884937, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t)1542463182, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)61325, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
        p129_zgyro_SET((int16_t)(int16_t) -10897, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t)7034, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t)20955, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t) -4829, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t) -10017, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t)10573, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t)24318, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t) -29353, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)3671968546L, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t) -3370, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_width_SET((uint16_t)(uint16_t)27498, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)35042, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)15921, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p130_size_SET((uint32_t)3089642794L, PH.base.pack) ;
        c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)188, (uint8_t)71, (uint8_t)31, (uint8_t)26, (uint8_t)155, (uint8_t)207, (uint8_t)21, (uint8_t)68, (uint8_t)137, (uint8_t)226, (uint8_t)143, (uint8_t)217, (uint8_t)86, (uint8_t)121, (uint8_t)157, (uint8_t)189, (uint8_t)157, (uint8_t)228, (uint8_t)229, (uint8_t)14, (uint8_t)117, (uint8_t)124, (uint8_t)105, (uint8_t)242, (uint8_t)59, (uint8_t)229, (uint8_t)60, (uint8_t)29, (uint8_t)253, (uint8_t)66, (uint8_t)206, (uint8_t)23, (uint8_t)110, (uint8_t)206, (uint8_t)91, (uint8_t)171, (uint8_t)5, (uint8_t)231, (uint8_t)153, (uint8_t)152, (uint8_t)140, (uint8_t)160, (uint8_t)8, (uint8_t)89, (uint8_t)60, (uint8_t)116, (uint8_t)115, (uint8_t)247, (uint8_t)46, (uint8_t)109, (uint8_t)195, (uint8_t)83, (uint8_t)220, (uint8_t)21, (uint8_t)98, (uint8_t)115, (uint8_t)15, (uint8_t)26, (uint8_t)192, (uint8_t)166, (uint8_t)245, (uint8_t)223, (uint8_t)178, (uint8_t)3, (uint8_t)147, (uint8_t)51, (uint8_t)187, (uint8_t)9, (uint8_t)137, (uint8_t)126, (uint8_t)55, (uint8_t)42, (uint8_t)32, (uint8_t)86, (uint8_t)118, (uint8_t)58, (uint8_t)155, (uint8_t)172, (uint8_t)137, (uint8_t)165, (uint8_t)46, (uint8_t)75, (uint8_t)68, (uint8_t)234, (uint8_t)21, (uint8_t)229, (uint8_t)241, (uint8_t)130, (uint8_t)230, (uint8_t)110, (uint8_t)148, (uint8_t)4, (uint8_t)216, (uint8_t)149, (uint8_t)56, (uint8_t)25, (uint8_t)126, (uint8_t)80, (uint8_t)105, (uint8_t)68, (uint8_t)134, (uint8_t)202, (uint8_t)38, (uint8_t)28, (uint8_t)3, (uint8_t)78, (uint8_t)51, (uint8_t)152, (uint8_t)76, (uint8_t)157, (uint8_t)2, (uint8_t)123, (uint8_t)57, (uint8_t)92, (uint8_t)228, (uint8_t)248, (uint8_t)191, (uint8_t)80, (uint8_t)193, (uint8_t)206, (uint8_t)47, (uint8_t)56, (uint8_t)73, (uint8_t)61, (uint8_t)86, (uint8_t)200, (uint8_t)69, (uint8_t)96, (uint8_t)142, (uint8_t)0, (uint8_t)0, (uint8_t)167, (uint8_t)182, (uint8_t)223, (uint8_t)124, (uint8_t)174, (uint8_t)249, (uint8_t)43, (uint8_t)26, (uint8_t)202, (uint8_t)136, (uint8_t)97, (uint8_t)246, (uint8_t)98, (uint8_t)19, (uint8_t)115, (uint8_t)39, (uint8_t)15, (uint8_t)27, (uint8_t)5, (uint8_t)37, (uint8_t)196, (uint8_t)136, (uint8_t)154, (uint8_t)77, (uint8_t)56, (uint8_t)46, (uint8_t)206, (uint8_t)171, (uint8_t)123, (uint8_t)144, (uint8_t)190, (uint8_t)237, (uint8_t)111, (uint8_t)172, (uint8_t)25, (uint8_t)210, (uint8_t)215, (uint8_t)84, (uint8_t)221, (uint8_t)141, (uint8_t)15, (uint8_t)235, (uint8_t)31, (uint8_t)60, (uint8_t)237, (uint8_t)77, (uint8_t)66, (uint8_t)169, (uint8_t)68, (uint8_t)91, (uint8_t)66, (uint8_t)176, (uint8_t)162, (uint8_t)203, (uint8_t)215, (uint8_t)97, (uint8_t)164, (uint8_t)85, (uint8_t)71, (uint8_t)200, (uint8_t)104, (uint8_t)60, (uint8_t)216, (uint8_t)55, (uint8_t)103, (uint8_t)61, (uint8_t)4, (uint8_t)117, (uint8_t)242, (uint8_t)159, (uint8_t)161, (uint8_t)132, (uint8_t)4, (uint8_t)245, (uint8_t)148, (uint8_t)137, (uint8_t)210, (uint8_t)57, (uint8_t)112, (uint8_t)48, (uint8_t)39, (uint8_t)77, (uint8_t)121, (uint8_t)131, (uint8_t)155, (uint8_t)5, (uint8_t)235, (uint8_t)145, (uint8_t)206, (uint8_t)116, (uint8_t)91, (uint8_t)231, (uint8_t)247, (uint8_t)196, (uint8_t)155, (uint8_t)160, (uint8_t)98, (uint8_t)135, (uint8_t)242, (uint8_t)253, (uint8_t)56, (uint8_t)24, (uint8_t)98, (uint8_t)156, (uint8_t)157, (uint8_t)50, (uint8_t)233, (uint8_t)204, (uint8_t)113, (uint8_t)24, (uint8_t)224, (uint8_t)241, (uint8_t)18, (uint8_t)68, (uint8_t)150, (uint8_t)126, (uint8_t)36, (uint8_t)135, (uint8_t)42, (uint8_t)212, (uint8_t)67, (uint8_t)122};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        p131_seqnr_SET((uint16_t)(uint16_t)36259, PH.base.pack) ;
        c_CommunicationChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_id_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)2242400218L, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)3223, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_YAW_90, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)7799, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)31401, PH.base.pack) ;
        c_CommunicationChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_lon_SET((int32_t) -740375538, PH.base.pack) ;
        p133_lat_SET((int32_t)1674825285, PH.base.pack) ;
        p133_mask_SET((uint64_t)724014654130113026L, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)60677, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
        p134_lon_SET((int32_t) -268865400, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t)10168, (int16_t)24181, (int16_t) -20530, (int16_t)4664, (int16_t)7164, (int16_t) -11506, (int16_t)12032, (int16_t)11300, (int16_t)8235, (int16_t)15854, (int16_t) -28226, (int16_t)18530, (int16_t) -14937, (int16_t) -30199, (int16_t)7233, (int16_t) -15838};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        p134_lat_SET((int32_t)1764363521, PH.base.pack) ;
        p134_grid_spacing_SET((uint16_t)(uint16_t)58542, PH.base.pack) ;
        p134_gridbit_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lon_SET((int32_t) -828604579, PH.base.pack) ;
        p135_lat_SET((int32_t)110618588, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_pending_SET((uint16_t)(uint16_t)35571, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)135, PH.base.pack) ;
        p136_lon_SET((int32_t)1208102702, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)15338, PH.base.pack) ;
        p136_lat_SET((int32_t) -267791488, PH.base.pack) ;
        p136_terrain_height_SET((float) -5.1448715E37F, PH.base.pack) ;
        p136_current_height_SET((float)2.0727344E37F, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_press_abs_SET((float)3.3252278E38F, PH.base.pack) ;
        p137_time_boot_ms_SET((uint32_t)3894017802L, PH.base.pack) ;
        p137_press_diff_SET((float) -7.167319E37F, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t)9799, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ATT_POS_MOCAP_138(), &PH);
        p138_x_SET((float)1.6030427E38F, PH.base.pack) ;
        p138_z_SET((float) -1.6967688E38F, PH.base.pack) ;
        p138_y_SET((float)1.6723194E38F, PH.base.pack) ;
        {
            float q[] =  {5.098307E37F, 1.5198215E38F, 7.2430863E37F, -1.4739468E38F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_time_usec_SET((uint64_t)5490821407814724854L, PH.base.pack) ;
        c_CommunicationChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_group_mlx_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p139_target_system_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
        {
            float controls[] =  {-3.3818807E38F, -2.9335617E38F, 1.9238835E38F, 1.856661E38F, -9.109852E37F, 2.7458614E38F, -8.921846E37F, 9.478225E37F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_target_component_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p139_time_usec_SET((uint64_t)7058463238121930420L, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        p140_time_usec_SET((uint64_t)1952369112243036347L, PH.base.pack) ;
        p140_group_mlx_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        {
            float controls[] =  {6.623479E37F, -3.151085E38F, -1.1793655E38F, 1.2202488E38F, -1.2448248E38F, -2.504331E38F, -2.2064822E38F, -1.8405727E38F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ALTITUDE_141(), &PH);
        p141_altitude_amsl_SET((float)9.430941E37F, PH.base.pack) ;
        p141_altitude_local_SET((float)1.8667983E38F, PH.base.pack) ;
        p141_altitude_relative_SET((float) -2.086277E38F, PH.base.pack) ;
        p141_bottom_clearance_SET((float) -1.6151378E38F, PH.base.pack) ;
        p141_altitude_monotonic_SET((float)1.293918E38F, PH.base.pack) ;
        p141_altitude_terrain_SET((float)1.3030429E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)4007552051117726338L, PH.base.pack) ;
        c_CommunicationChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_RESOURCE_REQUEST_142(), &PH);
        p142_uri_type_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        {
            uint8_t uri[] =  {(uint8_t)74, (uint8_t)80, (uint8_t)69, (uint8_t)21, (uint8_t)115, (uint8_t)87, (uint8_t)108, (uint8_t)111, (uint8_t)84, (uint8_t)89, (uint8_t)151, (uint8_t)156, (uint8_t)176, (uint8_t)235, (uint8_t)23, (uint8_t)147, (uint8_t)100, (uint8_t)191, (uint8_t)20, (uint8_t)79, (uint8_t)194, (uint8_t)148, (uint8_t)206, (uint8_t)34, (uint8_t)237, (uint8_t)93, (uint8_t)11, (uint8_t)249, (uint8_t)114, (uint8_t)254, (uint8_t)35, (uint8_t)241, (uint8_t)208, (uint8_t)254, (uint8_t)171, (uint8_t)218, (uint8_t)136, (uint8_t)156, (uint8_t)17, (uint8_t)181, (uint8_t)150, (uint8_t)216, (uint8_t)74, (uint8_t)76, (uint8_t)225, (uint8_t)191, (uint8_t)213, (uint8_t)84, (uint8_t)74, (uint8_t)76, (uint8_t)250, (uint8_t)131, (uint8_t)112, (uint8_t)234, (uint8_t)97, (uint8_t)98, (uint8_t)61, (uint8_t)103, (uint8_t)8, (uint8_t)96, (uint8_t)53, (uint8_t)41, (uint8_t)42, (uint8_t)226, (uint8_t)151, (uint8_t)180, (uint8_t)211, (uint8_t)64, (uint8_t)238, (uint8_t)137, (uint8_t)46, (uint8_t)242, (uint8_t)55, (uint8_t)188, (uint8_t)207, (uint8_t)215, (uint8_t)148, (uint8_t)175, (uint8_t)111, (uint8_t)44, (uint8_t)166, (uint8_t)198, (uint8_t)246, (uint8_t)174, (uint8_t)115, (uint8_t)42, (uint8_t)7, (uint8_t)57, (uint8_t)82, (uint8_t)228, (uint8_t)218, (uint8_t)147, (uint8_t)161, (uint8_t)15, (uint8_t)124, (uint8_t)165, (uint8_t)152, (uint8_t)75, (uint8_t)176, (uint8_t)124, (uint8_t)94, (uint8_t)59, (uint8_t)78, (uint8_t)234, (uint8_t)22, (uint8_t)211, (uint8_t)39, (uint8_t)249, (uint8_t)99, (uint8_t)248, (uint8_t)125, (uint8_t)121, (uint8_t)84, (uint8_t)43, (uint8_t)143, (uint8_t)201, (uint8_t)86, (uint8_t)85, (uint8_t)155, (uint8_t)252};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        p142_transfer_type_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p142_request_id_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        {
            uint8_t storage[] =  {(uint8_t)147, (uint8_t)149, (uint8_t)209, (uint8_t)81, (uint8_t)16, (uint8_t)33, (uint8_t)182, (uint8_t)10, (uint8_t)231, (uint8_t)21, (uint8_t)42, (uint8_t)212, (uint8_t)3, (uint8_t)8, (uint8_t)85, (uint8_t)171, (uint8_t)63, (uint8_t)77, (uint8_t)255, (uint8_t)163, (uint8_t)3, (uint8_t)5, (uint8_t)57, (uint8_t)107, (uint8_t)188, (uint8_t)22, (uint8_t)72, (uint8_t)60, (uint8_t)124, (uint8_t)238, (uint8_t)114, (uint8_t)195, (uint8_t)24, (uint8_t)204, (uint8_t)160, (uint8_t)147, (uint8_t)196, (uint8_t)120, (uint8_t)222, (uint8_t)12, (uint8_t)242, (uint8_t)138, (uint8_t)153, (uint8_t)113, (uint8_t)206, (uint8_t)48, (uint8_t)182, (uint8_t)55, (uint8_t)50, (uint8_t)72, (uint8_t)245, (uint8_t)88, (uint8_t)129, (uint8_t)183, (uint8_t)154, (uint8_t)116, (uint8_t)48, (uint8_t)106, (uint8_t)156, (uint8_t)161, (uint8_t)229, (uint8_t)136, (uint8_t)240, (uint8_t)82, (uint8_t)242, (uint8_t)51, (uint8_t)89, (uint8_t)83, (uint8_t)228, (uint8_t)201, (uint8_t)84, (uint8_t)111, (uint8_t)170, (uint8_t)118, (uint8_t)3, (uint8_t)202, (uint8_t)139, (uint8_t)79, (uint8_t)103, (uint8_t)61, (uint8_t)84, (uint8_t)196, (uint8_t)190, (uint8_t)110, (uint8_t)217, (uint8_t)94, (uint8_t)216, (uint8_t)154, (uint8_t)4, (uint8_t)2, (uint8_t)100, (uint8_t)141, (uint8_t)83, (uint8_t)237, (uint8_t)137, (uint8_t)69, (uint8_t)255, (uint8_t)245, (uint8_t)22, (uint8_t)113, (uint8_t)96, (uint8_t)110, (uint8_t)84, (uint8_t)187, (uint8_t)238, (uint8_t)134, (uint8_t)67, (uint8_t)232, (uint8_t)169, (uint8_t)158, (uint8_t)3, (uint8_t)241, (uint8_t)102, (uint8_t)53, (uint8_t)163, (uint8_t)161, (uint8_t)122, (uint8_t)25, (uint8_t)122, (uint8_t)204};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SCALED_PRESSURE3_143(), &PH);
        p143_temperature_SET((int16_t)(int16_t) -840, PH.base.pack) ;
        p143_press_diff_SET((float) -1.2129372E38F, PH.base.pack) ;
        p143_press_abs_SET((float) -1.092815E38F, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)139150093L, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FOLLOW_TARGET_144(), &PH);
        p144_lon_SET((int32_t)218545717, PH.base.pack) ;
        p144_lat_SET((int32_t)1920781709, PH.base.pack) ;
        {
            float acc[] =  {-1.5771143E37F, -1.5511651E38F, -2.5844528E38F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        {
            float attitude_q[] =  {-7.327044E37F, -2.1559611E38F, 3.2146299E38F, 5.4868694E37F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        p144_est_capabilities_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
        {
            float vel[] =  {7.9504373E37F, 1.025579E36F, -3.108935E38F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        p144_custom_state_SET((uint64_t)5296345303237363771L, PH.base.pack) ;
        {
            float rates[] =  {-2.2650533E38F, -1.1005457E38F, -1.1568884E38F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        p144_timestamp_SET((uint64_t)1662993579140645087L, PH.base.pack) ;
        {
            float position_cov[] =  {-2.4871958E38F, 9.138984E37F, 1.353687E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        p144_alt_SET((float) -1.4831561E38F, PH.base.pack) ;
        c_CommunicationChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_z_vel_SET((float) -9.789147E37F, PH.base.pack) ;
        p146_pitch_rate_SET((float)2.8002883E37F, PH.base.pack) ;
        p146_time_usec_SET((uint64_t)8509353078048676886L, PH.base.pack) ;
        p146_y_pos_SET((float) -1.4153027E38F, PH.base.pack) ;
        p146_roll_rate_SET((float) -2.5388368E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {3.326445E38F, -1.5882006E38F, 6.9287475E37F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        {
            float pos_variance[] =  {-1.6913545E38F, 1.9884469E38F, -1.9107016E38F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_y_vel_SET((float) -3.1724181E38F, PH.base.pack) ;
        p146_x_vel_SET((float)9.955363E37F, PH.base.pack) ;
        p146_x_acc_SET((float) -1.1053693E38F, PH.base.pack) ;
        p146_y_acc_SET((float) -1.6723242E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float) -2.4519535E38F, PH.base.pack) ;
        p146_x_pos_SET((float) -6.7378256E37F, PH.base.pack) ;
        p146_airspeed_SET((float)2.613459E37F, PH.base.pack) ;
        {
            float q[] =  {3.3645337E38F, -3.158734E38F, 1.2938342E38F, -3.176179E38F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_z_pos_SET((float) -2.8726934E38F, PH.base.pack) ;
        p146_z_acc_SET((float)3.2074994E38F, PH.base.pack) ;
        c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BATTERY_STATUS_147(), &PH);
        p147_current_consumed_SET((int32_t) -1854145163, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t) -28272, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t)1645962199, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t)25, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t)16112, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_NIMH, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)60504, (uint16_t)64551, (uint16_t)30296, (uint16_t)51308, (uint16_t)42299, (uint16_t)56612, (uint16_t)13666, (uint16_t)47894, (uint16_t)14475, (uint16_t)63758};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AUTOPILOT_VERSION_148(), &PH);
        p148_product_id_SET((uint16_t)(uint16_t)53413, PH.base.pack) ;
        p148_middleware_sw_version_SET((uint32_t)359373337L, PH.base.pack) ;
        p148_vendor_id_SET((uint16_t)(uint16_t)62707, PH.base.pack) ;
        p148_uid_SET((uint64_t)2594835786036186779L, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)15, (uint8_t)82, (uint8_t)224, (uint8_t)123, (uint8_t)227, (uint8_t)53, (uint8_t)175, (uint8_t)156};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t flight_custom_version[] =  {(uint8_t)50, (uint8_t)190, (uint8_t)65, (uint8_t)243, (uint8_t)115, (uint8_t)38, (uint8_t)95, (uint8_t)86};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t uid2[] =  {(uint8_t)88, (uint8_t)82, (uint8_t)114, (uint8_t)66, (uint8_t)55, (uint8_t)209, (uint8_t)151, (uint8_t)170, (uint8_t)181, (uint8_t)253, (uint8_t)209, (uint8_t)137, (uint8_t)25, (uint8_t)84, (uint8_t)136, (uint8_t)12, (uint8_t)160, (uint8_t)80};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET, PH.base.pack) ;
        p148_os_sw_version_SET((uint32_t)3540612850L, PH.base.pack) ;
        p148_flight_sw_version_SET((uint32_t)4218438231L, PH.base.pack) ;
        p148_board_version_SET((uint32_t)3960252267L, PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)230, (uint8_t)166, (uint8_t)101, (uint8_t)136, (uint8_t)149, (uint8_t)249, (uint8_t)67, (uint8_t)48};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LANDING_TARGET_149(), &PH);
        p149_x_SET((float) -1.2791094E38F, &PH) ;
        p149_distance_SET((float)3.1731703E37F, PH.base.pack) ;
        p149_size_y_SET((float) -1.0287314E38F, PH.base.pack) ;
        p149_target_num_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        p149_y_SET((float)3.2696847E38F, &PH) ;
        p149_position_valid_SET((uint8_t)(uint8_t)112, &PH) ;
        p149_time_usec_SET((uint64_t)3524217692185537446L, PH.base.pack) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        p149_size_x_SET((float) -1.4439333E38F, PH.base.pack) ;
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON, PH.base.pack) ;
        {
            float q[] =  {1.6074327E38F, 1.587068E38F, -2.6074135E38F, -1.9125173E37F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_angle_x_SET((float) -1.5790047E38F, PH.base.pack) ;
        p149_z_SET((float) -4.625304E35F, &PH) ;
        p149_angle_y_SET((float)3.1041047E38F, PH.base.pack) ;
        c_CommunicationChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_pos_vert_accuracy_SET((float) -1.920556E38F, PH.base.pack) ;
        p230_tas_ratio_SET((float) -2.8322243E37F, PH.base.pack) ;
        p230_mag_ratio_SET((float)1.3411761E38F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)458122051073468469L, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float) -6.073017E37F, PH.base.pack) ;
        p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE, PH.base.pack) ;
        p230_hagl_ratio_SET((float) -1.0016293E38F, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float)1.1981848E38F, PH.base.pack) ;
        p230_vel_ratio_SET((float) -3.2219419E38F, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float)9.035926E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIND_COV_231(), &PH);
        p231_wind_x_SET((float) -3.1513508E38F, PH.base.pack) ;
        p231_vert_accuracy_SET((float)8.0174656E37F, PH.base.pack) ;
        p231_var_vert_SET((float)2.0974182E38F, PH.base.pack) ;
        p231_wind_alt_SET((float) -3.2568875E38F, PH.base.pack) ;
        p231_wind_y_SET((float)3.0845192E38F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float) -2.640934E38F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)2336586632304887778L, PH.base.pack) ;
        p231_var_horiz_SET((float) -2.9981192E38F, PH.base.pack) ;
        p231_wind_z_SET((float)1.3620373E38F, PH.base.pack) ;
        c_CommunicationChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_INPUT_232(), &PH);
        p232_ve_SET((float)2.172752E38F, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)33874, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p232_vd_SET((float) -5.312828E37F, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        p232_lat_SET((int32_t)1243583318, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)619800937L, PH.base.pack) ;
        p232_vdop_SET((float) -2.4410793E38F, PH.base.pack) ;
        p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY, PH.base.pack) ;
        p232_vn_SET((float)1.900894E38F, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)9157232551296201146L, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        p232_alt_SET((float) -3.1577773E38F, PH.base.pack) ;
        p232_vert_accuracy_SET((float)1.1758327E38F, PH.base.pack) ;
        p232_hdop_SET((float) -1.2214809E38F, PH.base.pack) ;
        p232_lon_SET((int32_t)1559763295, PH.base.pack) ;
        p232_horiz_accuracy_SET((float) -2.2124544E37F, PH.base.pack) ;
        p232_speed_accuracy_SET((float)2.0534031E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_RTCM_DATA_233(), &PH);
        p233_len_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        p233_flags_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)96, (uint8_t)12, (uint8_t)210, (uint8_t)25, (uint8_t)37, (uint8_t)30, (uint8_t)151, (uint8_t)60, (uint8_t)151, (uint8_t)138, (uint8_t)199, (uint8_t)196, (uint8_t)43, (uint8_t)247, (uint8_t)110, (uint8_t)48, (uint8_t)114, (uint8_t)165, (uint8_t)127, (uint8_t)227, (uint8_t)131, (uint8_t)254, (uint8_t)201, (uint8_t)144, (uint8_t)16, (uint8_t)74, (uint8_t)119, (uint8_t)33, (uint8_t)147, (uint8_t)234, (uint8_t)254, (uint8_t)116, (uint8_t)137, (uint8_t)136, (uint8_t)152, (uint8_t)13, (uint8_t)146, (uint8_t)103, (uint8_t)95, (uint8_t)114, (uint8_t)100, (uint8_t)254, (uint8_t)59, (uint8_t)191, (uint8_t)232, (uint8_t)230, (uint8_t)229, (uint8_t)248, (uint8_t)64, (uint8_t)14, (uint8_t)245, (uint8_t)167, (uint8_t)197, (uint8_t)97, (uint8_t)44, (uint8_t)103, (uint8_t)191, (uint8_t)223, (uint8_t)141, (uint8_t)253, (uint8_t)140, (uint8_t)77, (uint8_t)168, (uint8_t)187, (uint8_t)137, (uint8_t)81, (uint8_t)176, (uint8_t)66, (uint8_t)170, (uint8_t)47, (uint8_t)58, (uint8_t)191, (uint8_t)128, (uint8_t)201, (uint8_t)101, (uint8_t)236, (uint8_t)35, (uint8_t)136, (uint8_t)170, (uint8_t)249, (uint8_t)159, (uint8_t)130, (uint8_t)108, (uint8_t)240, (uint8_t)225, (uint8_t)24, (uint8_t)131, (uint8_t)207, (uint8_t)156, (uint8_t)211, (uint8_t)87, (uint8_t)106, (uint8_t)62, (uint8_t)242, (uint8_t)7, (uint8_t)192, (uint8_t)197, (uint8_t)64, (uint8_t)100, (uint8_t)210, (uint8_t)82, (uint8_t)1, (uint8_t)21, (uint8_t)222, (uint8_t)33, (uint8_t)123, (uint8_t)90, (uint8_t)159, (uint8_t)106, (uint8_t)114, (uint8_t)217, (uint8_t)49, (uint8_t)36, (uint8_t)101, (uint8_t)176, (uint8_t)186, (uint8_t)0, (uint8_t)44, (uint8_t)168, (uint8_t)207, (uint8_t)99, (uint8_t)207, (uint8_t)106, (uint8_t)35, (uint8_t)171, (uint8_t)37, (uint8_t)249, (uint8_t)187, (uint8_t)35, (uint8_t)56, (uint8_t)252, (uint8_t)149, (uint8_t)191, (uint8_t)166, (uint8_t)212, (uint8_t)180, (uint8_t)82, (uint8_t)221, (uint8_t)138, (uint8_t)35, (uint8_t)91, (uint8_t)181, (uint8_t)144, (uint8_t)242, (uint8_t)36, (uint8_t)100, (uint8_t)101, (uint8_t)64, (uint8_t)148, (uint8_t)243, (uint8_t)54, (uint8_t)147, (uint8_t)242, (uint8_t)214, (uint8_t)92, (uint8_t)120, (uint8_t)211, (uint8_t)60, (uint8_t)19, (uint8_t)182, (uint8_t)108, (uint8_t)241, (uint8_t)134, (uint8_t)240, (uint8_t)219, (uint8_t)248, (uint8_t)245, (uint8_t)231, (uint8_t)253, (uint8_t)155, (uint8_t)127, (uint8_t)192, (uint8_t)55, (uint8_t)8, (uint8_t)58, (uint8_t)19, (uint8_t)71, (uint8_t)220, (uint8_t)172, (uint8_t)20};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HIGH_LATENCY_234(), &PH);
        p234_heading_sp_SET((int16_t)(int16_t) -27036, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t) -10882, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t)27, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)670235846L, PH.base.pack) ;
        p234_latitude_SET((int32_t) -755283619, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t)54, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t)86, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)7103, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)42910, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t) -101, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t) -5561, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t)8480, PH.base.pack) ;
        p234_longitude_SET((int32_t) -1585980856, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t)7085, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, PH.base.pack) ;
        c_CommunicationChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIBRATION_241(), &PH);
        p241_vibration_z_SET((float)1.9286985E38F, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)3631332488L, PH.base.pack) ;
        p241_vibration_y_SET((float) -1.4091136E38F, PH.base.pack) ;
        p241_vibration_x_SET((float) -2.0242742E37F, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)8970279177692227870L, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)1000508911L, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)399335996L, PH.base.pack) ;
        c_CommunicationChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HOME_POSITION_242(), &PH);
        p242_longitude_SET((int32_t) -879890244, PH.base.pack) ;
        p242_approach_z_SET((float) -2.9492849E38F, PH.base.pack) ;
        p242_latitude_SET((int32_t)1630036899, PH.base.pack) ;
        {
            float q[] =  {2.739841E38F, -3.1571601E37F, -3.0655292E38F, 1.358867E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_x_SET((float) -3.1355708E37F, PH.base.pack) ;
        p242_approach_x_SET((float) -2.8365126E38F, PH.base.pack) ;
        p242_y_SET((float) -1.8440994E38F, PH.base.pack) ;
        p242_altitude_SET((int32_t) -1797860752, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)1171379104338470627L, &PH) ;
        p242_approach_y_SET((float) -2.6353497E38F, PH.base.pack) ;
        p242_z_SET((float) -2.4530142E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_HOME_POSITION_243(), &PH);
        p243_altitude_SET((int32_t)1978935974, PH.base.pack) ;
        p243_approach_x_SET((float)6.3864706E36F, PH.base.pack) ;
        p243_y_SET((float)2.5845518E38F, PH.base.pack) ;
        p243_longitude_SET((int32_t)810225029, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)4842925762552016569L, &PH) ;
        p243_approach_z_SET((float)1.1586706E38F, PH.base.pack) ;
        p243_z_SET((float) -9.575267E37F, PH.base.pack) ;
        {
            float q[] =  {1.053879E38F, -4.2280338E37F, -5.177782E37F, 2.2434531E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_latitude_SET((int32_t) -783338466, PH.base.pack) ;
        p243_x_SET((float) -1.5966577E37F, PH.base.pack) ;
        p243_approach_y_SET((float) -3.1858033E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_message_id_SET((uint16_t)(uint16_t)46746, PH.base.pack) ;
        p244_interval_us_SET((int32_t) -1544289579, PH.base.pack) ;
        c_CommunicationChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW, PH.base.pack) ;
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, PH.base.pack) ;
        c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ADSB_VEHICLE_246(), &PH);
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t)12555, PH.base.pack) ;
        p246_altitude_SET((int32_t)601224448, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)56235, PH.base.pack) ;
        p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)7550, PH.base.pack) ;
        p246_lat_SET((int32_t) -1397242436, PH.base.pack) ;
        p246_lon_SET((int32_t)254333280, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_ULTRA_LIGHT, PH.base.pack) ;
        {
            char16_t* callsign = u"fgf";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_ICAO_address_SET((uint32_t)1750075344L, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)49319, PH.base.pack) ;
        c_CommunicationChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_COLLISION_247(), &PH);
        p247_id_SET((uint32_t)155619091L, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float) -2.820458E37F, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_HORIZONTALLY, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float)2.74995E38F, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float) -1.9374388E38F, PH.base.pack) ;
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, PH.base.pack) ;
        c_CommunicationChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_V2_EXTENSION_248(), &PH);
        p248_target_component_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        p248_target_system_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        p248_message_type_SET((uint16_t)(uint16_t)57662, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)189, (uint8_t)221, (uint8_t)70, (uint8_t)231, (uint8_t)172, (uint8_t)58, (uint8_t)242, (uint8_t)141, (uint8_t)109, (uint8_t)109, (uint8_t)90, (uint8_t)5, (uint8_t)246, (uint8_t)125, (uint8_t)188, (uint8_t)197, (uint8_t)40, (uint8_t)87, (uint8_t)242, (uint8_t)161, (uint8_t)106, (uint8_t)177, (uint8_t)76, (uint8_t)15, (uint8_t)130, (uint8_t)155, (uint8_t)155, (uint8_t)120, (uint8_t)155, (uint8_t)249, (uint8_t)120, (uint8_t)98, (uint8_t)142, (uint8_t)129, (uint8_t)2, (uint8_t)90, (uint8_t)106, (uint8_t)105, (uint8_t)233, (uint8_t)123, (uint8_t)110, (uint8_t)145, (uint8_t)179, (uint8_t)226, (uint8_t)89, (uint8_t)216, (uint8_t)137, (uint8_t)196, (uint8_t)243, (uint8_t)51, (uint8_t)239, (uint8_t)137, (uint8_t)13, (uint8_t)220, (uint8_t)8, (uint8_t)143, (uint8_t)135, (uint8_t)149, (uint8_t)236, (uint8_t)155, (uint8_t)128, (uint8_t)199, (uint8_t)147, (uint8_t)165, (uint8_t)129, (uint8_t)128, (uint8_t)137, (uint8_t)35, (uint8_t)101, (uint8_t)133, (uint8_t)233, (uint8_t)104, (uint8_t)170, (uint8_t)2, (uint8_t)153, (uint8_t)60, (uint8_t)201, (uint8_t)30, (uint8_t)231, (uint8_t)89, (uint8_t)92, (uint8_t)125, (uint8_t)81, (uint8_t)186, (uint8_t)43, (uint8_t)250, (uint8_t)25, (uint8_t)25, (uint8_t)90, (uint8_t)161, (uint8_t)74, (uint8_t)244, (uint8_t)54, (uint8_t)56, (uint8_t)84, (uint8_t)229, (uint8_t)152, (uint8_t)128, (uint8_t)149, (uint8_t)115, (uint8_t)28, (uint8_t)39, (uint8_t)59, (uint8_t)123, (uint8_t)12, (uint8_t)44, (uint8_t)95, (uint8_t)180, (uint8_t)87, (uint8_t)242, (uint8_t)217, (uint8_t)205, (uint8_t)44, (uint8_t)94, (uint8_t)123, (uint8_t)52, (uint8_t)214, (uint8_t)224, (uint8_t)122, (uint8_t)45, (uint8_t)2, (uint8_t)16, (uint8_t)181, (uint8_t)79, (uint8_t)17, (uint8_t)136, (uint8_t)51, (uint8_t)202, (uint8_t)182, (uint8_t)3, (uint8_t)86, (uint8_t)68, (uint8_t)200, (uint8_t)53, (uint8_t)160, (uint8_t)85, (uint8_t)157, (uint8_t)165, (uint8_t)167, (uint8_t)47, (uint8_t)192, (uint8_t)204, (uint8_t)217, (uint8_t)70, (uint8_t)97, (uint8_t)179, (uint8_t)158, (uint8_t)180, (uint8_t)20, (uint8_t)3, (uint8_t)169, (uint8_t)10, (uint8_t)41, (uint8_t)242, (uint8_t)175, (uint8_t)142, (uint8_t)78, (uint8_t)105, (uint8_t)38, (uint8_t)63, (uint8_t)134, (uint8_t)254, (uint8_t)207, (uint8_t)190, (uint8_t)216, (uint8_t)69, (uint8_t)194, (uint8_t)77, (uint8_t)146, (uint8_t)95, (uint8_t)178, (uint8_t)36, (uint8_t)73, (uint8_t)13, (uint8_t)210, (uint8_t)31, (uint8_t)8, (uint8_t)240, (uint8_t)168, (uint8_t)15, (uint8_t)230, (uint8_t)157, (uint8_t)158, (uint8_t)3, (uint8_t)37, (uint8_t)146, (uint8_t)6, (uint8_t)253, (uint8_t)185, (uint8_t)57, (uint8_t)199, (uint8_t)99, (uint8_t)108, (uint8_t)45, (uint8_t)141, (uint8_t)134, (uint8_t)86, (uint8_t)205, (uint8_t)24, (uint8_t)104, (uint8_t)201, (uint8_t)248, (uint8_t)202, (uint8_t)86, (uint8_t)248, (uint8_t)61, (uint8_t)217, (uint8_t)104, (uint8_t)249, (uint8_t)71, (uint8_t)253, (uint8_t)142, (uint8_t)143, (uint8_t)141, (uint8_t)200, (uint8_t)20, (uint8_t)181, (uint8_t)72, (uint8_t)220, (uint8_t)192, (uint8_t)90, (uint8_t)217, (uint8_t)52, (uint8_t)155, (uint8_t)33, (uint8_t)72, (uint8_t)12, (uint8_t)185, (uint8_t)208, (uint8_t)242, (uint8_t)83, (uint8_t)164, (uint8_t)44, (uint8_t)38, (uint8_t)159, (uint8_t)226, (uint8_t)95, (uint8_t)64, (uint8_t)236, (uint8_t)151, (uint8_t)154, (uint8_t)147, (uint8_t)37, (uint8_t)119, (uint8_t)137, (uint8_t)154, (uint8_t)37, (uint8_t)60, (uint8_t)188};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p248_target_network_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        c_CommunicationChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MEMORY_VECT_249(), &PH);
        p249_type_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        p249_address_SET((uint16_t)(uint16_t)49165, PH.base.pack) ;
        p249_ver_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t) -17, (int8_t)59, (int8_t)93, (int8_t) -9, (int8_t) -119, (int8_t)116, (int8_t)127, (int8_t) -4, (int8_t) -51, (int8_t) -115, (int8_t)124, (int8_t) -49, (int8_t)94, (int8_t) -124, (int8_t)88, (int8_t) -70, (int8_t)124, (int8_t) -52, (int8_t)126, (int8_t) -81, (int8_t) -36, (int8_t)30, (int8_t) -70, (int8_t) -93, (int8_t) -23, (int8_t) -45, (int8_t) -60, (int8_t) -111, (int8_t) -43, (int8_t)72, (int8_t)90, (int8_t) -2};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_VECT_250(), &PH);
        p250_x_SET((float)2.0048541E38F, PH.base.pack) ;
        p250_z_SET((float)9.525829E37F, PH.base.pack) ;
        p250_y_SET((float)1.8717147E38F, PH.base.pack) ;
        p250_time_usec_SET((uint64_t)3699949712977014533L, PH.base.pack) ;
        {
            char16_t* name = u"sbjcoueh";
            p250_name_SET_(name, &PH) ;
        }
        c_CommunicationChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_FLOAT_251(), &PH);
        p251_time_boot_ms_SET((uint32_t)3383671205L, PH.base.pack) ;
        {
            char16_t* name = u"Ge";
            p251_name_SET_(name, &PH) ;
        }
        p251_value_SET((float) -3.1888672E38F, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_INT_252(), &PH);
        p252_value_SET((int32_t)532688143, PH.base.pack) ;
        p252_time_boot_ms_SET((uint32_t)3439804464L, PH.base.pack) ;
        {
            char16_t* name = u"vayzknshbf";
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
            char16_t* text = u"uvBgkrItoetvqddomOqwourdsugjxmvc";
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
        p254_ind_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)2074052379L, PH.base.pack) ;
        p254_value_SET((float)9.326201E37F, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SETUP_SIGNING_256(), &PH);
        p256_initial_timestamp_SET((uint64_t)4217961436245929757L, PH.base.pack) ;
        p256_target_system_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p256_target_component_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)54, (uint8_t)57, (uint8_t)112, (uint8_t)233, (uint8_t)128, (uint8_t)109, (uint8_t)154, (uint8_t)214, (uint8_t)5, (uint8_t)195, (uint8_t)186, (uint8_t)27, (uint8_t)181, (uint8_t)226, (uint8_t)107, (uint8_t)39, (uint8_t)214, (uint8_t)37, (uint8_t)118, (uint8_t)30, (uint8_t)113, (uint8_t)65, (uint8_t)192, (uint8_t)97, (uint8_t)164, (uint8_t)84, (uint8_t)22, (uint8_t)110, (uint8_t)203, (uint8_t)166, (uint8_t)195, (uint8_t)156};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BUTTON_CHANGE_257(), &PH);
        p257_last_change_ms_SET((uint32_t)1255025544L, PH.base.pack) ;
        p257_time_boot_ms_SET((uint32_t)3440392118L, PH.base.pack) ;
        p257_state_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
        c_CommunicationChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PLAY_TUNE_258(), &PH);
        {
            char16_t* tune = u"r";
            p258_tune_SET_(tune, &PH) ;
        }
        p258_target_component_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        p258_target_system_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
        c_CommunicationChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_INFORMATION_259(), &PH);
        p259_time_boot_ms_SET((uint32_t)1908702952L, PH.base.pack) ;
        p259_cam_definition_version_SET((uint16_t)(uint16_t)13913, PH.base.pack) ;
        p259_sensor_size_h_SET((float) -1.0950048E38F, PH.base.pack) ;
        p259_lens_id_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)27347, PH.base.pack) ;
        p259_focal_length_SET((float)2.3028774E38F, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)196, (uint8_t)145, (uint8_t)37, (uint8_t)159, (uint8_t)249, (uint8_t)110, (uint8_t)212, (uint8_t)31, (uint8_t)206, (uint8_t)57, (uint8_t)102, (uint8_t)96, (uint8_t)167, (uint8_t)109, (uint8_t)86, (uint8_t)25, (uint8_t)209, (uint8_t)191, (uint8_t)185, (uint8_t)180, (uint8_t)168, (uint8_t)149, (uint8_t)84, (uint8_t)84, (uint8_t)49, (uint8_t)247, (uint8_t)190, (uint8_t)153, (uint8_t)156, (uint8_t)5, (uint8_t)221, (uint8_t)179};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        {
            char16_t* cam_definition_uri = u"mdefrpyjoohupuFqyuwokrulvdgwxlpjbmMkGnmpGuonvleezXkrSllejpjaayuSzkTauDLpjeBatlEpgszmluCsmtjwibhzjymiGuaftiXksvwdkwe";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_sensor_size_v_SET((float)2.1928816E38F, PH.base.pack) ;
        p259_resolution_v_SET((uint16_t)(uint16_t)13013, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)48, (uint8_t)170, (uint8_t)129, (uint8_t)135, (uint8_t)120, (uint8_t)107, (uint8_t)253, (uint8_t)209, (uint8_t)165, (uint8_t)31, (uint8_t)136, (uint8_t)54, (uint8_t)138, (uint8_t)98, (uint8_t)81, (uint8_t)73, (uint8_t)73, (uint8_t)61, (uint8_t)105, (uint8_t)57, (uint8_t)50, (uint8_t)8, (uint8_t)213, (uint8_t)195, (uint8_t)126, (uint8_t)17, (uint8_t)64, (uint8_t)78, (uint8_t)159, (uint8_t)174, (uint8_t)117, (uint8_t)88};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_firmware_version_SET((uint32_t)3414453684L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_SETTINGS_260(), &PH);
        p260_time_boot_ms_SET((uint32_t)3844437543L, PH.base.pack) ;
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STORAGE_INFORMATION_261(), &PH);
        p261_storage_count_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        p261_write_speed_SET((float)2.4142434E38F, PH.base.pack) ;
        p261_used_capacity_SET((float) -2.1259401E38F, PH.base.pack) ;
        p261_read_speed_SET((float) -3.0214024E38F, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)369446823L, PH.base.pack) ;
        p261_available_capacity_SET((float)2.9172966E37F, PH.base.pack) ;
        p261_total_capacity_SET((float) -2.9027678E38F, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        c_CommunicationChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_image_status_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p262_available_capacity_SET((float) -2.5149407E38F, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)2179558520L, PH.base.pack) ;
        p262_image_interval_SET((float) -3.6851128E37F, PH.base.pack) ;
        p262_recording_time_ms_SET((uint32_t)448837371L, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_time_utc_SET((uint64_t)3494969922375431417L, PH.base.pack) ;
        {
            float q[] =  {-2.1644392E38F, 4.3834475E37F, -7.081031E37F, 2.0900936E38F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        {
            char16_t* file_url = u"bluvbymxvqpzqteMewiJlduj";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_camera_id_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        p263_lat_SET((int32_t)1226509761, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t)123, PH.base.pack) ;
        p263_lon_SET((int32_t) -2129276720, PH.base.pack) ;
        p263_alt_SET((int32_t)788043192, PH.base.pack) ;
        p263_relative_alt_SET((int32_t)188459201, PH.base.pack) ;
        p263_image_index_SET((int32_t) -1858746548, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)2260959201L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_takeoff_time_utc_SET((uint64_t)7042662703160545242L, PH.base.pack) ;
        p264_time_boot_ms_SET((uint32_t)55443782L, PH.base.pack) ;
        p264_arming_time_utc_SET((uint64_t)5346752962439253047L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)1747591698458444238L, PH.base.pack) ;
        c_CommunicationChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_yaw_SET((float) -2.1447638E38F, PH.base.pack) ;
        p265_roll_SET((float) -4.428499E37F, PH.base.pack) ;
        p265_pitch_SET((float)3.2605223E38F, PH.base.pack) ;
        p265_time_boot_ms_SET((uint32_t)969844230L, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_266(), &PH);
        p266_first_message_offset_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)108, (uint8_t)187, (uint8_t)178, (uint8_t)254, (uint8_t)138, (uint8_t)250, (uint8_t)121, (uint8_t)106, (uint8_t)221, (uint8_t)220, (uint8_t)245, (uint8_t)160, (uint8_t)33, (uint8_t)163, (uint8_t)117, (uint8_t)56, (uint8_t)58, (uint8_t)202, (uint8_t)10, (uint8_t)173, (uint8_t)2, (uint8_t)213, (uint8_t)244, (uint8_t)144, (uint8_t)139, (uint8_t)178, (uint8_t)147, (uint8_t)181, (uint8_t)240, (uint8_t)167, (uint8_t)230, (uint8_t)26, (uint8_t)88, (uint8_t)18, (uint8_t)133, (uint8_t)250, (uint8_t)198, (uint8_t)33, (uint8_t)197, (uint8_t)177, (uint8_t)153, (uint8_t)126, (uint8_t)241, (uint8_t)115, (uint8_t)81, (uint8_t)60, (uint8_t)43, (uint8_t)142, (uint8_t)12, (uint8_t)242, (uint8_t)169, (uint8_t)11, (uint8_t)175, (uint8_t)61, (uint8_t)115, (uint8_t)67, (uint8_t)72, (uint8_t)63, (uint8_t)9, (uint8_t)24, (uint8_t)198, (uint8_t)108, (uint8_t)26, (uint8_t)205, (uint8_t)188, (uint8_t)81, (uint8_t)86, (uint8_t)208, (uint8_t)208, (uint8_t)63, (uint8_t)175, (uint8_t)63, (uint8_t)25, (uint8_t)185, (uint8_t)142, (uint8_t)68, (uint8_t)222, (uint8_t)42, (uint8_t)30, (uint8_t)183, (uint8_t)34, (uint8_t)233, (uint8_t)222, (uint8_t)79, (uint8_t)64, (uint8_t)130, (uint8_t)198, (uint8_t)252, (uint8_t)26, (uint8_t)159, (uint8_t)109, (uint8_t)127, (uint8_t)215, (uint8_t)139, (uint8_t)11, (uint8_t)116, (uint8_t)191, (uint8_t)173, (uint8_t)96, (uint8_t)135, (uint8_t)33, (uint8_t)111, (uint8_t)23, (uint8_t)199, (uint8_t)5, (uint8_t)97, (uint8_t)192, (uint8_t)0, (uint8_t)252, (uint8_t)114, (uint8_t)234, (uint8_t)200, (uint8_t)108, (uint8_t)214, (uint8_t)6, (uint8_t)114, (uint8_t)212, (uint8_t)60, (uint8_t)81, (uint8_t)86, (uint8_t)10, (uint8_t)143, (uint8_t)90, (uint8_t)124, (uint8_t)101, (uint8_t)184, (uint8_t)32, (uint8_t)84, (uint8_t)153, (uint8_t)180, (uint8_t)205, (uint8_t)69, (uint8_t)216, (uint8_t)95, (uint8_t)109, (uint8_t)116, (uint8_t)244, (uint8_t)160, (uint8_t)73, (uint8_t)16, (uint8_t)56, (uint8_t)28, (uint8_t)154, (uint8_t)32, (uint8_t)7, (uint8_t)8, (uint8_t)243, (uint8_t)129, (uint8_t)0, (uint8_t)11, (uint8_t)168, (uint8_t)67, (uint8_t)25, (uint8_t)219, (uint8_t)191, (uint8_t)140, (uint8_t)75, (uint8_t)89, (uint8_t)60, (uint8_t)151, (uint8_t)191, (uint8_t)175, (uint8_t)26, (uint8_t)213, (uint8_t)99, (uint8_t)110, (uint8_t)227, (uint8_t)55, (uint8_t)133, (uint8_t)103, (uint8_t)201, (uint8_t)114, (uint8_t)45, (uint8_t)253, (uint8_t)116, (uint8_t)240, (uint8_t)183, (uint8_t)244, (uint8_t)164, (uint8_t)62, (uint8_t)25, (uint8_t)57, (uint8_t)249, (uint8_t)131, (uint8_t)60, (uint8_t)250, (uint8_t)234, (uint8_t)209, (uint8_t)242, (uint8_t)229, (uint8_t)254, (uint8_t)111, (uint8_t)53, (uint8_t)63, (uint8_t)146, (uint8_t)246, (uint8_t)199, (uint8_t)201, (uint8_t)211, (uint8_t)220, (uint8_t)184, (uint8_t)215, (uint8_t)249, (uint8_t)113, (uint8_t)103, (uint8_t)246, (uint8_t)116, (uint8_t)55, (uint8_t)66, (uint8_t)150, (uint8_t)200, (uint8_t)139, (uint8_t)40, (uint8_t)118, (uint8_t)179, (uint8_t)134, (uint8_t)172, (uint8_t)42, (uint8_t)195, (uint8_t)70, (uint8_t)233, (uint8_t)110, (uint8_t)39, (uint8_t)145, (uint8_t)45, (uint8_t)15, (uint8_t)60, (uint8_t)42, (uint8_t)243, (uint8_t)254, (uint8_t)147, (uint8_t)145, (uint8_t)176, (uint8_t)71, (uint8_t)15, (uint8_t)195, (uint8_t)238, (uint8_t)200, (uint8_t)241, (uint8_t)184, (uint8_t)200, (uint8_t)167, (uint8_t)179, (uint8_t)54, (uint8_t)64, (uint8_t)122, (uint8_t)150, (uint8_t)138, (uint8_t)64};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_length_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        p266_sequence_SET((uint16_t)(uint16_t)28672, PH.base.pack) ;
        p266_target_system_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_ACKED_267(), &PH);
        p267_first_message_offset_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        p267_target_component_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)120, (uint8_t)50, (uint8_t)187, (uint8_t)51, (uint8_t)249, (uint8_t)105, (uint8_t)124, (uint8_t)155, (uint8_t)202, (uint8_t)143, (uint8_t)82, (uint8_t)124, (uint8_t)144, (uint8_t)3, (uint8_t)139, (uint8_t)218, (uint8_t)4, (uint8_t)69, (uint8_t)97, (uint8_t)11, (uint8_t)45, (uint8_t)249, (uint8_t)64, (uint8_t)97, (uint8_t)100, (uint8_t)190, (uint8_t)48, (uint8_t)192, (uint8_t)119, (uint8_t)208, (uint8_t)229, (uint8_t)75, (uint8_t)116, (uint8_t)12, (uint8_t)219, (uint8_t)16, (uint8_t)13, (uint8_t)255, (uint8_t)36, (uint8_t)113, (uint8_t)126, (uint8_t)1, (uint8_t)143, (uint8_t)15, (uint8_t)234, (uint8_t)125, (uint8_t)136, (uint8_t)74, (uint8_t)230, (uint8_t)27, (uint8_t)233, (uint8_t)146, (uint8_t)52, (uint8_t)152, (uint8_t)75, (uint8_t)89, (uint8_t)147, (uint8_t)91, (uint8_t)72, (uint8_t)38, (uint8_t)169, (uint8_t)12, (uint8_t)197, (uint8_t)86, (uint8_t)39, (uint8_t)157, (uint8_t)181, (uint8_t)110, (uint8_t)131, (uint8_t)223, (uint8_t)231, (uint8_t)88, (uint8_t)253, (uint8_t)99, (uint8_t)208, (uint8_t)47, (uint8_t)145, (uint8_t)206, (uint8_t)209, (uint8_t)158, (uint8_t)210, (uint8_t)0, (uint8_t)198, (uint8_t)7, (uint8_t)213, (uint8_t)32, (uint8_t)30, (uint8_t)43, (uint8_t)29, (uint8_t)110, (uint8_t)4, (uint8_t)80, (uint8_t)42, (uint8_t)248, (uint8_t)51, (uint8_t)95, (uint8_t)5, (uint8_t)217, (uint8_t)157, (uint8_t)23, (uint8_t)21, (uint8_t)119, (uint8_t)57, (uint8_t)131, (uint8_t)182, (uint8_t)109, (uint8_t)187, (uint8_t)104, (uint8_t)128, (uint8_t)186, (uint8_t)225, (uint8_t)44, (uint8_t)236, (uint8_t)157, (uint8_t)124, (uint8_t)224, (uint8_t)25, (uint8_t)68, (uint8_t)105, (uint8_t)206, (uint8_t)18, (uint8_t)186, (uint8_t)69, (uint8_t)39, (uint8_t)179, (uint8_t)204, (uint8_t)166, (uint8_t)205, (uint8_t)141, (uint8_t)6, (uint8_t)120, (uint8_t)118, (uint8_t)17, (uint8_t)147, (uint8_t)153, (uint8_t)90, (uint8_t)251, (uint8_t)174, (uint8_t)110, (uint8_t)199, (uint8_t)29, (uint8_t)157, (uint8_t)96, (uint8_t)226, (uint8_t)189, (uint8_t)59, (uint8_t)145, (uint8_t)119, (uint8_t)103, (uint8_t)243, (uint8_t)23, (uint8_t)192, (uint8_t)25, (uint8_t)22, (uint8_t)126, (uint8_t)19, (uint8_t)171, (uint8_t)112, (uint8_t)10, (uint8_t)85, (uint8_t)4, (uint8_t)179, (uint8_t)192, (uint8_t)93, (uint8_t)217, (uint8_t)208, (uint8_t)207, (uint8_t)100, (uint8_t)151, (uint8_t)131, (uint8_t)229, (uint8_t)5, (uint8_t)40, (uint8_t)42, (uint8_t)186, (uint8_t)144, (uint8_t)252, (uint8_t)145, (uint8_t)232, (uint8_t)144, (uint8_t)14, (uint8_t)67, (uint8_t)221, (uint8_t)188, (uint8_t)108, (uint8_t)52, (uint8_t)104, (uint8_t)13, (uint8_t)18, (uint8_t)227, (uint8_t)181, (uint8_t)147, (uint8_t)215, (uint8_t)164, (uint8_t)213, (uint8_t)86, (uint8_t)44, (uint8_t)139, (uint8_t)88, (uint8_t)61, (uint8_t)218, (uint8_t)126, (uint8_t)243, (uint8_t)95, (uint8_t)146, (uint8_t)153, (uint8_t)172, (uint8_t)62, (uint8_t)188, (uint8_t)170, (uint8_t)14, (uint8_t)252, (uint8_t)203, (uint8_t)207, (uint8_t)76, (uint8_t)107, (uint8_t)33, (uint8_t)33, (uint8_t)205, (uint8_t)12, (uint8_t)66, (uint8_t)19, (uint8_t)209, (uint8_t)181, (uint8_t)151, (uint8_t)201, (uint8_t)60, (uint8_t)89, (uint8_t)122, (uint8_t)113, (uint8_t)214, (uint8_t)1, (uint8_t)186, (uint8_t)113, (uint8_t)225, (uint8_t)193, (uint8_t)154, (uint8_t)249, (uint8_t)123, (uint8_t)141, (uint8_t)51, (uint8_t)148, (uint8_t)201, (uint8_t)173, (uint8_t)14, (uint8_t)3, (uint8_t)144, (uint8_t)187, (uint8_t)146};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_sequence_SET((uint16_t)(uint16_t)14249, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_ACK_268(), &PH);
        p268_target_system_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        p268_sequence_SET((uint16_t)(uint16_t)32098, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_framerate_SET((float) -9.563977E37F, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)33494, PH.base.pack) ;
        p269_bitrate_SET((uint32_t)2475350522L, PH.base.pack) ;
        p269_rotation_SET((uint16_t)(uint16_t)4305, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)26421, PH.base.pack) ;
        {
            char16_t* uri = u"PhlVCtqgkwFyxaezcikuqpygkatajKvuaizeBOszosltRjbnynpxtiujlruaZacjucrkfqltodohvvdhhodXwrsnmRknmdhFgcxjksuwMiqszurlcvkmRnftsbbvgyzhqnehttFyiumNegfgicqxutmdrfpao";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_status_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_framerate_SET((float) -3.143124E36F, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)52540, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)49948, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)2235839958L, PH.base.pack) ;
        p270_target_component_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)51394, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        {
            char16_t* uri = u"rwa";
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
            char16_t* ssid = u"hjiiCfoFhtdfyhKzHospevqrriqxe";
            p299_ssid_SET_(ssid, &PH) ;
        }
        {
            char16_t* password = u"srrmwlxsuAxAydpuxGanPfcmLjrbnsmo";
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
            uint8_t spec_version_hash[] =  {(uint8_t)156, (uint8_t)152, (uint8_t)245, (uint8_t)129, (uint8_t)117, (uint8_t)171, (uint8_t)144, (uint8_t)173};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        p300_min_version_SET((uint16_t)(uint16_t)18937, PH.base.pack) ;
        p300_max_version_SET((uint16_t)(uint16_t)56447, PH.base.pack) ;
        p300_version_SET((uint16_t)(uint16_t)56162, PH.base.pack) ;
        {
            uint8_t library_version_hash[] =  {(uint8_t)235, (uint8_t)204, (uint8_t)126, (uint8_t)95, (uint8_t)16, (uint8_t)50, (uint8_t)187, (uint8_t)31};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)64593, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)63184929590684970L, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)1963039189L, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_uptime_sec_SET((uint32_t)3551104389L, PH.base.pack) ;
        p311_sw_version_major_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        p311_hw_version_minor_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        p311_sw_version_minor_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)4275976630L, PH.base.pack) ;
        {
            char16_t* name = u"umxabCydFrbClZmjKlqkaTlmlcoficlfgygbpfxvytqzvtrKhnfAuavhopWmtkzowrdqq";
            p311_name_SET_(name, &PH) ;
        }
        {
            uint8_t hw_unique_id[] =  {(uint8_t)29, (uint8_t)85, (uint8_t)104, (uint8_t)74, (uint8_t)15, (uint8_t)227, (uint8_t)197, (uint8_t)51, (uint8_t)176, (uint8_t)16, (uint8_t)246, (uint8_t)177, (uint8_t)217, (uint8_t)103, (uint8_t)120, (uint8_t)154};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        p311_time_usec_SET((uint64_t)2283214408316546566L, PH.base.pack) ;
        p311_hw_version_major_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        {
            char16_t* param_id = u"kdnhtxbocxdrk";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_param_index_SET((int16_t)(int16_t)14063, PH.base.pack) ;
        p320_target_system_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        p320_target_component_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_system_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        p321_target_component_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_VALUE_322(), &PH);
        {
            char16_t* param_id = u"kyhvygei";
            p322_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"zrmHptfcnLdcuztPmrewvysquceluFyQaarWptmhbqapTeefxmmjlpiivsudozkyzlyjNviecaksjszsbbjbihnsojgebukmhdbcoqixxnZjJgiairkvzfxfuasmwai";
            p322_param_value_SET_(param_value, &PH) ;
        }
        p322_param_index_SET((uint16_t)(uint16_t)21368, PH.base.pack) ;
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16, PH.base.pack) ;
        p322_param_count_SET((uint16_t)(uint16_t)34217, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_SET_323(), &PH);
        p323_target_system_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p323_target_component_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16, PH.base.pack) ;
        {
            char16_t* param_id = u"tquvrogglfosb";
            p323_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"uweywjnyqEcuvszsolqefpaldnsklqfduhyjbjYrwtmbbxl";
            p323_param_value_SET_(param_value, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_ACK_324(), &PH);
        {
            char16_t* param_value = u"csGcswcnFcuktghfprlskXXsjgGSkxqrxelzAiwmuoxrxbwxrhmxlyEticwmynEGisceaUuqtervsxkxfrqjsiyac";
            p324_param_value_SET_(param_value, &PH) ;
        }
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_FAILED, PH.base.pack) ;
        {
            char16_t* param_id = u"alerjxqn";
            p324_param_id_SET_(param_id, &PH) ;
        }
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_OBSTACLE_DISTANCE_330(), &PH);
        p330_max_distance_SET((uint16_t)(uint16_t)43236, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)48338, (uint16_t)41453, (uint16_t)204, (uint16_t)3262, (uint16_t)63208, (uint16_t)31564, (uint16_t)52364, (uint16_t)14614, (uint16_t)9421, (uint16_t)25658, (uint16_t)52815, (uint16_t)2739, (uint16_t)60154, (uint16_t)7059, (uint16_t)22026, (uint16_t)30192, (uint16_t)18286, (uint16_t)55210, (uint16_t)31923, (uint16_t)26647, (uint16_t)6048, (uint16_t)11687, (uint16_t)12371, (uint16_t)5036, (uint16_t)39164, (uint16_t)9655, (uint16_t)61350, (uint16_t)32016, (uint16_t)23997, (uint16_t)54990, (uint16_t)79, (uint16_t)26926, (uint16_t)47170, (uint16_t)64320, (uint16_t)2815, (uint16_t)52653, (uint16_t)38351, (uint16_t)2387, (uint16_t)32453, (uint16_t)45805, (uint16_t)47179, (uint16_t)41261, (uint16_t)3002, (uint16_t)35159, (uint16_t)40148, (uint16_t)2258, (uint16_t)63676, (uint16_t)3539, (uint16_t)37164, (uint16_t)55421, (uint16_t)36815, (uint16_t)8118, (uint16_t)30171, (uint16_t)8596, (uint16_t)63435, (uint16_t)24008, (uint16_t)19694, (uint16_t)56647, (uint16_t)46717, (uint16_t)21273, (uint16_t)63002, (uint16_t)60743, (uint16_t)62874, (uint16_t)52053, (uint16_t)56619, (uint16_t)53251, (uint16_t)60366, (uint16_t)47190, (uint16_t)59409, (uint16_t)12000, (uint16_t)9942, (uint16_t)19973};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
        p330_time_usec_SET((uint64_t)5741823915613362407L, PH.base.pack) ;
        p330_increment_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)49923, PH.base.pack) ;
        c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
}



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
/**
*Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
*	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
*	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
*	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
*	bit 11: yaw, bit 12: yaw rat*/
INLINER uint16_t p3_type_mask_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint32_t p3_time_boot_ms_GET(Pack * src)//Timestamp in milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER float p3_x_GET(Pack * src)//X Position in NED frame in meters
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  6, 4)));
}
INLINER float p3_y_GET(Pack * src)//Y Position in NED frame in meters
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  10, 4)));
}
INLINER float p3_z_GET(Pack * src)//Z Position in NED frame in meters (note, altitude is negative in NED)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER float p3_vx_GET(Pack * src)//X velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER float p3_vy_GET(Pack * src)//Y velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
INLINER float p3_vz_GET(Pack * src)//Z velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  26, 4)));
}
INLINER float p3_afx_GET(Pack * src)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  30, 4)));
}
INLINER float p3_afy_GET(Pack * src)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  34, 4)));
}
INLINER float p3_afz_GET(Pack * src)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  38, 4)));
}
INLINER float p3_yaw_GET(Pack * src)//yaw setpoint in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  42, 4)));
}
INLINER float p3_yaw_rate_GET(Pack * src)//yaw rate setpoint in rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  46, 4)));
}
/**
*Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
*	=*/
INLINER e_MAV_FRAME p3_coordinate_frame_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 400, 4);
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
            return e_MAV_CMD_MAV_CMD_NAV_ALTITUDE_WAIT;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 23:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 24:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 25:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 28:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 29:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 30:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 56:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_GRIPPER;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_AUTOTUNE_ENABLE;
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
        case 130:
            return e_MAV_CMD_MAV_CMD_POWER_OFF_INITIATED;
        case 131:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_CLICK;
        case 132:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_HOLD;
        case 133:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_PAUSE_CLICK;
        case 134:
            return e_MAV_CMD_MAV_CMD_DO_START_MAG_CAL;
        case 135:
            return e_MAV_CMD_MAV_CMD_DO_ACCEPT_MAG_CAL;
        case 136:
            return e_MAV_CMD_MAV_CMD_DO_CANCEL_MAG_CAL;
        case 137:
            return e_MAV_CMD_MAV_CMD_SET_FACTORY_TEST_MODE;
        case 138:
            return e_MAV_CMD_MAV_CMD_DO_SEND_BANNER;
        case 139:
            return e_MAV_CMD_MAV_CMD_ACCELCAL_VEHICLE_POS;
        case 140:
            return e_MAV_CMD_MAV_CMD_GIMBAL_RESET;
        case 141:
            return e_MAV_CMD_MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS;
        case 142:
            return e_MAV_CMD_MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION;
        case 143:
            return e_MAV_CMD_MAV_CMD_GIMBAL_FULL_RESET;
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
            return e_MAV_CMD_MAV_CMD_NAV_ALTITUDE_WAIT;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 23:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 24:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 25:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 28:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 29:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 30:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 56:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_GRIPPER;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_AUTOTUNE_ENABLE;
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
        case 130:
            return e_MAV_CMD_MAV_CMD_POWER_OFF_INITIATED;
        case 131:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_CLICK;
        case 132:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_HOLD;
        case 133:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_PAUSE_CLICK;
        case 134:
            return e_MAV_CMD_MAV_CMD_DO_START_MAG_CAL;
        case 135:
            return e_MAV_CMD_MAV_CMD_DO_ACCEPT_MAG_CAL;
        case 136:
            return e_MAV_CMD_MAV_CMD_DO_CANCEL_MAG_CAL;
        case 137:
            return e_MAV_CMD_MAV_CMD_SET_FACTORY_TEST_MODE;
        case 138:
            return e_MAV_CMD_MAV_CMD_DO_SEND_BANNER;
        case 139:
            return e_MAV_CMD_MAV_CMD_ACCELCAL_VEHICLE_POS;
        case 140:
            return e_MAV_CMD_MAV_CMD_GIMBAL_RESET;
        case 141:
            return e_MAV_CMD_MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS;
        case 142:
            return e_MAV_CMD_MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION;
        case 143:
            return e_MAV_CMD_MAV_CMD_GIMBAL_FULL_RESET;
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
            return e_MAV_CMD_MAV_CMD_NAV_ALTITUDE_WAIT;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 23:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 24:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 25:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 28:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 29:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 30:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 56:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_GRIPPER;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_AUTOTUNE_ENABLE;
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
        case 130:
            return e_MAV_CMD_MAV_CMD_POWER_OFF_INITIATED;
        case 131:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_CLICK;
        case 132:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_HOLD;
        case 133:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_PAUSE_CLICK;
        case 134:
            return e_MAV_CMD_MAV_CMD_DO_START_MAG_CAL;
        case 135:
            return e_MAV_CMD_MAV_CMD_DO_ACCEPT_MAG_CAL;
        case 136:
            return e_MAV_CMD_MAV_CMD_DO_CANCEL_MAG_CAL;
        case 137:
            return e_MAV_CMD_MAV_CMD_SET_FACTORY_TEST_MODE;
        case 138:
            return e_MAV_CMD_MAV_CMD_DO_SEND_BANNER;
        case 139:
            return e_MAV_CMD_MAV_CMD_ACCELCAL_VEHICLE_POS;
        case 140:
            return e_MAV_CMD_MAV_CMD_GIMBAL_RESET;
        case 141:
            return e_MAV_CMD_MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS;
        case 142:
            return e_MAV_CMD_MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION;
        case 143:
            return e_MAV_CMD_MAV_CMD_GIMBAL_FULL_RESET;
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
            return e_MAV_CMD_MAV_CMD_NAV_ALTITUDE_WAIT;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 23:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 24:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 25:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 28:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 29:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 30:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 56:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_GRIPPER;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_AUTOTUNE_ENABLE;
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
        case 130:
            return e_MAV_CMD_MAV_CMD_POWER_OFF_INITIATED;
        case 131:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_CLICK;
        case 132:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_HOLD;
        case 133:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_PAUSE_CLICK;
        case 134:
            return e_MAV_CMD_MAV_CMD_DO_START_MAG_CAL;
        case 135:
            return e_MAV_CMD_MAV_CMD_DO_ACCEPT_MAG_CAL;
        case 136:
            return e_MAV_CMD_MAV_CMD_DO_CANCEL_MAG_CAL;
        case 137:
            return e_MAV_CMD_MAV_CMD_SET_FACTORY_TEST_MODE;
        case 138:
            return e_MAV_CMD_MAV_CMD_DO_SEND_BANNER;
        case 139:
            return e_MAV_CMD_MAV_CMD_ACCELCAL_VEHICLE_POS;
        case 140:
            return e_MAV_CMD_MAV_CMD_GIMBAL_RESET;
        case 141:
            return e_MAV_CMD_MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS;
        case 142:
            return e_MAV_CMD_MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION;
        case 143:
            return e_MAV_CMD_MAV_CMD_GIMBAL_FULL_RESET;
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
            return e_MAV_CMD_MAV_CMD_NAV_ALTITUDE_WAIT;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 23:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 24:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 25:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 28:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 29:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 30:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 56:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_GRIPPER;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_AUTOTUNE_ENABLE;
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
        case 130:
            return e_MAV_CMD_MAV_CMD_POWER_OFF_INITIATED;
        case 131:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_CLICK;
        case 132:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_HOLD;
        case 133:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_PAUSE_CLICK;
        case 134:
            return e_MAV_CMD_MAV_CMD_DO_START_MAG_CAL;
        case 135:
            return e_MAV_CMD_MAV_CMD_DO_ACCEPT_MAG_CAL;
        case 136:
            return e_MAV_CMD_MAV_CMD_DO_CANCEL_MAG_CAL;
        case 137:
            return e_MAV_CMD_MAV_CMD_SET_FACTORY_TEST_MODE;
        case 138:
            return e_MAV_CMD_MAV_CMD_DO_SEND_BANNER;
        case 139:
            return e_MAV_CMD_MAV_CMD_ACCELCAL_VEHICLE_POS;
        case 140:
            return e_MAV_CMD_MAV_CMD_GIMBAL_RESET;
        case 141:
            return e_MAV_CMD_MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS;
        case 142:
            return e_MAV_CMD_MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION;
        case 143:
            return e_MAV_CMD_MAV_CMD_GIMBAL_FULL_RESET;
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
/**
*Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
*	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
*	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
*	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
*	bit 11: yaw, bit 12: yaw rat*/
INLINER uint16_t p84_type_mask_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint32_t p84_time_boot_ms_GET(Pack * src)//Timestamp in milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER uint8_t p84_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 1)));
}
INLINER uint8_t p84_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  7, 1)));
}
INLINER float p84_x_GET(Pack * src)//X Position in NED frame in meters
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p84_y_GET(Pack * src)//Y Position in NED frame in meters
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p84_z_GET(Pack * src)//Z Position in NED frame in meters (note, altitude is negative in NED)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p84_vx_GET(Pack * src)//X velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER float p84_vy_GET(Pack * src)//Y velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER float p84_vz_GET(Pack * src)//Z velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER float p84_afx_GET(Pack * src)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER float p84_afy_GET(Pack * src)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER float p84_afz_GET(Pack * src)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
INLINER float p84_yaw_GET(Pack * src)//yaw setpoint in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  44, 4)));
}
INLINER float p84_yaw_rate_GET(Pack * src)//yaw rate setpoint in rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  48, 4)));
}
/**
*Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
*	=*/
INLINER e_MAV_FRAME p84_coordinate_frame_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 416, 4);
}
/**
*Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
*	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
*	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
*	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
*	bit 11: yaw, bit 12: yaw rat*/
INLINER uint16_t p86_type_mask_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
/**
*Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
*	the system to compensate for the transport delay of the setpoint. This allows the system to compensate
*	processing latency*/
INLINER uint32_t p86_time_boot_ms_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER uint8_t p86_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 1)));
}
INLINER uint8_t p86_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  7, 1)));
}
INLINER int32_t p86_lat_int_GET(Pack * src)//X Position in WGS84 frame in 1e7 * meters
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  8, 4)));
}
INLINER int32_t p86_lon_int_GET(Pack * src)//Y Position in WGS84 frame in 1e7 * meters
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  12, 4)));
}
INLINER float p86_alt_GET(Pack * src)//Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p86_vx_GET(Pack * src)//X velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER float p86_vy_GET(Pack * src)//Y velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER float p86_vz_GET(Pack * src)//Z velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER float p86_afx_GET(Pack * src)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER float p86_afy_GET(Pack * src)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER float p86_afz_GET(Pack * src)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
INLINER float p86_yaw_GET(Pack * src)//yaw setpoint in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  44, 4)));
}
INLINER float p86_yaw_rate_GET(Pack * src)//yaw rate setpoint in rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  48, 4)));
}
/**
*Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
*	= 1*/
INLINER e_MAV_FRAME p86_coordinate_frame_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 416, 4);
}
/**
*Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
*	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
*	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
*	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
*	bit 11: yaw, bit 12: yaw rat*/
INLINER uint16_t p87_type_mask_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
/**
*Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
*	the system to compensate for the transport delay of the setpoint. This allows the system to compensate
*	processing latency*/
INLINER uint32_t p87_time_boot_ms_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER int32_t p87_lat_int_GET(Pack * src)//X Position in WGS84 frame in 1e7 * meters
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  6, 4)));
}
INLINER int32_t p87_lon_int_GET(Pack * src)//Y Position in WGS84 frame in 1e7 * meters
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  10, 4)));
}
INLINER float p87_alt_GET(Pack * src)//Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER float p87_vx_GET(Pack * src)//X velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER float p87_vy_GET(Pack * src)//Y velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
INLINER float p87_vz_GET(Pack * src)//Z velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  26, 4)));
}
INLINER float p87_afx_GET(Pack * src)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  30, 4)));
}
INLINER float p87_afy_GET(Pack * src)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  34, 4)));
}
INLINER float p87_afz_GET(Pack * src)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  38, 4)));
}
INLINER float p87_yaw_GET(Pack * src)//yaw setpoint in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  42, 4)));
}
INLINER float p87_yaw_rate_GET(Pack * src)//yaw rate setpoint in rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  46, 4)));
}
/**
*Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
*	= 1*/
INLINER e_MAV_FRAME p87_coordinate_frame_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 400, 4);
}
INLINER uint32_t p89_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER float p89_x_GET(Pack * src)//X Position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER float p89_y_GET(Pack * src)//Y Position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p89_z_GET(Pack * src)//Z Position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p89_roll_GET(Pack * src)//Roll
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p89_pitch_GET(Pack * src)//Pitch
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER float p89_yaw_GET(Pack * src)//Yaw
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER uint64_t p90_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER float p90_roll_GET(Pack * src)//Roll angle (rad)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p90_pitch_GET(Pack * src)//Pitch angle (rad)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p90_yaw_GET(Pack * src)//Yaw angle (rad)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p90_rollspeed_GET(Pack * src)//Body frame roll / phi angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER float p90_pitchspeed_GET(Pack * src)//Body frame pitch / theta angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER float p90_yawspeed_GET(Pack * src)//Body frame yaw / psi angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER int32_t p90_lat_GET(Pack * src)//Latitude, expressed as * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  32, 4)));
}
INLINER int32_t p90_lon_GET(Pack * src)//Longitude, expressed as * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  36, 4)));
}
INLINER int32_t p90_alt_GET(Pack * src)//Altitude in meters, expressed as * 1000 (millimeters)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  40, 4)));
}
INLINER int16_t p90_vx_GET(Pack * src)//Ground X Speed (Latitude), expressed as m/s * 100
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  44, 2)));
}
INLINER int16_t p90_vy_GET(Pack * src)//Ground Y Speed (Longitude), expressed as m/s * 100
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  46, 2)));
}
INLINER int16_t p90_vz_GET(Pack * src)//Ground Z Speed (Altitude), expressed as m/s * 100
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  48, 2)));
}
INLINER int16_t p90_xacc_GET(Pack * src)//X acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  50, 2)));
}
INLINER int16_t p90_yacc_GET(Pack * src)//Y acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  52, 2)));
}
INLINER int16_t p90_zacc_GET(Pack * src)//Z acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  54, 2)));
}
INLINER uint64_t p91_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER float p91_roll_ailerons_GET(Pack * src)//Control output -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p91_pitch_elevator_GET(Pack * src)//Control output -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p91_yaw_rudder_GET(Pack * src)//Control output -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p91_throttle_GET(Pack * src)//Throttle 0 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER float p91_aux1_GET(Pack * src)//Aux 1, -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER float p91_aux2_GET(Pack * src)//Aux 2, -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER float p91_aux3_GET(Pack * src)//Aux 3, -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER float p91_aux4_GET(Pack * src)//Aux 4, -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER uint8_t p91_nav_mode_GET(Pack * src)//Navigation mode (MAV_NAV_MODE)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  40, 1)));
}
INLINER e_MAV_MODE p91_mode_GET(Pack * src)//System mode (MAV_MODE)
{
    uint8_t * data = src->data;
    switch(get_bits(data, 328, 4))
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
INLINER uint16_t p92_chan1_raw_GET(Pack * src)//RC channel 1 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint16_t p92_chan2_raw_GET(Pack * src)//RC channel 2 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER uint16_t p92_chan3_raw_GET(Pack * src)//RC channel 3 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER uint16_t p92_chan4_raw_GET(Pack * src)//RC channel 4 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER uint16_t p92_chan5_raw_GET(Pack * src)//RC channel 5 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 2)));
}
INLINER uint16_t p92_chan6_raw_GET(Pack * src)//RC channel 6 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 2)));
}
INLINER uint16_t p92_chan7_raw_GET(Pack * src)//RC channel 7 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 2)));
}
INLINER uint16_t p92_chan8_raw_GET(Pack * src)//RC channel 8 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 2)));
}
INLINER uint16_t p92_chan9_raw_GET(Pack * src)//RC channel 9 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 2)));
}
INLINER uint16_t p92_chan10_raw_GET(Pack * src)//RC channel 10 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  18, 2)));
}
INLINER uint16_t p92_chan11_raw_GET(Pack * src)//RC channel 11 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  20, 2)));
}
INLINER uint16_t p92_chan12_raw_GET(Pack * src)//RC channel 12 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  22, 2)));
}
INLINER uint64_t p92_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  24, 8)));
}
INLINER uint8_t p92_rssi_GET(Pack * src)//Receive signal strength indicator, 0: 0%, 255: 100%
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  32, 1)));
}
INLINER uint64_t p93_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER uint64_t p93_flags_GET(Pack * src)//Flags as bitfield, reserved for future use.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 8)));
}
INLINER float* p93_controls_GET(Pack * src, float*  dst, int32_t pos) //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 16, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p93_controls_LEN = 16; //return array length

INLINER  float*  p93_controls_GET_(Pack * src) {return p93_controls_GET(src, malloc(16 * sizeof(float)), 0);}
INLINER e_MAV_MODE p93_mode_GET(Pack * src)//System mode (MAV_MODE), includes arming state.
{
    uint8_t * data = src->data;
    switch(get_bits(data, 640, 4))
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
INLINER uint64_t p100_time_usec_GET(Pack * src)//Timestamp (UNIX)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER uint8_t p100_sensor_id_GET(Pack * src)//Sensor ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER int16_t p100_flow_x_GET(Pack * src)//Flow in pixels * 10 in x-sensor direction (dezi-pixels)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  9, 2)));
}
INLINER int16_t p100_flow_y_GET(Pack * src)//Flow in pixels * 10 in y-sensor direction (dezi-pixels)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  11, 2)));
}
INLINER float p100_flow_comp_m_x_GET(Pack * src)//Flow in meters in x-sensor direction, angular-speed compensated
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  13, 4)));
}
INLINER float p100_flow_comp_m_y_GET(Pack * src)//Flow in meters in y-sensor direction, angular-speed compensated
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  17, 4)));
}
INLINER uint8_t p100_quality_GET(Pack * src)//Optical flow quality / confidence. 0: bad, 255: maximum quality
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  21, 1)));
}
INLINER float p100_ground_distance_GET(Pack * src)//Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
INLINER float  p100_flow_rate_x_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  208 && !try_visit_field(src, 208)) return 0;
    uint8_t * data = src->base.pack->data;
    return (intBitsToFloat(get_bytes(data,  src->BYTE, 4)));
}
INLINER float  p100_flow_rate_y_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  209 && !try_visit_field(src, 209)) return 0;
    uint8_t * data = src->base.pack->data;
    return (intBitsToFloat(get_bytes(data,  src->BYTE, 4)));
}
INLINER uint64_t p101_usec_GET(Pack * src)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER float p101_x_GET(Pack * src)//Global X position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p101_y_GET(Pack * src)//Global Y position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p101_z_GET(Pack * src)//Global Z position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p101_roll_GET(Pack * src)//Roll angle in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER float p101_pitch_GET(Pack * src)//Pitch angle in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER float p101_yaw_GET(Pack * src)//Yaw angle in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p150_mag_ofs_x_SET(int16_t  src, Pack * dst)//magnetometer X offset
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  0);
}
INLINER void p150_mag_ofs_y_SET(int16_t  src, Pack * dst)//magnetometer Y offset
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
INLINER void p150_mag_ofs_z_SET(int16_t  src, Pack * dst)//magnetometer Z offset
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  4);
}
INLINER void p150_mag_declination_SET(float  src, Pack * dst)//magnetic declination (radians)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER void p150_raw_press_SET(int32_t  src, Pack * dst)//raw pressure from barometer
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  10);
}
INLINER void p150_raw_temp_SET(int32_t  src, Pack * dst)//raw temperature from barometer
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  14);
}
INLINER void p150_gyro_cal_x_SET(float  src, Pack * dst)//gyro X calibration
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER void p150_gyro_cal_y_SET(float  src, Pack * dst)//gyro Y calibration
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER void p150_gyro_cal_z_SET(float  src, Pack * dst)//gyro Z calibration
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  26);
}
INLINER void p150_accel_cal_x_SET(float  src, Pack * dst)//accel X calibration
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  30);
}
INLINER void p150_accel_cal_y_SET(float  src, Pack * dst)//accel Y calibration
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  34);
}
INLINER void p150_accel_cal_z_SET(float  src, Pack * dst)//accel Z calibration
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  38);
}
Pack * c_TEST_Channel_new_SENSOR_OFFSETS_150()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 150));
};
INLINER void p151_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p151_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p151_mag_ofs_x_SET(int16_t  src, Pack * dst)//magnetometer X offset
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
INLINER void p151_mag_ofs_y_SET(int16_t  src, Pack * dst)//magnetometer Y offset
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  4);
}
INLINER void p151_mag_ofs_z_SET(int16_t  src, Pack * dst)//magnetometer Z offset
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  6);
}
Pack * c_TEST_Channel_new_SET_MAG_OFFSETS_151()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 151));
};
INLINER void p152_brkval_SET(uint16_t  src, Pack * dst)//heap top
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p152_freemem_SET(uint16_t  src, Pack * dst)//free memory
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p152_freemem32_SET(uint32_t  src, Bounds_Inside * dst)//free memory (32 bit)
{
    if(dst->base.field_bit != 32)insert_field(dst, 32, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 4, data,  dst->BYTE);
}
Pack * c_TEST_Channel_new_MEMINFO_152()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 152));
};
INLINER void p153_adc1_SET(uint16_t  src, Pack * dst)//ADC output 1
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p153_adc2_SET(uint16_t  src, Pack * dst)//ADC output 2
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p153_adc3_SET(uint16_t  src, Pack * dst)//ADC output 3
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER void p153_adc4_SET(uint16_t  src, Pack * dst)//ADC output 4
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER void p153_adc5_SET(uint16_t  src, Pack * dst)//ADC output 5
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  8);
}
INLINER void p153_adc6_SET(uint16_t  src, Pack * dst)//ADC output 6
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  10);
}
Pack * c_TEST_Channel_new_AP_ADC_153()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 153));
};
INLINER void p154_shutter_speed_SET(uint16_t  src, Pack * dst)//Divisor number e.g. 1000 means 1/1000 (0 means ignore)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p154_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p154_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p154_mode_SET(uint8_t  src, Pack * dst)//Mode enumeration from 1 to N P, TV, AV, M, Etc (0 means ignore)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p154_aperture_SET(uint8_t  src, Pack * dst)//F stop number x 10 e.g. 28 means 2.8 (0 means ignore)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p154_iso_SET(uint8_t  src, Pack * dst)//ISO enumeration from 1 to N e.g. 80, 100, 200, Etc (0 means ignore)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p154_exposure_type_SET(uint8_t  src, Pack * dst)//Exposure type enumeration from 1 to N (0 means ignore)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
/**
*Command Identity (incremental loop: 0 to 255)A command sent multiple times will be executed or pooled
*	just onc*/
INLINER void p154_command_id_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER void p154_engine_cut_off_SET(uint8_t  src, Pack * dst)//Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  9);
}
INLINER void p154_extra_param_SET(uint8_t  src, Pack * dst)//Extra parameters enumeration (0 means ignore)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER void p154_extra_value_SET(float  src, Pack * dst)//Correspondent value to given extra_param
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  11);
}
Pack * c_TEST_Channel_new_DIGICAM_CONFIGURE_154()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 154));
};
INLINER void p155_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p155_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p155_session_SET(uint8_t  src, Pack * dst)//0: stop, 1: start or keep it up Session control e.g. show/hide lens
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p155_zoom_pos_SET(uint8_t  src, Pack * dst)//1 to N Zoom's absolute position (0 means ignore)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p155_zoom_step_SET(int8_t  src, Pack * dst)//-100 to 100 Zooming step value to offset zoom from the current position
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  4);
}
INLINER void p155_focus_lock_SET(uint8_t  src, Pack * dst)//0: unlock focus or keep unlocked, 1: lock focus or keep locked, 3: re-lock focus
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p155_shot_SET(uint8_t  src, Pack * dst)//0: ignore, 1: shot or start filming
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
/**
*Command Identity (incremental loop: 0 to 255)A command sent multiple times will be executed or pooled
*	just onc*/
INLINER void p155_command_id_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
INLINER void p155_extra_param_SET(uint8_t  src, Pack * dst)//Extra parameters enumeration (0 means ignore)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER void p155_extra_value_SET(float  src, Pack * dst)//Correspondent value to given extra_param
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  9);
}
Pack * c_TEST_Channel_new_DIGICAM_CONTROL_155()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 155));
};
INLINER void p156_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p156_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p156_stab_roll_SET(uint8_t  src, Pack * dst)//(1 = yes, 0 = no)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p156_stab_pitch_SET(uint8_t  src, Pack * dst)//(1 = yes, 0 = no)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p156_stab_yaw_SET(uint8_t  src, Pack * dst)//(1 = yes, 0 = no)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p156_mount_mode_SET(e_MAV_MOUNT_MODE  src, Pack * dst)//mount operating mode (see MAV_MOUNT_MODE enum)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 40);
}
Pack * c_TEST_Channel_new_MOUNT_CONFIGURE_156()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 156));
};
INLINER void p157_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p157_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p157_input_a_SET(int32_t  src, Pack * dst)//pitch(deg*100) or lat, depending on mount mode
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  2);
}
INLINER void p157_input_b_SET(int32_t  src, Pack * dst)//roll(deg*100) or lon depending on mount mode
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  6);
}
INLINER void p157_input_c_SET(int32_t  src, Pack * dst)//yaw(deg*100) or alt (in cm) depending on mount mode
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  10);
}
INLINER void p157_save_position_SET(uint8_t  src, Pack * dst)//if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  14);
}
Pack * c_TEST_Channel_new_MOUNT_CONTROL_157()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 157));
};
INLINER void p158_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p158_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p158_pointing_a_SET(int32_t  src, Pack * dst)//pitch(deg*100)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  2);
}
INLINER void p158_pointing_b_SET(int32_t  src, Pack * dst)//roll(deg*100)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  6);
}
INLINER void p158_pointing_c_SET(int32_t  src, Pack * dst)//yaw(deg*100)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  10);
}
Pack * c_TEST_Channel_new_MOUNT_STATUS_158()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 158));
};
INLINER void p160_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p160_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p160_idx_SET(uint8_t  src, Pack * dst)//point index (first point is 1, 0 is for return point)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p160_count_SET(uint8_t  src, Pack * dst)//total number of points (for sanity checking)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p160_lat_SET(float  src, Pack * dst)//Latitude of point
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p160_lng_SET(float  src, Pack * dst)//Longitude of point
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
Pack * c_TEST_Channel_new_FENCE_POINT_160()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 160));
};
INLINER void p161_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p161_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p161_idx_SET(uint8_t  src, Pack * dst)//point index (first point is 1, 0 is for return point)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
Pack * c_TEST_Channel_new_FENCE_FETCH_POINT_161()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 161));
};
INLINER void p162_breach_count_SET(uint16_t  src, Pack * dst)//number of fence breaches
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p162_breach_time_SET(uint32_t  src, Pack * dst)//time of last breach in milliseconds since boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER void p162_breach_status_SET(uint8_t  src, Pack * dst)//0 if currently inside fence, 1 if outside
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p162_breach_type_SET(e_FENCE_BREACH  src, Pack * dst)//last breach type (see FENCE_BREACH_* enum)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 56);
}
Pack * c_TEST_Channel_new_FENCE_STATUS_162()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 162));
};
INLINER void p163_omegaIx_SET(float  src, Pack * dst)//X gyro drift estimate rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p163_omegaIy_SET(float  src, Pack * dst)//Y gyro drift estimate rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p163_omegaIz_SET(float  src, Pack * dst)//Z gyro drift estimate rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p163_accel_weight_SET(float  src, Pack * dst)//average accel_weight
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p163_renorm_val_SET(float  src, Pack * dst)//average renormalisation value
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p163_error_rp_SET(float  src, Pack * dst)//average error_roll_pitch value
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p163_error_yaw_SET(float  src, Pack * dst)//average error_yaw value
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
Pack * c_TEST_Channel_new_AHRS_163()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 163));
};
INLINER void p164_roll_SET(float  src, Pack * dst)//Roll angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p164_pitch_SET(float  src, Pack * dst)//Pitch angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p164_yaw_SET(float  src, Pack * dst)//Yaw angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p164_xacc_SET(float  src, Pack * dst)//X acceleration m/s/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p164_yacc_SET(float  src, Pack * dst)//Y acceleration m/s/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p164_zacc_SET(float  src, Pack * dst)//Z acceleration m/s/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p164_xgyro_SET(float  src, Pack * dst)//Angular speed around X axis rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p164_ygyro_SET(float  src, Pack * dst)//Angular speed around Y axis rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p164_zgyro_SET(float  src, Pack * dst)//Angular speed around Z axis rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p164_lat_SET(int32_t  src, Pack * dst)//Latitude in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  36);
}
INLINER void p164_lng_SET(int32_t  src, Pack * dst)//Longitude in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  40);
}
Pack * c_TEST_Channel_new_SIMSTATE_164()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 164));
};
INLINER void p165_Vcc_SET(uint16_t  src, Pack * dst)//board voltage (mV)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p165_I2Cerr_SET(uint8_t  src, Pack * dst)//I2C error count
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
Pack * c_TEST_Channel_new_HWSTATUS_165()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 165));
};
INLINER void p166_rxerrors_SET(uint16_t  src, Pack * dst)//receive errors
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p166_fixed__SET(uint16_t  src, Pack * dst)//count of error corrected packets
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p166_rssi_SET(uint8_t  src, Pack * dst)//local signal strength
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p166_remrssi_SET(uint8_t  src, Pack * dst)//remote signal strength
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p166_txbuf_SET(uint8_t  src, Pack * dst)//how full the tx buffer is as a percentage
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p166_noise_SET(uint8_t  src, Pack * dst)//background noise level
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
INLINER void p166_remnoise_SET(uint8_t  src, Pack * dst)//remote background noise level
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
Pack * c_TEST_Channel_new_RADIO_166()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 166));
};
INLINER void p167_breach_count_SET(uint16_t  src, Pack * dst)//number of fence breaches
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p167_last_trigger_SET(uint32_t  src, Pack * dst)//time of last breach in milliseconds since boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER void p167_last_action_SET(uint32_t  src, Pack * dst)//time of last recovery action in milliseconds since boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER void p167_last_recovery_SET(uint32_t  src, Pack * dst)//time of last successful recovery in milliseconds since boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  10);
}
INLINER void p167_last_clear_SET(uint32_t  src, Pack * dst)//time of last all-clear in milliseconds since boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  14);
}
INLINER void p167_limits_state_SET(e_LIMITS_STATE  src, Pack * dst)//state of AP_Limits, (see enum LimitState, LIMITS_STATE)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 144);
}
INLINER void p167_mods_enabled_SET(e_LIMIT_MODULE  src, Pack * dst)//AP_Limit_Module bitfield of enabled modules, (see enum moduleid or LIMIT_MODULE)
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_LIMIT_MODULE_LIMIT_GPSLOCK:
            id = 0;
            break;
        case e_LIMIT_MODULE_LIMIT_GEOFENCE:
            id = 1;
            break;
        case e_LIMIT_MODULE_LIMIT_ALTITUDE:
            id = 2;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 2, data, 147);
}
INLINER void p167_mods_required_SET(e_LIMIT_MODULE  src, Pack * dst)//AP_Limit_Module bitfield of required modules, (see enum moduleid or LIMIT_MODULE)
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_LIMIT_MODULE_LIMIT_GPSLOCK:
            id = 0;
            break;
        case e_LIMIT_MODULE_LIMIT_GEOFENCE:
            id = 1;
            break;
        case e_LIMIT_MODULE_LIMIT_ALTITUDE:
            id = 2;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 2, data, 149);
}
INLINER void p167_mods_triggered_SET(e_LIMIT_MODULE  src, Pack * dst)//AP_Limit_Module bitfield of triggered modules, (see enum moduleid or LIMIT_MODULE)
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_LIMIT_MODULE_LIMIT_GPSLOCK:
            id = 0;
            break;
        case e_LIMIT_MODULE_LIMIT_GEOFENCE:
            id = 1;
            break;
        case e_LIMIT_MODULE_LIMIT_ALTITUDE:
            id = 2;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 2, data, 151);
}
Pack * c_TEST_Channel_new_LIMITS_STATUS_167()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 167));
};
INLINER void p168_direction_SET(float  src, Pack * dst)//wind direction that wind is coming from (degrees)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p168_speed_SET(float  src, Pack * dst)//wind speed in ground plane (m/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p168_speed_z_SET(float  src, Pack * dst)//vertical wind speed (m/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
Pack * c_TEST_Channel_new_WIND_168()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 168));
};
INLINER void p169_type_SET(uint8_t  src, Pack * dst)//data type
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p169_len_SET(uint8_t  src, Pack * dst)//data length
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p169_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //raw data
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 16; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_DATA16_169()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 169));
};
INLINER void p170_type_SET(uint8_t  src, Pack * dst)//data type
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p170_len_SET(uint8_t  src, Pack * dst)//data length
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p170_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //raw data
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_DATA32_170()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 170));
};
INLINER void p171_type_SET(uint8_t  src, Pack * dst)//data type
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p171_len_SET(uint8_t  src, Pack * dst)//data length
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p171_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //raw data
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 64; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_DATA64_171()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 171));
};
INLINER void p172_type_SET(uint8_t  src, Pack * dst)//data type
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p172_len_SET(uint8_t  src, Pack * dst)//data length
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p172_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //raw data
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 96; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_DATA96_172()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 172));
};
INLINER void p173_distance_SET(float  src, Pack * dst)//distance in meters
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p173_voltage_SET(float  src, Pack * dst)//raw voltage if available, zero otherwise
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
Pack * c_TEST_Channel_new_RANGEFINDER_173()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 173));
};
INLINER void p174_vx_SET(float  src, Pack * dst)//GPS velocity north m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p174_vy_SET(float  src, Pack * dst)//GPS velocity east m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p174_vz_SET(float  src, Pack * dst)//GPS velocity down m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p174_diff_pressure_SET(float  src, Pack * dst)//Differential pressure pascals
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p174_EAS2TAS_SET(float  src, Pack * dst)//Estimated to true airspeed ratio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p174_ratio_SET(float  src, Pack * dst)//Airspeed ratio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p174_state_x_SET(float  src, Pack * dst)//EKF state x
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p174_state_y_SET(float  src, Pack * dst)//EKF state y
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p174_state_z_SET(float  src, Pack * dst)//EKF state z
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p174_Pax_SET(float  src, Pack * dst)//EKF Pax
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER void p174_Pby_SET(float  src, Pack * dst)//EKF Pby
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER void p174_Pcz_SET(float  src, Pack * dst)//EKF Pcz
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
Pack * c_TEST_Channel_new_AIRSPEED_AUTOCAL_174()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 174));
};
INLINER void p175_land_dir_SET(uint16_t  src, Pack * dst)//Heading to aim for when landing. In centi-degrees.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p175_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p175_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p175_idx_SET(uint8_t  src, Pack * dst)//point index (first point is 0)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p175_count_SET(uint8_t  src, Pack * dst)//total number of points (for sanity checking)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p175_lat_SET(int32_t  src, Pack * dst)//Latitude of point in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  6);
}
INLINER void p175_lng_SET(int32_t  src, Pack * dst)//Longitude of point in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  10);
}
INLINER void p175_alt_SET(int16_t  src, Pack * dst)//Transit / loiter altitude in meters relative to home
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  14);
}
INLINER void p175_break_alt_SET(int16_t  src, Pack * dst)//Break altitude in meters relative to home
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  16);
}
INLINER void p175_flags_SET(e_RALLY_FLAGS  src, Pack * dst)//See RALLY_FLAGS enum for definition of the bitmask.
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 2, data, 144);
}
Pack * c_TEST_Channel_new_RALLY_POINT_175()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 175));
};
INLINER void p176_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p176_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p176_idx_SET(uint8_t  src, Pack * dst)//point index (first point is 0)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
Pack * c_TEST_Channel_new_RALLY_FETCH_POINT_176()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 176));
};
INLINER void p177_throttle_SET(uint16_t  src, Pack * dst)//throttle (percent*10)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p177_interference_SET(uint16_t  src, Pack * dst)//interference (percent)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p177_current_SET(float  src, Pack * dst)//current (Ampere)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p177_CompensationX_SET(float  src, Pack * dst)//Motor Compensation X
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p177_CompensationY_SET(float  src, Pack * dst)//Motor Compensation Y
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p177_CompensationZ_SET(float  src, Pack * dst)//Motor Compensation Z
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
Pack * c_TEST_Channel_new_COMPASSMOT_STATUS_177()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 177));
};
INLINER void p178_roll_SET(float  src, Pack * dst)//Roll angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p178_pitch_SET(float  src, Pack * dst)//Pitch angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p178_yaw_SET(float  src, Pack * dst)//Yaw angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p178_altitude_SET(float  src, Pack * dst)//Altitude (MSL)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p178_lat_SET(int32_t  src, Pack * dst)//Latitude in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  16);
}
INLINER void p178_lng_SET(int32_t  src, Pack * dst)//Longitude in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  20);
}
Pack * c_TEST_Channel_new_AHRS2_178()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 178));
};
INLINER void p179_img_idx_SET(uint16_t  src, Pack * dst)//Image index
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p179_time_usec_SET(uint64_t  src, Pack * dst)//Image timestamp (microseconds since UNIX epoch, according to camera clock)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  2);
}
INLINER void p179_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER void p179_cam_idx_SET(uint8_t  src, Pack * dst)//Camera ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  11);
}
INLINER void p179_p1_SET(float  src, Pack * dst)//Parameter 1 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p179_p2_SET(float  src, Pack * dst)//Parameter 2 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p179_p3_SET(float  src, Pack * dst)//Parameter 3 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p179_p4_SET(float  src, Pack * dst)//Parameter 4 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p179_event_id_SET(e_CAMERA_STATUS_TYPES  src, Pack * dst)//See CAMERA_STATUS_TYPES enum for definition of the bitmask
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 224);
}
Pack * c_TEST_Channel_new_CAMERA_STATUS_179()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 179));
};
INLINER void p180_img_idx_SET(uint16_t  src, Pack * dst)//Image index
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
/**
*Image timestamp (microseconds since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if
*	no CCB*/
INLINER void p180_time_usec_SET(uint64_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  2);
}
INLINER void p180_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER void p180_cam_idx_SET(uint8_t  src, Pack * dst)//Camera ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  11);
}
INLINER void p180_lat_SET(int32_t  src, Pack * dst)//Latitude in (deg * 1E7)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  12);
}
INLINER void p180_lng_SET(int32_t  src, Pack * dst)//Longitude in (deg * 1E7)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  16);
}
INLINER void p180_alt_msl_SET(float  src, Pack * dst)//Altitude Absolute (meters AMSL)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p180_alt_rel_SET(float  src, Pack * dst)//Altitude Relative (meters above HOME location)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p180_roll_SET(float  src, Pack * dst)//Camera Roll angle (earth frame, degrees, +-180)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p180_pitch_SET(float  src, Pack * dst)//Camera Pitch angle (earth frame, degrees, +-180)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p180_yaw_SET(float  src, Pack * dst)//Camera Yaw (earth frame, degrees, 0-360, true)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER void p180_foc_len_SET(float  src, Pack * dst)//Focal Length (mm)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER void p180_flags_SET(e_CAMERA_FEEDBACK_FLAGS  src, Pack * dst)//See CAMERA_FEEDBACK_FLAGS enum for definition of the bitmask
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 352);
}
Pack * c_TEST_Channel_new_CAMERA_FEEDBACK_180()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 180));
};
INLINER void p181_voltage_SET(uint16_t  src, Pack * dst)//voltage in millivolts
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p181_current_battery_SET(int16_t  src, Pack * dst)//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
Pack * c_TEST_Channel_new_BATTERY2_181()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 181));
};
INLINER void p182_roll_SET(float  src, Pack * dst)//Roll angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p182_pitch_SET(float  src, Pack * dst)//Pitch angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p182_yaw_SET(float  src, Pack * dst)//Yaw angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p182_altitude_SET(float  src, Pack * dst)//Altitude (MSL)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p182_lat_SET(int32_t  src, Pack * dst)//Latitude in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  16);
}
INLINER void p182_lng_SET(int32_t  src, Pack * dst)//Longitude in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  20);
}
INLINER void p182_v1_SET(float  src, Pack * dst)//test variable1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p182_v2_SET(float  src, Pack * dst)//test variable2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p182_v3_SET(float  src, Pack * dst)//test variable3
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p182_v4_SET(float  src, Pack * dst)//test variable4
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
Pack * c_TEST_Channel_new_AHRS3_182()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 182));
};
INLINER void p183_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p183_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
Pack * c_TEST_Channel_new_AUTOPILOT_VERSION_REQUEST_183()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 183));
};
INLINER void p184_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p184_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p184_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //log data block
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 200; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p184_seqno_SET(e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS  src, Pack * dst)//log data block sequence number
{
    uint8_t * data = dst->data;
    set_bits(- 2147483645 +   src, 2, data, 1616);
}
Pack * c_TEST_Channel_new_REMOTE_LOG_DATA_BLOCK_184()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 184));
};
INLINER void p185_seqno_SET(uint32_t  src, Pack * dst)//log data block sequence number
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p185_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p185_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p185_status_SET(e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES  src, Pack * dst)//log data block status
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 48);
}
Pack * c_TEST_Channel_new_REMOTE_LOG_BLOCK_STATUS_185()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 185));
};
INLINER void p186_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p186_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p186_instance_SET(uint8_t  src, Pack * dst)//Instance (LED instance to control or 255 for all LEDs)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p186_pattern_SET(uint8_t  src, Pack * dst)//Pattern (see LED_PATTERN_ENUM)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p186_custom_len_SET(uint8_t  src, Pack * dst)//Custom Byte Length
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p186_custom_bytes_SET(uint8_t*  src, int32_t pos, Pack * dst) //Custom Bytes
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  5, src_max = pos + 24; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_LED_CONTROL_186()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 186));
};
INLINER void p191_compass_id_SET(uint8_t  src, Pack * dst)//Compass being calibrated
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p191_cal_mask_SET(uint8_t  src, Pack * dst)//Bitmask of compasses being calibrated
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p191_attempt_SET(uint8_t  src, Pack * dst)//Attempt number
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p191_completion_pct_SET(uint8_t  src, Pack * dst)//Completion percentage
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p191_completion_mask_SET(uint8_t*  src, int32_t pos, Pack * dst) //Bitmask of sphere sections (see http:en.wikipedia.org/wiki/Geodesic_grid)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  4, src_max = pos + 10; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p191_direction_x_SET(float  src, Pack * dst)//Body frame direction vector for display
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER void p191_direction_y_SET(float  src, Pack * dst)//Body frame direction vector for display
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER void p191_direction_z_SET(float  src, Pack * dst)//Body frame direction vector for display
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER void p191_cal_status_SET(e_MAG_CAL_STATUS  src, Pack * dst)//Status (see MAG_CAL_STATUS enum)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 208);
}
Pack * c_TEST_Channel_new_MAG_CAL_PROGRESS_191()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 191));
};
INLINER void p192_compass_id_SET(uint8_t  src, Pack * dst)//Compass being calibrated
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p192_cal_mask_SET(uint8_t  src, Pack * dst)//Bitmask of compasses being calibrated
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p192_autosaved_SET(uint8_t  src, Pack * dst)//0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p192_fitness_SET(float  src, Pack * dst)//RMS milligauss residuals
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  3);
}
INLINER void p192_ofs_x_SET(float  src, Pack * dst)//X offset
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  7);
}
INLINER void p192_ofs_y_SET(float  src, Pack * dst)//Y offset
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  11);
}
INLINER void p192_ofs_z_SET(float  src, Pack * dst)//Z offset
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  15);
}
INLINER void p192_diag_x_SET(float  src, Pack * dst)//X diagonal (matrix 11)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  19);
}
INLINER void p192_diag_y_SET(float  src, Pack * dst)//Y diagonal (matrix 22)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  23);
}
INLINER void p192_diag_z_SET(float  src, Pack * dst)//Z diagonal (matrix 33)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  27);
}
INLINER void p192_offdiag_x_SET(float  src, Pack * dst)//X off-diagonal (matrix 12 and 21)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  31);
}
INLINER void p192_offdiag_y_SET(float  src, Pack * dst)//Y off-diagonal (matrix 13 and 31)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  35);
}
INLINER void p192_offdiag_z_SET(float  src, Pack * dst)//Z off-diagonal (matrix 32 and 23)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  39);
}
INLINER void p192_cal_status_SET(e_MAG_CAL_STATUS  src, Pack * dst)//Status (see MAG_CAL_STATUS enum)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 344);
}
Pack * c_TEST_Channel_new_MAG_CAL_REPORT_192()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 192));
};
INLINER void p193_velocity_variance_SET(float  src, Pack * dst)//Velocity variance
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p193_pos_horiz_variance_SET(float  src, Pack * dst)//Horizontal Position variance
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p193_pos_vert_variance_SET(float  src, Pack * dst)//Vertical Position variance
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p193_compass_variance_SET(float  src, Pack * dst)//Compass variance
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p193_terrain_alt_variance_SET(float  src, Pack * dst)//Terrain Altitude variance
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p193_flags_SET(e_EKF_STATUS_FLAGS  src, Pack * dst)//Flags
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_EKF_STATUS_FLAGS_EKF_ATTITUDE:
            id = 0;
            break;
        case e_EKF_STATUS_FLAGS_EKF_VELOCITY_HORIZ:
            id = 1;
            break;
        case e_EKF_STATUS_FLAGS_EKF_VELOCITY_VERT:
            id = 2;
            break;
        case e_EKF_STATUS_FLAGS_EKF_POS_HORIZ_REL:
            id = 3;
            break;
        case e_EKF_STATUS_FLAGS_EKF_POS_HORIZ_ABS:
            id = 4;
            break;
        case e_EKF_STATUS_FLAGS_EKF_POS_VERT_ABS:
            id = 5;
            break;
        case e_EKF_STATUS_FLAGS_EKF_POS_VERT_AGL:
            id = 6;
            break;
        case e_EKF_STATUS_FLAGS_EKF_CONST_POS_MODE:
            id = 7;
            break;
        case e_EKF_STATUS_FLAGS_EKF_PRED_POS_HORIZ_REL:
            id = 8;
            break;
        case e_EKF_STATUS_FLAGS_EKF_PRED_POS_HORIZ_ABS:
            id = 9;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 4, data, 160);
}
Pack * c_TEST_Channel_new_EKF_STATUS_REPORT_193()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 193));
};
INLINER void p194_desired_SET(float  src, Pack * dst)//desired rate (degrees/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p194_achieved_SET(float  src, Pack * dst)//achieved rate (degrees/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p194_FF_SET(float  src, Pack * dst)//FF component
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p194_P_SET(float  src, Pack * dst)//P component
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p194_I_SET(float  src, Pack * dst)//I component
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p194_D_SET(float  src, Pack * dst)//D component
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p194_axis_SET(e_PID_TUNING_AXIS  src, Pack * dst)//axis
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 3, data, 192);
}
Pack * c_TEST_Channel_new_PID_TUNING_194()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 194));
};
INLINER void p200_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p200_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p200_delta_time_SET(float  src, Pack * dst)//Time since last update (seconds)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  2);
}
INLINER void p200_delta_angle_x_SET(float  src, Pack * dst)//Delta angle X (radians)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER void p200_delta_angle_y_SET(float  src, Pack * dst)//Delta angle Y (radians)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER void p200_delta_angle_z_SET(float  src, Pack * dst)//Delta angle X (radians)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER void p200_delta_velocity_x_SET(float  src, Pack * dst)//Delta velocity X (m/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER void p200_delta_velocity_y_SET(float  src, Pack * dst)//Delta velocity Y (m/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER void p200_delta_velocity_z_SET(float  src, Pack * dst)//Delta velocity Z (m/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  26);
}
INLINER void p200_joint_roll_SET(float  src, Pack * dst)//Joint ROLL (radians)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  30);
}
INLINER void p200_joint_el_SET(float  src, Pack * dst)//Joint EL (radians)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  34);
}
INLINER void p200_joint_az_SET(float  src, Pack * dst)//Joint AZ (radians)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  38);
}
Pack * c_TEST_Channel_new_GIMBAL_REPORT_200()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 200));
};
INLINER void p201_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p201_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p201_demanded_rate_x_SET(float  src, Pack * dst)//Demanded angular rate X (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  2);
}
INLINER void p201_demanded_rate_y_SET(float  src, Pack * dst)//Demanded angular rate Y (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER void p201_demanded_rate_z_SET(float  src, Pack * dst)//Demanded angular rate Z (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
Pack * c_TEST_Channel_new_GIMBAL_CONTROL_201()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 201));
};
INLINER void p214_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p214_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p214_rl_torque_cmd_SET(int16_t  src, Pack * dst)//Roll Torque Command
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
INLINER void p214_el_torque_cmd_SET(int16_t  src, Pack * dst)//Elevation Torque Command
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  4);
}
INLINER void p214_az_torque_cmd_SET(int16_t  src, Pack * dst)//Azimuth Torque Command
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  6);
}
Pack * c_TEST_Channel_new_GIMBAL_TORQUE_CMD_REPORT_214()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 214));
};
INLINER void p215_status_SET(e_GOPRO_HEARTBEAT_STATUS  src, Pack * dst)//Status
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 0);
}
INLINER void p215_capture_mode_SET(e_GOPRO_CAPTURE_MODE  src, Pack * dst)//Current capture mode
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_VIDEO:
            id = 0;
            break;
        case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_PHOTO:
            id = 1;
            break;
        case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_BURST:
            id = 2;
            break;
        case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_TIME_LAPSE:
            id = 3;
            break;
        case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_MULTI_SHOT:
            id = 4;
            break;
        case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_PLAYBACK:
            id = 5;
            break;
        case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_SETUP:
            id = 6;
            break;
        case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_UNKNOWN:
            id = 7;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 4, data, 3);
}
INLINER void p215_flags_SET(e_GOPRO_HEARTBEAT_FLAGS  src, Pack * dst)//additional status bits
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 1, data, 7);
}
Pack * c_TEST_Channel_new_GOPRO_HEARTBEAT_215()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 215));
};
INLINER void p216_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p216_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p216_cmd_id_SET(e_GOPRO_COMMAND  src, Pack * dst)//Command ID
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 5, data, 16);
}
Pack * c_TEST_Channel_new_GOPRO_GET_REQUEST_216()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 216));
};
INLINER void p217_value_SET(uint8_t*  src, int32_t pos, Pack * dst) //Value
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  0, src_max = pos + 4; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p217_cmd_id_SET(e_GOPRO_COMMAND  src, Pack * dst)//Command ID
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 5, data, 32);
}
INLINER void p217_status_SET(e_GOPRO_REQUEST_STATUS  src, Pack * dst)//Status
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 37);
}
Pack * c_TEST_Channel_new_GOPRO_GET_RESPONSE_217()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 217));
};
INLINER void p218_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p218_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p218_value_SET(uint8_t*  src, int32_t pos, Pack * dst) //Value
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 4; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p218_cmd_id_SET(e_GOPRO_COMMAND  src, Pack * dst)//Command ID
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 5, data, 48);
}
Pack * c_TEST_Channel_new_GOPRO_SET_REQUEST_218()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 218));
};
INLINER void p219_cmd_id_SET(e_GOPRO_COMMAND  src, Pack * dst)//Command ID
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 5, data, 0);
}
INLINER void p219_status_SET(e_GOPRO_REQUEST_STATUS  src, Pack * dst)//Status
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 5);
}
Pack * c_TEST_Channel_new_GOPRO_SET_RESPONSE_219()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 219));
};
INLINER void p226_rpm1_SET(float  src, Pack * dst)//RPM Sensor1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p226_rpm2_SET(float  src, Pack * dst)//RPM Sensor2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
Pack * c_TEST_Channel_new_RPM_226()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 226));
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
INLINER void p10001_stallSpeed_SET(uint16_t  src, Pack * dst)//Aircraft stall speed in cm/s
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p10001_ICAO_SET(uint32_t  src, Pack * dst)//Vehicle address (24 bit)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER void p10001_emitterType_SET(e_ADSB_EMITTER_TYPE  src, Pack * dst)//Transmitting vehicle type. See ADSB_EMITTER_TYPE enum
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 5, data, 48);
}
INLINER void p10001_aircraftSize_SET(e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE  src, Pack * dst)//Aircraft length and width encoding (table 2-35 of DO-282B)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 5, data, 53);
}
INLINER void p10001_gpsOffsetLat_SET(e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT  src, Pack * dst)//GPS antenna lateral offset (table 2-36 of DO-282B)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 58);
}
/**
*GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and add
*	one] (table 2-37 DO-282B*/
INLINER void p10001_gpsOffsetLon_SET(e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 62);
}
INLINER void p10001_rfSelect_SET(e_UAVIONIX_ADSB_OUT_RF_SELECT  src, Pack * dst)//ADS-B transponder reciever and transmit enable flags
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 64);
}
INLINER void p10001_callsign_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
{
    if(dst->base.field_bit != 66 && insert_field(dst, 66, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p10001_callsign_SET_(char16_t*  src, Bounds_Inside * dst) {p10001_callsign_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_UAVIONIX_ADSB_OUT_CFG_10001()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 10001));
};
INLINER void p10002_accuracyVert_SET(uint16_t  src, Pack * dst)//Vertical accuracy in cm. If unknown set to UINT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p10002_accuracyVel_SET(uint16_t  src, Pack * dst)//Velocity accuracy in mm/s (m * 1E-3). If unknown set to UINT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p10002_squawk_SET(uint16_t  src, Pack * dst)//Mode A code (typically 1200 [0x04B0] for VFR)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER void p10002_utcTime_SET(uint32_t  src, Pack * dst)//UTC time in seconds since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MAX
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER void p10002_accuracyHor_SET(uint32_t  src, Pack * dst)//Horizontal accuracy in mm (m * 1E-3). If unknown set to UINT32_MAX
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  10);
}
INLINER void p10002_gpsLat_SET(int32_t  src, Pack * dst)//Latitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  14);
}
INLINER void p10002_gpsLon_SET(int32_t  src, Pack * dst)//Longitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  18);
}
INLINER void p10002_gpsAlt_SET(int32_t  src, Pack * dst)//Altitude in mm (m * 1E-3) UP +ve. WGS84 altitude. If unknown set to INT32_MAX
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  22);
}
INLINER void p10002_numSats_SET(uint8_t  src, Pack * dst)//Number of satellites visible. If unknown set to UINT8_MAX
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  26);
}
/**
*Barometric pressure altitude relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude
*	(m * 1E-3). (up +ve). If unknown set to INT32_MA*/
INLINER void p10002_baroAltMSL_SET(int32_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  27);
}
INLINER void p10002_velVert_SET(int16_t  src, Pack * dst)//GPS vertical speed in cm/s. If unknown set to INT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  31);
}
INLINER void p10002_velNS_SET(int16_t  src, Pack * dst)//North-South velocity over ground in cm/s North +ve. If unknown set to INT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  33);
}
INLINER void p10002_VelEW_SET(int16_t  src, Pack * dst)//East-West velocity over ground in cm/s East +ve. If unknown set to INT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  35);
}
INLINER void p10002_gpsFix_SET(e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX  src, Pack * dst)//0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 296);
}
INLINER void p10002_emergencyStatus_SET(e_UAVIONIX_ADSB_EMERGENCY_STATUS  src, Pack * dst)//Emergency status
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 299);
}
INLINER void p10002_state_SET(e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE  src, Pack * dst)//ADS-B transponder dynamic input state flags
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE:
            id = 0;
            break;
        case e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED:
            id = 1;
            break;
        case e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED:
            id = 2;
            break;
        case e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND:
            id = 3;
            break;
        case e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT:
            id = 4;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 303);
}
Pack * c_TEST_Channel_new_UAVIONIX_ADSB_OUT_DYNAMIC_10002()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 10002));
};
INLINER void p10003_rfHealth_SET(e_UAVIONIX_ADSB_RF_HEALTH  src, Pack * dst)//ADS-B transponder messages
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_INITIALIZING:
            id = 0;
            break;
        case e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_OK:
            id = 1;
            break;
        case e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_FAIL_TX:
            id = 2;
            break;
        case e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_FAIL_RX:
            id = 3;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 0);
}
Pack * c_TEST_Channel_new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_10003()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 10003));
};
INLINER void p11000_request_id_SET(uint32_t  src, Pack * dst)//request ID - copied to reply
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p11000_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p11000_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p11000_bus_SET(uint8_t  src, Pack * dst)//Bus number
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p11000_address_SET(uint8_t  src, Pack * dst)//Bus address
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
INLINER void p11000_regstart_SET(uint8_t  src, Pack * dst)//First register to read
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER void p11000_count_SET(uint8_t  src, Pack * dst)//count of registers to read
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  9);
}
INLINER void p11000_bustype_SET(e_DEVICE_OP_BUSTYPE  src, Pack * dst)//The bus type
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 80);
}
INLINER void p11000_busname_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Name of device on bus (for SPI)
{
    if(dst->base.field_bit != 82 && insert_field(dst, 82, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p11000_busname_SET_(char16_t*  src, Bounds_Inside * dst) {p11000_busname_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_DEVICE_OP_READ_11000()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 11000));
};
INLINER void p11001_request_id_SET(uint32_t  src, Pack * dst)//request ID - copied from request
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p11001_result_SET(uint8_t  src, Pack * dst)//0 for success, anything else is failure code
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p11001_regstart_SET(uint8_t  src, Pack * dst)//starting register
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p11001_count_SET(uint8_t  src, Pack * dst)//count of bytes read
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p11001_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //reply data
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  7, src_max = pos + 128; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_DEVICE_OP_READ_REPLY_11001()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 11001));
};
INLINER void p11002_request_id_SET(uint32_t  src, Pack * dst)//request ID - copied to reply
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p11002_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p11002_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p11002_bus_SET(uint8_t  src, Pack * dst)//Bus number
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p11002_address_SET(uint8_t  src, Pack * dst)//Bus address
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
INLINER void p11002_regstart_SET(uint8_t  src, Pack * dst)//First register to write
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER void p11002_count_SET(uint8_t  src, Pack * dst)//count of registers to write
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  9);
}
INLINER void p11002_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //write data
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  10, src_max = pos + 128; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p11002_bustype_SET(e_DEVICE_OP_BUSTYPE  src, Pack * dst)//The bus type
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 1104);
}
INLINER void p11002_busname_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Name of device on bus (for SPI)
{
    if(dst->base.field_bit != 1106 && insert_field(dst, 1106, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p11002_busname_SET_(char16_t*  src, Bounds_Inside * dst) {p11002_busname_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_DEVICE_OP_WRITE_11002()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 11002));
};
INLINER void p11003_request_id_SET(uint32_t  src, Pack * dst)//request ID - copied from request
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p11003_result_SET(uint8_t  src, Pack * dst)//0 for success, anything else is failure code
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
Pack * c_TEST_Channel_new_DEVICE_OP_WRITE_REPLY_11003()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 11003));
};
INLINER void p11010_desired_SET(float  src, Pack * dst)//desired rate (degrees/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p11010_achieved_SET(float  src, Pack * dst)//achieved rate (degrees/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p11010_error_SET(float  src, Pack * dst)//error between model and vehicle
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p11010_theta_SET(float  src, Pack * dst)//theta estimated state predictor
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p11010_omega_SET(float  src, Pack * dst)//omega estimated state predictor
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p11010_sigma_SET(float  src, Pack * dst)//sigma estimated state predictor
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p11010_theta_dot_SET(float  src, Pack * dst)//theta derivative
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p11010_omega_dot_SET(float  src, Pack * dst)//omega derivative
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p11010_sigma_dot_SET(float  src, Pack * dst)//sigma derivative
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p11010_f_SET(float  src, Pack * dst)//projection operator value
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER void p11010_f_dot_SET(float  src, Pack * dst)//projection operator derivative
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER void p11010_u_SET(float  src, Pack * dst)//u adaptive controlled output command
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
INLINER void p11010_axis_SET(e_PID_TUNING_AXIS  src, Pack * dst)//axis
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 3, data, 384);
}
Pack * c_TEST_Channel_new_ADAP_TUNING_11010()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 11010));
};
INLINER void p11011_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER void p11011_time_delta_usec_SET(uint64_t  src, Pack * dst)//Time in microseconds since the last reported camera frame
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
INLINER void p11011_angle_delta_SET(float*  src, int32_t pos, Pack * dst) //Defines a rotation vector in body frame that rotates the vehicle from the previous to the current orientatio
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  16, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
/**
*Change in position in meters from previous to current frame rotated into body frame (0=forward, 1=right,
*	2=down*/
INLINER void p11011_position_delta_SET(float*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  28, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER void p11011_confidence_SET(float  src, Pack * dst)//normalised confidence value from 0 to 100
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
Pack * c_TEST_Channel_new_VISION_POSITION_DELTA_11011()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 11011));
};


void c_TEST_Channel_on_HEARTBEAT_0(Bounds_Inside * ph, Pack * pack)
{
    assert(p0_custom_mode_GET(pack) == (uint32_t)3757286026L);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)145);
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_INVALID);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_FLAPPING_WING);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_UNINIT);
    assert(p0_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED);
};


void c_TEST_Channel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_onboard_control_sensors_present_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION);
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)52808);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)33636);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)53854);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)27164);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)48193);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)21859);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t) -99);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)48981);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t)25263);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)27525);
    assert(p1_onboard_control_sensors_health_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER);
};


void c_TEST_Channel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)2377802486913103117L);
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)2502737219L);
};


void c_TEST_Channel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_vx_GET(pack) == (float) -2.1997894E38F);
    assert(p3_yaw_rate_GET(pack) == (float) -2.7285621E38F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p3_z_GET(pack) == (float) -3.3883094E38F);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)1935353043L);
    assert(p3_x_GET(pack) == (float)6.052137E36F);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)6692);
    assert(p3_y_GET(pack) == (float) -1.5107369E38F);
    assert(p3_yaw_GET(pack) == (float)2.9714496E37F);
    assert(p3_afx_GET(pack) == (float)2.2964852E38F);
    assert(p3_afy_GET(pack) == (float)2.2289277E38F);
    assert(p3_afz_GET(pack) == (float) -1.961987E37F);
    assert(p3_vz_GET(pack) == (float)1.1547073E38F);
    assert(p3_vy_GET(pack) == (float)1.9042449E38F);
};


void c_TEST_Channel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_seq_GET(pack) == (uint32_t)2877900857L);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p4_time_usec_GET(pack) == (uint64_t)3862608586981643298L);
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)216);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p5_passkey_LEN(ph) == 18);
    {
        char16_t * exemplary = u"KxsyjjifnipQftrhyv";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)78);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)76);
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)32);
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)252);
};


void c_TEST_Channel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 12);
    {
        char16_t * exemplary = u"pfxqAehmmqsj";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_custom_mode_GET(pack) == (uint32_t)4047015740L);
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_STABILIZE_ARMED);
};


void c_TEST_Channel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_param_id_LEN(ph) == 7);
    {
        char16_t * exemplary = u"yqcyKvp";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)153);
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t)17189);
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)52);
};


void c_TEST_Channel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)51);
};


void c_TEST_Channel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)4296);
    assert(p22_param_id_LEN(ph) == 9);
    {
        char16_t * exemplary = u"dhlikctqn";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_value_GET(pack) == (float)2.18129E38F);
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)25810);
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT8);
};


void c_TEST_Channel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_param_value_GET(pack) == (float) -2.5471102E38F);
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)3);
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32);
    assert(p23_param_id_LEN(ph) == 16);
    {
        char16_t * exemplary = u"hvwxdlSbvtbbdowp";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)239);
};


void c_TEST_Channel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)23843);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)33785);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)3121924690L);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)145);
    assert(p24_time_usec_GET(pack) == (uint64_t)1564219986044564487L);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)504176859L);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)40953);
    assert(p24_alt_GET(pack) == (int32_t) -880291799);
    assert(p24_h_acc_TRY(ph) == (uint32_t)2008357889L);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)32566);
    assert(p24_v_acc_TRY(ph) == (uint32_t)1782828515L);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t)1236021779);
    assert(p24_lat_GET(pack) == (int32_t)301445565);
    assert(p24_lon_GET(pack) == (int32_t)100196939);
};


void c_TEST_Channel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)9, (uint8_t)235, (uint8_t)52, (uint8_t)171, (uint8_t)142, (uint8_t)107, (uint8_t)20, (uint8_t)89, (uint8_t)21, (uint8_t)107, (uint8_t)27, (uint8_t)255, (uint8_t)183, (uint8_t)205, (uint8_t)40, (uint8_t)239, (uint8_t)220, (uint8_t)68, (uint8_t)18, (uint8_t)15} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)146, (uint8_t)14, (uint8_t)99, (uint8_t)18, (uint8_t)101, (uint8_t)152, (uint8_t)211, (uint8_t)151, (uint8_t)25, (uint8_t)246, (uint8_t)38, (uint8_t)54, (uint8_t)2, (uint8_t)130, (uint8_t)164, (uint8_t)171, (uint8_t)15, (uint8_t)124, (uint8_t)234, (uint8_t)165} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)207);
    {
        uint8_t exemplary[] =  {(uint8_t)199, (uint8_t)221, (uint8_t)116, (uint8_t)164, (uint8_t)38, (uint8_t)151, (uint8_t)98, (uint8_t)158, (uint8_t)138, (uint8_t)22, (uint8_t)237, (uint8_t)235, (uint8_t)192, (uint8_t)15, (uint8_t)63, (uint8_t)228, (uint8_t)49, (uint8_t)231, (uint8_t)239, (uint8_t)183} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)243, (uint8_t)129, (uint8_t)232, (uint8_t)201, (uint8_t)198, (uint8_t)173, (uint8_t)191, (uint8_t)80, (uint8_t)241, (uint8_t)116, (uint8_t)137, (uint8_t)14, (uint8_t)28, (uint8_t)31, (uint8_t)252, (uint8_t)210, (uint8_t)166, (uint8_t)4, (uint8_t)193, (uint8_t)31} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)117, (uint8_t)106, (uint8_t)147, (uint8_t)110, (uint8_t)188, (uint8_t)182, (uint8_t)54, (uint8_t)83, (uint8_t)165, (uint8_t)66, (uint8_t)61, (uint8_t)130, (uint8_t)96, (uint8_t)146, (uint8_t)210, (uint8_t)164, (uint8_t)209, (uint8_t)24, (uint8_t)190, (uint8_t)163} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t)15990);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t)6569);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t) -30252);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t) -32278);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t)21168);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)3739400978L);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t)20951);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t) -20368);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t) -26616);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t) -26198);
};


void c_TEST_Channel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t)1102);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t) -26511);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t) -10064);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t) -5755);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t) -24390);
    assert(p27_time_usec_GET(pack) == (uint64_t)196834432168508760L);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t)29322);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t)526);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t) -9822);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t)19503);
};


void c_TEST_Channel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t)28994);
    assert(p28_time_usec_GET(pack) == (uint64_t)69720345991366460L);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t) -22194);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t)18642);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t)11930);
};


void c_TEST_Channel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_press_abs_GET(pack) == (float) -1.2052405E38F);
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t)24046);
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)171980328L);
    assert(p29_press_diff_GET(pack) == (float)2.363488E38F);
};


void c_TEST_Channel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_yaw_GET(pack) == (float) -6.6120163E37F);
    assert(p30_roll_GET(pack) == (float)1.9303931E38F);
    assert(p30_pitchspeed_GET(pack) == (float)9.323722E37F);
    assert(p30_rollspeed_GET(pack) == (float)9.865922E37F);
    assert(p30_pitch_GET(pack) == (float) -2.8197577E38F);
    assert(p30_yawspeed_GET(pack) == (float) -1.2593523E38F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)2433965492L);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_yawspeed_GET(pack) == (float)5.583059E36F);
    assert(p31_q1_GET(pack) == (float)8.602989E37F);
    assert(p31_q2_GET(pack) == (float)1.7537528E38F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)3633934419L);
    assert(p31_pitchspeed_GET(pack) == (float) -2.6873919E38F);
    assert(p31_rollspeed_GET(pack) == (float)2.779231E38F);
    assert(p31_q4_GET(pack) == (float)5.6644267E37F);
    assert(p31_q3_GET(pack) == (float) -2.3504644E38F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_x_GET(pack) == (float) -3.776003E37F);
    assert(p32_z_GET(pack) == (float)6.929247E37F);
    assert(p32_vy_GET(pack) == (float)1.4952403E38F);
    assert(p32_y_GET(pack) == (float) -3.2339548E38F);
    assert(p32_vx_GET(pack) == (float) -2.5328415E37F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)1731293342L);
    assert(p32_vz_GET(pack) == (float) -8.095612E36F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)64482);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)2011811309L);
    assert(p33_relative_alt_GET(pack) == (int32_t)1542460366);
    assert(p33_alt_GET(pack) == (int32_t) -1561981782);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t) -11770);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t)30793);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t)21333);
    assert(p33_lat_GET(pack) == (int32_t) -1752074534);
    assert(p33_lon_GET(pack) == (int32_t)278208853);
};


void c_TEST_Channel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t)23063);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t)15760);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t) -16612);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t) -12665);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t)13208);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t) -13629);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)1738099037L);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t) -29452);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t)3769);
};


void c_TEST_Channel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)1454964265L);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)19562);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)60087);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)53389);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)14583);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)55970);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)54027);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)57953);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)6278);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)150);
};


void c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)31107);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)39897);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)1639);
    assert(p36_time_usec_GET(pack) == (uint32_t)852117647L);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)41977);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)11269);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)30364);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)12194);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)3932);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)17116);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)65000);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)3369);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)13295);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)57672);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)44167);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)61537);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)16900);
};


void c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)182);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t) -22399);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t)30706);
};


void c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t) -7328);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t) -6475);
};


void c_TEST_Channel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_z_GET(pack) == (float)3.013889E37F);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p39_param4_GET(pack) == (float)1.9674838E38F);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS);
    assert(p39_param2_GET(pack) == (float)1.4797372E38F);
    assert(p39_param1_GET(pack) == (float) -3.0278273E38F);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)3908);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_NED);
    assert(p39_x_GET(pack) == (float) -2.188893E38F);
    assert(p39_y_GET(pack) == (float)1.6603599E38F);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)227);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)96);
    assert(p39_param3_GET(pack) == (float)2.0670962E38F);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)149);
};


void c_TEST_Channel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)177);
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)59947);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)171);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
};


void c_TEST_Channel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)51202);
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)124);
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)198);
};


void c_TEST_Channel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)32378);
};


void c_TEST_Channel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)234);
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
};


void c_TEST_Channel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)1699);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)67);
};


void c_TEST_Channel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)95);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)230);
};


void c_TEST_Channel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)42333);
};


void c_TEST_Channel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM7);
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
};


void c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_latitude_GET(pack) == (int32_t)376404914);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p48_altitude_GET(pack) == (int32_t) -1293298328);
    assert(p48_time_usec_TRY(ph) == (uint64_t)964399069680555709L);
    assert(p48_longitude_GET(pack) == (int32_t) -683305881);
};


void c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_altitude_GET(pack) == (int32_t) -365133171);
    assert(p49_time_usec_TRY(ph) == (uint64_t)4706696726007529983L);
    assert(p49_latitude_GET(pack) == (int32_t)2129936714);
    assert(p49_longitude_GET(pack) == (int32_t) -1110240306);
};


void c_TEST_Channel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"u";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_scale_GET(pack) == (float)1.4242772E38F);
    assert(p50_param_value_min_GET(pack) == (float)3.6540713E37F);
    assert(p50_param_value_max_GET(pack) == (float)3.1689229E38F);
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)12);
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)186);
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)32);
    assert(p50_param_value0_GET(pack) == (float)1.7454348E38F);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t)28601);
};


void c_TEST_Channel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)126);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)117);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)4262);
};


void c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_p1y_GET(pack) == (float)7.376964E37F);
    assert(p54_p2y_GET(pack) == (float)1.1746327E38F);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p54_p1x_GET(pack) == (float) -2.7770272E38F);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)117);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p54_p2z_GET(pack) == (float) -2.9913822E38F);
    assert(p54_p2x_GET(pack) == (float) -3.2599E38F);
    assert(p54_p1z_GET(pack) == (float) -2.8051295E38F);
};


void c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p1x_GET(pack) == (float) -5.0733933E37F);
    assert(p55_p2z_GET(pack) == (float)3.0944515E38F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_NED);
    assert(p55_p1z_GET(pack) == (float) -3.6524594E37F);
    assert(p55_p1y_GET(pack) == (float)2.4278415E38F);
    assert(p55_p2y_GET(pack) == (float)1.2518875E38F);
    assert(p55_p2x_GET(pack) == (float) -1.1205563E38F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-4.393846E37F, -3.2402794E38F, 1.836071E37F, -4.971769E37F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {3.2383684E38F, 1.0553909E38F, 2.8662773E38F, -2.5292966E38F, -2.940768E38F, 1.7403784E38F, -3.0656292E38F, -1.3328743E38F, 2.3754117E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_yawspeed_GET(pack) == (float)2.5848173E38F);
    assert(p61_pitchspeed_GET(pack) == (float) -2.3585017E38F);
    assert(p61_rollspeed_GET(pack) == (float)2.1007445E38F);
    assert(p61_time_usec_GET(pack) == (uint64_t)2497490024104014366L);
};


void c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_nav_roll_GET(pack) == (float) -4.3224954E37F);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)25265);
    assert(p62_xtrack_error_GET(pack) == (float) -2.0097927E38F);
    assert(p62_alt_error_GET(pack) == (float)7.3341117E37F);
    assert(p62_aspd_error_GET(pack) == (float) -7.8472633E37F);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t)14460);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t)15682);
    assert(p62_nav_pitch_GET(pack) == (float)9.375259E37F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_lon_GET(pack) == (int32_t)1527722216);
    assert(p63_relative_alt_GET(pack) == (int32_t)772888927);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION);
    assert(p63_vz_GET(pack) == (float) -1.6504563E38F);
    assert(p63_lat_GET(pack) == (int32_t) -59026835);
    assert(p63_vy_GET(pack) == (float)2.2309924E38F);
    {
        float exemplary[] =  {-1.9718074E37F, -1.5759383E37F, 5.8307045E37F, 3.187205E38F, 1.9234068E37F, 2.2460193E38F, 9.094833E37F, 2.4000462E37F, 1.8664839E38F, -1.1292885E38F, -1.8378374E38F, -2.3629374E38F, -2.2120156E38F, 1.9695895E38F, 2.128531E38F, 6.363106E37F, -3.314871E38F, -2.8594364E37F, 2.2070543E38F, -1.9736943E38F, 5.8883334E37F, -2.500916E38F, -2.0889021E37F, -1.8185847E38F, -8.951783E37F, -2.9005637E38F, -2.4219156E38F, 2.2724844E38F, -2.6371714E38F, 3.3944677E38F, 1.2036241E37F, 1.0733374E38F, -2.9657463E38F, -3.339916E38F, 2.9904689E38F, -1.9407804E37F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_time_usec_GET(pack) == (uint64_t)6947815258434180851L);
    assert(p63_alt_GET(pack) == (int32_t) -37097484);
    assert(p63_vx_GET(pack) == (float) -1.6408788E38F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-1.9759142E38F, 1.1199236E38F, -1.087245E38F, -4.7696916E37F, 2.183891E38F, -1.4713713E38F, 2.6253452E37F, 1.4869073E38F, 3.558574E37F, -1.6714743E38F, -1.0508726E38F, -1.9890296E38F, 1.1480092E38F, -2.1071093E38F, 1.3335088E38F, -2.0490436E38F, -2.520602E38F, -1.2865638E38F, -1.2270626E37F, -1.9845616E38F, 2.8274192E38F, 1.1513269E38F, 1.191909E38F, -1.8967986E38F, 2.2565779E38F, 2.7393025E38F, 1.4412951E38F, 1.3571741E38F, 2.7382097E38F, 9.755008E37F, -1.0798661E38F, -2.5767144E38F, 8.0016803E37F, 4.6501455E36F, 2.9940716E37F, 1.7724374E38F, 2.4500084E38F, 1.041267E38F, -9.776023E37F, -1.1676083E38F, 5.033777E36F, -6.758617E37F, 2.0507575E38F, 8.5023075E37F, -1.4028092E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_vy_GET(pack) == (float)5.9029894E37F);
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION);
    assert(p64_x_GET(pack) == (float) -2.1978534E38F);
    assert(p64_z_GET(pack) == (float)3.1576008E38F);
    assert(p64_y_GET(pack) == (float)1.8415288E38F);
    assert(p64_ax_GET(pack) == (float) -3.8415353E37F);
    assert(p64_ay_GET(pack) == (float)1.2095718E38F);
    assert(p64_vx_GET(pack) == (float)3.305407E38F);
    assert(p64_vz_GET(pack) == (float) -2.3921891E38F);
    assert(p64_time_usec_GET(pack) == (uint64_t)3925006818284411813L);
    assert(p64_az_GET(pack) == (float)2.8160087E38F);
};


void c_TEST_Channel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)12286);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)24634);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)6477);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)41707);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)50186);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)50667);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)44533);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)25165);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)65049);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)3664);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)47525);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)107);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)483053390L);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)31920);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)61141);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)35914);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)15600);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)5);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)36503);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)58481);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)4566);
};


void c_TEST_Channel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)172);
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)33572);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)208);
};


void c_TEST_Channel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)65);
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)120);
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)52747);
};


void c_TEST_Channel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)47695);
    assert(p69_r_GET(pack) == (int16_t)(int16_t)27400);
    assert(p69_x_GET(pack) == (int16_t)(int16_t)19913);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p69_z_GET(pack) == (int16_t)(int16_t) -16524);
    assert(p69_y_GET(pack) == (int16_t)(int16_t)15264);
};


void c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)103);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)31678);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)58126);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)32018);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)4651);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)53650);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)15322);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)53847);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)15589);
};


void c_TEST_Channel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_param4_GET(pack) == (float) -3.3684194E38F);
    assert(p73_z_GET(pack) == (float)8.034974E37F);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p73_y_GET(pack) == (int32_t) -2093445574);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_LAST);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)63767);
    assert(p73_param3_GET(pack) == (float)1.5379197E38F);
    assert(p73_x_GET(pack) == (int32_t) -1501390649);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p73_param1_GET(pack) == (float)3.308039E38F);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p73_param2_GET(pack) == (float)3.9620673E36F);
};


void c_TEST_Channel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_alt_GET(pack) == (float)2.7960746E38F);
    assert(p74_groundspeed_GET(pack) == (float) -1.8443324E38F);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)38152);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t)14858);
    assert(p74_airspeed_GET(pack) == (float)1.1883593E38F);
    assert(p74_climb_GET(pack) == (float)2.9922657E38F);
};


void c_TEST_Channel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_x_GET(pack) == (int32_t) -607636037);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)115);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p75_param2_GET(pack) == (float) -2.9092129E38F);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)255);
    assert(p75_param3_GET(pack) == (float)3.317899E38F);
    assert(p75_param4_GET(pack) == (float) -2.3101364E38F);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER);
    assert(p75_y_GET(pack) == (int32_t)1382602225);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p75_z_GET(pack) == (float) -9.390353E37F);
    assert(p75_param1_GET(pack) == (float)2.6057375E38F);
};


void c_TEST_Channel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_param2_GET(pack) == (float) -9.975706E37F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)48);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)248);
    assert(p76_param6_GET(pack) == (float)2.5492608E38F);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)146);
    assert(p76_param5_GET(pack) == (float) -2.4103725E38F);
    assert(p76_param3_GET(pack) == (float)5.740333E37F);
    assert(p76_param7_GET(pack) == (float) -2.2883292E38F);
    assert(p76_param1_GET(pack) == (float) -1.1421363E38F);
    assert(p76_param4_GET(pack) == (float)1.0890053E38F);
};


void c_TEST_Channel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)160);
    assert(p77_result_param2_TRY(ph) == (int32_t) -2083155699);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)139);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_RALLY_LAND);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)114);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_IN_PROGRESS);
};


void c_TEST_Channel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)12);
    assert(p81_roll_GET(pack) == (float) -2.9063184E38F);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p81_pitch_GET(pack) == (float)3.5380757E37F);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)1672259551L);
    assert(p81_yaw_GET(pack) == (float)6.984934E37F);
    assert(p81_thrust_GET(pack) == (float) -1.6486037E38F);
};


void c_TEST_Channel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)873824322L);
    assert(p82_body_pitch_rate_GET(pack) == (float)1.4932367E38F);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)223);
    assert(p82_thrust_GET(pack) == (float)1.6836219E38F);
    assert(p82_body_yaw_rate_GET(pack) == (float)2.1139009E38F);
    {
        float exemplary[] =  {5.3570407E37F, -2.8731066E38F, -9.298205E37F, -3.1754737E38F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p82_body_roll_rate_GET(pack) == (float) -2.0595278E38F);
};


void c_TEST_Channel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p83_body_yaw_rate_GET(pack) == (float)2.7193653E38F);
    {
        float exemplary[] =  {1.7237136E38F, -1.9448908E38F, 2.045152E38F, 1.7882465E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)975910876L);
    assert(p83_thrust_GET(pack) == (float) -2.695336E38F);
    assert(p83_body_roll_rate_GET(pack) == (float)1.2823207E37F);
    assert(p83_body_pitch_rate_GET(pack) == (float) -2.1078843E38F);
};


void c_TEST_Channel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_afy_GET(pack) == (float)2.930559E36F);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)9);
    assert(p84_afx_GET(pack) == (float) -2.801151E38F);
    assert(p84_z_GET(pack) == (float) -2.0751978E37F);
    assert(p84_vx_GET(pack) == (float)2.3492484E37F);
    assert(p84_y_GET(pack) == (float) -8.670094E37F);
    assert(p84_vz_GET(pack) == (float) -1.8543017E38F);
    assert(p84_x_GET(pack) == (float)2.67391E38F);
    assert(p84_vy_GET(pack) == (float) -7.931537E37F);
    assert(p84_afz_GET(pack) == (float) -1.896281E38F);
    assert(p84_yaw_rate_GET(pack) == (float) -8.980201E37F);
    assert(p84_yaw_GET(pack) == (float) -2.1626186E38F);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)2243434056L);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)6862);
};


void c_TEST_Channel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_afx_GET(pack) == (float)2.4619987E38F);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)14849);
    assert(p86_vy_GET(pack) == (float)2.024968E37F);
    assert(p86_vx_GET(pack) == (float)1.858552E38F);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p86_lon_int_GET(pack) == (int32_t) -304259324);
    assert(p86_alt_GET(pack) == (float) -1.8652473E38F);
    assert(p86_lat_int_GET(pack) == (int32_t) -339255875);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p86_yaw_rate_GET(pack) == (float)2.2443036E38F);
    assert(p86_vz_GET(pack) == (float) -7.24514E36F);
    assert(p86_afy_GET(pack) == (float)1.6936133E38F);
    assert(p86_yaw_GET(pack) == (float) -1.3182083E37F);
    assert(p86_afz_GET(pack) == (float)2.0159281E38F);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)2316502151L);
};


void c_TEST_Channel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p87_vx_GET(pack) == (float) -1.871773E38F);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)63725);
    assert(p87_afx_GET(pack) == (float)3.5334128E37F);
    assert(p87_yaw_rate_GET(pack) == (float)1.4572205E37F);
    assert(p87_alt_GET(pack) == (float) -5.4364347E37F);
    assert(p87_lon_int_GET(pack) == (int32_t) -1698974892);
    assert(p87_lat_int_GET(pack) == (int32_t)1765952953);
    assert(p87_vy_GET(pack) == (float)2.0189774E38F);
    assert(p87_afy_GET(pack) == (float) -7.846838E36F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)1336715628L);
    assert(p87_afz_GET(pack) == (float) -1.0998153E38F);
    assert(p87_vz_GET(pack) == (float) -2.453252E38F);
    assert(p87_yaw_GET(pack) == (float) -5.524502E37F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_yaw_GET(pack) == (float)1.819299E38F);
    assert(p89_y_GET(pack) == (float)1.3927233E38F);
    assert(p89_x_GET(pack) == (float)3.136394E38F);
    assert(p89_roll_GET(pack) == (float)3.2170344E38F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)3639711002L);
    assert(p89_pitch_GET(pack) == (float) -1.0178925E38F);
    assert(p89_z_GET(pack) == (float)6.4790266E37F);
};


void c_TEST_Channel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_time_usec_GET(pack) == (uint64_t)6081314777698915113L);
    assert(p90_pitchspeed_GET(pack) == (float) -2.8366406E38F);
    assert(p90_lon_GET(pack) == (int32_t) -1806385373);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t)8164);
    assert(p90_lat_GET(pack) == (int32_t)598096338);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t) -172);
    assert(p90_yaw_GET(pack) == (float)2.8375947E38F);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t) -3474);
    assert(p90_rollspeed_GET(pack) == (float)1.4637278E38F);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t) -24879);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t) -30405);
    assert(p90_yawspeed_GET(pack) == (float)3.0686334E38F);
    assert(p90_roll_GET(pack) == (float) -3.3031695E38F);
    assert(p90_alt_GET(pack) == (int32_t) -1137524312);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t)4447);
    assert(p90_pitch_GET(pack) == (float) -1.687662E38F);
};


void c_TEST_Channel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_aux1_GET(pack) == (float)1.9962027E38F);
    assert(p91_throttle_GET(pack) == (float)4.7955704E37F);
    assert(p91_yaw_rudder_GET(pack) == (float)1.7918291E37F);
    assert(p91_aux3_GET(pack) == (float)3.0990242E38F);
    assert(p91_time_usec_GET(pack) == (uint64_t)6623985788348441034L);
    assert(p91_aux4_GET(pack) == (float)2.9231834E38F);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p91_pitch_elevator_GET(pack) == (float)2.3375453E38F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_MANUAL_ARMED);
    assert(p91_roll_ailerons_GET(pack) == (float) -1.5366309E38F);
    assert(p91_aux2_GET(pack) == (float) -2.3436083E38F);
};


void c_TEST_Channel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)64953);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)53561);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)12423);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)5165);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)11487);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)12407);
    assert(p92_time_usec_GET(pack) == (uint64_t)1630581090942475903L);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)39940);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)47529);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)34736);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)46989);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)54892);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)32980);
};


void c_TEST_Channel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    assert(p93_time_usec_GET(pack) == (uint64_t)8454991304750796385L);
    {
        float exemplary[] =  {2.0212393E38F, 3.3694644E38F, 1.1538606E37F, 5.4415484E37F, -2.2634017E38F, -1.6503768E38F, -2.6334737E38F, -1.1312243E36F, -2.149682E38F, -2.7605327E38F, -2.8128057E38F, -1.8805834E38F, -2.0167901E38F, -1.848552E38F, 2.7865664E38F, 3.2410834E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_TEST_DISARMED);
    assert(p93_flags_GET(pack) == (uint64_t)8936923776884257108L);
};


void c_TEST_Channel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t)28798);
    assert(p100_flow_comp_m_x_GET(pack) == (float) -1.2048791E38F);
    assert(p100_flow_comp_m_y_GET(pack) == (float)3.0536877E38F);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)49);
    assert(p100_flow_rate_y_TRY(ph) == (float) -1.3732768E38F);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p100_ground_distance_GET(pack) == (float)4.7310657E37F);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t)26934);
    assert(p100_flow_rate_x_TRY(ph) == (float)1.5653577E38F);
    assert(p100_time_usec_GET(pack) == (uint64_t)7181754580670574436L);
};


void c_TEST_Channel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_y_GET(pack) == (float)9.8270353E36F);
    assert(p101_z_GET(pack) == (float) -2.053783E38F);
    assert(p101_roll_GET(pack) == (float)1.7366118E38F);
    assert(p101_usec_GET(pack) == (uint64_t)4986183000393551627L);
    assert(p101_yaw_GET(pack) == (float)1.8189685E37F);
    assert(p101_x_GET(pack) == (float)2.2984941E38F);
    assert(p101_pitch_GET(pack) == (float)2.7518646E38F);
};


void c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_x_GET(pack) == (float) -1.8202943E38F);
    assert(p102_yaw_GET(pack) == (float) -3.1450344E38F);
    assert(p102_z_GET(pack) == (float) -1.083662E38F);
    assert(p102_usec_GET(pack) == (uint64_t)6278435554115547332L);
    assert(p102_y_GET(pack) == (float)1.8415848E38F);
    assert(p102_roll_GET(pack) == (float) -3.1200252E38F);
    assert(p102_pitch_GET(pack) == (float) -1.2737819E38F);
};


void c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_y_GET(pack) == (float)2.8840262E38F);
    assert(p103_x_GET(pack) == (float) -2.6129924E38F);
    assert(p103_usec_GET(pack) == (uint64_t)6258964868240759567L);
    assert(p103_z_GET(pack) == (float)8.748628E37F);
};


void c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_yaw_GET(pack) == (float)8.0502765E37F);
    assert(p104_y_GET(pack) == (float)2.8636978E37F);
    assert(p104_z_GET(pack) == (float) -4.4800025E37F);
    assert(p104_pitch_GET(pack) == (float) -2.8671107E38F);
    assert(p104_roll_GET(pack) == (float) -1.0672434E38F);
    assert(p104_x_GET(pack) == (float) -9.539132E37F);
    assert(p104_usec_GET(pack) == (uint64_t)8975799484182240523L);
};


void c_CommunicationChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_time_usec_GET(pack) == (uint64_t)4550553380381982591L);
    assert(p105_zmag_GET(pack) == (float) -3.2635398E37F);
    assert(p105_yacc_GET(pack) == (float) -6.768442E36F);
    assert(p105_zgyro_GET(pack) == (float)3.6992567E37F);
    assert(p105_xmag_GET(pack) == (float) -1.7417103E38F);
    assert(p105_ygyro_GET(pack) == (float)2.8546905E38F);
    assert(p105_pressure_alt_GET(pack) == (float) -2.5856766E38F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)54394);
    assert(p105_temperature_GET(pack) == (float) -2.3017363E38F);
    assert(p105_ymag_GET(pack) == (float) -1.4406729E38F);
    assert(p105_diff_pressure_GET(pack) == (float) -2.1710652E38F);
    assert(p105_xacc_GET(pack) == (float)5.737627E37F);
    assert(p105_abs_pressure_GET(pack) == (float)2.489318E38F);
    assert(p105_zacc_GET(pack) == (float) -8.991809E37F);
    assert(p105_xgyro_GET(pack) == (float) -2.1472186E38F);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_integration_time_us_GET(pack) == (uint32_t)192592727L);
    assert(p106_time_usec_GET(pack) == (uint64_t)4447517559160544808L);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)1783822328L);
    assert(p106_integrated_y_GET(pack) == (float) -1.060912E38F);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)69);
    assert(p106_integrated_ygyro_GET(pack) == (float)2.5507925E38F);
    assert(p106_integrated_x_GET(pack) == (float)2.1357515E38F);
    assert(p106_distance_GET(pack) == (float)2.28673E37F);
    assert(p106_integrated_xgyro_GET(pack) == (float)1.6002546E38F);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)245);
    assert(p106_integrated_zgyro_GET(pack) == (float)1.0702308E38F);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t)16766);
};


void c_CommunicationChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_zacc_GET(pack) == (float)1.6202832E38F);
    assert(p107_pressure_alt_GET(pack) == (float)3.2395132E38F);
    assert(p107_time_usec_GET(pack) == (uint64_t)7052235023388606278L);
    assert(p107_abs_pressure_GET(pack) == (float) -1.3571519E38F);
    assert(p107_xgyro_GET(pack) == (float) -5.629325E37F);
    assert(p107_xmag_GET(pack) == (float) -2.446263E38F);
    assert(p107_temperature_GET(pack) == (float) -1.3379012E38F);
    assert(p107_xacc_GET(pack) == (float)2.3321284E38F);
    assert(p107_zgyro_GET(pack) == (float) -1.5151402E38F);
    assert(p107_yacc_GET(pack) == (float) -1.0643177E38F);
    assert(p107_zmag_GET(pack) == (float)1.4807482E37F);
    assert(p107_ygyro_GET(pack) == (float) -2.2171383E38F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)2353275106L);
    assert(p107_ymag_GET(pack) == (float) -2.7117756E38F);
    assert(p107_diff_pressure_GET(pack) == (float) -1.1685672E38F);
};


void c_CommunicationChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_vn_GET(pack) == (float)7.0752123E37F);
    assert(p108_q3_GET(pack) == (float) -1.0970345E38F);
    assert(p108_zacc_GET(pack) == (float)2.9891931E37F);
    assert(p108_pitch_GET(pack) == (float)1.7827183E38F);
    assert(p108_xacc_GET(pack) == (float)1.3909587E37F);
    assert(p108_zgyro_GET(pack) == (float)7.510917E37F);
    assert(p108_xgyro_GET(pack) == (float)3.7916824E37F);
    assert(p108_roll_GET(pack) == (float)1.6009205E38F);
    assert(p108_alt_GET(pack) == (float) -2.2223943E38F);
    assert(p108_yacc_GET(pack) == (float) -8.553067E37F);
    assert(p108_q2_GET(pack) == (float)2.9441047E38F);
    assert(p108_q4_GET(pack) == (float) -7.733451E37F);
    assert(p108_q1_GET(pack) == (float) -1.1288222E38F);
    assert(p108_std_dev_horz_GET(pack) == (float)3.1067745E38F);
    assert(p108_lon_GET(pack) == (float) -2.1285752E38F);
    assert(p108_vd_GET(pack) == (float)3.002798E38F);
    assert(p108_yaw_GET(pack) == (float)2.2792096E38F);
    assert(p108_std_dev_vert_GET(pack) == (float)3.6337499E37F);
    assert(p108_ve_GET(pack) == (float)2.2567854E38F);
    assert(p108_ygyro_GET(pack) == (float)1.6179841E37F);
    assert(p108_lat_GET(pack) == (float)1.1429547E38F);
};


void c_CommunicationChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)42634);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)63713);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)245);
};


void c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)11);
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)77);
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)61);
    {
        uint8_t exemplary[] =  {(uint8_t)197, (uint8_t)70, (uint8_t)108, (uint8_t)247, (uint8_t)72, (uint8_t)106, (uint8_t)140, (uint8_t)13, (uint8_t)99, (uint8_t)88, (uint8_t)61, (uint8_t)42, (uint8_t)178, (uint8_t)49, (uint8_t)119, (uint8_t)101, (uint8_t)60, (uint8_t)75, (uint8_t)49, (uint8_t)171, (uint8_t)190, (uint8_t)5, (uint8_t)122, (uint8_t)225, (uint8_t)180, (uint8_t)181, (uint8_t)17, (uint8_t)104, (uint8_t)200, (uint8_t)253, (uint8_t)113, (uint8_t)124, (uint8_t)40, (uint8_t)88, (uint8_t)123, (uint8_t)236, (uint8_t)240, (uint8_t)18, (uint8_t)250, (uint8_t)150, (uint8_t)227, (uint8_t)78, (uint8_t)159, (uint8_t)76, (uint8_t)64, (uint8_t)169, (uint8_t)42, (uint8_t)119, (uint8_t)63, (uint8_t)251, (uint8_t)106, (uint8_t)22, (uint8_t)30, (uint8_t)3, (uint8_t)178, (uint8_t)190, (uint8_t)167, (uint8_t)238, (uint8_t)209, (uint8_t)19, (uint8_t)155, (uint8_t)32, (uint8_t)166, (uint8_t)25, (uint8_t)121, (uint8_t)207, (uint8_t)85, (uint8_t)208, (uint8_t)128, (uint8_t)90, (uint8_t)64, (uint8_t)194, (uint8_t)76, (uint8_t)144, (uint8_t)140, (uint8_t)159, (uint8_t)135, (uint8_t)208, (uint8_t)73, (uint8_t)152, (uint8_t)184, (uint8_t)212, (uint8_t)227, (uint8_t)159, (uint8_t)236, (uint8_t)6, (uint8_t)255, (uint8_t)37, (uint8_t)137, (uint8_t)163, (uint8_t)165, (uint8_t)159, (uint8_t)94, (uint8_t)9, (uint8_t)59, (uint8_t)240, (uint8_t)235, (uint8_t)106, (uint8_t)26, (uint8_t)26, (uint8_t)6, (uint8_t)57, (uint8_t)211, (uint8_t)137, (uint8_t)127, (uint8_t)85, (uint8_t)247, (uint8_t)184, (uint8_t)48, (uint8_t)161, (uint8_t)169, (uint8_t)31, (uint8_t)35, (uint8_t)31, (uint8_t)118, (uint8_t)60, (uint8_t)48, (uint8_t)228, (uint8_t)72, (uint8_t)156, (uint8_t)88, (uint8_t)115, (uint8_t)135, (uint8_t)223, (uint8_t)144, (uint8_t)96, (uint8_t)42, (uint8_t)152, (uint8_t)59, (uint8_t)172, (uint8_t)16, (uint8_t)212, (uint8_t)51, (uint8_t)36, (uint8_t)96, (uint8_t)40, (uint8_t)105, (uint8_t)77, (uint8_t)85, (uint8_t)120, (uint8_t)68, (uint8_t)58, (uint8_t)90, (uint8_t)40, (uint8_t)98, (uint8_t)183, (uint8_t)159, (uint8_t)7, (uint8_t)67, (uint8_t)23, (uint8_t)183, (uint8_t)163, (uint8_t)115, (uint8_t)85, (uint8_t)168, (uint8_t)120, (uint8_t)193, (uint8_t)4, (uint8_t)169, (uint8_t)48, (uint8_t)212, (uint8_t)6, (uint8_t)174, (uint8_t)3, (uint8_t)140, (uint8_t)20, (uint8_t)39, (uint8_t)119, (uint8_t)160, (uint8_t)120, (uint8_t)146, (uint8_t)198, (uint8_t)235, (uint8_t)189, (uint8_t)89, (uint8_t)61, (uint8_t)37, (uint8_t)63, (uint8_t)229, (uint8_t)60, (uint8_t)173, (uint8_t)204, (uint8_t)145, (uint8_t)174, (uint8_t)36, (uint8_t)63, (uint8_t)165, (uint8_t)93, (uint8_t)59, (uint8_t)203, (uint8_t)10, (uint8_t)179, (uint8_t)226, (uint8_t)121, (uint8_t)32, (uint8_t)183, (uint8_t)67, (uint8_t)144, (uint8_t)133, (uint8_t)132, (uint8_t)212, (uint8_t)252, (uint8_t)178, (uint8_t)176, (uint8_t)85, (uint8_t)213, (uint8_t)238, (uint8_t)69, (uint8_t)86, (uint8_t)12, (uint8_t)83, (uint8_t)141, (uint8_t)13, (uint8_t)9, (uint8_t)15, (uint8_t)56, (uint8_t)68, (uint8_t)74, (uint8_t)40, (uint8_t)24, (uint8_t)152, (uint8_t)216, (uint8_t)149, (uint8_t)59, (uint8_t)156, (uint8_t)31, (uint8_t)136, (uint8_t)119, (uint8_t)15, (uint8_t)135, (uint8_t)123, (uint8_t)254, (uint8_t)83, (uint8_t)213, (uint8_t)48, (uint8_t)157, (uint8_t)205, (uint8_t)220, (uint8_t)84, (uint8_t)171, (uint8_t)193, (uint8_t)55, (uint8_t)23, (uint8_t)245, (uint8_t)246, (uint8_t)188, (uint8_t)177, (uint8_t)142, (uint8_t)45, (uint8_t)124, (uint8_t)185} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_tc1_GET(pack) == (int64_t) -1074397645646006785L);
    assert(p111_ts1_GET(pack) == (int64_t) -4503979501572626575L);
};


void c_CommunicationChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_time_usec_GET(pack) == (uint64_t)7523972220779639704L);
    assert(p112_seq_GET(pack) == (uint32_t)2574819736L);
};


void c_CommunicationChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_alt_GET(pack) == (int32_t)1759643662);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)41106);
    assert(p113_lon_GET(pack) == (int32_t) -1139321670);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)127);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)44578);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t) -11795);
    assert(p113_lat_GET(pack) == (int32_t)2016367475);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)33475);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t)6595);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t)32418);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)33986);
    assert(p113_time_usec_GET(pack) == (uint64_t)2014320210638385258L);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)138);
};


void c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p114_integrated_ygyro_GET(pack) == (float)2.2311883E37F);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)174);
    assert(p114_time_usec_GET(pack) == (uint64_t)6059417121101003904L);
    assert(p114_integrated_y_GET(pack) == (float)5.4142163E37F);
    assert(p114_integrated_zgyro_GET(pack) == (float) -1.4736041E38F);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)4210302917L);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)4243594955L);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t) -1094);
    assert(p114_integrated_x_GET(pack) == (float) -2.0581642E38F);
    assert(p114_distance_GET(pack) == (float) -1.9346744E38F);
    assert(p114_integrated_xgyro_GET(pack) == (float) -1.0142576E38F);
};


void c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-2.0536662E38F, 2.4515257E38F, 3.047777E38F, -6.0362844E37F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t) -22450);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t)883);
    assert(p115_lon_GET(pack) == (int32_t)1225589712);
    assert(p115_pitchspeed_GET(pack) == (float)3.3829465E38F);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t)10371);
    assert(p115_time_usec_GET(pack) == (uint64_t)7843812968240807637L);
    assert(p115_alt_GET(pack) == (int32_t)1669851940);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t) -11974);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t) -3944);
    assert(p115_rollspeed_GET(pack) == (float) -1.1068961E38F);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t) -32595);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)9177);
    assert(p115_yawspeed_GET(pack) == (float)1.6720357E38F);
    assert(p115_lat_GET(pack) == (int32_t) -1810980656);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)7510);
};


void c_CommunicationChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t)20766);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)2625390658L);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t)25997);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t) -2109);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t) -17810);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t) -27819);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t)1573);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t) -26040);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t)4707);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t)6131);
};


void c_CommunicationChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)62);
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)55743);
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)18937);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)219);
};


void c_CommunicationChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)43475);
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)41435);
    assert(p118_time_utc_GET(pack) == (uint32_t)519077390L);
    assert(p118_size_GET(pack) == (uint32_t)1698359049L);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)34580);
};


void c_CommunicationChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)60398);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)174);
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)177);
    assert(p119_count_GET(pack) == (uint32_t)315230903L);
    assert(p119_ofs_GET(pack) == (uint32_t)3195580485L);
};


void c_CommunicationChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)106, (uint8_t)79, (uint8_t)24, (uint8_t)215, (uint8_t)130, (uint8_t)40, (uint8_t)144, (uint8_t)24, (uint8_t)8, (uint8_t)199, (uint8_t)87, (uint8_t)153, (uint8_t)34, (uint8_t)139, (uint8_t)210, (uint8_t)193, (uint8_t)122, (uint8_t)215, (uint8_t)176, (uint8_t)28, (uint8_t)115, (uint8_t)75, (uint8_t)233, (uint8_t)196, (uint8_t)232, (uint8_t)138, (uint8_t)78, (uint8_t)159, (uint8_t)32, (uint8_t)250, (uint8_t)193, (uint8_t)143, (uint8_t)107, (uint8_t)180, (uint8_t)205, (uint8_t)201, (uint8_t)196, (uint8_t)113, (uint8_t)56, (uint8_t)77, (uint8_t)214, (uint8_t)196, (uint8_t)189, (uint8_t)110, (uint8_t)224, (uint8_t)75, (uint8_t)197, (uint8_t)21, (uint8_t)71, (uint8_t)191, (uint8_t)123, (uint8_t)196, (uint8_t)62, (uint8_t)122, (uint8_t)91, (uint8_t)189, (uint8_t)117, (uint8_t)57, (uint8_t)248, (uint8_t)50, (uint8_t)141, (uint8_t)89, (uint8_t)12, (uint8_t)12, (uint8_t)74, (uint8_t)48, (uint8_t)219, (uint8_t)22, (uint8_t)119, (uint8_t)112, (uint8_t)243, (uint8_t)170, (uint8_t)17, (uint8_t)36, (uint8_t)26, (uint8_t)1, (uint8_t)124, (uint8_t)21, (uint8_t)115, (uint8_t)74, (uint8_t)37, (uint8_t)227, (uint8_t)93, (uint8_t)189, (uint8_t)62, (uint8_t)141, (uint8_t)133, (uint8_t)161, (uint8_t)193, (uint8_t)234} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)144);
    assert(p120_ofs_GET(pack) == (uint32_t)3173452605L);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)52632);
};


void c_CommunicationChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)193);
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)47);
};


void c_CommunicationChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)129);
};


void c_CommunicationChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)225);
    {
        uint8_t exemplary[] =  {(uint8_t)21, (uint8_t)245, (uint8_t)101, (uint8_t)163, (uint8_t)168, (uint8_t)212, (uint8_t)201, (uint8_t)168, (uint8_t)164, (uint8_t)72, (uint8_t)145, (uint8_t)166, (uint8_t)2, (uint8_t)34, (uint8_t)116, (uint8_t)211, (uint8_t)126, (uint8_t)48, (uint8_t)134, (uint8_t)30, (uint8_t)115, (uint8_t)104, (uint8_t)64, (uint8_t)252, (uint8_t)188, (uint8_t)152, (uint8_t)128, (uint8_t)199, (uint8_t)76, (uint8_t)38, (uint8_t)117, (uint8_t)128, (uint8_t)64, (uint8_t)7, (uint8_t)162, (uint8_t)112, (uint8_t)163, (uint8_t)9, (uint8_t)4, (uint8_t)68, (uint8_t)211, (uint8_t)127, (uint8_t)113, (uint8_t)226, (uint8_t)239, (uint8_t)77, (uint8_t)21, (uint8_t)115, (uint8_t)166, (uint8_t)50, (uint8_t)22, (uint8_t)196, (uint8_t)231, (uint8_t)14, (uint8_t)140, (uint8_t)93, (uint8_t)15, (uint8_t)197, (uint8_t)144, (uint8_t)238, (uint8_t)30, (uint8_t)179, (uint8_t)94, (uint8_t)142, (uint8_t)248, (uint8_t)166, (uint8_t)124, (uint8_t)108, (uint8_t)43, (uint8_t)49, (uint8_t)6, (uint8_t)202, (uint8_t)75, (uint8_t)99, (uint8_t)13, (uint8_t)114, (uint8_t)250, (uint8_t)41, (uint8_t)161, (uint8_t)134, (uint8_t)233, (uint8_t)188, (uint8_t)187, (uint8_t)190, (uint8_t)58, (uint8_t)76, (uint8_t)213, (uint8_t)184, (uint8_t)60, (uint8_t)14, (uint8_t)86, (uint8_t)79, (uint8_t)184, (uint8_t)14, (uint8_t)185, (uint8_t)188, (uint8_t)204, (uint8_t)35, (uint8_t)171, (uint8_t)248, (uint8_t)118, (uint8_t)41, (uint8_t)219, (uint8_t)206, (uint8_t)205, (uint8_t)21, (uint8_t)228, (uint8_t)58, (uint8_t)215, (uint8_t)172} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)146);
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)77);
};


void c_CommunicationChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_time_usec_GET(pack) == (uint64_t)8409079341114938581L);
    assert(p124_dgps_age_GET(pack) == (uint32_t)924723069L);
    assert(p124_alt_GET(pack) == (int32_t)1824737713);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED);
    assert(p124_lat_GET(pack) == (int32_t)327628374);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)246);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)21);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)37161);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)49005);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)34265);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)21515);
    assert(p124_lon_GET(pack) == (int32_t) -1214939568);
};


void c_CommunicationChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)34879);
    assert(p125_flags_GET(pack) == e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT);
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)39298);
};


void c_CommunicationChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    assert(p126_baudrate_GET(pack) == (uint32_t)2389447527L);
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2);
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)27880);
    {
        uint8_t exemplary[] =  {(uint8_t)214, (uint8_t)187, (uint8_t)112, (uint8_t)67, (uint8_t)73, (uint8_t)65, (uint8_t)90, (uint8_t)85, (uint8_t)104, (uint8_t)113, (uint8_t)17, (uint8_t)87, (uint8_t)20, (uint8_t)13, (uint8_t)169, (uint8_t)218, (uint8_t)255, (uint8_t)128, (uint8_t)216, (uint8_t)242, (uint8_t)23, (uint8_t)92, (uint8_t)142, (uint8_t)85, (uint8_t)66, (uint8_t)10, (uint8_t)236, (uint8_t)48, (uint8_t)145, (uint8_t)33, (uint8_t)0, (uint8_t)45, (uint8_t)180, (uint8_t)14, (uint8_t)252, (uint8_t)21, (uint8_t)253, (uint8_t)141, (uint8_t)99, (uint8_t)223, (uint8_t)170, (uint8_t)37, (uint8_t)96, (uint8_t)234, (uint8_t)92, (uint8_t)214, (uint8_t)144, (uint8_t)87, (uint8_t)195, (uint8_t)155, (uint8_t)177, (uint8_t)113, (uint8_t)235, (uint8_t)182, (uint8_t)54, (uint8_t)76, (uint8_t)212, (uint8_t)143, (uint8_t)196, (uint8_t)219, (uint8_t)229, (uint8_t)159, (uint8_t)73, (uint8_t)177, (uint8_t)102, (uint8_t)139, (uint8_t)69, (uint8_t)242, (uint8_t)163, (uint8_t)178} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_flags_GET(pack) == e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE);
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)104);
};


void c_CommunicationChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_accuracy_GET(pack) == (uint32_t)3826159446L);
    assert(p127_tow_GET(pack) == (uint32_t)3956015755L);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t)1521052173);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)33254);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)30);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t) -740340532);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t) -1549676544);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)407696327L);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t)1072032946);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)54);
};


void c_CommunicationChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t) -808789491);
    assert(p128_accuracy_GET(pack) == (uint32_t)4108065037L);
    assert(p128_tow_GET(pack) == (uint32_t)2062685219L);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)232);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t) -1920082763);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -775747667);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)132);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t) -1585444943);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)4183668997L);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)32218);
};


void c_CommunicationChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t)23290);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t)18839);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t) -28677);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t) -731);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t)29331);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t)30988);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)1437903733L);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t) -30815);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t)992);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)26659);
};


void c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)116);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)29514);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)172);
    assert(p130_size_GET(pack) == (uint32_t)993204203L);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)12577);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)7213);
};


void c_CommunicationChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)67, (uint8_t)195, (uint8_t)78, (uint8_t)138, (uint8_t)28, (uint8_t)176, (uint8_t)122, (uint8_t)37, (uint8_t)174, (uint8_t)147, (uint8_t)245, (uint8_t)0, (uint8_t)17, (uint8_t)29, (uint8_t)223, (uint8_t)207, (uint8_t)98, (uint8_t)185, (uint8_t)59, (uint8_t)192, (uint8_t)11, (uint8_t)145, (uint8_t)155, (uint8_t)1, (uint8_t)155, (uint8_t)69, (uint8_t)186, (uint8_t)169, (uint8_t)174, (uint8_t)22, (uint8_t)122, (uint8_t)36, (uint8_t)170, (uint8_t)37, (uint8_t)160, (uint8_t)3, (uint8_t)103, (uint8_t)248, (uint8_t)71, (uint8_t)143, (uint8_t)174, (uint8_t)216, (uint8_t)161, (uint8_t)207, (uint8_t)178, (uint8_t)76, (uint8_t)166, (uint8_t)36, (uint8_t)75, (uint8_t)172, (uint8_t)109, (uint8_t)164, (uint8_t)209, (uint8_t)166, (uint8_t)57, (uint8_t)40, (uint8_t)29, (uint8_t)95, (uint8_t)186, (uint8_t)51, (uint8_t)224, (uint8_t)210, (uint8_t)114, (uint8_t)205, (uint8_t)148, (uint8_t)188, (uint8_t)56, (uint8_t)43, (uint8_t)68, (uint8_t)69, (uint8_t)7, (uint8_t)44, (uint8_t)156, (uint8_t)253, (uint8_t)228, (uint8_t)6, (uint8_t)134, (uint8_t)176, (uint8_t)45, (uint8_t)6, (uint8_t)60, (uint8_t)176, (uint8_t)223, (uint8_t)20, (uint8_t)69, (uint8_t)223, (uint8_t)39, (uint8_t)209, (uint8_t)41, (uint8_t)134, (uint8_t)217, (uint8_t)208, (uint8_t)2, (uint8_t)67, (uint8_t)217, (uint8_t)242, (uint8_t)69, (uint8_t)211, (uint8_t)78, (uint8_t)229, (uint8_t)57, (uint8_t)63, (uint8_t)211, (uint8_t)42, (uint8_t)154, (uint8_t)22, (uint8_t)93, (uint8_t)76, (uint8_t)78, (uint8_t)39, (uint8_t)114, (uint8_t)129, (uint8_t)16, (uint8_t)0, (uint8_t)189, (uint8_t)243, (uint8_t)55, (uint8_t)210, (uint8_t)170, (uint8_t)218, (uint8_t)26, (uint8_t)197, (uint8_t)211, (uint8_t)233, (uint8_t)13, (uint8_t)208, (uint8_t)57, (uint8_t)20, (uint8_t)191, (uint8_t)55, (uint8_t)142, (uint8_t)247, (uint8_t)187, (uint8_t)209, (uint8_t)220, (uint8_t)69, (uint8_t)185, (uint8_t)142, (uint8_t)211, (uint8_t)145, (uint8_t)203, (uint8_t)194, (uint8_t)173, (uint8_t)166, (uint8_t)159, (uint8_t)59, (uint8_t)247, (uint8_t)175, (uint8_t)135, (uint8_t)97, (uint8_t)169, (uint8_t)237, (uint8_t)224, (uint8_t)176, (uint8_t)42, (uint8_t)190, (uint8_t)198, (uint8_t)203, (uint8_t)52, (uint8_t)187, (uint8_t)152, (uint8_t)99, (uint8_t)103, (uint8_t)12, (uint8_t)204, (uint8_t)118, (uint8_t)113, (uint8_t)192, (uint8_t)221, (uint8_t)191, (uint8_t)128, (uint8_t)171, (uint8_t)168, (uint8_t)100, (uint8_t)14, (uint8_t)247, (uint8_t)218, (uint8_t)171, (uint8_t)195, (uint8_t)44, (uint8_t)18, (uint8_t)121, (uint8_t)186, (uint8_t)177, (uint8_t)137, (uint8_t)226, (uint8_t)126, (uint8_t)227, (uint8_t)126, (uint8_t)195, (uint8_t)9, (uint8_t)8, (uint8_t)60, (uint8_t)47, (uint8_t)82, (uint8_t)49, (uint8_t)29, (uint8_t)204, (uint8_t)224, (uint8_t)6, (uint8_t)108, (uint8_t)181, (uint8_t)48, (uint8_t)115, (uint8_t)80, (uint8_t)21, (uint8_t)207, (uint8_t)182, (uint8_t)250, (uint8_t)240, (uint8_t)218, (uint8_t)89, (uint8_t)255, (uint8_t)75, (uint8_t)240, (uint8_t)196, (uint8_t)48, (uint8_t)62, (uint8_t)142, (uint8_t)228, (uint8_t)40, (uint8_t)17, (uint8_t)163, (uint8_t)80, (uint8_t)47, (uint8_t)221, (uint8_t)33, (uint8_t)54, (uint8_t)39, (uint8_t)204, (uint8_t)229, (uint8_t)64, (uint8_t)16, (uint8_t)126, (uint8_t)118, (uint8_t)147, (uint8_t)141, (uint8_t)95, (uint8_t)250, (uint8_t)49, (uint8_t)111, (uint8_t)115, (uint8_t)4, (uint8_t)163, (uint8_t)198, (uint8_t)230, (uint8_t)250, (uint8_t)39, (uint8_t)185, (uint8_t)163, (uint8_t)92, (uint8_t)4, (uint8_t)124} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)65191);
};


void c_CommunicationChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)72);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_YAW_45);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)1267960422L);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)14543);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)51722);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)50042);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)123);
};


void c_CommunicationChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_lat_GET(pack) == (int32_t) -209950872);
    assert(p133_mask_GET(pack) == (uint64_t)6082779728082239696L);
    assert(p133_lon_GET(pack) == (int32_t) -826887086);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)48333);
};


void c_CommunicationChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    {
        int16_t exemplary[] =  {(int16_t) -25505, (int16_t) -26078, (int16_t)381, (int16_t) -31878, (int16_t)8913, (int16_t)18187, (int16_t) -29538, (int16_t)17916, (int16_t)32061, (int16_t)6934, (int16_t)24234, (int16_t)25784, (int16_t) -11780, (int16_t) -7239, (int16_t) -29844, (int16_t)19914} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_lon_GET(pack) == (int32_t)1817922006);
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)40403);
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)49);
    assert(p134_lat_GET(pack) == (int32_t)1209111922);
};


void c_CommunicationChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lon_GET(pack) == (int32_t)362569806);
    assert(p135_lat_GET(pack) == (int32_t)1991355880);
};


void c_CommunicationChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_lon_GET(pack) == (int32_t)1988798008);
    assert(p136_lat_GET(pack) == (int32_t)950020838);
    assert(p136_current_height_GET(pack) == (float)2.8193229E38F);
    assert(p136_terrain_height_GET(pack) == (float)2.3784914E38F);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)11969);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)46215);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)39496);
};


void c_CommunicationChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_press_abs_GET(pack) == (float) -2.8014953E38F);
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t) -27648);
    assert(p137_press_diff_GET(pack) == (float) -2.2756995E38F);
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)142501613L);
};


void c_CommunicationChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {1.1891355E38F, -3.3445432E38F, -5.5984136E37F, 2.598731E37F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_y_GET(pack) == (float)1.4725488E38F);
    assert(p138_z_GET(pack) == (float) -1.4701388E38F);
    assert(p138_time_usec_GET(pack) == (uint64_t)8409474775135957733L);
    assert(p138_x_GET(pack) == (float)7.5147337E37F);
};


void c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-2.0460351E38F, -2.6385028E38F, -2.9043023E38F, 1.7854013E38F, -2.9608913E38F, -2.774348E38F, -1.3605871E37F, 2.3192108E38F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)138);
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)21);
    assert(p139_time_usec_GET(pack) == (uint64_t)4227698352066896780L);
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)106);
};


void c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)7);
    assert(p140_time_usec_GET(pack) == (uint64_t)661060580851391339L);
    {
        float exemplary[] =  {-3.0913299E38F, 1.1143582E38F, -1.0871971E38F, -2.7190203E38F, -1.1998448E38F, -1.3092109E38F, 4.9332656E37F, -2.2168404E38F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_altitude_terrain_GET(pack) == (float)3.1842848E38F);
    assert(p141_time_usec_GET(pack) == (uint64_t)2143236324645149855L);
    assert(p141_bottom_clearance_GET(pack) == (float)2.5894333E38F);
    assert(p141_altitude_monotonic_GET(pack) == (float) -1.3929623E38F);
    assert(p141_altitude_relative_GET(pack) == (float)9.161881E37F);
    assert(p141_altitude_local_GET(pack) == (float)1.2399897E38F);
    assert(p141_altitude_amsl_GET(pack) == (float)1.6077256E38F);
};


void c_CommunicationChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)169);
    {
        uint8_t exemplary[] =  {(uint8_t)109, (uint8_t)193, (uint8_t)100, (uint8_t)148, (uint8_t)84, (uint8_t)212, (uint8_t)197, (uint8_t)212, (uint8_t)66, (uint8_t)34, (uint8_t)126, (uint8_t)179, (uint8_t)63, (uint8_t)184, (uint8_t)42, (uint8_t)26, (uint8_t)88, (uint8_t)73, (uint8_t)145, (uint8_t)1, (uint8_t)139, (uint8_t)168, (uint8_t)72, (uint8_t)126, (uint8_t)72, (uint8_t)167, (uint8_t)156, (uint8_t)190, (uint8_t)6, (uint8_t)242, (uint8_t)127, (uint8_t)237, (uint8_t)213, (uint8_t)165, (uint8_t)210, (uint8_t)251, (uint8_t)65, (uint8_t)23, (uint8_t)145, (uint8_t)145, (uint8_t)133, (uint8_t)73, (uint8_t)59, (uint8_t)61, (uint8_t)62, (uint8_t)70, (uint8_t)27, (uint8_t)153, (uint8_t)148, (uint8_t)92, (uint8_t)211, (uint8_t)108, (uint8_t)224, (uint8_t)144, (uint8_t)169, (uint8_t)134, (uint8_t)108, (uint8_t)136, (uint8_t)241, (uint8_t)160, (uint8_t)80, (uint8_t)17, (uint8_t)160, (uint8_t)47, (uint8_t)243, (uint8_t)176, (uint8_t)184, (uint8_t)212, (uint8_t)82, (uint8_t)75, (uint8_t)154, (uint8_t)201, (uint8_t)71, (uint8_t)187, (uint8_t)169, (uint8_t)29, (uint8_t)148, (uint8_t)193, (uint8_t)125, (uint8_t)11, (uint8_t)0, (uint8_t)106, (uint8_t)64, (uint8_t)144, (uint8_t)109, (uint8_t)16, (uint8_t)54, (uint8_t)63, (uint8_t)162, (uint8_t)230, (uint8_t)160, (uint8_t)251, (uint8_t)105, (uint8_t)194, (uint8_t)171, (uint8_t)216, (uint8_t)183, (uint8_t)18, (uint8_t)60, (uint8_t)244, (uint8_t)140, (uint8_t)239, (uint8_t)168, (uint8_t)157, (uint8_t)44, (uint8_t)159, (uint8_t)10, (uint8_t)83, (uint8_t)225, (uint8_t)115, (uint8_t)231, (uint8_t)121, (uint8_t)35, (uint8_t)166, (uint8_t)143, (uint8_t)171, (uint8_t)97, (uint8_t)195, (uint8_t)162, (uint8_t)230} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)206);
    {
        uint8_t exemplary[] =  {(uint8_t)107, (uint8_t)62, (uint8_t)85, (uint8_t)48, (uint8_t)19, (uint8_t)236, (uint8_t)193, (uint8_t)205, (uint8_t)132, (uint8_t)240, (uint8_t)60, (uint8_t)127, (uint8_t)231, (uint8_t)195, (uint8_t)250, (uint8_t)200, (uint8_t)23, (uint8_t)35, (uint8_t)244, (uint8_t)220, (uint8_t)150, (uint8_t)131, (uint8_t)65, (uint8_t)69, (uint8_t)42, (uint8_t)47, (uint8_t)114, (uint8_t)128, (uint8_t)76, (uint8_t)232, (uint8_t)36, (uint8_t)80, (uint8_t)30, (uint8_t)214, (uint8_t)206, (uint8_t)128, (uint8_t)97, (uint8_t)64, (uint8_t)249, (uint8_t)216, (uint8_t)58, (uint8_t)36, (uint8_t)87, (uint8_t)255, (uint8_t)133, (uint8_t)116, (uint8_t)0, (uint8_t)184, (uint8_t)189, (uint8_t)92, (uint8_t)128, (uint8_t)210, (uint8_t)253, (uint8_t)248, (uint8_t)185, (uint8_t)129, (uint8_t)93, (uint8_t)175, (uint8_t)208, (uint8_t)62, (uint8_t)247, (uint8_t)94, (uint8_t)154, (uint8_t)194, (uint8_t)169, (uint8_t)191, (uint8_t)49, (uint8_t)159, (uint8_t)172, (uint8_t)62, (uint8_t)133, (uint8_t)45, (uint8_t)244, (uint8_t)104, (uint8_t)38, (uint8_t)189, (uint8_t)43, (uint8_t)196, (uint8_t)59, (uint8_t)62, (uint8_t)254, (uint8_t)24, (uint8_t)104, (uint8_t)149, (uint8_t)71, (uint8_t)67, (uint8_t)117, (uint8_t)235, (uint8_t)205, (uint8_t)217, (uint8_t)251, (uint8_t)50, (uint8_t)35, (uint8_t)58, (uint8_t)114, (uint8_t)123, (uint8_t)85, (uint8_t)195, (uint8_t)182, (uint8_t)29, (uint8_t)37, (uint8_t)45, (uint8_t)212, (uint8_t)158, (uint8_t)234, (uint8_t)219, (uint8_t)203, (uint8_t)112, (uint8_t)165, (uint8_t)223, (uint8_t)94, (uint8_t)185, (uint8_t)182, (uint8_t)144, (uint8_t)158, (uint8_t)165, (uint8_t)207, (uint8_t)88, (uint8_t)249, (uint8_t)11} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)248);
};


void c_CommunicationChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_press_diff_GET(pack) == (float)5.46746E36F);
    assert(p143_press_abs_GET(pack) == (float)3.1304135E37F);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t)7715);
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)1157523802L);
};


void c_CommunicationChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    assert(p144_alt_GET(pack) == (float) -2.3858624E38F);
    assert(p144_lon_GET(pack) == (int32_t) -692924682);
    {
        float exemplary[] =  {4.7285377E36F, 2.7566556E37F, -1.9341726E38F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lat_GET(pack) == (int32_t)998017586);
    assert(p144_custom_state_GET(pack) == (uint64_t)7664430270382174409L);
    assert(p144_timestamp_GET(pack) == (uint64_t)1568909963772582877L);
    {
        float exemplary[] =  {-1.883556E38F, 4.044618E37F, 2.875043E38F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {3.6147346E37F, 9.872865E37F, 3.0164602E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)218);
    {
        float exemplary[] =  {-3.222998E37F, -2.9452622E37F, 5.1835485E37F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-4.241236E37F, 1.7161782E38F, -9.343324E37F, 2.6561594E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_x_acc_GET(pack) == (float)2.5916753E38F);
    assert(p146_z_pos_GET(pack) == (float) -2.3488075E37F);
    assert(p146_y_pos_GET(pack) == (float)2.1224114E37F);
    assert(p146_time_usec_GET(pack) == (uint64_t)8831759570835792413L);
    assert(p146_y_vel_GET(pack) == (float)2.9054515E38F);
    assert(p146_airspeed_GET(pack) == (float) -8.863676E37F);
    assert(p146_roll_rate_GET(pack) == (float) -1.783759E38F);
    assert(p146_pitch_rate_GET(pack) == (float) -2.8401908E38F);
    assert(p146_x_vel_GET(pack) == (float)1.8112543E38F);
    assert(p146_z_acc_GET(pack) == (float) -2.7851304E38F);
    {
        float exemplary[] =  {2.027015E38F, 2.9524974E38F, 2.374385E38F, 1.1899764E38F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {4.7025715E37F, 2.48962E38F, -6.9095867E37F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_yaw_rate_GET(pack) == (float)1.6090581E38F);
    assert(p146_x_pos_GET(pack) == (float) -1.9288375E38F);
    assert(p146_z_vel_GET(pack) == (float) -2.7355914E38F);
    {
        float exemplary[] =  {-2.7666464E38F, -2.629334E38F, 1.6348374E38F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_y_acc_GET(pack) == (float) -1.9888423E37F);
};


void c_CommunicationChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t)274);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD);
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)147);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN);
    assert(p147_current_consumed_GET(pack) == (int32_t)1599460467);
    {
        uint16_t exemplary[] =  {(uint16_t)12488, (uint16_t)64949, (uint16_t)53511, (uint16_t)38656, (uint16_t)16543, (uint16_t)5882, (uint16_t)29444, (uint16_t)28913, (uint16_t)45448, (uint16_t)119} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t)72);
    assert(p147_energy_consumed_GET(pack) == (int32_t)700548000);
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t) -5544);
};


void c_CommunicationChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_board_version_GET(pack) == (uint32_t)1865562068L);
    {
        uint8_t exemplary[] =  {(uint8_t)157, (uint8_t)61, (uint8_t)230, (uint8_t)101, (uint8_t)119, (uint8_t)110, (uint8_t)244, (uint8_t)12} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_capabilities_GET(pack) == e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_UNION);
    assert(p148_uid_GET(pack) == (uint64_t)2060550490583120016L);
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)1362169386L);
    assert(p148_os_sw_version_GET(pack) == (uint32_t)1952435554L);
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)98256008L);
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)52606);
    {
        uint8_t exemplary[] =  {(uint8_t)192, (uint8_t)176, (uint8_t)124, (uint8_t)213, (uint8_t)189, (uint8_t)193, (uint8_t)132, (uint8_t)79, (uint8_t)144, (uint8_t)13, (uint8_t)228, (uint8_t)55, (uint8_t)8, (uint8_t)10, (uint8_t)199, (uint8_t)237, (uint8_t)93, (uint8_t)13} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)18163);
    {
        uint8_t exemplary[] =  {(uint8_t)245, (uint8_t)77, (uint8_t)115, (uint8_t)21, (uint8_t)210, (uint8_t)165, (uint8_t)254, (uint8_t)110} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)219, (uint8_t)245, (uint8_t)237, (uint8_t)18, (uint8_t)212, (uint8_t)24, (uint8_t)169, (uint8_t)213} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_x_TRY(ph) == (float) -8.836036E37F);
    assert(p149_angle_x_GET(pack) == (float)2.9192776E38F);
    assert(p149_z_TRY(ph) == (float) -2.3940825E38F);
    assert(p149_y_TRY(ph) == (float)2.8163116E38F);
    assert(p149_size_x_GET(pack) == (float)2.4800537E38F);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER);
    assert(p149_time_usec_GET(pack) == (uint64_t)682114256495233422L);
    {
        float exemplary[] =  {-6.3727113E35F, 3.2469268E38F, -2.725903E38F, 1.4243398E38F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p149_distance_GET(pack) == (float) -3.1468696E38F);
    assert(p149_size_y_GET(pack) == (float) -8.128081E37F);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)174);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)176);
    assert(p149_angle_y_GET(pack) == (float) -3.6380523E37F);
};


void c_CommunicationChannel_on_SENSOR_OFFSETS_150(Bounds_Inside * ph, Pack * pack)
{
    assert(p150_mag_ofs_z_GET(pack) == (int16_t)(int16_t)19019);
    assert(p150_raw_press_GET(pack) == (int32_t) -567587202);
    assert(p150_accel_cal_x_GET(pack) == (float)3.4028147E38F);
    assert(p150_accel_cal_z_GET(pack) == (float) -3.0005215E38F);
    assert(p150_accel_cal_y_GET(pack) == (float)1.2799672E38F);
    assert(p150_gyro_cal_y_GET(pack) == (float) -2.1859718E38F);
    assert(p150_mag_ofs_x_GET(pack) == (int16_t)(int16_t)14507);
    assert(p150_gyro_cal_z_GET(pack) == (float) -1.3649805E38F);
    assert(p150_gyro_cal_x_GET(pack) == (float)1.9117508E38F);
    assert(p150_raw_temp_GET(pack) == (int32_t) -969331691);
    assert(p150_mag_declination_GET(pack) == (float) -7.462029E37F);
    assert(p150_mag_ofs_y_GET(pack) == (int16_t)(int16_t) -9086);
};


void c_CommunicationChannel_on_SET_MAG_OFFSETS_151(Bounds_Inside * ph, Pack * pack)
{
    assert(p151_mag_ofs_x_GET(pack) == (int16_t)(int16_t)23784);
    assert(p151_target_system_GET(pack) == (uint8_t)(uint8_t)159);
    assert(p151_mag_ofs_z_GET(pack) == (int16_t)(int16_t) -10785);
    assert(p151_target_component_GET(pack) == (uint8_t)(uint8_t)142);
    assert(p151_mag_ofs_y_GET(pack) == (int16_t)(int16_t) -23537);
};


void c_CommunicationChannel_on_MEMINFO_152(Bounds_Inside * ph, Pack * pack)
{
    assert(p152_freemem_GET(pack) == (uint16_t)(uint16_t)48464);
    assert(p152_freemem32_TRY(ph) == (uint32_t)663973652L);
    assert(p152_brkval_GET(pack) == (uint16_t)(uint16_t)41161);
};


void c_CommunicationChannel_on_AP_ADC_153(Bounds_Inside * ph, Pack * pack)
{
    assert(p153_adc1_GET(pack) == (uint16_t)(uint16_t)6505);
    assert(p153_adc6_GET(pack) == (uint16_t)(uint16_t)21677);
    assert(p153_adc2_GET(pack) == (uint16_t)(uint16_t)13226);
    assert(p153_adc3_GET(pack) == (uint16_t)(uint16_t)2387);
    assert(p153_adc4_GET(pack) == (uint16_t)(uint16_t)7047);
    assert(p153_adc5_GET(pack) == (uint16_t)(uint16_t)53226);
};


void c_CommunicationChannel_on_DIGICAM_CONFIGURE_154(Bounds_Inside * ph, Pack * pack)
{
    assert(p154_mode_GET(pack) == (uint8_t)(uint8_t)250);
    assert(p154_shutter_speed_GET(pack) == (uint16_t)(uint16_t)57125);
    assert(p154_iso_GET(pack) == (uint8_t)(uint8_t)147);
    assert(p154_command_id_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p154_exposure_type_GET(pack) == (uint8_t)(uint8_t)106);
    assert(p154_engine_cut_off_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p154_aperture_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p154_target_system_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p154_extra_value_GET(pack) == (float)1.0332788E38F);
    assert(p154_extra_param_GET(pack) == (uint8_t)(uint8_t)231);
    assert(p154_target_component_GET(pack) == (uint8_t)(uint8_t)246);
};


void c_CommunicationChannel_on_DIGICAM_CONTROL_155(Bounds_Inside * ph, Pack * pack)
{
    assert(p155_target_component_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p155_target_system_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p155_zoom_pos_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p155_extra_param_GET(pack) == (uint8_t)(uint8_t)2);
    assert(p155_command_id_GET(pack) == (uint8_t)(uint8_t)163);
    assert(p155_shot_GET(pack) == (uint8_t)(uint8_t)65);
    assert(p155_extra_value_GET(pack) == (float) -2.8971163E38F);
    assert(p155_focus_lock_GET(pack) == (uint8_t)(uint8_t)199);
    assert(p155_session_GET(pack) == (uint8_t)(uint8_t)95);
    assert(p155_zoom_step_GET(pack) == (int8_t)(int8_t)41);
};


void c_CommunicationChannel_on_MOUNT_CONFIGURE_156(Bounds_Inside * ph, Pack * pack)
{
    assert(p156_stab_roll_GET(pack) == (uint8_t)(uint8_t)163);
    assert(p156_stab_yaw_GET(pack) == (uint8_t)(uint8_t)96);
    assert(p156_target_system_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p156_target_component_GET(pack) == (uint8_t)(uint8_t)50);
    assert(p156_stab_pitch_GET(pack) == (uint8_t)(uint8_t)138);
    assert(p156_mount_mode_GET(pack) == e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_NEUTRAL);
};


void c_CommunicationChannel_on_MOUNT_CONTROL_157(Bounds_Inside * ph, Pack * pack)
{
    assert(p157_save_position_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p157_input_c_GET(pack) == (int32_t) -470244822);
    assert(p157_target_system_GET(pack) == (uint8_t)(uint8_t)43);
    assert(p157_input_a_GET(pack) == (int32_t)167750892);
    assert(p157_input_b_GET(pack) == (int32_t)1133727391);
    assert(p157_target_component_GET(pack) == (uint8_t)(uint8_t)56);
};


void c_CommunicationChannel_on_MOUNT_STATUS_158(Bounds_Inside * ph, Pack * pack)
{
    assert(p158_target_component_GET(pack) == (uint8_t)(uint8_t)82);
    assert(p158_target_system_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p158_pointing_b_GET(pack) == (int32_t) -659029092);
    assert(p158_pointing_a_GET(pack) == (int32_t)278412514);
    assert(p158_pointing_c_GET(pack) == (int32_t)1237361865);
};


void c_CommunicationChannel_on_FENCE_POINT_160(Bounds_Inside * ph, Pack * pack)
{
    assert(p160_idx_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p160_target_system_GET(pack) == (uint8_t)(uint8_t)124);
    assert(p160_target_component_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p160_count_GET(pack) == (uint8_t)(uint8_t)113);
    assert(p160_lng_GET(pack) == (float) -2.9939031E38F);
    assert(p160_lat_GET(pack) == (float)2.6523684E38F);
};


void c_CommunicationChannel_on_FENCE_FETCH_POINT_161(Bounds_Inside * ph, Pack * pack)
{
    assert(p161_target_component_GET(pack) == (uint8_t)(uint8_t)227);
    assert(p161_idx_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p161_target_system_GET(pack) == (uint8_t)(uint8_t)183);
};


void c_CommunicationChannel_on_FENCE_STATUS_162(Bounds_Inside * ph, Pack * pack)
{
    assert(p162_breach_time_GET(pack) == (uint32_t)3672566917L);
    assert(p162_breach_count_GET(pack) == (uint16_t)(uint16_t)60320);
    assert(p162_breach_type_GET(pack) == e_FENCE_BREACH_FENCE_BREACH_BOUNDARY);
    assert(p162_breach_status_GET(pack) == (uint8_t)(uint8_t)100);
};


void c_CommunicationChannel_on_AHRS_163(Bounds_Inside * ph, Pack * pack)
{
    assert(p163_error_rp_GET(pack) == (float) -1.2217591E38F);
    assert(p163_omegaIx_GET(pack) == (float)2.079657E38F);
    assert(p163_omegaIz_GET(pack) == (float)2.6220113E38F);
    assert(p163_renorm_val_GET(pack) == (float)3.312148E38F);
    assert(p163_error_yaw_GET(pack) == (float) -5.7865005E37F);
    assert(p163_accel_weight_GET(pack) == (float)2.9375032E38F);
    assert(p163_omegaIy_GET(pack) == (float)2.3252622E38F);
};


void c_CommunicationChannel_on_SIMSTATE_164(Bounds_Inside * ph, Pack * pack)
{
    assert(p164_xacc_GET(pack) == (float) -1.2955067E38F);
    assert(p164_lng_GET(pack) == (int32_t)1221361565);
    assert(p164_roll_GET(pack) == (float)2.109632E38F);
    assert(p164_pitch_GET(pack) == (float)2.210673E38F);
    assert(p164_yaw_GET(pack) == (float) -1.4853159E38F);
    assert(p164_lat_GET(pack) == (int32_t) -13860839);
    assert(p164_ygyro_GET(pack) == (float) -1.187554E38F);
    assert(p164_xgyro_GET(pack) == (float) -2.782022E38F);
    assert(p164_zacc_GET(pack) == (float)2.8230082E38F);
    assert(p164_yacc_GET(pack) == (float)1.3844797E38F);
    assert(p164_zgyro_GET(pack) == (float)2.1796135E38F);
};


void c_CommunicationChannel_on_HWSTATUS_165(Bounds_Inside * ph, Pack * pack)
{
    assert(p165_Vcc_GET(pack) == (uint16_t)(uint16_t)59699);
    assert(p165_I2Cerr_GET(pack) == (uint8_t)(uint8_t)166);
};


void c_CommunicationChannel_on_RADIO_166(Bounds_Inside * ph, Pack * pack)
{
    assert(p166_remnoise_GET(pack) == (uint8_t)(uint8_t)39);
    assert(p166_txbuf_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p166_fixed__GET(pack) == (uint16_t)(uint16_t)48602);
    assert(p166_noise_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p166_remrssi_GET(pack) == (uint8_t)(uint8_t)90);
    assert(p166_rxerrors_GET(pack) == (uint16_t)(uint16_t)21420);
    assert(p166_rssi_GET(pack) == (uint8_t)(uint8_t)0);
};


void c_CommunicationChannel_on_LIMITS_STATUS_167(Bounds_Inside * ph, Pack * pack)
{
    assert(p167_last_clear_GET(pack) == (uint32_t)19725738L);
    assert(p167_last_recovery_GET(pack) == (uint32_t)2214965221L);
    assert(p167_last_action_GET(pack) == (uint32_t)2570883566L);
    assert(p167_mods_triggered_GET(pack) == e_LIMIT_MODULE_LIMIT_ALTITUDE);
    assert(p167_mods_enabled_GET(pack) == e_LIMIT_MODULE_LIMIT_GEOFENCE);
    assert(p167_mods_required_GET(pack) == e_LIMIT_MODULE_LIMIT_ALTITUDE);
    assert(p167_limits_state_GET(pack) == e_LIMITS_STATE_LIMITS_DISABLED);
    assert(p167_last_trigger_GET(pack) == (uint32_t)17897427L);
    assert(p167_breach_count_GET(pack) == (uint16_t)(uint16_t)19168);
};


void c_CommunicationChannel_on_WIND_168(Bounds_Inside * ph, Pack * pack)
{
    assert(p168_speed_GET(pack) == (float)7.412861E37F);
    assert(p168_direction_GET(pack) == (float)1.1393186E38F);
    assert(p168_speed_z_GET(pack) == (float)9.038563E37F);
};


void c_CommunicationChannel_on_DATA16_169(Bounds_Inside * ph, Pack * pack)
{
    assert(p169_len_GET(pack) == (uint8_t)(uint8_t)123);
    {
        uint8_t exemplary[] =  {(uint8_t)232, (uint8_t)239, (uint8_t)215, (uint8_t)115, (uint8_t)45, (uint8_t)241, (uint8_t)162, (uint8_t)141, (uint8_t)105, (uint8_t)254, (uint8_t)49, (uint8_t)47, (uint8_t)51, (uint8_t)153, (uint8_t)236, (uint8_t)146} ;
        uint8_t*  sample = p169_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p169_type_GET(pack) == (uint8_t)(uint8_t)105);
};


void c_CommunicationChannel_on_DATA32_170(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)181, (uint8_t)215, (uint8_t)229, (uint8_t)70, (uint8_t)153, (uint8_t)152, (uint8_t)164, (uint8_t)181, (uint8_t)54, (uint8_t)164, (uint8_t)248, (uint8_t)49, (uint8_t)32, (uint8_t)153, (uint8_t)134, (uint8_t)111, (uint8_t)49, (uint8_t)232, (uint8_t)106, (uint8_t)65, (uint8_t)63, (uint8_t)227, (uint8_t)111, (uint8_t)182, (uint8_t)240, (uint8_t)193, (uint8_t)88, (uint8_t)38, (uint8_t)100, (uint8_t)148, (uint8_t)169, (uint8_t)220} ;
        uint8_t*  sample = p170_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p170_type_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p170_len_GET(pack) == (uint8_t)(uint8_t)6);
};


void c_CommunicationChannel_on_DATA64_171(Bounds_Inside * ph, Pack * pack)
{
    assert(p171_type_GET(pack) == (uint8_t)(uint8_t)8);
    {
        uint8_t exemplary[] =  {(uint8_t)214, (uint8_t)85, (uint8_t)98, (uint8_t)0, (uint8_t)141, (uint8_t)20, (uint8_t)113, (uint8_t)213, (uint8_t)101, (uint8_t)22, (uint8_t)232, (uint8_t)23, (uint8_t)76, (uint8_t)125, (uint8_t)60, (uint8_t)246, (uint8_t)51, (uint8_t)0, (uint8_t)255, (uint8_t)109, (uint8_t)117, (uint8_t)63, (uint8_t)16, (uint8_t)254, (uint8_t)109, (uint8_t)120, (uint8_t)220, (uint8_t)248, (uint8_t)123, (uint8_t)61, (uint8_t)178, (uint8_t)94, (uint8_t)198, (uint8_t)198, (uint8_t)174, (uint8_t)242, (uint8_t)100, (uint8_t)143, (uint8_t)199, (uint8_t)113, (uint8_t)162, (uint8_t)68, (uint8_t)71, (uint8_t)21, (uint8_t)9, (uint8_t)79, (uint8_t)11, (uint8_t)229, (uint8_t)17, (uint8_t)152, (uint8_t)2, (uint8_t)220, (uint8_t)68, (uint8_t)114, (uint8_t)245, (uint8_t)20, (uint8_t)190, (uint8_t)1, (uint8_t)194, (uint8_t)28, (uint8_t)199, (uint8_t)51, (uint8_t)30, (uint8_t)177} ;
        uint8_t*  sample = p171_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p171_len_GET(pack) == (uint8_t)(uint8_t)32);
};


void c_CommunicationChannel_on_DATA96_172(Bounds_Inside * ph, Pack * pack)
{
    assert(p172_len_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p172_type_GET(pack) == (uint8_t)(uint8_t)103);
    {
        uint8_t exemplary[] =  {(uint8_t)188, (uint8_t)96, (uint8_t)185, (uint8_t)241, (uint8_t)110, (uint8_t)224, (uint8_t)126, (uint8_t)4, (uint8_t)8, (uint8_t)169, (uint8_t)58, (uint8_t)157, (uint8_t)85, (uint8_t)54, (uint8_t)45, (uint8_t)21, (uint8_t)197, (uint8_t)31, (uint8_t)48, (uint8_t)143, (uint8_t)41, (uint8_t)24, (uint8_t)255, (uint8_t)43, (uint8_t)202, (uint8_t)233, (uint8_t)37, (uint8_t)215, (uint8_t)222, (uint8_t)198, (uint8_t)100, (uint8_t)82, (uint8_t)29, (uint8_t)176, (uint8_t)151, (uint8_t)250, (uint8_t)197, (uint8_t)179, (uint8_t)39, (uint8_t)55, (uint8_t)4, (uint8_t)115, (uint8_t)153, (uint8_t)83, (uint8_t)127, (uint8_t)34, (uint8_t)86, (uint8_t)161, (uint8_t)37, (uint8_t)189, (uint8_t)65, (uint8_t)93, (uint8_t)145, (uint8_t)195, (uint8_t)44, (uint8_t)221, (uint8_t)60, (uint8_t)232, (uint8_t)27, (uint8_t)224, (uint8_t)230, (uint8_t)207, (uint8_t)128, (uint8_t)207, (uint8_t)22, (uint8_t)85, (uint8_t)19, (uint8_t)181, (uint8_t)132, (uint8_t)163, (uint8_t)33, (uint8_t)94, (uint8_t)169, (uint8_t)245, (uint8_t)223, (uint8_t)56, (uint8_t)83, (uint8_t)202, (uint8_t)214, (uint8_t)190, (uint8_t)13, (uint8_t)30, (uint8_t)28, (uint8_t)201, (uint8_t)133, (uint8_t)170, (uint8_t)77, (uint8_t)78, (uint8_t)129, (uint8_t)160, (uint8_t)133, (uint8_t)68, (uint8_t)247, (uint8_t)162, (uint8_t)11, (uint8_t)247} ;
        uint8_t*  sample = p172_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 96);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_RANGEFINDER_173(Bounds_Inside * ph, Pack * pack)
{
    assert(p173_voltage_GET(pack) == (float)1.2420648E38F);
    assert(p173_distance_GET(pack) == (float) -8.847356E37F);
};


void c_CommunicationChannel_on_AIRSPEED_AUTOCAL_174(Bounds_Inside * ph, Pack * pack)
{
    assert(p174_vz_GET(pack) == (float) -1.0121591E38F);
    assert(p174_vy_GET(pack) == (float) -2.8929529E38F);
    assert(p174_vx_GET(pack) == (float)2.1135506E38F);
    assert(p174_state_x_GET(pack) == (float)1.026782E38F);
    assert(p174_ratio_GET(pack) == (float)3.2384214E38F);
    assert(p174_EAS2TAS_GET(pack) == (float)1.5063874E37F);
    assert(p174_state_z_GET(pack) == (float)3.2555922E37F);
    assert(p174_state_y_GET(pack) == (float)3.1902632E38F);
    assert(p174_Pby_GET(pack) == (float) -3.2710322E38F);
    assert(p174_diff_pressure_GET(pack) == (float)1.3840495E38F);
    assert(p174_Pcz_GET(pack) == (float)9.992568E37F);
    assert(p174_Pax_GET(pack) == (float) -2.0524077E38F);
};


void c_CommunicationChannel_on_RALLY_POINT_175(Bounds_Inside * ph, Pack * pack)
{
    assert(p175_idx_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p175_flags_GET(pack) == e_RALLY_FLAGS_FAVORABLE_WIND);
    assert(p175_lng_GET(pack) == (int32_t) -1757734763);
    assert(p175_count_GET(pack) == (uint8_t)(uint8_t)79);
    assert(p175_break_alt_GET(pack) == (int16_t)(int16_t) -24961);
    assert(p175_target_system_GET(pack) == (uint8_t)(uint8_t)171);
    assert(p175_alt_GET(pack) == (int16_t)(int16_t)2485);
    assert(p175_target_component_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p175_lat_GET(pack) == (int32_t) -107140519);
    assert(p175_land_dir_GET(pack) == (uint16_t)(uint16_t)63471);
};


void c_CommunicationChannel_on_RALLY_FETCH_POINT_176(Bounds_Inside * ph, Pack * pack)
{
    assert(p176_target_system_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p176_idx_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p176_target_component_GET(pack) == (uint8_t)(uint8_t)216);
};


void c_CommunicationChannel_on_COMPASSMOT_STATUS_177(Bounds_Inside * ph, Pack * pack)
{
    assert(p177_CompensationY_GET(pack) == (float)1.3884438E38F);
    assert(p177_interference_GET(pack) == (uint16_t)(uint16_t)51091);
    assert(p177_CompensationX_GET(pack) == (float) -7.1497005E37F);
    assert(p177_throttle_GET(pack) == (uint16_t)(uint16_t)12703);
    assert(p177_current_GET(pack) == (float)5.7454463E37F);
    assert(p177_CompensationZ_GET(pack) == (float) -2.8568315E38F);
};


void c_CommunicationChannel_on_AHRS2_178(Bounds_Inside * ph, Pack * pack)
{
    assert(p178_altitude_GET(pack) == (float) -1.0814306E38F);
    assert(p178_roll_GET(pack) == (float)2.9456784E38F);
    assert(p178_pitch_GET(pack) == (float) -8.11421E37F);
    assert(p178_lat_GET(pack) == (int32_t)1059753037);
    assert(p178_yaw_GET(pack) == (float)2.4026428E37F);
    assert(p178_lng_GET(pack) == (int32_t)1481700438);
};


void c_CommunicationChannel_on_CAMERA_STATUS_179(Bounds_Inside * ph, Pack * pack)
{
    assert(p179_target_system_GET(pack) == (uint8_t)(uint8_t)199);
    assert(p179_p3_GET(pack) == (float)9.869209E37F);
    assert(p179_p2_GET(pack) == (float)2.1029585E38F);
    assert(p179_p1_GET(pack) == (float) -2.2114112E38F);
    assert(p179_p4_GET(pack) == (float)2.849375E38F);
    assert(p179_img_idx_GET(pack) == (uint16_t)(uint16_t)34700);
    assert(p179_event_id_GET(pack) == e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_LOWSTORE);
    assert(p179_cam_idx_GET(pack) == (uint8_t)(uint8_t)144);
    assert(p179_time_usec_GET(pack) == (uint64_t)1369099398011732729L);
};


void c_CommunicationChannel_on_CAMERA_FEEDBACK_180(Bounds_Inside * ph, Pack * pack)
{
    assert(p180_alt_msl_GET(pack) == (float) -9.285432E37F);
    assert(p180_img_idx_GET(pack) == (uint16_t)(uint16_t)54263);
    assert(p180_lng_GET(pack) == (int32_t) -753035525);
    assert(p180_alt_rel_GET(pack) == (float)2.4836744E38F);
    assert(p180_pitch_GET(pack) == (float) -1.6304319E37F);
    assert(p180_yaw_GET(pack) == (float) -8.22784E37F);
    assert(p180_lat_GET(pack) == (int32_t)12310009);
    assert(p180_cam_idx_GET(pack) == (uint8_t)(uint8_t)105);
    assert(p180_foc_len_GET(pack) == (float)1.2209146E38F);
    assert(p180_time_usec_GET(pack) == (uint64_t)4126725099677240666L);
    assert(p180_flags_GET(pack) == e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_OPENLOOP);
    assert(p180_roll_GET(pack) == (float)2.8266892E38F);
    assert(p180_target_system_GET(pack) == (uint8_t)(uint8_t)34);
};


void c_CommunicationChannel_on_BATTERY2_181(Bounds_Inside * ph, Pack * pack)
{
    assert(p181_current_battery_GET(pack) == (int16_t)(int16_t) -30170);
    assert(p181_voltage_GET(pack) == (uint16_t)(uint16_t)1763);
};


void c_CommunicationChannel_on_AHRS3_182(Bounds_Inside * ph, Pack * pack)
{
    assert(p182_altitude_GET(pack) == (float) -2.4425108E38F);
    assert(p182_pitch_GET(pack) == (float) -3.4014252E38F);
    assert(p182_roll_GET(pack) == (float) -8.3960946E37F);
    assert(p182_v1_GET(pack) == (float) -2.7842406E38F);
    assert(p182_lat_GET(pack) == (int32_t)2125659037);
    assert(p182_yaw_GET(pack) == (float)1.7979948E38F);
    assert(p182_v2_GET(pack) == (float)1.3667943E38F);
    assert(p182_lng_GET(pack) == (int32_t)433182698);
    assert(p182_v3_GET(pack) == (float)3.0828476E38F);
    assert(p182_v4_GET(pack) == (float)3.447434E36F);
};


void c_CommunicationChannel_on_AUTOPILOT_VERSION_REQUEST_183(Bounds_Inside * ph, Pack * pack)
{
    assert(p183_target_component_GET(pack) == (uint8_t)(uint8_t)106);
    assert(p183_target_system_GET(pack) == (uint8_t)(uint8_t)225);
};


void c_CommunicationChannel_on_REMOTE_LOG_DATA_BLOCK_184(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)27, (uint8_t)97, (uint8_t)11, (uint8_t)129, (uint8_t)42, (uint8_t)156, (uint8_t)84, (uint8_t)36, (uint8_t)245, (uint8_t)221, (uint8_t)245, (uint8_t)176, (uint8_t)115, (uint8_t)38, (uint8_t)46, (uint8_t)20, (uint8_t)165, (uint8_t)231, (uint8_t)19, (uint8_t)78, (uint8_t)46, (uint8_t)116, (uint8_t)202, (uint8_t)23, (uint8_t)43, (uint8_t)64, (uint8_t)3, (uint8_t)32, (uint8_t)112, (uint8_t)130, (uint8_t)7, (uint8_t)148, (uint8_t)214, (uint8_t)71, (uint8_t)66, (uint8_t)119, (uint8_t)230, (uint8_t)200, (uint8_t)60, (uint8_t)39, (uint8_t)61, (uint8_t)179, (uint8_t)205, (uint8_t)126, (uint8_t)56, (uint8_t)238, (uint8_t)38, (uint8_t)102, (uint8_t)18, (uint8_t)74, (uint8_t)150, (uint8_t)231, (uint8_t)85, (uint8_t)247, (uint8_t)165, (uint8_t)47, (uint8_t)115, (uint8_t)243, (uint8_t)134, (uint8_t)172, (uint8_t)143, (uint8_t)7, (uint8_t)217, (uint8_t)140, (uint8_t)172, (uint8_t)198, (uint8_t)122, (uint8_t)82, (uint8_t)3, (uint8_t)115, (uint8_t)59, (uint8_t)2, (uint8_t)198, (uint8_t)2, (uint8_t)132, (uint8_t)49, (uint8_t)149, (uint8_t)1, (uint8_t)89, (uint8_t)92, (uint8_t)63, (uint8_t)10, (uint8_t)128, (uint8_t)101, (uint8_t)151, (uint8_t)0, (uint8_t)208, (uint8_t)192, (uint8_t)21, (uint8_t)189, (uint8_t)18, (uint8_t)148, (uint8_t)70, (uint8_t)181, (uint8_t)138, (uint8_t)153, (uint8_t)13, (uint8_t)111, (uint8_t)14, (uint8_t)69, (uint8_t)99, (uint8_t)215, (uint8_t)155, (uint8_t)30, (uint8_t)68, (uint8_t)163, (uint8_t)202, (uint8_t)175, (uint8_t)123, (uint8_t)183, (uint8_t)144, (uint8_t)176, (uint8_t)150, (uint8_t)107, (uint8_t)78, (uint8_t)150, (uint8_t)148, (uint8_t)228, (uint8_t)239, (uint8_t)3, (uint8_t)25, (uint8_t)20, (uint8_t)142, (uint8_t)152, (uint8_t)160, (uint8_t)63, (uint8_t)134, (uint8_t)205, (uint8_t)240, (uint8_t)216, (uint8_t)28, (uint8_t)91, (uint8_t)214, (uint8_t)132, (uint8_t)39, (uint8_t)55, (uint8_t)7, (uint8_t)202, (uint8_t)162, (uint8_t)134, (uint8_t)63, (uint8_t)99, (uint8_t)226, (uint8_t)116, (uint8_t)99, (uint8_t)138, (uint8_t)60, (uint8_t)110, (uint8_t)107, (uint8_t)83, (uint8_t)145, (uint8_t)40, (uint8_t)185, (uint8_t)168, (uint8_t)98, (uint8_t)92, (uint8_t)230, (uint8_t)94, (uint8_t)77, (uint8_t)228, (uint8_t)12, (uint8_t)228, (uint8_t)110, (uint8_t)246, (uint8_t)222, (uint8_t)179, (uint8_t)197, (uint8_t)24, (uint8_t)252, (uint8_t)190, (uint8_t)7, (uint8_t)155, (uint8_t)205, (uint8_t)53, (uint8_t)194, (uint8_t)173, (uint8_t)183, (uint8_t)228, (uint8_t)53, (uint8_t)128, (uint8_t)69, (uint8_t)198, (uint8_t)5, (uint8_t)76, (uint8_t)238, (uint8_t)245, (uint8_t)227, (uint8_t)47, (uint8_t)172, (uint8_t)134, (uint8_t)6, (uint8_t)209, (uint8_t)246, (uint8_t)167, (uint8_t)222, (uint8_t)22, (uint8_t)244, (uint8_t)238, (uint8_t)94, (uint8_t)150} ;
        uint8_t*  sample = p184_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 200);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p184_target_component_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p184_target_system_GET(pack) == (uint8_t)(uint8_t)136);
    assert(p184_seqno_GET(pack) == e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_STOP);
};


void c_CommunicationChannel_on_REMOTE_LOG_BLOCK_STATUS_185(Bounds_Inside * ph, Pack * pack)
{
    assert(p185_seqno_GET(pack) == (uint32_t)1520469425L);
    assert(p185_target_component_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p185_status_GET(pack) == e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_NACK);
    assert(p185_target_system_GET(pack) == (uint8_t)(uint8_t)163);
};


void c_CommunicationChannel_on_LED_CONTROL_186(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)251, (uint8_t)164, (uint8_t)2, (uint8_t)101, (uint8_t)18, (uint8_t)87, (uint8_t)151, (uint8_t)161, (uint8_t)68, (uint8_t)232, (uint8_t)137, (uint8_t)40, (uint8_t)31, (uint8_t)230, (uint8_t)20, (uint8_t)44, (uint8_t)230, (uint8_t)44, (uint8_t)45, (uint8_t)228, (uint8_t)253, (uint8_t)79, (uint8_t)42, (uint8_t)61} ;
        uint8_t*  sample = p186_custom_bytes_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p186_instance_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p186_pattern_GET(pack) == (uint8_t)(uint8_t)237);
    assert(p186_target_component_GET(pack) == (uint8_t)(uint8_t)120);
    assert(p186_target_system_GET(pack) == (uint8_t)(uint8_t)48);
    assert(p186_custom_len_GET(pack) == (uint8_t)(uint8_t)146);
};


void c_CommunicationChannel_on_MAG_CAL_PROGRESS_191(Bounds_Inside * ph, Pack * pack)
{
    assert(p191_cal_mask_GET(pack) == (uint8_t)(uint8_t)87);
    assert(p191_direction_y_GET(pack) == (float)4.6060967E37F);
    assert(p191_direction_x_GET(pack) == (float)3.1144338E38F);
    assert(p191_attempt_GET(pack) == (uint8_t)(uint8_t)7);
    assert(p191_cal_status_GET(pack) == e_MAG_CAL_STATUS_MAG_CAL_WAITING_TO_START);
    assert(p191_direction_z_GET(pack) == (float)1.612769E38F);
    assert(p191_compass_id_GET(pack) == (uint8_t)(uint8_t)70);
    {
        uint8_t exemplary[] =  {(uint8_t)142, (uint8_t)105, (uint8_t)30, (uint8_t)174, (uint8_t)239, (uint8_t)167, (uint8_t)34, (uint8_t)218, (uint8_t)34, (uint8_t)123} ;
        uint8_t*  sample = p191_completion_mask_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p191_completion_pct_GET(pack) == (uint8_t)(uint8_t)31);
};


void c_CommunicationChannel_on_MAG_CAL_REPORT_192(Bounds_Inside * ph, Pack * pack)
{
    assert(p192_compass_id_GET(pack) == (uint8_t)(uint8_t)30);
    assert(p192_diag_y_GET(pack) == (float) -2.8261203E38F);
    assert(p192_offdiag_y_GET(pack) == (float) -3.1056967E38F);
    assert(p192_diag_x_GET(pack) == (float) -2.5431902E38F);
    assert(p192_ofs_x_GET(pack) == (float)2.572909E38F);
    assert(p192_diag_z_GET(pack) == (float)1.3654549E38F);
    assert(p192_cal_status_GET(pack) == e_MAG_CAL_STATUS_MAG_CAL_NOT_STARTED);
    assert(p192_cal_mask_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p192_offdiag_x_GET(pack) == (float)8.75737E37F);
    assert(p192_ofs_z_GET(pack) == (float) -2.4512328E38F);
    assert(p192_offdiag_z_GET(pack) == (float)1.9967588E38F);
    assert(p192_fitness_GET(pack) == (float) -2.3420279E38F);
    assert(p192_ofs_y_GET(pack) == (float)4.307279E37F);
    assert(p192_autosaved_GET(pack) == (uint8_t)(uint8_t)113);
};


void c_CommunicationChannel_on_EKF_STATUS_REPORT_193(Bounds_Inside * ph, Pack * pack)
{
    assert(p193_flags_GET(pack) == e_EKF_STATUS_FLAGS_EKF_POS_HORIZ_ABS);
    assert(p193_velocity_variance_GET(pack) == (float)1.1670139E38F);
    assert(p193_pos_horiz_variance_GET(pack) == (float) -2.7863382E38F);
    assert(p193_terrain_alt_variance_GET(pack) == (float) -2.7641125E38F);
    assert(p193_pos_vert_variance_GET(pack) == (float)5.0384893E37F);
    assert(p193_compass_variance_GET(pack) == (float)2.6158382E38F);
};


void c_CommunicationChannel_on_PID_TUNING_194(Bounds_Inside * ph, Pack * pack)
{
    assert(p194_axis_GET(pack) == e_PID_TUNING_AXIS_PID_TUNING_LANDING);
    assert(p194_P_GET(pack) == (float)3.0020309E38F);
    assert(p194_I_GET(pack) == (float) -5.6601035E37F);
    assert(p194_FF_GET(pack) == (float)2.6934004E38F);
    assert(p194_desired_GET(pack) == (float)3.2457417E38F);
    assert(p194_D_GET(pack) == (float) -1.4351653E37F);
    assert(p194_achieved_GET(pack) == (float)1.7247403E38F);
};


void c_CommunicationChannel_on_GIMBAL_REPORT_200(Bounds_Inside * ph, Pack * pack)
{
    assert(p200_delta_time_GET(pack) == (float)3.023549E38F);
    assert(p200_delta_velocity_y_GET(pack) == (float)6.886656E37F);
    assert(p200_joint_el_GET(pack) == (float) -2.6706597E38F);
    assert(p200_joint_roll_GET(pack) == (float)2.4386008E38F);
    assert(p200_delta_velocity_z_GET(pack) == (float) -1.3848447E38F);
    assert(p200_joint_az_GET(pack) == (float) -2.0580664E38F);
    assert(p200_target_component_GET(pack) == (uint8_t)(uint8_t)2);
    assert(p200_delta_velocity_x_GET(pack) == (float)3.1297355E37F);
    assert(p200_delta_angle_z_GET(pack) == (float)2.3349217E38F);
    assert(p200_delta_angle_x_GET(pack) == (float) -1.4955731E38F);
    assert(p200_delta_angle_y_GET(pack) == (float)3.2568968E38F);
    assert(p200_target_system_GET(pack) == (uint8_t)(uint8_t)253);
};


void c_CommunicationChannel_on_GIMBAL_CONTROL_201(Bounds_Inside * ph, Pack * pack)
{
    assert(p201_demanded_rate_y_GET(pack) == (float) -2.552407E38F);
    assert(p201_target_system_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p201_demanded_rate_z_GET(pack) == (float)2.2607092E38F);
    assert(p201_demanded_rate_x_GET(pack) == (float) -3.7213757E37F);
    assert(p201_target_component_GET(pack) == (uint8_t)(uint8_t)60);
};


void c_CommunicationChannel_on_GIMBAL_TORQUE_CMD_REPORT_214(Bounds_Inside * ph, Pack * pack)
{
    assert(p214_rl_torque_cmd_GET(pack) == (int16_t)(int16_t)29066);
    assert(p214_az_torque_cmd_GET(pack) == (int16_t)(int16_t)8958);
    assert(p214_target_component_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p214_target_system_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p214_el_torque_cmd_GET(pack) == (int16_t)(int16_t)24124);
};


void c_CommunicationChannel_on_GOPRO_HEARTBEAT_215(Bounds_Inside * ph, Pack * pack)
{
    assert(p215_status_GET(pack) == e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE);
    assert(p215_flags_GET(pack) == e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING);
    assert(p215_capture_mode_GET(pack) == e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_MULTI_SHOT);
};


void c_CommunicationChannel_on_GOPRO_GET_REQUEST_216(Bounds_Inside * ph, Pack * pack)
{
    assert(p216_cmd_id_GET(pack) == e_GOPRO_COMMAND_GOPRO_COMMAND_SHUTTER);
    assert(p216_target_component_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p216_target_system_GET(pack) == (uint8_t)(uint8_t)248);
};


void c_CommunicationChannel_on_GOPRO_GET_RESPONSE_217(Bounds_Inside * ph, Pack * pack)
{
    assert(p217_status_GET(pack) == e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_FAILED);
    {
        uint8_t exemplary[] =  {(uint8_t)174, (uint8_t)61, (uint8_t)83, (uint8_t)162} ;
        uint8_t*  sample = p217_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p217_cmd_id_GET(pack) == e_GOPRO_COMMAND_GOPRO_COMMAND_CHARGING);
};


void c_CommunicationChannel_on_GOPRO_SET_REQUEST_218(Bounds_Inside * ph, Pack * pack)
{
    assert(p218_cmd_id_GET(pack) == e_GOPRO_COMMAND_GOPRO_COMMAND_POWER);
    {
        uint8_t exemplary[] =  {(uint8_t)202, (uint8_t)250, (uint8_t)14, (uint8_t)187} ;
        uint8_t*  sample = p218_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p218_target_system_GET(pack) == (uint8_t)(uint8_t)144);
    assert(p218_target_component_GET(pack) == (uint8_t)(uint8_t)30);
};


void c_CommunicationChannel_on_GOPRO_SET_RESPONSE_219(Bounds_Inside * ph, Pack * pack)
{
    assert(p219_status_GET(pack) == e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_FAILED);
    assert(p219_cmd_id_GET(pack) == e_GOPRO_COMMAND_GOPRO_COMMAND_PHOTO_RESOLUTION);
};


void c_CommunicationChannel_on_RPM_226(Bounds_Inside * ph, Pack * pack)
{
    assert(p226_rpm2_GET(pack) == (float)1.2263879E38F);
    assert(p226_rpm1_GET(pack) == (float) -1.8173366E38F);
};


void c_CommunicationChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_pos_vert_accuracy_GET(pack) == (float)5.470962E37F);
    assert(p230_mag_ratio_GET(pack) == (float) -3.1739142E38F);
    assert(p230_pos_vert_ratio_GET(pack) == (float) -2.5257376E38F);
    assert(p230_vel_ratio_GET(pack) == (float)1.8896472E38F);
    assert(p230_tas_ratio_GET(pack) == (float) -1.4637933E37F);
    assert(p230_time_usec_GET(pack) == (uint64_t)248650046008015872L);
    assert(p230_pos_horiz_ratio_GET(pack) == (float) -3.4164028E37F);
    assert(p230_hagl_ratio_GET(pack) == (float)1.8591025E37F);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float)2.4063223E38F);
    assert(p230_flags_GET(pack) == e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_GPS_GLITCH);
};


void c_CommunicationChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_vert_accuracy_GET(pack) == (float) -3.3188253E38F);
    assert(p231_horiz_accuracy_GET(pack) == (float)3.1302106E38F);
    assert(p231_wind_z_GET(pack) == (float) -2.9977314E38F);
    assert(p231_wind_y_GET(pack) == (float)2.9119435E38F);
    assert(p231_wind_alt_GET(pack) == (float)1.2842868E38F);
    assert(p231_wind_x_GET(pack) == (float) -3.2955754E38F);
    assert(p231_var_horiz_GET(pack) == (float) -2.600833E38F);
    assert(p231_time_usec_GET(pack) == (uint64_t)9082765025820744926L);
    assert(p231_var_vert_GET(pack) == (float) -2.858627E38F);
};


void c_CommunicationChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_speed_accuracy_GET(pack) == (float)3.2961258E38F);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p232_horiz_accuracy_GET(pack) == (float) -5.6489406E37F);
    assert(p232_vdop_GET(pack) == (float) -2.2505258E38F);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)2222818859L);
    assert(p232_ignore_flags_GET(pack) == e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p232_vert_accuracy_GET(pack) == (float)1.1618179E38F);
    assert(p232_hdop_GET(pack) == (float) -3.1430916E38F);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)182);
    assert(p232_alt_GET(pack) == (float)3.1723052E38F);
    assert(p232_vn_GET(pack) == (float)2.2971161E38F);
    assert(p232_lat_GET(pack) == (int32_t) -1421815011);
    assert(p232_ve_GET(pack) == (float) -2.4268998E38F);
    assert(p232_lon_GET(pack) == (int32_t) -1988797236);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)2929);
    assert(p232_time_usec_GET(pack) == (uint64_t)2071795378945817606L);
    assert(p232_vd_GET(pack) == (float)3.189505E38F);
};


void c_CommunicationChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)95, (uint8_t)208, (uint8_t)176, (uint8_t)156, (uint8_t)161, (uint8_t)63, (uint8_t)210, (uint8_t)19, (uint8_t)47, (uint8_t)35, (uint8_t)29, (uint8_t)96, (uint8_t)157, (uint8_t)115, (uint8_t)107, (uint8_t)89, (uint8_t)179, (uint8_t)81, (uint8_t)191, (uint8_t)214, (uint8_t)159, (uint8_t)43, (uint8_t)120, (uint8_t)60, (uint8_t)5, (uint8_t)183, (uint8_t)216, (uint8_t)162, (uint8_t)5, (uint8_t)212, (uint8_t)113, (uint8_t)15, (uint8_t)73, (uint8_t)130, (uint8_t)44, (uint8_t)53, (uint8_t)232, (uint8_t)176, (uint8_t)249, (uint8_t)84, (uint8_t)208, (uint8_t)211, (uint8_t)28, (uint8_t)133, (uint8_t)129, (uint8_t)109, (uint8_t)100, (uint8_t)110, (uint8_t)211, (uint8_t)231, (uint8_t)234, (uint8_t)72, (uint8_t)160, (uint8_t)172, (uint8_t)117, (uint8_t)63, (uint8_t)244, (uint8_t)0, (uint8_t)211, (uint8_t)1, (uint8_t)59, (uint8_t)80, (uint8_t)20, (uint8_t)71, (uint8_t)83, (uint8_t)36, (uint8_t)33, (uint8_t)25, (uint8_t)150, (uint8_t)119, (uint8_t)140, (uint8_t)160, (uint8_t)187, (uint8_t)74, (uint8_t)251, (uint8_t)160, (uint8_t)73, (uint8_t)42, (uint8_t)162, (uint8_t)52, (uint8_t)58, (uint8_t)11, (uint8_t)232, (uint8_t)130, (uint8_t)158, (uint8_t)17, (uint8_t)155, (uint8_t)248, (uint8_t)160, (uint8_t)166, (uint8_t)37, (uint8_t)26, (uint8_t)207, (uint8_t)227, (uint8_t)244, (uint8_t)56, (uint8_t)137, (uint8_t)182, (uint8_t)92, (uint8_t)95, (uint8_t)55, (uint8_t)188, (uint8_t)96, (uint8_t)223, (uint8_t)170, (uint8_t)158, (uint8_t)238, (uint8_t)225, (uint8_t)74, (uint8_t)54, (uint8_t)24, (uint8_t)223, (uint8_t)2, (uint8_t)48, (uint8_t)58, (uint8_t)250, (uint8_t)219, (uint8_t)109, (uint8_t)99, (uint8_t)163, (uint8_t)32, (uint8_t)105, (uint8_t)203, (uint8_t)72, (uint8_t)77, (uint8_t)183, (uint8_t)8, (uint8_t)182, (uint8_t)41, (uint8_t)222, (uint8_t)109, (uint8_t)58, (uint8_t)150, (uint8_t)107, (uint8_t)226, (uint8_t)193, (uint8_t)166, (uint8_t)157, (uint8_t)12, (uint8_t)78, (uint8_t)194, (uint8_t)101, (uint8_t)7, (uint8_t)107, (uint8_t)47, (uint8_t)253, (uint8_t)194, (uint8_t)85, (uint8_t)113, (uint8_t)110, (uint8_t)80, (uint8_t)101, (uint8_t)170, (uint8_t)8, (uint8_t)22, (uint8_t)101, (uint8_t)238, (uint8_t)139, (uint8_t)129, (uint8_t)188, (uint8_t)183, (uint8_t)39, (uint8_t)14, (uint8_t)125, (uint8_t)142, (uint8_t)152, (uint8_t)189, (uint8_t)164, (uint8_t)236, (uint8_t)27, (uint8_t)217, (uint8_t)69, (uint8_t)209, (uint8_t)179, (uint8_t)82, (uint8_t)97, (uint8_t)124, (uint8_t)254, (uint8_t)138, (uint8_t)97} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)125);
};


void c_CommunicationChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_roll_GET(pack) == (int16_t)(int16_t)21413);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)27121);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)20);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t)97);
    assert(p234_custom_mode_GET(pack) == (uint32_t)4282768129L);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t)18011);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)26);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)222);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t)28043);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)201);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t)96);
    assert(p234_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED);
    assert(p234_longitude_GET(pack) == (int32_t)690522);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)225);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t)20206);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t)27);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t)10157);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t)93);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP);
    assert(p234_latitude_GET(pack) == (int32_t)1393651403);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)54975);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)246);
};


void c_CommunicationChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_time_usec_GET(pack) == (uint64_t)1217626708836182004L);
    assert(p241_clipping_2_GET(pack) == (uint32_t)69703481L);
    assert(p241_clipping_0_GET(pack) == (uint32_t)448690303L);
    assert(p241_vibration_x_GET(pack) == (float)1.5120582E38F);
    assert(p241_clipping_1_GET(pack) == (uint32_t)3864651756L);
    assert(p241_vibration_z_GET(pack) == (float)9.893109E37F);
    assert(p241_vibration_y_GET(pack) == (float) -3.2061835E38F);
};


void c_CommunicationChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_z_GET(pack) == (float) -7.97693E37F);
    assert(p242_approach_z_GET(pack) == (float)2.0349473E38F);
    assert(p242_time_usec_TRY(ph) == (uint64_t)6357227412629425417L);
    assert(p242_latitude_GET(pack) == (int32_t) -125019900);
    assert(p242_approach_x_GET(pack) == (float) -1.5601182E38F);
    {
        float exemplary[] =  {-2.123009E38F, 9.758344E36F, -1.573103E38F, 2.5156244E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_longitude_GET(pack) == (int32_t)1181089546);
    assert(p242_y_GET(pack) == (float) -5.7388546E37F);
    assert(p242_altitude_GET(pack) == (int32_t) -531433883);
    assert(p242_x_GET(pack) == (float)2.6093026E38F);
    assert(p242_approach_y_GET(pack) == (float)1.1335036E38F);
};


void c_CommunicationChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_longitude_GET(pack) == (int32_t)1859959094);
    assert(p243_x_GET(pack) == (float)3.2505253E38F);
    assert(p243_y_GET(pack) == (float) -1.3587373E38F);
    assert(p243_altitude_GET(pack) == (int32_t)665446740);
    assert(p243_approach_z_GET(pack) == (float)3.2951062E38F);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p243_approach_x_GET(pack) == (float)1.6687696E38F);
    {
        float exemplary[] =  {1.4521999E38F, -9.049117E37F, 2.6868092E38F, 4.927494E37F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_approach_y_GET(pack) == (float)3.1046357E38F);
    assert(p243_latitude_GET(pack) == (int32_t)2046327104);
    assert(p243_z_GET(pack) == (float) -2.5839333E38F);
    assert(p243_time_usec_TRY(ph) == (uint64_t)611136243249599342L);
};


void c_CommunicationChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_interval_us_GET(pack) == (int32_t)1822755925);
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)49844);
};


void c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED);
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF);
};


void c_CommunicationChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)32767);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UNASSGINED3);
    assert(p246_lat_GET(pack) == (int32_t) -1631143221);
    assert(p246_callsign_LEN(ph) == 8);
    {
        char16_t * exemplary = u"hvmrxonf";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)62402);
    assert(p246_lon_GET(pack) == (int32_t)926353628);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t) -57);
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)13);
    assert(p246_altitude_GET(pack) == (int32_t) -92761215);
    assert(p246_flags_GET(pack) == e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)2949266623L);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)15028);
};


void c_CommunicationChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_PERPENDICULAR);
    assert(p247_id_GET(pack) == (uint32_t)1363240111L);
    assert(p247_altitude_minimum_delta_GET(pack) == (float) -9.217915E37F);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float)7.7806923E37F);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
    assert(p247_time_to_minimum_delta_GET(pack) == (float)1.2336657E38F);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW);
};


void c_CommunicationChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)59);
    {
        uint8_t exemplary[] =  {(uint8_t)125, (uint8_t)126, (uint8_t)37, (uint8_t)24, (uint8_t)54, (uint8_t)221, (uint8_t)66, (uint8_t)96, (uint8_t)188, (uint8_t)174, (uint8_t)33, (uint8_t)187, (uint8_t)231, (uint8_t)62, (uint8_t)113, (uint8_t)117, (uint8_t)185, (uint8_t)143, (uint8_t)18, (uint8_t)241, (uint8_t)118, (uint8_t)214, (uint8_t)110, (uint8_t)247, (uint8_t)152, (uint8_t)218, (uint8_t)159, (uint8_t)15, (uint8_t)33, (uint8_t)185, (uint8_t)207, (uint8_t)51, (uint8_t)156, (uint8_t)89, (uint8_t)153, (uint8_t)92, (uint8_t)110, (uint8_t)141, (uint8_t)253, (uint8_t)84, (uint8_t)189, (uint8_t)186, (uint8_t)81, (uint8_t)224, (uint8_t)120, (uint8_t)129, (uint8_t)87, (uint8_t)47, (uint8_t)8, (uint8_t)24, (uint8_t)131, (uint8_t)44, (uint8_t)200, (uint8_t)160, (uint8_t)114, (uint8_t)31, (uint8_t)143, (uint8_t)116, (uint8_t)14, (uint8_t)235, (uint8_t)104, (uint8_t)198, (uint8_t)190, (uint8_t)1, (uint8_t)56, (uint8_t)249, (uint8_t)42, (uint8_t)189, (uint8_t)178, (uint8_t)197, (uint8_t)96, (uint8_t)147, (uint8_t)75, (uint8_t)213, (uint8_t)77, (uint8_t)80, (uint8_t)185, (uint8_t)228, (uint8_t)128, (uint8_t)24, (uint8_t)30, (uint8_t)205, (uint8_t)228, (uint8_t)70, (uint8_t)176, (uint8_t)56, (uint8_t)62, (uint8_t)111, (uint8_t)36, (uint8_t)101, (uint8_t)217, (uint8_t)91, (uint8_t)210, (uint8_t)6, (uint8_t)245, (uint8_t)18, (uint8_t)138, (uint8_t)226, (uint8_t)159, (uint8_t)112, (uint8_t)200, (uint8_t)232, (uint8_t)45, (uint8_t)134, (uint8_t)250, (uint8_t)192, (uint8_t)125, (uint8_t)27, (uint8_t)199, (uint8_t)23, (uint8_t)239, (uint8_t)244, (uint8_t)70, (uint8_t)204, (uint8_t)173, (uint8_t)245, (uint8_t)191, (uint8_t)55, (uint8_t)75, (uint8_t)57, (uint8_t)109, (uint8_t)216, (uint8_t)201, (uint8_t)34, (uint8_t)215, (uint8_t)175, (uint8_t)36, (uint8_t)18, (uint8_t)148, (uint8_t)125, (uint8_t)76, (uint8_t)207, (uint8_t)198, (uint8_t)132, (uint8_t)61, (uint8_t)215, (uint8_t)50, (uint8_t)14, (uint8_t)86, (uint8_t)13, (uint8_t)134, (uint8_t)235, (uint8_t)10, (uint8_t)107, (uint8_t)108, (uint8_t)9, (uint8_t)112, (uint8_t)239, (uint8_t)92, (uint8_t)116, (uint8_t)252, (uint8_t)251, (uint8_t)34, (uint8_t)15, (uint8_t)99, (uint8_t)215, (uint8_t)78, (uint8_t)28, (uint8_t)30, (uint8_t)46, (uint8_t)174, (uint8_t)64, (uint8_t)60, (uint8_t)150, (uint8_t)117, (uint8_t)94, (uint8_t)128, (uint8_t)121, (uint8_t)203, (uint8_t)95, (uint8_t)45, (uint8_t)167, (uint8_t)95, (uint8_t)163, (uint8_t)198, (uint8_t)212, (uint8_t)11, (uint8_t)44, (uint8_t)157, (uint8_t)107, (uint8_t)223, (uint8_t)43, (uint8_t)180, (uint8_t)190, (uint8_t)173, (uint8_t)167, (uint8_t)237, (uint8_t)126, (uint8_t)81, (uint8_t)176, (uint8_t)65, (uint8_t)143, (uint8_t)109, (uint8_t)39, (uint8_t)96, (uint8_t)45, (uint8_t)66, (uint8_t)176, (uint8_t)25, (uint8_t)188, (uint8_t)113, (uint8_t)143, (uint8_t)106, (uint8_t)157, (uint8_t)189, (uint8_t)137, (uint8_t)91, (uint8_t)2, (uint8_t)107, (uint8_t)55, (uint8_t)113, (uint8_t)220, (uint8_t)177, (uint8_t)57, (uint8_t)249, (uint8_t)1, (uint8_t)73, (uint8_t)182, (uint8_t)172, (uint8_t)95, (uint8_t)45, (uint8_t)55, (uint8_t)169, (uint8_t)4, (uint8_t)119, (uint8_t)139, (uint8_t)165, (uint8_t)94, (uint8_t)28, (uint8_t)108, (uint8_t)62, (uint8_t)206, (uint8_t)163, (uint8_t)194, (uint8_t)159, (uint8_t)190, (uint8_t)90, (uint8_t)50, (uint8_t)157, (uint8_t)166, (uint8_t)3, (uint8_t)163, (uint8_t)208, (uint8_t)164, (uint8_t)105, (uint8_t)168, (uint8_t)5, (uint8_t)152, (uint8_t)240} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)3477);
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)251);
};


void c_CommunicationChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)142);
    {
        int8_t exemplary[] =  {(int8_t) -116, (int8_t) -61, (int8_t)100, (int8_t) -4, (int8_t) -9, (int8_t)116, (int8_t) -74, (int8_t) -30, (int8_t)80, (int8_t) -117, (int8_t)56, (int8_t) -49, (int8_t)45, (int8_t) -116, (int8_t)39, (int8_t) -92, (int8_t)95, (int8_t)102, (int8_t)88, (int8_t) -34, (int8_t)28, (int8_t) -95, (int8_t) -55, (int8_t)126, (int8_t)23, (int8_t) -119, (int8_t)32, (int8_t) -3, (int8_t)99, (int8_t)22, (int8_t)49, (int8_t)47} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)32707);
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)144);
};


void c_CommunicationChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_x_GET(pack) == (float)3.2455023E38F);
    assert(p250_y_GET(pack) == (float) -1.5546891E38F);
    assert(p250_name_LEN(ph) == 8);
    {
        char16_t * exemplary = u"uaavcsuy";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_time_usec_GET(pack) == (uint64_t)2637063938511420752L);
    assert(p250_z_GET(pack) == (float) -2.535671E38F);
};


void c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_name_LEN(ph) == 5);
    {
        char16_t * exemplary = u"hpsmK";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)1316278535L);
    assert(p251_value_GET(pack) == (float) -3.0028637E38F);
};


void c_CommunicationChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)2398153666L);
    assert(p252_value_GET(pack) == (int32_t)787673637);
    assert(p252_name_LEN(ph) == 2);
    {
        char16_t * exemplary = u"zn";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_DEBUG);
    assert(p253_text_LEN(ph) == 24);
    {
        char16_t * exemplary = u"ySfbczrkfvisrdUruhltvpsf";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 48);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p254_value_GET(pack) == (float)3.3159085E38F);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)1784053255L);
};


void c_CommunicationChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)156);
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)700822186448103909L);
    {
        uint8_t exemplary[] =  {(uint8_t)136, (uint8_t)227, (uint8_t)10, (uint8_t)81, (uint8_t)154, (uint8_t)238, (uint8_t)218, (uint8_t)55, (uint8_t)202, (uint8_t)57, (uint8_t)227, (uint8_t)137, (uint8_t)97, (uint8_t)17, (uint8_t)94, (uint8_t)179, (uint8_t)177, (uint8_t)139, (uint8_t)113, (uint8_t)188, (uint8_t)2, (uint8_t)164, (uint8_t)70, (uint8_t)62, (uint8_t)143, (uint8_t)240, (uint8_t)56, (uint8_t)4, (uint8_t)160, (uint8_t)223, (uint8_t)242, (uint8_t)14} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)14);
};


void c_CommunicationChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)232);
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)3729506112L);
    assert(p257_last_change_ms_GET(pack) == (uint32_t)4103053109L);
};


void c_CommunicationChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)69);
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p258_tune_LEN(ph) == 25);
    {
        char16_t * exemplary = u"YCuLnnvdmxzhaqmhkauynyrku";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 50);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_focal_length_GET(pack) == (float) -2.5371747E38F);
    assert(p259_sensor_size_h_GET(pack) == (float) -1.2356487E38F);
    {
        uint8_t exemplary[] =  {(uint8_t)255, (uint8_t)152, (uint8_t)37, (uint8_t)47, (uint8_t)10, (uint8_t)78, (uint8_t)236, (uint8_t)159, (uint8_t)53, (uint8_t)199, (uint8_t)198, (uint8_t)59, (uint8_t)175, (uint8_t)99, (uint8_t)171, (uint8_t)145, (uint8_t)168, (uint8_t)246, (uint8_t)177, (uint8_t)35, (uint8_t)84, (uint8_t)246, (uint8_t)20, (uint8_t)157, (uint8_t)230, (uint8_t)136, (uint8_t)48, (uint8_t)199, (uint8_t)170, (uint8_t)94, (uint8_t)251, (uint8_t)96} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)56058);
    assert(p259_sensor_size_v_GET(pack) == (float) -1.7008044E38F);
    assert(p259_firmware_version_GET(pack) == (uint32_t)3849110563L);
    assert(p259_flags_GET(pack) == e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
    {
        uint8_t exemplary[] =  {(uint8_t)40, (uint8_t)78, (uint8_t)77, (uint8_t)142, (uint8_t)196, (uint8_t)225, (uint8_t)162, (uint8_t)199, (uint8_t)122, (uint8_t)192, (uint8_t)70, (uint8_t)67, (uint8_t)252, (uint8_t)26, (uint8_t)17, (uint8_t)103, (uint8_t)97, (uint8_t)199, (uint8_t)64, (uint8_t)211, (uint8_t)11, (uint8_t)73, (uint8_t)149, (uint8_t)86, (uint8_t)238, (uint8_t)241, (uint8_t)246, (uint8_t)187, (uint8_t)201, (uint8_t)13, (uint8_t)28, (uint8_t)231} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)34467);
    assert(p259_cam_definition_uri_LEN(ph) == 15);
    {
        char16_t * exemplary = u"vxqawvdYsXwllDc";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)11178);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)3963523674L);
};


void c_CommunicationChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)3067896154L);
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_VIDEO);
};


void c_CommunicationChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p261_write_speed_GET(pack) == (float)2.6695673E38F);
    assert(p261_used_capacity_GET(pack) == (float)2.2025539E38F);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)142);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)43);
    assert(p261_read_speed_GET(pack) == (float)8.387959E37F);
    assert(p261_total_capacity_GET(pack) == (float) -3.0558584E38F);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)1028373128L);
    assert(p261_available_capacity_GET(pack) == (float) -2.8826334E38F);
};


void c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_image_interval_GET(pack) == (float) -1.669174E36F);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)907083870L);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)215);
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)2119954947L);
    assert(p262_available_capacity_GET(pack) == (float)1.4725072E38F);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)111);
};


void c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_relative_alt_GET(pack) == (int32_t) -511502216);
    assert(p263_alt_GET(pack) == (int32_t) -947138213);
    assert(p263_lon_GET(pack) == (int32_t)1022031940);
    assert(p263_image_index_GET(pack) == (int32_t)1778403594);
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)3787108044L);
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t) -16);
    assert(p263_file_url_LEN(ph) == 61);
    {
        char16_t * exemplary = u"utsqqmzWhmlFhotMuqkewzeynRjgdmohbmdrvfroWsrnjevnEmyagpbjvxkkx";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 122);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {3.3734077E38F, 1.394257E38F, 1.416001E38F, 9.938413E36F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p263_time_utc_GET(pack) == (uint64_t)2661778188252590438L);
    assert(p263_lat_GET(pack) == (int32_t)1180071505);
};


void c_CommunicationChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)2996265288L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)5711009840114461792L);
    assert(p264_flight_uuid_GET(pack) == (uint64_t)8180957634720839837L);
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)8238585564253823638L);
};


void c_CommunicationChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_yaw_GET(pack) == (float)2.5878223E38F);
    assert(p265_roll_GET(pack) == (float)1.7065788E38F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)3446844325L);
    assert(p265_pitch_GET(pack) == (float)1.1720304E38F);
};


void c_CommunicationChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)11, (uint8_t)153, (uint8_t)199, (uint8_t)175, (uint8_t)156, (uint8_t)230, (uint8_t)35, (uint8_t)238, (uint8_t)9, (uint8_t)205, (uint8_t)197, (uint8_t)156, (uint8_t)120, (uint8_t)155, (uint8_t)229, (uint8_t)2, (uint8_t)155, (uint8_t)14, (uint8_t)34, (uint8_t)13, (uint8_t)207, (uint8_t)184, (uint8_t)177, (uint8_t)224, (uint8_t)235, (uint8_t)105, (uint8_t)52, (uint8_t)116, (uint8_t)119, (uint8_t)53, (uint8_t)37, (uint8_t)32, (uint8_t)243, (uint8_t)65, (uint8_t)197, (uint8_t)193, (uint8_t)24, (uint8_t)164, (uint8_t)21, (uint8_t)17, (uint8_t)52, (uint8_t)8, (uint8_t)162, (uint8_t)12, (uint8_t)152, (uint8_t)38, (uint8_t)120, (uint8_t)19, (uint8_t)62, (uint8_t)77, (uint8_t)203, (uint8_t)77, (uint8_t)185, (uint8_t)166, (uint8_t)32, (uint8_t)136, (uint8_t)205, (uint8_t)189, (uint8_t)38, (uint8_t)68, (uint8_t)23, (uint8_t)241, (uint8_t)218, (uint8_t)40, (uint8_t)220, (uint8_t)50, (uint8_t)157, (uint8_t)181, (uint8_t)94, (uint8_t)235, (uint8_t)244, (uint8_t)160, (uint8_t)151, (uint8_t)1, (uint8_t)11, (uint8_t)74, (uint8_t)42, (uint8_t)58, (uint8_t)183, (uint8_t)141, (uint8_t)237, (uint8_t)92, (uint8_t)150, (uint8_t)33, (uint8_t)240, (uint8_t)120, (uint8_t)104, (uint8_t)170, (uint8_t)148, (uint8_t)168, (uint8_t)195, (uint8_t)82, (uint8_t)175, (uint8_t)243, (uint8_t)116, (uint8_t)18, (uint8_t)66, (uint8_t)7, (uint8_t)73, (uint8_t)78, (uint8_t)225, (uint8_t)7, (uint8_t)83, (uint8_t)1, (uint8_t)244, (uint8_t)246, (uint8_t)142, (uint8_t)228, (uint8_t)220, (uint8_t)179, (uint8_t)246, (uint8_t)125, (uint8_t)166, (uint8_t)12, (uint8_t)121, (uint8_t)26, (uint8_t)46, (uint8_t)154, (uint8_t)97, (uint8_t)170, (uint8_t)92, (uint8_t)156, (uint8_t)223, (uint8_t)169, (uint8_t)136, (uint8_t)30, (uint8_t)184, (uint8_t)137, (uint8_t)32, (uint8_t)142, (uint8_t)236, (uint8_t)107, (uint8_t)98, (uint8_t)37, (uint8_t)177, (uint8_t)73, (uint8_t)126, (uint8_t)180, (uint8_t)9, (uint8_t)101, (uint8_t)93, (uint8_t)43, (uint8_t)126, (uint8_t)22, (uint8_t)215, (uint8_t)164, (uint8_t)230, (uint8_t)236, (uint8_t)224, (uint8_t)156, (uint8_t)135, (uint8_t)140, (uint8_t)52, (uint8_t)139, (uint8_t)66, (uint8_t)230, (uint8_t)98, (uint8_t)202, (uint8_t)142, (uint8_t)37, (uint8_t)74, (uint8_t)120, (uint8_t)185, (uint8_t)222, (uint8_t)60, (uint8_t)21, (uint8_t)172, (uint8_t)7, (uint8_t)101, (uint8_t)158, (uint8_t)241, (uint8_t)128, (uint8_t)253, (uint8_t)226, (uint8_t)225, (uint8_t)140, (uint8_t)32, (uint8_t)41, (uint8_t)51, (uint8_t)224, (uint8_t)141, (uint8_t)19, (uint8_t)226, (uint8_t)35, (uint8_t)68, (uint8_t)122, (uint8_t)128, (uint8_t)8, (uint8_t)176, (uint8_t)129, (uint8_t)101, (uint8_t)188, (uint8_t)134, (uint8_t)208, (uint8_t)0, (uint8_t)28, (uint8_t)188, (uint8_t)78, (uint8_t)95, (uint8_t)119, (uint8_t)155, (uint8_t)14, (uint8_t)75, (uint8_t)22, (uint8_t)255, (uint8_t)236, (uint8_t)216, (uint8_t)92, (uint8_t)253, (uint8_t)59, (uint8_t)76, (uint8_t)64, (uint8_t)225, (uint8_t)71, (uint8_t)63, (uint8_t)237, (uint8_t)105, (uint8_t)250, (uint8_t)65, (uint8_t)184, (uint8_t)145, (uint8_t)91, (uint8_t)136, (uint8_t)30, (uint8_t)116, (uint8_t)102, (uint8_t)0, (uint8_t)194, (uint8_t)86, (uint8_t)115, (uint8_t)198, (uint8_t)207, (uint8_t)132, (uint8_t)129, (uint8_t)22, (uint8_t)102, (uint8_t)220, (uint8_t)149, (uint8_t)141, (uint8_t)57, (uint8_t)130, (uint8_t)176, (uint8_t)193, (uint8_t)71, (uint8_t)228, (uint8_t)237, (uint8_t)33, (uint8_t)216, (uint8_t)158} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)36);
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)178);
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)56191);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)8);
};


void c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)191, (uint8_t)39, (uint8_t)195, (uint8_t)120, (uint8_t)116, (uint8_t)47, (uint8_t)66, (uint8_t)52, (uint8_t)141, (uint8_t)33, (uint8_t)83, (uint8_t)181, (uint8_t)172, (uint8_t)177, (uint8_t)52, (uint8_t)155, (uint8_t)149, (uint8_t)225, (uint8_t)84, (uint8_t)148, (uint8_t)175, (uint8_t)225, (uint8_t)78, (uint8_t)51, (uint8_t)32, (uint8_t)208, (uint8_t)117, (uint8_t)236, (uint8_t)78, (uint8_t)68, (uint8_t)207, (uint8_t)218, (uint8_t)227, (uint8_t)24, (uint8_t)15, (uint8_t)37, (uint8_t)121, (uint8_t)116, (uint8_t)63, (uint8_t)220, (uint8_t)142, (uint8_t)145, (uint8_t)67, (uint8_t)215, (uint8_t)148, (uint8_t)153, (uint8_t)42, (uint8_t)207, (uint8_t)26, (uint8_t)213, (uint8_t)197, (uint8_t)227, (uint8_t)202, (uint8_t)15, (uint8_t)75, (uint8_t)29, (uint8_t)18, (uint8_t)244, (uint8_t)40, (uint8_t)149, (uint8_t)240, (uint8_t)163, (uint8_t)54, (uint8_t)69, (uint8_t)5, (uint8_t)42, (uint8_t)20, (uint8_t)154, (uint8_t)196, (uint8_t)89, (uint8_t)83, (uint8_t)134, (uint8_t)158, (uint8_t)215, (uint8_t)160, (uint8_t)143, (uint8_t)42, (uint8_t)42, (uint8_t)197, (uint8_t)57, (uint8_t)45, (uint8_t)51, (uint8_t)101, (uint8_t)105, (uint8_t)75, (uint8_t)17, (uint8_t)20, (uint8_t)20, (uint8_t)162, (uint8_t)233, (uint8_t)107, (uint8_t)10, (uint8_t)169, (uint8_t)95, (uint8_t)117, (uint8_t)226, (uint8_t)167, (uint8_t)50, (uint8_t)150, (uint8_t)155, (uint8_t)250, (uint8_t)6, (uint8_t)57, (uint8_t)149, (uint8_t)197, (uint8_t)94, (uint8_t)178, (uint8_t)125, (uint8_t)54, (uint8_t)238, (uint8_t)84, (uint8_t)18, (uint8_t)163, (uint8_t)255, (uint8_t)18, (uint8_t)150, (uint8_t)62, (uint8_t)151, (uint8_t)243, (uint8_t)231, (uint8_t)204, (uint8_t)2, (uint8_t)96, (uint8_t)237, (uint8_t)131, (uint8_t)130, (uint8_t)127, (uint8_t)218, (uint8_t)181, (uint8_t)210, (uint8_t)143, (uint8_t)236, (uint8_t)114, (uint8_t)166, (uint8_t)213, (uint8_t)17, (uint8_t)174, (uint8_t)17, (uint8_t)29, (uint8_t)74, (uint8_t)77, (uint8_t)148, (uint8_t)101, (uint8_t)92, (uint8_t)20, (uint8_t)153, (uint8_t)255, (uint8_t)87, (uint8_t)57, (uint8_t)93, (uint8_t)110, (uint8_t)54, (uint8_t)31, (uint8_t)80, (uint8_t)20, (uint8_t)230, (uint8_t)104, (uint8_t)146, (uint8_t)21, (uint8_t)87, (uint8_t)58, (uint8_t)129, (uint8_t)208, (uint8_t)135, (uint8_t)150, (uint8_t)250, (uint8_t)192, (uint8_t)29, (uint8_t)244, (uint8_t)206, (uint8_t)59, (uint8_t)154, (uint8_t)31, (uint8_t)106, (uint8_t)207, (uint8_t)14, (uint8_t)185, (uint8_t)128, (uint8_t)121, (uint8_t)210, (uint8_t)104, (uint8_t)189, (uint8_t)42, (uint8_t)122, (uint8_t)108, (uint8_t)232, (uint8_t)117, (uint8_t)154, (uint8_t)177, (uint8_t)147, (uint8_t)215, (uint8_t)25, (uint8_t)211, (uint8_t)208, (uint8_t)2, (uint8_t)176, (uint8_t)172, (uint8_t)42, (uint8_t)48, (uint8_t)241, (uint8_t)31, (uint8_t)155, (uint8_t)243, (uint8_t)249, (uint8_t)242, (uint8_t)37, (uint8_t)212, (uint8_t)229, (uint8_t)198, (uint8_t)85, (uint8_t)214, (uint8_t)214, (uint8_t)5, (uint8_t)178, (uint8_t)87, (uint8_t)41, (uint8_t)32, (uint8_t)45, (uint8_t)118, (uint8_t)70, (uint8_t)3, (uint8_t)199, (uint8_t)36, (uint8_t)171, (uint8_t)195, (uint8_t)236, (uint8_t)236, (uint8_t)179, (uint8_t)67, (uint8_t)230, (uint8_t)229, (uint8_t)11, (uint8_t)13, (uint8_t)137, (uint8_t)208, (uint8_t)52, (uint8_t)167, (uint8_t)181, (uint8_t)212, (uint8_t)147, (uint8_t)31, (uint8_t)165, (uint8_t)40, (uint8_t)243, (uint8_t)46, (uint8_t)49, (uint8_t)127, (uint8_t)184, (uint8_t)169} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)32748);
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)138);
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)103);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)205);
};


void c_CommunicationChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)45);
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)34556);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)71);
};


void c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_bitrate_GET(pack) == (uint32_t)919304751L);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)51);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)36);
    assert(p269_uri_LEN(ph) == 32);
    {
        char16_t * exemplary = u"pxtnhcgxvzkvoyAbwUqvkgiqckyqijgc";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)60441);
    assert(p269_framerate_GET(pack) == (float)1.85183E38F);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)58075);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)7857);
};


void c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)52652);
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)146);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)146);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)62147);
    assert(p270_bitrate_GET(pack) == (uint32_t)3926114129L);
    assert(p270_uri_LEN(ph) == 140);
    {
        char16_t * exemplary = u"dlcegzbjidfsswlrjklbfqxqdNporiVcvvyDkiuqzsurkvunxigqhxuidbvfwslhxphhudAegbjchoncebrkbemkpihbvpsfjddrwottmczuefufcuVjhzyiuqugyycirhkVwhgvgfsz";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 280);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)27001);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p270_framerate_GET(pack) == (float) -3.4455615E37F);
};


void c_CommunicationChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_ssid_LEN(ph) == 21);
    {
        char16_t * exemplary = u"anmktGmBHetgfltzhnzbo";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 42);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_password_LEN(ph) == 3);
    {
        char16_t * exemplary = u"pih";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)54154);
    {
        uint8_t exemplary[] =  {(uint8_t)26, (uint8_t)205, (uint8_t)252, (uint8_t)19, (uint8_t)214, (uint8_t)183, (uint8_t)233, (uint8_t)103} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)60, (uint8_t)67, (uint8_t)209, (uint8_t)253, (uint8_t)41, (uint8_t)114, (uint8_t)178, (uint8_t)57} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)46890);
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)64535);
};


void c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_uptime_sec_GET(pack) == (uint32_t)1654715427L);
    assert(p310_time_usec_GET(pack) == (uint64_t)6366633922272232498L);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)144);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)51516);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING);
};


void c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)31, (uint8_t)34, (uint8_t)254, (uint8_t)253, (uint8_t)99, (uint8_t)173, (uint8_t)101, (uint8_t)123, (uint8_t)193, (uint8_t)128, (uint8_t)128, (uint8_t)88, (uint8_t)212, (uint8_t)0, (uint8_t)100, (uint8_t)213} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p311_uptime_sec_GET(pack) == (uint32_t)1969869180L);
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)248);
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)361162760L);
    assert(p311_name_LEN(ph) == 55);
    {
        char16_t * exemplary = u"rgXcMdigshrcukZrtsryflzyxseoPbvrCCrybsnfvtavbimslukUowo";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_time_usec_GET(pack) == (uint64_t)7353026846595050098L);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t)25859);
    assert(p320_param_id_LEN(ph) == 7);
    {
        char16_t * exemplary = u"qnxtslu";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)115);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)223);
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)72);
};


void c_CommunicationChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32);
    assert(p322_param_value_LEN(ph) == 86);
    {
        char16_t * exemplary = u"fkbxdysnSeFmfvyFlcmhetsAsmscmwbobtpyzeceupvqiqjeTcPkIzwmhhewlpxpbidrnzhtgvutfzJugegdzy";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 172);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)24236);
    assert(p322_param_id_LEN(ph) == 3);
    {
        char16_t * exemplary = u"hey";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)4493);
};


void c_CommunicationChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)113);
    assert(p323_param_value_LEN(ph) == 70);
    {
        char16_t * exemplary = u"pwmpdqbklwcstedqzdumwTCqkzwBodrXgbwfrhssfrXewlqoxtooxwzjzzszdvdtzawjac";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 140);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p323_param_id_LEN(ph) == 11);
    {
        char16_t * exemplary = u"eynyvbkjphx";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 22);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16);
};


void c_CommunicationChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_value_LEN(ph) == 2);
    {
        char16_t * exemplary = u"xp";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_CUSTOM);
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_FAILED);
    assert(p324_param_id_LEN(ph) == 12);
    {
        char16_t * exemplary = u"vqvrfbjeqoei";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    {
        uint16_t exemplary[] =  {(uint16_t)53209, (uint16_t)38945, (uint16_t)20398, (uint16_t)8285, (uint16_t)19976, (uint16_t)59507, (uint16_t)39001, (uint16_t)19216, (uint16_t)15892, (uint16_t)33083, (uint16_t)27784, (uint16_t)1848, (uint16_t)4783, (uint16_t)9261, (uint16_t)7917, (uint16_t)37219, (uint16_t)14373, (uint16_t)4711, (uint16_t)58931, (uint16_t)50920, (uint16_t)21808, (uint16_t)42118, (uint16_t)47775, (uint16_t)34806, (uint16_t)56136, (uint16_t)12325, (uint16_t)64116, (uint16_t)59876, (uint16_t)46110, (uint16_t)51897, (uint16_t)2653, (uint16_t)64854, (uint16_t)692, (uint16_t)23978, (uint16_t)61870, (uint16_t)16422, (uint16_t)40059, (uint16_t)16306, (uint16_t)58739, (uint16_t)10376, (uint16_t)23222, (uint16_t)38130, (uint16_t)25747, (uint16_t)63841, (uint16_t)11481, (uint16_t)49402, (uint16_t)53949, (uint16_t)32230, (uint16_t)36921, (uint16_t)43447, (uint16_t)58421, (uint16_t)28568, (uint16_t)44290, (uint16_t)15619, (uint16_t)38286, (uint16_t)11529, (uint16_t)29269, (uint16_t)25833, (uint16_t)55009, (uint16_t)26719, (uint16_t)32684, (uint16_t)19466, (uint16_t)12432, (uint16_t)9472, (uint16_t)41808, (uint16_t)63101, (uint16_t)967, (uint16_t)67, (uint16_t)14737, (uint16_t)41009, (uint16_t)33366, (uint16_t)32454} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_time_usec_GET(pack) == (uint64_t)3248363537762008590L);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR);
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)49910);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)14887);
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)10);
};


void c_CommunicationChannel_on_UAVIONIX_ADSB_OUT_CFG_10001(Bounds_Inside * ph, Pack * pack)
{
    assert(p10001_emitterType_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_POINT_OBSTACLE);
    assert(p10001_gpsOffsetLon_GET(pack) == e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA);
    assert(p10001_stallSpeed_GET(pack) == (uint16_t)(uint16_t)31026);
    assert(p10001_ICAO_GET(pack) == (uint32_t)1157802710L);
    assert(p10001_callsign_LEN(ph) == 8);
    {
        char16_t * exemplary = u"zLlpqGpb";
        char16_t * sample = p10001_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p10001_aircraftSize_GET(pack) == e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L75_W72P5M);
    assert(p10001_gpsOffsetLat_GET(pack) == e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_4M);
    assert(p10001_rfSelect_GET(pack) == e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED);
};


void c_CommunicationChannel_on_UAVIONIX_ADSB_OUT_DYNAMIC_10002(Bounds_Inside * ph, Pack * pack)
{
    assert(p10002_squawk_GET(pack) == (uint16_t)(uint16_t)34753);
    assert(p10002_velNS_GET(pack) == (int16_t)(int16_t) -32652);
    assert(p10002_accuracyHor_GET(pack) == (uint32_t)1896413619L);
    assert(p10002_baroAltMSL_GET(pack) == (int32_t)1788311892);
    assert(p10002_gpsAlt_GET(pack) == (int32_t) -760840074);
    assert(p10002_gpsFix_GET(pack) == e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0);
    assert(p10002_state_GET(pack) == e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED);
    assert(p10002_accuracyVel_GET(pack) == (uint16_t)(uint16_t)18806);
    assert(p10002_velVert_GET(pack) == (int16_t)(int16_t)1187);
    assert(p10002_accuracyVert_GET(pack) == (uint16_t)(uint16_t)62882);
    assert(p10002_gpsLat_GET(pack) == (int32_t) -497373778);
    assert(p10002_emergencyStatus_GET(pack) == e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY);
    assert(p10002_gpsLon_GET(pack) == (int32_t)718159832);
    assert(p10002_VelEW_GET(pack) == (int16_t)(int16_t)16041);
    assert(p10002_numSats_GET(pack) == (uint8_t)(uint8_t)13);
    assert(p10002_utcTime_GET(pack) == (uint32_t)2592212055L);
};


void c_CommunicationChannel_on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_10003(Bounds_Inside * ph, Pack * pack)
{
    assert(p10003_rfHealth_GET(pack) == e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_INITIALIZING);
};


void c_CommunicationChannel_on_DEVICE_OP_READ_11000(Bounds_Inside * ph, Pack * pack)
{
    assert(p11000_busname_LEN(ph) == 7);
    {
        char16_t * exemplary = u"hlihmaf";
        char16_t * sample = p11000_busname_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p11000_target_component_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p11000_request_id_GET(pack) == (uint32_t)3137963143L);
    assert(p11000_target_system_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p11000_address_GET(pack) == (uint8_t)(uint8_t)235);
    assert(p11000_regstart_GET(pack) == (uint8_t)(uint8_t)40);
    assert(p11000_count_GET(pack) == (uint8_t)(uint8_t)201);
    assert(p11000_bus_GET(pack) == (uint8_t)(uint8_t)143);
    assert(p11000_bustype_GET(pack) == e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_I2C);
};


void c_CommunicationChannel_on_DEVICE_OP_READ_REPLY_11001(Bounds_Inside * ph, Pack * pack)
{
    assert(p11001_result_GET(pack) == (uint8_t)(uint8_t)11);
    assert(p11001_regstart_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p11001_count_GET(pack) == (uint8_t)(uint8_t)122);
    {
        uint8_t exemplary[] =  {(uint8_t)187, (uint8_t)59, (uint8_t)85, (uint8_t)93, (uint8_t)63, (uint8_t)21, (uint8_t)228, (uint8_t)145, (uint8_t)92, (uint8_t)12, (uint8_t)21, (uint8_t)205, (uint8_t)122, (uint8_t)209, (uint8_t)25, (uint8_t)6, (uint8_t)183, (uint8_t)106, (uint8_t)148, (uint8_t)12, (uint8_t)9, (uint8_t)193, (uint8_t)189, (uint8_t)82, (uint8_t)14, (uint8_t)250, (uint8_t)173, (uint8_t)15, (uint8_t)185, (uint8_t)222, (uint8_t)26, (uint8_t)176, (uint8_t)42, (uint8_t)160, (uint8_t)22, (uint8_t)4, (uint8_t)44, (uint8_t)59, (uint8_t)209, (uint8_t)55, (uint8_t)10, (uint8_t)78, (uint8_t)149, (uint8_t)121, (uint8_t)229, (uint8_t)209, (uint8_t)44, (uint8_t)6, (uint8_t)155, (uint8_t)164, (uint8_t)177, (uint8_t)201, (uint8_t)32, (uint8_t)49, (uint8_t)69, (uint8_t)178, (uint8_t)45, (uint8_t)33, (uint8_t)138, (uint8_t)77, (uint8_t)209, (uint8_t)43, (uint8_t)197, (uint8_t)111, (uint8_t)100, (uint8_t)3, (uint8_t)108, (uint8_t)36, (uint8_t)38, (uint8_t)245, (uint8_t)38, (uint8_t)139, (uint8_t)35, (uint8_t)157, (uint8_t)31, (uint8_t)125, (uint8_t)42, (uint8_t)81, (uint8_t)63, (uint8_t)214, (uint8_t)161, (uint8_t)24, (uint8_t)4, (uint8_t)141, (uint8_t)10, (uint8_t)154, (uint8_t)163, (uint8_t)101, (uint8_t)105, (uint8_t)48, (uint8_t)80, (uint8_t)238, (uint8_t)201, (uint8_t)9, (uint8_t)69, (uint8_t)184, (uint8_t)99, (uint8_t)62, (uint8_t)47, (uint8_t)38, (uint8_t)136, (uint8_t)24, (uint8_t)138, (uint8_t)52, (uint8_t)223, (uint8_t)39, (uint8_t)197, (uint8_t)100, (uint8_t)4, (uint8_t)142, (uint8_t)152, (uint8_t)206, (uint8_t)228, (uint8_t)240, (uint8_t)71, (uint8_t)202, (uint8_t)154, (uint8_t)198, (uint8_t)232, (uint8_t)140, (uint8_t)12, (uint8_t)204, (uint8_t)152, (uint8_t)216, (uint8_t)19, (uint8_t)240, (uint8_t)23, (uint8_t)120} ;
        uint8_t*  sample = p11001_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 128);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p11001_request_id_GET(pack) == (uint32_t)2245592289L);
};


void c_CommunicationChannel_on_DEVICE_OP_WRITE_11002(Bounds_Inside * ph, Pack * pack)
{
    assert(p11002_bustype_GET(pack) == e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI);
    assert(p11002_address_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p11002_regstart_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p11002_request_id_GET(pack) == (uint32_t)999558945L);
    assert(p11002_bus_GET(pack) == (uint8_t)(uint8_t)221);
    assert(p11002_target_component_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p11002_target_system_GET(pack) == (uint8_t)(uint8_t)180);
    assert(p11002_count_GET(pack) == (uint8_t)(uint8_t)176);
    assert(p11002_busname_LEN(ph) == 29);
    {
        char16_t * exemplary = u"xIsybfytbwavlwkntcajyVvxbcfqx";
        char16_t * sample = p11002_busname_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 58);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)128, (uint8_t)28, (uint8_t)188, (uint8_t)222, (uint8_t)209, (uint8_t)134, (uint8_t)48, (uint8_t)132, (uint8_t)231, (uint8_t)187, (uint8_t)199, (uint8_t)236, (uint8_t)128, (uint8_t)156, (uint8_t)31, (uint8_t)217, (uint8_t)155, (uint8_t)246, (uint8_t)78, (uint8_t)165, (uint8_t)244, (uint8_t)1, (uint8_t)210, (uint8_t)204, (uint8_t)159, (uint8_t)2, (uint8_t)39, (uint8_t)41, (uint8_t)94, (uint8_t)204, (uint8_t)44, (uint8_t)88, (uint8_t)129, (uint8_t)22, (uint8_t)185, (uint8_t)45, (uint8_t)190, (uint8_t)41, (uint8_t)70, (uint8_t)154, (uint8_t)113, (uint8_t)103, (uint8_t)109, (uint8_t)231, (uint8_t)217, (uint8_t)201, (uint8_t)19, (uint8_t)253, (uint8_t)185, (uint8_t)206, (uint8_t)240, (uint8_t)98, (uint8_t)231, (uint8_t)176, (uint8_t)8, (uint8_t)112, (uint8_t)107, (uint8_t)210, (uint8_t)84, (uint8_t)140, (uint8_t)116, (uint8_t)188, (uint8_t)193, (uint8_t)199, (uint8_t)83, (uint8_t)39, (uint8_t)192, (uint8_t)70, (uint8_t)71, (uint8_t)208, (uint8_t)97, (uint8_t)119, (uint8_t)38, (uint8_t)43, (uint8_t)69, (uint8_t)119, (uint8_t)84, (uint8_t)170, (uint8_t)118, (uint8_t)60, (uint8_t)81, (uint8_t)239, (uint8_t)59, (uint8_t)200, (uint8_t)99, (uint8_t)209, (uint8_t)119, (uint8_t)135, (uint8_t)3, (uint8_t)42, (uint8_t)12, (uint8_t)241, (uint8_t)209, (uint8_t)178, (uint8_t)84, (uint8_t)246, (uint8_t)32, (uint8_t)158, (uint8_t)46, (uint8_t)231, (uint8_t)112, (uint8_t)134, (uint8_t)188, (uint8_t)237, (uint8_t)233, (uint8_t)215, (uint8_t)31, (uint8_t)63, (uint8_t)253, (uint8_t)79, (uint8_t)24, (uint8_t)115, (uint8_t)96, (uint8_t)132, (uint8_t)180, (uint8_t)189, (uint8_t)44, (uint8_t)140, (uint8_t)250, (uint8_t)233, (uint8_t)61, (uint8_t)245, (uint8_t)45, (uint8_t)191, (uint8_t)38, (uint8_t)104, (uint8_t)243, (uint8_t)143} ;
        uint8_t*  sample = p11002_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 128);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DEVICE_OP_WRITE_REPLY_11003(Bounds_Inside * ph, Pack * pack)
{
    assert(p11003_request_id_GET(pack) == (uint32_t)1171522401L);
    assert(p11003_result_GET(pack) == (uint8_t)(uint8_t)10);
};


void c_CommunicationChannel_on_ADAP_TUNING_11010(Bounds_Inside * ph, Pack * pack)
{
    assert(p11010_sigma_GET(pack) == (float)2.5300924E37F);
    assert(p11010_desired_GET(pack) == (float)1.8161123E38F);
    assert(p11010_sigma_dot_GET(pack) == (float)2.2069787E38F);
    assert(p11010_achieved_GET(pack) == (float) -3.213574E37F);
    assert(p11010_axis_GET(pack) == e_PID_TUNING_AXIS_PID_TUNING_YAW);
    assert(p11010_theta_dot_GET(pack) == (float)1.4757622E38F);
    assert(p11010_u_GET(pack) == (float)2.9683876E38F);
    assert(p11010_theta_GET(pack) == (float) -1.6369829E38F);
    assert(p11010_omega_dot_GET(pack) == (float) -2.3996904E38F);
    assert(p11010_error_GET(pack) == (float) -1.1788335E37F);
    assert(p11010_f_dot_GET(pack) == (float)2.8000038E36F);
    assert(p11010_omega_GET(pack) == (float)2.728428E38F);
    assert(p11010_f_GET(pack) == (float) -2.8415076E38F);
};


void c_CommunicationChannel_on_VISION_POSITION_DELTA_11011(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-1.6983454E38F, 2.7353405E38F, 4.3185104E37F} ;
        float*  sample = p11011_angle_delta_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p11011_confidence_GET(pack) == (float) -9.461622E37F);
    assert(p11011_time_delta_usec_GET(pack) == (uint64_t)2588857796910194624L);
    assert(p11011_time_usec_GET(pack) == (uint64_t)8848821314919318574L);
    {
        float exemplary[] =  {3.0842056E37F, -2.5747063E38F, 1.9477526E37F} ;
        float*  sample = p11011_position_delta_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
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
            case 3:
                if(pack == NULL) return c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3();
                c_TEST_Channel_on_POSITION_TARGET_LOCAL_NED_3(&ph, pack);
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
            case 84:
                if(pack == NULL) return c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84();
                c_TEST_Channel_on_SET_POSITION_TARGET_LOCAL_NED_84(&ph, pack);
                break;
            case 86:
                if(pack == NULL) return c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86();
                c_TEST_Channel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&ph, pack);
                break;
            case 87:
                if(pack == NULL) return c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87();
                c_TEST_Channel_on_POSITION_TARGET_GLOBAL_INT_87(&ph, pack);
                break;
            case 89:
                if(pack == NULL) return c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89();
                c_TEST_Channel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&ph, pack);
                break;
            case 90:
                if(pack == NULL) return c_CommunicationChannel_new_HIL_STATE_90();
                c_TEST_Channel_on_HIL_STATE_90(&ph, pack);
                break;
            case 91:
                if(pack == NULL) return c_CommunicationChannel_new_HIL_CONTROLS_91();
                c_TEST_Channel_on_HIL_CONTROLS_91(&ph, pack);
                break;
            case 92:
                if(pack == NULL) return c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92();
                c_TEST_Channel_on_HIL_RC_INPUTS_RAW_92(&ph, pack);
                break;
            case 93:
                if(pack == NULL) return c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93();
                c_TEST_Channel_on_HIL_ACTUATOR_CONTROLS_93(&ph, pack);
                break;
            case 100:
                if(pack == NULL) return c_CommunicationChannel_new_OPTICAL_FLOW_100();
                c_TEST_Channel_on_OPTICAL_FLOW_100(&ph, pack);
                break;
            case 101:
                if(pack == NULL) return c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101();
                c_TEST_Channel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&ph, pack);
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
        p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED, PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_FLAPPING_WING, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_INVALID, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_UNINIT, PH.base.pack) ;
        p0_custom_mode_SET((uint32_t)3757286026L, PH.base.pack) ;
        c_TEST_Channel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
        p1_voltage_battery_SET((uint16_t)(uint16_t)48981, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t) -99, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)52808, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)53854, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t)25263, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)21859, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)48193, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)27525, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)33636, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)27164, PH.base.pack) ;
        c_TEST_Channel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_boot_ms_SET((uint32_t)2502737219L, PH.base.pack) ;
        p2_time_unix_usec_SET((uint64_t)2377802486913103117L, PH.base.pack) ;
        c_TEST_Channel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_x_SET((float)6.052137E36F, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)1935353043L, PH.base.pack) ;
        p3_yaw_SET((float)2.9714496E37F, PH.base.pack) ;
        p3_yaw_rate_SET((float) -2.7285621E38F, PH.base.pack) ;
        p3_afx_SET((float)2.2964852E38F, PH.base.pack) ;
        p3_z_SET((float) -3.3883094E38F, PH.base.pack) ;
        p3_vx_SET((float) -2.1997894E38F, PH.base.pack) ;
        p3_afz_SET((float) -1.961987E37F, PH.base.pack) ;
        p3_y_SET((float) -1.5107369E38F, PH.base.pack) ;
        p3_afy_SET((float)2.2289277E38F, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)6692, PH.base.pack) ;
        p3_vy_SET((float)1.9042449E38F, PH.base.pack) ;
        p3_vz_SET((float)1.1547073E38F, PH.base.pack) ;
        c_TEST_Channel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PING_4(), &PH);
        p4_target_component_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        p4_target_system_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        p4_time_usec_SET((uint64_t)3862608586981643298L, PH.base.pack) ;
        p4_seq_SET((uint32_t)2877900857L, PH.base.pack) ;
        c_TEST_Channel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_target_system_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p5_version_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        {
            char16_t* passkey = u"KxsyjjifnipQftrhyv";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_control_request_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_ack_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        p6_control_request_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"pfxqAehmmqsj";
            p7_key_SET_(key, &PH) ;
        }
        c_TEST_Channel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
        p11_target_system_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p11_custom_mode_SET((uint32_t)4047015740L, PH.base.pack) ;
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_ARMED, PH.base.pack) ;
        c_TEST_Channel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
        p20_target_component_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        p20_param_index_SET((int16_t)(int16_t)17189, PH.base.pack) ;
        {
            char16_t* param_id = u"yqcyKvp";
            p20_param_id_SET_(param_id, &PH) ;
        }
        p20_target_system_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_component_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        p21_target_system_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
        p22_param_index_SET((uint16_t)(uint16_t)4296, PH.base.pack) ;
        {
            char16_t* param_id = u"dhlikctqn";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_count_SET((uint16_t)(uint16_t)25810, PH.base.pack) ;
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT8, PH.base.pack) ;
        p22_param_value_SET((float)2.18129E38F, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32, PH.base.pack) ;
        p23_target_system_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p23_param_value_SET((float) -2.5471102E38F, PH.base.pack) ;
        p23_target_component_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        {
            char16_t* param_id = u"hvwxdlSbvtbbdowp";
            p23_param_id_SET_(param_id, &PH) ;
        }
        c_TEST_Channel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
        p24_eph_SET((uint16_t)(uint16_t)33785, PH.base.pack) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)32566, PH.base.pack) ;
        p24_epv_SET((uint16_t)(uint16_t)23843, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)3121924690L, &PH) ;
        p24_alt_SET((int32_t) -880291799, PH.base.pack) ;
        p24_lat_SET((int32_t)301445565, PH.base.pack) ;
        p24_alt_ellipsoid_SET((int32_t)1236021779, &PH) ;
        p24_v_acc_SET((uint32_t)1782828515L, &PH) ;
        p24_vel_SET((uint16_t)(uint16_t)40953, PH.base.pack) ;
        p24_time_usec_SET((uint64_t)1564219986044564487L, PH.base.pack) ;
        p24_hdg_acc_SET((uint32_t)504176859L, &PH) ;
        p24_h_acc_SET((uint32_t)2008357889L, &PH) ;
        p24_lon_SET((int32_t)100196939, PH.base.pack) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        c_TEST_Channel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_snr[] =  {(uint8_t)117, (uint8_t)106, (uint8_t)147, (uint8_t)110, (uint8_t)188, (uint8_t)182, (uint8_t)54, (uint8_t)83, (uint8_t)165, (uint8_t)66, (uint8_t)61, (uint8_t)130, (uint8_t)96, (uint8_t)146, (uint8_t)210, (uint8_t)164, (uint8_t)209, (uint8_t)24, (uint8_t)190, (uint8_t)163};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)199, (uint8_t)221, (uint8_t)116, (uint8_t)164, (uint8_t)38, (uint8_t)151, (uint8_t)98, (uint8_t)158, (uint8_t)138, (uint8_t)22, (uint8_t)237, (uint8_t)235, (uint8_t)192, (uint8_t)15, (uint8_t)63, (uint8_t)228, (uint8_t)49, (uint8_t)231, (uint8_t)239, (uint8_t)183};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)243, (uint8_t)129, (uint8_t)232, (uint8_t)201, (uint8_t)198, (uint8_t)173, (uint8_t)191, (uint8_t)80, (uint8_t)241, (uint8_t)116, (uint8_t)137, (uint8_t)14, (uint8_t)28, (uint8_t)31, (uint8_t)252, (uint8_t)210, (uint8_t)166, (uint8_t)4, (uint8_t)193, (uint8_t)31};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
        {
            uint8_t satellite_prn[] =  {(uint8_t)9, (uint8_t)235, (uint8_t)52, (uint8_t)171, (uint8_t)142, (uint8_t)107, (uint8_t)20, (uint8_t)89, (uint8_t)21, (uint8_t)107, (uint8_t)27, (uint8_t)255, (uint8_t)183, (uint8_t)205, (uint8_t)40, (uint8_t)239, (uint8_t)220, (uint8_t)68, (uint8_t)18, (uint8_t)15};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_used[] =  {(uint8_t)146, (uint8_t)14, (uint8_t)99, (uint8_t)18, (uint8_t)101, (uint8_t)152, (uint8_t)211, (uint8_t)151, (uint8_t)25, (uint8_t)246, (uint8_t)38, (uint8_t)54, (uint8_t)2, (uint8_t)130, (uint8_t)164, (uint8_t)171, (uint8_t)15, (uint8_t)124, (uint8_t)234, (uint8_t)165};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
        p26_ymag_SET((int16_t)(int16_t) -26198, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t) -20368, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t)6569, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)3739400978L, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t) -26616, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t)15990, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t) -30252, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t) -32278, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t)21168, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t)20951, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
        p27_zacc_SET((int16_t)(int16_t)19503, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t) -9822, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t) -10064, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t) -26511, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t) -5755, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t) -24390, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t)29322, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t)526, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t)1102, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)196834432168508760L, PH.base.pack) ;
        c_TEST_Channel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
        p28_press_diff2_SET((int16_t)(int16_t)18642, PH.base.pack) ;
        p28_press_diff1_SET((int16_t)(int16_t) -22194, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t)11930, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)69720345991366460L, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t)28994, PH.base.pack) ;
        c_TEST_Channel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_time_boot_ms_SET((uint32_t)171980328L, PH.base.pack) ;
        p29_press_diff_SET((float)2.363488E38F, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t)24046, PH.base.pack) ;
        p29_press_abs_SET((float) -1.2052405E38F, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
        p30_yawspeed_SET((float) -1.2593523E38F, PH.base.pack) ;
        p30_rollspeed_SET((float)9.865922E37F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)2433965492L, PH.base.pack) ;
        p30_yaw_SET((float) -6.6120163E37F, PH.base.pack) ;
        p30_pitch_SET((float) -2.8197577E38F, PH.base.pack) ;
        p30_roll_SET((float)1.9303931E38F, PH.base.pack) ;
        p30_pitchspeed_SET((float)9.323722E37F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_q3_SET((float) -2.3504644E38F, PH.base.pack) ;
        p31_pitchspeed_SET((float) -2.6873919E38F, PH.base.pack) ;
        p31_q1_SET((float)8.602989E37F, PH.base.pack) ;
        p31_rollspeed_SET((float)2.779231E38F, PH.base.pack) ;
        p31_q4_SET((float)5.6644267E37F, PH.base.pack) ;
        p31_yawspeed_SET((float)5.583059E36F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)3633934419L, PH.base.pack) ;
        p31_q2_SET((float)1.7537528E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_y_SET((float) -3.2339548E38F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)1731293342L, PH.base.pack) ;
        p32_x_SET((float) -3.776003E37F, PH.base.pack) ;
        p32_z_SET((float)6.929247E37F, PH.base.pack) ;
        p32_vx_SET((float) -2.5328415E37F, PH.base.pack) ;
        p32_vy_SET((float)1.4952403E38F, PH.base.pack) ;
        p32_vz_SET((float) -8.095612E36F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_hdg_SET((uint16_t)(uint16_t)64482, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t)21333, PH.base.pack) ;
        p33_alt_SET((int32_t) -1561981782, PH.base.pack) ;
        p33_lat_SET((int32_t) -1752074534, PH.base.pack) ;
        p33_relative_alt_SET((int32_t)1542460366, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t)30793, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)2011811309L, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t) -11770, PH.base.pack) ;
        p33_lon_SET((int32_t)278208853, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_rssi_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t) -13629, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t)23063, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t)13208, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t)3769, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t) -12665, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t) -29452, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t) -16612, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)1738099037L, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t)15760, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_chan8_raw_SET((uint16_t)(uint16_t)60087, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)53389, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)1454964265L, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)54027, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)57953, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)6278, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)14583, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)55970, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)19562, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo8_raw_SET((uint16_t)(uint16_t)41977, PH.base.pack) ;
        p36_port_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)61537, &PH) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)31107, &PH) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)3369, PH.base.pack) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)3932, PH.base.pack) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)12194, &PH) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)44167, &PH) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)16900, &PH) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)57672, PH.base.pack) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)39897, PH.base.pack) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)1639, &PH) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)30364, &PH) ;
        p36_time_usec_SET((uint32_t)852117647L, PH.base.pack) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)17116, PH.base.pack) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)11269, PH.base.pack) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)65000, &PH) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)13295, PH.base.pack) ;
        c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_start_index_SET((int16_t)(int16_t) -22399, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t)30706, PH.base.pack) ;
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_end_index_SET((int16_t)(int16_t) -7328, PH.base.pack) ;
        p38_target_component_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t) -6475, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
        p39_z_SET((float)3.013889E37F, PH.base.pack) ;
        p39_x_SET((float) -2.188893E38F, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p39_param3_SET((float)2.0670962E38F, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p39_param4_SET((float)1.9674838E38F, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p39_param1_SET((float) -3.0278273E38F, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)3908, PH.base.pack) ;
        p39_y_SET((float)1.6603599E38F, PH.base.pack) ;
        p39_param2_SET((float)1.4797372E38F, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
        p40_seq_SET((uint16_t)(uint16_t)59947, PH.base.pack) ;
        p40_target_component_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_target_system_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        p41_seq_SET((uint16_t)(uint16_t)51202, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)32378, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_target_component_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
        p43_target_system_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p44_count_SET((uint16_t)(uint16_t)1699, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_target_system_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)42333, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM7, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_longitude_SET((int32_t) -683305881, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)964399069680555709L, &PH) ;
        p48_target_system_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p48_altitude_SET((int32_t) -1293298328, PH.base.pack) ;
        p48_latitude_SET((int32_t)376404914, PH.base.pack) ;
        c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_time_usec_SET((uint64_t)4706696726007529983L, &PH) ;
        p49_longitude_SET((int32_t) -1110240306, PH.base.pack) ;
        p49_altitude_SET((int32_t) -365133171, PH.base.pack) ;
        p49_latitude_SET((int32_t)2129936714, PH.base.pack) ;
        c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_param_value_max_SET((float)3.1689229E38F, PH.base.pack) ;
        p50_param_value0_SET((float)1.7454348E38F, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        p50_param_value_min_SET((float)3.6540713E37F, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t)28601, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        {
            char16_t* param_id = u"u";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        p50_scale_SET((float)1.4242772E38F, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_target_component_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)4262, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_p1z_SET((float) -2.8051295E38F, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p54_p2y_SET((float)1.1746327E38F, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
        p54_p1y_SET((float)7.376964E37F, PH.base.pack) ;
        p54_p2x_SET((float) -3.2599E38F, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        p54_p2z_SET((float) -2.9913822E38F, PH.base.pack) ;
        p54_p1x_SET((float) -2.7770272E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p1x_SET((float) -5.0733933E37F, PH.base.pack) ;
        p55_p2x_SET((float) -1.1205563E38F, PH.base.pack) ;
        p55_p2z_SET((float)3.0944515E38F, PH.base.pack) ;
        p55_p1y_SET((float)2.4278415E38F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
        p55_p1z_SET((float) -3.6524594E37F, PH.base.pack) ;
        p55_p2y_SET((float)1.2518875E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_time_usec_SET((uint64_t)2497490024104014366L, PH.base.pack) ;
        {
            float q[] =  {-4.393846E37F, -3.2402794E38F, 1.836071E37F, -4.971769E37F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        p61_rollspeed_SET((float)2.1007445E38F, PH.base.pack) ;
        {
            float covariance[] =  {3.2383684E38F, 1.0553909E38F, 2.8662773E38F, -2.5292966E38F, -2.940768E38F, 1.7403784E38F, -3.0656292E38F, -1.3328743E38F, 2.3754117E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p61_yawspeed_SET((float)2.5848173E38F, PH.base.pack) ;
        p61_pitchspeed_SET((float) -2.3585017E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_xtrack_error_SET((float) -2.0097927E38F, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t)14460, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)25265, PH.base.pack) ;
        p62_nav_roll_SET((float) -4.3224954E37F, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t)15682, PH.base.pack) ;
        p62_aspd_error_SET((float) -7.8472633E37F, PH.base.pack) ;
        p62_nav_pitch_SET((float)9.375259E37F, PH.base.pack) ;
        p62_alt_error_SET((float)7.3341117E37F, PH.base.pack) ;
        c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_vx_SET((float) -1.6408788E38F, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)6947815258434180851L, PH.base.pack) ;
        p63_relative_alt_SET((int32_t)772888927, PH.base.pack) ;
        p63_lat_SET((int32_t) -59026835, PH.base.pack) ;
        p63_lon_SET((int32_t)1527722216, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, PH.base.pack) ;
        p63_vy_SET((float)2.2309924E38F, PH.base.pack) ;
        {
            float covariance[] =  {-1.9718074E37F, -1.5759383E37F, 5.8307045E37F, 3.187205E38F, 1.9234068E37F, 2.2460193E38F, 9.094833E37F, 2.4000462E37F, 1.8664839E38F, -1.1292885E38F, -1.8378374E38F, -2.3629374E38F, -2.2120156E38F, 1.9695895E38F, 2.128531E38F, 6.363106E37F, -3.314871E38F, -2.8594364E37F, 2.2070543E38F, -1.9736943E38F, 5.8883334E37F, -2.500916E38F, -2.0889021E37F, -1.8185847E38F, -8.951783E37F, -2.9005637E38F, -2.4219156E38F, 2.2724844E38F, -2.6371714E38F, 3.3944677E38F, 1.2036241E37F, 1.0733374E38F, -2.9657463E38F, -3.339916E38F, 2.9904689E38F, -1.9407804E37F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_vz_SET((float) -1.6504563E38F, PH.base.pack) ;
        p63_alt_SET((int32_t) -37097484, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_az_SET((float)2.8160087E38F, PH.base.pack) ;
        p64_y_SET((float)1.8415288E38F, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)3925006818284411813L, PH.base.pack) ;
        p64_ax_SET((float) -3.8415353E37F, PH.base.pack) ;
        p64_ay_SET((float)1.2095718E38F, PH.base.pack) ;
        p64_z_SET((float)3.1576008E38F, PH.base.pack) ;
        p64_vy_SET((float)5.9029894E37F, PH.base.pack) ;
        p64_vz_SET((float) -2.3921891E38F, PH.base.pack) ;
        p64_x_SET((float) -2.1978534E38F, PH.base.pack) ;
        p64_vx_SET((float)3.305407E38F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, PH.base.pack) ;
        {
            float covariance[] =  {-1.9759142E38F, 1.1199236E38F, -1.087245E38F, -4.7696916E37F, 2.183891E38F, -1.4713713E38F, 2.6253452E37F, 1.4869073E38F, 3.558574E37F, -1.6714743E38F, -1.0508726E38F, -1.9890296E38F, 1.1480092E38F, -2.1071093E38F, 1.3335088E38F, -2.0490436E38F, -2.520602E38F, -1.2865638E38F, -1.2270626E37F, -1.9845616E38F, 2.8274192E38F, 1.1513269E38F, 1.191909E38F, -1.8967986E38F, 2.2565779E38F, 2.7393025E38F, 1.4412951E38F, 1.3571741E38F, 2.7382097E38F, 9.755008E37F, -1.0798661E38F, -2.5767144E38F, 8.0016803E37F, 4.6501455E36F, 2.9940716E37F, 1.7724374E38F, 2.4500084E38F, 1.041267E38F, -9.776023E37F, -1.1676083E38F, 5.033777E36F, -6.758617E37F, 2.0507575E38F, 8.5023075E37F, -1.4028092E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan4_raw_SET((uint16_t)(uint16_t)3664, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)36503, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)31920, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)35914, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)25165, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)44533, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)50186, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)483053390L, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)15600, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)41707, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)65049, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)61141, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)47525, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)24634, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)50667, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)4566, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)58481, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)6477, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)12286, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_req_stream_id_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p66_req_message_rate_SET((uint16_t)(uint16_t)33572, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        c_TEST_Channel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
        p67_stream_id_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        p67_on_off_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p67_message_rate_SET((uint16_t)(uint16_t)52747, PH.base.pack) ;
        c_TEST_Channel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_target_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p69_buttons_SET((uint16_t)(uint16_t)47695, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t)19913, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t) -16524, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t)15264, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t)27400, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_target_component_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)15322, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)53650, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)31678, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)4651, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)32018, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)15589, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)53847, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)58126, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_seq_SET((uint16_t)(uint16_t)63767, PH.base.pack) ;
        p73_param3_SET((float)1.5379197E38F, PH.base.pack) ;
        p73_param1_SET((float)3.308039E38F, PH.base.pack) ;
        p73_x_SET((int32_t) -1501390649, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p73_param4_SET((float) -3.3684194E38F, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p73_z_SET((float)8.034974E37F, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        p73_param2_SET((float)3.9620673E36F, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_DO_LAST, PH.base.pack) ;
        p73_y_SET((int32_t) -2093445574, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
        p74_alt_SET((float)2.7960746E38F, PH.base.pack) ;
        p74_groundspeed_SET((float) -1.8443324E38F, PH.base.pack) ;
        p74_airspeed_SET((float)1.1883593E38F, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)38152, PH.base.pack) ;
        p74_climb_SET((float)2.9922657E38F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t)14858, PH.base.pack) ;
        c_TEST_Channel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
        p75_param3_SET((float)3.317899E38F, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p75_param4_SET((float) -2.3101364E38F, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p75_param1_SET((float)2.6057375E38F, PH.base.pack) ;
        p75_param2_SET((float) -2.9092129E38F, PH.base.pack) ;
        p75_z_SET((float) -9.390353E37F, PH.base.pack) ;
        p75_x_SET((int32_t) -607636037, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER, PH.base.pack) ;
        p75_y_SET((int32_t)1382602225, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
        p76_target_component_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        p76_param7_SET((float) -2.2883292E38F, PH.base.pack) ;
        p76_param5_SET((float) -2.4103725E38F, PH.base.pack) ;
        p76_param4_SET((float)1.0890053E38F, PH.base.pack) ;
        p76_param3_SET((float)5.740333E37F, PH.base.pack) ;
        p76_param2_SET((float) -9.975706E37F, PH.base.pack) ;
        p76_param6_SET((float)2.5492608E38F, PH.base.pack) ;
        p76_param1_SET((float) -1.1421363E38F, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_IN_PROGRESS, PH.base.pack) ;
        p77_target_component_SET((uint8_t)(uint8_t)139, &PH) ;
        p77_target_system_SET((uint8_t)(uint8_t)114, &PH) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_DO_RALLY_LAND, PH.base.pack) ;
        p77_result_param2_SET((int32_t) -2083155699, &PH) ;
        p77_progress_SET((uint8_t)(uint8_t)160, &PH) ;
        c_TEST_Channel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_thrust_SET((float) -1.6486037E38F, PH.base.pack) ;
        p81_roll_SET((float) -2.9063184E38F, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        p81_yaw_SET((float)6.984934E37F, PH.base.pack) ;
        p81_pitch_SET((float)3.5380757E37F, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)1672259551L, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_thrust_SET((float)1.6836219E38F, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p82_time_boot_ms_SET((uint32_t)873824322L, PH.base.pack) ;
        {
            float q[] =  {5.3570407E37F, -2.8731066E38F, -9.298205E37F, -3.1754737E38F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_body_pitch_rate_SET((float)1.4932367E38F, PH.base.pack) ;
        p82_body_roll_rate_SET((float) -2.0595278E38F, PH.base.pack) ;
        p82_body_yaw_rate_SET((float)2.1139009E38F, PH.base.pack) ;
        c_TEST_Channel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_body_pitch_rate_SET((float) -2.1078843E38F, PH.base.pack) ;
        p83_body_roll_rate_SET((float)1.2823207E37F, PH.base.pack) ;
        {
            float q[] =  {1.7237136E38F, -1.9448908E38F, 2.045152E38F, 1.7882465E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_thrust_SET((float) -2.695336E38F, PH.base.pack) ;
        p83_body_yaw_rate_SET((float)2.7193653E38F, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)975910876L, PH.base.pack) ;
        p83_type_mask_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_afx_SET((float) -2.801151E38F, PH.base.pack) ;
        p84_x_SET((float)2.67391E38F, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p84_afy_SET((float)2.930559E36F, PH.base.pack) ;
        p84_vx_SET((float)2.3492484E37F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)2243434056L, PH.base.pack) ;
        p84_vz_SET((float) -1.8543017E38F, PH.base.pack) ;
        p84_yaw_rate_SET((float) -8.980201E37F, PH.base.pack) ;
        p84_y_SET((float) -8.670094E37F, PH.base.pack) ;
        p84_vy_SET((float) -7.931537E37F, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p84_z_SET((float) -2.0751978E37F, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)6862, PH.base.pack) ;
        p84_yaw_SET((float) -2.1626186E38F, PH.base.pack) ;
        p84_afz_SET((float) -1.896281E38F, PH.base.pack) ;
        c_TEST_Channel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_lat_int_SET((int32_t) -339255875, PH.base.pack) ;
        p86_alt_SET((float) -1.8652473E38F, PH.base.pack) ;
        p86_vy_SET((float)2.024968E37F, PH.base.pack) ;
        p86_lon_int_SET((int32_t) -304259324, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)2316502151L, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p86_vz_SET((float) -7.24514E36F, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p86_afy_SET((float)1.6936133E38F, PH.base.pack) ;
        p86_afz_SET((float)2.0159281E38F, PH.base.pack) ;
        p86_yaw_SET((float) -1.3182083E37F, PH.base.pack) ;
        p86_vx_SET((float)1.858552E38F, PH.base.pack) ;
        p86_yaw_rate_SET((float)2.2443036E38F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)14849, PH.base.pack) ;
        p86_afx_SET((float)2.4619987E38F, PH.base.pack) ;
        c_TEST_Channel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_vz_SET((float) -2.453252E38F, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)1336715628L, PH.base.pack) ;
        p87_lon_int_SET((int32_t) -1698974892, PH.base.pack) ;
        p87_yaw_rate_SET((float)1.4572205E37F, PH.base.pack) ;
        p87_vx_SET((float) -1.871773E38F, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p87_yaw_SET((float) -5.524502E37F, PH.base.pack) ;
        p87_afy_SET((float) -7.846838E36F, PH.base.pack) ;
        p87_afx_SET((float)3.5334128E37F, PH.base.pack) ;
        p87_lat_int_SET((int32_t)1765952953, PH.base.pack) ;
        p87_alt_SET((float) -5.4364347E37F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)63725, PH.base.pack) ;
        p87_afz_SET((float) -1.0998153E38F, PH.base.pack) ;
        p87_vy_SET((float)2.0189774E38F, PH.base.pack) ;
        c_TEST_Channel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_roll_SET((float)3.2170344E38F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)3639711002L, PH.base.pack) ;
        p89_y_SET((float)1.3927233E38F, PH.base.pack) ;
        p89_z_SET((float)6.4790266E37F, PH.base.pack) ;
        p89_pitch_SET((float) -1.0178925E38F, PH.base.pack) ;
        p89_x_SET((float)3.136394E38F, PH.base.pack) ;
        p89_yaw_SET((float)1.819299E38F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
        p90_vx_SET((int16_t)(int16_t) -3474, PH.base.pack) ;
        p90_rollspeed_SET((float)1.4637278E38F, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t) -30405, PH.base.pack) ;
        p90_alt_SET((int32_t) -1137524312, PH.base.pack) ;
        p90_yawspeed_SET((float)3.0686334E38F, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t) -24879, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)6081314777698915113L, PH.base.pack) ;
        p90_yaw_SET((float)2.8375947E38F, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t)4447, PH.base.pack) ;
        p90_pitch_SET((float) -1.687662E38F, PH.base.pack) ;
        p90_roll_SET((float) -3.3031695E38F, PH.base.pack) ;
        p90_lat_SET((int32_t)598096338, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t) -172, PH.base.pack) ;
        p90_lon_SET((int32_t) -1806385373, PH.base.pack) ;
        p90_pitchspeed_SET((float) -2.8366406E38F, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t)8164, PH.base.pack) ;
        c_TEST_Channel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
        p91_aux1_SET((float)1.9962027E38F, PH.base.pack) ;
        p91_roll_ailerons_SET((float) -1.5366309E38F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_ARMED, PH.base.pack) ;
        p91_pitch_elevator_SET((float)2.3375453E38F, PH.base.pack) ;
        p91_throttle_SET((float)4.7955704E37F, PH.base.pack) ;
        p91_aux4_SET((float)2.9231834E38F, PH.base.pack) ;
        p91_yaw_rudder_SET((float)1.7918291E37F, PH.base.pack) ;
        p91_aux2_SET((float) -2.3436083E38F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)6623985788348441034L, PH.base.pack) ;
        p91_aux3_SET((float)3.0990242E38F, PH.base.pack) ;
        c_TEST_Channel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan12_raw_SET((uint16_t)(uint16_t)47529, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)34736, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)1630581090942475903L, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)12423, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)53561, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)32980, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)46989, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)12407, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)54892, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)64953, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)11487, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)39940, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)5165, PH.base.pack) ;
        c_TEST_Channel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_time_usec_SET((uint64_t)8454991304750796385L, PH.base.pack) ;
        p93_flags_SET((uint64_t)8936923776884257108L, PH.base.pack) ;
        p93_mode_SET(e_MAV_MODE_MAV_MODE_TEST_DISARMED, PH.base.pack) ;
        {
            float controls[] =  {2.0212393E38F, 3.3694644E38F, 1.1538606E37F, 5.4415484E37F, -2.2634017E38F, -1.6503768E38F, -2.6334737E38F, -1.1312243E36F, -2.149682E38F, -2.7605327E38F, -2.8128057E38F, -1.8805834E38F, -2.0167901E38F, -1.848552E38F, 2.7865664E38F, 3.2410834E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_sensor_id_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float) -1.2048791E38F, PH.base.pack) ;
        p100_flow_y_SET((int16_t)(int16_t)28798, PH.base.pack) ;
        p100_ground_distance_SET((float)4.7310657E37F, PH.base.pack) ;
        p100_flow_x_SET((int16_t)(int16_t)26934, PH.base.pack) ;
        p100_flow_rate_x_SET((float)1.5653577E38F, &PH) ;
        p100_quality_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)7181754580670574436L, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float)3.0536877E38F, PH.base.pack) ;
        p100_flow_rate_y_SET((float) -1.3732768E38F, &PH) ;
        c_TEST_Channel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_usec_SET((uint64_t)4986183000393551627L, PH.base.pack) ;
        p101_x_SET((float)2.2984941E38F, PH.base.pack) ;
        p101_pitch_SET((float)2.7518646E38F, PH.base.pack) ;
        p101_yaw_SET((float)1.8189685E37F, PH.base.pack) ;
        p101_roll_SET((float)1.7366118E38F, PH.base.pack) ;
        p101_z_SET((float) -2.053783E38F, PH.base.pack) ;
        p101_y_SET((float)9.8270353E36F, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_z_SET((float) -1.083662E38F, PH.base.pack) ;
        p102_roll_SET((float) -3.1200252E38F, PH.base.pack) ;
        p102_pitch_SET((float) -1.2737819E38F, PH.base.pack) ;
        p102_yaw_SET((float) -3.1450344E38F, PH.base.pack) ;
        p102_usec_SET((uint64_t)6278435554115547332L, PH.base.pack) ;
        p102_x_SET((float) -1.8202943E38F, PH.base.pack) ;
        p102_y_SET((float)1.8415848E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_usec_SET((uint64_t)6258964868240759567L, PH.base.pack) ;
        p103_x_SET((float) -2.6129924E38F, PH.base.pack) ;
        p103_z_SET((float)8.748628E37F, PH.base.pack) ;
        p103_y_SET((float)2.8840262E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_z_SET((float) -4.4800025E37F, PH.base.pack) ;
        p104_usec_SET((uint64_t)8975799484182240523L, PH.base.pack) ;
        p104_x_SET((float) -9.539132E37F, PH.base.pack) ;
        p104_y_SET((float)2.8636978E37F, PH.base.pack) ;
        p104_pitch_SET((float) -2.8671107E38F, PH.base.pack) ;
        p104_roll_SET((float) -1.0672434E38F, PH.base.pack) ;
        p104_yaw_SET((float)8.0502765E37F, PH.base.pack) ;
        c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
        p105_temperature_SET((float) -2.3017363E38F, PH.base.pack) ;
        p105_zmag_SET((float) -3.2635398E37F, PH.base.pack) ;
        p105_ymag_SET((float) -1.4406729E38F, PH.base.pack) ;
        p105_xmag_SET((float) -1.7417103E38F, PH.base.pack) ;
        p105_ygyro_SET((float)2.8546905E38F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)4550553380381982591L, PH.base.pack) ;
        p105_zgyro_SET((float)3.6992567E37F, PH.base.pack) ;
        p105_diff_pressure_SET((float) -2.1710652E38F, PH.base.pack) ;
        p105_pressure_alt_SET((float) -2.5856766E38F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)54394, PH.base.pack) ;
        p105_abs_pressure_SET((float)2.489318E38F, PH.base.pack) ;
        p105_yacc_SET((float) -6.768442E36F, PH.base.pack) ;
        p105_xgyro_SET((float) -2.1472186E38F, PH.base.pack) ;
        p105_zacc_SET((float) -8.991809E37F, PH.base.pack) ;
        p105_xacc_SET((float)5.737627E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_integrated_y_SET((float) -1.060912E38F, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
        p106_integrated_xgyro_SET((float)1.6002546E38F, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)192592727L, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)4447517559160544808L, PH.base.pack) ;
        p106_distance_SET((float)2.28673E37F, PH.base.pack) ;
        p106_integrated_zgyro_SET((float)1.0702308E38F, PH.base.pack) ;
        p106_integrated_ygyro_SET((float)2.5507925E38F, PH.base.pack) ;
        p106_integrated_x_SET((float)2.1357515E38F, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)1783822328L, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t)16766, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
        p107_fields_updated_SET((uint32_t)2353275106L, PH.base.pack) ;
        p107_ymag_SET((float) -2.7117756E38F, PH.base.pack) ;
        p107_temperature_SET((float) -1.3379012E38F, PH.base.pack) ;
        p107_pressure_alt_SET((float)3.2395132E38F, PH.base.pack) ;
        p107_ygyro_SET((float) -2.2171383E38F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)7052235023388606278L, PH.base.pack) ;
        p107_diff_pressure_SET((float) -1.1685672E38F, PH.base.pack) ;
        p107_xgyro_SET((float) -5.629325E37F, PH.base.pack) ;
        p107_abs_pressure_SET((float) -1.3571519E38F, PH.base.pack) ;
        p107_zgyro_SET((float) -1.5151402E38F, PH.base.pack) ;
        p107_zmag_SET((float)1.4807482E37F, PH.base.pack) ;
        p107_yacc_SET((float) -1.0643177E38F, PH.base.pack) ;
        p107_xacc_SET((float)2.3321284E38F, PH.base.pack) ;
        p107_zacc_SET((float)1.6202832E38F, PH.base.pack) ;
        p107_xmag_SET((float) -2.446263E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
        p108_yacc_SET((float) -8.553067E37F, PH.base.pack) ;
        p108_zacc_SET((float)2.9891931E37F, PH.base.pack) ;
        p108_std_dev_horz_SET((float)3.1067745E38F, PH.base.pack) ;
        p108_xgyro_SET((float)3.7916824E37F, PH.base.pack) ;
        p108_q2_SET((float)2.9441047E38F, PH.base.pack) ;
        p108_xacc_SET((float)1.3909587E37F, PH.base.pack) ;
        p108_vn_SET((float)7.0752123E37F, PH.base.pack) ;
        p108_q3_SET((float) -1.0970345E38F, PH.base.pack) ;
        p108_yaw_SET((float)2.2792096E38F, PH.base.pack) ;
        p108_zgyro_SET((float)7.510917E37F, PH.base.pack) ;
        p108_ve_SET((float)2.2567854E38F, PH.base.pack) ;
        p108_alt_SET((float) -2.2223943E38F, PH.base.pack) ;
        p108_q1_SET((float) -1.1288222E38F, PH.base.pack) ;
        p108_q4_SET((float) -7.733451E37F, PH.base.pack) ;
        p108_pitch_SET((float)1.7827183E38F, PH.base.pack) ;
        p108_lat_SET((float)1.1429547E38F, PH.base.pack) ;
        p108_std_dev_vert_SET((float)3.6337499E37F, PH.base.pack) ;
        p108_roll_SET((float)1.6009205E38F, PH.base.pack) ;
        p108_lon_SET((float) -2.1285752E38F, PH.base.pack) ;
        p108_vd_SET((float)3.002798E38F, PH.base.pack) ;
        p108_ygyro_SET((float)1.6179841E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
        p109_rssi_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)42634, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)63713, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        c_CommunicationChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        p110_target_network_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        p110_target_system_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)197, (uint8_t)70, (uint8_t)108, (uint8_t)247, (uint8_t)72, (uint8_t)106, (uint8_t)140, (uint8_t)13, (uint8_t)99, (uint8_t)88, (uint8_t)61, (uint8_t)42, (uint8_t)178, (uint8_t)49, (uint8_t)119, (uint8_t)101, (uint8_t)60, (uint8_t)75, (uint8_t)49, (uint8_t)171, (uint8_t)190, (uint8_t)5, (uint8_t)122, (uint8_t)225, (uint8_t)180, (uint8_t)181, (uint8_t)17, (uint8_t)104, (uint8_t)200, (uint8_t)253, (uint8_t)113, (uint8_t)124, (uint8_t)40, (uint8_t)88, (uint8_t)123, (uint8_t)236, (uint8_t)240, (uint8_t)18, (uint8_t)250, (uint8_t)150, (uint8_t)227, (uint8_t)78, (uint8_t)159, (uint8_t)76, (uint8_t)64, (uint8_t)169, (uint8_t)42, (uint8_t)119, (uint8_t)63, (uint8_t)251, (uint8_t)106, (uint8_t)22, (uint8_t)30, (uint8_t)3, (uint8_t)178, (uint8_t)190, (uint8_t)167, (uint8_t)238, (uint8_t)209, (uint8_t)19, (uint8_t)155, (uint8_t)32, (uint8_t)166, (uint8_t)25, (uint8_t)121, (uint8_t)207, (uint8_t)85, (uint8_t)208, (uint8_t)128, (uint8_t)90, (uint8_t)64, (uint8_t)194, (uint8_t)76, (uint8_t)144, (uint8_t)140, (uint8_t)159, (uint8_t)135, (uint8_t)208, (uint8_t)73, (uint8_t)152, (uint8_t)184, (uint8_t)212, (uint8_t)227, (uint8_t)159, (uint8_t)236, (uint8_t)6, (uint8_t)255, (uint8_t)37, (uint8_t)137, (uint8_t)163, (uint8_t)165, (uint8_t)159, (uint8_t)94, (uint8_t)9, (uint8_t)59, (uint8_t)240, (uint8_t)235, (uint8_t)106, (uint8_t)26, (uint8_t)26, (uint8_t)6, (uint8_t)57, (uint8_t)211, (uint8_t)137, (uint8_t)127, (uint8_t)85, (uint8_t)247, (uint8_t)184, (uint8_t)48, (uint8_t)161, (uint8_t)169, (uint8_t)31, (uint8_t)35, (uint8_t)31, (uint8_t)118, (uint8_t)60, (uint8_t)48, (uint8_t)228, (uint8_t)72, (uint8_t)156, (uint8_t)88, (uint8_t)115, (uint8_t)135, (uint8_t)223, (uint8_t)144, (uint8_t)96, (uint8_t)42, (uint8_t)152, (uint8_t)59, (uint8_t)172, (uint8_t)16, (uint8_t)212, (uint8_t)51, (uint8_t)36, (uint8_t)96, (uint8_t)40, (uint8_t)105, (uint8_t)77, (uint8_t)85, (uint8_t)120, (uint8_t)68, (uint8_t)58, (uint8_t)90, (uint8_t)40, (uint8_t)98, (uint8_t)183, (uint8_t)159, (uint8_t)7, (uint8_t)67, (uint8_t)23, (uint8_t)183, (uint8_t)163, (uint8_t)115, (uint8_t)85, (uint8_t)168, (uint8_t)120, (uint8_t)193, (uint8_t)4, (uint8_t)169, (uint8_t)48, (uint8_t)212, (uint8_t)6, (uint8_t)174, (uint8_t)3, (uint8_t)140, (uint8_t)20, (uint8_t)39, (uint8_t)119, (uint8_t)160, (uint8_t)120, (uint8_t)146, (uint8_t)198, (uint8_t)235, (uint8_t)189, (uint8_t)89, (uint8_t)61, (uint8_t)37, (uint8_t)63, (uint8_t)229, (uint8_t)60, (uint8_t)173, (uint8_t)204, (uint8_t)145, (uint8_t)174, (uint8_t)36, (uint8_t)63, (uint8_t)165, (uint8_t)93, (uint8_t)59, (uint8_t)203, (uint8_t)10, (uint8_t)179, (uint8_t)226, (uint8_t)121, (uint8_t)32, (uint8_t)183, (uint8_t)67, (uint8_t)144, (uint8_t)133, (uint8_t)132, (uint8_t)212, (uint8_t)252, (uint8_t)178, (uint8_t)176, (uint8_t)85, (uint8_t)213, (uint8_t)238, (uint8_t)69, (uint8_t)86, (uint8_t)12, (uint8_t)83, (uint8_t)141, (uint8_t)13, (uint8_t)9, (uint8_t)15, (uint8_t)56, (uint8_t)68, (uint8_t)74, (uint8_t)40, (uint8_t)24, (uint8_t)152, (uint8_t)216, (uint8_t)149, (uint8_t)59, (uint8_t)156, (uint8_t)31, (uint8_t)136, (uint8_t)119, (uint8_t)15, (uint8_t)135, (uint8_t)123, (uint8_t)254, (uint8_t)83, (uint8_t)213, (uint8_t)48, (uint8_t)157, (uint8_t)205, (uint8_t)220, (uint8_t)84, (uint8_t)171, (uint8_t)193, (uint8_t)55, (uint8_t)23, (uint8_t)245, (uint8_t)246, (uint8_t)188, (uint8_t)177, (uint8_t)142, (uint8_t)45, (uint8_t)124, (uint8_t)185};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_component_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
        p111_tc1_SET((int64_t) -1074397645646006785L, PH.base.pack) ;
        p111_ts1_SET((int64_t) -4503979501572626575L, PH.base.pack) ;
        c_CommunicationChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_seq_SET((uint32_t)2574819736L, PH.base.pack) ;
        p112_time_usec_SET((uint64_t)7523972220779639704L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
        p113_ve_SET((int16_t)(int16_t)6595, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)44578, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)33475, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
        p113_lon_SET((int32_t) -1139321670, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t)32418, PH.base.pack) ;
        p113_alt_SET((int32_t)1759643662, PH.base.pack) ;
        p113_lat_SET((int32_t)2016367475, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)41106, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t) -11795, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)2014320210638385258L, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)33986, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_sensor_id_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)4210302917L, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)4243594955L, PH.base.pack) ;
        p114_integrated_y_SET((float)5.4142163E37F, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)6059417121101003904L, PH.base.pack) ;
        p114_integrated_xgyro_SET((float) -1.0142576E38F, PH.base.pack) ;
        p114_integrated_ygyro_SET((float)2.2311883E37F, PH.base.pack) ;
        p114_distance_SET((float) -1.9346744E38F, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p114_integrated_zgyro_SET((float) -1.4736041E38F, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t) -1094, PH.base.pack) ;
        p114_integrated_x_SET((float) -2.0581642E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_ind_airspeed_SET((uint16_t)(uint16_t)9177, PH.base.pack) ;
        p115_yawspeed_SET((float)1.6720357E38F, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t)10371, PH.base.pack) ;
        p115_rollspeed_SET((float) -1.1068961E38F, PH.base.pack) ;
        p115_alt_SET((int32_t)1669851940, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t) -32595, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t) -11974, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t)883, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)7510, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)7843812968240807637L, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t) -3944, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t) -22450, PH.base.pack) ;
        p115_pitchspeed_SET((float)3.3829465E38F, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {-2.0536662E38F, 2.4515257E38F, 3.047777E38F, -6.0362844E37F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_lon_SET((int32_t)1225589712, PH.base.pack) ;
        p115_lat_SET((int32_t) -1810980656, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
        p116_xmag_SET((int16_t)(int16_t) -26040, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t)1573, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)2625390658L, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t)6131, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t) -27819, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t) -17810, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t)25997, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t)4707, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t)20766, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t) -2109, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_start_SET((uint16_t)(uint16_t)55743, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        p117_end_SET((uint16_t)(uint16_t)18937, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
        p118_num_logs_SET((uint16_t)(uint16_t)41435, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)34580, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)519077390L, PH.base.pack) ;
        p118_size_SET((uint32_t)1698359049L, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)43475, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_id_SET((uint16_t)(uint16_t)60398, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        p119_ofs_SET((uint32_t)3195580485L, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p119_count_SET((uint32_t)315230903L, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
        p120_id_SET((uint16_t)(uint16_t)52632, PH.base.pack) ;
        p120_ofs_SET((uint32_t)3173452605L, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)106, (uint8_t)79, (uint8_t)24, (uint8_t)215, (uint8_t)130, (uint8_t)40, (uint8_t)144, (uint8_t)24, (uint8_t)8, (uint8_t)199, (uint8_t)87, (uint8_t)153, (uint8_t)34, (uint8_t)139, (uint8_t)210, (uint8_t)193, (uint8_t)122, (uint8_t)215, (uint8_t)176, (uint8_t)28, (uint8_t)115, (uint8_t)75, (uint8_t)233, (uint8_t)196, (uint8_t)232, (uint8_t)138, (uint8_t)78, (uint8_t)159, (uint8_t)32, (uint8_t)250, (uint8_t)193, (uint8_t)143, (uint8_t)107, (uint8_t)180, (uint8_t)205, (uint8_t)201, (uint8_t)196, (uint8_t)113, (uint8_t)56, (uint8_t)77, (uint8_t)214, (uint8_t)196, (uint8_t)189, (uint8_t)110, (uint8_t)224, (uint8_t)75, (uint8_t)197, (uint8_t)21, (uint8_t)71, (uint8_t)191, (uint8_t)123, (uint8_t)196, (uint8_t)62, (uint8_t)122, (uint8_t)91, (uint8_t)189, (uint8_t)117, (uint8_t)57, (uint8_t)248, (uint8_t)50, (uint8_t)141, (uint8_t)89, (uint8_t)12, (uint8_t)12, (uint8_t)74, (uint8_t)48, (uint8_t)219, (uint8_t)22, (uint8_t)119, (uint8_t)112, (uint8_t)243, (uint8_t)170, (uint8_t)17, (uint8_t)36, (uint8_t)26, (uint8_t)1, (uint8_t)124, (uint8_t)21, (uint8_t)115, (uint8_t)74, (uint8_t)37, (uint8_t)227, (uint8_t)93, (uint8_t)189, (uint8_t)62, (uint8_t)141, (uint8_t)133, (uint8_t)161, (uint8_t)193, (uint8_t)234};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        p120_count_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
        p121_target_system_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
        p121_target_component_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_system_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p122_target_component_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)21, (uint8_t)245, (uint8_t)101, (uint8_t)163, (uint8_t)168, (uint8_t)212, (uint8_t)201, (uint8_t)168, (uint8_t)164, (uint8_t)72, (uint8_t)145, (uint8_t)166, (uint8_t)2, (uint8_t)34, (uint8_t)116, (uint8_t)211, (uint8_t)126, (uint8_t)48, (uint8_t)134, (uint8_t)30, (uint8_t)115, (uint8_t)104, (uint8_t)64, (uint8_t)252, (uint8_t)188, (uint8_t)152, (uint8_t)128, (uint8_t)199, (uint8_t)76, (uint8_t)38, (uint8_t)117, (uint8_t)128, (uint8_t)64, (uint8_t)7, (uint8_t)162, (uint8_t)112, (uint8_t)163, (uint8_t)9, (uint8_t)4, (uint8_t)68, (uint8_t)211, (uint8_t)127, (uint8_t)113, (uint8_t)226, (uint8_t)239, (uint8_t)77, (uint8_t)21, (uint8_t)115, (uint8_t)166, (uint8_t)50, (uint8_t)22, (uint8_t)196, (uint8_t)231, (uint8_t)14, (uint8_t)140, (uint8_t)93, (uint8_t)15, (uint8_t)197, (uint8_t)144, (uint8_t)238, (uint8_t)30, (uint8_t)179, (uint8_t)94, (uint8_t)142, (uint8_t)248, (uint8_t)166, (uint8_t)124, (uint8_t)108, (uint8_t)43, (uint8_t)49, (uint8_t)6, (uint8_t)202, (uint8_t)75, (uint8_t)99, (uint8_t)13, (uint8_t)114, (uint8_t)250, (uint8_t)41, (uint8_t)161, (uint8_t)134, (uint8_t)233, (uint8_t)188, (uint8_t)187, (uint8_t)190, (uint8_t)58, (uint8_t)76, (uint8_t)213, (uint8_t)184, (uint8_t)60, (uint8_t)14, (uint8_t)86, (uint8_t)79, (uint8_t)184, (uint8_t)14, (uint8_t)185, (uint8_t)188, (uint8_t)204, (uint8_t)35, (uint8_t)171, (uint8_t)248, (uint8_t)118, (uint8_t)41, (uint8_t)219, (uint8_t)206, (uint8_t)205, (uint8_t)21, (uint8_t)228, (uint8_t)58, (uint8_t)215, (uint8_t)172};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_target_component_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        p123_len_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p123_target_system_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)34265, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)37161, PH.base.pack) ;
        p124_lat_SET((int32_t)327628374, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)49005, PH.base.pack) ;
        p124_lon_SET((int32_t) -1214939568, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)8409079341114938581L, PH.base.pack) ;
        p124_alt_SET((int32_t)1824737713, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)21515, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)924723069L, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
        p125_Vservo_SET((uint16_t)(uint16_t)39298, PH.base.pack) ;
        p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT, PH.base.pack) ;
        p125_Vcc_SET((uint16_t)(uint16_t)34879, PH.base.pack) ;
        c_CommunicationChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_timeout_SET((uint16_t)(uint16_t)27880, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)214, (uint8_t)187, (uint8_t)112, (uint8_t)67, (uint8_t)73, (uint8_t)65, (uint8_t)90, (uint8_t)85, (uint8_t)104, (uint8_t)113, (uint8_t)17, (uint8_t)87, (uint8_t)20, (uint8_t)13, (uint8_t)169, (uint8_t)218, (uint8_t)255, (uint8_t)128, (uint8_t)216, (uint8_t)242, (uint8_t)23, (uint8_t)92, (uint8_t)142, (uint8_t)85, (uint8_t)66, (uint8_t)10, (uint8_t)236, (uint8_t)48, (uint8_t)145, (uint8_t)33, (uint8_t)0, (uint8_t)45, (uint8_t)180, (uint8_t)14, (uint8_t)252, (uint8_t)21, (uint8_t)253, (uint8_t)141, (uint8_t)99, (uint8_t)223, (uint8_t)170, (uint8_t)37, (uint8_t)96, (uint8_t)234, (uint8_t)92, (uint8_t)214, (uint8_t)144, (uint8_t)87, (uint8_t)195, (uint8_t)155, (uint8_t)177, (uint8_t)113, (uint8_t)235, (uint8_t)182, (uint8_t)54, (uint8_t)76, (uint8_t)212, (uint8_t)143, (uint8_t)196, (uint8_t)219, (uint8_t)229, (uint8_t)159, (uint8_t)73, (uint8_t)177, (uint8_t)102, (uint8_t)139, (uint8_t)69, (uint8_t)242, (uint8_t)163, (uint8_t)178};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2, PH.base.pack) ;
        p126_baudrate_SET((uint32_t)2389447527L, PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
        p127_baseline_b_mm_SET((int32_t) -740340532, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)33254, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)3826159446L, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)407696327L, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t) -1549676544, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t)1072032946, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t)1521052173, PH.base.pack) ;
        p127_tow_SET((uint32_t)3956015755L, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
        p128_tow_SET((uint32_t)2062685219L, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t) -1920082763, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t) -1585444943, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t) -775747667, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)4183668997L, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)4108065037L, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)32218, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t) -808789491, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
        p129_xacc_SET((int16_t)(int16_t)30988, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t)26659, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t)18839, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t)992, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t) -30815, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t) -28677, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)1437903733L, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t)23290, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t) -731, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t)29331, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_width_SET((uint16_t)(uint16_t)29514, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)12577, PH.base.pack) ;
        p130_size_SET((uint32_t)993204203L, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)7213, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
        p131_seqnr_SET((uint16_t)(uint16_t)65191, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)67, (uint8_t)195, (uint8_t)78, (uint8_t)138, (uint8_t)28, (uint8_t)176, (uint8_t)122, (uint8_t)37, (uint8_t)174, (uint8_t)147, (uint8_t)245, (uint8_t)0, (uint8_t)17, (uint8_t)29, (uint8_t)223, (uint8_t)207, (uint8_t)98, (uint8_t)185, (uint8_t)59, (uint8_t)192, (uint8_t)11, (uint8_t)145, (uint8_t)155, (uint8_t)1, (uint8_t)155, (uint8_t)69, (uint8_t)186, (uint8_t)169, (uint8_t)174, (uint8_t)22, (uint8_t)122, (uint8_t)36, (uint8_t)170, (uint8_t)37, (uint8_t)160, (uint8_t)3, (uint8_t)103, (uint8_t)248, (uint8_t)71, (uint8_t)143, (uint8_t)174, (uint8_t)216, (uint8_t)161, (uint8_t)207, (uint8_t)178, (uint8_t)76, (uint8_t)166, (uint8_t)36, (uint8_t)75, (uint8_t)172, (uint8_t)109, (uint8_t)164, (uint8_t)209, (uint8_t)166, (uint8_t)57, (uint8_t)40, (uint8_t)29, (uint8_t)95, (uint8_t)186, (uint8_t)51, (uint8_t)224, (uint8_t)210, (uint8_t)114, (uint8_t)205, (uint8_t)148, (uint8_t)188, (uint8_t)56, (uint8_t)43, (uint8_t)68, (uint8_t)69, (uint8_t)7, (uint8_t)44, (uint8_t)156, (uint8_t)253, (uint8_t)228, (uint8_t)6, (uint8_t)134, (uint8_t)176, (uint8_t)45, (uint8_t)6, (uint8_t)60, (uint8_t)176, (uint8_t)223, (uint8_t)20, (uint8_t)69, (uint8_t)223, (uint8_t)39, (uint8_t)209, (uint8_t)41, (uint8_t)134, (uint8_t)217, (uint8_t)208, (uint8_t)2, (uint8_t)67, (uint8_t)217, (uint8_t)242, (uint8_t)69, (uint8_t)211, (uint8_t)78, (uint8_t)229, (uint8_t)57, (uint8_t)63, (uint8_t)211, (uint8_t)42, (uint8_t)154, (uint8_t)22, (uint8_t)93, (uint8_t)76, (uint8_t)78, (uint8_t)39, (uint8_t)114, (uint8_t)129, (uint8_t)16, (uint8_t)0, (uint8_t)189, (uint8_t)243, (uint8_t)55, (uint8_t)210, (uint8_t)170, (uint8_t)218, (uint8_t)26, (uint8_t)197, (uint8_t)211, (uint8_t)233, (uint8_t)13, (uint8_t)208, (uint8_t)57, (uint8_t)20, (uint8_t)191, (uint8_t)55, (uint8_t)142, (uint8_t)247, (uint8_t)187, (uint8_t)209, (uint8_t)220, (uint8_t)69, (uint8_t)185, (uint8_t)142, (uint8_t)211, (uint8_t)145, (uint8_t)203, (uint8_t)194, (uint8_t)173, (uint8_t)166, (uint8_t)159, (uint8_t)59, (uint8_t)247, (uint8_t)175, (uint8_t)135, (uint8_t)97, (uint8_t)169, (uint8_t)237, (uint8_t)224, (uint8_t)176, (uint8_t)42, (uint8_t)190, (uint8_t)198, (uint8_t)203, (uint8_t)52, (uint8_t)187, (uint8_t)152, (uint8_t)99, (uint8_t)103, (uint8_t)12, (uint8_t)204, (uint8_t)118, (uint8_t)113, (uint8_t)192, (uint8_t)221, (uint8_t)191, (uint8_t)128, (uint8_t)171, (uint8_t)168, (uint8_t)100, (uint8_t)14, (uint8_t)247, (uint8_t)218, (uint8_t)171, (uint8_t)195, (uint8_t)44, (uint8_t)18, (uint8_t)121, (uint8_t)186, (uint8_t)177, (uint8_t)137, (uint8_t)226, (uint8_t)126, (uint8_t)227, (uint8_t)126, (uint8_t)195, (uint8_t)9, (uint8_t)8, (uint8_t)60, (uint8_t)47, (uint8_t)82, (uint8_t)49, (uint8_t)29, (uint8_t)204, (uint8_t)224, (uint8_t)6, (uint8_t)108, (uint8_t)181, (uint8_t)48, (uint8_t)115, (uint8_t)80, (uint8_t)21, (uint8_t)207, (uint8_t)182, (uint8_t)250, (uint8_t)240, (uint8_t)218, (uint8_t)89, (uint8_t)255, (uint8_t)75, (uint8_t)240, (uint8_t)196, (uint8_t)48, (uint8_t)62, (uint8_t)142, (uint8_t)228, (uint8_t)40, (uint8_t)17, (uint8_t)163, (uint8_t)80, (uint8_t)47, (uint8_t)221, (uint8_t)33, (uint8_t)54, (uint8_t)39, (uint8_t)204, (uint8_t)229, (uint8_t)64, (uint8_t)16, (uint8_t)126, (uint8_t)118, (uint8_t)147, (uint8_t)141, (uint8_t)95, (uint8_t)250, (uint8_t)49, (uint8_t)111, (uint8_t)115, (uint8_t)4, (uint8_t)163, (uint8_t)198, (uint8_t)230, (uint8_t)250, (uint8_t)39, (uint8_t)185, (uint8_t)163, (uint8_t)92, (uint8_t)4, (uint8_t)124};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_id_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)50042, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)51722, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)14543, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_YAW_45, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)1267960422L, PH.base.pack) ;
        c_CommunicationChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_mask_SET((uint64_t)6082779728082239696L, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)48333, PH.base.pack) ;
        p133_lon_SET((int32_t) -826887086, PH.base.pack) ;
        p133_lat_SET((int32_t) -209950872, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
        p134_grid_spacing_SET((uint16_t)(uint16_t)40403, PH.base.pack) ;
        p134_lon_SET((int32_t)1817922006, PH.base.pack) ;
        p134_gridbit_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        p134_lat_SET((int32_t)1209111922, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t) -25505, (int16_t) -26078, (int16_t)381, (int16_t) -31878, (int16_t)8913, (int16_t)18187, (int16_t) -29538, (int16_t)17916, (int16_t)32061, (int16_t)6934, (int16_t)24234, (int16_t)25784, (int16_t) -11780, (int16_t) -7239, (int16_t) -29844, (int16_t)19914};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lat_SET((int32_t)1991355880, PH.base.pack) ;
        p135_lon_SET((int32_t)362569806, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_loaded_SET((uint16_t)(uint16_t)46215, PH.base.pack) ;
        p136_lat_SET((int32_t)950020838, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)39496, PH.base.pack) ;
        p136_lon_SET((int32_t)1988798008, PH.base.pack) ;
        p136_current_height_SET((float)2.8193229E38F, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)11969, PH.base.pack) ;
        p136_terrain_height_SET((float)2.3784914E38F, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_temperature_SET((int16_t)(int16_t) -27648, PH.base.pack) ;
        p137_time_boot_ms_SET((uint32_t)142501613L, PH.base.pack) ;
        p137_press_abs_SET((float) -2.8014953E38F, PH.base.pack) ;
        p137_press_diff_SET((float) -2.2756995E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
        p138_x_SET((float)7.5147337E37F, PH.base.pack) ;
        {
            float q[] =  {1.1891355E38F, -3.3445432E38F, -5.5984136E37F, 2.598731E37F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_time_usec_SET((uint64_t)8409474775135957733L, PH.base.pack) ;
        p138_z_SET((float) -1.4701388E38F, PH.base.pack) ;
        p138_y_SET((float)1.4725488E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_group_mlx_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        p139_time_usec_SET((uint64_t)4227698352066896780L, PH.base.pack) ;
        {
            float controls[] =  {-2.0460351E38F, -2.6385028E38F, -2.9043023E38F, 1.7854013E38F, -2.9608913E38F, -2.774348E38F, -1.3605871E37F, 2.3192108E38F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_target_system_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        p139_target_component_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        {
            float controls[] =  {-3.0913299E38F, 1.1143582E38F, -1.0871971E38F, -2.7190203E38F, -1.1998448E38F, -1.3092109E38F, 4.9332656E37F, -2.2168404E38F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p140_group_mlx_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
        p140_time_usec_SET((uint64_t)661060580851391339L, PH.base.pack) ;
        c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ALTITUDE_141(), &PH);
        p141_altitude_monotonic_SET((float) -1.3929623E38F, PH.base.pack) ;
        p141_altitude_amsl_SET((float)1.6077256E38F, PH.base.pack) ;
        p141_altitude_local_SET((float)1.2399897E38F, PH.base.pack) ;
        p141_altitude_terrain_SET((float)3.1842848E38F, PH.base.pack) ;
        p141_altitude_relative_SET((float)9.161881E37F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)2143236324645149855L, PH.base.pack) ;
        p141_bottom_clearance_SET((float)2.5894333E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RESOURCE_REQUEST_142(), &PH);
        p142_uri_type_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        p142_request_id_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        {
            uint8_t storage[] =  {(uint8_t)109, (uint8_t)193, (uint8_t)100, (uint8_t)148, (uint8_t)84, (uint8_t)212, (uint8_t)197, (uint8_t)212, (uint8_t)66, (uint8_t)34, (uint8_t)126, (uint8_t)179, (uint8_t)63, (uint8_t)184, (uint8_t)42, (uint8_t)26, (uint8_t)88, (uint8_t)73, (uint8_t)145, (uint8_t)1, (uint8_t)139, (uint8_t)168, (uint8_t)72, (uint8_t)126, (uint8_t)72, (uint8_t)167, (uint8_t)156, (uint8_t)190, (uint8_t)6, (uint8_t)242, (uint8_t)127, (uint8_t)237, (uint8_t)213, (uint8_t)165, (uint8_t)210, (uint8_t)251, (uint8_t)65, (uint8_t)23, (uint8_t)145, (uint8_t)145, (uint8_t)133, (uint8_t)73, (uint8_t)59, (uint8_t)61, (uint8_t)62, (uint8_t)70, (uint8_t)27, (uint8_t)153, (uint8_t)148, (uint8_t)92, (uint8_t)211, (uint8_t)108, (uint8_t)224, (uint8_t)144, (uint8_t)169, (uint8_t)134, (uint8_t)108, (uint8_t)136, (uint8_t)241, (uint8_t)160, (uint8_t)80, (uint8_t)17, (uint8_t)160, (uint8_t)47, (uint8_t)243, (uint8_t)176, (uint8_t)184, (uint8_t)212, (uint8_t)82, (uint8_t)75, (uint8_t)154, (uint8_t)201, (uint8_t)71, (uint8_t)187, (uint8_t)169, (uint8_t)29, (uint8_t)148, (uint8_t)193, (uint8_t)125, (uint8_t)11, (uint8_t)0, (uint8_t)106, (uint8_t)64, (uint8_t)144, (uint8_t)109, (uint8_t)16, (uint8_t)54, (uint8_t)63, (uint8_t)162, (uint8_t)230, (uint8_t)160, (uint8_t)251, (uint8_t)105, (uint8_t)194, (uint8_t)171, (uint8_t)216, (uint8_t)183, (uint8_t)18, (uint8_t)60, (uint8_t)244, (uint8_t)140, (uint8_t)239, (uint8_t)168, (uint8_t)157, (uint8_t)44, (uint8_t)159, (uint8_t)10, (uint8_t)83, (uint8_t)225, (uint8_t)115, (uint8_t)231, (uint8_t)121, (uint8_t)35, (uint8_t)166, (uint8_t)143, (uint8_t)171, (uint8_t)97, (uint8_t)195, (uint8_t)162, (uint8_t)230};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        {
            uint8_t uri[] =  {(uint8_t)107, (uint8_t)62, (uint8_t)85, (uint8_t)48, (uint8_t)19, (uint8_t)236, (uint8_t)193, (uint8_t)205, (uint8_t)132, (uint8_t)240, (uint8_t)60, (uint8_t)127, (uint8_t)231, (uint8_t)195, (uint8_t)250, (uint8_t)200, (uint8_t)23, (uint8_t)35, (uint8_t)244, (uint8_t)220, (uint8_t)150, (uint8_t)131, (uint8_t)65, (uint8_t)69, (uint8_t)42, (uint8_t)47, (uint8_t)114, (uint8_t)128, (uint8_t)76, (uint8_t)232, (uint8_t)36, (uint8_t)80, (uint8_t)30, (uint8_t)214, (uint8_t)206, (uint8_t)128, (uint8_t)97, (uint8_t)64, (uint8_t)249, (uint8_t)216, (uint8_t)58, (uint8_t)36, (uint8_t)87, (uint8_t)255, (uint8_t)133, (uint8_t)116, (uint8_t)0, (uint8_t)184, (uint8_t)189, (uint8_t)92, (uint8_t)128, (uint8_t)210, (uint8_t)253, (uint8_t)248, (uint8_t)185, (uint8_t)129, (uint8_t)93, (uint8_t)175, (uint8_t)208, (uint8_t)62, (uint8_t)247, (uint8_t)94, (uint8_t)154, (uint8_t)194, (uint8_t)169, (uint8_t)191, (uint8_t)49, (uint8_t)159, (uint8_t)172, (uint8_t)62, (uint8_t)133, (uint8_t)45, (uint8_t)244, (uint8_t)104, (uint8_t)38, (uint8_t)189, (uint8_t)43, (uint8_t)196, (uint8_t)59, (uint8_t)62, (uint8_t)254, (uint8_t)24, (uint8_t)104, (uint8_t)149, (uint8_t)71, (uint8_t)67, (uint8_t)117, (uint8_t)235, (uint8_t)205, (uint8_t)217, (uint8_t)251, (uint8_t)50, (uint8_t)35, (uint8_t)58, (uint8_t)114, (uint8_t)123, (uint8_t)85, (uint8_t)195, (uint8_t)182, (uint8_t)29, (uint8_t)37, (uint8_t)45, (uint8_t)212, (uint8_t)158, (uint8_t)234, (uint8_t)219, (uint8_t)203, (uint8_t)112, (uint8_t)165, (uint8_t)223, (uint8_t)94, (uint8_t)185, (uint8_t)182, (uint8_t)144, (uint8_t)158, (uint8_t)165, (uint8_t)207, (uint8_t)88, (uint8_t)249, (uint8_t)11};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        p142_transfer_type_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        c_CommunicationChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_press_abs_SET((float)3.1304135E37F, PH.base.pack) ;
        p143_press_diff_SET((float)5.46746E36F, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)1157523802L, PH.base.pack) ;
        p143_temperature_SET((int16_t)(int16_t)7715, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FOLLOW_TARGET_144(), &PH);
        {
            float position_cov[] =  {3.6147346E37F, 9.872865E37F, 3.0164602E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        {
            float vel[] =  {4.7285377E36F, 2.7566556E37F, -1.9341726E38F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        {
            float rates[] =  {-3.222998E37F, -2.9452622E37F, 5.1835485E37F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        p144_est_capabilities_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
        {
            float attitude_q[] =  {-4.241236E37F, 1.7161782E38F, -9.343324E37F, 2.6561594E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        p144_lon_SET((int32_t) -692924682, PH.base.pack) ;
        p144_timestamp_SET((uint64_t)1568909963772582877L, PH.base.pack) ;
        {
            float acc[] =  {-1.883556E38F, 4.044618E37F, 2.875043E38F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        p144_alt_SET((float) -2.3858624E38F, PH.base.pack) ;
        p144_custom_state_SET((uint64_t)7664430270382174409L, PH.base.pack) ;
        p144_lat_SET((int32_t)998017586, PH.base.pack) ;
        c_CommunicationChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_x_pos_SET((float) -1.9288375E38F, PH.base.pack) ;
        p146_roll_rate_SET((float) -1.783759E38F, PH.base.pack) ;
        p146_z_pos_SET((float) -2.3488075E37F, PH.base.pack) ;
        p146_airspeed_SET((float) -8.863676E37F, PH.base.pack) ;
        p146_x_vel_SET((float)1.8112543E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {4.7025715E37F, 2.48962E38F, -6.9095867E37F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_x_acc_SET((float)2.5916753E38F, PH.base.pack) ;
        {
            float q[] =  {2.027015E38F, 2.9524974E38F, 2.374385E38F, 1.1899764E38F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_time_usec_SET((uint64_t)8831759570835792413L, PH.base.pack) ;
        p146_y_pos_SET((float)2.1224114E37F, PH.base.pack) ;
        p146_y_acc_SET((float) -1.9888423E37F, PH.base.pack) ;
        p146_y_vel_SET((float)2.9054515E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float)1.6090581E38F, PH.base.pack) ;
        p146_pitch_rate_SET((float) -2.8401908E38F, PH.base.pack) ;
        {
            float pos_variance[] =  {-2.7666464E38F, -2.629334E38F, 1.6348374E38F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_z_acc_SET((float) -2.7851304E38F, PH.base.pack) ;
        p146_z_vel_SET((float) -2.7355914E38F, PH.base.pack) ;
        c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_BATTERY_STATUS_147(), &PH);
        p147_current_consumed_SET((int32_t)1599460467, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t)72, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t) -5544, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t)700548000, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t)274, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)12488, (uint16_t)64949, (uint16_t)53511, (uint16_t)38656, (uint16_t)16543, (uint16_t)5882, (uint16_t)29444, (uint16_t)28913, (uint16_t)45448, (uint16_t)119};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD, PH.base.pack) ;
        c_CommunicationChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTOPILOT_VERSION_148(), &PH);
        p148_flight_sw_version_SET((uint32_t)98256008L, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)219, (uint8_t)245, (uint8_t)237, (uint8_t)18, (uint8_t)212, (uint8_t)24, (uint8_t)169, (uint8_t)213};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        p148_middleware_sw_version_SET((uint32_t)1362169386L, PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)245, (uint8_t)77, (uint8_t)115, (uint8_t)21, (uint8_t)210, (uint8_t)165, (uint8_t)254, (uint8_t)110};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        p148_product_id_SET((uint16_t)(uint16_t)52606, PH.base.pack) ;
        {
            uint8_t flight_custom_version[] =  {(uint8_t)157, (uint8_t)61, (uint8_t)230, (uint8_t)101, (uint8_t)119, (uint8_t)110, (uint8_t)244, (uint8_t)12};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_UNION, PH.base.pack) ;
        {
            uint8_t uid2[] =  {(uint8_t)192, (uint8_t)176, (uint8_t)124, (uint8_t)213, (uint8_t)189, (uint8_t)193, (uint8_t)132, (uint8_t)79, (uint8_t)144, (uint8_t)13, (uint8_t)228, (uint8_t)55, (uint8_t)8, (uint8_t)10, (uint8_t)199, (uint8_t)237, (uint8_t)93, (uint8_t)13};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_vendor_id_SET((uint16_t)(uint16_t)18163, PH.base.pack) ;
        p148_uid_SET((uint64_t)2060550490583120016L, PH.base.pack) ;
        p148_os_sw_version_SET((uint32_t)1952435554L, PH.base.pack) ;
        p148_board_version_SET((uint32_t)1865562068L, PH.base.pack) ;
        c_CommunicationChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LANDING_TARGET_149(), &PH);
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p149_target_num_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        {
            float q[] =  {-6.3727113E35F, 3.2469268E38F, -2.725903E38F, 1.4243398E38F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_size_y_SET((float) -8.128081E37F, PH.base.pack) ;
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, PH.base.pack) ;
        p149_size_x_SET((float)2.4800537E38F, PH.base.pack) ;
        p149_z_SET((float) -2.3940825E38F, &PH) ;
        p149_angle_y_SET((float) -3.6380523E37F, PH.base.pack) ;
        p149_time_usec_SET((uint64_t)682114256495233422L, PH.base.pack) ;
        p149_x_SET((float) -8.836036E37F, &PH) ;
        p149_position_valid_SET((uint8_t)(uint8_t)174, &PH) ;
        p149_distance_SET((float) -3.1468696E38F, PH.base.pack) ;
        p149_angle_x_SET((float)2.9192776E38F, PH.base.pack) ;
        p149_y_SET((float)2.8163116E38F, &PH) ;
        c_CommunicationChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENSOR_OFFSETS_150(), &PH);
        p150_accel_cal_z_SET((float) -3.0005215E38F, PH.base.pack) ;
        p150_gyro_cal_y_SET((float) -2.1859718E38F, PH.base.pack) ;
        p150_mag_ofs_x_SET((int16_t)(int16_t)14507, PH.base.pack) ;
        p150_gyro_cal_x_SET((float)1.9117508E38F, PH.base.pack) ;
        p150_raw_press_SET((int32_t) -567587202, PH.base.pack) ;
        p150_accel_cal_x_SET((float)3.4028147E38F, PH.base.pack) ;
        p150_mag_declination_SET((float) -7.462029E37F, PH.base.pack) ;
        p150_accel_cal_y_SET((float)1.2799672E38F, PH.base.pack) ;
        p150_gyro_cal_z_SET((float) -1.3649805E38F, PH.base.pack) ;
        p150_raw_temp_SET((int32_t) -969331691, PH.base.pack) ;
        p150_mag_ofs_z_SET((int16_t)(int16_t)19019, PH.base.pack) ;
        p150_mag_ofs_y_SET((int16_t)(int16_t) -9086, PH.base.pack) ;
        c_CommunicationChannel_on_SENSOR_OFFSETS_150(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_MAG_OFFSETS_151(), &PH);
        p151_target_component_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        p151_target_system_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        p151_mag_ofs_z_SET((int16_t)(int16_t) -10785, PH.base.pack) ;
        p151_mag_ofs_y_SET((int16_t)(int16_t) -23537, PH.base.pack) ;
        p151_mag_ofs_x_SET((int16_t)(int16_t)23784, PH.base.pack) ;
        c_CommunicationChannel_on_SET_MAG_OFFSETS_151(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MEMINFO_152(), &PH);
        p152_brkval_SET((uint16_t)(uint16_t)41161, PH.base.pack) ;
        p152_freemem32_SET((uint32_t)663973652L, &PH) ;
        p152_freemem_SET((uint16_t)(uint16_t)48464, PH.base.pack) ;
        c_CommunicationChannel_on_MEMINFO_152(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AP_ADC_153(), &PH);
        p153_adc2_SET((uint16_t)(uint16_t)13226, PH.base.pack) ;
        p153_adc4_SET((uint16_t)(uint16_t)7047, PH.base.pack) ;
        p153_adc6_SET((uint16_t)(uint16_t)21677, PH.base.pack) ;
        p153_adc5_SET((uint16_t)(uint16_t)53226, PH.base.pack) ;
        p153_adc1_SET((uint16_t)(uint16_t)6505, PH.base.pack) ;
        p153_adc3_SET((uint16_t)(uint16_t)2387, PH.base.pack) ;
        c_CommunicationChannel_on_AP_ADC_153(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DIGICAM_CONFIGURE_154(), &PH);
        p154_target_component_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p154_extra_param_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        p154_command_id_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p154_mode_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        p154_target_system_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p154_shutter_speed_SET((uint16_t)(uint16_t)57125, PH.base.pack) ;
        p154_exposure_type_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        p154_extra_value_SET((float)1.0332788E38F, PH.base.pack) ;
        p154_engine_cut_off_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p154_iso_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
        p154_aperture_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        c_CommunicationChannel_on_DIGICAM_CONFIGURE_154(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DIGICAM_CONTROL_155(), &PH);
        p155_extra_param_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        p155_zoom_pos_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p155_target_system_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p155_extra_value_SET((float) -2.8971163E38F, PH.base.pack) ;
        p155_target_component_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p155_focus_lock_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
        p155_session_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        p155_shot_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p155_command_id_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        p155_zoom_step_SET((int8_t)(int8_t)41, PH.base.pack) ;
        c_CommunicationChannel_on_DIGICAM_CONTROL_155(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_CONFIGURE_156(), &PH);
        p156_target_system_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p156_stab_roll_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        p156_mount_mode_SET(e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_NEUTRAL, PH.base.pack) ;
        p156_stab_yaw_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
        p156_stab_pitch_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
        p156_target_component_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_CONFIGURE_156(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_CONTROL_157(), &PH);
        p157_input_a_SET((int32_t)167750892, PH.base.pack) ;
        p157_input_c_SET((int32_t) -470244822, PH.base.pack) ;
        p157_input_b_SET((int32_t)1133727391, PH.base.pack) ;
        p157_target_system_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        p157_target_component_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        p157_save_position_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_CONTROL_157(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_STATUS_158(), &PH);
        p158_pointing_a_SET((int32_t)278412514, PH.base.pack) ;
        p158_pointing_b_SET((int32_t) -659029092, PH.base.pack) ;
        p158_pointing_c_SET((int32_t)1237361865, PH.base.pack) ;
        p158_target_system_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p158_target_component_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_STATUS_158(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FENCE_POINT_160(), &PH);
        p160_target_system_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        p160_target_component_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p160_lat_SET((float)2.6523684E38F, PH.base.pack) ;
        p160_count_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        p160_idx_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p160_lng_SET((float) -2.9939031E38F, PH.base.pack) ;
        c_CommunicationChannel_on_FENCE_POINT_160(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FENCE_FETCH_POINT_161(), &PH);
        p161_target_system_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p161_idx_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p161_target_component_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        c_CommunicationChannel_on_FENCE_FETCH_POINT_161(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FENCE_STATUS_162(), &PH);
        p162_breach_count_SET((uint16_t)(uint16_t)60320, PH.base.pack) ;
        p162_breach_type_SET(e_FENCE_BREACH_FENCE_BREACH_BOUNDARY, PH.base.pack) ;
        p162_breach_time_SET((uint32_t)3672566917L, PH.base.pack) ;
        p162_breach_status_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
        c_CommunicationChannel_on_FENCE_STATUS_162(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AHRS_163(), &PH);
        p163_omegaIy_SET((float)2.3252622E38F, PH.base.pack) ;
        p163_omegaIx_SET((float)2.079657E38F, PH.base.pack) ;
        p163_omegaIz_SET((float)2.6220113E38F, PH.base.pack) ;
        p163_renorm_val_SET((float)3.312148E38F, PH.base.pack) ;
        p163_error_yaw_SET((float) -5.7865005E37F, PH.base.pack) ;
        p163_error_rp_SET((float) -1.2217591E38F, PH.base.pack) ;
        p163_accel_weight_SET((float)2.9375032E38F, PH.base.pack) ;
        c_CommunicationChannel_on_AHRS_163(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SIMSTATE_164(), &PH);
        p164_ygyro_SET((float) -1.187554E38F, PH.base.pack) ;
        p164_zgyro_SET((float)2.1796135E38F, PH.base.pack) ;
        p164_yacc_SET((float)1.3844797E38F, PH.base.pack) ;
        p164_yaw_SET((float) -1.4853159E38F, PH.base.pack) ;
        p164_lng_SET((int32_t)1221361565, PH.base.pack) ;
        p164_lat_SET((int32_t) -13860839, PH.base.pack) ;
        p164_zacc_SET((float)2.8230082E38F, PH.base.pack) ;
        p164_roll_SET((float)2.109632E38F, PH.base.pack) ;
        p164_xgyro_SET((float) -2.782022E38F, PH.base.pack) ;
        p164_pitch_SET((float)2.210673E38F, PH.base.pack) ;
        p164_xacc_SET((float) -1.2955067E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SIMSTATE_164(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HWSTATUS_165(), &PH);
        p165_I2Cerr_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
        p165_Vcc_SET((uint16_t)(uint16_t)59699, PH.base.pack) ;
        c_CommunicationChannel_on_HWSTATUS_165(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_RADIO_166(), &PH);
        p166_fixed__SET((uint16_t)(uint16_t)48602, PH.base.pack) ;
        p166_noise_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p166_remnoise_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        p166_remrssi_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        p166_txbuf_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p166_rxerrors_SET((uint16_t)(uint16_t)21420, PH.base.pack) ;
        p166_rssi_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        c_CommunicationChannel_on_RADIO_166(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LIMITS_STATUS_167(), &PH);
        p167_last_trigger_SET((uint32_t)17897427L, PH.base.pack) ;
        p167_last_clear_SET((uint32_t)19725738L, PH.base.pack) ;
        p167_mods_required_SET(e_LIMIT_MODULE_LIMIT_ALTITUDE, PH.base.pack) ;
        p167_limits_state_SET(e_LIMITS_STATE_LIMITS_DISABLED, PH.base.pack) ;
        p167_breach_count_SET((uint16_t)(uint16_t)19168, PH.base.pack) ;
        p167_mods_enabled_SET(e_LIMIT_MODULE_LIMIT_GEOFENCE, PH.base.pack) ;
        p167_mods_triggered_SET(e_LIMIT_MODULE_LIMIT_ALTITUDE, PH.base.pack) ;
        p167_last_recovery_SET((uint32_t)2214965221L, PH.base.pack) ;
        p167_last_action_SET((uint32_t)2570883566L, PH.base.pack) ;
        c_CommunicationChannel_on_LIMITS_STATUS_167(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIND_168(), &PH);
        p168_speed_z_SET((float)9.038563E37F, PH.base.pack) ;
        p168_direction_SET((float)1.1393186E38F, PH.base.pack) ;
        p168_speed_SET((float)7.412861E37F, PH.base.pack) ;
        c_CommunicationChannel_on_WIND_168(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DATA16_169(), &PH);
        p169_len_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)232, (uint8_t)239, (uint8_t)215, (uint8_t)115, (uint8_t)45, (uint8_t)241, (uint8_t)162, (uint8_t)141, (uint8_t)105, (uint8_t)254, (uint8_t)49, (uint8_t)47, (uint8_t)51, (uint8_t)153, (uint8_t)236, (uint8_t)146};
            p169_data__SET(&data_, 0, PH.base.pack) ;
        }
        p169_type_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
        c_CommunicationChannel_on_DATA16_169(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DATA32_170(), &PH);
        p170_type_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        p170_len_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)181, (uint8_t)215, (uint8_t)229, (uint8_t)70, (uint8_t)153, (uint8_t)152, (uint8_t)164, (uint8_t)181, (uint8_t)54, (uint8_t)164, (uint8_t)248, (uint8_t)49, (uint8_t)32, (uint8_t)153, (uint8_t)134, (uint8_t)111, (uint8_t)49, (uint8_t)232, (uint8_t)106, (uint8_t)65, (uint8_t)63, (uint8_t)227, (uint8_t)111, (uint8_t)182, (uint8_t)240, (uint8_t)193, (uint8_t)88, (uint8_t)38, (uint8_t)100, (uint8_t)148, (uint8_t)169, (uint8_t)220};
            p170_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_DATA32_170(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DATA64_171(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)214, (uint8_t)85, (uint8_t)98, (uint8_t)0, (uint8_t)141, (uint8_t)20, (uint8_t)113, (uint8_t)213, (uint8_t)101, (uint8_t)22, (uint8_t)232, (uint8_t)23, (uint8_t)76, (uint8_t)125, (uint8_t)60, (uint8_t)246, (uint8_t)51, (uint8_t)0, (uint8_t)255, (uint8_t)109, (uint8_t)117, (uint8_t)63, (uint8_t)16, (uint8_t)254, (uint8_t)109, (uint8_t)120, (uint8_t)220, (uint8_t)248, (uint8_t)123, (uint8_t)61, (uint8_t)178, (uint8_t)94, (uint8_t)198, (uint8_t)198, (uint8_t)174, (uint8_t)242, (uint8_t)100, (uint8_t)143, (uint8_t)199, (uint8_t)113, (uint8_t)162, (uint8_t)68, (uint8_t)71, (uint8_t)21, (uint8_t)9, (uint8_t)79, (uint8_t)11, (uint8_t)229, (uint8_t)17, (uint8_t)152, (uint8_t)2, (uint8_t)220, (uint8_t)68, (uint8_t)114, (uint8_t)245, (uint8_t)20, (uint8_t)190, (uint8_t)1, (uint8_t)194, (uint8_t)28, (uint8_t)199, (uint8_t)51, (uint8_t)30, (uint8_t)177};
            p171_data__SET(&data_, 0, PH.base.pack) ;
        }
        p171_len_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        p171_type_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        c_CommunicationChannel_on_DATA64_171(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DATA96_172(), &PH);
        p172_type_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)188, (uint8_t)96, (uint8_t)185, (uint8_t)241, (uint8_t)110, (uint8_t)224, (uint8_t)126, (uint8_t)4, (uint8_t)8, (uint8_t)169, (uint8_t)58, (uint8_t)157, (uint8_t)85, (uint8_t)54, (uint8_t)45, (uint8_t)21, (uint8_t)197, (uint8_t)31, (uint8_t)48, (uint8_t)143, (uint8_t)41, (uint8_t)24, (uint8_t)255, (uint8_t)43, (uint8_t)202, (uint8_t)233, (uint8_t)37, (uint8_t)215, (uint8_t)222, (uint8_t)198, (uint8_t)100, (uint8_t)82, (uint8_t)29, (uint8_t)176, (uint8_t)151, (uint8_t)250, (uint8_t)197, (uint8_t)179, (uint8_t)39, (uint8_t)55, (uint8_t)4, (uint8_t)115, (uint8_t)153, (uint8_t)83, (uint8_t)127, (uint8_t)34, (uint8_t)86, (uint8_t)161, (uint8_t)37, (uint8_t)189, (uint8_t)65, (uint8_t)93, (uint8_t)145, (uint8_t)195, (uint8_t)44, (uint8_t)221, (uint8_t)60, (uint8_t)232, (uint8_t)27, (uint8_t)224, (uint8_t)230, (uint8_t)207, (uint8_t)128, (uint8_t)207, (uint8_t)22, (uint8_t)85, (uint8_t)19, (uint8_t)181, (uint8_t)132, (uint8_t)163, (uint8_t)33, (uint8_t)94, (uint8_t)169, (uint8_t)245, (uint8_t)223, (uint8_t)56, (uint8_t)83, (uint8_t)202, (uint8_t)214, (uint8_t)190, (uint8_t)13, (uint8_t)30, (uint8_t)28, (uint8_t)201, (uint8_t)133, (uint8_t)170, (uint8_t)77, (uint8_t)78, (uint8_t)129, (uint8_t)160, (uint8_t)133, (uint8_t)68, (uint8_t)247, (uint8_t)162, (uint8_t)11, (uint8_t)247};
            p172_data__SET(&data_, 0, PH.base.pack) ;
        }
        p172_len_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        c_CommunicationChannel_on_DATA96_172(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_RANGEFINDER_173(), &PH);
        p173_distance_SET((float) -8.847356E37F, PH.base.pack) ;
        p173_voltage_SET((float)1.2420648E38F, PH.base.pack) ;
        c_CommunicationChannel_on_RANGEFINDER_173(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AIRSPEED_AUTOCAL_174(), &PH);
        p174_vy_SET((float) -2.8929529E38F, PH.base.pack) ;
        p174_EAS2TAS_SET((float)1.5063874E37F, PH.base.pack) ;
        p174_vx_SET((float)2.1135506E38F, PH.base.pack) ;
        p174_Pax_SET((float) -2.0524077E38F, PH.base.pack) ;
        p174_ratio_SET((float)3.2384214E38F, PH.base.pack) ;
        p174_Pcz_SET((float)9.992568E37F, PH.base.pack) ;
        p174_state_y_SET((float)3.1902632E38F, PH.base.pack) ;
        p174_state_z_SET((float)3.2555922E37F, PH.base.pack) ;
        p174_vz_SET((float) -1.0121591E38F, PH.base.pack) ;
        p174_Pby_SET((float) -3.2710322E38F, PH.base.pack) ;
        p174_state_x_SET((float)1.026782E38F, PH.base.pack) ;
        p174_diff_pressure_SET((float)1.3840495E38F, PH.base.pack) ;
        c_CommunicationChannel_on_AIRSPEED_AUTOCAL_174(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_RALLY_POINT_175(), &PH);
        p175_lat_SET((int32_t) -107140519, PH.base.pack) ;
        p175_alt_SET((int16_t)(int16_t)2485, PH.base.pack) ;
        p175_lng_SET((int32_t) -1757734763, PH.base.pack) ;
        p175_land_dir_SET((uint16_t)(uint16_t)63471, PH.base.pack) ;
        p175_break_alt_SET((int16_t)(int16_t) -24961, PH.base.pack) ;
        p175_target_system_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
        p175_count_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        p175_target_component_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p175_idx_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p175_flags_SET(e_RALLY_FLAGS_FAVORABLE_WIND, PH.base.pack) ;
        c_CommunicationChannel_on_RALLY_POINT_175(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_RALLY_FETCH_POINT_176(), &PH);
        p176_target_system_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p176_idx_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p176_target_component_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        c_CommunicationChannel_on_RALLY_FETCH_POINT_176(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_COMPASSMOT_STATUS_177(), &PH);
        p177_CompensationX_SET((float) -7.1497005E37F, PH.base.pack) ;
        p177_throttle_SET((uint16_t)(uint16_t)12703, PH.base.pack) ;
        p177_CompensationY_SET((float)1.3884438E38F, PH.base.pack) ;
        p177_CompensationZ_SET((float) -2.8568315E38F, PH.base.pack) ;
        p177_interference_SET((uint16_t)(uint16_t)51091, PH.base.pack) ;
        p177_current_SET((float)5.7454463E37F, PH.base.pack) ;
        c_CommunicationChannel_on_COMPASSMOT_STATUS_177(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AHRS2_178(), &PH);
        p178_roll_SET((float)2.9456784E38F, PH.base.pack) ;
        p178_lat_SET((int32_t)1059753037, PH.base.pack) ;
        p178_altitude_SET((float) -1.0814306E38F, PH.base.pack) ;
        p178_yaw_SET((float)2.4026428E37F, PH.base.pack) ;
        p178_lng_SET((int32_t)1481700438, PH.base.pack) ;
        p178_pitch_SET((float) -8.11421E37F, PH.base.pack) ;
        c_CommunicationChannel_on_AHRS2_178(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_STATUS_179(), &PH);
        p179_time_usec_SET((uint64_t)1369099398011732729L, PH.base.pack) ;
        p179_p1_SET((float) -2.2114112E38F, PH.base.pack) ;
        p179_p4_SET((float)2.849375E38F, PH.base.pack) ;
        p179_target_system_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
        p179_p3_SET((float)9.869209E37F, PH.base.pack) ;
        p179_event_id_SET(e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_LOWSTORE, PH.base.pack) ;
        p179_p2_SET((float)2.1029585E38F, PH.base.pack) ;
        p179_img_idx_SET((uint16_t)(uint16_t)34700, PH.base.pack) ;
        p179_cam_idx_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_STATUS_179(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_FEEDBACK_180(), &PH);
        p180_flags_SET(e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_OPENLOOP, PH.base.pack) ;
        p180_pitch_SET((float) -1.6304319E37F, PH.base.pack) ;
        p180_lng_SET((int32_t) -753035525, PH.base.pack) ;
        p180_roll_SET((float)2.8266892E38F, PH.base.pack) ;
        p180_alt_msl_SET((float) -9.285432E37F, PH.base.pack) ;
        p180_lat_SET((int32_t)12310009, PH.base.pack) ;
        p180_time_usec_SET((uint64_t)4126725099677240666L, PH.base.pack) ;
        p180_target_system_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p180_foc_len_SET((float)1.2209146E38F, PH.base.pack) ;
        p180_img_idx_SET((uint16_t)(uint16_t)54263, PH.base.pack) ;
        p180_alt_rel_SET((float)2.4836744E38F, PH.base.pack) ;
        p180_yaw_SET((float) -8.22784E37F, PH.base.pack) ;
        p180_cam_idx_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_FEEDBACK_180(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BATTERY2_181(), &PH);
        p181_current_battery_SET((int16_t)(int16_t) -30170, PH.base.pack) ;
        p181_voltage_SET((uint16_t)(uint16_t)1763, PH.base.pack) ;
        c_CommunicationChannel_on_BATTERY2_181(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AHRS3_182(), &PH);
        p182_lng_SET((int32_t)433182698, PH.base.pack) ;
        p182_yaw_SET((float)1.7979948E38F, PH.base.pack) ;
        p182_altitude_SET((float) -2.4425108E38F, PH.base.pack) ;
        p182_v4_SET((float)3.447434E36F, PH.base.pack) ;
        p182_v1_SET((float) -2.7842406E38F, PH.base.pack) ;
        p182_pitch_SET((float) -3.4014252E38F, PH.base.pack) ;
        p182_v2_SET((float)1.3667943E38F, PH.base.pack) ;
        p182_roll_SET((float) -8.3960946E37F, PH.base.pack) ;
        p182_lat_SET((int32_t)2125659037, PH.base.pack) ;
        p182_v3_SET((float)3.0828476E38F, PH.base.pack) ;
        c_CommunicationChannel_on_AHRS3_182(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AUTOPILOT_VERSION_REQUEST_183(), &PH);
        p183_target_system_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        p183_target_component_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        c_CommunicationChannel_on_AUTOPILOT_VERSION_REQUEST_183(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_REMOTE_LOG_DATA_BLOCK_184(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)27, (uint8_t)97, (uint8_t)11, (uint8_t)129, (uint8_t)42, (uint8_t)156, (uint8_t)84, (uint8_t)36, (uint8_t)245, (uint8_t)221, (uint8_t)245, (uint8_t)176, (uint8_t)115, (uint8_t)38, (uint8_t)46, (uint8_t)20, (uint8_t)165, (uint8_t)231, (uint8_t)19, (uint8_t)78, (uint8_t)46, (uint8_t)116, (uint8_t)202, (uint8_t)23, (uint8_t)43, (uint8_t)64, (uint8_t)3, (uint8_t)32, (uint8_t)112, (uint8_t)130, (uint8_t)7, (uint8_t)148, (uint8_t)214, (uint8_t)71, (uint8_t)66, (uint8_t)119, (uint8_t)230, (uint8_t)200, (uint8_t)60, (uint8_t)39, (uint8_t)61, (uint8_t)179, (uint8_t)205, (uint8_t)126, (uint8_t)56, (uint8_t)238, (uint8_t)38, (uint8_t)102, (uint8_t)18, (uint8_t)74, (uint8_t)150, (uint8_t)231, (uint8_t)85, (uint8_t)247, (uint8_t)165, (uint8_t)47, (uint8_t)115, (uint8_t)243, (uint8_t)134, (uint8_t)172, (uint8_t)143, (uint8_t)7, (uint8_t)217, (uint8_t)140, (uint8_t)172, (uint8_t)198, (uint8_t)122, (uint8_t)82, (uint8_t)3, (uint8_t)115, (uint8_t)59, (uint8_t)2, (uint8_t)198, (uint8_t)2, (uint8_t)132, (uint8_t)49, (uint8_t)149, (uint8_t)1, (uint8_t)89, (uint8_t)92, (uint8_t)63, (uint8_t)10, (uint8_t)128, (uint8_t)101, (uint8_t)151, (uint8_t)0, (uint8_t)208, (uint8_t)192, (uint8_t)21, (uint8_t)189, (uint8_t)18, (uint8_t)148, (uint8_t)70, (uint8_t)181, (uint8_t)138, (uint8_t)153, (uint8_t)13, (uint8_t)111, (uint8_t)14, (uint8_t)69, (uint8_t)99, (uint8_t)215, (uint8_t)155, (uint8_t)30, (uint8_t)68, (uint8_t)163, (uint8_t)202, (uint8_t)175, (uint8_t)123, (uint8_t)183, (uint8_t)144, (uint8_t)176, (uint8_t)150, (uint8_t)107, (uint8_t)78, (uint8_t)150, (uint8_t)148, (uint8_t)228, (uint8_t)239, (uint8_t)3, (uint8_t)25, (uint8_t)20, (uint8_t)142, (uint8_t)152, (uint8_t)160, (uint8_t)63, (uint8_t)134, (uint8_t)205, (uint8_t)240, (uint8_t)216, (uint8_t)28, (uint8_t)91, (uint8_t)214, (uint8_t)132, (uint8_t)39, (uint8_t)55, (uint8_t)7, (uint8_t)202, (uint8_t)162, (uint8_t)134, (uint8_t)63, (uint8_t)99, (uint8_t)226, (uint8_t)116, (uint8_t)99, (uint8_t)138, (uint8_t)60, (uint8_t)110, (uint8_t)107, (uint8_t)83, (uint8_t)145, (uint8_t)40, (uint8_t)185, (uint8_t)168, (uint8_t)98, (uint8_t)92, (uint8_t)230, (uint8_t)94, (uint8_t)77, (uint8_t)228, (uint8_t)12, (uint8_t)228, (uint8_t)110, (uint8_t)246, (uint8_t)222, (uint8_t)179, (uint8_t)197, (uint8_t)24, (uint8_t)252, (uint8_t)190, (uint8_t)7, (uint8_t)155, (uint8_t)205, (uint8_t)53, (uint8_t)194, (uint8_t)173, (uint8_t)183, (uint8_t)228, (uint8_t)53, (uint8_t)128, (uint8_t)69, (uint8_t)198, (uint8_t)5, (uint8_t)76, (uint8_t)238, (uint8_t)245, (uint8_t)227, (uint8_t)47, (uint8_t)172, (uint8_t)134, (uint8_t)6, (uint8_t)209, (uint8_t)246, (uint8_t)167, (uint8_t)222, (uint8_t)22, (uint8_t)244, (uint8_t)238, (uint8_t)94, (uint8_t)150};
            p184_data__SET(&data_, 0, PH.base.pack) ;
        }
        p184_target_component_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p184_seqno_SET(e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_STOP, PH.base.pack) ;
        p184_target_system_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
        c_CommunicationChannel_on_REMOTE_LOG_DATA_BLOCK_184(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_REMOTE_LOG_BLOCK_STATUS_185(), &PH);
        p185_seqno_SET((uint32_t)1520469425L, PH.base.pack) ;
        p185_target_component_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p185_target_system_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        p185_status_SET(e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_NACK, PH.base.pack) ;
        c_CommunicationChannel_on_REMOTE_LOG_BLOCK_STATUS_185(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LED_CONTROL_186(), &PH);
        p186_custom_len_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p186_target_component_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        p186_pattern_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        p186_instance_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        {
            uint8_t custom_bytes[] =  {(uint8_t)251, (uint8_t)164, (uint8_t)2, (uint8_t)101, (uint8_t)18, (uint8_t)87, (uint8_t)151, (uint8_t)161, (uint8_t)68, (uint8_t)232, (uint8_t)137, (uint8_t)40, (uint8_t)31, (uint8_t)230, (uint8_t)20, (uint8_t)44, (uint8_t)230, (uint8_t)44, (uint8_t)45, (uint8_t)228, (uint8_t)253, (uint8_t)79, (uint8_t)42, (uint8_t)61};
            p186_custom_bytes_SET(&custom_bytes, 0, PH.base.pack) ;
        }
        p186_target_system_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
        c_CommunicationChannel_on_LED_CONTROL_186(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MAG_CAL_PROGRESS_191(), &PH);
        p191_cal_status_SET(e_MAG_CAL_STATUS_MAG_CAL_WAITING_TO_START, PH.base.pack) ;
        p191_compass_id_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        p191_direction_z_SET((float)1.612769E38F, PH.base.pack) ;
        p191_attempt_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
        p191_completion_pct_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        p191_direction_y_SET((float)4.6060967E37F, PH.base.pack) ;
        {
            uint8_t completion_mask[] =  {(uint8_t)142, (uint8_t)105, (uint8_t)30, (uint8_t)174, (uint8_t)239, (uint8_t)167, (uint8_t)34, (uint8_t)218, (uint8_t)34, (uint8_t)123};
            p191_completion_mask_SET(&completion_mask, 0, PH.base.pack) ;
        }
        p191_direction_x_SET((float)3.1144338E38F, PH.base.pack) ;
        p191_cal_mask_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        c_CommunicationChannel_on_MAG_CAL_PROGRESS_191(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MAG_CAL_REPORT_192(), &PH);
        p192_ofs_y_SET((float)4.307279E37F, PH.base.pack) ;
        p192_diag_z_SET((float)1.3654549E38F, PH.base.pack) ;
        p192_offdiag_y_SET((float) -3.1056967E38F, PH.base.pack) ;
        p192_compass_id_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        p192_offdiag_x_SET((float)8.75737E37F, PH.base.pack) ;
        p192_offdiag_z_SET((float)1.9967588E38F, PH.base.pack) ;
        p192_cal_status_SET(e_MAG_CAL_STATUS_MAG_CAL_NOT_STARTED, PH.base.pack) ;
        p192_diag_y_SET((float) -2.8261203E38F, PH.base.pack) ;
        p192_autosaved_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        p192_fitness_SET((float) -2.3420279E38F, PH.base.pack) ;
        p192_diag_x_SET((float) -2.5431902E38F, PH.base.pack) ;
        p192_cal_mask_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p192_ofs_x_SET((float)2.572909E38F, PH.base.pack) ;
        p192_ofs_z_SET((float) -2.4512328E38F, PH.base.pack) ;
        c_CommunicationChannel_on_MAG_CAL_REPORT_192(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_EKF_STATUS_REPORT_193(), &PH);
        p193_compass_variance_SET((float)2.6158382E38F, PH.base.pack) ;
        p193_pos_vert_variance_SET((float)5.0384893E37F, PH.base.pack) ;
        p193_velocity_variance_SET((float)1.1670139E38F, PH.base.pack) ;
        p193_pos_horiz_variance_SET((float) -2.7863382E38F, PH.base.pack) ;
        p193_terrain_alt_variance_SET((float) -2.7641125E38F, PH.base.pack) ;
        p193_flags_SET(e_EKF_STATUS_FLAGS_EKF_POS_HORIZ_ABS, PH.base.pack) ;
        c_CommunicationChannel_on_EKF_STATUS_REPORT_193(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PID_TUNING_194(), &PH);
        p194_P_SET((float)3.0020309E38F, PH.base.pack) ;
        p194_axis_SET(e_PID_TUNING_AXIS_PID_TUNING_LANDING, PH.base.pack) ;
        p194_achieved_SET((float)1.7247403E38F, PH.base.pack) ;
        p194_D_SET((float) -1.4351653E37F, PH.base.pack) ;
        p194_I_SET((float) -5.6601035E37F, PH.base.pack) ;
        p194_desired_SET((float)3.2457417E38F, PH.base.pack) ;
        p194_FF_SET((float)2.6934004E38F, PH.base.pack) ;
        c_CommunicationChannel_on_PID_TUNING_194(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GIMBAL_REPORT_200(), &PH);
        p200_delta_velocity_y_SET((float)6.886656E37F, PH.base.pack) ;
        p200_target_system_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p200_delta_angle_x_SET((float) -1.4955731E38F, PH.base.pack) ;
        p200_delta_angle_y_SET((float)3.2568968E38F, PH.base.pack) ;
        p200_delta_time_SET((float)3.023549E38F, PH.base.pack) ;
        p200_delta_angle_z_SET((float)2.3349217E38F, PH.base.pack) ;
        p200_joint_roll_SET((float)2.4386008E38F, PH.base.pack) ;
        p200_delta_velocity_x_SET((float)3.1297355E37F, PH.base.pack) ;
        p200_joint_el_SET((float) -2.6706597E38F, PH.base.pack) ;
        p200_delta_velocity_z_SET((float) -1.3848447E38F, PH.base.pack) ;
        p200_joint_az_SET((float) -2.0580664E38F, PH.base.pack) ;
        p200_target_component_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        c_CommunicationChannel_on_GIMBAL_REPORT_200(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GIMBAL_CONTROL_201(), &PH);
        p201_demanded_rate_x_SET((float) -3.7213757E37F, PH.base.pack) ;
        p201_demanded_rate_z_SET((float)2.2607092E38F, PH.base.pack) ;
        p201_target_system_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p201_target_component_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p201_demanded_rate_y_SET((float) -2.552407E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GIMBAL_CONTROL_201(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GIMBAL_TORQUE_CMD_REPORT_214(), &PH);
        p214_az_torque_cmd_SET((int16_t)(int16_t)8958, PH.base.pack) ;
        p214_el_torque_cmd_SET((int16_t)(int16_t)24124, PH.base.pack) ;
        p214_target_system_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p214_rl_torque_cmd_SET((int16_t)(int16_t)29066, PH.base.pack) ;
        p214_target_component_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        c_CommunicationChannel_on_GIMBAL_TORQUE_CMD_REPORT_214(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GOPRO_HEARTBEAT_215(), &PH);
        p215_capture_mode_SET(e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_MULTI_SHOT, PH.base.pack) ;
        p215_flags_SET(e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING, PH.base.pack) ;
        p215_status_SET(e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE, PH.base.pack) ;
        c_CommunicationChannel_on_GOPRO_HEARTBEAT_215(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GOPRO_GET_REQUEST_216(), &PH);
        p216_target_system_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        p216_target_component_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p216_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_SHUTTER, PH.base.pack) ;
        c_CommunicationChannel_on_GOPRO_GET_REQUEST_216(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GOPRO_GET_RESPONSE_217(), &PH);
        {
            uint8_t value[] =  {(uint8_t)174, (uint8_t)61, (uint8_t)83, (uint8_t)162};
            p217_value_SET(&value, 0, PH.base.pack) ;
        }
        p217_status_SET(e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_FAILED, PH.base.pack) ;
        p217_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_CHARGING, PH.base.pack) ;
        c_CommunicationChannel_on_GOPRO_GET_RESPONSE_217(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GOPRO_SET_REQUEST_218(), &PH);
        p218_target_component_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        {
            uint8_t value[] =  {(uint8_t)202, (uint8_t)250, (uint8_t)14, (uint8_t)187};
            p218_value_SET(&value, 0, PH.base.pack) ;
        }
        p218_target_system_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        p218_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_POWER, PH.base.pack) ;
        c_CommunicationChannel_on_GOPRO_SET_REQUEST_218(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GOPRO_SET_RESPONSE_219(), &PH);
        p219_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_PHOTO_RESOLUTION, PH.base.pack) ;
        p219_status_SET(e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_FAILED, PH.base.pack) ;
        c_CommunicationChannel_on_GOPRO_SET_RESPONSE_219(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_RPM_226(), &PH);
        p226_rpm1_SET((float) -1.8173366E38F, PH.base.pack) ;
        p226_rpm2_SET((float)1.2263879E38F, PH.base.pack) ;
        c_CommunicationChannel_on_RPM_226(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_GPS_GLITCH, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)248650046008015872L, PH.base.pack) ;
        p230_tas_ratio_SET((float) -1.4637933E37F, PH.base.pack) ;
        p230_mag_ratio_SET((float) -3.1739142E38F, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float) -3.4164028E37F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float) -2.5257376E38F, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float)2.4063223E38F, PH.base.pack) ;
        p230_vel_ratio_SET((float)1.8896472E38F, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float)5.470962E37F, PH.base.pack) ;
        p230_hagl_ratio_SET((float)1.8591025E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_WIND_COV_231(), &PH);
        p231_horiz_accuracy_SET((float)3.1302106E38F, PH.base.pack) ;
        p231_vert_accuracy_SET((float) -3.3188253E38F, PH.base.pack) ;
        p231_wind_z_SET((float) -2.9977314E38F, PH.base.pack) ;
        p231_var_horiz_SET((float) -2.600833E38F, PH.base.pack) ;
        p231_wind_y_SET((float)2.9119435E38F, PH.base.pack) ;
        p231_wind_x_SET((float) -3.2955754E38F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)9082765025820744926L, PH.base.pack) ;
        p231_var_vert_SET((float) -2.858627E38F, PH.base.pack) ;
        p231_wind_alt_SET((float)1.2842868E38F, PH.base.pack) ;
        c_CommunicationChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INPUT_232(), &PH);
        p232_ve_SET((float) -2.4268998E38F, PH.base.pack) ;
        p232_alt_SET((float)3.1723052E38F, PH.base.pack) ;
        p232_vdop_SET((float) -2.2505258E38F, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)2222818859L, PH.base.pack) ;
        p232_horiz_accuracy_SET((float) -5.6489406E37F, PH.base.pack) ;
        p232_hdop_SET((float) -3.1430916E38F, PH.base.pack) ;
        p232_vd_SET((float)3.189505E38F, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p232_speed_accuracy_SET((float)3.2961258E38F, PH.base.pack) ;
        p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)2071795378945817606L, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)2929, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        p232_lat_SET((int32_t) -1421815011, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p232_vert_accuracy_SET((float)1.1618179E38F, PH.base.pack) ;
        p232_vn_SET((float)2.2971161E38F, PH.base.pack) ;
        p232_lon_SET((int32_t) -1988797236, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTCM_DATA_233(), &PH);
        p233_len_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p233_flags_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)95, (uint8_t)208, (uint8_t)176, (uint8_t)156, (uint8_t)161, (uint8_t)63, (uint8_t)210, (uint8_t)19, (uint8_t)47, (uint8_t)35, (uint8_t)29, (uint8_t)96, (uint8_t)157, (uint8_t)115, (uint8_t)107, (uint8_t)89, (uint8_t)179, (uint8_t)81, (uint8_t)191, (uint8_t)214, (uint8_t)159, (uint8_t)43, (uint8_t)120, (uint8_t)60, (uint8_t)5, (uint8_t)183, (uint8_t)216, (uint8_t)162, (uint8_t)5, (uint8_t)212, (uint8_t)113, (uint8_t)15, (uint8_t)73, (uint8_t)130, (uint8_t)44, (uint8_t)53, (uint8_t)232, (uint8_t)176, (uint8_t)249, (uint8_t)84, (uint8_t)208, (uint8_t)211, (uint8_t)28, (uint8_t)133, (uint8_t)129, (uint8_t)109, (uint8_t)100, (uint8_t)110, (uint8_t)211, (uint8_t)231, (uint8_t)234, (uint8_t)72, (uint8_t)160, (uint8_t)172, (uint8_t)117, (uint8_t)63, (uint8_t)244, (uint8_t)0, (uint8_t)211, (uint8_t)1, (uint8_t)59, (uint8_t)80, (uint8_t)20, (uint8_t)71, (uint8_t)83, (uint8_t)36, (uint8_t)33, (uint8_t)25, (uint8_t)150, (uint8_t)119, (uint8_t)140, (uint8_t)160, (uint8_t)187, (uint8_t)74, (uint8_t)251, (uint8_t)160, (uint8_t)73, (uint8_t)42, (uint8_t)162, (uint8_t)52, (uint8_t)58, (uint8_t)11, (uint8_t)232, (uint8_t)130, (uint8_t)158, (uint8_t)17, (uint8_t)155, (uint8_t)248, (uint8_t)160, (uint8_t)166, (uint8_t)37, (uint8_t)26, (uint8_t)207, (uint8_t)227, (uint8_t)244, (uint8_t)56, (uint8_t)137, (uint8_t)182, (uint8_t)92, (uint8_t)95, (uint8_t)55, (uint8_t)188, (uint8_t)96, (uint8_t)223, (uint8_t)170, (uint8_t)158, (uint8_t)238, (uint8_t)225, (uint8_t)74, (uint8_t)54, (uint8_t)24, (uint8_t)223, (uint8_t)2, (uint8_t)48, (uint8_t)58, (uint8_t)250, (uint8_t)219, (uint8_t)109, (uint8_t)99, (uint8_t)163, (uint8_t)32, (uint8_t)105, (uint8_t)203, (uint8_t)72, (uint8_t)77, (uint8_t)183, (uint8_t)8, (uint8_t)182, (uint8_t)41, (uint8_t)222, (uint8_t)109, (uint8_t)58, (uint8_t)150, (uint8_t)107, (uint8_t)226, (uint8_t)193, (uint8_t)166, (uint8_t)157, (uint8_t)12, (uint8_t)78, (uint8_t)194, (uint8_t)101, (uint8_t)7, (uint8_t)107, (uint8_t)47, (uint8_t)253, (uint8_t)194, (uint8_t)85, (uint8_t)113, (uint8_t)110, (uint8_t)80, (uint8_t)101, (uint8_t)170, (uint8_t)8, (uint8_t)22, (uint8_t)101, (uint8_t)238, (uint8_t)139, (uint8_t)129, (uint8_t)188, (uint8_t)183, (uint8_t)39, (uint8_t)14, (uint8_t)125, (uint8_t)142, (uint8_t)152, (uint8_t)189, (uint8_t)164, (uint8_t)236, (uint8_t)27, (uint8_t)217, (uint8_t)69, (uint8_t)209, (uint8_t)179, (uint8_t)82, (uint8_t)97, (uint8_t)124, (uint8_t)254, (uint8_t)138, (uint8_t)97};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGH_LATENCY_234(), &PH);
        p234_battery_remaining_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING, PH.base.pack) ;
        p234_latitude_SET((int32_t)1393651403, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t)28043, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t)93, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t)96, PH.base.pack) ;
        p234_longitude_SET((int32_t)690522, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t)27, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t)97, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t)20206, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t)10157, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)27121, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t)18011, PH.base.pack) ;
        p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)4282768129L, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t)21413, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)54975, PH.base.pack) ;
        c_CommunicationChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VIBRATION_241(), &PH);
        p241_vibration_z_SET((float)9.893109E37F, PH.base.pack) ;
        p241_vibration_y_SET((float) -3.2061835E38F, PH.base.pack) ;
        p241_vibration_x_SET((float)1.5120582E38F, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)1217626708836182004L, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)69703481L, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)3864651756L, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)448690303L, PH.base.pack) ;
        c_CommunicationChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HOME_POSITION_242(), &PH);
        {
            float q[] =  {-2.123009E38F, 9.758344E36F, -1.573103E38F, 2.5156244E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_approach_y_SET((float)1.1335036E38F, PH.base.pack) ;
        p242_approach_x_SET((float) -1.5601182E38F, PH.base.pack) ;
        p242_approach_z_SET((float)2.0349473E38F, PH.base.pack) ;
        p242_x_SET((float)2.6093026E38F, PH.base.pack) ;
        p242_z_SET((float) -7.97693E37F, PH.base.pack) ;
        p242_latitude_SET((int32_t) -125019900, PH.base.pack) ;
        p242_longitude_SET((int32_t)1181089546, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)6357227412629425417L, &PH) ;
        p242_altitude_SET((int32_t) -531433883, PH.base.pack) ;
        p242_y_SET((float) -5.7388546E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_HOME_POSITION_243(), &PH);
        p243_approach_z_SET((float)3.2951062E38F, PH.base.pack) ;
        p243_approach_x_SET((float)1.6687696E38F, PH.base.pack) ;
        p243_z_SET((float) -2.5839333E38F, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p243_approach_y_SET((float)3.1046357E38F, PH.base.pack) ;
        p243_x_SET((float)3.2505253E38F, PH.base.pack) ;
        {
            float q[] =  {1.4521999E38F, -9.049117E37F, 2.6868092E38F, 4.927494E37F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_y_SET((float) -1.3587373E38F, PH.base.pack) ;
        p243_latitude_SET((int32_t)2046327104, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)611136243249599342L, &PH) ;
        p243_longitude_SET((int32_t)1859959094, PH.base.pack) ;
        p243_altitude_SET((int32_t)665446740, PH.base.pack) ;
        c_CommunicationChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_interval_us_SET((int32_t)1822755925, PH.base.pack) ;
        p244_message_id_SET((uint16_t)(uint16_t)49844, PH.base.pack) ;
        c_CommunicationChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, PH.base.pack) ;
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED, PH.base.pack) ;
        c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ADSB_VEHICLE_246(), &PH);
        p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY, PH.base.pack) ;
        p246_altitude_SET((int32_t) -92761215, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)62402, PH.base.pack) ;
        p246_lat_SET((int32_t) -1631143221, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UNASSGINED3, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t) -57, PH.base.pack) ;
        {
            char16_t* callsign = u"hvmrxonf";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_lon_SET((int32_t)926353628, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)2949266623L, PH.base.pack) ;
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)32767, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)15028, PH.base.pack) ;
        c_CommunicationChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COLLISION_247(), &PH);
        p247_time_to_minimum_delta_SET((float)1.2336657E38F, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, PH.base.pack) ;
        p247_id_SET((uint32_t)1363240111L, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_PERPENDICULAR, PH.base.pack) ;
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float)7.7806923E37F, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float) -9.217915E37F, PH.base.pack) ;
        c_CommunicationChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_V2_EXTENSION_248(), &PH);
        p248_message_type_SET((uint16_t)(uint16_t)3477, PH.base.pack) ;
        p248_target_network_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        p248_target_component_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        p248_target_system_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)125, (uint8_t)126, (uint8_t)37, (uint8_t)24, (uint8_t)54, (uint8_t)221, (uint8_t)66, (uint8_t)96, (uint8_t)188, (uint8_t)174, (uint8_t)33, (uint8_t)187, (uint8_t)231, (uint8_t)62, (uint8_t)113, (uint8_t)117, (uint8_t)185, (uint8_t)143, (uint8_t)18, (uint8_t)241, (uint8_t)118, (uint8_t)214, (uint8_t)110, (uint8_t)247, (uint8_t)152, (uint8_t)218, (uint8_t)159, (uint8_t)15, (uint8_t)33, (uint8_t)185, (uint8_t)207, (uint8_t)51, (uint8_t)156, (uint8_t)89, (uint8_t)153, (uint8_t)92, (uint8_t)110, (uint8_t)141, (uint8_t)253, (uint8_t)84, (uint8_t)189, (uint8_t)186, (uint8_t)81, (uint8_t)224, (uint8_t)120, (uint8_t)129, (uint8_t)87, (uint8_t)47, (uint8_t)8, (uint8_t)24, (uint8_t)131, (uint8_t)44, (uint8_t)200, (uint8_t)160, (uint8_t)114, (uint8_t)31, (uint8_t)143, (uint8_t)116, (uint8_t)14, (uint8_t)235, (uint8_t)104, (uint8_t)198, (uint8_t)190, (uint8_t)1, (uint8_t)56, (uint8_t)249, (uint8_t)42, (uint8_t)189, (uint8_t)178, (uint8_t)197, (uint8_t)96, (uint8_t)147, (uint8_t)75, (uint8_t)213, (uint8_t)77, (uint8_t)80, (uint8_t)185, (uint8_t)228, (uint8_t)128, (uint8_t)24, (uint8_t)30, (uint8_t)205, (uint8_t)228, (uint8_t)70, (uint8_t)176, (uint8_t)56, (uint8_t)62, (uint8_t)111, (uint8_t)36, (uint8_t)101, (uint8_t)217, (uint8_t)91, (uint8_t)210, (uint8_t)6, (uint8_t)245, (uint8_t)18, (uint8_t)138, (uint8_t)226, (uint8_t)159, (uint8_t)112, (uint8_t)200, (uint8_t)232, (uint8_t)45, (uint8_t)134, (uint8_t)250, (uint8_t)192, (uint8_t)125, (uint8_t)27, (uint8_t)199, (uint8_t)23, (uint8_t)239, (uint8_t)244, (uint8_t)70, (uint8_t)204, (uint8_t)173, (uint8_t)245, (uint8_t)191, (uint8_t)55, (uint8_t)75, (uint8_t)57, (uint8_t)109, (uint8_t)216, (uint8_t)201, (uint8_t)34, (uint8_t)215, (uint8_t)175, (uint8_t)36, (uint8_t)18, (uint8_t)148, (uint8_t)125, (uint8_t)76, (uint8_t)207, (uint8_t)198, (uint8_t)132, (uint8_t)61, (uint8_t)215, (uint8_t)50, (uint8_t)14, (uint8_t)86, (uint8_t)13, (uint8_t)134, (uint8_t)235, (uint8_t)10, (uint8_t)107, (uint8_t)108, (uint8_t)9, (uint8_t)112, (uint8_t)239, (uint8_t)92, (uint8_t)116, (uint8_t)252, (uint8_t)251, (uint8_t)34, (uint8_t)15, (uint8_t)99, (uint8_t)215, (uint8_t)78, (uint8_t)28, (uint8_t)30, (uint8_t)46, (uint8_t)174, (uint8_t)64, (uint8_t)60, (uint8_t)150, (uint8_t)117, (uint8_t)94, (uint8_t)128, (uint8_t)121, (uint8_t)203, (uint8_t)95, (uint8_t)45, (uint8_t)167, (uint8_t)95, (uint8_t)163, (uint8_t)198, (uint8_t)212, (uint8_t)11, (uint8_t)44, (uint8_t)157, (uint8_t)107, (uint8_t)223, (uint8_t)43, (uint8_t)180, (uint8_t)190, (uint8_t)173, (uint8_t)167, (uint8_t)237, (uint8_t)126, (uint8_t)81, (uint8_t)176, (uint8_t)65, (uint8_t)143, (uint8_t)109, (uint8_t)39, (uint8_t)96, (uint8_t)45, (uint8_t)66, (uint8_t)176, (uint8_t)25, (uint8_t)188, (uint8_t)113, (uint8_t)143, (uint8_t)106, (uint8_t)157, (uint8_t)189, (uint8_t)137, (uint8_t)91, (uint8_t)2, (uint8_t)107, (uint8_t)55, (uint8_t)113, (uint8_t)220, (uint8_t)177, (uint8_t)57, (uint8_t)249, (uint8_t)1, (uint8_t)73, (uint8_t)182, (uint8_t)172, (uint8_t)95, (uint8_t)45, (uint8_t)55, (uint8_t)169, (uint8_t)4, (uint8_t)119, (uint8_t)139, (uint8_t)165, (uint8_t)94, (uint8_t)28, (uint8_t)108, (uint8_t)62, (uint8_t)206, (uint8_t)163, (uint8_t)194, (uint8_t)159, (uint8_t)190, (uint8_t)90, (uint8_t)50, (uint8_t)157, (uint8_t)166, (uint8_t)3, (uint8_t)163, (uint8_t)208, (uint8_t)164, (uint8_t)105, (uint8_t)168, (uint8_t)5, (uint8_t)152, (uint8_t)240};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MEMORY_VECT_249(), &PH);
        p249_ver_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        p249_type_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t) -116, (int8_t) -61, (int8_t)100, (int8_t) -4, (int8_t) -9, (int8_t)116, (int8_t) -74, (int8_t) -30, (int8_t)80, (int8_t) -117, (int8_t)56, (int8_t) -49, (int8_t)45, (int8_t) -116, (int8_t)39, (int8_t) -92, (int8_t)95, (int8_t)102, (int8_t)88, (int8_t) -34, (int8_t)28, (int8_t) -95, (int8_t) -55, (int8_t)126, (int8_t)23, (int8_t) -119, (int8_t)32, (int8_t) -3, (int8_t)99, (int8_t)22, (int8_t)49, (int8_t)47};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_address_SET((uint16_t)(uint16_t)32707, PH.base.pack) ;
        c_CommunicationChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DEBUG_VECT_250(), &PH);
        p250_y_SET((float) -1.5546891E38F, PH.base.pack) ;
        p250_z_SET((float) -2.535671E38F, PH.base.pack) ;
        p250_x_SET((float)3.2455023E38F, PH.base.pack) ;
        {
            char16_t* name = u"uaavcsuy";
            p250_name_SET_(name, &PH) ;
        }
        p250_time_usec_SET((uint64_t)2637063938511420752L, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
        {
            char16_t* name = u"hpsmK";
            p251_name_SET_(name, &PH) ;
        }
        p251_value_SET((float) -3.0028637E38F, PH.base.pack) ;
        p251_time_boot_ms_SET((uint32_t)1316278535L, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAMED_VALUE_INT_252(), &PH);
        p252_time_boot_ms_SET((uint32_t)2398153666L, PH.base.pack) ;
        {
            char16_t* name = u"zn";
            p252_name_SET_(name, &PH) ;
        }
        p252_value_SET((int32_t)787673637, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_STATUSTEXT_253(), &PH);
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_DEBUG, PH.base.pack) ;
        {
            char16_t* text = u"ySfbczrkfvisrdUruhltvpsf";
            p253_text_SET_(text, &PH) ;
        }
        c_CommunicationChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DEBUG_254(), &PH);
        p254_time_boot_ms_SET((uint32_t)1784053255L, PH.base.pack) ;
        p254_ind_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p254_value_SET((float)3.3159085E38F, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SETUP_SIGNING_256(), &PH);
        p256_initial_timestamp_SET((uint64_t)700822186448103909L, PH.base.pack) ;
        p256_target_system_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)136, (uint8_t)227, (uint8_t)10, (uint8_t)81, (uint8_t)154, (uint8_t)238, (uint8_t)218, (uint8_t)55, (uint8_t)202, (uint8_t)57, (uint8_t)227, (uint8_t)137, (uint8_t)97, (uint8_t)17, (uint8_t)94, (uint8_t)179, (uint8_t)177, (uint8_t)139, (uint8_t)113, (uint8_t)188, (uint8_t)2, (uint8_t)164, (uint8_t)70, (uint8_t)62, (uint8_t)143, (uint8_t)240, (uint8_t)56, (uint8_t)4, (uint8_t)160, (uint8_t)223, (uint8_t)242, (uint8_t)14};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        p256_target_component_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        c_CommunicationChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_BUTTON_CHANGE_257(), &PH);
        p257_state_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        p257_time_boot_ms_SET((uint32_t)3729506112L, PH.base.pack) ;
        p257_last_change_ms_SET((uint32_t)4103053109L, PH.base.pack) ;
        c_CommunicationChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PLAY_TUNE_258(), &PH);
        {
            char16_t* tune = u"YCuLnnvdmxzhaqmhkauynyrku";
            p258_tune_SET_(tune, &PH) ;
        }
        p258_target_system_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p258_target_component_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
        c_CommunicationChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_INFORMATION_259(), &PH);
        p259_sensor_size_h_SET((float) -1.2356487E38F, PH.base.pack) ;
        p259_sensor_size_v_SET((float) -1.7008044E38F, PH.base.pack) ;
        p259_cam_definition_version_SET((uint16_t)(uint16_t)56058, PH.base.pack) ;
        p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)34467, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)40, (uint8_t)78, (uint8_t)77, (uint8_t)142, (uint8_t)196, (uint8_t)225, (uint8_t)162, (uint8_t)199, (uint8_t)122, (uint8_t)192, (uint8_t)70, (uint8_t)67, (uint8_t)252, (uint8_t)26, (uint8_t)17, (uint8_t)103, (uint8_t)97, (uint8_t)199, (uint8_t)64, (uint8_t)211, (uint8_t)11, (uint8_t)73, (uint8_t)149, (uint8_t)86, (uint8_t)238, (uint8_t)241, (uint8_t)246, (uint8_t)187, (uint8_t)201, (uint8_t)13, (uint8_t)28, (uint8_t)231};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        {
            uint8_t vendor_name[] =  {(uint8_t)255, (uint8_t)152, (uint8_t)37, (uint8_t)47, (uint8_t)10, (uint8_t)78, (uint8_t)236, (uint8_t)159, (uint8_t)53, (uint8_t)199, (uint8_t)198, (uint8_t)59, (uint8_t)175, (uint8_t)99, (uint8_t)171, (uint8_t)145, (uint8_t)168, (uint8_t)246, (uint8_t)177, (uint8_t)35, (uint8_t)84, (uint8_t)246, (uint8_t)20, (uint8_t)157, (uint8_t)230, (uint8_t)136, (uint8_t)48, (uint8_t)199, (uint8_t)170, (uint8_t)94, (uint8_t)251, (uint8_t)96};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_time_boot_ms_SET((uint32_t)3963523674L, PH.base.pack) ;
        p259_focal_length_SET((float) -2.5371747E38F, PH.base.pack) ;
        p259_firmware_version_SET((uint32_t)3849110563L, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"vxqawvdYsXwllDc";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_resolution_v_SET((uint16_t)(uint16_t)11178, PH.base.pack) ;
        p259_lens_id_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_SETTINGS_260(), &PH);
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_VIDEO, PH.base.pack) ;
        p260_time_boot_ms_SET((uint32_t)3067896154L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_STORAGE_INFORMATION_261(), &PH);
        p261_read_speed_SET((float)8.387959E37F, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)1028373128L, PH.base.pack) ;
        p261_write_speed_SET((float)2.6695673E38F, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p261_total_capacity_SET((float) -3.0558584E38F, PH.base.pack) ;
        p261_available_capacity_SET((float) -2.8826334E38F, PH.base.pack) ;
        p261_used_capacity_SET((float)2.2025539E38F, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        c_CommunicationChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_recording_time_ms_SET((uint32_t)907083870L, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)2119954947L, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        p262_available_capacity_SET((float)1.4725072E38F, PH.base.pack) ;
        p262_image_interval_SET((float) -1.669174E36F, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_camera_id_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t) -16, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)3787108044L, PH.base.pack) ;
        p263_image_index_SET((int32_t)1778403594, PH.base.pack) ;
        {
            float q[] =  {3.3734077E38F, 1.394257E38F, 1.416001E38F, 9.938413E36F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_relative_alt_SET((int32_t) -511502216, PH.base.pack) ;
        p263_time_utc_SET((uint64_t)2661778188252590438L, PH.base.pack) ;
        p263_alt_SET((int32_t) -947138213, PH.base.pack) ;
        p263_lon_SET((int32_t)1022031940, PH.base.pack) ;
        p263_lat_SET((int32_t)1180071505, PH.base.pack) ;
        {
            char16_t* file_url = u"utsqqmzWhmlFhotMuqkewzeynRjgdmohbmdrvfroWsrnjevnEmyagpbjvxkkx";
            p263_file_url_SET_(file_url, &PH) ;
        }
        c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_takeoff_time_utc_SET((uint64_t)5711009840114461792L, PH.base.pack) ;
        p264_time_boot_ms_SET((uint32_t)2996265288L, PH.base.pack) ;
        p264_arming_time_utc_SET((uint64_t)8238585564253823638L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)8180957634720839837L, PH.base.pack) ;
        c_CommunicationChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_time_boot_ms_SET((uint32_t)3446844325L, PH.base.pack) ;
        p265_yaw_SET((float)2.5878223E38F, PH.base.pack) ;
        p265_roll_SET((float)1.7065788E38F, PH.base.pack) ;
        p265_pitch_SET((float)1.1720304E38F, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_266(), &PH);
        p266_target_system_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
        p266_first_message_offset_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)11, (uint8_t)153, (uint8_t)199, (uint8_t)175, (uint8_t)156, (uint8_t)230, (uint8_t)35, (uint8_t)238, (uint8_t)9, (uint8_t)205, (uint8_t)197, (uint8_t)156, (uint8_t)120, (uint8_t)155, (uint8_t)229, (uint8_t)2, (uint8_t)155, (uint8_t)14, (uint8_t)34, (uint8_t)13, (uint8_t)207, (uint8_t)184, (uint8_t)177, (uint8_t)224, (uint8_t)235, (uint8_t)105, (uint8_t)52, (uint8_t)116, (uint8_t)119, (uint8_t)53, (uint8_t)37, (uint8_t)32, (uint8_t)243, (uint8_t)65, (uint8_t)197, (uint8_t)193, (uint8_t)24, (uint8_t)164, (uint8_t)21, (uint8_t)17, (uint8_t)52, (uint8_t)8, (uint8_t)162, (uint8_t)12, (uint8_t)152, (uint8_t)38, (uint8_t)120, (uint8_t)19, (uint8_t)62, (uint8_t)77, (uint8_t)203, (uint8_t)77, (uint8_t)185, (uint8_t)166, (uint8_t)32, (uint8_t)136, (uint8_t)205, (uint8_t)189, (uint8_t)38, (uint8_t)68, (uint8_t)23, (uint8_t)241, (uint8_t)218, (uint8_t)40, (uint8_t)220, (uint8_t)50, (uint8_t)157, (uint8_t)181, (uint8_t)94, (uint8_t)235, (uint8_t)244, (uint8_t)160, (uint8_t)151, (uint8_t)1, (uint8_t)11, (uint8_t)74, (uint8_t)42, (uint8_t)58, (uint8_t)183, (uint8_t)141, (uint8_t)237, (uint8_t)92, (uint8_t)150, (uint8_t)33, (uint8_t)240, (uint8_t)120, (uint8_t)104, (uint8_t)170, (uint8_t)148, (uint8_t)168, (uint8_t)195, (uint8_t)82, (uint8_t)175, (uint8_t)243, (uint8_t)116, (uint8_t)18, (uint8_t)66, (uint8_t)7, (uint8_t)73, (uint8_t)78, (uint8_t)225, (uint8_t)7, (uint8_t)83, (uint8_t)1, (uint8_t)244, (uint8_t)246, (uint8_t)142, (uint8_t)228, (uint8_t)220, (uint8_t)179, (uint8_t)246, (uint8_t)125, (uint8_t)166, (uint8_t)12, (uint8_t)121, (uint8_t)26, (uint8_t)46, (uint8_t)154, (uint8_t)97, (uint8_t)170, (uint8_t)92, (uint8_t)156, (uint8_t)223, (uint8_t)169, (uint8_t)136, (uint8_t)30, (uint8_t)184, (uint8_t)137, (uint8_t)32, (uint8_t)142, (uint8_t)236, (uint8_t)107, (uint8_t)98, (uint8_t)37, (uint8_t)177, (uint8_t)73, (uint8_t)126, (uint8_t)180, (uint8_t)9, (uint8_t)101, (uint8_t)93, (uint8_t)43, (uint8_t)126, (uint8_t)22, (uint8_t)215, (uint8_t)164, (uint8_t)230, (uint8_t)236, (uint8_t)224, (uint8_t)156, (uint8_t)135, (uint8_t)140, (uint8_t)52, (uint8_t)139, (uint8_t)66, (uint8_t)230, (uint8_t)98, (uint8_t)202, (uint8_t)142, (uint8_t)37, (uint8_t)74, (uint8_t)120, (uint8_t)185, (uint8_t)222, (uint8_t)60, (uint8_t)21, (uint8_t)172, (uint8_t)7, (uint8_t)101, (uint8_t)158, (uint8_t)241, (uint8_t)128, (uint8_t)253, (uint8_t)226, (uint8_t)225, (uint8_t)140, (uint8_t)32, (uint8_t)41, (uint8_t)51, (uint8_t)224, (uint8_t)141, (uint8_t)19, (uint8_t)226, (uint8_t)35, (uint8_t)68, (uint8_t)122, (uint8_t)128, (uint8_t)8, (uint8_t)176, (uint8_t)129, (uint8_t)101, (uint8_t)188, (uint8_t)134, (uint8_t)208, (uint8_t)0, (uint8_t)28, (uint8_t)188, (uint8_t)78, (uint8_t)95, (uint8_t)119, (uint8_t)155, (uint8_t)14, (uint8_t)75, (uint8_t)22, (uint8_t)255, (uint8_t)236, (uint8_t)216, (uint8_t)92, (uint8_t)253, (uint8_t)59, (uint8_t)76, (uint8_t)64, (uint8_t)225, (uint8_t)71, (uint8_t)63, (uint8_t)237, (uint8_t)105, (uint8_t)250, (uint8_t)65, (uint8_t)184, (uint8_t)145, (uint8_t)91, (uint8_t)136, (uint8_t)30, (uint8_t)116, (uint8_t)102, (uint8_t)0, (uint8_t)194, (uint8_t)86, (uint8_t)115, (uint8_t)198, (uint8_t)207, (uint8_t)132, (uint8_t)129, (uint8_t)22, (uint8_t)102, (uint8_t)220, (uint8_t)149, (uint8_t)141, (uint8_t)57, (uint8_t)130, (uint8_t)176, (uint8_t)193, (uint8_t)71, (uint8_t)228, (uint8_t)237, (uint8_t)33, (uint8_t)216, (uint8_t)158};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_sequence_SET((uint16_t)(uint16_t)56191, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        p266_length_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_ACKED_267(), &PH);
        p267_sequence_SET((uint16_t)(uint16_t)32748, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        p267_target_component_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)191, (uint8_t)39, (uint8_t)195, (uint8_t)120, (uint8_t)116, (uint8_t)47, (uint8_t)66, (uint8_t)52, (uint8_t)141, (uint8_t)33, (uint8_t)83, (uint8_t)181, (uint8_t)172, (uint8_t)177, (uint8_t)52, (uint8_t)155, (uint8_t)149, (uint8_t)225, (uint8_t)84, (uint8_t)148, (uint8_t)175, (uint8_t)225, (uint8_t)78, (uint8_t)51, (uint8_t)32, (uint8_t)208, (uint8_t)117, (uint8_t)236, (uint8_t)78, (uint8_t)68, (uint8_t)207, (uint8_t)218, (uint8_t)227, (uint8_t)24, (uint8_t)15, (uint8_t)37, (uint8_t)121, (uint8_t)116, (uint8_t)63, (uint8_t)220, (uint8_t)142, (uint8_t)145, (uint8_t)67, (uint8_t)215, (uint8_t)148, (uint8_t)153, (uint8_t)42, (uint8_t)207, (uint8_t)26, (uint8_t)213, (uint8_t)197, (uint8_t)227, (uint8_t)202, (uint8_t)15, (uint8_t)75, (uint8_t)29, (uint8_t)18, (uint8_t)244, (uint8_t)40, (uint8_t)149, (uint8_t)240, (uint8_t)163, (uint8_t)54, (uint8_t)69, (uint8_t)5, (uint8_t)42, (uint8_t)20, (uint8_t)154, (uint8_t)196, (uint8_t)89, (uint8_t)83, (uint8_t)134, (uint8_t)158, (uint8_t)215, (uint8_t)160, (uint8_t)143, (uint8_t)42, (uint8_t)42, (uint8_t)197, (uint8_t)57, (uint8_t)45, (uint8_t)51, (uint8_t)101, (uint8_t)105, (uint8_t)75, (uint8_t)17, (uint8_t)20, (uint8_t)20, (uint8_t)162, (uint8_t)233, (uint8_t)107, (uint8_t)10, (uint8_t)169, (uint8_t)95, (uint8_t)117, (uint8_t)226, (uint8_t)167, (uint8_t)50, (uint8_t)150, (uint8_t)155, (uint8_t)250, (uint8_t)6, (uint8_t)57, (uint8_t)149, (uint8_t)197, (uint8_t)94, (uint8_t)178, (uint8_t)125, (uint8_t)54, (uint8_t)238, (uint8_t)84, (uint8_t)18, (uint8_t)163, (uint8_t)255, (uint8_t)18, (uint8_t)150, (uint8_t)62, (uint8_t)151, (uint8_t)243, (uint8_t)231, (uint8_t)204, (uint8_t)2, (uint8_t)96, (uint8_t)237, (uint8_t)131, (uint8_t)130, (uint8_t)127, (uint8_t)218, (uint8_t)181, (uint8_t)210, (uint8_t)143, (uint8_t)236, (uint8_t)114, (uint8_t)166, (uint8_t)213, (uint8_t)17, (uint8_t)174, (uint8_t)17, (uint8_t)29, (uint8_t)74, (uint8_t)77, (uint8_t)148, (uint8_t)101, (uint8_t)92, (uint8_t)20, (uint8_t)153, (uint8_t)255, (uint8_t)87, (uint8_t)57, (uint8_t)93, (uint8_t)110, (uint8_t)54, (uint8_t)31, (uint8_t)80, (uint8_t)20, (uint8_t)230, (uint8_t)104, (uint8_t)146, (uint8_t)21, (uint8_t)87, (uint8_t)58, (uint8_t)129, (uint8_t)208, (uint8_t)135, (uint8_t)150, (uint8_t)250, (uint8_t)192, (uint8_t)29, (uint8_t)244, (uint8_t)206, (uint8_t)59, (uint8_t)154, (uint8_t)31, (uint8_t)106, (uint8_t)207, (uint8_t)14, (uint8_t)185, (uint8_t)128, (uint8_t)121, (uint8_t)210, (uint8_t)104, (uint8_t)189, (uint8_t)42, (uint8_t)122, (uint8_t)108, (uint8_t)232, (uint8_t)117, (uint8_t)154, (uint8_t)177, (uint8_t)147, (uint8_t)215, (uint8_t)25, (uint8_t)211, (uint8_t)208, (uint8_t)2, (uint8_t)176, (uint8_t)172, (uint8_t)42, (uint8_t)48, (uint8_t)241, (uint8_t)31, (uint8_t)155, (uint8_t)243, (uint8_t)249, (uint8_t)242, (uint8_t)37, (uint8_t)212, (uint8_t)229, (uint8_t)198, (uint8_t)85, (uint8_t)214, (uint8_t)214, (uint8_t)5, (uint8_t)178, (uint8_t)87, (uint8_t)41, (uint8_t)32, (uint8_t)45, (uint8_t)118, (uint8_t)70, (uint8_t)3, (uint8_t)199, (uint8_t)36, (uint8_t)171, (uint8_t)195, (uint8_t)236, (uint8_t)236, (uint8_t)179, (uint8_t)67, (uint8_t)230, (uint8_t)229, (uint8_t)11, (uint8_t)13, (uint8_t)137, (uint8_t)208, (uint8_t)52, (uint8_t)167, (uint8_t)181, (uint8_t)212, (uint8_t)147, (uint8_t)31, (uint8_t)165, (uint8_t)40, (uint8_t)243, (uint8_t)46, (uint8_t)49, (uint8_t)127, (uint8_t)184, (uint8_t)169};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_ACK_268(), &PH);
        p268_sequence_SET((uint16_t)(uint16_t)34556, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p268_target_system_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_bitrate_SET((uint32_t)919304751L, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)58075, PH.base.pack) ;
        p269_framerate_SET((float)1.85183E38F, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)7857, PH.base.pack) ;
        p269_rotation_SET((uint16_t)(uint16_t)60441, PH.base.pack) ;
        {
            char16_t* uri = u"pxtnhcgxvzkvoyAbwUqvkgiqckyqijgc";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_status_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_framerate_SET((float) -3.4455615E37F, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)52652, PH.base.pack) ;
        p270_target_component_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)27001, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)62147, PH.base.pack) ;
        {
            char16_t* uri = u"dlcegzbjidfsswlrjklbfqxqdNporiVcvvyDkiuqzsurkvunxigqhxuidbvfwslhxphhudAegbjchoncebrkbemkpihbvpsfjddrwottmczuefufcuVjhzyiuqugyycirhkVwhgvgfsz";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_camera_id_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)3926114129L, PH.base.pack) ;
        c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* ssid = u"anmktGmBHetgfltzhnzbo";
            p299_ssid_SET_(ssid, &PH) ;
        }
        {
            char16_t* password = u"pih";
            p299_password_SET_(password, &PH) ;
        }
        c_CommunicationChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PROTOCOL_VERSION_300(), &PH);
        p300_max_version_SET((uint16_t)(uint16_t)46890, PH.base.pack) ;
        {
            uint8_t library_version_hash[] =  {(uint8_t)26, (uint8_t)205, (uint8_t)252, (uint8_t)19, (uint8_t)214, (uint8_t)183, (uint8_t)233, (uint8_t)103};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        {
            uint8_t spec_version_hash[] =  {(uint8_t)60, (uint8_t)67, (uint8_t)209, (uint8_t)253, (uint8_t)41, (uint8_t)114, (uint8_t)178, (uint8_t)57};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        p300_version_SET((uint16_t)(uint16_t)54154, PH.base.pack) ;
        p300_min_version_SET((uint16_t)(uint16_t)64535, PH.base.pack) ;
        c_CommunicationChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)51516, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)6366633922272232498L, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)1654715427L, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_hw_version_major_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)361162760L, PH.base.pack) ;
        p311_time_usec_SET((uint64_t)7353026846595050098L, PH.base.pack) ;
        p311_sw_version_minor_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)1969869180L, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)31, (uint8_t)34, (uint8_t)254, (uint8_t)253, (uint8_t)99, (uint8_t)173, (uint8_t)101, (uint8_t)123, (uint8_t)193, (uint8_t)128, (uint8_t)128, (uint8_t)88, (uint8_t)212, (uint8_t)0, (uint8_t)100, (uint8_t)213};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        p311_sw_version_major_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p311_hw_version_minor_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        {
            char16_t* name = u"rgXcMdigshrcukZrtsryflzyxseoPbvrCCrybsnfvtavbimslukUowo";
            p311_name_SET_(name, &PH) ;
        }
        c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        p320_target_component_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        {
            char16_t* param_id = u"qnxtslu";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_target_system_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        p320_param_index_SET((int16_t)(int16_t)25859, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_system_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        p321_target_component_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_VALUE_322(), &PH);
        p322_param_index_SET((uint16_t)(uint16_t)24236, PH.base.pack) ;
        p322_param_count_SET((uint16_t)(uint16_t)4493, PH.base.pack) ;
        {
            char16_t* param_value = u"fkbxdysnSeFmfvyFlcmhetsAsmscmwbobtpyzeceupvqiqjeTcPkIzwmhhewlpxpbidrnzhtgvutfzJugegdzy";
            p322_param_value_SET_(param_value, &PH) ;
        }
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32, PH.base.pack) ;
        {
            char16_t* param_id = u"hey";
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
            char16_t* param_id = u"eynyvbkjphx";
            p323_param_id_SET_(param_id, &PH) ;
        }
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16, PH.base.pack) ;
        {
            char16_t* param_value = u"pwmpdqbklwcstedqzdumwTCqkzwBodrXgbwfrhssfrXewlqoxtooxwzjzzszdvdtzawjac";
            p323_param_value_SET_(param_value, &PH) ;
        }
        p323_target_component_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        p323_target_system_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_ACK_324(), &PH);
        {
            char16_t* param_id = u"vqvrfbjeqoei";
            p324_param_id_SET_(param_id, &PH) ;
        }
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_FAILED, PH.base.pack) ;
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_CUSTOM, PH.base.pack) ;
        {
            char16_t* param_value = u"xp";
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
            uint16_t distances[] =  {(uint16_t)53209, (uint16_t)38945, (uint16_t)20398, (uint16_t)8285, (uint16_t)19976, (uint16_t)59507, (uint16_t)39001, (uint16_t)19216, (uint16_t)15892, (uint16_t)33083, (uint16_t)27784, (uint16_t)1848, (uint16_t)4783, (uint16_t)9261, (uint16_t)7917, (uint16_t)37219, (uint16_t)14373, (uint16_t)4711, (uint16_t)58931, (uint16_t)50920, (uint16_t)21808, (uint16_t)42118, (uint16_t)47775, (uint16_t)34806, (uint16_t)56136, (uint16_t)12325, (uint16_t)64116, (uint16_t)59876, (uint16_t)46110, (uint16_t)51897, (uint16_t)2653, (uint16_t)64854, (uint16_t)692, (uint16_t)23978, (uint16_t)61870, (uint16_t)16422, (uint16_t)40059, (uint16_t)16306, (uint16_t)58739, (uint16_t)10376, (uint16_t)23222, (uint16_t)38130, (uint16_t)25747, (uint16_t)63841, (uint16_t)11481, (uint16_t)49402, (uint16_t)53949, (uint16_t)32230, (uint16_t)36921, (uint16_t)43447, (uint16_t)58421, (uint16_t)28568, (uint16_t)44290, (uint16_t)15619, (uint16_t)38286, (uint16_t)11529, (uint16_t)29269, (uint16_t)25833, (uint16_t)55009, (uint16_t)26719, (uint16_t)32684, (uint16_t)19466, (uint16_t)12432, (uint16_t)9472, (uint16_t)41808, (uint16_t)63101, (uint16_t)967, (uint16_t)67, (uint16_t)14737, (uint16_t)41009, (uint16_t)33366, (uint16_t)32454};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_increment_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p330_time_usec_SET((uint64_t)3248363537762008590L, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)14887, PH.base.pack) ;
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
        p330_max_distance_SET((uint16_t)(uint16_t)49910, PH.base.pack) ;
        c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVIONIX_ADSB_OUT_CFG_10001(), &PH);
        p10001_aircraftSize_SET(e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L75_W72P5M, PH.base.pack) ;
        p10001_stallSpeed_SET((uint16_t)(uint16_t)31026, PH.base.pack) ;
        p10001_emitterType_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_POINT_OBSTACLE, PH.base.pack) ;
        p10001_gpsOffsetLon_SET(e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA, PH.base.pack) ;
        {
            char16_t* callsign = u"zLlpqGpb";
            p10001_callsign_SET_(callsign, &PH) ;
        }
        p10001_ICAO_SET((uint32_t)1157802710L, PH.base.pack) ;
        p10001_gpsOffsetLat_SET(e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_4M, PH.base.pack) ;
        p10001_rfSelect_SET(e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED, PH.base.pack) ;
        c_CommunicationChannel_on_UAVIONIX_ADSB_OUT_CFG_10001(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVIONIX_ADSB_OUT_DYNAMIC_10002(), &PH);
        p10002_numSats_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        p10002_velNS_SET((int16_t)(int16_t) -32652, PH.base.pack) ;
        p10002_gpsAlt_SET((int32_t) -760840074, PH.base.pack) ;
        p10002_VelEW_SET((int16_t)(int16_t)16041, PH.base.pack) ;
        p10002_baroAltMSL_SET((int32_t)1788311892, PH.base.pack) ;
        p10002_gpsFix_SET(e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0, PH.base.pack) ;
        p10002_gpsLon_SET((int32_t)718159832, PH.base.pack) ;
        p10002_gpsLat_SET((int32_t) -497373778, PH.base.pack) ;
        p10002_squawk_SET((uint16_t)(uint16_t)34753, PH.base.pack) ;
        p10002_accuracyVel_SET((uint16_t)(uint16_t)18806, PH.base.pack) ;
        p10002_velVert_SET((int16_t)(int16_t)1187, PH.base.pack) ;
        p10002_accuracyHor_SET((uint32_t)1896413619L, PH.base.pack) ;
        p10002_state_SET(e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED, PH.base.pack) ;
        p10002_emergencyStatus_SET(e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY, PH.base.pack) ;
        p10002_accuracyVert_SET((uint16_t)(uint16_t)62882, PH.base.pack) ;
        p10002_utcTime_SET((uint32_t)2592212055L, PH.base.pack) ;
        c_CommunicationChannel_on_UAVIONIX_ADSB_OUT_DYNAMIC_10002(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_10003(), &PH);
        p10003_rfHealth_SET(e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_INITIALIZING, PH.base.pack) ;
        c_CommunicationChannel_on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_10003(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEVICE_OP_READ_11000(), &PH);
        p11000_target_component_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p11000_bustype_SET(e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_I2C, PH.base.pack) ;
        p11000_request_id_SET((uint32_t)3137963143L, PH.base.pack) ;
        p11000_regstart_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        {
            char16_t* busname = u"hlihmaf";
            p11000_busname_SET_(busname, &PH) ;
        }
        p11000_count_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        p11000_address_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        p11000_target_system_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p11000_bus_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        c_CommunicationChannel_on_DEVICE_OP_READ_11000(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEVICE_OP_READ_REPLY_11001(), &PH);
        p11001_request_id_SET((uint32_t)2245592289L, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)187, (uint8_t)59, (uint8_t)85, (uint8_t)93, (uint8_t)63, (uint8_t)21, (uint8_t)228, (uint8_t)145, (uint8_t)92, (uint8_t)12, (uint8_t)21, (uint8_t)205, (uint8_t)122, (uint8_t)209, (uint8_t)25, (uint8_t)6, (uint8_t)183, (uint8_t)106, (uint8_t)148, (uint8_t)12, (uint8_t)9, (uint8_t)193, (uint8_t)189, (uint8_t)82, (uint8_t)14, (uint8_t)250, (uint8_t)173, (uint8_t)15, (uint8_t)185, (uint8_t)222, (uint8_t)26, (uint8_t)176, (uint8_t)42, (uint8_t)160, (uint8_t)22, (uint8_t)4, (uint8_t)44, (uint8_t)59, (uint8_t)209, (uint8_t)55, (uint8_t)10, (uint8_t)78, (uint8_t)149, (uint8_t)121, (uint8_t)229, (uint8_t)209, (uint8_t)44, (uint8_t)6, (uint8_t)155, (uint8_t)164, (uint8_t)177, (uint8_t)201, (uint8_t)32, (uint8_t)49, (uint8_t)69, (uint8_t)178, (uint8_t)45, (uint8_t)33, (uint8_t)138, (uint8_t)77, (uint8_t)209, (uint8_t)43, (uint8_t)197, (uint8_t)111, (uint8_t)100, (uint8_t)3, (uint8_t)108, (uint8_t)36, (uint8_t)38, (uint8_t)245, (uint8_t)38, (uint8_t)139, (uint8_t)35, (uint8_t)157, (uint8_t)31, (uint8_t)125, (uint8_t)42, (uint8_t)81, (uint8_t)63, (uint8_t)214, (uint8_t)161, (uint8_t)24, (uint8_t)4, (uint8_t)141, (uint8_t)10, (uint8_t)154, (uint8_t)163, (uint8_t)101, (uint8_t)105, (uint8_t)48, (uint8_t)80, (uint8_t)238, (uint8_t)201, (uint8_t)9, (uint8_t)69, (uint8_t)184, (uint8_t)99, (uint8_t)62, (uint8_t)47, (uint8_t)38, (uint8_t)136, (uint8_t)24, (uint8_t)138, (uint8_t)52, (uint8_t)223, (uint8_t)39, (uint8_t)197, (uint8_t)100, (uint8_t)4, (uint8_t)142, (uint8_t)152, (uint8_t)206, (uint8_t)228, (uint8_t)240, (uint8_t)71, (uint8_t)202, (uint8_t)154, (uint8_t)198, (uint8_t)232, (uint8_t)140, (uint8_t)12, (uint8_t)204, (uint8_t)152, (uint8_t)216, (uint8_t)19, (uint8_t)240, (uint8_t)23, (uint8_t)120};
            p11001_data__SET(&data_, 0, PH.base.pack) ;
        }
        p11001_result_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        p11001_regstart_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p11001_count_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        c_CommunicationChannel_on_DEVICE_OP_READ_REPLY_11001(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEVICE_OP_WRITE_11002(), &PH);
        p11002_bus_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        p11002_request_id_SET((uint32_t)999558945L, PH.base.pack) ;
        p11002_address_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)128, (uint8_t)28, (uint8_t)188, (uint8_t)222, (uint8_t)209, (uint8_t)134, (uint8_t)48, (uint8_t)132, (uint8_t)231, (uint8_t)187, (uint8_t)199, (uint8_t)236, (uint8_t)128, (uint8_t)156, (uint8_t)31, (uint8_t)217, (uint8_t)155, (uint8_t)246, (uint8_t)78, (uint8_t)165, (uint8_t)244, (uint8_t)1, (uint8_t)210, (uint8_t)204, (uint8_t)159, (uint8_t)2, (uint8_t)39, (uint8_t)41, (uint8_t)94, (uint8_t)204, (uint8_t)44, (uint8_t)88, (uint8_t)129, (uint8_t)22, (uint8_t)185, (uint8_t)45, (uint8_t)190, (uint8_t)41, (uint8_t)70, (uint8_t)154, (uint8_t)113, (uint8_t)103, (uint8_t)109, (uint8_t)231, (uint8_t)217, (uint8_t)201, (uint8_t)19, (uint8_t)253, (uint8_t)185, (uint8_t)206, (uint8_t)240, (uint8_t)98, (uint8_t)231, (uint8_t)176, (uint8_t)8, (uint8_t)112, (uint8_t)107, (uint8_t)210, (uint8_t)84, (uint8_t)140, (uint8_t)116, (uint8_t)188, (uint8_t)193, (uint8_t)199, (uint8_t)83, (uint8_t)39, (uint8_t)192, (uint8_t)70, (uint8_t)71, (uint8_t)208, (uint8_t)97, (uint8_t)119, (uint8_t)38, (uint8_t)43, (uint8_t)69, (uint8_t)119, (uint8_t)84, (uint8_t)170, (uint8_t)118, (uint8_t)60, (uint8_t)81, (uint8_t)239, (uint8_t)59, (uint8_t)200, (uint8_t)99, (uint8_t)209, (uint8_t)119, (uint8_t)135, (uint8_t)3, (uint8_t)42, (uint8_t)12, (uint8_t)241, (uint8_t)209, (uint8_t)178, (uint8_t)84, (uint8_t)246, (uint8_t)32, (uint8_t)158, (uint8_t)46, (uint8_t)231, (uint8_t)112, (uint8_t)134, (uint8_t)188, (uint8_t)237, (uint8_t)233, (uint8_t)215, (uint8_t)31, (uint8_t)63, (uint8_t)253, (uint8_t)79, (uint8_t)24, (uint8_t)115, (uint8_t)96, (uint8_t)132, (uint8_t)180, (uint8_t)189, (uint8_t)44, (uint8_t)140, (uint8_t)250, (uint8_t)233, (uint8_t)61, (uint8_t)245, (uint8_t)45, (uint8_t)191, (uint8_t)38, (uint8_t)104, (uint8_t)243, (uint8_t)143};
            p11002_data__SET(&data_, 0, PH.base.pack) ;
        }
        p11002_target_component_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        {
            char16_t* busname = u"xIsybfytbwavlwkntcajyVvxbcfqx";
            p11002_busname_SET_(busname, &PH) ;
        }
        p11002_target_system_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p11002_bustype_SET(e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI, PH.base.pack) ;
        p11002_regstart_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p11002_count_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        c_CommunicationChannel_on_DEVICE_OP_WRITE_11002(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEVICE_OP_WRITE_REPLY_11003(), &PH);
        p11003_request_id_SET((uint32_t)1171522401L, PH.base.pack) ;
        p11003_result_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        c_CommunicationChannel_on_DEVICE_OP_WRITE_REPLY_11003(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ADAP_TUNING_11010(), &PH);
        p11010_error_SET((float) -1.1788335E37F, PH.base.pack) ;
        p11010_sigma_dot_SET((float)2.2069787E38F, PH.base.pack) ;
        p11010_omega_dot_SET((float) -2.3996904E38F, PH.base.pack) ;
        p11010_axis_SET(e_PID_TUNING_AXIS_PID_TUNING_YAW, PH.base.pack) ;
        p11010_theta_SET((float) -1.6369829E38F, PH.base.pack) ;
        p11010_omega_SET((float)2.728428E38F, PH.base.pack) ;
        p11010_f_dot_SET((float)2.8000038E36F, PH.base.pack) ;
        p11010_f_SET((float) -2.8415076E38F, PH.base.pack) ;
        p11010_achieved_SET((float) -3.213574E37F, PH.base.pack) ;
        p11010_theta_dot_SET((float)1.4757622E38F, PH.base.pack) ;
        p11010_desired_SET((float)1.8161123E38F, PH.base.pack) ;
        p11010_u_SET((float)2.9683876E38F, PH.base.pack) ;
        p11010_sigma_SET((float)2.5300924E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ADAP_TUNING_11010(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VISION_POSITION_DELTA_11011(), &PH);
        p11011_time_delta_usec_SET((uint64_t)2588857796910194624L, PH.base.pack) ;
        p11011_time_usec_SET((uint64_t)8848821314919318574L, PH.base.pack) ;
        {
            float angle_delta[] =  {-1.6983454E38F, 2.7353405E38F, 4.3185104E37F};
            p11011_angle_delta_SET(&angle_delta, 0, PH.base.pack) ;
        }
        {
            float position_delta[] =  {3.0842056E37F, -2.5747063E38F, 1.9477526E37F};
            p11011_position_delta_SET(&position_delta, 0, PH.base.pack) ;
        }
        p11011_confidence_SET((float) -9.461622E37F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_POSITION_DELTA_11011(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
}


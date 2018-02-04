
package org.noname;
import java.util.*;
import static org.unirail.BlackBox.BitUtils.*;
import org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds;
import java.io.IOException;

public class Test extends GroundControl
{


    public static class HEARTBEAT extends GroundControl.HEARTBEAT
    {
        public void custom_mode_SET(long  src) //A bitfield for use for autopilot-specific flags.
        {  set_bytes((src) & -1L, 4, data,  0); }
        public void mavlink_version_SET(char  src) //MAVLink version, not writable by user, gets added by protocol because of magic data type: char_mavlink_versio
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public void type_SET(@MAV_TYPE int  src) //Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
        {  set_bits(- 0 +   src, 5, data, 40); }
        public void autopilot_SET(@MAV_AUTOPILOT int  src) //Autopilot type / class. defined in MAV_AUTOPILOT ENUM
        {  set_bits(- 0 +   src, 5, data, 45); }
        public void base_mode_SET(@MAV_MODE_FLAG int  src) //System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
        {
            long id = 0;
            switch(src)
            {
                case MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
                    id = 0;
                    break;
                case MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED:
                    id = 1;
                    break;
                case MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED:
                    id = 2;
                    break;
                case MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED:
                    id = 3;
                    break;
                case MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED:
                    id = 4;
                    break;
                case MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED:
                    id = 5;
                    break;
                case MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED:
                    id = 6;
                    break;
                case MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED:
                    id = 7;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 4, data, 50);
        }
        public void system_status_SET(@MAV_STATE int  src) //System status flag, see MAV_STATE ENUM
        {  set_bits(- 0 +   src, 4, data, 54); }
    }
    public static class SYS_STATUS extends GroundControl.SYS_STATUS
    {
        public void load_SET(char  src) //Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void voltage_battery_SET(char  src) //Battery voltage, in millivolts (1 = 1 millivolt)
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        /**
        *Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links
        *	(packets that were corrupted on reception on the MAV*/
        public void drop_rate_comm_SET(char  src)
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        /**
        *Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted
        *	on reception on the MAV*/
        public void errors_comm_SET(char  src)
        {  set_bytes((char)(src) & -1L, 2, data,  6); }
        public void errors_count1_SET(char  src) //Autopilot-specific errors
        {  set_bytes((char)(src) & -1L, 2, data,  8); }
        public void errors_count2_SET(char  src) //Autopilot-specific errors
        {  set_bytes((char)(src) & -1L, 2, data,  10); }
        public void errors_count3_SET(char  src) //Autopilot-specific errors
        {  set_bytes((char)(src) & -1L, 2, data,  12); }
        public void errors_count4_SET(char  src) //Autopilot-specific errors
        {  set_bytes((char)(src) & -1L, 2, data,  14); }
        public void current_battery_SET(short  src) //Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
        {  set_bytes((short)(src) & -1L, 2, data,  16); }
        public void battery_remaining_SET(byte  src) //Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
        {  set_bytes((byte)(src) & -1L, 1, data,  18); }
        /**
        *Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1:
        *	present. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
        public void onboard_control_sensors_present_SET(@MAV_SYS_STATUS_SENSOR int  src)
        {
            long id = 0;
            switch(src)
            {
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO:
                    id = 0;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL:
                    id = 1;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG:
                    id = 2;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE:
                    id = 3;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE:
                    id = 4;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS:
                    id = 5;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW:
                    id = 6;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION:
                    id = 7;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION:
                    id = 8;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH:
                    id = 9;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL:
                    id = 10;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION:
                    id = 11;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION:
                    id = 12;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL:
                    id = 13;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL:
                    id = 14;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS:
                    id = 15;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER:
                    id = 16;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2:
                    id = 17;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2:
                    id = 18;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2:
                    id = 19;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE:
                    id = 20;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS:
                    id = 21;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN:
                    id = 22;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR:
                    id = 23;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING:
                    id = 24;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY:
                    id = 25;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 5, data, 152);
        }
        /**
        *Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of
        *	1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
        public void onboard_control_sensors_enabled_SET(@MAV_SYS_STATUS_SENSOR int  src)
        {
            long id = 0;
            switch(src)
            {
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO:
                    id = 0;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL:
                    id = 1;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG:
                    id = 2;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE:
                    id = 3;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE:
                    id = 4;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS:
                    id = 5;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW:
                    id = 6;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION:
                    id = 7;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION:
                    id = 8;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH:
                    id = 9;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL:
                    id = 10;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION:
                    id = 11;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION:
                    id = 12;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL:
                    id = 13;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL:
                    id = 14;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS:
                    id = 15;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER:
                    id = 16;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2:
                    id = 17;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2:
                    id = 18;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2:
                    id = 19;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE:
                    id = 20;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS:
                    id = 21;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN:
                    id = 22;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR:
                    id = 23;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING:
                    id = 24;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY:
                    id = 25;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 5, data, 157);
        }
        /**
        *Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not
        *	enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
        public void onboard_control_sensors_health_SET(@MAV_SYS_STATUS_SENSOR int  src)
        {
            long id = 0;
            switch(src)
            {
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO:
                    id = 0;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL:
                    id = 1;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG:
                    id = 2;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE:
                    id = 3;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE:
                    id = 4;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS:
                    id = 5;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW:
                    id = 6;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION:
                    id = 7;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION:
                    id = 8;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH:
                    id = 9;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL:
                    id = 10;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION:
                    id = 11;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION:
                    id = 12;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL:
                    id = 13;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL:
                    id = 14;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS:
                    id = 15;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER:
                    id = 16;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2:
                    id = 17;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2:
                    id = 18;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2:
                    id = 19;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE:
                    id = 20;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS:
                    id = 21;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN:
                    id = 22;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR:
                    id = 23;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING:
                    id = 24;
                    break;
                case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY:
                    id = 25;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 5, data, 162);
        }
    }
    public static class SYSTEM_TIME extends GroundControl.SYSTEM_TIME
    {
        public void time_boot_ms_SET(long  src) //Timestamp of the component clock since boot time in milliseconds.
        {  set_bytes((src) & -1L, 4, data,  0); }
        public void time_unix_usec_SET(long  src) //Timestamp of the master clock in microseconds since UNIX epoch.
        {  set_bytes((src) & -1L, 8, data,  4); }
    }
    public static class POSITION_TARGET_LOCAL_NED extends GroundControl.POSITION_TARGET_LOCAL_NED
    {
        /**
        *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
        *	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
        *	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
        *	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
        *	bit 11: yaw, bit 12: yaw rat*/
        public void type_mask_SET(char  src)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void time_boot_ms_SET(long  src) //Timestamp in milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  2); }
        public void x_SET(float  src) //X Position in NED frame in meters
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }
        public void y_SET(float  src) //Y Position in NED frame in meters
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 10); }
        public void z_SET(float  src) //Z Position in NED frame in meters (note, altitude is negative in NED)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }
        public void vx_SET(float  src) //X velocity in NED frame in meter / s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }
        public void vy_SET(float  src) //Y velocity in NED frame in meter / s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }
        public void vz_SET(float  src) //Z velocity in NED frame in meter / s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 26); }
        public void afx_SET(float  src) //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 30); }
        public void afy_SET(float  src) //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 34); }
        public void afz_SET(float  src) //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 38); }
        public void yaw_SET(float  src) //yaw setpoint in rad
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 42); }
        public void yaw_rate_SET(float  src) //yaw rate setpoint in rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 46); }
        /**
        *Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
        *	=*/
        public void coordinate_frame_SET(@MAV_FRAME int  src)
        {  set_bits(- 0 +   src, 4, data, 400); }
    }
    public static class PING extends GroundControl.PING
    {
        public void seq_SET(long  src) //PING sequence
        {  set_bytes((src) & -1L, 4, data,  0); }
        public void time_usec_SET(long  src) //Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)
        {  set_bytes((src) & -1L, 8, data,  4); }
        /**
        *0: request ping from all receiving systems, if greater than 0: message is a ping response and number is
        *	the system id of the requesting syste*/
        public void target_system_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  12); }
        /**
        *0: request ping from all receiving components, if greater than 0: message is a ping response and number
        *	is the system id of the requesting syste*/
        public void target_component_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  13); }
    }
    public static class CHANGE_OPERATOR_CONTROL extends GroundControl.CHANGE_OPERATOR_CONTROL
    {
        public void target_system_SET(char  src) //System the GCS requests control for
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void control_request_SET(char  src) //0: request control of this MAV, 1: Release control of this MAV
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        /**
        *0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use
        *	the safest mode possible initially and then gradually move down the encryption level if it gets a NACK
        *	message indicating an encryption mismatch*/
        public void version_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        /**
        *Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
        *	characters may involve A-Z, a-z, 0-9, and "!?,.-*/
        public void passkey_SET(String src, Bounds.Inside ph)
        {passkey_SET(src.toCharArray(), 0, src.length(), ph);}/**
*Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
*	characters may involve A-Z, a-z, 0-9, and "!?,.-*/
        public void passkey_SET(char[]  src, int pos, int items, Bounds.Inside ph)
        {
            if(ph.field_bit != 24 && insert_field(ph, 24, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        }
    }
    public static class CHANGE_OPERATOR_CONTROL_ACK extends GroundControl.CHANGE_OPERATOR_CONTROL_ACK
    {
        public void gcs_system_id_SET(char  src) //ID of the GCS this message
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void control_request_SET(char  src) //0: request control of this MAV, 1: Release control of this MAV
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        /**
        *0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under
        *	contro*/
        public void ack_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
    }
    public static class AUTH_KEY extends GroundControl.AUTH_KEY
    {
        public void key_SET(String src, Bounds.Inside ph)//key
        {key_SET(src.toCharArray(), 0, src.length(), ph);} public void key_SET(char[]  src, int pos, int items, Bounds.Inside ph) //key
        {
            if(ph.field_bit != 0 && insert_field(ph, 0, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        }
    }
    public static class SET_MODE extends GroundControl.SET_MODE
    {
        public void custom_mode_SET(long  src) //The new autopilot-specific mode. This field can be ignored by an autopilot.
        {  set_bytes((src) & -1L, 4, data,  0); }
        public void target_system_SET(char  src) //The system setting the mode
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public void base_mode_SET(@MAV_MODE int  src) //The new base mode
        {
            long id = 0;
            switch(src)
            {
                case MAV_MODE.MAV_MODE_PREFLIGHT:
                    id = 0;
                    break;
                case MAV_MODE.MAV_MODE_MANUAL_DISARMED:
                    id = 1;
                    break;
                case MAV_MODE.MAV_MODE_TEST_DISARMED:
                    id = 2;
                    break;
                case MAV_MODE.MAV_MODE_STABILIZE_DISARMED:
                    id = 3;
                    break;
                case MAV_MODE.MAV_MODE_GUIDED_DISARMED:
                    id = 4;
                    break;
                case MAV_MODE.MAV_MODE_AUTO_DISARMED:
                    id = 5;
                    break;
                case MAV_MODE.MAV_MODE_MANUAL_ARMED:
                    id = 6;
                    break;
                case MAV_MODE.MAV_MODE_TEST_ARMED:
                    id = 7;
                    break;
                case MAV_MODE.MAV_MODE_STABILIZE_ARMED:
                    id = 8;
                    break;
                case MAV_MODE.MAV_MODE_GUIDED_ARMED:
                    id = 9;
                    break;
                case MAV_MODE.MAV_MODE_AUTO_ARMED:
                    id = 10;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 4, data, 40);
        }
    }
    public static class PARAM_REQUEST_READ extends GroundControl.PARAM_REQUEST_READ
    {
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void param_index_SET(short  src) //Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored
        {  set_bytes((short)(src) & -1L, 2, data,  2); }
        /**
        *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
        *	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
        *	storage if the ID is stored as strin*/
        public void param_id_SET(String src, Bounds.Inside ph)
        {param_id_SET(src.toCharArray(), 0, src.length(), ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
        public void param_id_SET(char[]  src, int pos, int items, Bounds.Inside ph)
        {
            if(ph.field_bit != 32 && insert_field(ph, 32, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        }
    }
    public static class PARAM_REQUEST_LIST extends GroundControl.PARAM_REQUEST_LIST
    {
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
    }
    public static class PARAM_VALUE extends GroundControl.PARAM_VALUE
    {
        public void param_count_SET(char  src) //Total number of onboard parameters
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void param_index_SET(char  src) //Index of this onboard parameter
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public void param_value_SET(float  src) //Onboard parameter value
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void param_type_SET(@MAV_PARAM_TYPE int  src) //Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
        {  set_bits(- 1 +   src, 4, data, 64); }
        /**
        *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
        *	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
        *	storage if the ID is stored as strin*/
        public void param_id_SET(String src, Bounds.Inside ph)
        {param_id_SET(src.toCharArray(), 0, src.length(), ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
        public void param_id_SET(char[]  src, int pos, int items, Bounds.Inside ph)
        {
            if(ph.field_bit != 68 && insert_field(ph, 68, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        }
    }
    public static class PARAM_SET extends GroundControl.PARAM_SET
    {
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void param_value_SET(float  src) //Onboard parameter value
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 2); }
        public void param_type_SET(@MAV_PARAM_TYPE int  src) //Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
        {  set_bits(- 1 +   src, 4, data, 48); }
        /**
        *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
        *	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
        *	storage if the ID is stored as strin*/
        public void param_id_SET(String src, Bounds.Inside ph)
        {param_id_SET(src.toCharArray(), 0, src.length(), ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
        public void param_id_SET(char[]  src, int pos, int items, Bounds.Inside ph)
        {
            if(ph.field_bit != 52 && insert_field(ph, 52, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        }
    }
    public static class GPS_RAW_INT extends GroundControl.GPS_RAW_INT
    {
        public void eph_SET(char  src) //GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void epv_SET(char  src) //GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public void vel_SET(char  src) //GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        /**
        *Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
        *	unknown, set to: UINT16_MA*/
        public void cog_SET(char  src)
        {  set_bytes((char)(src) & -1L, 2, data,  6); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  set_bytes((src) & -1L, 8, data,  8); }
        public void lat_SET(int  src) //Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  16); }
        public void lon_SET(int  src) //Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  20); }
        /**
        *Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide
        *	the AMSL altitude in addition to the WGS84 altitude*/
        public void alt_SET(int  src)
        {  set_bytes((int)(src) & -1L, 4, data,  24); }
        public void satellites_visible_SET(char  src) //Number of satellites visible. If unknown, set to 255
        {  set_bytes((char)(src) & -1L, 1, data,  28); }
        public void fix_type_SET(@GPS_FIX_TYPE int  src) //See the GPS_FIX_TYPE enum.
        {  set_bits(- 0 +   src, 4, data, 232); }
        public void alt_ellipsoid_SET(int  src, Bounds.Inside ph)//Altitude (above WGS84, EGM96 ellipsoid), in meters * 1000 (positive for up).
        {
            if(ph.field_bit != 236)insert_field(ph, 236, 0);
            set_bytes((int)(src) & -1L, 4, data,  ph.BYTE);
        } public void h_acc_SET(long  src, Bounds.Inside ph) //Position uncertainty in meters * 1000 (positive for up).
        {
            if(ph.field_bit != 237)insert_field(ph, 237, 0);
            set_bytes((src) & -1L, 4, data,  ph.BYTE);
        } public void v_acc_SET(long  src, Bounds.Inside ph) //Altitude uncertainty in meters * 1000 (positive for up).
        {
            if(ph.field_bit != 238)insert_field(ph, 238, 0);
            set_bytes((src) & -1L, 4, data,  ph.BYTE);
        } public void vel_acc_SET(long  src, Bounds.Inside ph) //Speed uncertainty in meters * 1000 (positive for up).
        {
            if(ph.field_bit != 239)insert_field(ph, 239, 0);
            set_bytes((src) & -1L, 4, data,  ph.BYTE);
        } public void hdg_acc_SET(long  src, Bounds.Inside ph) //Heading / track uncertainty in degrees * 1e5.
        {
            if(ph.field_bit != 240)insert_field(ph, 240, 0);
            set_bytes((src) & -1L, 4, data,  ph.BYTE);
        }
    }
    public static class GPS_STATUS extends GroundControl.GPS_STATUS
    {
        public void satellites_visible_SET(char  src) //Number of satellites visible
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void satellite_prn_SET(char[]  src, int pos)  //Global satellite ID
        {
            for(int BYTE =  1, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public void satellite_used_SET(char[]  src, int pos)  //0: Satellite not used, 1: used for localization
        {
            for(int BYTE =  21, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public void satellite_elevation_SET(char[]  src, int pos)  //Elevation (0: right on top of receiver, 90: on the horizon) of satellite
        {
            for(int BYTE =  41, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public void satellite_azimuth_SET(char[]  src, int pos)  //Direction of satellite, 0: 0 deg, 255: 360 deg.
        {
            for(int BYTE =  61, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public void satellite_snr_SET(char[]  src, int pos)  //Signal to noise ratio of satellite
        {
            for(int BYTE =  81, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
    }
    public static class SCALED_IMU extends GroundControl.SCALED_IMU
    {
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public void xacc_SET(short  src) //X acceleration (mg)
        {  set_bytes((short)(src) & -1L, 2, data,  4); }
        public void yacc_SET(short  src) //Y acceleration (mg)
        {  set_bytes((short)(src) & -1L, 2, data,  6); }
        public void zacc_SET(short  src) //Z acceleration (mg)
        {  set_bytes((short)(src) & -1L, 2, data,  8); }
        public void xgyro_SET(short  src) //Angular speed around X axis (millirad /sec)
        {  set_bytes((short)(src) & -1L, 2, data,  10); }
        public void ygyro_SET(short  src) //Angular speed around Y axis (millirad /sec)
        {  set_bytes((short)(src) & -1L, 2, data,  12); }
        public void zgyro_SET(short  src) //Angular speed around Z axis (millirad /sec)
        {  set_bytes((short)(src) & -1L, 2, data,  14); }
        public void xmag_SET(short  src) //X Magnetic field (milli tesla)
        {  set_bytes((short)(src) & -1L, 2, data,  16); }
        public void ymag_SET(short  src) //Y Magnetic field (milli tesla)
        {  set_bytes((short)(src) & -1L, 2, data,  18); }
        public void zmag_SET(short  src) //Z Magnetic field (milli tesla)
        {  set_bytes((short)(src) & -1L, 2, data,  20); }
    }
    public static class RAW_IMU extends GroundControl.RAW_IMU
    {
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  set_bytes((src) & -1L, 8, data,  0); }
        public void xacc_SET(short  src) //X acceleration (raw)
        {  set_bytes((short)(src) & -1L, 2, data,  8); }
        public void yacc_SET(short  src) //Y acceleration (raw)
        {  set_bytes((short)(src) & -1L, 2, data,  10); }
        public void zacc_SET(short  src) //Z acceleration (raw)
        {  set_bytes((short)(src) & -1L, 2, data,  12); }
        public void xgyro_SET(short  src) //Angular speed around X axis (raw)
        {  set_bytes((short)(src) & -1L, 2, data,  14); }
        public void ygyro_SET(short  src) //Angular speed around Y axis (raw)
        {  set_bytes((short)(src) & -1L, 2, data,  16); }
        public void zgyro_SET(short  src) //Angular speed around Z axis (raw)
        {  set_bytes((short)(src) & -1L, 2, data,  18); }
        public void xmag_SET(short  src) //X Magnetic field (raw)
        {  set_bytes((short)(src) & -1L, 2, data,  20); }
        public void ymag_SET(short  src) //Y Magnetic field (raw)
        {  set_bytes((short)(src) & -1L, 2, data,  22); }
        public void zmag_SET(short  src) //Z Magnetic field (raw)
        {  set_bytes((short)(src) & -1L, 2, data,  24); }
    }
    public static class RAW_PRESSURE extends GroundControl.RAW_PRESSURE
    {
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  set_bytes((src) & -1L, 8, data,  0); }
        public void press_abs_SET(short  src) //Absolute pressure (raw)
        {  set_bytes((short)(src) & -1L, 2, data,  8); }
        public void press_diff1_SET(short  src) //Differential pressure 1 (raw, 0 if nonexistant)
        {  set_bytes((short)(src) & -1L, 2, data,  10); }
        public void press_diff2_SET(short  src) //Differential pressure 2 (raw, 0 if nonexistant)
        {  set_bytes((short)(src) & -1L, 2, data,  12); }
        public void temperature_SET(short  src) //Raw Temperature measurement (raw)
        {  set_bytes((short)(src) & -1L, 2, data,  14); }
    }
    public static class SCALED_PRESSURE extends GroundControl.SCALED_PRESSURE
    {
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public void press_abs_SET(float  src) //Absolute pressure (hectopascal)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void press_diff_SET(float  src) //Differential pressure 1 (hectopascal)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void temperature_SET(short  src) //Temperature measurement (0.01 degrees celsius)
        {  set_bytes((short)(src) & -1L, 2, data,  12); }
    }
    public static class ATTITUDE extends GroundControl.ATTITUDE
    {
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public void roll_SET(float  src) //Roll angle (rad, -pi..+pi)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void pitch_SET(float  src) //Pitch angle (rad, -pi..+pi)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void yaw_SET(float  src) //Yaw angle (rad, -pi..+pi)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void rollspeed_SET(float  src) //Roll angular speed (rad/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public void pitchspeed_SET(float  src) //Pitch angular speed (rad/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public void yawspeed_SET(float  src) //Yaw angular speed (rad/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
    }
    public static class ATTITUDE_QUATERNION extends GroundControl.ATTITUDE_QUATERNION
    {
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public void q1_SET(float  src) //Quaternion component 1, w (1 in null-rotation)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void q2_SET(float  src) //Quaternion component 2, x (0 in null-rotation)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void q3_SET(float  src) //Quaternion component 3, y (0 in null-rotation)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void q4_SET(float  src) //Quaternion component 4, z (0 in null-rotation)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public void rollspeed_SET(float  src) //Roll angular speed (rad/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public void pitchspeed_SET(float  src) //Pitch angular speed (rad/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public void yawspeed_SET(float  src) //Yaw angular speed (rad/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
    }
    public static class LOCAL_POSITION_NED extends GroundControl.LOCAL_POSITION_NED
    {
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public void x_SET(float  src) //X Position
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void y_SET(float  src) //Y Position
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void z_SET(float  src) //Z Position
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void vx_SET(float  src) //X Speed
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public void vy_SET(float  src) //Y Speed
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public void vz_SET(float  src) //Z Speed
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
    }
    public static class GLOBAL_POSITION_INT extends GroundControl.GLOBAL_POSITION_INT
    {
        public void hdg_SET(char  src) //Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  2); }
        public void lat_SET(int  src) //Latitude, expressed as degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  6); }
        public void lon_SET(int  src) //Longitude, expressed as degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  10); }
        /**
        *Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules
        *	provide the AMSL as well*/
        public void alt_SET(int  src)
        {  set_bytes((int)(src) & -1L, 4, data,  14); }
        public void relative_alt_SET(int  src) //Altitude above ground in meters, expressed as * 1000 (millimeters)
        {  set_bytes((int)(src) & -1L, 4, data,  18); }
        public void vx_SET(short  src) //Ground X Speed (Latitude, positive north), expressed as m/s * 100
        {  set_bytes((short)(src) & -1L, 2, data,  22); }
        public void vy_SET(short  src) //Ground Y Speed (Longitude, positive east), expressed as m/s * 100
        {  set_bytes((short)(src) & -1L, 2, data,  24); }
        public void vz_SET(short  src) //Ground Z Speed (Altitude, positive down), expressed as m/s * 100
        {  set_bytes((short)(src) & -1L, 2, data,  26); }
    }
    public static class RC_CHANNELS_SCALED extends GroundControl.RC_CHANNELS_SCALED
    {
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        /**
        *Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than
        *	8 servos*/
        public void port_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public void chan1_scaled_SET(short  src) //RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        {  set_bytes((short)(src) & -1L, 2, data,  5); }
        public void chan2_scaled_SET(short  src) //RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        {  set_bytes((short)(src) & -1L, 2, data,  7); }
        public void chan3_scaled_SET(short  src) //RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        {  set_bytes((short)(src) & -1L, 2, data,  9); }
        public void chan4_scaled_SET(short  src) //RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        {  set_bytes((short)(src) & -1L, 2, data,  11); }
        public void chan5_scaled_SET(short  src) //RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        {  set_bytes((short)(src) & -1L, 2, data,  13); }
        public void chan6_scaled_SET(short  src) //RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        {  set_bytes((short)(src) & -1L, 2, data,  15); }
        public void chan7_scaled_SET(short  src) //RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        {  set_bytes((short)(src) & -1L, 2, data,  17); }
        public void chan8_scaled_SET(short  src) //RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        {  set_bytes((short)(src) & -1L, 2, data,  19); }
        public void rssi_SET(char  src) //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
        {  set_bytes((char)(src) & -1L, 1, data,  21); }
    }
    public static class RC_CHANNELS_RAW extends GroundControl.RC_CHANNELS_RAW
    {
        public void chan1_raw_SET(char  src) //RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void chan2_raw_SET(char  src) //RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public void chan3_raw_SET(char  src) //RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public void chan4_raw_SET(char  src) //RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  6); }
        public void chan5_raw_SET(char  src) //RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  8); }
        public void chan6_raw_SET(char  src) //RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  10); }
        public void chan7_raw_SET(char  src) //RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  12); }
        public void chan8_raw_SET(char  src) //RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  14); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  16); }
        /**
        *Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than
        *	8 servos*/
        public void port_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  20); }
        public void rssi_SET(char  src) //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
        {  set_bytes((char)(src) & -1L, 1, data,  21); }
    }
    public static class SERVO_OUTPUT_RAW extends GroundControl.SERVO_OUTPUT_RAW
    {
        public void servo1_raw_SET(char  src) //Servo output 1 value, in microseconds
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void servo2_raw_SET(char  src) //Servo output 2 value, in microseconds
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public void servo3_raw_SET(char  src) //Servo output 3 value, in microseconds
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public void servo4_raw_SET(char  src) //Servo output 4 value, in microseconds
        {  set_bytes((char)(src) & -1L, 2, data,  6); }
        public void servo5_raw_SET(char  src) //Servo output 5 value, in microseconds
        {  set_bytes((char)(src) & -1L, 2, data,  8); }
        public void servo6_raw_SET(char  src) //Servo output 6 value, in microseconds
        {  set_bytes((char)(src) & -1L, 2, data,  10); }
        public void servo7_raw_SET(char  src) //Servo output 7 value, in microseconds
        {  set_bytes((char)(src) & -1L, 2, data,  12); }
        public void servo8_raw_SET(char  src) //Servo output 8 value, in microseconds
        {  set_bytes((char)(src) & -1L, 2, data,  14); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  16); }
        /**
        *Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode
        *	more than 8 servos*/
        public void port_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  20); }
        public void servo9_raw_SET(char  src, Bounds.Inside ph)//Servo output 9 value, in microseconds
        {
            if(ph.field_bit != 168)insert_field(ph, 168, 0);
            set_bytes((char)(src) & -1L, 2, data,  ph.BYTE);
        } public void servo10_raw_SET(char  src, Bounds.Inside ph) //Servo output 10 value, in microseconds
        {
            if(ph.field_bit != 169)insert_field(ph, 169, 0);
            set_bytes((char)(src) & -1L, 2, data,  ph.BYTE);
        } public void servo11_raw_SET(char  src, Bounds.Inside ph) //Servo output 11 value, in microseconds
        {
            if(ph.field_bit != 170)insert_field(ph, 170, 0);
            set_bytes((char)(src) & -1L, 2, data,  ph.BYTE);
        } public void servo12_raw_SET(char  src, Bounds.Inside ph) //Servo output 12 value, in microseconds
        {
            if(ph.field_bit != 171)insert_field(ph, 171, 0);
            set_bytes((char)(src) & -1L, 2, data,  ph.BYTE);
        } public void servo13_raw_SET(char  src, Bounds.Inside ph) //Servo output 13 value, in microseconds
        {
            if(ph.field_bit != 172)insert_field(ph, 172, 0);
            set_bytes((char)(src) & -1L, 2, data,  ph.BYTE);
        } public void servo14_raw_SET(char  src, Bounds.Inside ph) //Servo output 14 value, in microseconds
        {
            if(ph.field_bit != 173)insert_field(ph, 173, 0);
            set_bytes((char)(src) & -1L, 2, data,  ph.BYTE);
        } public void servo15_raw_SET(char  src, Bounds.Inside ph) //Servo output 15 value, in microseconds
        {
            if(ph.field_bit != 174)insert_field(ph, 174, 0);
            set_bytes((char)(src) & -1L, 2, data,  ph.BYTE);
        } public void servo16_raw_SET(char  src, Bounds.Inside ph) //Servo output 16 value, in microseconds
        {
            if(ph.field_bit != 175)insert_field(ph, 175, 0);
            set_bytes((char)(src) & -1L, 2, data,  ph.BYTE);
        }
    }
    public static class MISSION_REQUEST_PARTIAL_LIST extends GroundControl.MISSION_REQUEST_PARTIAL_LIST
    {
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void start_index_SET(short  src) //Start index, 0 by default
        {  set_bytes((short)(src) & -1L, 2, data,  2); }
        public void end_index_SET(short  src) //End index, -1 by default (-1: send list to end). Else a valid index of the list
        {  set_bytes((short)(src) & -1L, 2, data,  4); }
        public void mission_type_SET(@MAV_MISSION_TYPE int  src) //Mission type, see MAV_MISSION_TYPE
        {
            long id = 0;
            switch(src)
            {
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION:
                    id = 0;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE:
                    id = 1;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY:
                    id = 2;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                    id = 3;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 48);
        }
    }
    public static class MISSION_WRITE_PARTIAL_LIST extends GroundControl.MISSION_WRITE_PARTIAL_LIST
    {
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void start_index_SET(short  src) //Start index, 0 by default and smaller / equal to the largest index of the current onboard list.
        {  set_bytes((short)(src) & -1L, 2, data,  2); }
        public void end_index_SET(short  src) //End index, equal or greater than start index.
        {  set_bytes((short)(src) & -1L, 2, data,  4); }
        public void mission_type_SET(@MAV_MISSION_TYPE int  src) //Mission type, see MAV_MISSION_TYPE
        {
            long id = 0;
            switch(src)
            {
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION:
                    id = 0;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE:
                    id = 1;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY:
                    id = 2;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                    id = 3;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 48);
        }
    }
    public static class MISSION_ITEM extends GroundControl.MISSION_ITEM
    {
        public void seq_SET(char  src) //Sequence
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public void current_SET(char  src) //false:0, true:1
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public void autocontinue_SET(char  src) //autocontinue to next wp
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public void param1_SET(float  src) //PARAM1, see MAV_CMD enum
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }
        public void param2_SET(float  src) //PARAM2, see MAV_CMD enum
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 10); }
        public void param3_SET(float  src) //PARAM3, see MAV_CMD enum
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }
        public void param4_SET(float  src) //PARAM4, see MAV_CMD enum
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }
        public void x_SET(float  src) //PARAM5 / local: x position, global: latitude
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }
        public void y_SET(float  src) //PARAM6 / y position: global: longitude
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 26); }
        public void z_SET(float  src) //PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 30); }
        public void frame_SET(@MAV_FRAME int  src) //The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
        {  set_bits(- 0 +   src, 4, data, 272); }
        public void command_SET(@MAV_CMD int  src) //The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs
        {
            long id = 0;
            switch(src)
            {
                case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                    id = 0;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
                    id = 1;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
                    id = 2;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
                    id = 3;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                    id = 4;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND:
                    id = 5;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                    id = 6;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
                    id = 7;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
                    id = 8;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FOLLOW:
                    id = 9;
                    break;
                case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
                    id = 10;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
                    id = 11;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW:
                    id = 12;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
                    id = 13;
                    break;
                case MAV_CMD.MAV_CMD_NAV_ROI:
                    id = 14;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
                    id = 15;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
                    id = 16;
                    break;
                case MAV_CMD.MAV_CMD_NAV_ALTITUDE_WAIT:
                    id = 17;
                    break;
                case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
                    id = 18;
                    break;
                case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
                    id = 19;
                    break;
                case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
                    id = 20;
                    break;
                case MAV_CMD.MAV_CMD_NAV_DELAY:
                    id = 21;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
                    id = 22;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAST:
                    id = 23;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DELAY:
                    id = 24;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
                    id = 25;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
                    id = 26;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_YAW:
                    id = 27;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_LAST:
                    id = 28;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_MODE:
                    id = 29;
                    break;
                case MAV_CMD.MAV_CMD_DO_JUMP:
                    id = 30;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
                    id = 31;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_HOME:
                    id = 32;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
                    id = 33;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_RELAY:
                    id = 34;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
                    id = 35;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_SERVO:
                    id = 36;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
                    id = 37;
                    break;
                case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
                    id = 38;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
                    id = 39;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAND_START:
                    id = 40;
                    break;
                case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
                    id = 41;
                    break;
                case MAV_CMD.MAV_CMD_DO_GO_AROUND:
                    id = 42;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPOSITION:
                    id = 43;
                    break;
                case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
                    id = 44;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
                    id = 45;
                    break;
                case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
                    id = 46;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_ROI:
                    id = 47;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
                    id = 48;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
                    id = 49;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
                    id = 50;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                    id = 51;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                    id = 52;
                    break;
                case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
                    id = 53;
                    break;
                case MAV_CMD.MAV_CMD_DO_PARACHUTE:
                    id = 54;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
                    id = 55;
                    break;
                case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
                    id = 56;
                    break;
                case MAV_CMD.MAV_CMD_DO_GRIPPER:
                    id = 57;
                    break;
                case MAV_CMD.MAV_CMD_DO_AUTOTUNE_ENABLE:
                    id = 58;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                    id = 59;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                    id = 60;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                    id = 61;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                    id = 62;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                    id = 63;
                    break;
                case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                    id = 64;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAST:
                    id = 65;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                    id = 66;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                    id = 67;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                    id = 68;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                    id = 69;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                    id = 70;
                    break;
                case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                    id = 71;
                    break;
                case MAV_CMD.MAV_CMD_MISSION_START:
                    id = 72;
                    break;
                case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                    id = 73;
                    break;
                case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                    id = 74;
                    break;
                case MAV_CMD.MAV_CMD_START_RX_PAIR:
                    id = 75;
                    break;
                case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                    id = 76;
                    break;
                case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                    id = 77;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                    id = 78;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                    id = 79;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                    id = 80;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                    id = 81;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                    id = 82;
                    break;
                case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                    id = 83;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                    id = 84;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                    id = 85;
                    break;
                case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                    id = 86;
                    break;
                case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                    id = 87;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                    id = 88;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                    id = 89;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                    id = 90;
                    break;
                case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                    id = 91;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                    id = 92;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                    id = 93;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                    id = 94;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                    id = 95;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                    id = 96;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_START:
                    id = 97;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_STOP:
                    id = 98;
                    break;
                case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                    id = 99;
                    break;
                case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                    id = 100;
                    break;
                case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                    id = 101;
                    break;
                case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                    id = 102;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                    id = 103;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                    id = 104;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_GATE:
                    id = 105;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                    id = 106;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                    id = 107;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                    id = 108;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                    id = 109;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                    id = 110;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                    id = 111;
                    break;
                case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                    id = 112;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                    id = 113;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                    id = 114;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                    id = 115;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                    id = 116;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                    id = 117;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                    id = 118;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                    id = 119;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                    id = 120;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                    id = 121;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                    id = 122;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                    id = 123;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                    id = 124;
                    break;
                case MAV_CMD.MAV_CMD_USER_1:
                    id = 125;
                    break;
                case MAV_CMD.MAV_CMD_USER_2:
                    id = 126;
                    break;
                case MAV_CMD.MAV_CMD_USER_3:
                    id = 127;
                    break;
                case MAV_CMD.MAV_CMD_USER_4:
                    id = 128;
                    break;
                case MAV_CMD.MAV_CMD_USER_5:
                    id = 129;
                    break;
                case MAV_CMD.MAV_CMD_POWER_OFF_INITIATED:
                    id = 130;
                    break;
                case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_CLICK:
                    id = 131;
                    break;
                case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_HOLD:
                    id = 132;
                    break;
                case MAV_CMD.MAV_CMD_SOLO_BTN_PAUSE_CLICK:
                    id = 133;
                    break;
                case MAV_CMD.MAV_CMD_DO_START_MAG_CAL:
                    id = 134;
                    break;
                case MAV_CMD.MAV_CMD_DO_ACCEPT_MAG_CAL:
                    id = 135;
                    break;
                case MAV_CMD.MAV_CMD_DO_CANCEL_MAG_CAL:
                    id = 136;
                    break;
                case MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE:
                    id = 137;
                    break;
                case MAV_CMD.MAV_CMD_DO_SEND_BANNER:
                    id = 138;
                    break;
                case MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS:
                    id = 139;
                    break;
                case MAV_CMD.MAV_CMD_GIMBAL_RESET:
                    id = 140;
                    break;
                case MAV_CMD.MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS:
                    id = 141;
                    break;
                case MAV_CMD.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION:
                    id = 142;
                    break;
                case MAV_CMD.MAV_CMD_GIMBAL_FULL_RESET:
                    id = 143;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 8, data, 276);
        }
        public void mission_type_SET(@MAV_MISSION_TYPE int  src) //Mission type, see MAV_MISSION_TYPE
        {
            long id = 0;
            switch(src)
            {
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION:
                    id = 0;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE:
                    id = 1;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY:
                    id = 2;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                    id = 3;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 284);
        }
    }
    public static class MISSION_REQUEST extends GroundControl.MISSION_REQUEST
    {
        public void seq_SET(char  src) //Sequence
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public void mission_type_SET(@MAV_MISSION_TYPE int  src) //Mission type, see MAV_MISSION_TYPE
        {
            long id = 0;
            switch(src)
            {
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION:
                    id = 0;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE:
                    id = 1;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY:
                    id = 2;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                    id = 3;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 32);
        }
    }
    public static class MISSION_SET_CURRENT extends GroundControl.MISSION_SET_CURRENT
    {
        public void seq_SET(char  src) //Sequence
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
    }
    public static class MISSION_CURRENT extends GroundControl.MISSION_CURRENT
    {
        public void seq_SET(char  src) //Sequence
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
    }
    public static class MISSION_REQUEST_LIST extends GroundControl.MISSION_REQUEST_LIST
    {
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void mission_type_SET(@MAV_MISSION_TYPE int  src) //Mission type, see MAV_MISSION_TYPE
        {
            long id = 0;
            switch(src)
            {
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION:
                    id = 0;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE:
                    id = 1;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY:
                    id = 2;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                    id = 3;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 16);
        }
    }
    public static class MISSION_COUNT extends GroundControl.MISSION_COUNT
    {
        public void count_SET(char  src) //Number of mission items in the sequence
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public void mission_type_SET(@MAV_MISSION_TYPE int  src) //Mission type, see MAV_MISSION_TYPE
        {
            long id = 0;
            switch(src)
            {
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION:
                    id = 0;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE:
                    id = 1;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY:
                    id = 2;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                    id = 3;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 32);
        }
    }
    public static class MISSION_CLEAR_ALL extends GroundControl.MISSION_CLEAR_ALL
    {
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void mission_type_SET(@MAV_MISSION_TYPE int  src) //Mission type, see MAV_MISSION_TYPE
        {
            long id = 0;
            switch(src)
            {
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION:
                    id = 0;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE:
                    id = 1;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY:
                    id = 2;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                    id = 3;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 16);
        }
    }
    public static class MISSION_ITEM_REACHED extends GroundControl.MISSION_ITEM_REACHED
    {
        public void seq_SET(char  src) //Sequence
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
    }
    public static class MISSION_ACK extends GroundControl.MISSION_ACK
    {
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void type_SET(@MAV_MISSION_RESULT int  src) //See MAV_MISSION_RESULT enum
        {  set_bits(- 0 +   src, 4, data, 16); }
        public void mission_type_SET(@MAV_MISSION_TYPE int  src) //Mission type, see MAV_MISSION_TYPE
        {
            long id = 0;
            switch(src)
            {
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION:
                    id = 0;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE:
                    id = 1;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY:
                    id = 2;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                    id = 3;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 20);
        }
    }
    public static class SET_GPS_GLOBAL_ORIGIN extends GroundControl.SET_GPS_GLOBAL_ORIGIN
    {
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void latitude_SET(int  src) //Latitude (WGS84), in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  1); }
        public void longitude_SET(int  src) //Longitude (WGS84, in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  5); }
        public void altitude_SET(int  src) //Altitude (AMSL), in meters * 1000 (positive for up)
        {  set_bytes((int)(src) & -1L, 4, data,  9); }
        public void time_usec_SET(long  src, Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {
            if(ph.field_bit != 104)insert_field(ph, 104, 0);
            set_bytes((src) & -1L, 8, data,  ph.BYTE);
        }
    }
    public static class GPS_GLOBAL_ORIGIN extends GroundControl.GPS_GLOBAL_ORIGIN
    {
        public void latitude_SET(int  src) //Latitude (WGS84), in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  0); }
        public void longitude_SET(int  src) //Longitude (WGS84), in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  4); }
        public void altitude_SET(int  src) //Altitude (AMSL), in meters * 1000 (positive for up)
        {  set_bytes((int)(src) & -1L, 4, data,  8); }
        public void time_usec_SET(long  src, Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {
            if(ph.field_bit != 96)insert_field(ph, 96, 0);
            set_bytes((src) & -1L, 8, data,  ph.BYTE);
        }
    }
    public static class PARAM_MAP_RC extends GroundControl.PARAM_MAP_RC
    {
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        /**
        *Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored),
        *	send -2 to disable any existing map for this rc_channel_index*/
        public void param_index_SET(short  src)
        {  set_bytes((short)(src) & -1L, 2, data,  2); }
        /**
        *Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob
        *	on the RC*/
        public void parameter_rc_channel_index_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public void param_value0_SET(float  src) //Initial parameter value
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 5); }
        public void scale_SET(float  src) //Scale, maps the RC range [-1, 1] to a parameter value
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 9); }
        /**
        *Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends
        *	on implementation*/
        public void param_value_min_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 13); }
        /**
        *Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends
        *	on implementation*/
        public void param_value_max_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 17); }
        /**
        *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
        *	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
        *	storage if the ID is stored as strin*/
        public void param_id_SET(String src, Bounds.Inside ph)
        {param_id_SET(src.toCharArray(), 0, src.length(), ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
        public void param_id_SET(char[]  src, int pos, int items, Bounds.Inside ph)
        {
            if(ph.field_bit != 168 && insert_field(ph, 168, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        }
    }
    public static class MISSION_REQUEST_INT extends GroundControl.MISSION_REQUEST_INT
    {
        public void seq_SET(char  src) //Sequence
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public void mission_type_SET(@MAV_MISSION_TYPE int  src) //Mission type, see MAV_MISSION_TYPE
        {
            long id = 0;
            switch(src)
            {
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION:
                    id = 0;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE:
                    id = 1;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY:
                    id = 2;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                    id = 3;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 32);
        }
    }
    public static class SAFETY_SET_ALLOWED_AREA extends GroundControl.SAFETY_SET_ALLOWED_AREA
    {
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void p1x_SET(float  src) //x position 1 / Latitude 1
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 2); }
        public void p1y_SET(float  src) //y position 1 / Longitude 1
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }
        public void p1z_SET(float  src) //z position 1 / Altitude 1
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 10); }
        public void p2x_SET(float  src) //x position 2 / Latitude 2
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }
        public void p2y_SET(float  src) //y position 2 / Longitude 2
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }
        public void p2z_SET(float  src) //z position 2 / Altitude 2
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }
        /**
        *Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed
        *	with Z axis up or local, right handed, Z axis down*/
        public void frame_SET(@MAV_FRAME int  src)
        {  set_bits(- 0 +   src, 4, data, 208); }
    }
    public static class SAFETY_ALLOWED_AREA extends GroundControl.SAFETY_ALLOWED_AREA
    {
        public void p1x_SET(float  src) //x position 1 / Latitude 1
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 0); }
        public void p1y_SET(float  src) //y position 1 / Longitude 1
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void p1z_SET(float  src) //z position 1 / Altitude 1
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void p2x_SET(float  src) //x position 2 / Latitude 2
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void p2y_SET(float  src) //y position 2 / Longitude 2
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public void p2z_SET(float  src) //z position 2 / Altitude 2
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        /**
        *Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed
        *	with Z axis up or local, right handed, Z axis down*/
        public void frame_SET(@MAV_FRAME int  src)
        {  set_bits(- 0 +   src, 4, data, 192); }
    }
    public static class ATTITUDE_QUATERNION_COV extends GroundControl.ATTITUDE_QUATERNION_COV
    {
        public void time_usec_SET(long  src) //Timestamp (microseconds since system boot or since UNIX epoch)
        {  set_bytes((src) & -1L, 8, data,  0); }
        public void q_SET(float[]  src, int pos)  //Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
        {
            for(int BYTE =  8, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public void rollspeed_SET(float  src) //Roll angular speed (rad/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public void pitchspeed_SET(float  src) //Pitch angular speed (rad/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public void yawspeed_SET(float  src) //Yaw angular speed (rad/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public void covariance_SET(float[]  src, int pos)  //Attitude covariance
        {
            for(int BYTE =  36, src_max = pos + 9; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
    }
    public static class NAV_CONTROLLER_OUTPUT extends GroundControl.NAV_CONTROLLER_OUTPUT
    {
        public void wp_dist_SET(char  src) //Distance to active waypoint in meters
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void nav_roll_SET(float  src) //Current desired roll in degrees
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 2); }
        public void nav_pitch_SET(float  src) //Current desired pitch in degrees
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }
        public void nav_bearing_SET(short  src) //Current desired heading in degrees
        {  set_bytes((short)(src) & -1L, 2, data,  10); }
        public void target_bearing_SET(short  src) //Bearing to current waypoint/target in degrees
        {  set_bytes((short)(src) & -1L, 2, data,  12); }
        public void alt_error_SET(float  src) //Current altitude error in meters
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }
        public void aspd_error_SET(float  src) //Current airspeed error in meters/second
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }
        public void xtrack_error_SET(float  src) //Current crosstrack error on x-y plane in meters
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }
    }
    public static class GLOBAL_POSITION_INT_COV extends GroundControl.GLOBAL_POSITION_INT_COV
    {
        public void time_usec_SET(long  src) //Timestamp (microseconds since system boot or since UNIX epoch)
        {  set_bytes((src) & -1L, 8, data,  0); }
        public void lat_SET(int  src) //Latitude, expressed as degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  8); }
        public void lon_SET(int  src) //Longitude, expressed as degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  12); }
        public void alt_SET(int  src) //Altitude in meters, expressed as * 1000 (millimeters), above MSL
        {  set_bytes((int)(src) & -1L, 4, data,  16); }
        public void relative_alt_SET(int  src) //Altitude above ground in meters, expressed as * 1000 (millimeters)
        {  set_bytes((int)(src) & -1L, 4, data,  20); }
        public void vx_SET(float  src) //Ground X Speed (Latitude), expressed as m/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public void vy_SET(float  src) //Ground Y Speed (Longitude), expressed as m/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public void vz_SET(float  src) //Ground Z Speed (Altitude), expressed as m/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public void covariance_SET(float[]  src, int pos)  //Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
        {
            for(int BYTE =  36, src_max = pos + 36; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public void estimator_type_SET(@MAV_ESTIMATOR_TYPE int  src) //Class id of the estimator this estimate originated from.
        {  set_bits(- 1 +   src, 3, data, 1440); }
    }
    public static class LOCAL_POSITION_NED_COV extends GroundControl.LOCAL_POSITION_NED_COV
    {
        public void time_usec_SET(long  src) //Timestamp (microseconds since system boot or since UNIX epoch)
        {  set_bytes((src) & -1L, 8, data,  0); }
        public void x_SET(float  src) //X Position
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void y_SET(float  src) //Y Position
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void z_SET(float  src) //Z Position
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public void vx_SET(float  src) //X Speed (m/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public void vy_SET(float  src) //Y Speed (m/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public void vz_SET(float  src) //Z Speed (m/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public void ax_SET(float  src) //X Acceleration (m/s^2)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public void ay_SET(float  src) //Y Acceleration (m/s^2)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public void az_SET(float  src) //Z Acceleration (m/s^2)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        /**
        *Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are
        *	the second row, etc.*/
        public void covariance_SET(float[]  src, int pos)
        {
            for(int BYTE =  44, src_max = pos + 45; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public void estimator_type_SET(@MAV_ESTIMATOR_TYPE int  src) //Class id of the estimator this estimate originated from.
        {  set_bits(- 1 +   src, 3, data, 1792); }
    }
    public static class RC_CHANNELS extends GroundControl.RC_CHANNELS
    {
        public void chan1_raw_SET(char  src) //RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void chan2_raw_SET(char  src) //RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public void chan3_raw_SET(char  src) //RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public void chan4_raw_SET(char  src) //RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  6); }
        public void chan5_raw_SET(char  src) //RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  8); }
        public void chan6_raw_SET(char  src) //RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  10); }
        public void chan7_raw_SET(char  src) //RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  12); }
        public void chan8_raw_SET(char  src) //RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  14); }
        public void chan9_raw_SET(char  src) //RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  16); }
        public void chan10_raw_SET(char  src) //RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  18); }
        public void chan11_raw_SET(char  src) //RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  20); }
        public void chan12_raw_SET(char  src) //RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  22); }
        public void chan13_raw_SET(char  src) //RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  24); }
        public void chan14_raw_SET(char  src) //RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  26); }
        public void chan15_raw_SET(char  src) //RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  28); }
        public void chan16_raw_SET(char  src) //RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  30); }
        public void chan17_raw_SET(char  src) //RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  32); }
        public void chan18_raw_SET(char  src) //RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  set_bytes((char)(src) & -1L, 2, data,  34); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  36); }
        /**
        *Total number of RC channels being received. This can be larger than 18, indicating that more channels
        *	are available but not given in this message. This value should be 0 when no RC channels are available*/
        public void chancount_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  40); }
        public void rssi_SET(char  src) //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
        {  set_bytes((char)(src) & -1L, 1, data,  41); }
    }
    public static class REQUEST_DATA_STREAM extends GroundControl.REQUEST_DATA_STREAM
    {
        public void req_message_rate_SET(char  src) //The requested message rate
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void target_system_SET(char  src) //The target requested to send the message stream.
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void target_component_SET(char  src) //The target requested to send the message stream.
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public void req_stream_id_SET(char  src) //The ID of the requested data stream
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public void start_stop_SET(char  src) //1 to start sending, 0 to stop sending.
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
    }
    public static class DATA_STREAM extends GroundControl.DATA_STREAM
    {
        public void message_rate_SET(char  src) //The message rate
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void stream_id_SET(char  src) //The ID of the requested data stream
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void on_off_SET(char  src) //1 stream is enabled, 0 stream is stopped.
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
    }
    public static class MANUAL_CONTROL extends GroundControl.MANUAL_CONTROL
    {
        /**
        *A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest
        *	bit corresponds to Button 1*/
        public void buttons_SET(char  src)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void target_SET(char  src) //The system to be controlled.
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        /**
        *X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
        *	Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle*/
        public void x_SET(short  src)
        {  set_bytes((short)(src) & -1L, 2, data,  3); }
        /**
        *Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
        *	Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle*/
        public void y_SET(short  src)
        {  set_bytes((short)(src) & -1L, 2, data,  5); }
        /**
        *Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
        *	Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on
        *	a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative
        *	thrust*/
        public void z_SET(short  src)
        {  set_bytes((short)(src) & -1L, 2, data,  7); }
        /**
        *R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
        *	Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise
        *	being -1000, and the yaw of a vehicle*/
        public void r_SET(short  src)
        {  set_bytes((short)(src) & -1L, 2, data,  9); }
    }
    public static class RC_CHANNELS_OVERRIDE extends GroundControl.RC_CHANNELS_OVERRIDE
    {
        public void chan1_raw_SET(char  src) //RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void chan2_raw_SET(char  src) //RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public void chan3_raw_SET(char  src) //RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public void chan4_raw_SET(char  src) //RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        {  set_bytes((char)(src) & -1L, 2, data,  6); }
        public void chan5_raw_SET(char  src) //RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        {  set_bytes((char)(src) & -1L, 2, data,  8); }
        public void chan6_raw_SET(char  src) //RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        {  set_bytes((char)(src) & -1L, 2, data,  10); }
        public void chan7_raw_SET(char  src) //RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        {  set_bytes((char)(src) & -1L, 2, data,  12); }
        public void chan8_raw_SET(char  src) //RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        {  set_bytes((char)(src) & -1L, 2, data,  14); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  16); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  17); }
    }
    public static class MISSION_ITEM_INT extends GroundControl.MISSION_ITEM_INT
    {
        /**
        *Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the
        *	sequence (0,1,2,3,4)*/
        public void seq_SET(char  src)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public void current_SET(char  src) //false:0, true:1
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public void autocontinue_SET(char  src) //autocontinue to next wp
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public void param1_SET(float  src) //PARAM1, see MAV_CMD enum
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }
        public void param2_SET(float  src) //PARAM2, see MAV_CMD enum
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 10); }
        public void param3_SET(float  src) //PARAM3, see MAV_CMD enum
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }
        public void param4_SET(float  src) //PARAM4, see MAV_CMD enum
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }
        public void x_SET(int  src) //PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
        {  set_bytes((int)(src) & -1L, 4, data,  22); }
        public void y_SET(int  src) //PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
        {  set_bytes((int)(src) & -1L, 4, data,  26); }
        public void z_SET(float  src) //PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 30); }
        public void frame_SET(@MAV_FRAME int  src) //The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
        {  set_bits(- 0 +   src, 4, data, 272); }
        public void command_SET(@MAV_CMD int  src) //The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs
        {
            long id = 0;
            switch(src)
            {
                case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                    id = 0;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
                    id = 1;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
                    id = 2;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
                    id = 3;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                    id = 4;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND:
                    id = 5;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                    id = 6;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
                    id = 7;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
                    id = 8;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FOLLOW:
                    id = 9;
                    break;
                case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
                    id = 10;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
                    id = 11;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW:
                    id = 12;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
                    id = 13;
                    break;
                case MAV_CMD.MAV_CMD_NAV_ROI:
                    id = 14;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
                    id = 15;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
                    id = 16;
                    break;
                case MAV_CMD.MAV_CMD_NAV_ALTITUDE_WAIT:
                    id = 17;
                    break;
                case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
                    id = 18;
                    break;
                case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
                    id = 19;
                    break;
                case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
                    id = 20;
                    break;
                case MAV_CMD.MAV_CMD_NAV_DELAY:
                    id = 21;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
                    id = 22;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAST:
                    id = 23;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DELAY:
                    id = 24;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
                    id = 25;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
                    id = 26;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_YAW:
                    id = 27;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_LAST:
                    id = 28;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_MODE:
                    id = 29;
                    break;
                case MAV_CMD.MAV_CMD_DO_JUMP:
                    id = 30;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
                    id = 31;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_HOME:
                    id = 32;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
                    id = 33;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_RELAY:
                    id = 34;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
                    id = 35;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_SERVO:
                    id = 36;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
                    id = 37;
                    break;
                case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
                    id = 38;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
                    id = 39;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAND_START:
                    id = 40;
                    break;
                case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
                    id = 41;
                    break;
                case MAV_CMD.MAV_CMD_DO_GO_AROUND:
                    id = 42;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPOSITION:
                    id = 43;
                    break;
                case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
                    id = 44;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
                    id = 45;
                    break;
                case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
                    id = 46;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_ROI:
                    id = 47;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
                    id = 48;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
                    id = 49;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
                    id = 50;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                    id = 51;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                    id = 52;
                    break;
                case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
                    id = 53;
                    break;
                case MAV_CMD.MAV_CMD_DO_PARACHUTE:
                    id = 54;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
                    id = 55;
                    break;
                case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
                    id = 56;
                    break;
                case MAV_CMD.MAV_CMD_DO_GRIPPER:
                    id = 57;
                    break;
                case MAV_CMD.MAV_CMD_DO_AUTOTUNE_ENABLE:
                    id = 58;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                    id = 59;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                    id = 60;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                    id = 61;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                    id = 62;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                    id = 63;
                    break;
                case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                    id = 64;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAST:
                    id = 65;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                    id = 66;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                    id = 67;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                    id = 68;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                    id = 69;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                    id = 70;
                    break;
                case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                    id = 71;
                    break;
                case MAV_CMD.MAV_CMD_MISSION_START:
                    id = 72;
                    break;
                case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                    id = 73;
                    break;
                case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                    id = 74;
                    break;
                case MAV_CMD.MAV_CMD_START_RX_PAIR:
                    id = 75;
                    break;
                case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                    id = 76;
                    break;
                case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                    id = 77;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                    id = 78;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                    id = 79;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                    id = 80;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                    id = 81;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                    id = 82;
                    break;
                case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                    id = 83;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                    id = 84;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                    id = 85;
                    break;
                case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                    id = 86;
                    break;
                case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                    id = 87;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                    id = 88;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                    id = 89;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                    id = 90;
                    break;
                case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                    id = 91;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                    id = 92;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                    id = 93;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                    id = 94;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                    id = 95;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                    id = 96;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_START:
                    id = 97;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_STOP:
                    id = 98;
                    break;
                case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                    id = 99;
                    break;
                case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                    id = 100;
                    break;
                case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                    id = 101;
                    break;
                case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                    id = 102;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                    id = 103;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                    id = 104;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_GATE:
                    id = 105;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                    id = 106;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                    id = 107;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                    id = 108;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                    id = 109;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                    id = 110;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                    id = 111;
                    break;
                case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                    id = 112;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                    id = 113;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                    id = 114;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                    id = 115;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                    id = 116;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                    id = 117;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                    id = 118;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                    id = 119;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                    id = 120;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                    id = 121;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                    id = 122;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                    id = 123;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                    id = 124;
                    break;
                case MAV_CMD.MAV_CMD_USER_1:
                    id = 125;
                    break;
                case MAV_CMD.MAV_CMD_USER_2:
                    id = 126;
                    break;
                case MAV_CMD.MAV_CMD_USER_3:
                    id = 127;
                    break;
                case MAV_CMD.MAV_CMD_USER_4:
                    id = 128;
                    break;
                case MAV_CMD.MAV_CMD_USER_5:
                    id = 129;
                    break;
                case MAV_CMD.MAV_CMD_POWER_OFF_INITIATED:
                    id = 130;
                    break;
                case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_CLICK:
                    id = 131;
                    break;
                case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_HOLD:
                    id = 132;
                    break;
                case MAV_CMD.MAV_CMD_SOLO_BTN_PAUSE_CLICK:
                    id = 133;
                    break;
                case MAV_CMD.MAV_CMD_DO_START_MAG_CAL:
                    id = 134;
                    break;
                case MAV_CMD.MAV_CMD_DO_ACCEPT_MAG_CAL:
                    id = 135;
                    break;
                case MAV_CMD.MAV_CMD_DO_CANCEL_MAG_CAL:
                    id = 136;
                    break;
                case MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE:
                    id = 137;
                    break;
                case MAV_CMD.MAV_CMD_DO_SEND_BANNER:
                    id = 138;
                    break;
                case MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS:
                    id = 139;
                    break;
                case MAV_CMD.MAV_CMD_GIMBAL_RESET:
                    id = 140;
                    break;
                case MAV_CMD.MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS:
                    id = 141;
                    break;
                case MAV_CMD.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION:
                    id = 142;
                    break;
                case MAV_CMD.MAV_CMD_GIMBAL_FULL_RESET:
                    id = 143;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 8, data, 276);
        }
        public void mission_type_SET(@MAV_MISSION_TYPE int  src) //Mission type, see MAV_MISSION_TYPE
        {
            long id = 0;
            switch(src)
            {
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION:
                    id = 0;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE:
                    id = 1;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY:
                    id = 2;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                    id = 3;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 284);
        }
    }
    public static class VFR_HUD extends GroundControl.VFR_HUD
    {
        public void throttle_SET(char  src) //Current throttle setting in integer percent, 0 to 100
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void airspeed_SET(float  src) //Current airspeed in m/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 2); }
        public void groundspeed_SET(float  src) //Current ground speed in m/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }
        public void heading_SET(short  src) //Current heading in degrees, in compass units (0..360, 0=north)
        {  set_bytes((short)(src) & -1L, 2, data,  10); }
        public void alt_SET(float  src) //Current altitude (MSL), in meters
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void climb_SET(float  src) //Current climb rate in meters/second
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
    }
    public static class COMMAND_INT extends GroundControl.COMMAND_INT
    {
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void current_SET(char  src) //false:0, true:1
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void autocontinue_SET(char  src) //autocontinue to next wp
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public void param1_SET(float  src) //PARAM1, see MAV_CMD enum
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void param2_SET(float  src) //PARAM2, see MAV_CMD enum
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void param3_SET(float  src) //PARAM3, see MAV_CMD enum
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void param4_SET(float  src) //PARAM4, see MAV_CMD enum
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public void x_SET(int  src) //PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
        {  set_bytes((int)(src) & -1L, 4, data,  20); }
        public void y_SET(int  src) //PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
        {  set_bytes((int)(src) & -1L, 4, data,  24); }
        public void z_SET(float  src) //PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public void frame_SET(@MAV_FRAME int  src) //The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h
        {  set_bits(- 0 +   src, 4, data, 256); }
        public void command_SET(@MAV_CMD int  src) //The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs
        {
            long id = 0;
            switch(src)
            {
                case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                    id = 0;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
                    id = 1;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
                    id = 2;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
                    id = 3;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                    id = 4;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND:
                    id = 5;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                    id = 6;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
                    id = 7;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
                    id = 8;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FOLLOW:
                    id = 9;
                    break;
                case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
                    id = 10;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
                    id = 11;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW:
                    id = 12;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
                    id = 13;
                    break;
                case MAV_CMD.MAV_CMD_NAV_ROI:
                    id = 14;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
                    id = 15;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
                    id = 16;
                    break;
                case MAV_CMD.MAV_CMD_NAV_ALTITUDE_WAIT:
                    id = 17;
                    break;
                case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
                    id = 18;
                    break;
                case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
                    id = 19;
                    break;
                case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
                    id = 20;
                    break;
                case MAV_CMD.MAV_CMD_NAV_DELAY:
                    id = 21;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
                    id = 22;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAST:
                    id = 23;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DELAY:
                    id = 24;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
                    id = 25;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
                    id = 26;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_YAW:
                    id = 27;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_LAST:
                    id = 28;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_MODE:
                    id = 29;
                    break;
                case MAV_CMD.MAV_CMD_DO_JUMP:
                    id = 30;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
                    id = 31;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_HOME:
                    id = 32;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
                    id = 33;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_RELAY:
                    id = 34;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
                    id = 35;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_SERVO:
                    id = 36;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
                    id = 37;
                    break;
                case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
                    id = 38;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
                    id = 39;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAND_START:
                    id = 40;
                    break;
                case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
                    id = 41;
                    break;
                case MAV_CMD.MAV_CMD_DO_GO_AROUND:
                    id = 42;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPOSITION:
                    id = 43;
                    break;
                case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
                    id = 44;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
                    id = 45;
                    break;
                case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
                    id = 46;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_ROI:
                    id = 47;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
                    id = 48;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
                    id = 49;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
                    id = 50;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                    id = 51;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                    id = 52;
                    break;
                case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
                    id = 53;
                    break;
                case MAV_CMD.MAV_CMD_DO_PARACHUTE:
                    id = 54;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
                    id = 55;
                    break;
                case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
                    id = 56;
                    break;
                case MAV_CMD.MAV_CMD_DO_GRIPPER:
                    id = 57;
                    break;
                case MAV_CMD.MAV_CMD_DO_AUTOTUNE_ENABLE:
                    id = 58;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                    id = 59;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                    id = 60;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                    id = 61;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                    id = 62;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                    id = 63;
                    break;
                case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                    id = 64;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAST:
                    id = 65;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                    id = 66;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                    id = 67;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                    id = 68;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                    id = 69;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                    id = 70;
                    break;
                case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                    id = 71;
                    break;
                case MAV_CMD.MAV_CMD_MISSION_START:
                    id = 72;
                    break;
                case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                    id = 73;
                    break;
                case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                    id = 74;
                    break;
                case MAV_CMD.MAV_CMD_START_RX_PAIR:
                    id = 75;
                    break;
                case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                    id = 76;
                    break;
                case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                    id = 77;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                    id = 78;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                    id = 79;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                    id = 80;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                    id = 81;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                    id = 82;
                    break;
                case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                    id = 83;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                    id = 84;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                    id = 85;
                    break;
                case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                    id = 86;
                    break;
                case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                    id = 87;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                    id = 88;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                    id = 89;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                    id = 90;
                    break;
                case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                    id = 91;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                    id = 92;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                    id = 93;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                    id = 94;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                    id = 95;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                    id = 96;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_START:
                    id = 97;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_STOP:
                    id = 98;
                    break;
                case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                    id = 99;
                    break;
                case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                    id = 100;
                    break;
                case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                    id = 101;
                    break;
                case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                    id = 102;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                    id = 103;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                    id = 104;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_GATE:
                    id = 105;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                    id = 106;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                    id = 107;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                    id = 108;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                    id = 109;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                    id = 110;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                    id = 111;
                    break;
                case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                    id = 112;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                    id = 113;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                    id = 114;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                    id = 115;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                    id = 116;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                    id = 117;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                    id = 118;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                    id = 119;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                    id = 120;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                    id = 121;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                    id = 122;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                    id = 123;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                    id = 124;
                    break;
                case MAV_CMD.MAV_CMD_USER_1:
                    id = 125;
                    break;
                case MAV_CMD.MAV_CMD_USER_2:
                    id = 126;
                    break;
                case MAV_CMD.MAV_CMD_USER_3:
                    id = 127;
                    break;
                case MAV_CMD.MAV_CMD_USER_4:
                    id = 128;
                    break;
                case MAV_CMD.MAV_CMD_USER_5:
                    id = 129;
                    break;
                case MAV_CMD.MAV_CMD_POWER_OFF_INITIATED:
                    id = 130;
                    break;
                case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_CLICK:
                    id = 131;
                    break;
                case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_HOLD:
                    id = 132;
                    break;
                case MAV_CMD.MAV_CMD_SOLO_BTN_PAUSE_CLICK:
                    id = 133;
                    break;
                case MAV_CMD.MAV_CMD_DO_START_MAG_CAL:
                    id = 134;
                    break;
                case MAV_CMD.MAV_CMD_DO_ACCEPT_MAG_CAL:
                    id = 135;
                    break;
                case MAV_CMD.MAV_CMD_DO_CANCEL_MAG_CAL:
                    id = 136;
                    break;
                case MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE:
                    id = 137;
                    break;
                case MAV_CMD.MAV_CMD_DO_SEND_BANNER:
                    id = 138;
                    break;
                case MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS:
                    id = 139;
                    break;
                case MAV_CMD.MAV_CMD_GIMBAL_RESET:
                    id = 140;
                    break;
                case MAV_CMD.MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS:
                    id = 141;
                    break;
                case MAV_CMD.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION:
                    id = 142;
                    break;
                case MAV_CMD.MAV_CMD_GIMBAL_FULL_RESET:
                    id = 143;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 8, data, 260);
        }
    }
    public static class COMMAND_LONG extends GroundControl.COMMAND_LONG
    {
        public void target_system_SET(char  src) //System which should execute the command
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component which should execute the command, 0 for all components
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void confirmation_SET(char  src) //0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void param1_SET(float  src) //Parameter 1, as defined by MAV_CMD enum.
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 3); }
        public void param2_SET(float  src) //Parameter 2, as defined by MAV_CMD enum.
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 7); }
        public void param3_SET(float  src) //Parameter 3, as defined by MAV_CMD enum.
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 11); }
        public void param4_SET(float  src) //Parameter 4, as defined by MAV_CMD enum.
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 15); }
        public void param5_SET(float  src) //Parameter 5, as defined by MAV_CMD enum.
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 19); }
        public void param6_SET(float  src) //Parameter 6, as defined by MAV_CMD enum.
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 23); }
        public void param7_SET(float  src) //Parameter 7, as defined by MAV_CMD enum.
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 27); }
        public void command_SET(@MAV_CMD int  src) //Command ID, as defined by MAV_CMD enum.
        {
            long id = 0;
            switch(src)
            {
                case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                    id = 0;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
                    id = 1;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
                    id = 2;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
                    id = 3;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                    id = 4;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND:
                    id = 5;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                    id = 6;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
                    id = 7;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
                    id = 8;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FOLLOW:
                    id = 9;
                    break;
                case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
                    id = 10;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
                    id = 11;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW:
                    id = 12;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
                    id = 13;
                    break;
                case MAV_CMD.MAV_CMD_NAV_ROI:
                    id = 14;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
                    id = 15;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
                    id = 16;
                    break;
                case MAV_CMD.MAV_CMD_NAV_ALTITUDE_WAIT:
                    id = 17;
                    break;
                case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
                    id = 18;
                    break;
                case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
                    id = 19;
                    break;
                case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
                    id = 20;
                    break;
                case MAV_CMD.MAV_CMD_NAV_DELAY:
                    id = 21;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
                    id = 22;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAST:
                    id = 23;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DELAY:
                    id = 24;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
                    id = 25;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
                    id = 26;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_YAW:
                    id = 27;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_LAST:
                    id = 28;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_MODE:
                    id = 29;
                    break;
                case MAV_CMD.MAV_CMD_DO_JUMP:
                    id = 30;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
                    id = 31;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_HOME:
                    id = 32;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
                    id = 33;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_RELAY:
                    id = 34;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
                    id = 35;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_SERVO:
                    id = 36;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
                    id = 37;
                    break;
                case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
                    id = 38;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
                    id = 39;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAND_START:
                    id = 40;
                    break;
                case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
                    id = 41;
                    break;
                case MAV_CMD.MAV_CMD_DO_GO_AROUND:
                    id = 42;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPOSITION:
                    id = 43;
                    break;
                case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
                    id = 44;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
                    id = 45;
                    break;
                case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
                    id = 46;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_ROI:
                    id = 47;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
                    id = 48;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
                    id = 49;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
                    id = 50;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                    id = 51;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                    id = 52;
                    break;
                case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
                    id = 53;
                    break;
                case MAV_CMD.MAV_CMD_DO_PARACHUTE:
                    id = 54;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
                    id = 55;
                    break;
                case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
                    id = 56;
                    break;
                case MAV_CMD.MAV_CMD_DO_GRIPPER:
                    id = 57;
                    break;
                case MAV_CMD.MAV_CMD_DO_AUTOTUNE_ENABLE:
                    id = 58;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                    id = 59;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                    id = 60;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                    id = 61;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                    id = 62;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                    id = 63;
                    break;
                case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                    id = 64;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAST:
                    id = 65;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                    id = 66;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                    id = 67;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                    id = 68;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                    id = 69;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                    id = 70;
                    break;
                case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                    id = 71;
                    break;
                case MAV_CMD.MAV_CMD_MISSION_START:
                    id = 72;
                    break;
                case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                    id = 73;
                    break;
                case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                    id = 74;
                    break;
                case MAV_CMD.MAV_CMD_START_RX_PAIR:
                    id = 75;
                    break;
                case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                    id = 76;
                    break;
                case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                    id = 77;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                    id = 78;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                    id = 79;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                    id = 80;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                    id = 81;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                    id = 82;
                    break;
                case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                    id = 83;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                    id = 84;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                    id = 85;
                    break;
                case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                    id = 86;
                    break;
                case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                    id = 87;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                    id = 88;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                    id = 89;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                    id = 90;
                    break;
                case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                    id = 91;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                    id = 92;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                    id = 93;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                    id = 94;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                    id = 95;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                    id = 96;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_START:
                    id = 97;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_STOP:
                    id = 98;
                    break;
                case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                    id = 99;
                    break;
                case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                    id = 100;
                    break;
                case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                    id = 101;
                    break;
                case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                    id = 102;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                    id = 103;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                    id = 104;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_GATE:
                    id = 105;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                    id = 106;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                    id = 107;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                    id = 108;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                    id = 109;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                    id = 110;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                    id = 111;
                    break;
                case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                    id = 112;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                    id = 113;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                    id = 114;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                    id = 115;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                    id = 116;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                    id = 117;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                    id = 118;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                    id = 119;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                    id = 120;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                    id = 121;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                    id = 122;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                    id = 123;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                    id = 124;
                    break;
                case MAV_CMD.MAV_CMD_USER_1:
                    id = 125;
                    break;
                case MAV_CMD.MAV_CMD_USER_2:
                    id = 126;
                    break;
                case MAV_CMD.MAV_CMD_USER_3:
                    id = 127;
                    break;
                case MAV_CMD.MAV_CMD_USER_4:
                    id = 128;
                    break;
                case MAV_CMD.MAV_CMD_USER_5:
                    id = 129;
                    break;
                case MAV_CMD.MAV_CMD_POWER_OFF_INITIATED:
                    id = 130;
                    break;
                case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_CLICK:
                    id = 131;
                    break;
                case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_HOLD:
                    id = 132;
                    break;
                case MAV_CMD.MAV_CMD_SOLO_BTN_PAUSE_CLICK:
                    id = 133;
                    break;
                case MAV_CMD.MAV_CMD_DO_START_MAG_CAL:
                    id = 134;
                    break;
                case MAV_CMD.MAV_CMD_DO_ACCEPT_MAG_CAL:
                    id = 135;
                    break;
                case MAV_CMD.MAV_CMD_DO_CANCEL_MAG_CAL:
                    id = 136;
                    break;
                case MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE:
                    id = 137;
                    break;
                case MAV_CMD.MAV_CMD_DO_SEND_BANNER:
                    id = 138;
                    break;
                case MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS:
                    id = 139;
                    break;
                case MAV_CMD.MAV_CMD_GIMBAL_RESET:
                    id = 140;
                    break;
                case MAV_CMD.MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS:
                    id = 141;
                    break;
                case MAV_CMD.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION:
                    id = 142;
                    break;
                case MAV_CMD.MAV_CMD_GIMBAL_FULL_RESET:
                    id = 143;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 8, data, 248);
        }
    }
    public static class COMMAND_ACK extends GroundControl.COMMAND_ACK
    {
        public void command_SET(@MAV_CMD int  src) //Command ID, as defined by MAV_CMD enum.
        {
            long id = 0;
            switch(src)
            {
                case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                    id = 0;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
                    id = 1;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
                    id = 2;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
                    id = 3;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                    id = 4;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND:
                    id = 5;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                    id = 6;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
                    id = 7;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
                    id = 8;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FOLLOW:
                    id = 9;
                    break;
                case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
                    id = 10;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
                    id = 11;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW:
                    id = 12;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
                    id = 13;
                    break;
                case MAV_CMD.MAV_CMD_NAV_ROI:
                    id = 14;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
                    id = 15;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
                    id = 16;
                    break;
                case MAV_CMD.MAV_CMD_NAV_ALTITUDE_WAIT:
                    id = 17;
                    break;
                case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
                    id = 18;
                    break;
                case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
                    id = 19;
                    break;
                case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
                    id = 20;
                    break;
                case MAV_CMD.MAV_CMD_NAV_DELAY:
                    id = 21;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
                    id = 22;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAST:
                    id = 23;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DELAY:
                    id = 24;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
                    id = 25;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
                    id = 26;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_YAW:
                    id = 27;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_LAST:
                    id = 28;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_MODE:
                    id = 29;
                    break;
                case MAV_CMD.MAV_CMD_DO_JUMP:
                    id = 30;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
                    id = 31;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_HOME:
                    id = 32;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
                    id = 33;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_RELAY:
                    id = 34;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
                    id = 35;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_SERVO:
                    id = 36;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
                    id = 37;
                    break;
                case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
                    id = 38;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
                    id = 39;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAND_START:
                    id = 40;
                    break;
                case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
                    id = 41;
                    break;
                case MAV_CMD.MAV_CMD_DO_GO_AROUND:
                    id = 42;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPOSITION:
                    id = 43;
                    break;
                case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
                    id = 44;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
                    id = 45;
                    break;
                case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
                    id = 46;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_ROI:
                    id = 47;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
                    id = 48;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
                    id = 49;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
                    id = 50;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                    id = 51;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                    id = 52;
                    break;
                case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
                    id = 53;
                    break;
                case MAV_CMD.MAV_CMD_DO_PARACHUTE:
                    id = 54;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
                    id = 55;
                    break;
                case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
                    id = 56;
                    break;
                case MAV_CMD.MAV_CMD_DO_GRIPPER:
                    id = 57;
                    break;
                case MAV_CMD.MAV_CMD_DO_AUTOTUNE_ENABLE:
                    id = 58;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                    id = 59;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                    id = 60;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                    id = 61;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                    id = 62;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                    id = 63;
                    break;
                case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                    id = 64;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAST:
                    id = 65;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                    id = 66;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                    id = 67;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                    id = 68;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                    id = 69;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                    id = 70;
                    break;
                case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                    id = 71;
                    break;
                case MAV_CMD.MAV_CMD_MISSION_START:
                    id = 72;
                    break;
                case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                    id = 73;
                    break;
                case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                    id = 74;
                    break;
                case MAV_CMD.MAV_CMD_START_RX_PAIR:
                    id = 75;
                    break;
                case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                    id = 76;
                    break;
                case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                    id = 77;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                    id = 78;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                    id = 79;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                    id = 80;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                    id = 81;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                    id = 82;
                    break;
                case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                    id = 83;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                    id = 84;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                    id = 85;
                    break;
                case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                    id = 86;
                    break;
                case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                    id = 87;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                    id = 88;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                    id = 89;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                    id = 90;
                    break;
                case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                    id = 91;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                    id = 92;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                    id = 93;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                    id = 94;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                    id = 95;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                    id = 96;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_START:
                    id = 97;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_STOP:
                    id = 98;
                    break;
                case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                    id = 99;
                    break;
                case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                    id = 100;
                    break;
                case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                    id = 101;
                    break;
                case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                    id = 102;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                    id = 103;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                    id = 104;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_GATE:
                    id = 105;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                    id = 106;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                    id = 107;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                    id = 108;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                    id = 109;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                    id = 110;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                    id = 111;
                    break;
                case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                    id = 112;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                    id = 113;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                    id = 114;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                    id = 115;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                    id = 116;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                    id = 117;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                    id = 118;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                    id = 119;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                    id = 120;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                    id = 121;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                    id = 122;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                    id = 123;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                    id = 124;
                    break;
                case MAV_CMD.MAV_CMD_USER_1:
                    id = 125;
                    break;
                case MAV_CMD.MAV_CMD_USER_2:
                    id = 126;
                    break;
                case MAV_CMD.MAV_CMD_USER_3:
                    id = 127;
                    break;
                case MAV_CMD.MAV_CMD_USER_4:
                    id = 128;
                    break;
                case MAV_CMD.MAV_CMD_USER_5:
                    id = 129;
                    break;
                case MAV_CMD.MAV_CMD_POWER_OFF_INITIATED:
                    id = 130;
                    break;
                case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_CLICK:
                    id = 131;
                    break;
                case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_HOLD:
                    id = 132;
                    break;
                case MAV_CMD.MAV_CMD_SOLO_BTN_PAUSE_CLICK:
                    id = 133;
                    break;
                case MAV_CMD.MAV_CMD_DO_START_MAG_CAL:
                    id = 134;
                    break;
                case MAV_CMD.MAV_CMD_DO_ACCEPT_MAG_CAL:
                    id = 135;
                    break;
                case MAV_CMD.MAV_CMD_DO_CANCEL_MAG_CAL:
                    id = 136;
                    break;
                case MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE:
                    id = 137;
                    break;
                case MAV_CMD.MAV_CMD_DO_SEND_BANNER:
                    id = 138;
                    break;
                case MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS:
                    id = 139;
                    break;
                case MAV_CMD.MAV_CMD_GIMBAL_RESET:
                    id = 140;
                    break;
                case MAV_CMD.MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS:
                    id = 141;
                    break;
                case MAV_CMD.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION:
                    id = 142;
                    break;
                case MAV_CMD.MAV_CMD_GIMBAL_FULL_RESET:
                    id = 143;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 8, data, 0);
        }
        public void result_SET(@MAV_RESULT int  src) //See MAV_RESULT enum
        {  set_bits(- 0 +   src, 3, data, 8); }
        /**
        *WIP: Also used as result_param1, it can be set with a enum containing the errors reasons of why the command
        *	was denied or the progress percentage or 255 if unknown the progress when result is MAV_RESULT_IN_PROGRESS*/
        public void progress_SET(char  src, Bounds.Inside ph)
        {
            if(ph.field_bit != 11)insert_field(ph, 11, 0);
            set_bytes((char)(src) & -1L, 1, data,  ph.BYTE);
        }/**
*WIP: Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to
*	be denied*/
        public void result_param2_SET(int  src, Bounds.Inside ph)
        {
            if(ph.field_bit != 12)insert_field(ph, 12, 0);
            set_bytes((int)(src) & -1L, 4, data,  ph.BYTE);
        } public void target_system_SET(char  src, Bounds.Inside ph) //WIP: System which requested the command to be executed
        {
            if(ph.field_bit != 13)insert_field(ph, 13, 0);
            set_bytes((char)(src) & -1L, 1, data,  ph.BYTE);
        } public void target_component_SET(char  src, Bounds.Inside ph) //WIP: Component which requested the command to be executed
        {
            if(ph.field_bit != 14)insert_field(ph, 14, 0);
            set_bytes((char)(src) & -1L, 1, data,  ph.BYTE);
        }
    }
    public static class MANUAL_SETPOINT extends GroundControl.MANUAL_SETPOINT
    {
        public void time_boot_ms_SET(long  src) //Timestamp in milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public void roll_SET(float  src) //Desired roll rate in radians per second
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void pitch_SET(float  src) //Desired pitch rate in radians per second
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void yaw_SET(float  src) //Desired yaw rate in radians per second
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void thrust_SET(float  src) //Collective thrust, normalized to 0 .. 1
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public void mode_switch_SET(char  src) //Flight mode switch position, 0.. 255
        {  set_bytes((char)(src) & -1L, 1, data,  20); }
        public void manual_override_switch_SET(char  src) //Override mode switch position, 0.. 255
        {  set_bytes((char)(src) & -1L, 1, data,  21); }
    }
    public static class SET_ATTITUDE_TARGET extends GroundControl.SET_ATTITUDE_TARGET
    {
        public void time_boot_ms_SET(long  src) //Timestamp in milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        /**
        *Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
        *	bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitud*/
        public void type_mask_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  6); }
        public void q_SET(float[]  src, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        {
            for(int BYTE =  7, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public void body_roll_rate_SET(float  src) //Body roll rate in radians per second
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 23); }
        public void body_pitch_rate_SET(float  src) //Body roll rate in radians per second
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 27); }
        public void body_yaw_rate_SET(float  src) //Body roll rate in radians per second
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 31); }
        public void thrust_SET(float  src) //Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 35); }
    }
    public static class ATTITUDE_TARGET extends GroundControl.ATTITUDE_TARGET
    {
        public void time_boot_ms_SET(long  src) //Timestamp in milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        /**
        *Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
        *	bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitud*/
        public void type_mask_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public void q_SET(float[]  src, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        {
            for(int BYTE =  5, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public void body_roll_rate_SET(float  src) //Body roll rate in radians per second
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 21); }
        public void body_pitch_rate_SET(float  src) //Body pitch rate in radians per second
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 25); }
        public void body_yaw_rate_SET(float  src) //Body yaw rate in radians per second
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 29); }
        public void thrust_SET(float  src) //Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 33); }
    }
    public static class SET_POSITION_TARGET_LOCAL_NED extends GroundControl.SET_POSITION_TARGET_LOCAL_NED
    {
        /**
        *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
        *	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
        *	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
        *	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
        *	bit 11: yaw, bit 12: yaw rat*/
        public void type_mask_SET(char  src)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void time_boot_ms_SET(long  src) //Timestamp in milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  2); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  6); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  7); }
        public void x_SET(float  src) //X Position in NED frame in meters
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void y_SET(float  src) //Y Position in NED frame in meters
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void z_SET(float  src) //Z Position in NED frame in meters (note, altitude is negative in NED)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public void vx_SET(float  src) //X velocity in NED frame in meter / s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public void vy_SET(float  src) //Y velocity in NED frame in meter / s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public void vz_SET(float  src) //Z velocity in NED frame in meter / s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public void afx_SET(float  src) //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public void afy_SET(float  src) //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public void afz_SET(float  src) //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        public void yaw_SET(float  src) //yaw setpoint in rad
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 44); }
        public void yaw_rate_SET(float  src) //yaw rate setpoint in rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 48); }
        /**
        *Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
        *	=*/
        public void coordinate_frame_SET(@MAV_FRAME int  src)
        {  set_bits(- 0 +   src, 4, data, 416); }
    }
    public static class SET_POSITION_TARGET_GLOBAL_INT extends GroundControl.SET_POSITION_TARGET_GLOBAL_INT
    {
        /**
        *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
        *	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
        *	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
        *	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
        *	bit 11: yaw, bit 12: yaw rat*/
        public void type_mask_SET(char  src)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        /**
        *Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
        *	the system to compensate for the transport delay of the setpoint. This allows the system to compensate
        *	processing latency*/
        public void time_boot_ms_SET(long  src)
        {  set_bytes((src) & -1L, 4, data,  2); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  6); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  7); }
        public void lat_int_SET(int  src) //X Position in WGS84 frame in 1e7 * meters
        {  set_bytes((int)(src) & -1L, 4, data,  8); }
        public void lon_int_SET(int  src) //Y Position in WGS84 frame in 1e7 * meters
        {  set_bytes((int)(src) & -1L, 4, data,  12); }
        public void alt_SET(float  src) //Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public void vx_SET(float  src) //X velocity in NED frame in meter / s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public void vy_SET(float  src) //Y velocity in NED frame in meter / s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public void vz_SET(float  src) //Z velocity in NED frame in meter / s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public void afx_SET(float  src) //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public void afy_SET(float  src) //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public void afz_SET(float  src) //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        public void yaw_SET(float  src) //yaw setpoint in rad
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 44); }
        public void yaw_rate_SET(float  src) //yaw rate setpoint in rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 48); }
        /**
        *Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
        *	= 1*/
        public void coordinate_frame_SET(@MAV_FRAME int  src)
        {  set_bits(- 0 +   src, 4, data, 416); }
    }
    public static class POSITION_TARGET_GLOBAL_INT extends GroundControl.POSITION_TARGET_GLOBAL_INT
    {
        /**
        *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
        *	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
        *	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
        *	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
        *	bit 11: yaw, bit 12: yaw rat*/
        public void type_mask_SET(char  src)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        /**
        *Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
        *	the system to compensate for the transport delay of the setpoint. This allows the system to compensate
        *	processing latency*/
        public void time_boot_ms_SET(long  src)
        {  set_bytes((src) & -1L, 4, data,  2); }
        public void lat_int_SET(int  src) //X Position in WGS84 frame in 1e7 * meters
        {  set_bytes((int)(src) & -1L, 4, data,  6); }
        public void lon_int_SET(int  src) //Y Position in WGS84 frame in 1e7 * meters
        {  set_bytes((int)(src) & -1L, 4, data,  10); }
        public void alt_SET(float  src) //Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }
        public void vx_SET(float  src) //X velocity in NED frame in meter / s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }
        public void vy_SET(float  src) //Y velocity in NED frame in meter / s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }
        public void vz_SET(float  src) //Z velocity in NED frame in meter / s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 26); }
        public void afx_SET(float  src) //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 30); }
        public void afy_SET(float  src) //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 34); }
        public void afz_SET(float  src) //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 38); }
        public void yaw_SET(float  src) //yaw setpoint in rad
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 42); }
        public void yaw_rate_SET(float  src) //yaw rate setpoint in rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 46); }
        /**
        *Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
        *	= 1*/
        public void coordinate_frame_SET(@MAV_FRAME int  src)
        {  set_bits(- 0 +   src, 4, data, 400); }
    }
    public static class LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET extends GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET
    {
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public void x_SET(float  src) //X Position
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void y_SET(float  src) //Y Position
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void z_SET(float  src) //Z Position
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void roll_SET(float  src) //Roll
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public void pitch_SET(float  src) //Pitch
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public void yaw_SET(float  src) //Yaw
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
    }
    public static class HIL_STATE extends GroundControl.HIL_STATE
    {
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  set_bytes((src) & -1L, 8, data,  0); }
        public void roll_SET(float  src) //Roll angle (rad)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void pitch_SET(float  src) //Pitch angle (rad)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void yaw_SET(float  src) //Yaw angle (rad)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public void rollspeed_SET(float  src) //Body frame roll / phi angular speed (rad/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public void pitchspeed_SET(float  src) //Body frame pitch / theta angular speed (rad/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public void yawspeed_SET(float  src) //Body frame yaw / psi angular speed (rad/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public void lat_SET(int  src) //Latitude, expressed as * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  32); }
        public void lon_SET(int  src) //Longitude, expressed as * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  36); }
        public void alt_SET(int  src) //Altitude in meters, expressed as * 1000 (millimeters)
        {  set_bytes((int)(src) & -1L, 4, data,  40); }
        public void vx_SET(short  src) //Ground X Speed (Latitude), expressed as m/s * 100
        {  set_bytes((short)(src) & -1L, 2, data,  44); }
        public void vy_SET(short  src) //Ground Y Speed (Longitude), expressed as m/s * 100
        {  set_bytes((short)(src) & -1L, 2, data,  46); }
        public void vz_SET(short  src) //Ground Z Speed (Altitude), expressed as m/s * 100
        {  set_bytes((short)(src) & -1L, 2, data,  48); }
        public void xacc_SET(short  src) //X acceleration (mg)
        {  set_bytes((short)(src) & -1L, 2, data,  50); }
        public void yacc_SET(short  src) //Y acceleration (mg)
        {  set_bytes((short)(src) & -1L, 2, data,  52); }
        public void zacc_SET(short  src) //Z acceleration (mg)
        {  set_bytes((short)(src) & -1L, 2, data,  54); }
    }
    public static class HIL_CONTROLS extends GroundControl.HIL_CONTROLS
    {
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  set_bytes((src) & -1L, 8, data,  0); }
        public void roll_ailerons_SET(float  src) //Control output -1 .. 1
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void pitch_elevator_SET(float  src) //Control output -1 .. 1
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void yaw_rudder_SET(float  src) //Control output -1 .. 1
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public void throttle_SET(float  src) //Throttle 0 .. 1
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public void aux1_SET(float  src) //Aux 1, -1 .. 1
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public void aux2_SET(float  src) //Aux 2, -1 .. 1
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public void aux3_SET(float  src) //Aux 3, -1 .. 1
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public void aux4_SET(float  src) //Aux 4, -1 .. 1
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public void nav_mode_SET(char  src) //Navigation mode (MAV_NAV_MODE)
        {  set_bytes((char)(src) & -1L, 1, data,  40); }
        public void mode_SET(@MAV_MODE int  src) //System mode (MAV_MODE)
        {
            long id = 0;
            switch(src)
            {
                case MAV_MODE.MAV_MODE_PREFLIGHT:
                    id = 0;
                    break;
                case MAV_MODE.MAV_MODE_MANUAL_DISARMED:
                    id = 1;
                    break;
                case MAV_MODE.MAV_MODE_TEST_DISARMED:
                    id = 2;
                    break;
                case MAV_MODE.MAV_MODE_STABILIZE_DISARMED:
                    id = 3;
                    break;
                case MAV_MODE.MAV_MODE_GUIDED_DISARMED:
                    id = 4;
                    break;
                case MAV_MODE.MAV_MODE_AUTO_DISARMED:
                    id = 5;
                    break;
                case MAV_MODE.MAV_MODE_MANUAL_ARMED:
                    id = 6;
                    break;
                case MAV_MODE.MAV_MODE_TEST_ARMED:
                    id = 7;
                    break;
                case MAV_MODE.MAV_MODE_STABILIZE_ARMED:
                    id = 8;
                    break;
                case MAV_MODE.MAV_MODE_GUIDED_ARMED:
                    id = 9;
                    break;
                case MAV_MODE.MAV_MODE_AUTO_ARMED:
                    id = 10;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 4, data, 328);
        }
    }
    public static class HIL_RC_INPUTS_RAW extends GroundControl.HIL_RC_INPUTS_RAW
    {
        public void chan1_raw_SET(char  src) //RC channel 1 value, in microseconds
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void chan2_raw_SET(char  src) //RC channel 2 value, in microseconds
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public void chan3_raw_SET(char  src) //RC channel 3 value, in microseconds
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public void chan4_raw_SET(char  src) //RC channel 4 value, in microseconds
        {  set_bytes((char)(src) & -1L, 2, data,  6); }
        public void chan5_raw_SET(char  src) //RC channel 5 value, in microseconds
        {  set_bytes((char)(src) & -1L, 2, data,  8); }
        public void chan6_raw_SET(char  src) //RC channel 6 value, in microseconds
        {  set_bytes((char)(src) & -1L, 2, data,  10); }
        public void chan7_raw_SET(char  src) //RC channel 7 value, in microseconds
        {  set_bytes((char)(src) & -1L, 2, data,  12); }
        public void chan8_raw_SET(char  src) //RC channel 8 value, in microseconds
        {  set_bytes((char)(src) & -1L, 2, data,  14); }
        public void chan9_raw_SET(char  src) //RC channel 9 value, in microseconds
        {  set_bytes((char)(src) & -1L, 2, data,  16); }
        public void chan10_raw_SET(char  src) //RC channel 10 value, in microseconds
        {  set_bytes((char)(src) & -1L, 2, data,  18); }
        public void chan11_raw_SET(char  src) //RC channel 11 value, in microseconds
        {  set_bytes((char)(src) & -1L, 2, data,  20); }
        public void chan12_raw_SET(char  src) //RC channel 12 value, in microseconds
        {  set_bytes((char)(src) & -1L, 2, data,  22); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  set_bytes((src) & -1L, 8, data,  24); }
        public void rssi_SET(char  src) //Receive signal strength indicator, 0: 0%, 255: 100%
        {  set_bytes((char)(src) & -1L, 1, data,  32); }
    }
    public static class HIL_ACTUATOR_CONTROLS extends GroundControl.HIL_ACTUATOR_CONTROLS
    {
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  set_bytes((src) & -1L, 8, data,  0); }
        public void flags_SET(long  src) //Flags as bitfield, reserved for future use.
        {  set_bytes((src) & -1L, 8, data,  8); }
        public void controls_SET(float[]  src, int pos)  //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
        {
            for(int BYTE =  16, src_max = pos + 16; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public void mode_SET(@MAV_MODE int  src) //System mode (MAV_MODE), includes arming state.
        {
            long id = 0;
            switch(src)
            {
                case MAV_MODE.MAV_MODE_PREFLIGHT:
                    id = 0;
                    break;
                case MAV_MODE.MAV_MODE_MANUAL_DISARMED:
                    id = 1;
                    break;
                case MAV_MODE.MAV_MODE_TEST_DISARMED:
                    id = 2;
                    break;
                case MAV_MODE.MAV_MODE_STABILIZE_DISARMED:
                    id = 3;
                    break;
                case MAV_MODE.MAV_MODE_GUIDED_DISARMED:
                    id = 4;
                    break;
                case MAV_MODE.MAV_MODE_AUTO_DISARMED:
                    id = 5;
                    break;
                case MAV_MODE.MAV_MODE_MANUAL_ARMED:
                    id = 6;
                    break;
                case MAV_MODE.MAV_MODE_TEST_ARMED:
                    id = 7;
                    break;
                case MAV_MODE.MAV_MODE_STABILIZE_ARMED:
                    id = 8;
                    break;
                case MAV_MODE.MAV_MODE_GUIDED_ARMED:
                    id = 9;
                    break;
                case MAV_MODE.MAV_MODE_AUTO_ARMED:
                    id = 10;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 4, data, 640);
        }
    }
    public static class OPTICAL_FLOW extends GroundControl.OPTICAL_FLOW
    {
        public void time_usec_SET(long  src) //Timestamp (UNIX)
        {  set_bytes((src) & -1L, 8, data,  0); }
        public void sensor_id_SET(char  src) //Sensor ID
        {  set_bytes((char)(src) & -1L, 1, data,  8); }
        public void flow_x_SET(short  src) //Flow in pixels * 10 in x-sensor direction (dezi-pixels)
        {  set_bytes((short)(src) & -1L, 2, data,  9); }
        public void flow_y_SET(short  src) //Flow in pixels * 10 in y-sensor direction (dezi-pixels)
        {  set_bytes((short)(src) & -1L, 2, data,  11); }
        public void flow_comp_m_x_SET(float  src) //Flow in meters in x-sensor direction, angular-speed compensated
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 13); }
        public void flow_comp_m_y_SET(float  src) //Flow in meters in y-sensor direction, angular-speed compensated
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 17); }
        public void quality_SET(char  src) //Optical flow quality / confidence. 0: bad, 255: maximum quality
        {  set_bytes((char)(src) & -1L, 1, data,  21); }
        public void ground_distance_SET(float  src) //Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }
        public void flow_rate_x_SET(float  src, Bounds.Inside ph)//Flow rate in radians/second about X axis
        {
            if(ph.field_bit != 208)insert_field(ph, 208, 0);
            set_bytes(Float.floatToIntBits(src) & -1L, 4, data, ph.BYTE);
        } public void flow_rate_y_SET(float  src, Bounds.Inside ph) //Flow rate in radians/second about Y axis
        {
            if(ph.field_bit != 209)insert_field(ph, 209, 0);
            set_bytes(Float.floatToIntBits(src) & -1L, 4, data, ph.BYTE);
        }
    }
    public static class GLOBAL_VISION_POSITION_ESTIMATE extends GroundControl.GLOBAL_VISION_POSITION_ESTIMATE
    {
        public void usec_SET(long  src) //Timestamp (microseconds, synced to UNIX time or since system boot)
        {  set_bytes((src) & -1L, 8, data,  0); }
        public void x_SET(float  src) //Global X position
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void y_SET(float  src) //Global Y position
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void z_SET(float  src) //Global Z position
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public void roll_SET(float  src) //Roll angle in rad
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public void pitch_SET(float  src) //Pitch angle in rad
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public void yaw_SET(float  src) //Yaw angle in rad
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
    }
    public static class SENSOR_OFFSETS extends GroundControl.SENSOR_OFFSETS
    {
        public short mag_ofs_x_GET()//magnetometer X offset
        {  return (short)((short) get_bytes(data,  0, 2)); }
        public short mag_ofs_y_GET()//magnetometer Y offset
        {  return (short)((short) get_bytes(data,  2, 2)); }
        public short mag_ofs_z_GET()//magnetometer Z offset
        {  return (short)((short) get_bytes(data,  4, 2)); }
        public float mag_declination_GET()//magnetic declination (radians)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  6, 4))); }
        public int raw_press_GET()//raw pressure from barometer
        {  return (int)((int) get_bytes(data,  10, 4)); }
        public int raw_temp_GET()//raw temperature from barometer
        {  return (int)((int) get_bytes(data,  14, 4)); }
        public float gyro_cal_x_GET()//gyro X calibration
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public float gyro_cal_y_GET()//gyro Y calibration
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        public float gyro_cal_z_GET()//gyro Z calibration
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  26, 4))); }
        public float accel_cal_x_GET()//accel X calibration
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  30, 4))); }
        public float accel_cal_y_GET()//accel Y calibration
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  34, 4))); }
        public float accel_cal_z_GET()//accel Z calibration
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  38, 4))); }
    }
    public static class SET_MAG_OFFSETS extends GroundControl.SET_MAG_OFFSETS
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public short mag_ofs_x_GET()//magnetometer X offset
        {  return (short)((short) get_bytes(data,  2, 2)); }
        public short mag_ofs_y_GET()//magnetometer Y offset
        {  return (short)((short) get_bytes(data,  4, 2)); }
        public short mag_ofs_z_GET()//magnetometer Z offset
        {  return (short)((short) get_bytes(data,  6, 2)); }
    }
    public static class MEMINFO extends GroundControl.MEMINFO
    {
        public char brkval_GET()//heap top
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char freemem_GET()//free memory
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public long  freemem32_TRY(Bounds.Inside ph)//free memory (32 bit)
        {
            if(ph.field_bit !=  32 && !try_visit_field(ph, 32)) return 0;
            return (get_bytes(data,  ph.BYTE, 4));
        }
    }
    public static class AP_ADC extends GroundControl.AP_ADC
    {
        public char adc1_GET()//ADC output 1
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char adc2_GET()//ADC output 2
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public char adc3_GET()//ADC output 3
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public char adc4_GET()//ADC output 4
        {  return (char)((char) get_bytes(data,  6, 2)); }
        public char adc5_GET()//ADC output 5
        {  return (char)((char) get_bytes(data,  8, 2)); }
        public char adc6_GET()//ADC output 6
        {  return (char)((char) get_bytes(data,  10, 2)); }
    }
    public static class DIGICAM_CONFIGURE extends GroundControl.DIGICAM_CONFIGURE
    {
        public char shutter_speed_GET()//Divisor number e.g. 1000 means 1/1000 (0 means ignore)
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public char mode_GET()//Mode enumeration from 1 to N P, TV, AV, M, Etc (0 means ignore)
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public char aperture_GET()//F stop number x 10 e.g. 28 means 2.8 (0 means ignore)
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public char iso_GET()//ISO enumeration from 1 to N e.g. 80, 100, 200, Etc (0 means ignore)
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public char exposure_type_GET()//Exposure type enumeration from 1 to N (0 means ignore)
        {  return (char)((char) get_bytes(data,  7, 1)); }
        /**
        *Command Identity (incremental loop: 0 to 255)A command sent multiple times will be executed or pooled
        *	just onc*/
        public char command_id_GET()
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public char engine_cut_off_GET()//Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)
        {  return (char)((char) get_bytes(data,  9, 1)); }
        public char extra_param_GET()//Extra parameters enumeration (0 means ignore)
        {  return (char)((char) get_bytes(data,  10, 1)); }
        public float extra_value_GET()//Correspondent value to given extra_param
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  11, 4))); }
    }
    public static class DIGICAM_CONTROL extends GroundControl.DIGICAM_CONTROL
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char session_GET()//0: stop, 1: start or keep it up Session control e.g. show/hide lens
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char zoom_pos_GET()//1 to N Zoom's absolute position (0 means ignore)
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public byte zoom_step_GET()//-100 to 100 Zooming step value to offset zoom from the current position
        {  return (byte)((byte) get_bytes(data,  4, 1)); }
        public char focus_lock_GET()//0: unlock focus or keep unlocked, 1: lock focus or keep locked, 3: re-lock focus
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public char shot_GET()//0: ignore, 1: shot or start filming
        {  return (char)((char) get_bytes(data,  6, 1)); }
        /**
        *Command Identity (incremental loop: 0 to 255)A command sent multiple times will be executed or pooled
        *	just onc*/
        public char command_id_GET()
        {  return (char)((char) get_bytes(data,  7, 1)); }
        public char extra_param_GET()//Extra parameters enumeration (0 means ignore)
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public float extra_value_GET()//Correspondent value to given extra_param
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  9, 4))); }
    }
    public static class MOUNT_CONFIGURE extends GroundControl.MOUNT_CONFIGURE
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char stab_roll_GET()//(1 = yes, 0 = no)
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char stab_pitch_GET()//(1 = yes, 0 = no)
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public char stab_yaw_GET()//(1 = yes, 0 = no)
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public @MAV_MOUNT_MODE int mount_mode_GET()//mount operating mode (see MAV_MOUNT_MODE enum)
        {  return  0 + (int)get_bits(data, 40, 3); }
    }
    public static class MOUNT_CONTROL extends GroundControl.MOUNT_CONTROL
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public int input_a_GET()//pitch(deg*100) or lat, depending on mount mode
        {  return (int)((int) get_bytes(data,  2, 4)); }
        public int input_b_GET()//roll(deg*100) or lon depending on mount mode
        {  return (int)((int) get_bytes(data,  6, 4)); }
        public int input_c_GET()//yaw(deg*100) or alt (in cm) depending on mount mode
        {  return (int)((int) get_bytes(data,  10, 4)); }
        public char save_position_GET()//if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
        {  return (char)((char) get_bytes(data,  14, 1)); }
    }
    public static class MOUNT_STATUS extends GroundControl.MOUNT_STATUS
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public int pointing_a_GET()//pitch(deg*100)
        {  return (int)((int) get_bytes(data,  2, 4)); }
        public int pointing_b_GET()//roll(deg*100)
        {  return (int)((int) get_bytes(data,  6, 4)); }
        public int pointing_c_GET()//yaw(deg*100)
        {  return (int)((int) get_bytes(data,  10, 4)); }
    }
    public static class FENCE_POINT extends GroundControl.FENCE_POINT
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char idx_GET()//point index (first point is 1, 0 is for return point)
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char count_GET()//total number of points (for sanity checking)
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public float lat_GET()//Latitude of point
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float lng_GET()//Longitude of point
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
    }
    public static class FENCE_FETCH_POINT extends GroundControl.FENCE_FETCH_POINT
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char idx_GET()//point index (first point is 1, 0 is for return point)
        {  return (char)((char) get_bytes(data,  2, 1)); }
    }
    public static class FENCE_STATUS extends GroundControl.FENCE_STATUS
    {
        public char breach_count_GET()//number of fence breaches
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public long breach_time_GET()//time of last breach in milliseconds since boot
        {  return (get_bytes(data,  2, 4)); }
        public char breach_status_GET()//0 if currently inside fence, 1 if outside
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public @FENCE_BREACH int breach_type_GET()//last breach type (see FENCE_BREACH_* enum)
        {  return  0 + (int)get_bits(data, 56, 3); }
    }
    public static class AHRS extends GroundControl.AHRS
    {
        public float omegaIx_GET()//X gyro drift estimate rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float omegaIy_GET()//Y gyro drift estimate rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float omegaIz_GET()//Z gyro drift estimate rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float accel_weight_GET()//average accel_weight
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float renorm_val_GET()//average renormalisation value
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float error_rp_GET()//average error_roll_pitch value
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float error_yaw_GET()//average error_yaw value
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
    }
    public static class SIMSTATE extends GroundControl.SIMSTATE
    {
        public float roll_GET()//Roll angle (rad)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float pitch_GET()//Pitch angle (rad)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float yaw_GET()//Yaw angle (rad)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float xacc_GET()//X acceleration m/s/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float yacc_GET()//Y acceleration m/s/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float zacc_GET()//Z acceleration m/s/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float xgyro_GET()//Angular speed around X axis rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float ygyro_GET()//Angular speed around Y axis rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public float zgyro_GET()//Angular speed around Z axis rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public int lat_GET()//Latitude in degrees * 1E7
        {  return (int)((int) get_bytes(data,  36, 4)); }
        public int lng_GET()//Longitude in degrees * 1E7
        {  return (int)((int) get_bytes(data,  40, 4)); }
    }
    public static class HWSTATUS extends GroundControl.HWSTATUS
    {
        public char Vcc_GET()//board voltage (mV)
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char I2Cerr_GET()//I2C error count
        {  return (char)((char) get_bytes(data,  2, 1)); }
    }
    public static class RADIO extends GroundControl.RADIO
    {
        public char rxerrors_GET()//receive errors
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char fixed__GET()//count of error corrected packets
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public char rssi_GET()//local signal strength
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public char remrssi_GET()//remote signal strength
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public char txbuf_GET()//how full the tx buffer is as a percentage
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public char noise_GET()//background noise level
        {  return (char)((char) get_bytes(data,  7, 1)); }
        public char remnoise_GET()//remote background noise level
        {  return (char)((char) get_bytes(data,  8, 1)); }
    }
    public static class LIMITS_STATUS extends GroundControl.LIMITS_STATUS
    {
        public char breach_count_GET()//number of fence breaches
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public long last_trigger_GET()//time of last breach in milliseconds since boot
        {  return (get_bytes(data,  2, 4)); }
        public long last_action_GET()//time of last recovery action in milliseconds since boot
        {  return (get_bytes(data,  6, 4)); }
        public long last_recovery_GET()//time of last successful recovery in milliseconds since boot
        {  return (get_bytes(data,  10, 4)); }
        public long last_clear_GET()//time of last all-clear in milliseconds since boot
        {  return (get_bytes(data,  14, 4)); }
        public @LIMITS_STATE int limits_state_GET()//state of AP_Limits, (see enum LimitState, LIMITS_STATE)
        {  return  0 + (int)get_bits(data, 144, 3); }
        public @LIMIT_MODULE int mods_enabled_GET()//AP_Limit_Module bitfield of enabled modules, (see enum moduleid or LIMIT_MODULE)
        {
            switch((int)get_bits(data, 147, 2))
            {
                case 0:
                    return LIMIT_MODULE.LIMIT_GPSLOCK;
                case 1:
                    return LIMIT_MODULE.LIMIT_GEOFENCE;
                case 2:
                    return LIMIT_MODULE.LIMIT_ALTITUDE;
            }
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public @LIMIT_MODULE int mods_required_GET()//AP_Limit_Module bitfield of required modules, (see enum moduleid or LIMIT_MODULE)
        {
            switch((int)get_bits(data, 149, 2))
            {
                case 0:
                    return LIMIT_MODULE.LIMIT_GPSLOCK;
                case 1:
                    return LIMIT_MODULE.LIMIT_GEOFENCE;
                case 2:
                    return LIMIT_MODULE.LIMIT_ALTITUDE;
            }
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public @LIMIT_MODULE int mods_triggered_GET()//AP_Limit_Module bitfield of triggered modules, (see enum moduleid or LIMIT_MODULE)
        {
            switch((int)get_bits(data, 151, 2))
            {
                case 0:
                    return LIMIT_MODULE.LIMIT_GPSLOCK;
                case 1:
                    return LIMIT_MODULE.LIMIT_GEOFENCE;
                case 2:
                    return LIMIT_MODULE.LIMIT_ALTITUDE;
            }
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
    }
    public static class WIND extends GroundControl.WIND
    {
        public float direction_GET()//wind direction that wind is coming from (degrees)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float speed_GET()//wind speed in ground plane (m/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float speed_z_GET()//vertical wind speed (m/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
    }
    public static class DATA16 extends GroundControl.DATA16
    {
        public char type_GET()//data type
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char len_GET()//data length
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char[] data__GET(char[]  dst_ch, int pos)  //raw data
        {
            for(int BYTE = 2, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//raw data
        {return data__GET(new char[16], 0);}
    }
    public static class DATA32 extends GroundControl.DATA32
    {
        public char type_GET()//data type
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char len_GET()//data length
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char[] data__GET(char[]  dst_ch, int pos)  //raw data
        {
            for(int BYTE = 2, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//raw data
        {return data__GET(new char[32], 0);}
    }
    public static class DATA64 extends GroundControl.DATA64
    {
        public char type_GET()//data type
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char len_GET()//data length
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char[] data__GET(char[]  dst_ch, int pos)  //raw data
        {
            for(int BYTE = 2, dst_max = pos + 64; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//raw data
        {return data__GET(new char[64], 0);}
    }
    public static class DATA96 extends GroundControl.DATA96
    {
        public char type_GET()//data type
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char len_GET()//data length
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char[] data__GET(char[]  dst_ch, int pos)  //raw data
        {
            for(int BYTE = 2, dst_max = pos + 96; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//raw data
        {return data__GET(new char[96], 0);}
    }
    public static class RANGEFINDER extends GroundControl.RANGEFINDER
    {
        public float distance_GET()//distance in meters
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float voltage_GET()//raw voltage if available, zero otherwise
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
    }
    public static class AIRSPEED_AUTOCAL extends GroundControl.AIRSPEED_AUTOCAL
    {
        public float vx_GET()//GPS velocity north m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float vy_GET()//GPS velocity east m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float vz_GET()//GPS velocity down m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float diff_pressure_GET()//Differential pressure pascals
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float EAS2TAS_GET()//Estimated to true airspeed ratio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float ratio_GET()//Airspeed ratio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float state_x_GET()//EKF state x
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float state_y_GET()//EKF state y
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public float state_z_GET()//EKF state z
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public float Pax_GET()//EKF Pax
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public float Pby_GET()//EKF Pby
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public float Pcz_GET()//EKF Pcz
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
    }
    public static class RALLY_POINT extends GroundControl.RALLY_POINT
    {
        public char land_dir_GET()//Heading to aim for when landing. In centi-degrees.
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public char idx_GET()//point index (first point is 0)
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public char count_GET()//total number of points (for sanity checking)
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public int lat_GET()//Latitude of point in degrees * 1E7
        {  return (int)((int) get_bytes(data,  6, 4)); }
        public int lng_GET()//Longitude of point in degrees * 1E7
        {  return (int)((int) get_bytes(data,  10, 4)); }
        public short alt_GET()//Transit / loiter altitude in meters relative to home
        {  return (short)((short) get_bytes(data,  14, 2)); }
        public short break_alt_GET()//Break altitude in meters relative to home
        {  return (short)((short) get_bytes(data,  16, 2)); }
        public @RALLY_FLAGS int flags_GET()//See RALLY_FLAGS enum for definition of the bitmask.
        {  return  1 + (int)get_bits(data, 144, 2); }
    }
    public static class RALLY_FETCH_POINT extends GroundControl.RALLY_FETCH_POINT
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char idx_GET()//point index (first point is 0)
        {  return (char)((char) get_bytes(data,  2, 1)); }
    }
    public static class COMPASSMOT_STATUS extends GroundControl.COMPASSMOT_STATUS
    {
        public char throttle_GET()//throttle (percent*10)
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char interference_GET()//interference (percent)
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public float current_GET()//current (Ampere)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float CompensationX_GET()//Motor Compensation X
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float CompensationY_GET()//Motor Compensation Y
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float CompensationZ_GET()//Motor Compensation Z
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
    }
    public static class AHRS2 extends GroundControl.AHRS2
    {
        public float roll_GET()//Roll angle (rad)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float pitch_GET()//Pitch angle (rad)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float yaw_GET()//Yaw angle (rad)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float altitude_GET()//Altitude (MSL)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public int lat_GET()//Latitude in degrees * 1E7
        {  return (int)((int) get_bytes(data,  16, 4)); }
        public int lng_GET()//Longitude in degrees * 1E7
        {  return (int)((int) get_bytes(data,  20, 4)); }
    }
    public static class CAMERA_STATUS extends GroundControl.CAMERA_STATUS
    {
        public char img_idx_GET()//Image index
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public long time_usec_GET()//Image timestamp (microseconds since UNIX epoch, according to camera clock)
        {  return (get_bytes(data,  2, 8)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  10, 1)); }
        public char cam_idx_GET()//Camera ID
        {  return (char)((char) get_bytes(data,  11, 1)); }
        public float p1_GET()//Parameter 1 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float p2_GET()//Parameter 2 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float p3_GET()//Parameter 3 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float p4_GET()//Parameter 4 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public @CAMERA_STATUS_TYPES int event_id_GET()//See CAMERA_STATUS_TYPES enum for definition of the bitmask
        {  return  0 + (int)get_bits(data, 224, 3); }
    }
    public static class CAMERA_FEEDBACK extends GroundControl.CAMERA_FEEDBACK
    {
        public char img_idx_GET()//Image index
        {  return (char)((char) get_bytes(data,  0, 2)); }
        /**
        *Image timestamp (microseconds since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if
        *	no CCB*/
        public long time_usec_GET()
        {  return (get_bytes(data,  2, 8)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  10, 1)); }
        public char cam_idx_GET()//Camera ID
        {  return (char)((char) get_bytes(data,  11, 1)); }
        public int lat_GET()//Latitude in (deg * 1E7)
        {  return (int)((int) get_bytes(data,  12, 4)); }
        public int lng_GET()//Longitude in (deg * 1E7)
        {  return (int)((int) get_bytes(data,  16, 4)); }
        public float alt_msl_GET()//Altitude Absolute (meters AMSL)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float alt_rel_GET()//Altitude Relative (meters above HOME location)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float roll_GET()//Camera Roll angle (earth frame, degrees, +-180)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public float pitch_GET()//Camera Pitch angle (earth frame, degrees, +-180)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public float yaw_GET()//Camera Yaw (earth frame, degrees, 0-360, true)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public float foc_len_GET()//Focal Length (mm)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public @CAMERA_FEEDBACK_FLAGS int flags_GET()//See CAMERA_FEEDBACK_FLAGS enum for definition of the bitmask
        {  return  0 + (int)get_bits(data, 352, 3); }
    }
    public static class BATTERY2 extends GroundControl.BATTERY2
    {
        public char voltage_GET()//voltage in millivolts
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public short current_battery_GET()//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
        {  return (short)((short) get_bytes(data,  2, 2)); }
    }
    public static class AHRS3 extends GroundControl.AHRS3
    {
        public float roll_GET()//Roll angle (rad)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float pitch_GET()//Pitch angle (rad)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float yaw_GET()//Yaw angle (rad)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float altitude_GET()//Altitude (MSL)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public int lat_GET()//Latitude in degrees * 1E7
        {  return (int)((int) get_bytes(data,  16, 4)); }
        public int lng_GET()//Longitude in degrees * 1E7
        {  return (int)((int) get_bytes(data,  20, 4)); }
        public float v1_GET()//test variable1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float v2_GET()//test variable2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public float v3_GET()//test variable3
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public float v4_GET()//test variable4
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
    }
    public static class AUTOPILOT_VERSION_REQUEST extends GroundControl.AUTOPILOT_VERSION_REQUEST
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
    }
    public static class REMOTE_LOG_DATA_BLOCK extends GroundControl.REMOTE_LOG_DATA_BLOCK
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char[] data__GET(char[]  dst_ch, int pos)  //log data block
        {
            for(int BYTE = 2, dst_max = pos + 200; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//log data block
        {return data__GET(new char[200], 0);} public @MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS int seqno_GET()//log data block sequence number
        {  return  2147483645 + (int)get_bits(data, 1616, 2); }
    }
    public static class REMOTE_LOG_BLOCK_STATUS extends GroundControl.REMOTE_LOG_BLOCK_STATUS
    {
        public long seqno_GET()//log data block sequence number
        {  return (get_bytes(data,  0, 4)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public @MAV_REMOTE_LOG_DATA_BLOCK_STATUSES int status_GET()//log data block status
        {  return  0 + (int)get_bits(data, 48, 2); }
    }
    public static class LED_CONTROL extends GroundControl.LED_CONTROL
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char instance_GET()//Instance (LED instance to control or 255 for all LEDs)
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char pattern_GET()//Pattern (see LED_PATTERN_ENUM)
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public char custom_len_GET()//Custom Byte Length
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public char[] custom_bytes_GET(char[]  dst_ch, int pos)  //Custom Bytes
        {
            for(int BYTE = 5, dst_max = pos + 24; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] custom_bytes_GET()//Custom Bytes
        {return custom_bytes_GET(new char[24], 0);}
    }
    public static class MAG_CAL_PROGRESS extends GroundControl.MAG_CAL_PROGRESS
    {
        public char compass_id_GET()//Compass being calibrated
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char cal_mask_GET()//Bitmask of compasses being calibrated
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char attempt_GET()//Attempt number
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char completion_pct_GET()//Completion percentage
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public char[] completion_mask_GET(char[]  dst_ch, int pos)  //Bitmask of sphere sections (see http:en.wikipedia.org/wiki/Geodesic_grid)
        {
            for(int BYTE = 4, dst_max = pos + 10; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] completion_mask_GET()//Bitmask of sphere sections (see http:en.wikipedia.org/wiki/Geodesic_grid)
        {return completion_mask_GET(new char[10], 0);} public float direction_x_GET()//Body frame direction vector for display
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public float direction_y_GET()//Body frame direction vector for display
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public float direction_z_GET()//Body frame direction vector for display
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        public @MAG_CAL_STATUS int cal_status_GET()//Status (see MAG_CAL_STATUS enum)
        {  return  0 + (int)get_bits(data, 208, 3); }
    }
    public static class MAG_CAL_REPORT extends GroundControl.MAG_CAL_REPORT
    {
        public char compass_id_GET()//Compass being calibrated
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char cal_mask_GET()//Bitmask of compasses being calibrated
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char autosaved_GET()//0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public float fitness_GET()//RMS milligauss residuals
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  3, 4))); }
        public float ofs_x_GET()//X offset
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  7, 4))); }
        public float ofs_y_GET()//Y offset
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  11, 4))); }
        public float ofs_z_GET()//Z offset
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  15, 4))); }
        public float diag_x_GET()//X diagonal (matrix 11)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  19, 4))); }
        public float diag_y_GET()//Y diagonal (matrix 22)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  23, 4))); }
        public float diag_z_GET()//Z diagonal (matrix 33)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  27, 4))); }
        public float offdiag_x_GET()//X off-diagonal (matrix 12 and 21)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  31, 4))); }
        public float offdiag_y_GET()//Y off-diagonal (matrix 13 and 31)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  35, 4))); }
        public float offdiag_z_GET()//Z off-diagonal (matrix 32 and 23)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  39, 4))); }
        public @MAG_CAL_STATUS int cal_status_GET()//Status (see MAG_CAL_STATUS enum)
        {  return  0 + (int)get_bits(data, 344, 3); }
    }
    public static class EKF_STATUS_REPORT extends GroundControl.EKF_STATUS_REPORT
    {
        public float velocity_variance_GET()//Velocity variance
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float pos_horiz_variance_GET()//Horizontal Position variance
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float pos_vert_variance_GET()//Vertical Position variance
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float compass_variance_GET()//Compass variance
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float terrain_alt_variance_GET()//Terrain Altitude variance
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public @EKF_STATUS_FLAGS int flags_GET()//Flags
        {
            switch((int)get_bits(data, 160, 4))
            {
                case 0:
                    return EKF_STATUS_FLAGS.EKF_ATTITUDE;
                case 1:
                    return EKF_STATUS_FLAGS.EKF_VELOCITY_HORIZ;
                case 2:
                    return EKF_STATUS_FLAGS.EKF_VELOCITY_VERT;
                case 3:
                    return EKF_STATUS_FLAGS.EKF_POS_HORIZ_REL;
                case 4:
                    return EKF_STATUS_FLAGS.EKF_POS_HORIZ_ABS;
                case 5:
                    return EKF_STATUS_FLAGS.EKF_POS_VERT_ABS;
                case 6:
                    return EKF_STATUS_FLAGS.EKF_POS_VERT_AGL;
                case 7:
                    return EKF_STATUS_FLAGS.EKF_CONST_POS_MODE;
                case 8:
                    return EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_REL;
                case 9:
                    return EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_ABS;
            }
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
    }
    public static class PID_TUNING extends GroundControl.PID_TUNING
    {
        public float desired_GET()//desired rate (degrees/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float achieved_GET()//achieved rate (degrees/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float FF_GET()//FF component
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float P_GET()//P component
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float I_GET()//I component
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float D_GET()//D component
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public @PID_TUNING_AXIS int axis_GET()//axis
        {  return  1 + (int)get_bits(data, 192, 3); }
    }
    public static class GIMBAL_REPORT extends GroundControl.GIMBAL_REPORT
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public float delta_time_GET()//Time since last update (seconds)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  2, 4))); }
        public float delta_angle_x_GET()//Delta angle X (radians)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  6, 4))); }
        public float delta_angle_y_GET()//Delta angle Y (radians)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
        public float delta_angle_z_GET()//Delta angle X (radians)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public float delta_velocity_x_GET()//Delta velocity X (m/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public float delta_velocity_y_GET()//Delta velocity Y (m/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        public float delta_velocity_z_GET()//Delta velocity Z (m/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  26, 4))); }
        public float joint_roll_GET()//Joint ROLL (radians)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  30, 4))); }
        public float joint_el_GET()//Joint EL (radians)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  34, 4))); }
        public float joint_az_GET()//Joint AZ (radians)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  38, 4))); }
    }
    public static class GIMBAL_CONTROL extends GroundControl.GIMBAL_CONTROL
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public float demanded_rate_x_GET()//Demanded angular rate X (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  2, 4))); }
        public float demanded_rate_y_GET()//Demanded angular rate Y (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  6, 4))); }
        public float demanded_rate_z_GET()//Demanded angular rate Z (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
    }
    public static class GIMBAL_TORQUE_CMD_REPORT extends GroundControl.GIMBAL_TORQUE_CMD_REPORT
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public short rl_torque_cmd_GET()//Roll Torque Command
        {  return (short)((short) get_bytes(data,  2, 2)); }
        public short el_torque_cmd_GET()//Elevation Torque Command
        {  return (short)((short) get_bytes(data,  4, 2)); }
        public short az_torque_cmd_GET()//Azimuth Torque Command
        {  return (short)((short) get_bytes(data,  6, 2)); }
    }
    public static class GOPRO_HEARTBEAT extends GroundControl.GOPRO_HEARTBEAT
    {
        public @GOPRO_HEARTBEAT_STATUS int status_GET()//Status
        {  return  0 + (int)get_bits(data, 0, 3); }
        public @GOPRO_CAPTURE_MODE int capture_mode_GET()//Current capture mode
        {
            switch((int)get_bits(data, 3, 4))
            {
                case 0:
                    return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_VIDEO;
                case 1:
                    return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_PHOTO;
                case 2:
                    return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_BURST;
                case 3:
                    return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_TIME_LAPSE;
                case 4:
                    return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_MULTI_SHOT;
                case 5:
                    return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_PLAYBACK;
                case 6:
                    return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_SETUP;
                case 7:
                    return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_UNKNOWN;
            }
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public @GOPRO_HEARTBEAT_FLAGS int flags_GET()//additional status bits
        {  return  1 + (int)get_bits(data, 7, 1); }
    }
    public static class GOPRO_GET_REQUEST extends GroundControl.GOPRO_GET_REQUEST
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public @GOPRO_COMMAND int cmd_id_GET()//Command ID
        {  return  0 + (int)get_bits(data, 16, 5); }
    }
    public static class GOPRO_GET_RESPONSE extends GroundControl.GOPRO_GET_RESPONSE
    {
        public char[] value_GET(char[]  dst_ch, int pos)  //Value
        {
            for(int BYTE = 0, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] value_GET()//Value
        {return value_GET(new char[4], 0);} public @GOPRO_COMMAND int cmd_id_GET()//Command ID
        {  return  0 + (int)get_bits(data, 32, 5); }
        public @GOPRO_REQUEST_STATUS int status_GET()//Status
        {  return  0 + (int)get_bits(data, 37, 2); }
    }
    public static class GOPRO_SET_REQUEST extends GroundControl.GOPRO_SET_REQUEST
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char[] value_GET(char[]  dst_ch, int pos)  //Value
        {
            for(int BYTE = 2, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] value_GET()//Value
        {return value_GET(new char[4], 0);} public @GOPRO_COMMAND int cmd_id_GET()//Command ID
        {  return  0 + (int)get_bits(data, 48, 5); }
    }
    public static class GOPRO_SET_RESPONSE extends GroundControl.GOPRO_SET_RESPONSE
    {
        public @GOPRO_COMMAND int cmd_id_GET()//Command ID
        {  return  0 + (int)get_bits(data, 0, 5); }
        public @GOPRO_REQUEST_STATUS int status_GET()//Status
        {  return  0 + (int)get_bits(data, 5, 2); }
    }
    public static class RPM extends GroundControl.RPM
    {
        public float rpm1_GET()//RPM Sensor1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float rpm2_GET()//RPM Sensor2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
    }
    public static class FLIGHT_INFORMATION extends GroundControl.FLIGHT_INFORMATION
    {
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public long arming_time_utc_GET()//Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknown
        {  return (get_bytes(data,  4, 8)); }
        public long takeoff_time_utc_GET()//Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknown
        {  return (get_bytes(data,  12, 8)); }
        public long flight_uuid_GET()//Universally unique identifier (UUID) of flight, should correspond to name of logfiles
        {  return (get_bytes(data,  20, 8)); }
    }
    public static class MOUNT_ORIENTATION extends GroundControl.MOUNT_ORIENTATION
    {
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public float roll_GET()//Roll in degrees
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float pitch_GET()//Pitch in degrees
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float yaw_GET()//Yaw in degrees
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
    }
    public static class LOGGING_DATA extends GroundControl.LOGGING_DATA
    {
        public char sequence_GET()//sequence number (can wrap)
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char target_system_GET()//system ID of the target
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char target_component_GET()//component ID of the target
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public char length_GET()//data length
        {  return (char)((char) get_bytes(data,  4, 1)); }
        /**
        *offset into data where first message starts. This can be used for recovery, when a previous message got
        *	lost (set to 255 if no start exists)*/
        public char first_message_offset_GET()
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public char[] data__GET(char[]  dst_ch, int pos)  //logged data
        {
            for(int BYTE = 6, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//logged data
        {return data__GET(new char[249], 0);}
    }
    public static class LOGGING_DATA_ACKED extends GroundControl.LOGGING_DATA_ACKED
    {
        public char sequence_GET()//sequence number (can wrap)
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char target_system_GET()//system ID of the target
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char target_component_GET()//component ID of the target
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public char length_GET()//data length
        {  return (char)((char) get_bytes(data,  4, 1)); }
        /**
        *offset into data where first message starts. This can be used for recovery, when a previous message got
        *	lost (set to 255 if no start exists)*/
        public char first_message_offset_GET()
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public char[] data__GET(char[]  dst_ch, int pos)  //logged data
        {
            for(int BYTE = 6, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//logged data
        {return data__GET(new char[249], 0);}
    }
    public static class LOGGING_ACK extends GroundControl.LOGGING_ACK
    {
        public char sequence_GET()//sequence number (must match the one in LOGGING_DATA_ACKED)
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char target_system_GET()//system ID of the target
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char target_component_GET()//component ID of the target
        {  return (char)((char) get_bytes(data,  3, 1)); }
    }
    public static class VIDEO_STREAM_INFORMATION extends GroundControl.VIDEO_STREAM_INFORMATION
    {
        public char resolution_h_GET()//Resolution horizontal in pixels
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char resolution_v_GET()//Resolution vertical in pixels
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public char rotation_GET()//Video image rotation clockwise
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public long bitrate_GET()//Bit rate in bits per second
        {  return (get_bytes(data,  6, 4)); }
        public char camera_id_GET()//Camera ID (1 for first, 2 for second, etc.)
        {  return (char)((char) get_bytes(data,  10, 1)); }
        public char status_GET()//Current status of video streaming (0: not running, 1: in progress)
        {  return (char)((char) get_bytes(data,  11, 1)); }
        public float framerate_GET()//Frames per second
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public String uri_TRY(Bounds.Inside ph)//Video stream URI
        {
            if(ph.field_bit !=  130 && !try_visit_field(ph, 130)  ||  !try_visit_item(ph, 0)) return null;
            return new String(uri_GET(ph, new char[ph.items], 0));
        }
        public char[] uri_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Video stream URI
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int uri_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  130 && !try_visit_field(ph, 130)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class SET_VIDEO_STREAM_SETTINGS extends GroundControl.SET_VIDEO_STREAM_SETTINGS
    {
        public char resolution_h_GET()//Resolution horizontal in pixels (set to -1 for highest resolution possible)
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char resolution_v_GET()//Resolution vertical in pixels (set to -1 for highest resolution possible)
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public char rotation_GET()//Video image rotation clockwise (0-359 degrees)
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public long bitrate_GET()//Bit rate in bits per second (set to -1 for auto)
        {  return (get_bytes(data,  6, 4)); }
        public char target_system_GET()//system ID of the target
        {  return (char)((char) get_bytes(data,  10, 1)); }
        public char target_component_GET()//component ID of the target
        {  return (char)((char) get_bytes(data,  11, 1)); }
        public char camera_id_GET()//Camera ID (1 for first, 2 for second, etc.)
        {  return (char)((char) get_bytes(data,  12, 1)); }
        public float framerate_GET()//Frames per second (set to -1 for highest framerate possible)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  13, 4))); }
        public String uri_TRY(Bounds.Inside ph)//Video stream URI
        {
            if(ph.field_bit !=  138 && !try_visit_field(ph, 138)  ||  !try_visit_item(ph, 0)) return null;
            return new String(uri_GET(ph, new char[ph.items], 0));
        }
        public char[] uri_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Video stream URI
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int uri_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  138 && !try_visit_field(ph, 138)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class WIFI_CONFIG_AP extends GroundControl.WIFI_CONFIG_AP
    {
        public String ssid_TRY(Bounds.Inside ph)//Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged.
        {
            if(ph.field_bit !=  2 && !try_visit_field(ph, 2)  ||  !try_visit_item(ph, 0)) return null;
            return new String(ssid_GET(ph, new char[ph.items], 0));
        }
        public char[] ssid_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged.
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int ssid_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  2 && !try_visit_field(ph, 2)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public String password_TRY(Bounds.Inside ph)//Password. Leave it blank for an open AP.
        {
            if(ph.field_bit !=  3 && !try_visit_field(ph, 3)  ||  !try_visit_item(ph, 0)) return null;
            return new String(password_GET(ph, new char[ph.items], 0));
        }
        public char[] password_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Password. Leave it blank for an open AP.
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int password_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  3 && !try_visit_field(ph, 3)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class PROTOCOL_VERSION extends GroundControl.PROTOCOL_VERSION
    {
        public char version_GET()//Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char min_version_GET()//Minimum MAVLink version supported
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public char max_version_GET()//Maximum MAVLink version supported (set to the same value as version by default)
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public char[] spec_version_hash_GET(char[]  dst_ch, int pos)  //The first 8 bytes (not characters printed in hex!) of the git hash.
        {
            for(int BYTE = 6, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] spec_version_hash_GET()//The first 8 bytes (not characters printed in hex!) of the git hash.
        {return spec_version_hash_GET(new char[8], 0);} public char[] library_version_hash_GET(char[]  dst_ch, int pos)  //The first 8 bytes (not characters printed in hex!) of the git hash.
        {
            for(int BYTE = 14, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] library_version_hash_GET()//The first 8 bytes (not characters printed in hex!) of the git hash.
        {return library_version_hash_GET(new char[8], 0);}
    }
    public static class UAVCAN_NODE_STATUS extends GroundControl.UAVCAN_NODE_STATUS
    {
        public char vendor_specific_status_code_GET()//Vendor-specific status information.
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public long uptime_sec_GET()//The number of seconds since the start-up of the node.
        {  return (get_bytes(data,  2, 4)); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  return (get_bytes(data,  6, 8)); }
        public char sub_mode_GET()//Not used currently.
        {  return (char)((char) get_bytes(data,  14, 1)); }
        public @UAVCAN_NODE_HEALTH int health_GET()//Generalized node health status.
        {  return  0 + (int)get_bits(data, 120, 3); }
        public @UAVCAN_NODE_MODE int mode_GET()//Generalized operating mode.
        {
            switch((int)get_bits(data, 123, 3))
            {
                case 0:
                    return UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL;
                case 1:
                    return UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION;
                case 2:
                    return UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE;
                case 3:
                    return UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE;
                case 4:
                    return UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE;
            }
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
    }
    public static class UAVCAN_NODE_INFO extends GroundControl.UAVCAN_NODE_INFO
    {
        public long uptime_sec_GET()//The number of seconds since the start-up of the node.
        {  return (get_bytes(data,  0, 4)); }
        public long sw_vcs_commit_GET()//Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.
        {  return (get_bytes(data,  4, 4)); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  return (get_bytes(data,  8, 8)); }
        public char hw_version_major_GET()//Hardware major version number.
        {  return (char)((char) get_bytes(data,  16, 1)); }
        public char hw_version_minor_GET()//Hardware minor version number.
        {  return (char)((char) get_bytes(data,  17, 1)); }
        public char[] hw_unique_id_GET(char[]  dst_ch, int pos)  //Hardware unique 128-bit ID.
        {
            for(int BYTE = 18, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] hw_unique_id_GET()//Hardware unique 128-bit ID.
        {return hw_unique_id_GET(new char[16], 0);} public char sw_version_major_GET()//Software major version number.
        {  return (char)((char) get_bytes(data,  34, 1)); }
        public char sw_version_minor_GET()//Software minor version number.
        {  return (char)((char) get_bytes(data,  35, 1)); }
        public String name_TRY(Bounds.Inside ph)//Node name string. For example, "sapog.px4.io".
        {
            if(ph.field_bit !=  288 && !try_visit_field(ph, 288)  ||  !try_visit_item(ph, 0)) return null;
            return new String(name_GET(ph, new char[ph.items], 0));
        }
        public char[] name_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Node name string. For example, "sapog.px4.io".
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int name_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  288 && !try_visit_field(ph, 288)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class PARAM_EXT_REQUEST_READ extends GroundControl.PARAM_EXT_REQUEST_READ
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public short param_index_GET()//Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignored
        {  return (short)((short) get_bytes(data,  2, 2)); }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	ID is stored as strin*/
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	ID is stored as strin*/
        public char[] param_id_GET(Bounds.Inside ph, char[]  dst_ch, int pos)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_id_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class PARAM_EXT_REQUEST_LIST extends GroundControl.PARAM_EXT_REQUEST_LIST
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
    }
    public static class PARAM_EXT_VALUE extends GroundControl.PARAM_EXT_VALUE
    {
        public char param_count_GET()//Total number of parameters
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char param_index_GET()//Index of this parameter
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public @MAV_PARAM_EXT_TYPE int param_type_GET()//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
        {  return  1 + (int)get_bits(data, 32, 4); }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	ID is stored as strin*/
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  38 && !try_visit_field(ph, 38)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	ID is stored as strin*/
        public char[] param_id_GET(Bounds.Inside ph, char[]  dst_ch, int pos)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_id_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  38 && !try_visit_field(ph, 38)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public String param_value_TRY(Bounds.Inside ph)//Parameter value
        {
            if(ph.field_bit !=  39 && !try_visit_field(ph, 39)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_value_GET(ph, new char[ph.items], 0));
        }
        public char[] param_value_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Parameter value
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_value_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  39 && !try_visit_field(ph, 39)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class PARAM_EXT_SET extends GroundControl.PARAM_EXT_SET
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public @MAV_PARAM_EXT_TYPE int param_type_GET()//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
        {  return  1 + (int)get_bits(data, 16, 4); }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	ID is stored as strin*/
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  22 && !try_visit_field(ph, 22)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	ID is stored as strin*/
        public char[] param_id_GET(Bounds.Inside ph, char[]  dst_ch, int pos)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_id_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  22 && !try_visit_field(ph, 22)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public String param_value_TRY(Bounds.Inside ph)//Parameter value
        {
            if(ph.field_bit !=  23 && !try_visit_field(ph, 23)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_value_GET(ph, new char[ph.items], 0));
        }
        public char[] param_value_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Parameter value
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_value_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  23 && !try_visit_field(ph, 23)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class PARAM_EXT_ACK extends GroundControl.PARAM_EXT_ACK
    {
        public @MAV_PARAM_EXT_TYPE int param_type_GET()//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
        {  return  1 + (int)get_bits(data, 0, 4); }
        public @PARAM_ACK int param_result_GET()//Result code: see the PARAM_ACK enum for possible codes.
        {  return  0 + (int)get_bits(data, 4, 3); }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	ID is stored as strin*/
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  9 && !try_visit_field(ph, 9)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	ID is stored as strin*/
        public char[] param_id_GET(Bounds.Inside ph, char[]  dst_ch, int pos)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_id_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  9 && !try_visit_field(ph, 9)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public String param_value_TRY(Bounds.Inside ph)//Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
        {
            if(ph.field_bit !=  10 && !try_visit_field(ph, 10)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_value_GET(ph, new char[ph.items], 0));
        }
        public char[] param_value_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_value_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  10 && !try_visit_field(ph, 10)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class OBSTACLE_DISTANCE extends GroundControl.OBSTACLE_DISTANCE
    {
        /**
        *Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
        *	is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
        *	for unknown/not used. In a array element, each unit corresponds to 1cm*/
        public char[] distances_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 0, dst_max = pos + 72; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        /**
        *Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
        *	is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
        *	for unknown/not used. In a array element, each unit corresponds to 1cm*/
        public char[] distances_GET()
        {return distances_GET(new char[72], 0);} public char min_distance_GET()//Minimum distance the sensor can measure in centimeters
        {  return (char)((char) get_bytes(data,  144, 2)); }
        public char max_distance_GET()//Maximum distance the sensor can measure in centimeters
        {  return (char)((char) get_bytes(data,  146, 2)); }
        public long time_usec_GET()//Timestamp (microseconds since system boot or since UNIX epoch)
        {  return (get_bytes(data,  148, 8)); }
        public char increment_GET()//Angular width in degrees of each array element.
        {  return (char)((char) get_bytes(data,  156, 1)); }
        public @MAV_DISTANCE_SENSOR int sensor_type_GET()//Class id of the distance sensor type.
        {  return  0 + (int)get_bits(data, 1256, 3); }
    }
    public static class UAVIONIX_ADSB_OUT_CFG extends GroundControl.UAVIONIX_ADSB_OUT_CFG
    {
        public char stallSpeed_GET()//Aircraft stall speed in cm/s
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public long ICAO_GET()//Vehicle address (24 bit)
        {  return (get_bytes(data,  2, 4)); }
        public @ADSB_EMITTER_TYPE int emitterType_GET()//Transmitting vehicle type. See ADSB_EMITTER_TYPE enum
        {  return  0 + (int)get_bits(data, 48, 5); }
        public @UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE int aircraftSize_GET()//Aircraft length and width encoding (table 2-35 of DO-282B)
        {  return  0 + (int)get_bits(data, 53, 5); }
        public @UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT int gpsOffsetLat_GET()//GPS antenna lateral offset (table 2-36 of DO-282B)
        {  return  0 + (int)get_bits(data, 58, 4); }
        /**
        *GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and add
        *	one] (table 2-37 DO-282B*/
        public @UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON int gpsOffsetLon_GET()
        {  return  0 + (int)get_bits(data, 62, 2); }
        public @UAVIONIX_ADSB_OUT_RF_SELECT int rfSelect_GET()//ADS-B transponder reciever and transmit enable flags
        {  return  0 + (int)get_bits(data, 64, 2); }
        public String callsign_TRY(Bounds.Inside ph)//Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
        {
            if(ph.field_bit !=  66 && !try_visit_field(ph, 66)  ||  !try_visit_item(ph, 0)) return null;
            return new String(callsign_GET(ph, new char[ph.items], 0));
        }
        public char[] callsign_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int callsign_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  66 && !try_visit_field(ph, 66)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class UAVIONIX_ADSB_OUT_DYNAMIC extends GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC
    {
        public char accuracyVert_GET()//Vertical accuracy in cm. If unknown set to UINT16_MAX
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char accuracyVel_GET()//Velocity accuracy in mm/s (m * 1E-3). If unknown set to UINT16_MAX
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public char squawk_GET()//Mode A code (typically 1200 [0x04B0] for VFR)
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public long utcTime_GET()//UTC time in seconds since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MAX
        {  return (get_bytes(data,  6, 4)); }
        public long accuracyHor_GET()//Horizontal accuracy in mm (m * 1E-3). If unknown set to UINT32_MAX
        {  return (get_bytes(data,  10, 4)); }
        public int gpsLat_GET()//Latitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
        {  return (int)((int) get_bytes(data,  14, 4)); }
        public int gpsLon_GET()//Longitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
        {  return (int)((int) get_bytes(data,  18, 4)); }
        public int gpsAlt_GET()//Altitude in mm (m * 1E-3) UP +ve. WGS84 altitude. If unknown set to INT32_MAX
        {  return (int)((int) get_bytes(data,  22, 4)); }
        public char numSats_GET()//Number of satellites visible. If unknown set to UINT8_MAX
        {  return (char)((char) get_bytes(data,  26, 1)); }
        /**
        *Barometric pressure altitude relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude
        *	(m * 1E-3). (up +ve). If unknown set to INT32_MA*/
        public int baroAltMSL_GET()
        {  return (int)((int) get_bytes(data,  27, 4)); }
        public short velVert_GET()//GPS vertical speed in cm/s. If unknown set to INT16_MAX
        {  return (short)((short) get_bytes(data,  31, 2)); }
        public short velNS_GET()//North-South velocity over ground in cm/s North +ve. If unknown set to INT16_MAX
        {  return (short)((short) get_bytes(data,  33, 2)); }
        public short VelEW_GET()//East-West velocity over ground in cm/s East +ve. If unknown set to INT16_MAX
        {  return (short)((short) get_bytes(data,  35, 2)); }
        public @UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX int gpsFix_GET()//0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK
        {  return  0 + (int)get_bits(data, 296, 3); }
        public @UAVIONIX_ADSB_EMERGENCY_STATUS int emergencyStatus_GET()//Emergency status
        {  return  0 + (int)get_bits(data, 299, 4); }
        public @UAVIONIX_ADSB_OUT_DYNAMIC_STATE int state_GET()//ADS-B transponder dynamic input state flags
        {
            switch((int)get_bits(data, 303, 3))
            {
                case 0:
                    return UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE;
                case 1:
                    return UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED;
                case 2:
                    return UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED;
                case 3:
                    return UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND;
                case 4:
                    return UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT;
            }
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
    }
    public static class UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT extends GroundControl.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT
    {
        public @UAVIONIX_ADSB_RF_HEALTH int rfHealth_GET()//ADS-B transponder messages
        {
            switch((int)get_bits(data, 0, 3))
            {
                case 0:
                    return UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_INITIALIZING;
                case 1:
                    return UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_OK;
                case 2:
                    return UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_TX;
                case 3:
                    return UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_RX;
            }
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
    }
    public static class DEVICE_OP_READ extends GroundControl.DEVICE_OP_READ
    {
        public long request_id_GET()//request ID - copied to reply
        {  return (get_bytes(data,  0, 4)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public char bus_GET()//Bus number
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public char address_GET()//Bus address
        {  return (char)((char) get_bytes(data,  7, 1)); }
        public char regstart_GET()//First register to read
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public char count_GET()//count of registers to read
        {  return (char)((char) get_bytes(data,  9, 1)); }
        public @DEVICE_OP_BUSTYPE int bustype_GET()//The bus type
        {  return  0 + (int)get_bits(data, 80, 2); }
        public String busname_TRY(Bounds.Inside ph)//Name of device on bus (for SPI)
        {
            if(ph.field_bit !=  82 && !try_visit_field(ph, 82)  ||  !try_visit_item(ph, 0)) return null;
            return new String(busname_GET(ph, new char[ph.items], 0));
        }
        public char[] busname_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Name of device on bus (for SPI)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int busname_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  82 && !try_visit_field(ph, 82)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class DEVICE_OP_READ_REPLY extends GroundControl.DEVICE_OP_READ_REPLY
    {
        public long request_id_GET()//request ID - copied from request
        {  return (get_bytes(data,  0, 4)); }
        public char result_GET()//0 for success, anything else is failure code
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public char regstart_GET()//starting register
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public char count_GET()//count of bytes read
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public char[] data__GET(char[]  dst_ch, int pos)  //reply data
        {
            for(int BYTE = 7, dst_max = pos + 128; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//reply data
        {return data__GET(new char[128], 0);}
    }
    public static class DEVICE_OP_WRITE extends GroundControl.DEVICE_OP_WRITE
    {
        public long request_id_GET()//request ID - copied to reply
        {  return (get_bytes(data,  0, 4)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public char bus_GET()//Bus number
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public char address_GET()//Bus address
        {  return (char)((char) get_bytes(data,  7, 1)); }
        public char regstart_GET()//First register to write
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public char count_GET()//count of registers to write
        {  return (char)((char) get_bytes(data,  9, 1)); }
        public char[] data__GET(char[]  dst_ch, int pos)  //write data
        {
            for(int BYTE = 10, dst_max = pos + 128; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//write data
        {return data__GET(new char[128], 0);} public @DEVICE_OP_BUSTYPE int bustype_GET()//The bus type
        {  return  0 + (int)get_bits(data, 1104, 2); }
        public String busname_TRY(Bounds.Inside ph)//Name of device on bus (for SPI)
        {
            if(ph.field_bit !=  1106 && !try_visit_field(ph, 1106)  ||  !try_visit_item(ph, 0)) return null;
            return new String(busname_GET(ph, new char[ph.items], 0));
        }
        public char[] busname_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Name of device on bus (for SPI)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int busname_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  1106 && !try_visit_field(ph, 1106)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class DEVICE_OP_WRITE_REPLY extends GroundControl.DEVICE_OP_WRITE_REPLY
    {
        public long request_id_GET()//request ID - copied from request
        {  return (get_bytes(data,  0, 4)); }
        public char result_GET()//0 for success, anything else is failure code
        {  return (char)((char) get_bytes(data,  4, 1)); }
    }
    public static class ADAP_TUNING extends GroundControl.ADAP_TUNING
    {
        public float desired_GET()//desired rate (degrees/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float achieved_GET()//achieved rate (degrees/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float error_GET()//error between model and vehicle
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float theta_GET()//theta estimated state predictor
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float omega_GET()//omega estimated state predictor
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float sigma_GET()//sigma estimated state predictor
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float theta_dot_GET()//theta derivative
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float omega_dot_GET()//omega derivative
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public float sigma_dot_GET()//sigma derivative
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public float f_GET()//projection operator value
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public float f_dot_GET()//projection operator derivative
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public float u_GET()//u adaptive controlled output command
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        public @PID_TUNING_AXIS int axis_GET()//axis
        {  return  1 + (int)get_bits(data, 384, 3); }
    }
    public static class VISION_POSITION_DELTA extends GroundControl.VISION_POSITION_DELTA
    {
        public long time_usec_GET()//Timestamp (microseconds, synced to UNIX time or since system boot)
        {  return (get_bytes(data,  0, 8)); }
        public long time_delta_usec_GET()//Time in microseconds since the last reported camera frame
        {  return (get_bytes(data,  8, 8)); }
        public float[] angle_delta_GET(float[]  dst_ch, int pos)  //Defines a rotation vector in body frame that rotates the vehicle from the previous to the current orientatio
        {
            for(int BYTE = 16, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] angle_delta_GET()//Defines a rotation vector in body frame that rotates the vehicle from the previous to the current orientatio
        {return angle_delta_GET(new float[3], 0);}/**
*Change in position in meters from previous to current frame rotated into body frame (0=forward, 1=right,
*	2=down*/
        public float[] position_delta_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 28, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        /**
        *Change in position in meters from previous to current frame rotated into body frame (0=forward, 1=right,
        *	2=down*/
        public float[] position_delta_GET()
        {return position_delta_GET(new float[3], 0);} public float confidence_GET()//normalised confidence value from 0 to 100
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
    }

    @SuppressWarnings("unchecked")
    static class TestChannel extends Channel
    {
        static final TestChannel instance = new TestChannel(); //test channel

        public final java.io.InputStream  inputStream          = new InputStream();
        public final java.io.OutputStream outputStream         = new OutputStream();
        public final java.io.InputStream  inputStreamAdvanced  = new AdvancedInputStream();
        public final java.io.OutputStream outputStreamAdvanced = new AdvancedOutputStream();

        @Override protected void failure(String reason)
        {
            super.failure(reason);
            assert(false);
        }

        static final Collection<OnReceive.Handler<SENSOR_OFFSETS, Channel>> on_SENSOR_OFFSETS = new OnReceive<>();
        static final Collection<OnReceive.Handler<SET_MAG_OFFSETS, Channel>> on_SET_MAG_OFFSETS = new OnReceive<>();
        static final Collection<OnReceive.Handler<MEMINFO, Channel>> on_MEMINFO = new OnReceive<>();
        static final Collection<OnReceive.Handler<AP_ADC, Channel>> on_AP_ADC = new OnReceive<>();
        static final Collection<OnReceive.Handler<DIGICAM_CONFIGURE, Channel>> on_DIGICAM_CONFIGURE = new OnReceive<>();
        static final Collection<OnReceive.Handler<DIGICAM_CONTROL, Channel>> on_DIGICAM_CONTROL = new OnReceive<>();
        static final Collection<OnReceive.Handler<MOUNT_CONFIGURE, Channel>> on_MOUNT_CONFIGURE = new OnReceive<>();
        static final Collection<OnReceive.Handler<MOUNT_CONTROL, Channel>> on_MOUNT_CONTROL = new OnReceive<>();
        static final Collection<OnReceive.Handler<MOUNT_STATUS, Channel>> on_MOUNT_STATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<FENCE_POINT, Channel>> on_FENCE_POINT = new OnReceive<>();
        static final Collection<OnReceive.Handler<FENCE_FETCH_POINT, Channel>> on_FENCE_FETCH_POINT = new OnReceive<>();
        static final Collection<OnReceive.Handler<FENCE_STATUS, Channel>> on_FENCE_STATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<AHRS, Channel>> on_AHRS = new OnReceive<>();
        static final Collection<OnReceive.Handler<SIMSTATE, Channel>> on_SIMSTATE = new OnReceive<>();
        static final Collection<OnReceive.Handler<HWSTATUS, Channel>> on_HWSTATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<RADIO, Channel>> on_RADIO = new OnReceive<>();
        static final Collection<OnReceive.Handler<LIMITS_STATUS, Channel>> on_LIMITS_STATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<WIND, Channel>> on_WIND = new OnReceive<>();
        static final Collection<OnReceive.Handler<DATA16, Channel>> on_DATA16 = new OnReceive<>();
        static final Collection<OnReceive.Handler<DATA32, Channel>> on_DATA32 = new OnReceive<>();
        static final Collection<OnReceive.Handler<DATA64, Channel>> on_DATA64 = new OnReceive<>();
        static final Collection<OnReceive.Handler<DATA96, Channel>> on_DATA96 = new OnReceive<>();
        static final Collection<OnReceive.Handler<RANGEFINDER, Channel>> on_RANGEFINDER = new OnReceive<>();
        static final Collection<OnReceive.Handler<AIRSPEED_AUTOCAL, Channel>> on_AIRSPEED_AUTOCAL = new OnReceive<>();
        static final Collection<OnReceive.Handler<RALLY_POINT, Channel>> on_RALLY_POINT = new OnReceive<>();
        static final Collection<OnReceive.Handler<RALLY_FETCH_POINT, Channel>> on_RALLY_FETCH_POINT = new OnReceive<>();
        static final Collection<OnReceive.Handler<COMPASSMOT_STATUS, Channel>> on_COMPASSMOT_STATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<AHRS2, Channel>> on_AHRS2 = new OnReceive<>();
        static final Collection<OnReceive.Handler<CAMERA_STATUS, Channel>> on_CAMERA_STATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<CAMERA_FEEDBACK, Channel>> on_CAMERA_FEEDBACK = new OnReceive<>();
        static final Collection<OnReceive.Handler<BATTERY2, Channel>> on_BATTERY2 = new OnReceive<>();
        static final Collection<OnReceive.Handler<AHRS3, Channel>> on_AHRS3 = new OnReceive<>();
        static final Collection<OnReceive.Handler<AUTOPILOT_VERSION_REQUEST, Channel>> on_AUTOPILOT_VERSION_REQUEST = new OnReceive<>();
        static final Collection<OnReceive.Handler<REMOTE_LOG_DATA_BLOCK, Channel>> on_REMOTE_LOG_DATA_BLOCK = new OnReceive<>();
        static final Collection<OnReceive.Handler<REMOTE_LOG_BLOCK_STATUS, Channel>> on_REMOTE_LOG_BLOCK_STATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<LED_CONTROL, Channel>> on_LED_CONTROL = new OnReceive<>();
        static final Collection<OnReceive.Handler<MAG_CAL_PROGRESS, Channel>> on_MAG_CAL_PROGRESS = new OnReceive<>();
        static final Collection<OnReceive.Handler<MAG_CAL_REPORT, Channel>> on_MAG_CAL_REPORT = new OnReceive<>();
        static final Collection<OnReceive.Handler<EKF_STATUS_REPORT, Channel>> on_EKF_STATUS_REPORT = new OnReceive<>();
        static final Collection<OnReceive.Handler<PID_TUNING, Channel>> on_PID_TUNING = new OnReceive<>();
        static final Collection<OnReceive.Handler<GIMBAL_REPORT, Channel>> on_GIMBAL_REPORT = new OnReceive<>();
        static final Collection<OnReceive.Handler<GIMBAL_CONTROL, Channel>> on_GIMBAL_CONTROL = new OnReceive<>();
        static final Collection<OnReceive.Handler<GIMBAL_TORQUE_CMD_REPORT, Channel>> on_GIMBAL_TORQUE_CMD_REPORT = new OnReceive<>();
        static final Collection<OnReceive.Handler<GOPRO_HEARTBEAT, Channel>> on_GOPRO_HEARTBEAT = new OnReceive<>();
        static final Collection<OnReceive.Handler<GOPRO_GET_REQUEST, Channel>> on_GOPRO_GET_REQUEST = new OnReceive<>();
        static final Collection<OnReceive.Handler<GOPRO_GET_RESPONSE, Channel>> on_GOPRO_GET_RESPONSE = new OnReceive<>();
        static final Collection<OnReceive.Handler<GOPRO_SET_REQUEST, Channel>> on_GOPRO_SET_REQUEST = new OnReceive<>();
        static final Collection<OnReceive.Handler<GOPRO_SET_RESPONSE, Channel>> on_GOPRO_SET_RESPONSE = new OnReceive<>();
        static final Collection<OnReceive.Handler<RPM, Channel>> on_RPM = new OnReceive<>();
        static final Collection<OnReceive.Handler<FLIGHT_INFORMATION, Channel>> on_FLIGHT_INFORMATION = new OnReceive<>();
        static final Collection<OnReceive.Handler<MOUNT_ORIENTATION, Channel>> on_MOUNT_ORIENTATION = new OnReceive<>();
        static final Collection<OnReceive.Handler<LOGGING_DATA, Channel>> on_LOGGING_DATA = new OnReceive<>();
        static final Collection<OnReceive.Handler<LOGGING_DATA_ACKED, Channel>> on_LOGGING_DATA_ACKED = new OnReceive<>();
        static final Collection<OnReceive.Handler<LOGGING_ACK, Channel>> on_LOGGING_ACK = new OnReceive<>();
        static final Collection<OnReceive.Handler<VIDEO_STREAM_INFORMATION, Channel>> on_VIDEO_STREAM_INFORMATION = new OnReceive<>();
        static final Collection<OnReceive.Handler<SET_VIDEO_STREAM_SETTINGS, Channel>> on_SET_VIDEO_STREAM_SETTINGS = new OnReceive<>();
        static final Collection<OnReceive.Handler<WIFI_CONFIG_AP, Channel>> on_WIFI_CONFIG_AP = new OnReceive<>();
        static final Collection<OnReceive.Handler<PROTOCOL_VERSION, Channel>> on_PROTOCOL_VERSION = new OnReceive<>();
        static final Collection<OnReceive.Handler<UAVCAN_NODE_STATUS, Channel>> on_UAVCAN_NODE_STATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<UAVCAN_NODE_INFO, Channel>> on_UAVCAN_NODE_INFO = new OnReceive<>();
        static final Collection<OnReceive.Handler<PARAM_EXT_REQUEST_READ, Channel>> on_PARAM_EXT_REQUEST_READ = new OnReceive<>();
        static final Collection<OnReceive.Handler<PARAM_EXT_REQUEST_LIST, Channel>> on_PARAM_EXT_REQUEST_LIST = new OnReceive<>();
        static final Collection<OnReceive.Handler<PARAM_EXT_VALUE, Channel>> on_PARAM_EXT_VALUE = new OnReceive<>();
        static final Collection<OnReceive.Handler<PARAM_EXT_SET, Channel>> on_PARAM_EXT_SET = new OnReceive<>();
        static final Collection<OnReceive.Handler<PARAM_EXT_ACK, Channel>> on_PARAM_EXT_ACK = new OnReceive<>();
        static final Collection<OnReceive.Handler<OBSTACLE_DISTANCE, Channel>> on_OBSTACLE_DISTANCE = new OnReceive<>();
        static final Collection<OnReceive.Handler<UAVIONIX_ADSB_OUT_CFG, Channel>> on_UAVIONIX_ADSB_OUT_CFG = new OnReceive<>();
        static final Collection<OnReceive.Handler<UAVIONIX_ADSB_OUT_DYNAMIC, Channel>> on_UAVIONIX_ADSB_OUT_DYNAMIC = new OnReceive<>();
        static final Collection<OnReceive.Handler<UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT, Channel>> on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT = new OnReceive<>();
        static final Collection<OnReceive.Handler<DEVICE_OP_READ, Channel>> on_DEVICE_OP_READ = new OnReceive<>();
        static final Collection<OnReceive.Handler<DEVICE_OP_READ_REPLY, Channel>> on_DEVICE_OP_READ_REPLY = new OnReceive<>();
        static final Collection<OnReceive.Handler<DEVICE_OP_WRITE, Channel>> on_DEVICE_OP_WRITE = new OnReceive<>();
        static final Collection<OnReceive.Handler<DEVICE_OP_WRITE_REPLY, Channel>> on_DEVICE_OP_WRITE_REPLY = new OnReceive<>();
        static final Collection<OnReceive.Handler<ADAP_TUNING, Channel>> on_ADAP_TUNING = new OnReceive<>();
        static final Collection<OnReceive.Handler<VISION_POSITION_DELTA, Channel>> on_VISION_POSITION_DELTA = new OnReceive<>();


        static Pack testing_pack; //one pack send/receive buffer

        void send(Pack pack) { testing_pack = pack; }

        final Bounds.Inside ph = new Bounds.Inside();

        @Override protected Pack process(Pack pack, int id)
        {
            switch(id)
            {
                case Channel.PROCESS_CHANNEL_REQEST:
                    if(pack == null)
                    {
                        pack = testing_pack;
                        testing_pack = null;
                    }
                    else testing_pack = pack;
                    return pack;
                case Channel.PROCESS_RECEIVED:
                    if(testing_pack == null) return null;
                    ph.setPack(pack = testing_pack);
                    testing_pack = null;
                    id = pack.meta.id;
            }
            switch(id)
            {
                case 0:
                    if(pack == null) return new HEARTBEAT();
                    break;
                case 1:
                    if(pack == null) return new SYS_STATUS();
                    break;
                case 2:
                    if(pack == null) return new SYSTEM_TIME();
                    break;
                case 3:
                    if(pack == null) return new POSITION_TARGET_LOCAL_NED();
                    break;
                case 4:
                    if(pack == null) return new PING();
                    break;
                case 5:
                    if(pack == null) return new CHANGE_OPERATOR_CONTROL();
                    break;
                case 6:
                    if(pack == null) return new CHANGE_OPERATOR_CONTROL_ACK();
                    break;
                case 7:
                    if(pack == null) return new AUTH_KEY();
                    break;
                case 11:
                    if(pack == null) return new SET_MODE();
                    break;
                case 20:
                    if(pack == null) return new PARAM_REQUEST_READ();
                    break;
                case 21:
                    if(pack == null) return new PARAM_REQUEST_LIST();
                    break;
                case 22:
                    if(pack == null) return new PARAM_VALUE();
                    break;
                case 23:
                    if(pack == null) return new PARAM_SET();
                    break;
                case 24:
                    if(pack == null) return new GPS_RAW_INT();
                    break;
                case 25:
                    if(pack == null) return new GPS_STATUS();
                    break;
                case 26:
                    if(pack == null) return new SCALED_IMU();
                    break;
                case 27:
                    if(pack == null) return new RAW_IMU();
                    break;
                case 28:
                    if(pack == null) return new RAW_PRESSURE();
                    break;
                case 29:
                    if(pack == null) return new SCALED_PRESSURE();
                    break;
                case 30:
                    if(pack == null) return new ATTITUDE();
                    break;
                case 31:
                    if(pack == null) return new ATTITUDE_QUATERNION();
                    break;
                case 32:
                    if(pack == null) return new LOCAL_POSITION_NED();
                    break;
                case 33:
                    if(pack == null) return new GLOBAL_POSITION_INT();
                    break;
                case 34:
                    if(pack == null) return new RC_CHANNELS_SCALED();
                    break;
                case 35:
                    if(pack == null) return new RC_CHANNELS_RAW();
                    break;
                case 36:
                    if(pack == null) return new SERVO_OUTPUT_RAW();
                    break;
                case 37:
                    if(pack == null) return new MISSION_REQUEST_PARTIAL_LIST();
                    break;
                case 38:
                    if(pack == null) return new MISSION_WRITE_PARTIAL_LIST();
                    break;
                case 39:
                    if(pack == null) return new MISSION_ITEM();
                    break;
                case 40:
                    if(pack == null) return new MISSION_REQUEST();
                    break;
                case 41:
                    if(pack == null) return new MISSION_SET_CURRENT();
                    break;
                case 42:
                    if(pack == null) return new MISSION_CURRENT();
                    break;
                case 43:
                    if(pack == null) return new MISSION_REQUEST_LIST();
                    break;
                case 44:
                    if(pack == null) return new MISSION_COUNT();
                    break;
                case 45:
                    if(pack == null) return new MISSION_CLEAR_ALL();
                    break;
                case 46:
                    if(pack == null) return new MISSION_ITEM_REACHED();
                    break;
                case 47:
                    if(pack == null) return new MISSION_ACK();
                    break;
                case 48:
                    if(pack == null) return new SET_GPS_GLOBAL_ORIGIN();
                    break;
                case 49:
                    if(pack == null) return new GPS_GLOBAL_ORIGIN();
                    break;
                case 50:
                    if(pack == null) return new PARAM_MAP_RC();
                    break;
                case 51:
                    if(pack == null) return new MISSION_REQUEST_INT();
                    break;
                case 54:
                    if(pack == null) return new SAFETY_SET_ALLOWED_AREA();
                    break;
                case 55:
                    if(pack == null) return new SAFETY_ALLOWED_AREA();
                    break;
                case 61:
                    if(pack == null) return new ATTITUDE_QUATERNION_COV();
                    break;
                case 62:
                    if(pack == null) return new NAV_CONTROLLER_OUTPUT();
                    break;
                case 63:
                    if(pack == null) return new GLOBAL_POSITION_INT_COV();
                    break;
                case 64:
                    if(pack == null) return new LOCAL_POSITION_NED_COV();
                    break;
                case 65:
                    if(pack == null) return new RC_CHANNELS();
                    break;
                case 66:
                    if(pack == null) return new REQUEST_DATA_STREAM();
                    break;
                case 67:
                    if(pack == null) return new DATA_STREAM();
                    break;
                case 69:
                    if(pack == null) return new MANUAL_CONTROL();
                    break;
                case 70:
                    if(pack == null) return new RC_CHANNELS_OVERRIDE();
                    break;
                case 73:
                    if(pack == null) return new MISSION_ITEM_INT();
                    break;
                case 74:
                    if(pack == null) return new VFR_HUD();
                    break;
                case 75:
                    if(pack == null) return new COMMAND_INT();
                    break;
                case 76:
                    if(pack == null) return new COMMAND_LONG();
                    break;
                case 77:
                    if(pack == null) return new COMMAND_ACK();
                    break;
                case 81:
                    if(pack == null) return new MANUAL_SETPOINT();
                    break;
                case 82:
                    if(pack == null) return new SET_ATTITUDE_TARGET();
                    break;
                case 83:
                    if(pack == null) return new ATTITUDE_TARGET();
                    break;
                case 84:
                    if(pack == null) return new SET_POSITION_TARGET_LOCAL_NED();
                    break;
                case 86:
                    if(pack == null) return new SET_POSITION_TARGET_GLOBAL_INT();
                    break;
                case 87:
                    if(pack == null) return new POSITION_TARGET_GLOBAL_INT();
                    break;
                case 89:
                    if(pack == null) return new LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
                    break;
                case 90:
                    if(pack == null) return new HIL_STATE();
                    break;
                case 91:
                    if(pack == null) return new HIL_CONTROLS();
                    break;
                case 92:
                    if(pack == null) return new HIL_RC_INPUTS_RAW();
                    break;
                case 93:
                    if(pack == null) return new HIL_ACTUATOR_CONTROLS();
                    break;
                case 100:
                    if(pack == null) return new OPTICAL_FLOW();
                    break;
                case 101:
                    if(pack == null) return new GLOBAL_VISION_POSITION_ESTIMATE();
                    break;
                case 150:
                    if(pack == null) return new SENSOR_OFFSETS();
                    ((OnReceive) on_SENSOR_OFFSETS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 151:
                    if(pack == null) return new SET_MAG_OFFSETS();
                    ((OnReceive) on_SET_MAG_OFFSETS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 152:
                    if(pack == null) return new MEMINFO();
                    ((OnReceive) on_MEMINFO).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 153:
                    if(pack == null) return new AP_ADC();
                    ((OnReceive) on_AP_ADC).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 154:
                    if(pack == null) return new DIGICAM_CONFIGURE();
                    ((OnReceive) on_DIGICAM_CONFIGURE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 155:
                    if(pack == null) return new DIGICAM_CONTROL();
                    ((OnReceive) on_DIGICAM_CONTROL).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 156:
                    if(pack == null) return new MOUNT_CONFIGURE();
                    ((OnReceive) on_MOUNT_CONFIGURE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 157:
                    if(pack == null) return new MOUNT_CONTROL();
                    ((OnReceive) on_MOUNT_CONTROL).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 158:
                    if(pack == null) return new MOUNT_STATUS();
                    ((OnReceive) on_MOUNT_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 160:
                    if(pack == null) return new FENCE_POINT();
                    ((OnReceive) on_FENCE_POINT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 161:
                    if(pack == null) return new FENCE_FETCH_POINT();
                    ((OnReceive) on_FENCE_FETCH_POINT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 162:
                    if(pack == null) return new FENCE_STATUS();
                    ((OnReceive) on_FENCE_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 163:
                    if(pack == null) return new AHRS();
                    ((OnReceive) on_AHRS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 164:
                    if(pack == null) return new SIMSTATE();
                    ((OnReceive) on_SIMSTATE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 165:
                    if(pack == null) return new HWSTATUS();
                    ((OnReceive) on_HWSTATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 166:
                    if(pack == null) return new RADIO();
                    ((OnReceive) on_RADIO).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 167:
                    if(pack == null) return new LIMITS_STATUS();
                    ((OnReceive) on_LIMITS_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 168:
                    if(pack == null) return new WIND();
                    ((OnReceive) on_WIND).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 169:
                    if(pack == null) return new DATA16();
                    ((OnReceive) on_DATA16).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 170:
                    if(pack == null) return new DATA32();
                    ((OnReceive) on_DATA32).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 171:
                    if(pack == null) return new DATA64();
                    ((OnReceive) on_DATA64).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 172:
                    if(pack == null) return new DATA96();
                    ((OnReceive) on_DATA96).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 173:
                    if(pack == null) return new RANGEFINDER();
                    ((OnReceive) on_RANGEFINDER).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 174:
                    if(pack == null) return new AIRSPEED_AUTOCAL();
                    ((OnReceive) on_AIRSPEED_AUTOCAL).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 175:
                    if(pack == null) return new RALLY_POINT();
                    ((OnReceive) on_RALLY_POINT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 176:
                    if(pack == null) return new RALLY_FETCH_POINT();
                    ((OnReceive) on_RALLY_FETCH_POINT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 177:
                    if(pack == null) return new COMPASSMOT_STATUS();
                    ((OnReceive) on_COMPASSMOT_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 178:
                    if(pack == null) return new AHRS2();
                    ((OnReceive) on_AHRS2).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 179:
                    if(pack == null) return new CAMERA_STATUS();
                    ((OnReceive) on_CAMERA_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 180:
                    if(pack == null) return new CAMERA_FEEDBACK();
                    ((OnReceive) on_CAMERA_FEEDBACK).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 181:
                    if(pack == null) return new BATTERY2();
                    ((OnReceive) on_BATTERY2).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 182:
                    if(pack == null) return new AHRS3();
                    ((OnReceive) on_AHRS3).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 183:
                    if(pack == null) return new AUTOPILOT_VERSION_REQUEST();
                    ((OnReceive) on_AUTOPILOT_VERSION_REQUEST).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 184:
                    if(pack == null) return new REMOTE_LOG_DATA_BLOCK();
                    ((OnReceive) on_REMOTE_LOG_DATA_BLOCK).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 185:
                    if(pack == null) return new REMOTE_LOG_BLOCK_STATUS();
                    ((OnReceive) on_REMOTE_LOG_BLOCK_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 186:
                    if(pack == null) return new LED_CONTROL();
                    ((OnReceive) on_LED_CONTROL).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 191:
                    if(pack == null) return new MAG_CAL_PROGRESS();
                    ((OnReceive) on_MAG_CAL_PROGRESS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 192:
                    if(pack == null) return new MAG_CAL_REPORT();
                    ((OnReceive) on_MAG_CAL_REPORT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 193:
                    if(pack == null) return new EKF_STATUS_REPORT();
                    ((OnReceive) on_EKF_STATUS_REPORT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 194:
                    if(pack == null) return new PID_TUNING();
                    ((OnReceive) on_PID_TUNING).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 200:
                    if(pack == null) return new GIMBAL_REPORT();
                    ((OnReceive) on_GIMBAL_REPORT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 201:
                    if(pack == null) return new GIMBAL_CONTROL();
                    ((OnReceive) on_GIMBAL_CONTROL).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 214:
                    if(pack == null) return new GIMBAL_TORQUE_CMD_REPORT();
                    ((OnReceive) on_GIMBAL_TORQUE_CMD_REPORT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 215:
                    if(pack == null) return new GOPRO_HEARTBEAT();
                    ((OnReceive) on_GOPRO_HEARTBEAT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 216:
                    if(pack == null) return new GOPRO_GET_REQUEST();
                    ((OnReceive) on_GOPRO_GET_REQUEST).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 217:
                    if(pack == null) return new GOPRO_GET_RESPONSE();
                    ((OnReceive) on_GOPRO_GET_RESPONSE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 218:
                    if(pack == null) return new GOPRO_SET_REQUEST();
                    ((OnReceive) on_GOPRO_SET_REQUEST).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 219:
                    if(pack == null) return new GOPRO_SET_RESPONSE();
                    ((OnReceive) on_GOPRO_SET_RESPONSE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 226:
                    if(pack == null) return new RPM();
                    ((OnReceive) on_RPM).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 264:
                    if(pack == null) return new FLIGHT_INFORMATION();
                    ((OnReceive) on_FLIGHT_INFORMATION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 265:
                    if(pack == null) return new MOUNT_ORIENTATION();
                    ((OnReceive) on_MOUNT_ORIENTATION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 266:
                    if(pack == null) return new LOGGING_DATA();
                    ((OnReceive) on_LOGGING_DATA).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 267:
                    if(pack == null) return new LOGGING_DATA_ACKED();
                    ((OnReceive) on_LOGGING_DATA_ACKED).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 268:
                    if(pack == null) return new LOGGING_ACK();
                    ((OnReceive) on_LOGGING_ACK).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 269:
                    if(pack == null) return new VIDEO_STREAM_INFORMATION();
                    ((OnReceive) on_VIDEO_STREAM_INFORMATION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 270:
                    if(pack == null) return new SET_VIDEO_STREAM_SETTINGS();
                    ((OnReceive) on_SET_VIDEO_STREAM_SETTINGS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 299:
                    if(pack == null) return new WIFI_CONFIG_AP();
                    ((OnReceive) on_WIFI_CONFIG_AP).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 300:
                    if(pack == null) return new PROTOCOL_VERSION();
                    ((OnReceive) on_PROTOCOL_VERSION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 310:
                    if(pack == null) return new UAVCAN_NODE_STATUS();
                    ((OnReceive) on_UAVCAN_NODE_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 311:
                    if(pack == null) return new UAVCAN_NODE_INFO();
                    ((OnReceive) on_UAVCAN_NODE_INFO).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 320:
                    if(pack == null) return new PARAM_EXT_REQUEST_READ();
                    ((OnReceive) on_PARAM_EXT_REQUEST_READ).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 321:
                    if(pack == null) return new PARAM_EXT_REQUEST_LIST();
                    ((OnReceive) on_PARAM_EXT_REQUEST_LIST).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 322:
                    if(pack == null) return new PARAM_EXT_VALUE();
                    ((OnReceive) on_PARAM_EXT_VALUE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 323:
                    if(pack == null) return new PARAM_EXT_SET();
                    ((OnReceive) on_PARAM_EXT_SET).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 324:
                    if(pack == null) return new PARAM_EXT_ACK();
                    ((OnReceive) on_PARAM_EXT_ACK).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 330:
                    if(pack == null) return new OBSTACLE_DISTANCE();
                    ((OnReceive) on_OBSTACLE_DISTANCE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 10001:
                    if(pack == null) return new UAVIONIX_ADSB_OUT_CFG();
                    ((OnReceive) on_UAVIONIX_ADSB_OUT_CFG).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 10002:
                    if(pack == null) return new UAVIONIX_ADSB_OUT_DYNAMIC();
                    ((OnReceive) on_UAVIONIX_ADSB_OUT_DYNAMIC).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 10003:
                    if(pack == null) return new UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();
                    ((OnReceive) on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 11000:
                    if(pack == null) return new DEVICE_OP_READ();
                    ((OnReceive) on_DEVICE_OP_READ).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 11001:
                    if(pack == null) return new DEVICE_OP_READ_REPLY();
                    ((OnReceive) on_DEVICE_OP_READ_REPLY).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 11002:
                    if(pack == null) return new DEVICE_OP_WRITE();
                    ((OnReceive) on_DEVICE_OP_WRITE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 11003:
                    if(pack == null) return new DEVICE_OP_WRITE_REPLY();
                    ((OnReceive) on_DEVICE_OP_WRITE_REPLY).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 11010:
                    if(pack == null) return new ADAP_TUNING();
                    ((OnReceive) on_ADAP_TUNING).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 11011:
                    if(pack == null) return new VISION_POSITION_DELTA();
                    ((OnReceive) on_VISION_POSITION_DELTA).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
            }
            return null;
        }
        static final byte[] buff = new byte[1024];
        static void transmission(java.io.InputStream src, java.io.OutputStream dst, Channel dst_ch)
        {
            try
            {
                if(src instanceof AdvancedInputStream && !(dst instanceof AdvancedOutputStream))
                {
                    for(int bytes; 0 < (bytes = src.read(buff, 0, buff.length));) TestChannel.instance.outputStreamAdvanced.write(buff, 0, bytes);
                    for(int bytes; 0 < (bytes = TestChannel.instance.inputStream.read(buff, 0, buff.length));) dst.write(buff, 0, bytes);
                }
                else if(!(src instanceof AdvancedInputStream) && dst instanceof AdvancedOutputStream)
                {
                    for(int bytes; 0 < (bytes = src.read(buff, 0, buff.length));) TestChannel.instance.outputStream.write(buff, 0, bytes);
                    for(int bytes; 0 < (bytes = TestChannel.instance.inputStreamAdvanced.read(buff, 0, buff.length));) dst.write(buff, 0, bytes);
                }
                else
                    for(int bytes; 0 < (bytes = src.read(buff, 0, buff.length));) dst.write(buff, 0, bytes);
                processReceived(dst_ch);
            }
            catch(IOException e)
            {
                e.printStackTrace();
                assert(false);
            }
        }
    }

    public static void main(String[] args)
    {
        final Bounds.Inside PH = new Bounds.Inside();
        CommunicationChannel.instance.on_HEARTBEAT.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_SMACCMPILOT);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_STANDBY);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_TRICOPTER);
            assert(pack.custom_mode_GET() == 2925959887L);
            assert(pack.mavlink_version_GET() == (char)250);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.system_status_SET(MAV_STATE.MAV_STATE_STANDBY) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_TRICOPTER) ;
        p0.mavlink_version_SET((char)250) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_SMACCMPILOT) ;
        p0.custom_mode_SET(2925959887L) ;
        p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.current_battery_GET() == (short)7315);
            assert(pack.drop_rate_comm_GET() == (char)2416);
            assert(pack.errors_count1_GET() == (char)11113);
            assert(pack.load_GET() == (char)44004);
            assert(pack.onboard_control_sensors_health_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL);
            assert(pack.errors_count2_GET() == (char)63971);
            assert(pack.onboard_control_sensors_present_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN);
            assert(pack.onboard_control_sensors_enabled_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION);
            assert(pack.errors_comm_GET() == (char)3199);
            assert(pack.voltage_battery_GET() == (char)1444);
            assert(pack.battery_remaining_GET() == (byte)22);
            assert(pack.errors_count4_GET() == (char)34406);
            assert(pack.errors_count3_GET() == (char)40314);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.errors_count1_SET((char)11113) ;
        p1.current_battery_SET((short)7315) ;
        p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION) ;
        p1.errors_count3_SET((char)40314) ;
        p1.drop_rate_comm_SET((char)2416) ;
        p1.errors_comm_SET((char)3199) ;
        p1.errors_count4_SET((char)34406) ;
        p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN) ;
        p1.voltage_battery_SET((char)1444) ;
        p1.load_SET((char)44004) ;
        p1.errors_count2_SET((char)63971) ;
        p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL) ;
        p1.battery_remaining_SET((byte)22) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 4501359953452192122L);
            assert(pack.time_boot_ms_GET() == 1122082546L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(1122082546L) ;
        p2.time_unix_usec_SET(4501359953452192122L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == -3.0821527E38F);
            assert(pack.vy_GET() == 2.715262E38F);
            assert(pack.z_GET() == 2.002221E38F);
            assert(pack.time_boot_ms_GET() == 2181708535L);
            assert(pack.vz_GET() == -9.598931E37F);
            assert(pack.x_GET() == 3.0574019E37F);
            assert(pack.afy_GET() == -1.6544462E38F);
            assert(pack.afx_GET() == -1.5524818E38F);
            assert(pack.yaw_GET() == -3.1037328E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.type_mask_GET() == (char)60048);
            assert(pack.vx_GET() == 8.2941496E37F);
            assert(pack.afz_GET() == 3.1292582E38F);
            assert(pack.y_GET() == -1.0305669E38F);
        });
        POSITION_TARGET_LOCAL_NED p3 = new POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.yaw_rate_SET(-3.0821527E38F) ;
        p3.afy_SET(-1.6544462E38F) ;
        p3.yaw_SET(-3.1037328E38F) ;
        p3.vx_SET(8.2941496E37F) ;
        p3.z_SET(2.002221E38F) ;
        p3.afx_SET(-1.5524818E38F) ;
        p3.vy_SET(2.715262E38F) ;
        p3.x_SET(3.0574019E37F) ;
        p3.y_SET(-1.0305669E38F) ;
        p3.time_boot_ms_SET(2181708535L) ;
        p3.afz_SET(3.1292582E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p3.vz_SET(-9.598931E37F) ;
        p3.type_mask_SET((char)60048) ;
        TestChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 2385297053L);
            assert(pack.target_component_GET() == (char)214);
            assert(pack.time_usec_GET() == 670114174296676520L);
            assert(pack.target_system_GET() == (char)181);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.seq_SET(2385297053L) ;
        p4.target_system_SET((char)181) ;
        p4.time_usec_SET(670114174296676520L) ;
        p4.target_component_SET((char)214) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.control_request_GET() == (char)216);
            assert(pack.version_GET() == (char)12);
            assert(pack.passkey_LEN(ph) == 1);
            assert(pack.passkey_TRY(ph).equals("h"));
            assert(pack.target_system_GET() == (char)111);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.version_SET((char)12) ;
        p5.target_system_SET((char)111) ;
        p5.passkey_SET("h", PH) ;
        p5.control_request_SET((char)216) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.control_request_GET() == (char)199);
            assert(pack.gcs_system_id_GET() == (char)195);
            assert(pack.ack_GET() == (char)179);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.gcs_system_id_SET((char)195) ;
        p6.ack_SET((char)179) ;
        p6.control_request_SET((char)199) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 10);
            assert(pack.key_TRY(ph).equals("xqzFfbrrJq"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("xqzFfbrrJq", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_TEST_ARMED);
            assert(pack.target_system_GET() == (char)74);
            assert(pack.custom_mode_GET() == 65841590L);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_TEST_ARMED) ;
        p11.target_system_SET((char)74) ;
        p11.custom_mode_SET(65841590L) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)76);
            assert(pack.param_index_GET() == (short)30232);
            assert(pack.target_component_GET() == (char)209);
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("kdigRbis"));
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_component_SET((char)209) ;
        p20.param_id_SET("kdigRbis", PH) ;
        p20.param_index_SET((short)30232) ;
        p20.target_system_SET((char)76) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)160);
            assert(pack.target_component_GET() == (char)152);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)160) ;
        p21.target_component_SET((char)152) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (char)13854);
            assert(pack.param_id_LEN(ph) == 13);
            assert(pack.param_id_TRY(ph).equals("Bidjlmhsuxrmd"));
            assert(pack.param_count_GET() == (char)3620);
            assert(pack.param_value_GET() == 5.7618036E37F);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32);
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_id_SET("Bidjlmhsuxrmd", PH) ;
        p22.param_count_SET((char)3620) ;
        p22.param_value_SET(5.7618036E37F) ;
        p22.param_index_SET((char)13854) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_value_GET() == 1.0720256E38F);
            assert(pack.target_system_GET() == (char)28);
            assert(pack.param_id_LEN(ph) == 3);
            assert(pack.param_id_TRY(ph).equals("dmw"));
            assert(pack.target_component_GET() == (char)255);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.param_id_SET("dmw", PH) ;
        p23.target_system_SET((char)28) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32) ;
        p23.param_value_SET(1.0720256E38F) ;
        p23.target_component_SET((char)255) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1910216228);
            assert(pack.v_acc_TRY(ph) == 3644735526L);
            assert(pack.epv_GET() == (char)18081);
            assert(pack.lon_GET() == -1533177193);
            assert(pack.vel_acc_TRY(ph) == 2826373124L);
            assert(pack.vel_GET() == (char)51276);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
            assert(pack.alt_ellipsoid_TRY(ph) == 967692172);
            assert(pack.cog_GET() == (char)1490);
            assert(pack.time_usec_GET() == 8533284313693672425L);
            assert(pack.satellites_visible_GET() == (char)200);
            assert(pack.hdg_acc_TRY(ph) == 1875573758L);
            assert(pack.alt_GET() == -1891583169);
            assert(pack.h_acc_TRY(ph) == 3094084162L);
            assert(pack.eph_GET() == (char)37442);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.hdg_acc_SET(1875573758L, PH) ;
        p24.time_usec_SET(8533284313693672425L) ;
        p24.vel_acc_SET(2826373124L, PH) ;
        p24.cog_SET((char)1490) ;
        p24.satellites_visible_SET((char)200) ;
        p24.vel_SET((char)51276) ;
        p24.lat_SET(1910216228) ;
        p24.eph_SET((char)37442) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP) ;
        p24.v_acc_SET(3644735526L, PH) ;
        p24.alt_ellipsoid_SET(967692172, PH) ;
        p24.lon_SET(-1533177193) ;
        p24.epv_SET((char)18081) ;
        p24.alt_SET(-1891583169) ;
        p24.h_acc_SET(3094084162L, PH) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)7, (char)43, (char)254, (char)41, (char)174, (char)198, (char)120, (char)112, (char)208, (char)59, (char)30, (char)156, (char)145, (char)2, (char)32, (char)139, (char)81, (char)255, (char)1, (char)158}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)146, (char)141, (char)120, (char)173, (char)161, (char)207, (char)244, (char)118, (char)82, (char)51, (char)208, (char)184, (char)99, (char)39, (char)159, (char)202, (char)197, (char)228, (char)117, (char)107}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)124, (char)51, (char)24, (char)238, (char)207, (char)209, (char)175, (char)143, (char)153, (char)4, (char)144, (char)48, (char)197, (char)63, (char)187, (char)113, (char)88, (char)185, (char)128, (char)4}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)174, (char)73, (char)140, (char)138, (char)187, (char)184, (char)229, (char)121, (char)86, (char)165, (char)36, (char)164, (char)213, (char)155, (char)101, (char)127, (char)227, (char)93, (char)139, (char)27}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)206, (char)93, (char)188, (char)232, (char)25, (char)75, (char)1, (char)49, (char)67, (char)134, (char)116, (char)203, (char)203, (char)177, (char)137, (char)244, (char)205, (char)64, (char)33, (char)57}));
            assert(pack.satellites_visible_GET() == (char)52);
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_snr_SET(new char[] {(char)7, (char)43, (char)254, (char)41, (char)174, (char)198, (char)120, (char)112, (char)208, (char)59, (char)30, (char)156, (char)145, (char)2, (char)32, (char)139, (char)81, (char)255, (char)1, (char)158}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)124, (char)51, (char)24, (char)238, (char)207, (char)209, (char)175, (char)143, (char)153, (char)4, (char)144, (char)48, (char)197, (char)63, (char)187, (char)113, (char)88, (char)185, (char)128, (char)4}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)174, (char)73, (char)140, (char)138, (char)187, (char)184, (char)229, (char)121, (char)86, (char)165, (char)36, (char)164, (char)213, (char)155, (char)101, (char)127, (char)227, (char)93, (char)139, (char)27}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)146, (char)141, (char)120, (char)173, (char)161, (char)207, (char)244, (char)118, (char)82, (char)51, (char)208, (char)184, (char)99, (char)39, (char)159, (char)202, (char)197, (char)228, (char)117, (char)107}, 0) ;
        p25.satellites_visible_SET((char)52) ;
        p25.satellite_used_SET(new char[] {(char)206, (char)93, (char)188, (char)232, (char)25, (char)75, (char)1, (char)49, (char)67, (char)134, (char)116, (char)203, (char)203, (char)177, (char)137, (char)244, (char)205, (char)64, (char)33, (char)57}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == (short) -11935);
            assert(pack.zmag_GET() == (short)25430);
            assert(pack.xgyro_GET() == (short) -31354);
            assert(pack.time_boot_ms_GET() == 1534584924L);
            assert(pack.zacc_GET() == (short) -2009);
            assert(pack.xmag_GET() == (short)11786);
            assert(pack.ymag_GET() == (short) -4697);
            assert(pack.ygyro_GET() == (short)22838);
            assert(pack.xacc_GET() == (short)18285);
            assert(pack.yacc_GET() == (short)18211);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.yacc_SET((short)18211) ;
        p26.time_boot_ms_SET(1534584924L) ;
        p26.zgyro_SET((short) -11935) ;
        p26.zacc_SET((short) -2009) ;
        p26.ymag_SET((short) -4697) ;
        p26.zmag_SET((short)25430) ;
        p26.ygyro_SET((short)22838) ;
        p26.xgyro_SET((short) -31354) ;
        p26.xacc_SET((short)18285) ;
        p26.xmag_SET((short)11786) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short) -7488);
            assert(pack.zgyro_GET() == (short) -21873);
            assert(pack.yacc_GET() == (short) -24848);
            assert(pack.ygyro_GET() == (short) -21569);
            assert(pack.zmag_GET() == (short)26969);
            assert(pack.time_usec_GET() == 5226675872334656562L);
            assert(pack.ymag_GET() == (short) -13543);
            assert(pack.zacc_GET() == (short)12403);
            assert(pack.xgyro_GET() == (short)19764);
            assert(pack.xmag_GET() == (short) -4920);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.ymag_SET((short) -13543) ;
        p27.zacc_SET((short)12403) ;
        p27.time_usec_SET(5226675872334656562L) ;
        p27.zgyro_SET((short) -21873) ;
        p27.yacc_SET((short) -24848) ;
        p27.ygyro_SET((short) -21569) ;
        p27.xacc_SET((short) -7488) ;
        p27.xgyro_SET((short)19764) ;
        p27.zmag_SET((short)26969) ;
        p27.xmag_SET((short) -4920) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff1_GET() == (short)8285);
            assert(pack.press_diff2_GET() == (short) -22101);
            assert(pack.temperature_GET() == (short) -17040);
            assert(pack.time_usec_GET() == 9115779043990629556L);
            assert(pack.press_abs_GET() == (short) -15132);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_diff2_SET((short) -22101) ;
        p28.press_diff1_SET((short)8285) ;
        p28.temperature_SET((short) -17040) ;
        p28.press_abs_SET((short) -15132) ;
        p28.time_usec_SET(9115779043990629556L) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -2.538136E38F);
            assert(pack.press_diff_GET() == 1.6239564E38F);
            assert(pack.temperature_GET() == (short)509);
            assert(pack.time_boot_ms_GET() == 1015591397L);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.temperature_SET((short)509) ;
        p29.press_diff_SET(1.6239564E38F) ;
        p29.time_boot_ms_SET(1015591397L) ;
        p29.press_abs_SET(-2.538136E38F) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == 3.084658E38F);
            assert(pack.yawspeed_GET() == 2.0097629E38F);
            assert(pack.time_boot_ms_GET() == 3243921296L);
            assert(pack.yaw_GET() == 2.1350041E38F);
            assert(pack.roll_GET() == 1.4225869E38F);
            assert(pack.pitch_GET() == -2.9359741E38F);
            assert(pack.pitchspeed_GET() == 2.7215404E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.yawspeed_SET(2.0097629E38F) ;
        p30.pitch_SET(-2.9359741E38F) ;
        p30.pitchspeed_SET(2.7215404E38F) ;
        p30.rollspeed_SET(3.084658E38F) ;
        p30.time_boot_ms_SET(3243921296L) ;
        p30.yaw_SET(2.1350041E38F) ;
        p30.roll_SET(1.4225869E38F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q4_GET() == 2.5831216E38F);
            assert(pack.q2_GET() == -2.5680796E38F);
            assert(pack.yawspeed_GET() == 2.77611E38F);
            assert(pack.q3_GET() == -1.0467625E36F);
            assert(pack.time_boot_ms_GET() == 1146626273L);
            assert(pack.q1_GET() == -2.776484E38F);
            assert(pack.pitchspeed_GET() == -1.2361285E38F);
            assert(pack.rollspeed_GET() == -1.6919935E38F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.time_boot_ms_SET(1146626273L) ;
        p31.yawspeed_SET(2.77611E38F) ;
        p31.q2_SET(-2.5680796E38F) ;
        p31.rollspeed_SET(-1.6919935E38F) ;
        p31.q4_SET(2.5831216E38F) ;
        p31.q1_SET(-2.776484E38F) ;
        p31.q3_SET(-1.0467625E36F) ;
        p31.pitchspeed_SET(-1.2361285E38F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == 2.6306046E38F);
            assert(pack.x_GET() == -3.013701E38F);
            assert(pack.z_GET() == 3.2288702E38F);
            assert(pack.y_GET() == -3.2920564E38F);
            assert(pack.vy_GET() == -5.770422E37F);
            assert(pack.vz_GET() == 4.087673E37F);
            assert(pack.time_boot_ms_GET() == 4128175920L);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.y_SET(-3.2920564E38F) ;
        p32.vy_SET(-5.770422E37F) ;
        p32.x_SET(-3.013701E38F) ;
        p32.time_boot_ms_SET(4128175920L) ;
        p32.vz_SET(4.087673E37F) ;
        p32.z_SET(3.2288702E38F) ;
        p32.vx_SET(2.6306046E38F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.hdg_GET() == (char)55510);
            assert(pack.vy_GET() == (short) -12988);
            assert(pack.vx_GET() == (short) -2035);
            assert(pack.lon_GET() == -2065602380);
            assert(pack.relative_alt_GET() == 535015889);
            assert(pack.lat_GET() == -2108666995);
            assert(pack.time_boot_ms_GET() == 3628899234L);
            assert(pack.alt_GET() == -1925872771);
            assert(pack.vz_GET() == (short)9274);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.vz_SET((short)9274) ;
        p33.relative_alt_SET(535015889) ;
        p33.vx_SET((short) -2035) ;
        p33.lat_SET(-2108666995) ;
        p33.hdg_SET((char)55510) ;
        p33.alt_SET(-1925872771) ;
        p33.vy_SET((short) -12988) ;
        p33.lon_SET(-2065602380) ;
        p33.time_boot_ms_SET(3628899234L) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan6_scaled_GET() == (short) -17326);
            assert(pack.chan3_scaled_GET() == (short)22782);
            assert(pack.port_GET() == (char)9);
            assert(pack.chan4_scaled_GET() == (short) -21749);
            assert(pack.chan5_scaled_GET() == (short) -22220);
            assert(pack.chan1_scaled_GET() == (short) -1314);
            assert(pack.rssi_GET() == (char)28);
            assert(pack.chan7_scaled_GET() == (short)28295);
            assert(pack.chan2_scaled_GET() == (short) -21209);
            assert(pack.chan8_scaled_GET() == (short)9659);
            assert(pack.time_boot_ms_GET() == 3212474950L);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.rssi_SET((char)28) ;
        p34.port_SET((char)9) ;
        p34.chan7_scaled_SET((short)28295) ;
        p34.chan6_scaled_SET((short) -17326) ;
        p34.chan8_scaled_SET((short)9659) ;
        p34.chan5_scaled_SET((short) -22220) ;
        p34.chan3_scaled_SET((short)22782) ;
        p34.time_boot_ms_SET(3212474950L) ;
        p34.chan2_scaled_SET((short) -21209) ;
        p34.chan1_scaled_SET((short) -1314) ;
        p34.chan4_scaled_SET((short) -21749) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan4_raw_GET() == (char)18458);
            assert(pack.time_boot_ms_GET() == 115934759L);
            assert(pack.rssi_GET() == (char)7);
            assert(pack.chan7_raw_GET() == (char)40482);
            assert(pack.chan6_raw_GET() == (char)1884);
            assert(pack.chan3_raw_GET() == (char)63002);
            assert(pack.chan5_raw_GET() == (char)55020);
            assert(pack.chan1_raw_GET() == (char)62538);
            assert(pack.chan2_raw_GET() == (char)33906);
            assert(pack.port_GET() == (char)203);
            assert(pack.chan8_raw_GET() == (char)34326);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan5_raw_SET((char)55020) ;
        p35.chan2_raw_SET((char)33906) ;
        p35.chan3_raw_SET((char)63002) ;
        p35.chan7_raw_SET((char)40482) ;
        p35.chan4_raw_SET((char)18458) ;
        p35.chan8_raw_SET((char)34326) ;
        p35.rssi_SET((char)7) ;
        p35.chan6_raw_SET((char)1884) ;
        p35.port_SET((char)203) ;
        p35.time_boot_ms_SET(115934759L) ;
        p35.chan1_raw_SET((char)62538) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 333909682L);
            assert(pack.servo11_raw_TRY(ph) == (char)10866);
            assert(pack.servo10_raw_TRY(ph) == (char)10970);
            assert(pack.servo16_raw_TRY(ph) == (char)16719);
            assert(pack.servo9_raw_TRY(ph) == (char)35275);
            assert(pack.servo6_raw_GET() == (char)17314);
            assert(pack.servo8_raw_GET() == (char)49994);
            assert(pack.servo1_raw_GET() == (char)10761);
            assert(pack.servo3_raw_GET() == (char)6133);
            assert(pack.servo7_raw_GET() == (char)31860);
            assert(pack.servo13_raw_TRY(ph) == (char)58664);
            assert(pack.port_GET() == (char)179);
            assert(pack.servo12_raw_TRY(ph) == (char)49688);
            assert(pack.servo4_raw_GET() == (char)33205);
            assert(pack.servo15_raw_TRY(ph) == (char)14741);
            assert(pack.servo2_raw_GET() == (char)2173);
            assert(pack.servo14_raw_TRY(ph) == (char)48296);
            assert(pack.servo5_raw_GET() == (char)39626);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.port_SET((char)179) ;
        p36.servo2_raw_SET((char)2173) ;
        p36.servo6_raw_SET((char)17314) ;
        p36.servo7_raw_SET((char)31860) ;
        p36.servo13_raw_SET((char)58664, PH) ;
        p36.servo11_raw_SET((char)10866, PH) ;
        p36.servo8_raw_SET((char)49994) ;
        p36.servo3_raw_SET((char)6133) ;
        p36.servo5_raw_SET((char)39626) ;
        p36.servo1_raw_SET((char)10761) ;
        p36.servo15_raw_SET((char)14741, PH) ;
        p36.servo16_raw_SET((char)16719, PH) ;
        p36.servo9_raw_SET((char)35275, PH) ;
        p36.servo12_raw_SET((char)49688, PH) ;
        p36.servo14_raw_SET((char)48296, PH) ;
        p36.servo4_raw_SET((char)33205) ;
        p36.time_usec_SET(333909682L) ;
        p36.servo10_raw_SET((char)10970, PH) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_index_GET() == (short) -4738);
            assert(pack.start_index_GET() == (short) -29113);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)118);
            assert(pack.target_component_GET() == (char)225);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.end_index_SET((short) -4738) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p37.start_index_SET((short) -29113) ;
        p37.target_component_SET((char)225) ;
        p37.target_system_SET((char)118) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_index_GET() == (short) -11049);
            assert(pack.target_component_GET() == (char)104);
            assert(pack.start_index_GET() == (short) -22205);
            assert(pack.target_system_GET() == (char)246);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_system_SET((char)246) ;
        p38.target_component_SET((char)104) ;
        p38.end_index_SET((short) -11049) ;
        p38.start_index_SET((short) -22205) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)100);
            assert(pack.target_component_GET() == (char)57);
            assert(pack.y_GET() == 2.8076678E38F);
            assert(pack.param3_GET() == -3.1967394E38F);
            assert(pack.param2_GET() == -3.0648653E37F);
            assert(pack.current_GET() == (char)18);
            assert(pack.autocontinue_GET() == (char)34);
            assert(pack.param1_GET() == 1.0594282E38F);
            assert(pack.z_GET() == -2.8025086E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION);
            assert(pack.x_GET() == -2.8128305E38F);
            assert(pack.param4_GET() == 2.5199395E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.seq_GET() == (char)41011);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.param1_SET(1.0594282E38F) ;
        p39.target_component_SET((char)57) ;
        p39.x_SET(-2.8128305E38F) ;
        p39.current_SET((char)18) ;
        p39.autocontinue_SET((char)34) ;
        p39.target_system_SET((char)100) ;
        p39.param3_SET(-3.1967394E38F) ;
        p39.y_SET(2.8076678E38F) ;
        p39.param4_SET(2.5199395E38F) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p39.param2_SET(-3.0648653E37F) ;
        p39.z_SET(-2.8025086E38F) ;
        p39.command_SET(MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION) ;
        p39.seq_SET((char)41011) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)33102);
            assert(pack.target_system_GET() == (char)82);
            assert(pack.target_component_GET() == (char)240);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_system_SET((char)82) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p40.target_component_SET((char)240) ;
        p40.seq_SET((char)33102) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)14167);
            assert(pack.target_component_GET() == (char)114);
            assert(pack.target_system_GET() == (char)157);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_component_SET((char)114) ;
        p41.seq_SET((char)14167) ;
        p41.target_system_SET((char)157) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)8520);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)8520) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)183);
            assert(pack.target_component_GET() == (char)114);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_component_SET((char)114) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p43.target_system_SET((char)183) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)4270);
            assert(pack.target_system_GET() == (char)236);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)166);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.target_system_SET((char)236) ;
        p44.count_SET((char)4270) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p44.target_component_SET((char)166) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)142);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)147);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_component_SET((char)142) ;
        p45.target_system_SET((char)147) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)42540);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)42540) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)229);
            assert(pack.target_component_GET() == (char)52);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE) ;
        p47.target_component_SET((char)52) ;
        p47.target_system_SET((char)229) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == -511763293);
            assert(pack.time_usec_TRY(ph) == 5392743024890141820L);
            assert(pack.altitude_GET() == -689421950);
            assert(pack.latitude_GET() == 410563686);
            assert(pack.target_system_GET() == (char)176);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.altitude_SET(-689421950) ;
        p48.longitude_SET(-511763293) ;
        p48.latitude_SET(410563686) ;
        p48.target_system_SET((char)176) ;
        p48.time_usec_SET(5392743024890141820L, PH) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == 347833669);
            assert(pack.longitude_GET() == 1041317166);
            assert(pack.time_usec_TRY(ph) == 8138113914488735376L);
            assert(pack.latitude_GET() == 982828748);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.latitude_SET(982828748) ;
        p49.altitude_SET(347833669) ;
        p49.time_usec_SET(8138113914488735376L, PH) ;
        p49.longitude_SET(1041317166) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.parameter_rc_channel_index_GET() == (char)82);
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("wlWXvFMdR"));
            assert(pack.param_value0_GET() == -1.2138232E38F);
            assert(pack.target_system_GET() == (char)62);
            assert(pack.param_value_max_GET() == 9.247445E37F);
            assert(pack.target_component_GET() == (char)238);
            assert(pack.param_index_GET() == (short) -13227);
            assert(pack.param_value_min_GET() == -2.4232782E38F);
            assert(pack.scale_GET() == -2.1404364E38F);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.target_component_SET((char)238) ;
        p50.parameter_rc_channel_index_SET((char)82) ;
        p50.param_index_SET((short) -13227) ;
        p50.param_id_SET("wlWXvFMdR", PH) ;
        p50.param_value_max_SET(9.247445E37F) ;
        p50.param_value0_SET(-1.2138232E38F) ;
        p50.param_value_min_SET(-2.4232782E38F) ;
        p50.target_system_SET((char)62) ;
        p50.scale_SET(-2.1404364E38F) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)2291);
            assert(pack.target_component_GET() == (char)80);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)131);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.seq_SET((char)2291) ;
        p51.target_component_SET((char)80) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p51.target_system_SET((char)131) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)73);
            assert(pack.target_component_GET() == (char)14);
            assert(pack.p1y_GET() == 1.3382791E37F);
            assert(pack.p2y_GET() == 4.6023386E35F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.p2z_GET() == -9.6758034E36F);
            assert(pack.p1x_GET() == 1.957389E38F);
            assert(pack.p1z_GET() == 6.292585E36F);
            assert(pack.p2x_GET() == -3.1231408E38F);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p54.p1x_SET(1.957389E38F) ;
        p54.target_component_SET((char)14) ;
        p54.p1z_SET(6.292585E36F) ;
        p54.p2x_SET(-3.1231408E38F) ;
        p54.p2y_SET(4.6023386E35F) ;
        p54.target_system_SET((char)73) ;
        p54.p1y_SET(1.3382791E37F) ;
        p54.p2z_SET(-9.6758034E36F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2y_GET() == -1.6382523E38F);
            assert(pack.p2x_GET() == -6.1275345E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.p1x_GET() == -2.1173777E38F);
            assert(pack.p1y_GET() == -2.5278308E38F);
            assert(pack.p1z_GET() == -2.750104E38F);
            assert(pack.p2z_GET() == -1.697833E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p2x_SET(-6.1275345E37F) ;
        p55.p1x_SET(-2.1173777E38F) ;
        p55.p1y_SET(-2.5278308E38F) ;
        p55.p2z_SET(-1.697833E38F) ;
        p55.p2y_SET(-1.6382523E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p55.p1z_SET(-2.750104E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == -4.8680805E37F);
            assert(pack.yawspeed_GET() == 1.1621933E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.1433073E38F, -2.3014223E38F, 1.9525304E38F, -1.29396996E36F}));
            assert(pack.pitchspeed_GET() == 1.1013982E38F);
            assert(pack.time_usec_GET() == 4537007595579218652L);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.194449E38F, 9.060171E37F, 3.519729E37F, -5.1813515E37F, 8.0941534E37F, -4.8620683E37F, 2.143703E38F, 3.2307082E38F, -8.4947964E37F}));
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.time_usec_SET(4537007595579218652L) ;
        p61.rollspeed_SET(-4.8680805E37F) ;
        p61.pitchspeed_SET(1.1013982E38F) ;
        p61.q_SET(new float[] {-1.1433073E38F, -2.3014223E38F, 1.9525304E38F, -1.29396996E36F}, 0) ;
        p61.covariance_SET(new float[] {-2.194449E38F, 9.060171E37F, 3.519729E37F, -5.1813515E37F, 8.0941534E37F, -4.8620683E37F, 2.143703E38F, 3.2307082E38F, -8.4947964E37F}, 0) ;
        p61.yawspeed_SET(1.1621933E38F) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.nav_pitch_GET() == -4.416023E37F);
            assert(pack.target_bearing_GET() == (short) -31227);
            assert(pack.alt_error_GET() == 4.2643487E37F);
            assert(pack.wp_dist_GET() == (char)57175);
            assert(pack.nav_roll_GET() == 2.6730993E38F);
            assert(pack.nav_bearing_GET() == (short)10336);
            assert(pack.aspd_error_GET() == -6.0536345E37F);
            assert(pack.xtrack_error_GET() == 4.2312276E37F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.xtrack_error_SET(4.2312276E37F) ;
        p62.nav_roll_SET(2.6730993E38F) ;
        p62.target_bearing_SET((short) -31227) ;
        p62.alt_error_SET(4.2643487E37F) ;
        p62.wp_dist_SET((char)57175) ;
        p62.aspd_error_SET(-6.0536345E37F) ;
        p62.nav_pitch_SET(-4.416023E37F) ;
        p62.nav_bearing_SET((short)10336) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            assert(pack.lat_GET() == -563790638);
            assert(pack.vx_GET() == 1.802141E38F);
            assert(pack.vz_GET() == 2.3179563E38F);
            assert(pack.lon_GET() == -1027702959);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {3.0232927E37F, -1.3004483E38F, -2.6115691E38F, -1.2220924E38F, 4.7594865E37F, -1.2526158E38F, -1.5924217E38F, -8.898009E37F, 4.0837171E37F, -3.0073173E38F, 1.976905E38F, 1.1461712E38F, -2.874318E38F, 1.951663E38F, 1.3829762E38F, 1.6824379E38F, -1.9880682E38F, 3.2559234E38F, -3.1351512E38F, 1.5592087E38F, 5.252286E37F, 2.2956627E38F, 3.6697161E37F, 1.0077396E38F, 9.688003E37F, -1.2617635E38F, -2.1013704E38F, -8.955861E37F, -1.0091517E38F, 3.1665273E38F, -2.8261418E38F, -2.3298653E38F, -1.7336392E38F, -2.8431427E38F, 3.4008697E38F, -1.968948E38F}));
            assert(pack.time_usec_GET() == 4687793656102191961L);
            assert(pack.relative_alt_GET() == -62879934);
            assert(pack.alt_GET() == 1886056379);
            assert(pack.vy_GET() == 2.7114454E38F);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.vy_SET(2.7114454E38F) ;
        p63.vx_SET(1.802141E38F) ;
        p63.lon_SET(-1027702959) ;
        p63.vz_SET(2.3179563E38F) ;
        p63.covariance_SET(new float[] {3.0232927E37F, -1.3004483E38F, -2.6115691E38F, -1.2220924E38F, 4.7594865E37F, -1.2526158E38F, -1.5924217E38F, -8.898009E37F, 4.0837171E37F, -3.0073173E38F, 1.976905E38F, 1.1461712E38F, -2.874318E38F, 1.951663E38F, 1.3829762E38F, 1.6824379E38F, -1.9880682E38F, 3.2559234E38F, -3.1351512E38F, 1.5592087E38F, 5.252286E37F, 2.2956627E38F, 3.6697161E37F, 1.0077396E38F, 9.688003E37F, -1.2617635E38F, -2.1013704E38F, -8.955861E37F, -1.0091517E38F, 3.1665273E38F, -2.8261418E38F, -2.3298653E38F, -1.7336392E38F, -2.8431427E38F, 3.4008697E38F, -1.968948E38F}, 0) ;
        p63.lat_SET(-563790638) ;
        p63.relative_alt_SET(-62879934) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        p63.alt_SET(1886056379) ;
        p63.time_usec_SET(4687793656102191961L) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
            assert(pack.z_GET() == -1.044854E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.9256528E38F, -2.4657199E37F, 8.670032E37F, -6.058364E37F, 1.3484015E38F, 1.5582498E38F, 2.7666006E38F, 3.0570552E38F, -1.2718012E38F, -2.7449436E38F, -1.0612606E38F, -3.3029255E38F, -2.5812922E38F, 3.2243656E38F, 2.9716677E38F, -2.0599188E38F, 1.2310288E38F, -1.7073183E38F, 2.4816465E38F, 3.4956515E35F, 1.3845244E38F, -1.5732748E37F, -2.7842341E38F, -3.2214578E38F, 1.538737E38F, -1.7522292E38F, -2.5706611E38F, -2.2381146E38F, 6.8050684E37F, -1.4388168E38F, 1.2227514E38F, 2.3793528E37F, 2.2616465E38F, -1.667789E38F, 2.0728548E37F, 2.0501829E38F, -4.5996274E36F, -2.9965962E38F, -2.8228581E38F, 1.6526705E38F, 1.9925047E37F, 2.1784754E38F, -3.275051E38F, -6.790484E37F, -2.9078305E37F}));
            assert(pack.vx_GET() == -7.136746E37F);
            assert(pack.ax_GET() == 2.9462324E38F);
            assert(pack.y_GET() == 6.5606177E37F);
            assert(pack.vy_GET() == -2.1250655E38F);
            assert(pack.x_GET() == 1.3964801E38F);
            assert(pack.vz_GET() == -3.0222259E38F);
            assert(pack.ay_GET() == 2.7953065E38F);
            assert(pack.az_GET() == -7.6604775E37F);
            assert(pack.time_usec_GET() == 752725980776036359L);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.x_SET(1.3964801E38F) ;
        p64.vx_SET(-7.136746E37F) ;
        p64.time_usec_SET(752725980776036359L) ;
        p64.vy_SET(-2.1250655E38F) ;
        p64.ay_SET(2.7953065E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS) ;
        p64.z_SET(-1.044854E38F) ;
        p64.vz_SET(-3.0222259E38F) ;
        p64.covariance_SET(new float[] {-2.9256528E38F, -2.4657199E37F, 8.670032E37F, -6.058364E37F, 1.3484015E38F, 1.5582498E38F, 2.7666006E38F, 3.0570552E38F, -1.2718012E38F, -2.7449436E38F, -1.0612606E38F, -3.3029255E38F, -2.5812922E38F, 3.2243656E38F, 2.9716677E38F, -2.0599188E38F, 1.2310288E38F, -1.7073183E38F, 2.4816465E38F, 3.4956515E35F, 1.3845244E38F, -1.5732748E37F, -2.7842341E38F, -3.2214578E38F, 1.538737E38F, -1.7522292E38F, -2.5706611E38F, -2.2381146E38F, 6.8050684E37F, -1.4388168E38F, 1.2227514E38F, 2.3793528E37F, 2.2616465E38F, -1.667789E38F, 2.0728548E37F, 2.0501829E38F, -4.5996274E36F, -2.9965962E38F, -2.8228581E38F, 1.6526705E38F, 1.9925047E37F, 2.1784754E38F, -3.275051E38F, -6.790484E37F, -2.9078305E37F}, 0) ;
        p64.ax_SET(2.9462324E38F) ;
        p64.y_SET(6.5606177E37F) ;
        p64.az_SET(-7.6604775E37F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan11_raw_GET() == (char)34470);
            assert(pack.chan4_raw_GET() == (char)10805);
            assert(pack.chan6_raw_GET() == (char)7171);
            assert(pack.chan2_raw_GET() == (char)151);
            assert(pack.chan15_raw_GET() == (char)36490);
            assert(pack.rssi_GET() == (char)87);
            assert(pack.chan17_raw_GET() == (char)55246);
            assert(pack.chancount_GET() == (char)183);
            assert(pack.chan14_raw_GET() == (char)33583);
            assert(pack.chan16_raw_GET() == (char)19392);
            assert(pack.chan10_raw_GET() == (char)12485);
            assert(pack.chan3_raw_GET() == (char)60777);
            assert(pack.chan7_raw_GET() == (char)57824);
            assert(pack.chan5_raw_GET() == (char)43800);
            assert(pack.time_boot_ms_GET() == 2803708388L);
            assert(pack.chan9_raw_GET() == (char)22534);
            assert(pack.chan1_raw_GET() == (char)30846);
            assert(pack.chan13_raw_GET() == (char)12153);
            assert(pack.chan8_raw_GET() == (char)9199);
            assert(pack.chan18_raw_GET() == (char)45351);
            assert(pack.chan12_raw_GET() == (char)35543);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan17_raw_SET((char)55246) ;
        p65.rssi_SET((char)87) ;
        p65.time_boot_ms_SET(2803708388L) ;
        p65.chan10_raw_SET((char)12485) ;
        p65.chancount_SET((char)183) ;
        p65.chan2_raw_SET((char)151) ;
        p65.chan11_raw_SET((char)34470) ;
        p65.chan18_raw_SET((char)45351) ;
        p65.chan7_raw_SET((char)57824) ;
        p65.chan14_raw_SET((char)33583) ;
        p65.chan6_raw_SET((char)7171) ;
        p65.chan3_raw_SET((char)60777) ;
        p65.chan1_raw_SET((char)30846) ;
        p65.chan15_raw_SET((char)36490) ;
        p65.chan5_raw_SET((char)43800) ;
        p65.chan4_raw_SET((char)10805) ;
        p65.chan8_raw_SET((char)9199) ;
        p65.chan16_raw_SET((char)19392) ;
        p65.chan12_raw_SET((char)35543) ;
        p65.chan13_raw_SET((char)12153) ;
        p65.chan9_raw_SET((char)22534) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)128);
            assert(pack.start_stop_GET() == (char)188);
            assert(pack.req_message_rate_GET() == (char)48987);
            assert(pack.target_system_GET() == (char)19);
            assert(pack.req_stream_id_GET() == (char)152);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_system_SET((char)19) ;
        p66.req_message_rate_SET((char)48987) ;
        p66.start_stop_SET((char)188) ;
        p66.target_component_SET((char)128) ;
        p66.req_stream_id_SET((char)152) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.message_rate_GET() == (char)575);
            assert(pack.stream_id_GET() == (char)188);
            assert(pack.on_off_GET() == (char)200);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.on_off_SET((char)200) ;
        p67.message_rate_SET((char)575) ;
        p67.stream_id_SET((char)188) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == (short) -31161);
            assert(pack.target_GET() == (char)125);
            assert(pack.buttons_GET() == (char)7420);
            assert(pack.y_GET() == (short) -10912);
            assert(pack.r_GET() == (short)19763);
            assert(pack.z_GET() == (short) -19139);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.z_SET((short) -19139) ;
        p69.r_SET((short)19763) ;
        p69.y_SET((short) -10912) ;
        p69.buttons_SET((char)7420) ;
        p69.x_SET((short) -31161) ;
        p69.target_SET((char)125) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan6_raw_GET() == (char)34988);
            assert(pack.chan5_raw_GET() == (char)32474);
            assert(pack.chan8_raw_GET() == (char)60398);
            assert(pack.chan7_raw_GET() == (char)39743);
            assert(pack.chan4_raw_GET() == (char)42089);
            assert(pack.target_component_GET() == (char)229);
            assert(pack.target_system_GET() == (char)88);
            assert(pack.chan2_raw_GET() == (char)57411);
            assert(pack.chan1_raw_GET() == (char)1324);
            assert(pack.chan3_raw_GET() == (char)17330);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.target_system_SET((char)88) ;
        p70.chan7_raw_SET((char)39743) ;
        p70.chan3_raw_SET((char)17330) ;
        p70.chan6_raw_SET((char)34988) ;
        p70.chan1_raw_SET((char)1324) ;
        p70.chan5_raw_SET((char)32474) ;
        p70.target_component_SET((char)229) ;
        p70.chan8_raw_SET((char)60398) ;
        p70.chan2_raw_SET((char)57411) ;
        p70.chan4_raw_SET((char)42089) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)145);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.seq_GET() == (char)15621);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.autocontinue_GET() == (char)9);
            assert(pack.param2_GET() == 1.9390367E38F);
            assert(pack.param4_GET() == 1.2655954E38F);
            assert(pack.z_GET() == -1.0778935E38F);
            assert(pack.param1_GET() == 9.552606E37F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE);
            assert(pack.target_system_GET() == (char)140);
            assert(pack.y_GET() == -2057483185);
            assert(pack.x_GET() == -1454354966);
            assert(pack.current_GET() == (char)68);
            assert(pack.param3_GET() == 2.3127385E38F);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.x_SET(-1454354966) ;
        p73.y_SET(-2057483185) ;
        p73.command_SET(MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p73.target_system_SET((char)140) ;
        p73.target_component_SET((char)145) ;
        p73.z_SET(-1.0778935E38F) ;
        p73.param3_SET(2.3127385E38F) ;
        p73.current_SET((char)68) ;
        p73.param2_SET(1.9390367E38F) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p73.autocontinue_SET((char)9) ;
        p73.param4_SET(1.2655954E38F) ;
        p73.param1_SET(9.552606E37F) ;
        p73.seq_SET((char)15621) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.heading_GET() == (short)18115);
            assert(pack.climb_GET() == 2.3130386E38F);
            assert(pack.throttle_GET() == (char)27209);
            assert(pack.alt_GET() == -2.3020354E38F);
            assert(pack.airspeed_GET() == -2.7583517E38F);
            assert(pack.groundspeed_GET() == -8.488903E37F);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.groundspeed_SET(-8.488903E37F) ;
        p74.climb_SET(2.3130386E38F) ;
        p74.throttle_SET((char)27209) ;
        p74.heading_SET((short)18115) ;
        p74.airspeed_SET(-2.7583517E38F) ;
        p74.alt_SET(-2.3020354E38F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.param1_GET() == -1.566019E38F);
            assert(pack.target_component_GET() == (char)66);
            assert(pack.current_GET() == (char)179);
            assert(pack.x_GET() == -1964823502);
            assert(pack.param4_GET() == -1.210396E38F);
            assert(pack.target_system_GET() == (char)114);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.autocontinue_GET() == (char)197);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN);
            assert(pack.param3_GET() == 6.453713E37F);
            assert(pack.param2_GET() == -3.0455496E38F);
            assert(pack.z_GET() == 7.899345E37F);
            assert(pack.y_GET() == -201560697);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p75.x_SET(-1964823502) ;
        p75.target_component_SET((char)66) ;
        p75.current_SET((char)179) ;
        p75.target_system_SET((char)114) ;
        p75.param1_SET(-1.566019E38F) ;
        p75.y_SET(-201560697) ;
        p75.param4_SET(-1.210396E38F) ;
        p75.command_SET(MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN) ;
        p75.autocontinue_SET((char)197) ;
        p75.param3_SET(6.453713E37F) ;
        p75.param2_SET(-3.0455496E38F) ;
        p75.z_SET(7.899345E37F) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param7_GET() == -4.8469556E36F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE);
            assert(pack.target_system_GET() == (char)210);
            assert(pack.param5_GET() == 2.5776527E38F);
            assert(pack.target_component_GET() == (char)235);
            assert(pack.param6_GET() == -1.4954723E38F);
            assert(pack.param2_GET() == 2.582104E38F);
            assert(pack.param1_GET() == 3.3041402E38F);
            assert(pack.confirmation_GET() == (char)152);
            assert(pack.param4_GET() == 2.4128658E38F);
            assert(pack.param3_GET() == 1.4332428E38F);
        });
        COMMAND_LONG p76 = new COMMAND_LONG();
        PH.setPack(p76);
        p76.param2_SET(2.582104E38F) ;
        p76.confirmation_SET((char)152) ;
        p76.param3_SET(1.4332428E38F) ;
        p76.param5_SET(2.5776527E38F) ;
        p76.param1_SET(3.3041402E38F) ;
        p76.target_system_SET((char)210) ;
        p76.param7_SET(-4.8469556E36F) ;
        p76.param6_SET(-1.4954723E38F) ;
        p76.target_component_SET((char)235) ;
        p76.command_SET(MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE) ;
        p76.param4_SET(2.4128658E38F) ;
        TestChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.result_param2_TRY(ph) == 498632033);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_ROI);
            assert(pack.target_component_TRY(ph) == (char)20);
            assert(pack.progress_TRY(ph) == (char)21);
            assert(pack.target_system_TRY(ph) == (char)110);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_FAILED);
        });
        COMMAND_ACK p77 = new COMMAND_ACK();
        PH.setPack(p77);
        p77.result_param2_SET(498632033, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_FAILED) ;
        p77.command_SET(MAV_CMD.MAV_CMD_NAV_ROI) ;
        p77.progress_SET((char)21, PH) ;
        p77.target_system_SET((char)110, PH) ;
        p77.target_component_SET((char)20, PH) ;
        TestChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == 4.820696E37F);
            assert(pack.time_boot_ms_GET() == 2634297396L);
            assert(pack.manual_override_switch_GET() == (char)12);
            assert(pack.thrust_GET() == 2.533226E38F);
            assert(pack.mode_switch_GET() == (char)72);
            assert(pack.yaw_GET() == 1.752272E38F);
            assert(pack.pitch_GET() == -1.0675027E38F);
        });
        MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.thrust_SET(2.533226E38F) ;
        p81.mode_switch_SET((char)72) ;
        p81.pitch_SET(-1.0675027E38F) ;
        p81.manual_override_switch_SET((char)12) ;
        p81.time_boot_ms_SET(2634297396L) ;
        p81.roll_SET(4.820696E37F) ;
        p81.yaw_SET(1.752272E38F) ;
        TestChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)229);
            assert(pack.body_roll_rate_GET() == 2.016133E38F);
            assert(pack.target_system_GET() == (char)102);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.3795045E38F, 1.2791908E37F, 1.8539794E38F, -3.321212E38F}));
            assert(pack.type_mask_GET() == (char)222);
            assert(pack.body_yaw_rate_GET() == 1.3102058E38F);
            assert(pack.thrust_GET() == 8.690075E37F);
            assert(pack.body_pitch_rate_GET() == 6.933337E37F);
            assert(pack.time_boot_ms_GET() == 3374821067L);
        });
        SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.q_SET(new float[] {2.3795045E38F, 1.2791908E37F, 1.8539794E38F, -3.321212E38F}, 0) ;
        p82.time_boot_ms_SET(3374821067L) ;
        p82.body_yaw_rate_SET(1.3102058E38F) ;
        p82.body_roll_rate_SET(2.016133E38F) ;
        p82.body_pitch_rate_SET(6.933337E37F) ;
        p82.target_component_SET((char)229) ;
        p82.target_system_SET((char)102) ;
        p82.type_mask_SET((char)222) ;
        p82.thrust_SET(8.690075E37F) ;
        TestChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_roll_rate_GET() == -9.16939E37F);
            assert(pack.thrust_GET() == -2.4679535E38F);
            assert(pack.time_boot_ms_GET() == 2392500549L);
            assert(pack.body_yaw_rate_GET() == 2.8349531E38F);
            assert(pack.type_mask_GET() == (char)94);
            assert(pack.body_pitch_rate_GET() == -7.837101E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.4346781E38F, 7.1915913E37F, 2.5323124E38F, -3.2714805E38F}));
        });
        ATTITUDE_TARGET p83 = new ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.thrust_SET(-2.4679535E38F) ;
        p83.body_roll_rate_SET(-9.16939E37F) ;
        p83.body_yaw_rate_SET(2.8349531E38F) ;
        p83.time_boot_ms_SET(2392500549L) ;
        p83.q_SET(new float[] {-1.4346781E38F, 7.1915913E37F, 2.5323124E38F, -3.2714805E38F}, 0) ;
        p83.type_mask_SET((char)94) ;
        p83.body_pitch_rate_SET(-7.837101E37F) ;
        TestChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 1.7784753E38F);
            assert(pack.afz_GET() == -2.4910395E38F);
            assert(pack.vz_GET() == 2.4717655E37F);
            assert(pack.z_GET() == 2.723597E38F);
            assert(pack.type_mask_GET() == (char)33413);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.time_boot_ms_GET() == 3183856938L);
            assert(pack.yaw_rate_GET() == 2.9637718E38F);
            assert(pack.afy_GET() == 3.1075154E38F);
            assert(pack.target_component_GET() == (char)214);
            assert(pack.vy_GET() == 2.5187627E38F);
            assert(pack.x_GET() == -2.5824286E38F);
            assert(pack.vx_GET() == 8.349511E37F);
            assert(pack.yaw_GET() == -2.3428465E38F);
            assert(pack.target_system_GET() == (char)127);
            assert(pack.afx_GET() == -2.1009727E38F);
        });
        SET_POSITION_TARGET_LOCAL_NED p84 = new SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.time_boot_ms_SET(3183856938L) ;
        p84.target_component_SET((char)214) ;
        p84.vz_SET(2.4717655E37F) ;
        p84.type_mask_SET((char)33413) ;
        p84.vy_SET(2.5187627E38F) ;
        p84.afz_SET(-2.4910395E38F) ;
        p84.vx_SET(8.349511E37F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p84.yaw_SET(-2.3428465E38F) ;
        p84.afx_SET(-2.1009727E38F) ;
        p84.y_SET(1.7784753E38F) ;
        p84.afy_SET(3.1075154E38F) ;
        p84.target_system_SET((char)127) ;
        p84.z_SET(2.723597E38F) ;
        p84.yaw_rate_SET(2.9637718E38F) ;
        p84.x_SET(-2.5824286E38F) ;
        TestChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.lon_int_GET() == -593826651);
            assert(pack.yaw_rate_GET() == -3.1337317E38F);
            assert(pack.vx_GET() == 1.1572934E38F);
            assert(pack.afz_GET() == 1.3906117E38F);
            assert(pack.alt_GET() == -4.7512594E37F);
            assert(pack.lat_int_GET() == -933664988);
            assert(pack.time_boot_ms_GET() == 763459104L);
            assert(pack.vz_GET() == 2.1910335E38F);
            assert(pack.afy_GET() == 1.8300406E37F);
            assert(pack.yaw_GET() == 8.708054E36F);
            assert(pack.target_component_GET() == (char)93);
            assert(pack.afx_GET() == 3.198361E38F);
            assert(pack.target_system_GET() == (char)2);
            assert(pack.vy_GET() == -2.4051033E36F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.type_mask_GET() == (char)40819);
        });
        SET_POSITION_TARGET_GLOBAL_INT p86 = new SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.vz_SET(2.1910335E38F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p86.vx_SET(1.1572934E38F) ;
        p86.alt_SET(-4.7512594E37F) ;
        p86.afz_SET(1.3906117E38F) ;
        p86.vy_SET(-2.4051033E36F) ;
        p86.afy_SET(1.8300406E37F) ;
        p86.lat_int_SET(-933664988) ;
        p86.time_boot_ms_SET(763459104L) ;
        p86.yaw_rate_SET(-3.1337317E38F) ;
        p86.target_system_SET((char)2) ;
        p86.lon_int_SET(-593826651) ;
        p86.type_mask_SET((char)40819) ;
        p86.target_component_SET((char)93) ;
        p86.afx_SET(3.198361E38F) ;
        p86.yaw_SET(8.708054E36F) ;
        TestChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)17790);
            assert(pack.lon_int_GET() == 10113158);
            assert(pack.vy_GET() == 3.2685012E38F);
            assert(pack.alt_GET() == 3.2963885E38F);
            assert(pack.afy_GET() == -2.2350613E38F);
            assert(pack.vx_GET() == 3.0818677E38F);
            assert(pack.yaw_rate_GET() == -1.4357084E37F);
            assert(pack.lat_int_GET() == 1107212863);
            assert(pack.vz_GET() == -1.6104885E38F);
            assert(pack.time_boot_ms_GET() == 3799539072L);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.afx_GET() == 1.3369834E38F);
            assert(pack.yaw_GET() == -4.7380703E37F);
            assert(pack.afz_GET() == -2.6627944E38F);
        });
        POSITION_TARGET_GLOBAL_INT p87 = new POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.time_boot_ms_SET(3799539072L) ;
        p87.afx_SET(1.3369834E38F) ;
        p87.vz_SET(-1.6104885E38F) ;
        p87.vy_SET(3.2685012E38F) ;
        p87.afz_SET(-2.6627944E38F) ;
        p87.alt_SET(3.2963885E38F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p87.type_mask_SET((char)17790) ;
        p87.yaw_rate_SET(-1.4357084E37F) ;
        p87.afy_SET(-2.2350613E38F) ;
        p87.vx_SET(3.0818677E38F) ;
        p87.lat_int_SET(1107212863) ;
        p87.lon_int_SET(10113158) ;
        p87.yaw_SET(-4.7380703E37F) ;
        TestChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1029151282L);
            assert(pack.pitch_GET() == -1.9076282E38F);
            assert(pack.roll_GET() == 8.25622E37F);
            assert(pack.yaw_GET() == -6.228899E37F);
            assert(pack.x_GET() == -7.5170905E37F);
            assert(pack.z_GET() == 1.4740397E38F);
            assert(pack.y_GET() == -2.3613694E38F);
        });
        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = new LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.z_SET(1.4740397E38F) ;
        p89.y_SET(-2.3613694E38F) ;
        p89.x_SET(-7.5170905E37F) ;
        p89.time_boot_ms_SET(1029151282L) ;
        p89.roll_SET(8.25622E37F) ;
        p89.yaw_SET(-6.228899E37F) ;
        p89.pitch_SET(-1.9076282E38F) ;
        TestChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == (short) -30741);
            assert(pack.roll_GET() == 3.3351812E38F);
            assert(pack.vx_GET() == (short) -28986);
            assert(pack.pitch_GET() == -1.8778956E38F);
            assert(pack.pitchspeed_GET() == -1.647787E38F);
            assert(pack.yaw_GET() == 1.944044E38F);
            assert(pack.alt_GET() == -641371783);
            assert(pack.lon_GET() == 322634389);
            assert(pack.yacc_GET() == (short)30618);
            assert(pack.lat_GET() == -845331880);
            assert(pack.vz_GET() == (short)18393);
            assert(pack.vy_GET() == (short) -19639);
            assert(pack.time_usec_GET() == 3116363659919412656L);
            assert(pack.yawspeed_GET() == 1.8688048E38F);
            assert(pack.rollspeed_GET() == 2.828453E38F);
            assert(pack.xacc_GET() == (short)6377);
        });
        HIL_STATE p90 = new HIL_STATE();
        PH.setPack(p90);
        p90.pitchspeed_SET(-1.647787E38F) ;
        p90.xacc_SET((short)6377) ;
        p90.lon_SET(322634389) ;
        p90.pitch_SET(-1.8778956E38F) ;
        p90.yaw_SET(1.944044E38F) ;
        p90.yacc_SET((short)30618) ;
        p90.lat_SET(-845331880) ;
        p90.yawspeed_SET(1.8688048E38F) ;
        p90.zacc_SET((short) -30741) ;
        p90.vx_SET((short) -28986) ;
        p90.alt_SET(-641371783) ;
        p90.time_usec_SET(3116363659919412656L) ;
        p90.vz_SET((short)18393) ;
        p90.rollspeed_SET(2.828453E38F) ;
        p90.roll_SET(3.3351812E38F) ;
        p90.vy_SET((short) -19639) ;
        TestChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_ARMED);
            assert(pack.yaw_rudder_GET() == -1.2195063E38F);
            assert(pack.pitch_elevator_GET() == 1.518944E38F);
            assert(pack.time_usec_GET() == 5608891310925592169L);
            assert(pack.aux4_GET() == 2.5453473E38F);
            assert(pack.aux1_GET() == -2.2424135E38F);
            assert(pack.roll_ailerons_GET() == -1.4598784E38F);
            assert(pack.aux3_GET() == 5.3400826E37F);
            assert(pack.throttle_GET() == 2.2762376E36F);
            assert(pack.nav_mode_GET() == (char)40);
            assert(pack.aux2_GET() == 1.186719E38F);
        });
        HIL_CONTROLS p91 = new HIL_CONTROLS();
        PH.setPack(p91);
        p91.pitch_elevator_SET(1.518944E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_TEST_ARMED) ;
        p91.aux1_SET(-2.2424135E38F) ;
        p91.nav_mode_SET((char)40) ;
        p91.aux4_SET(2.5453473E38F) ;
        p91.yaw_rudder_SET(-1.2195063E38F) ;
        p91.throttle_SET(2.2762376E36F) ;
        p91.aux2_SET(1.186719E38F) ;
        p91.roll_ailerons_SET(-1.4598784E38F) ;
        p91.aux3_SET(5.3400826E37F) ;
        p91.time_usec_SET(5608891310925592169L) ;
        TestChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan6_raw_GET() == (char)27386);
            assert(pack.chan9_raw_GET() == (char)39021);
            assert(pack.rssi_GET() == (char)115);
            assert(pack.time_usec_GET() == 6998417421132432746L);
            assert(pack.chan10_raw_GET() == (char)15903);
            assert(pack.chan1_raw_GET() == (char)24473);
            assert(pack.chan7_raw_GET() == (char)42144);
            assert(pack.chan12_raw_GET() == (char)562);
            assert(pack.chan4_raw_GET() == (char)105);
            assert(pack.chan8_raw_GET() == (char)43425);
            assert(pack.chan2_raw_GET() == (char)5743);
            assert(pack.chan3_raw_GET() == (char)56008);
            assert(pack.chan5_raw_GET() == (char)20997);
            assert(pack.chan11_raw_GET() == (char)10540);
        });
        HIL_RC_INPUTS_RAW p92 = new HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan3_raw_SET((char)56008) ;
        p92.chan5_raw_SET((char)20997) ;
        p92.chan7_raw_SET((char)42144) ;
        p92.chan10_raw_SET((char)15903) ;
        p92.chan4_raw_SET((char)105) ;
        p92.time_usec_SET(6998417421132432746L) ;
        p92.chan6_raw_SET((char)27386) ;
        p92.chan2_raw_SET((char)5743) ;
        p92.chan8_raw_SET((char)43425) ;
        p92.chan11_raw_SET((char)10540) ;
        p92.chan12_raw_SET((char)562) ;
        p92.chan9_raw_SET((char)39021) ;
        p92.rssi_SET((char)115) ;
        p92.chan1_raw_SET((char)24473) ;
        TestChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-2.2167613E38F, -4.758156E37F, -2.5354943E38F, -2.5327576E38F, 1.9032796E38F, -1.4062793E37F, -6.2699165E36F, 2.468126E38F, -1.1902077E38F, -1.9251749E38F, 2.545479E38F, 1.6535182E38F, 1.1610221E38F, -2.9773314E38F, 5.3279056E37F, 2.048923E38F}));
            assert(pack.time_usec_GET() == 5385199467803984640L);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_PREFLIGHT);
            assert(pack.flags_GET() == 1502941570595510403L);
        });
        HIL_ACTUATOR_CONTROLS p93 = new HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.flags_SET(1502941570595510403L) ;
        p93.time_usec_SET(5385199467803984640L) ;
        p93.controls_SET(new float[] {-2.2167613E38F, -4.758156E37F, -2.5354943E38F, -2.5327576E38F, 1.9032796E38F, -1.4062793E37F, -6.2699165E36F, 2.468126E38F, -1.1902077E38F, -1.9251749E38F, 2.545479E38F, 1.6535182E38F, 1.1610221E38F, -2.9773314E38F, 5.3279056E37F, 2.048923E38F}, 0) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_PREFLIGHT) ;
        TestChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.sensor_id_GET() == (char)3);
            assert(pack.ground_distance_GET() == 1.7766939E38F);
            assert(pack.flow_comp_m_x_GET() == 2.7068322E38F);
            assert(pack.flow_y_GET() == (short)23124);
            assert(pack.flow_rate_y_TRY(ph) == 3.2941254E38F);
            assert(pack.flow_rate_x_TRY(ph) == -2.8339875E38F);
            assert(pack.flow_x_GET() == (short) -11398);
            assert(pack.time_usec_GET() == 4686121533844773168L);
            assert(pack.quality_GET() == (char)252);
            assert(pack.flow_comp_m_y_GET() == -6.806691E36F);
        });
        OPTICAL_FLOW p100 = new OPTICAL_FLOW();
        PH.setPack(p100);
        p100.quality_SET((char)252) ;
        p100.sensor_id_SET((char)3) ;
        p100.flow_comp_m_y_SET(-6.806691E36F) ;
        p100.time_usec_SET(4686121533844773168L) ;
        p100.flow_x_SET((short) -11398) ;
        p100.flow_rate_y_SET(3.2941254E38F, PH) ;
        p100.ground_distance_SET(1.7766939E38F) ;
        p100.flow_y_SET((short)23124) ;
        p100.flow_comp_m_x_SET(2.7068322E38F) ;
        p100.flow_rate_x_SET(-2.8339875E38F, PH) ;
        TestChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 6233794304693408255L);
            assert(pack.z_GET() == -3.0862047E38F);
            assert(pack.pitch_GET() == 1.0517831E38F);
            assert(pack.yaw_GET() == 5.906544E37F);
            assert(pack.x_GET() == 2.266511E38F);
            assert(pack.y_GET() == -1.1840893E38F);
            assert(pack.roll_GET() == 1.5146216E38F);
        });
        GLOBAL_VISION_POSITION_ESTIMATE p101 = new GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.y_SET(-1.1840893E38F) ;
        p101.x_SET(2.266511E38F) ;
        p101.usec_SET(6233794304693408255L) ;
        p101.pitch_SET(1.0517831E38F) ;
        p101.yaw_SET(5.906544E37F) ;
        p101.roll_SET(1.5146216E38F) ;
        p101.z_SET(-3.0862047E38F) ;
        TestChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == 3.1585484E38F);
            assert(pack.pitch_GET() == -7.577469E37F);
            assert(pack.usec_GET() == 24561176377257973L);
            assert(pack.y_GET() == -2.446198E37F);
            assert(pack.x_GET() == 6.9751704E37F);
            assert(pack.yaw_GET() == 7.7135216E37F);
            assert(pack.z_GET() == -1.4485004E38F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.y_SET(-2.446198E37F) ;
        p102.pitch_SET(-7.577469E37F) ;
        p102.z_SET(-1.4485004E38F) ;
        p102.roll_SET(3.1585484E38F) ;
        p102.usec_SET(24561176377257973L) ;
        p102.yaw_SET(7.7135216E37F) ;
        p102.x_SET(6.9751704E37F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 1.4551695E37F);
            assert(pack.y_GET() == 2.798461E38F);
            assert(pack.usec_GET() == 2807004306744176143L);
            assert(pack.z_GET() == 1.7026053E38F);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(2807004306744176143L) ;
        p103.z_SET(1.7026053E38F) ;
        p103.y_SET(2.798461E38F) ;
        p103.x_SET(1.4551695E37F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -7.471302E37F);
            assert(pack.yaw_GET() == 1.2175185E38F);
            assert(pack.usec_GET() == 1926259401857632756L);
            assert(pack.y_GET() == -9.915923E37F);
            assert(pack.z_GET() == 1.5789616E38F);
            assert(pack.roll_GET() == 7.4766595E37F);
            assert(pack.x_GET() == -1.674588E38F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.y_SET(-9.915923E37F) ;
        p104.usec_SET(1926259401857632756L) ;
        p104.x_SET(-1.674588E38F) ;
        p104.roll_SET(7.4766595E37F) ;
        p104.yaw_SET(1.2175185E38F) ;
        p104.z_SET(1.5789616E38F) ;
        p104.pitch_SET(-7.471302E37F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.pressure_alt_GET() == 2.3504771E38F);
            assert(pack.temperature_GET() == 5.008723E37F);
            assert(pack.zmag_GET() == 2.2458434E38F);
            assert(pack.ymag_GET() == -1.1181316E37F);
            assert(pack.xmag_GET() == -2.9224837E38F);
            assert(pack.xgyro_GET() == 2.9699208E38F);
            assert(pack.zgyro_GET() == -3.028349E38F);
            assert(pack.time_usec_GET() == 8460170058849921004L);
            assert(pack.yacc_GET() == -5.4235924E37F);
            assert(pack.abs_pressure_GET() == -1.1128546E37F);
            assert(pack.zacc_GET() == -2.4438963E38F);
            assert(pack.ygyro_GET() == 2.410888E38F);
            assert(pack.diff_pressure_GET() == -1.9686078E38F);
            assert(pack.xacc_GET() == -1.5428081E37F);
            assert(pack.fields_updated_GET() == (char)36490);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.fields_updated_SET((char)36490) ;
        p105.zmag_SET(2.2458434E38F) ;
        p105.ymag_SET(-1.1181316E37F) ;
        p105.zacc_SET(-2.4438963E38F) ;
        p105.temperature_SET(5.008723E37F) ;
        p105.yacc_SET(-5.4235924E37F) ;
        p105.time_usec_SET(8460170058849921004L) ;
        p105.xacc_SET(-1.5428081E37F) ;
        p105.pressure_alt_SET(2.3504771E38F) ;
        p105.abs_pressure_SET(-1.1128546E37F) ;
        p105.xmag_SET(-2.9224837E38F) ;
        p105.diff_pressure_SET(-1.9686078E38F) ;
        p105.xgyro_SET(2.9699208E38F) ;
        p105.zgyro_SET(-3.028349E38F) ;
        p105.ygyro_SET(2.410888E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integrated_y_GET() == -8.966199E37F);
            assert(pack.quality_GET() == (char)239);
            assert(pack.sensor_id_GET() == (char)253);
            assert(pack.integrated_x_GET() == -1.0445557E38F);
            assert(pack.integration_time_us_GET() == 3017003204L);
            assert(pack.temperature_GET() == (short)14654);
            assert(pack.time_usec_GET() == 2869405804008112202L);
            assert(pack.integrated_ygyro_GET() == 2.5959618E38F);
            assert(pack.time_delta_distance_us_GET() == 380262601L);
            assert(pack.integrated_zgyro_GET() == 8.628923E37F);
            assert(pack.distance_GET() == -2.5240816E38F);
            assert(pack.integrated_xgyro_GET() == -1.927725E38F);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_delta_distance_us_SET(380262601L) ;
        p106.time_usec_SET(2869405804008112202L) ;
        p106.integrated_zgyro_SET(8.628923E37F) ;
        p106.quality_SET((char)239) ;
        p106.temperature_SET((short)14654) ;
        p106.sensor_id_SET((char)253) ;
        p106.integrated_xgyro_SET(-1.927725E38F) ;
        p106.integrated_y_SET(-8.966199E37F) ;
        p106.distance_SET(-2.5240816E38F) ;
        p106.integrated_ygyro_SET(2.5959618E38F) ;
        p106.integration_time_us_SET(3017003204L) ;
        p106.integrated_x_SET(-1.0445557E38F) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == -1.3403481E38F);
            assert(pack.xmag_GET() == 2.8544563E38F);
            assert(pack.xacc_GET() == -3.0555908E38F);
            assert(pack.ymag_GET() == -1.5950123E38F);
            assert(pack.yacc_GET() == -9.824004E37F);
            assert(pack.xgyro_GET() == -3.2246257E38F);
            assert(pack.zgyro_GET() == 8.80833E36F);
            assert(pack.fields_updated_GET() == 1727759635L);
            assert(pack.temperature_GET() == 1.7385402E38F);
            assert(pack.zmag_GET() == -2.3754895E37F);
            assert(pack.ygyro_GET() == -3.218429E38F);
            assert(pack.time_usec_GET() == 5157281422352345831L);
            assert(pack.pressure_alt_GET() == -4.0802032E37F);
            assert(pack.abs_pressure_GET() == -2.3558962E36F);
            assert(pack.diff_pressure_GET() == -1.5096311E38F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.ygyro_SET(-3.218429E38F) ;
        p107.yacc_SET(-9.824004E37F) ;
        p107.zacc_SET(-1.3403481E38F) ;
        p107.fields_updated_SET(1727759635L) ;
        p107.time_usec_SET(5157281422352345831L) ;
        p107.abs_pressure_SET(-2.3558962E36F) ;
        p107.zgyro_SET(8.80833E36F) ;
        p107.pressure_alt_SET(-4.0802032E37F) ;
        p107.xmag_SET(2.8544563E38F) ;
        p107.zmag_SET(-2.3754895E37F) ;
        p107.xacc_SET(-3.0555908E38F) ;
        p107.diff_pressure_SET(-1.5096311E38F) ;
        p107.temperature_SET(1.7385402E38F) ;
        p107.ymag_SET(-1.5950123E38F) ;
        p107.xgyro_SET(-3.2246257E38F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 9.072713E37F);
            assert(pack.vd_GET() == -1.1399047E38F);
            assert(pack.q3_GET() == 2.1884101E38F);
            assert(pack.zacc_GET() == -2.2012096E38F);
            assert(pack.xgyro_GET() == -7.729366E37F);
            assert(pack.lon_GET() == -2.445612E38F);
            assert(pack.std_dev_horz_GET() == 1.4616099E38F);
            assert(pack.lat_GET() == -2.6583793E38F);
            assert(pack.yacc_GET() == 1.3093861E38F);
            assert(pack.ygyro_GET() == -2.0426398E38F);
            assert(pack.xacc_GET() == -1.3309871E38F);
            assert(pack.ve_GET() == 1.0626679E38F);
            assert(pack.alt_GET() == 4.463316E37F);
            assert(pack.zgyro_GET() == -8.538047E37F);
            assert(pack.roll_GET() == -2.6572265E38F);
            assert(pack.vn_GET() == 3.1493875E38F);
            assert(pack.std_dev_vert_GET() == -7.5807255E37F);
            assert(pack.q2_GET() == 9.38687E36F);
            assert(pack.q1_GET() == -2.066072E38F);
            assert(pack.q4_GET() == 3.1203351E38F);
            assert(pack.yaw_GET() == 2.5849132E38F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.q1_SET(-2.066072E38F) ;
        p108.std_dev_horz_SET(1.4616099E38F) ;
        p108.lon_SET(-2.445612E38F) ;
        p108.q4_SET(3.1203351E38F) ;
        p108.vn_SET(3.1493875E38F) ;
        p108.zgyro_SET(-8.538047E37F) ;
        p108.std_dev_vert_SET(-7.5807255E37F) ;
        p108.ve_SET(1.0626679E38F) ;
        p108.vd_SET(-1.1399047E38F) ;
        p108.alt_SET(4.463316E37F) ;
        p108.ygyro_SET(-2.0426398E38F) ;
        p108.q3_SET(2.1884101E38F) ;
        p108.q2_SET(9.38687E36F) ;
        p108.lat_SET(-2.6583793E38F) ;
        p108.roll_SET(-2.6572265E38F) ;
        p108.zacc_SET(-2.2012096E38F) ;
        p108.xacc_SET(-1.3309871E38F) ;
        p108.yaw_SET(2.5849132E38F) ;
        p108.yacc_SET(1.3093861E38F) ;
        p108.xgyro_SET(-7.729366E37F) ;
        p108.pitch_SET(9.072713E37F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.remnoise_GET() == (char)72);
            assert(pack.noise_GET() == (char)0);
            assert(pack.fixed__GET() == (char)31689);
            assert(pack.rxerrors_GET() == (char)10922);
            assert(pack.txbuf_GET() == (char)170);
            assert(pack.rssi_GET() == (char)187);
            assert(pack.remrssi_GET() == (char)191);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rxerrors_SET((char)10922) ;
        p109.remrssi_SET((char)191) ;
        p109.noise_SET((char)0) ;
        p109.rssi_SET((char)187) ;
        p109.fixed__SET((char)31689) ;
        p109.txbuf_SET((char)170) ;
        p109.remnoise_SET((char)72) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)207);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)65, (char)130, (char)192, (char)125, (char)190, (char)236, (char)216, (char)232, (char)222, (char)59, (char)54, (char)2, (char)215, (char)150, (char)34, (char)67, (char)237, (char)188, (char)18, (char)158, (char)237, (char)117, (char)48, (char)111, (char)201, (char)105, (char)218, (char)162, (char)37, (char)224, (char)145, (char)20, (char)135, (char)127, (char)96, (char)167, (char)41, (char)87, (char)233, (char)102, (char)159, (char)70, (char)150, (char)86, (char)208, (char)134, (char)185, (char)254, (char)120, (char)187, (char)16, (char)97, (char)57, (char)44, (char)241, (char)30, (char)83, (char)194, (char)185, (char)101, (char)75, (char)244, (char)145, (char)3, (char)190, (char)210, (char)5, (char)204, (char)96, (char)27, (char)98, (char)109, (char)127, (char)158, (char)162, (char)219, (char)222, (char)26, (char)19, (char)30, (char)189, (char)63, (char)45, (char)137, (char)42, (char)45, (char)186, (char)67, (char)91, (char)17, (char)193, (char)211, (char)206, (char)157, (char)137, (char)223, (char)51, (char)115, (char)5, (char)248, (char)59, (char)123, (char)192, (char)69, (char)212, (char)204, (char)165, (char)196, (char)108, (char)19, (char)195, (char)15, (char)167, (char)239, (char)253, (char)231, (char)212, (char)48, (char)49, (char)149, (char)135, (char)223, (char)176, (char)107, (char)95, (char)161, (char)42, (char)223, (char)67, (char)52, (char)121, (char)1, (char)185, (char)127, (char)214, (char)201, (char)213, (char)201, (char)22, (char)201, (char)162, (char)56, (char)68, (char)207, (char)86, (char)220, (char)77, (char)142, (char)196, (char)226, (char)218, (char)100, (char)65, (char)110, (char)48, (char)68, (char)222, (char)123, (char)193, (char)142, (char)66, (char)98, (char)185, (char)184, (char)4, (char)21, (char)94, (char)123, (char)246, (char)238, (char)141, (char)196, (char)181, (char)225, (char)66, (char)97, (char)177, (char)16, (char)27, (char)227, (char)14, (char)18, (char)14, (char)89, (char)40, (char)246, (char)72, (char)71, (char)63, (char)22, (char)25, (char)189, (char)220, (char)153, (char)47, (char)126, (char)33, (char)230, (char)64, (char)89, (char)37, (char)240, (char)48, (char)46, (char)216, (char)231, (char)94, (char)150, (char)129, (char)64, (char)10, (char)248, (char)248, (char)48, (char)132, (char)148, (char)4, (char)195, (char)44, (char)180, (char)136, (char)74, (char)118, (char)121, (char)52, (char)197, (char)245, (char)213, (char)155, (char)186, (char)70, (char)127, (char)125, (char)79, (char)31, (char)149, (char)219, (char)163, (char)164, (char)177, (char)245, (char)214, (char)109, (char)11, (char)237, (char)148, (char)7, (char)191, (char)15, (char)193, (char)150}));
            assert(pack.target_network_GET() == (char)184);
            assert(pack.target_component_GET() == (char)51);
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_system_SET((char)207) ;
        p110.target_network_SET((char)184) ;
        p110.payload_SET(new char[] {(char)65, (char)130, (char)192, (char)125, (char)190, (char)236, (char)216, (char)232, (char)222, (char)59, (char)54, (char)2, (char)215, (char)150, (char)34, (char)67, (char)237, (char)188, (char)18, (char)158, (char)237, (char)117, (char)48, (char)111, (char)201, (char)105, (char)218, (char)162, (char)37, (char)224, (char)145, (char)20, (char)135, (char)127, (char)96, (char)167, (char)41, (char)87, (char)233, (char)102, (char)159, (char)70, (char)150, (char)86, (char)208, (char)134, (char)185, (char)254, (char)120, (char)187, (char)16, (char)97, (char)57, (char)44, (char)241, (char)30, (char)83, (char)194, (char)185, (char)101, (char)75, (char)244, (char)145, (char)3, (char)190, (char)210, (char)5, (char)204, (char)96, (char)27, (char)98, (char)109, (char)127, (char)158, (char)162, (char)219, (char)222, (char)26, (char)19, (char)30, (char)189, (char)63, (char)45, (char)137, (char)42, (char)45, (char)186, (char)67, (char)91, (char)17, (char)193, (char)211, (char)206, (char)157, (char)137, (char)223, (char)51, (char)115, (char)5, (char)248, (char)59, (char)123, (char)192, (char)69, (char)212, (char)204, (char)165, (char)196, (char)108, (char)19, (char)195, (char)15, (char)167, (char)239, (char)253, (char)231, (char)212, (char)48, (char)49, (char)149, (char)135, (char)223, (char)176, (char)107, (char)95, (char)161, (char)42, (char)223, (char)67, (char)52, (char)121, (char)1, (char)185, (char)127, (char)214, (char)201, (char)213, (char)201, (char)22, (char)201, (char)162, (char)56, (char)68, (char)207, (char)86, (char)220, (char)77, (char)142, (char)196, (char)226, (char)218, (char)100, (char)65, (char)110, (char)48, (char)68, (char)222, (char)123, (char)193, (char)142, (char)66, (char)98, (char)185, (char)184, (char)4, (char)21, (char)94, (char)123, (char)246, (char)238, (char)141, (char)196, (char)181, (char)225, (char)66, (char)97, (char)177, (char)16, (char)27, (char)227, (char)14, (char)18, (char)14, (char)89, (char)40, (char)246, (char)72, (char)71, (char)63, (char)22, (char)25, (char)189, (char)220, (char)153, (char)47, (char)126, (char)33, (char)230, (char)64, (char)89, (char)37, (char)240, (char)48, (char)46, (char)216, (char)231, (char)94, (char)150, (char)129, (char)64, (char)10, (char)248, (char)248, (char)48, (char)132, (char)148, (char)4, (char)195, (char)44, (char)180, (char)136, (char)74, (char)118, (char)121, (char)52, (char)197, (char)245, (char)213, (char)155, (char)186, (char)70, (char)127, (char)125, (char)79, (char)31, (char)149, (char)219, (char)163, (char)164, (char)177, (char)245, (char)214, (char)109, (char)11, (char)237, (char)148, (char)7, (char)191, (char)15, (char)193, (char)150}, 0) ;
        p110.target_component_SET((char)51) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == -5114194503006301677L);
            assert(pack.ts1_GET() == -4696801088383991631L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(-4696801088383991631L) ;
        p111.tc1_SET(-5114194503006301677L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 715287030L);
            assert(pack.time_usec_GET() == 3775288640064715059L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(3775288640064715059L) ;
        p112.seq_SET(715287030L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.vn_GET() == (short) -531);
            assert(pack.alt_GET() == -1304242795);
            assert(pack.time_usec_GET() == 283051511443156634L);
            assert(pack.satellites_visible_GET() == (char)214);
            assert(pack.lat_GET() == -747570378);
            assert(pack.eph_GET() == (char)2419);
            assert(pack.cog_GET() == (char)45622);
            assert(pack.lon_GET() == -1260577976);
            assert(pack.ve_GET() == (short) -8766);
            assert(pack.vel_GET() == (char)956);
            assert(pack.fix_type_GET() == (char)6);
            assert(pack.vd_GET() == (short)28097);
            assert(pack.epv_GET() == (char)24002);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.eph_SET((char)2419) ;
        p113.lat_SET(-747570378) ;
        p113.vd_SET((short)28097) ;
        p113.alt_SET(-1304242795) ;
        p113.lon_SET(-1260577976) ;
        p113.ve_SET((short) -8766) ;
        p113.cog_SET((char)45622) ;
        p113.time_usec_SET(283051511443156634L) ;
        p113.satellites_visible_SET((char)214) ;
        p113.fix_type_SET((char)6) ;
        p113.vn_SET((short) -531) ;
        p113.vel_SET((char)956) ;
        p113.epv_SET((char)24002) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.sensor_id_GET() == (char)196);
            assert(pack.integrated_zgyro_GET() == 6.151516E37F);
            assert(pack.integrated_ygyro_GET() == 2.612861E38F);
            assert(pack.quality_GET() == (char)181);
            assert(pack.integrated_x_GET() == -2.1310969E37F);
            assert(pack.time_delta_distance_us_GET() == 1105766736L);
            assert(pack.distance_GET() == -2.8710576E38F);
            assert(pack.integration_time_us_GET() == 1809975718L);
            assert(pack.integrated_xgyro_GET() == 2.9229597E37F);
            assert(pack.integrated_y_GET() == -1.8640845E38F);
            assert(pack.temperature_GET() == (short) -6846);
            assert(pack.time_usec_GET() == 1092558481533364285L);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.temperature_SET((short) -6846) ;
        p114.integrated_x_SET(-2.1310969E37F) ;
        p114.integrated_y_SET(-1.8640845E38F) ;
        p114.integrated_ygyro_SET(2.612861E38F) ;
        p114.quality_SET((char)181) ;
        p114.sensor_id_SET((char)196) ;
        p114.integrated_xgyro_SET(2.9229597E37F) ;
        p114.integrated_zgyro_SET(6.151516E37F) ;
        p114.time_delta_distance_us_SET(1105766736L) ;
        p114.distance_SET(-2.8710576E38F) ;
        p114.integration_time_us_SET(1809975718L) ;
        p114.time_usec_SET(1092558481533364285L) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == -1.805421E38F);
            assert(pack.vx_GET() == (short)32203);
            assert(pack.yacc_GET() == (short) -20825);
            assert(pack.ind_airspeed_GET() == (char)27735);
            assert(pack.lat_GET() == 555606259);
            assert(pack.pitchspeed_GET() == 4.411833E37F);
            assert(pack.lon_GET() == 1141568674);
            assert(pack.vz_GET() == (short)23048);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {1.745011E38F, -1.1075491E38F, -3.504688E37F, 2.8206579E38F}));
            assert(pack.zacc_GET() == (short)4953);
            assert(pack.rollspeed_GET() == 1.0217305E38F);
            assert(pack.xacc_GET() == (short) -18308);
            assert(pack.vy_GET() == (short)31967);
            assert(pack.alt_GET() == 641002519);
            assert(pack.true_airspeed_GET() == (char)60467);
            assert(pack.time_usec_GET() == 2735077850572827216L);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.yacc_SET((short) -20825) ;
        p115.yawspeed_SET(-1.805421E38F) ;
        p115.xacc_SET((short) -18308) ;
        p115.vx_SET((short)32203) ;
        p115.ind_airspeed_SET((char)27735) ;
        p115.lat_SET(555606259) ;
        p115.zacc_SET((short)4953) ;
        p115.pitchspeed_SET(4.411833E37F) ;
        p115.true_airspeed_SET((char)60467) ;
        p115.lon_SET(1141568674) ;
        p115.attitude_quaternion_SET(new float[] {1.745011E38F, -1.1075491E38F, -3.504688E37F, 2.8206579E38F}, 0) ;
        p115.vz_SET((short)23048) ;
        p115.alt_SET(641002519) ;
        p115.vy_SET((short)31967) ;
        p115.time_usec_SET(2735077850572827216L) ;
        p115.rollspeed_SET(1.0217305E38F) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2770791688L);
            assert(pack.yacc_GET() == (short) -24808);
            assert(pack.ymag_GET() == (short)31009);
            assert(pack.zmag_GET() == (short)10002);
            assert(pack.xacc_GET() == (short) -24935);
            assert(pack.zgyro_GET() == (short)6224);
            assert(pack.xgyro_GET() == (short)31680);
            assert(pack.xmag_GET() == (short)24672);
            assert(pack.zacc_GET() == (short)12129);
            assert(pack.ygyro_GET() == (short)24599);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.zmag_SET((short)10002) ;
        p116.ygyro_SET((short)24599) ;
        p116.xacc_SET((short) -24935) ;
        p116.time_boot_ms_SET(2770791688L) ;
        p116.xgyro_SET((short)31680) ;
        p116.ymag_SET((short)31009) ;
        p116.zgyro_SET((short)6224) ;
        p116.yacc_SET((short) -24808) ;
        p116.xmag_SET((short)24672) ;
        p116.zacc_SET((short)12129) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_GET() == (char)24848);
            assert(pack.target_system_GET() == (char)140);
            assert(pack.end_GET() == (char)36886);
            assert(pack.target_component_GET() == (char)221);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.end_SET((char)36886) ;
        p117.target_system_SET((char)140) ;
        p117.start_SET((char)24848) ;
        p117.target_component_SET((char)221) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.time_utc_GET() == 1057323046L);
            assert(pack.id_GET() == (char)11060);
            assert(pack.num_logs_GET() == (char)40097);
            assert(pack.last_log_num_GET() == (char)25319);
            assert(pack.size_GET() == 1624788363L);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.size_SET(1624788363L) ;
        p118.time_utc_SET(1057323046L) ;
        p118.last_log_num_SET((char)25319) ;
        p118.id_SET((char)11060) ;
        p118.num_logs_SET((char)40097) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)238);
            assert(pack.ofs_GET() == 3789036216L);
            assert(pack.count_GET() == 3455804406L);
            assert(pack.target_component_GET() == (char)196);
            assert(pack.id_GET() == (char)21188);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.count_SET(3455804406L) ;
        p119.target_component_SET((char)196) ;
        p119.ofs_SET(3789036216L) ;
        p119.target_system_SET((char)238) ;
        p119.id_SET((char)21188) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)10938);
            assert(pack.count_GET() == (char)176);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)45, (char)151, (char)248, (char)77, (char)96, (char)25, (char)231, (char)190, (char)211, (char)87, (char)246, (char)10, (char)208, (char)104, (char)115, (char)123, (char)55, (char)196, (char)46, (char)147, (char)54, (char)232, (char)59, (char)168, (char)168, (char)203, (char)25, (char)23, (char)10, (char)158, (char)143, (char)218, (char)143, (char)34, (char)143, (char)45, (char)199, (char)30, (char)109, (char)210, (char)42, (char)177, (char)184, (char)166, (char)84, (char)153, (char)185, (char)178, (char)226, (char)178, (char)146, (char)88, (char)93, (char)187, (char)96, (char)40, (char)219, (char)189, (char)75, (char)224, (char)191, (char)62, (char)124, (char)209, (char)59, (char)190, (char)217, (char)3, (char)166, (char)102, (char)110, (char)29, (char)56, (char)151, (char)189, (char)202, (char)239, (char)35, (char)46, (char)74, (char)9, (char)107, (char)233, (char)85, (char)112, (char)148, (char)86, (char)115, (char)7, (char)48}));
            assert(pack.ofs_GET() == 1258340714L);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.ofs_SET(1258340714L) ;
        p120.id_SET((char)10938) ;
        p120.count_SET((char)176) ;
        p120.data__SET(new char[] {(char)45, (char)151, (char)248, (char)77, (char)96, (char)25, (char)231, (char)190, (char)211, (char)87, (char)246, (char)10, (char)208, (char)104, (char)115, (char)123, (char)55, (char)196, (char)46, (char)147, (char)54, (char)232, (char)59, (char)168, (char)168, (char)203, (char)25, (char)23, (char)10, (char)158, (char)143, (char)218, (char)143, (char)34, (char)143, (char)45, (char)199, (char)30, (char)109, (char)210, (char)42, (char)177, (char)184, (char)166, (char)84, (char)153, (char)185, (char)178, (char)226, (char)178, (char)146, (char)88, (char)93, (char)187, (char)96, (char)40, (char)219, (char)189, (char)75, (char)224, (char)191, (char)62, (char)124, (char)209, (char)59, (char)190, (char)217, (char)3, (char)166, (char)102, (char)110, (char)29, (char)56, (char)151, (char)189, (char)202, (char)239, (char)35, (char)46, (char)74, (char)9, (char)107, (char)233, (char)85, (char)112, (char)148, (char)86, (char)115, (char)7, (char)48}, 0) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)72);
            assert(pack.target_system_GET() == (char)244);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)244) ;
        p121.target_component_SET((char)72) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)239);
            assert(pack.target_component_GET() == (char)248);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_component_SET((char)248) ;
        p122.target_system_SET((char)239) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)176);
            assert(pack.len_GET() == (char)184);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)232, (char)10, (char)168, (char)39, (char)204, (char)252, (char)148, (char)8, (char)164, (char)57, (char)68, (char)112, (char)51, (char)229, (char)33, (char)56, (char)164, (char)136, (char)58, (char)175, (char)110, (char)114, (char)62, (char)225, (char)198, (char)118, (char)11, (char)56, (char)154, (char)201, (char)235, (char)16, (char)103, (char)175, (char)62, (char)52, (char)44, (char)99, (char)151, (char)249, (char)95, (char)188, (char)234, (char)9, (char)232, (char)4, (char)184, (char)177, (char)99, (char)168, (char)35, (char)26, (char)187, (char)186, (char)176, (char)34, (char)27, (char)150, (char)51, (char)223, (char)47, (char)34, (char)182, (char)22, (char)158, (char)69, (char)114, (char)26, (char)194, (char)192, (char)29, (char)79, (char)91, (char)229, (char)201, (char)94, (char)159, (char)163, (char)174, (char)123, (char)42, (char)117, (char)4, (char)21, (char)47, (char)52, (char)40, (char)61, (char)251, (char)161, (char)26, (char)142, (char)219, (char)175, (char)251, (char)140, (char)26, (char)71, (char)212, (char)16, (char)175, (char)34, (char)54, (char)17, (char)144, (char)75, (char)148, (char)54, (char)152, (char)196}));
            assert(pack.target_component_GET() == (char)11);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.data__SET(new char[] {(char)232, (char)10, (char)168, (char)39, (char)204, (char)252, (char)148, (char)8, (char)164, (char)57, (char)68, (char)112, (char)51, (char)229, (char)33, (char)56, (char)164, (char)136, (char)58, (char)175, (char)110, (char)114, (char)62, (char)225, (char)198, (char)118, (char)11, (char)56, (char)154, (char)201, (char)235, (char)16, (char)103, (char)175, (char)62, (char)52, (char)44, (char)99, (char)151, (char)249, (char)95, (char)188, (char)234, (char)9, (char)232, (char)4, (char)184, (char)177, (char)99, (char)168, (char)35, (char)26, (char)187, (char)186, (char)176, (char)34, (char)27, (char)150, (char)51, (char)223, (char)47, (char)34, (char)182, (char)22, (char)158, (char)69, (char)114, (char)26, (char)194, (char)192, (char)29, (char)79, (char)91, (char)229, (char)201, (char)94, (char)159, (char)163, (char)174, (char)123, (char)42, (char)117, (char)4, (char)21, (char)47, (char)52, (char)40, (char)61, (char)251, (char)161, (char)26, (char)142, (char)219, (char)175, (char)251, (char)140, (char)26, (char)71, (char)212, (char)16, (char)175, (char)34, (char)54, (char)17, (char)144, (char)75, (char)148, (char)54, (char)152, (char)196}, 0) ;
        p123.target_system_SET((char)176) ;
        p123.len_SET((char)184) ;
        p123.target_component_SET((char)11) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
            assert(pack.satellites_visible_GET() == (char)245);
            assert(pack.lat_GET() == 1468186879);
            assert(pack.eph_GET() == (char)16124);
            assert(pack.lon_GET() == -1602682916);
            assert(pack.dgps_numch_GET() == (char)162);
            assert(pack.vel_GET() == (char)21849);
            assert(pack.alt_GET() == -164277691);
            assert(pack.cog_GET() == (char)58451);
            assert(pack.epv_GET() == (char)60480);
            assert(pack.time_usec_GET() == 5553809112639247797L);
            assert(pack.dgps_age_GET() == 1870765089L);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.satellites_visible_SET((char)245) ;
        p124.dgps_age_SET(1870765089L) ;
        p124.lat_SET(1468186879) ;
        p124.time_usec_SET(5553809112639247797L) ;
        p124.epv_SET((char)60480) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS) ;
        p124.cog_SET((char)58451) ;
        p124.vel_SET((char)21849) ;
        p124.eph_SET((char)16124) ;
        p124.alt_SET(-164277691) ;
        p124.dgps_numch_SET((char)162) ;
        p124.lon_SET(-1602682916) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID);
            assert(pack.Vservo_GET() == (char)43446);
            assert(pack.Vcc_GET() == (char)38837);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID) ;
        p125.Vservo_SET((char)43446) ;
        p125.Vcc_SET((char)38837) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)18, (char)247, (char)101, (char)24, (char)54, (char)207, (char)21, (char)147, (char)111, (char)94, (char)141, (char)10, (char)0, (char)172, (char)193, (char)165, (char)119, (char)181, (char)29, (char)35, (char)122, (char)147, (char)159, (char)65, (char)76, (char)90, (char)172, (char)253, (char)63, (char)231, (char)77, (char)154, (char)168, (char)42, (char)139, (char)250, (char)43, (char)168, (char)171, (char)37, (char)153, (char)184, (char)173, (char)14, (char)206, (char)251, (char)94, (char)113, (char)50, (char)127, (char)71, (char)94, (char)152, (char)214, (char)56, (char)0, (char)17, (char)0, (char)135, (char)108, (char)32, (char)134, (char)29, (char)7, (char)69, (char)47, (char)119, (char)200, (char)183, (char)253}));
            assert(pack.timeout_GET() == (char)52789);
            assert(pack.flags_GET() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
            assert(pack.count_GET() == (char)230);
            assert(pack.baudrate_GET() == 4118269295L);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL) ;
        p126.timeout_SET((char)52789) ;
        p126.baudrate_SET(4118269295L) ;
        p126.data__SET(new char[] {(char)18, (char)247, (char)101, (char)24, (char)54, (char)207, (char)21, (char)147, (char)111, (char)94, (char)141, (char)10, (char)0, (char)172, (char)193, (char)165, (char)119, (char)181, (char)29, (char)35, (char)122, (char)147, (char)159, (char)65, (char)76, (char)90, (char)172, (char)253, (char)63, (char)231, (char)77, (char)154, (char)168, (char)42, (char)139, (char)250, (char)43, (char)168, (char)171, (char)37, (char)153, (char)184, (char)173, (char)14, (char)206, (char)251, (char)94, (char)113, (char)50, (char)127, (char)71, (char)94, (char)152, (char)214, (char)56, (char)0, (char)17, (char)0, (char)135, (char)108, (char)32, (char)134, (char)29, (char)7, (char)69, (char)47, (char)119, (char)200, (char)183, (char)253}, 0) ;
        p126.count_SET((char)230) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_a_mm_GET() == 1658458262);
            assert(pack.rtk_rate_GET() == (char)196);
            assert(pack.wn_GET() == (char)41712);
            assert(pack.accuracy_GET() == 571253333L);
            assert(pack.baseline_coords_type_GET() == (char)95);
            assert(pack.rtk_receiver_id_GET() == (char)33);
            assert(pack.baseline_c_mm_GET() == -426745965);
            assert(pack.tow_GET() == 2022182812L);
            assert(pack.nsats_GET() == (char)75);
            assert(pack.baseline_b_mm_GET() == -866423537);
            assert(pack.time_last_baseline_ms_GET() == 770579464L);
            assert(pack.rtk_health_GET() == (char)58);
            assert(pack.iar_num_hypotheses_GET() == -928904408);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.baseline_b_mm_SET(-866423537) ;
        p127.time_last_baseline_ms_SET(770579464L) ;
        p127.baseline_a_mm_SET(1658458262) ;
        p127.iar_num_hypotheses_SET(-928904408) ;
        p127.tow_SET(2022182812L) ;
        p127.baseline_coords_type_SET((char)95) ;
        p127.rtk_health_SET((char)58) ;
        p127.accuracy_SET(571253333L) ;
        p127.baseline_c_mm_SET(-426745965) ;
        p127.rtk_receiver_id_SET((char)33) ;
        p127.rtk_rate_SET((char)196) ;
        p127.nsats_SET((char)75) ;
        p127.wn_SET((char)41712) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.rtk_receiver_id_GET() == (char)249);
            assert(pack.wn_GET() == (char)32236);
            assert(pack.tow_GET() == 214510571L);
            assert(pack.iar_num_hypotheses_GET() == 692370659);
            assert(pack.baseline_coords_type_GET() == (char)205);
            assert(pack.rtk_health_GET() == (char)198);
            assert(pack.baseline_a_mm_GET() == 880268829);
            assert(pack.time_last_baseline_ms_GET() == 655764953L);
            assert(pack.nsats_GET() == (char)214);
            assert(pack.baseline_c_mm_GET() == -158346236);
            assert(pack.rtk_rate_GET() == (char)208);
            assert(pack.accuracy_GET() == 3007936297L);
            assert(pack.baseline_b_mm_GET() == 673139207);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.nsats_SET((char)214) ;
        p128.rtk_rate_SET((char)208) ;
        p128.baseline_c_mm_SET(-158346236) ;
        p128.baseline_b_mm_SET(673139207) ;
        p128.tow_SET(214510571L) ;
        p128.baseline_coords_type_SET((char)205) ;
        p128.accuracy_SET(3007936297L) ;
        p128.time_last_baseline_ms_SET(655764953L) ;
        p128.rtk_receiver_id_SET((char)249) ;
        p128.rtk_health_SET((char)198) ;
        p128.iar_num_hypotheses_SET(692370659) ;
        p128.baseline_a_mm_SET(880268829) ;
        p128.wn_SET((char)32236) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short) -20216);
            assert(pack.xgyro_GET() == (short)31824);
            assert(pack.time_boot_ms_GET() == 948285090L);
            assert(pack.ymag_GET() == (short)31581);
            assert(pack.zmag_GET() == (short) -9581);
            assert(pack.zacc_GET() == (short) -1329);
            assert(pack.xacc_GET() == (short) -20757);
            assert(pack.xmag_GET() == (short)8014);
            assert(pack.zgyro_GET() == (short) -32154);
            assert(pack.yacc_GET() == (short)5613);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.xacc_SET((short) -20757) ;
        p129.ygyro_SET((short) -20216) ;
        p129.zacc_SET((short) -1329) ;
        p129.time_boot_ms_SET(948285090L) ;
        p129.ymag_SET((short)31581) ;
        p129.xmag_SET((short)8014) ;
        p129.yacc_SET((short)5613) ;
        p129.xgyro_SET((short)31824) ;
        p129.zgyro_SET((short) -32154) ;
        p129.zmag_SET((short) -9581) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.height_GET() == (char)30660);
            assert(pack.type_GET() == (char)132);
            assert(pack.size_GET() == 2299697992L);
            assert(pack.jpg_quality_GET() == (char)133);
            assert(pack.packets_GET() == (char)37736);
            assert(pack.width_GET() == (char)38178);
            assert(pack.payload_GET() == (char)21);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.packets_SET((char)37736) ;
        p130.jpg_quality_SET((char)133) ;
        p130.type_SET((char)132) ;
        p130.size_SET(2299697992L) ;
        p130.height_SET((char)30660) ;
        p130.width_SET((char)38178) ;
        p130.payload_SET((char)21) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)226, (char)111, (char)100, (char)91, (char)145, (char)157, (char)0, (char)155, (char)124, (char)175, (char)29, (char)116, (char)246, (char)80, (char)87, (char)222, (char)52, (char)139, (char)129, (char)33, (char)238, (char)165, (char)180, (char)74, (char)244, (char)223, (char)152, (char)183, (char)158, (char)209, (char)35, (char)41, (char)105, (char)115, (char)234, (char)147, (char)183, (char)16, (char)144, (char)23, (char)189, (char)89, (char)204, (char)159, (char)200, (char)208, (char)6, (char)23, (char)7, (char)24, (char)39, (char)94, (char)49, (char)4, (char)0, (char)43, (char)32, (char)106, (char)169, (char)145, (char)67, (char)42, (char)49, (char)6, (char)158, (char)245, (char)204, (char)147, (char)96, (char)62, (char)140, (char)100, (char)139, (char)207, (char)73, (char)164, (char)237, (char)195, (char)79, (char)160, (char)140, (char)26, (char)240, (char)115, (char)213, (char)239, (char)209, (char)23, (char)12, (char)157, (char)40, (char)54, (char)212, (char)202, (char)237, (char)119, (char)230, (char)121, (char)160, (char)22, (char)69, (char)111, (char)223, (char)47, (char)254, (char)151, (char)246, (char)97, (char)61, (char)159, (char)96, (char)184, (char)67, (char)77, (char)69, (char)241, (char)119, (char)86, (char)247, (char)201, (char)221, (char)233, (char)48, (char)218, (char)231, (char)97, (char)251, (char)227, (char)13, (char)106, (char)139, (char)70, (char)64, (char)8, (char)165, (char)148, (char)171, (char)130, (char)57, (char)179, (char)202, (char)138, (char)121, (char)69, (char)57, (char)63, (char)119, (char)74, (char)229, (char)229, (char)210, (char)184, (char)48, (char)146, (char)155, (char)75, (char)190, (char)65, (char)148, (char)254, (char)4, (char)246, (char)119, (char)42, (char)138, (char)71, (char)19, (char)36, (char)191, (char)134, (char)26, (char)230, (char)4, (char)189, (char)208, (char)194, (char)167, (char)33, (char)170, (char)90, (char)252, (char)13, (char)99, (char)201, (char)20, (char)121, (char)68, (char)12, (char)231, (char)214, (char)118, (char)199, (char)87, (char)145, (char)174, (char)98, (char)109, (char)96, (char)180, (char)17, (char)69, (char)68, (char)192, (char)198, (char)243, (char)206, (char)186, (char)237, (char)113, (char)208, (char)35, (char)163, (char)23, (char)126, (char)245, (char)67, (char)130, (char)25, (char)224, (char)60, (char)203, (char)83, (char)46, (char)87, (char)234, (char)94, (char)235, (char)27, (char)223, (char)69, (char)1, (char)72, (char)177, (char)163, (char)206, (char)73, (char)2, (char)89, (char)253, (char)77, (char)222, (char)132, (char)217, (char)164, (char)182, (char)26, (char)23, (char)100, (char)239, (char)129, (char)22, (char)169, (char)204}));
            assert(pack.seqnr_GET() == (char)15885);
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)15885) ;
        p131.data__SET(new char[] {(char)226, (char)111, (char)100, (char)91, (char)145, (char)157, (char)0, (char)155, (char)124, (char)175, (char)29, (char)116, (char)246, (char)80, (char)87, (char)222, (char)52, (char)139, (char)129, (char)33, (char)238, (char)165, (char)180, (char)74, (char)244, (char)223, (char)152, (char)183, (char)158, (char)209, (char)35, (char)41, (char)105, (char)115, (char)234, (char)147, (char)183, (char)16, (char)144, (char)23, (char)189, (char)89, (char)204, (char)159, (char)200, (char)208, (char)6, (char)23, (char)7, (char)24, (char)39, (char)94, (char)49, (char)4, (char)0, (char)43, (char)32, (char)106, (char)169, (char)145, (char)67, (char)42, (char)49, (char)6, (char)158, (char)245, (char)204, (char)147, (char)96, (char)62, (char)140, (char)100, (char)139, (char)207, (char)73, (char)164, (char)237, (char)195, (char)79, (char)160, (char)140, (char)26, (char)240, (char)115, (char)213, (char)239, (char)209, (char)23, (char)12, (char)157, (char)40, (char)54, (char)212, (char)202, (char)237, (char)119, (char)230, (char)121, (char)160, (char)22, (char)69, (char)111, (char)223, (char)47, (char)254, (char)151, (char)246, (char)97, (char)61, (char)159, (char)96, (char)184, (char)67, (char)77, (char)69, (char)241, (char)119, (char)86, (char)247, (char)201, (char)221, (char)233, (char)48, (char)218, (char)231, (char)97, (char)251, (char)227, (char)13, (char)106, (char)139, (char)70, (char)64, (char)8, (char)165, (char)148, (char)171, (char)130, (char)57, (char)179, (char)202, (char)138, (char)121, (char)69, (char)57, (char)63, (char)119, (char)74, (char)229, (char)229, (char)210, (char)184, (char)48, (char)146, (char)155, (char)75, (char)190, (char)65, (char)148, (char)254, (char)4, (char)246, (char)119, (char)42, (char)138, (char)71, (char)19, (char)36, (char)191, (char)134, (char)26, (char)230, (char)4, (char)189, (char)208, (char)194, (char)167, (char)33, (char)170, (char)90, (char)252, (char)13, (char)99, (char)201, (char)20, (char)121, (char)68, (char)12, (char)231, (char)214, (char)118, (char)199, (char)87, (char)145, (char)174, (char)98, (char)109, (char)96, (char)180, (char)17, (char)69, (char)68, (char)192, (char)198, (char)243, (char)206, (char)186, (char)237, (char)113, (char)208, (char)35, (char)163, (char)23, (char)126, (char)245, (char)67, (char)130, (char)25, (char)224, (char)60, (char)203, (char)83, (char)46, (char)87, (char)234, (char)94, (char)235, (char)27, (char)223, (char)69, (char)1, (char)72, (char)177, (char)163, (char)206, (char)73, (char)2, (char)89, (char)253, (char)77, (char)222, (char)132, (char)217, (char)164, (char)182, (char)26, (char)23, (char)100, (char)239, (char)129, (char)22, (char)169, (char)204}, 0) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_PITCH_270);
            assert(pack.max_distance_GET() == (char)52945);
            assert(pack.id_GET() == (char)198);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
            assert(pack.min_distance_GET() == (char)32274);
            assert(pack.current_distance_GET() == (char)36950);
            assert(pack.covariance_GET() == (char)144);
            assert(pack.time_boot_ms_GET() == 716740837L);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.id_SET((char)198) ;
        p132.covariance_SET((char)144) ;
        p132.max_distance_SET((char)52945) ;
        p132.current_distance_SET((char)36950) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_PITCH_270) ;
        p132.time_boot_ms_SET(716740837L) ;
        p132.min_distance_SET((char)32274) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -1681491662);
            assert(pack.mask_GET() == 1019678430242216543L);
            assert(pack.grid_spacing_GET() == (char)59273);
            assert(pack.lat_GET() == 1423448633);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(1423448633) ;
        p133.mask_SET(1019678430242216543L) ;
        p133.grid_spacing_SET((char)59273) ;
        p133.lon_SET(-1681491662) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -28532, (short)17127, (short) -30424, (short)9456, (short) -24786, (short)14620, (short)691, (short)11812, (short) -21191, (short) -30789, (short) -6155, (short) -14236, (short) -23816, (short)26720, (short)5642, (short) -5035}));
            assert(pack.gridbit_GET() == (char)199);
            assert(pack.grid_spacing_GET() == (char)11051);
            assert(pack.lat_GET() == 1544481390);
            assert(pack.lon_GET() == -888018348);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lon_SET(-888018348) ;
        p134.lat_SET(1544481390) ;
        p134.gridbit_SET((char)199) ;
        p134.grid_spacing_SET((char)11051) ;
        p134.data__SET(new short[] {(short) -28532, (short)17127, (short) -30424, (short)9456, (short) -24786, (short)14620, (short)691, (short)11812, (short) -21191, (short) -30789, (short) -6155, (short) -14236, (short) -23816, (short)26720, (short)5642, (short) -5035}, 0) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 340311187);
            assert(pack.lat_GET() == -2037666619);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(-2037666619) ;
        p135.lon_SET(340311187) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.spacing_GET() == (char)40918);
            assert(pack.terrain_height_GET() == -2.3626484E38F);
            assert(pack.pending_GET() == (char)45122);
            assert(pack.loaded_GET() == (char)337);
            assert(pack.lat_GET() == -1094318686);
            assert(pack.current_height_GET() == 2.8643774E38F);
            assert(pack.lon_GET() == 1227405655);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.terrain_height_SET(-2.3626484E38F) ;
        p136.current_height_SET(2.8643774E38F) ;
        p136.loaded_SET((char)337) ;
        p136.lat_SET(-1094318686) ;
        p136.lon_SET(1227405655) ;
        p136.pending_SET((char)45122) ;
        p136.spacing_SET((char)40918) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3102165418L);
            assert(pack.temperature_GET() == (short)18069);
            assert(pack.press_abs_GET() == -1.7979153E38F);
            assert(pack.press_diff_GET() == 2.5491192E38F);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.temperature_SET((short)18069) ;
        p137.press_abs_SET(-1.7979153E38F) ;
        p137.time_boot_ms_SET(3102165418L) ;
        p137.press_diff_SET(2.5491192E38F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 3.0248466E38F);
            assert(pack.time_usec_GET() == 4828117413983261723L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.3551495E38F, 2.1432828E38F, -2.551742E38F, 5.077937E37F}));
            assert(pack.z_GET() == -1.1198938E38F);
            assert(pack.x_GET() == 3.324705E38F);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.q_SET(new float[] {3.3551495E38F, 2.1432828E38F, -2.551742E38F, 5.077937E37F}, 0) ;
        p138.y_SET(3.0248466E38F) ;
        p138.time_usec_SET(4828117413983261723L) ;
        p138.z_SET(-1.1198938E38F) ;
        p138.x_SET(3.324705E38F) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)172);
            assert(pack.time_usec_GET() == 7416522125173902988L);
            assert(pack.target_component_GET() == (char)120);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-6.9255763E37F, -7.123749E37F, -1.1407287E38F, 2.9940875E38F, 2.9795604E38F, -1.1462773E38F, -2.0130018E38F, -1.899574E38F}));
            assert(pack.group_mlx_GET() == (char)170);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(7416522125173902988L) ;
        p139.group_mlx_SET((char)170) ;
        p139.target_component_SET((char)120) ;
        p139.controls_SET(new float[] {-6.9255763E37F, -7.123749E37F, -1.1407287E38F, 2.9940875E38F, 2.9795604E38F, -1.1462773E38F, -2.0130018E38F, -1.899574E38F}, 0) ;
        p139.target_system_SET((char)172) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7684546447097947386L);
            assert(pack.group_mlx_GET() == (char)161);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.6229723E38F, -2.8927697E38F, 1.7602716E38F, -2.084363E38F, -1.1401419E38F, 2.895306E38F, -6.754428E36F, -2.672664E38F}));
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(7684546447097947386L) ;
        p140.controls_SET(new float[] {2.6229723E38F, -2.8927697E38F, 1.7602716E38F, -2.084363E38F, -1.1401419E38F, 2.895306E38F, -6.754428E36F, -2.672664E38F}, 0) ;
        p140.group_mlx_SET((char)161) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.bottom_clearance_GET() == 9.493844E37F);
            assert(pack.time_usec_GET() == 1338817607517914950L);
            assert(pack.altitude_monotonic_GET() == 1.5067877E38F);
            assert(pack.altitude_amsl_GET() == 3.3657957E38F);
            assert(pack.altitude_terrain_GET() == 1.7438854E38F);
            assert(pack.altitude_local_GET() == 2.691372E38F);
            assert(pack.altitude_relative_GET() == -1.6260437E38F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.time_usec_SET(1338817607517914950L) ;
        p141.altitude_monotonic_SET(1.5067877E38F) ;
        p141.bottom_clearance_SET(9.493844E37F) ;
        p141.altitude_terrain_SET(1.7438854E38F) ;
        p141.altitude_local_SET(2.691372E38F) ;
        p141.altitude_amsl_SET(3.3657957E38F) ;
        p141.altitude_relative_SET(-1.6260437E38F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)195, (char)240, (char)246, (char)244, (char)132, (char)15, (char)181, (char)110, (char)254, (char)138, (char)185, (char)114, (char)8, (char)128, (char)101, (char)147, (char)221, (char)35, (char)86, (char)246, (char)246, (char)209, (char)100, (char)209, (char)112, (char)49, (char)79, (char)241, (char)213, (char)237, (char)77, (char)108, (char)194, (char)168, (char)64, (char)240, (char)31, (char)237, (char)214, (char)106, (char)202, (char)112, (char)61, (char)114, (char)8, (char)33, (char)83, (char)7, (char)6, (char)20, (char)150, (char)110, (char)180, (char)121, (char)123, (char)61, (char)230, (char)155, (char)81, (char)180, (char)4, (char)53, (char)201, (char)236, (char)57, (char)39, (char)251, (char)140, (char)236, (char)200, (char)167, (char)70, (char)18, (char)83, (char)50, (char)215, (char)8, (char)26, (char)108, (char)254, (char)4, (char)234, (char)227, (char)147, (char)49, (char)2, (char)165, (char)136, (char)49, (char)145, (char)187, (char)188, (char)242, (char)28, (char)177, (char)113, (char)229, (char)28, (char)95, (char)183, (char)34, (char)54, (char)74, (char)228, (char)58, (char)223, (char)112, (char)78, (char)133, (char)147, (char)108, (char)144, (char)205, (char)255, (char)110, (char)45, (char)215, (char)61, (char)235, (char)3}));
            assert(pack.uri_type_GET() == (char)66);
            assert(pack.transfer_type_GET() == (char)156);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)138, (char)27, (char)255, (char)244, (char)178, (char)190, (char)180, (char)184, (char)154, (char)175, (char)0, (char)167, (char)185, (char)92, (char)23, (char)237, (char)145, (char)20, (char)217, (char)255, (char)189, (char)84, (char)199, (char)103, (char)76, (char)197, (char)130, (char)56, (char)197, (char)242, (char)122, (char)63, (char)3, (char)206, (char)176, (char)160, (char)102, (char)176, (char)144, (char)213, (char)203, (char)138, (char)78, (char)106, (char)225, (char)190, (char)56, (char)24, (char)59, (char)244, (char)47, (char)111, (char)129, (char)12, (char)131, (char)255, (char)153, (char)186, (char)147, (char)7, (char)215, (char)240, (char)113, (char)52, (char)192, (char)90, (char)129, (char)139, (char)133, (char)97, (char)247, (char)200, (char)57, (char)33, (char)63, (char)115, (char)201, (char)180, (char)5, (char)208, (char)34, (char)115, (char)22, (char)174, (char)240, (char)148, (char)57, (char)165, (char)108, (char)39, (char)93, (char)177, (char)252, (char)63, (char)60, (char)22, (char)75, (char)171, (char)156, (char)56, (char)11, (char)105, (char)183, (char)242, (char)63, (char)161, (char)199, (char)151, (char)129, (char)239, (char)13, (char)215, (char)74, (char)250, (char)32, (char)22, (char)214, (char)135, (char)36, (char)111}));
            assert(pack.request_id_GET() == (char)60);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.uri_SET(new char[] {(char)195, (char)240, (char)246, (char)244, (char)132, (char)15, (char)181, (char)110, (char)254, (char)138, (char)185, (char)114, (char)8, (char)128, (char)101, (char)147, (char)221, (char)35, (char)86, (char)246, (char)246, (char)209, (char)100, (char)209, (char)112, (char)49, (char)79, (char)241, (char)213, (char)237, (char)77, (char)108, (char)194, (char)168, (char)64, (char)240, (char)31, (char)237, (char)214, (char)106, (char)202, (char)112, (char)61, (char)114, (char)8, (char)33, (char)83, (char)7, (char)6, (char)20, (char)150, (char)110, (char)180, (char)121, (char)123, (char)61, (char)230, (char)155, (char)81, (char)180, (char)4, (char)53, (char)201, (char)236, (char)57, (char)39, (char)251, (char)140, (char)236, (char)200, (char)167, (char)70, (char)18, (char)83, (char)50, (char)215, (char)8, (char)26, (char)108, (char)254, (char)4, (char)234, (char)227, (char)147, (char)49, (char)2, (char)165, (char)136, (char)49, (char)145, (char)187, (char)188, (char)242, (char)28, (char)177, (char)113, (char)229, (char)28, (char)95, (char)183, (char)34, (char)54, (char)74, (char)228, (char)58, (char)223, (char)112, (char)78, (char)133, (char)147, (char)108, (char)144, (char)205, (char)255, (char)110, (char)45, (char)215, (char)61, (char)235, (char)3}, 0) ;
        p142.transfer_type_SET((char)156) ;
        p142.uri_type_SET((char)66) ;
        p142.storage_SET(new char[] {(char)138, (char)27, (char)255, (char)244, (char)178, (char)190, (char)180, (char)184, (char)154, (char)175, (char)0, (char)167, (char)185, (char)92, (char)23, (char)237, (char)145, (char)20, (char)217, (char)255, (char)189, (char)84, (char)199, (char)103, (char)76, (char)197, (char)130, (char)56, (char)197, (char)242, (char)122, (char)63, (char)3, (char)206, (char)176, (char)160, (char)102, (char)176, (char)144, (char)213, (char)203, (char)138, (char)78, (char)106, (char)225, (char)190, (char)56, (char)24, (char)59, (char)244, (char)47, (char)111, (char)129, (char)12, (char)131, (char)255, (char)153, (char)186, (char)147, (char)7, (char)215, (char)240, (char)113, (char)52, (char)192, (char)90, (char)129, (char)139, (char)133, (char)97, (char)247, (char)200, (char)57, (char)33, (char)63, (char)115, (char)201, (char)180, (char)5, (char)208, (char)34, (char)115, (char)22, (char)174, (char)240, (char)148, (char)57, (char)165, (char)108, (char)39, (char)93, (char)177, (char)252, (char)63, (char)60, (char)22, (char)75, (char)171, (char)156, (char)56, (char)11, (char)105, (char)183, (char)242, (char)63, (char)161, (char)199, (char)151, (char)129, (char)239, (char)13, (char)215, (char)74, (char)250, (char)32, (char)22, (char)214, (char)135, (char)36, (char)111}, 0) ;
        p142.request_id_SET((char)60) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2921678973L);
            assert(pack.temperature_GET() == (short)16199);
            assert(pack.press_abs_GET() == -2.9837155E38F);
            assert(pack.press_diff_GET() == -7.3179664E37F);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(2921678973L) ;
        p143.press_abs_SET(-2.9837155E38F) ;
        p143.press_diff_SET(-7.3179664E37F) ;
        p143.temperature_SET((short)16199) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.vel_GET(),  new float[] {3.0562346E38F, -1.757871E38F, -1.6602184E38F}));
            assert(pack.alt_GET() == 3.0906395E38F);
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {2.9010756E38F, 2.1009272E38F, -6.0311373E37F, 1.998124E38F}));
            assert(pack.custom_state_GET() == 2810534882703400238L);
            assert(pack.lat_GET() == 1860837519);
            assert(pack.lon_GET() == -972232340);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-1.7215172E38F, 3.606444E37F, 1.0051706E38F}));
            assert(Arrays.equals(pack.rates_GET(),  new float[] {6.637716E37F, -1.7843776E38F, -2.9569678E38F}));
            assert(pack.est_capabilities_GET() == (char)137);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {1.0992703E38F, -1.174839E38F, 3.183242E38F}));
            assert(pack.timestamp_GET() == 9164549351269361162L);
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.alt_SET(3.0906395E38F) ;
        p144.attitude_q_SET(new float[] {2.9010756E38F, 2.1009272E38F, -6.0311373E37F, 1.998124E38F}, 0) ;
        p144.rates_SET(new float[] {6.637716E37F, -1.7843776E38F, -2.9569678E38F}, 0) ;
        p144.timestamp_SET(9164549351269361162L) ;
        p144.lat_SET(1860837519) ;
        p144.position_cov_SET(new float[] {-1.7215172E38F, 3.606444E37F, 1.0051706E38F}, 0) ;
        p144.lon_SET(-972232340) ;
        p144.vel_SET(new float[] {3.0562346E38F, -1.757871E38F, -1.6602184E38F}, 0) ;
        p144.est_capabilities_SET((char)137) ;
        p144.custom_state_SET(2810534882703400238L) ;
        p144.acc_SET(new float[] {1.0992703E38F, -1.174839E38F, 3.183242E38F}, 0) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.roll_rate_GET() == 2.396302E38F);
            assert(pack.airspeed_GET() == 2.1219299E38F);
            assert(pack.y_vel_GET() == 2.6471664E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {1.0344796E36F, -5.7122907E37F, -2.7967662E38F}));
            assert(pack.x_vel_GET() == 6.95482E37F);
            assert(pack.time_usec_GET() == 2482981360808779136L);
            assert(pack.y_acc_GET() == -2.7710864E38F);
            assert(pack.x_pos_GET() == -2.4222205E38F);
            assert(pack.z_vel_GET() == 3.4200777E37F);
            assert(pack.yaw_rate_GET() == -3.3182336E38F);
            assert(pack.x_acc_GET() == -7.2743754E37F);
            assert(pack.z_pos_GET() == 2.6187958E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {-3.3527543E37F, -2.2219534E38F, -2.7046437E38F}));
            assert(pack.z_acc_GET() == -1.2914395E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.7328208E38F, 3.2296827E37F, 7.4166326E36F, -2.2893618E38F}));
            assert(pack.y_pos_GET() == -1.8027067E37F);
            assert(pack.pitch_rate_GET() == -2.8478318E38F);
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.y_pos_SET(-1.8027067E37F) ;
        p146.roll_rate_SET(2.396302E38F) ;
        p146.x_vel_SET(6.95482E37F) ;
        p146.pitch_rate_SET(-2.8478318E38F) ;
        p146.z_acc_SET(-1.2914395E38F) ;
        p146.time_usec_SET(2482981360808779136L) ;
        p146.z_vel_SET(3.4200777E37F) ;
        p146.q_SET(new float[] {-2.7328208E38F, 3.2296827E37F, 7.4166326E36F, -2.2893618E38F}, 0) ;
        p146.x_pos_SET(-2.4222205E38F) ;
        p146.y_vel_SET(2.6471664E38F) ;
        p146.pos_variance_SET(new float[] {1.0344796E36F, -5.7122907E37F, -2.7967662E38F}, 0) ;
        p146.airspeed_SET(2.1219299E38F) ;
        p146.y_acc_SET(-2.7710864E38F) ;
        p146.x_acc_SET(-7.2743754E37F) ;
        p146.yaw_rate_SET(-3.3182336E38F) ;
        p146.vel_variance_SET(new float[] {-3.3527543E37F, -2.2219534E38F, -2.7046437E38F}, 0) ;
        p146.z_pos_SET(2.6187958E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)54613, (char)7801, (char)14372, (char)46305, (char)43548, (char)50669, (char)20742, (char)10826, (char)37101, (char)47390}));
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
            assert(pack.energy_consumed_GET() == 954216599);
            assert(pack.temperature_GET() == (short) -6904);
            assert(pack.current_consumed_GET() == 1557622967);
            assert(pack.id_GET() == (char)124);
            assert(pack.current_battery_GET() == (short)14962);
            assert(pack.battery_remaining_GET() == (byte)9);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.voltages_SET(new char[] {(char)54613, (char)7801, (char)14372, (char)46305, (char)43548, (char)50669, (char)20742, (char)10826, (char)37101, (char)47390}, 0) ;
        p147.battery_remaining_SET((byte)9) ;
        p147.id_SET((char)124) ;
        p147.temperature_SET((short) -6904) ;
        p147.current_consumed_SET(1557622967) ;
        p147.current_battery_SET((short)14962) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO) ;
        p147.energy_consumed_SET(954216599) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)217, (char)159, (char)202, (char)98, (char)147, (char)1, (char)21, (char)143, (char)101, (char)77, (char)166, (char)211, (char)185, (char)237, (char)152, (char)53, (char)249, (char)109}));
            assert(pack.board_version_GET() == 2958911656L);
            assert(pack.os_sw_version_GET() == 362080044L);
            assert(pack.middleware_sw_version_GET() == 2556681066L);
            assert(pack.uid_GET() == 7395766475926747666L);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)221, (char)139, (char)38, (char)7, (char)171, (char)157, (char)125, (char)71}));
            assert(pack.vendor_id_GET() == (char)53754);
            assert(pack.flight_sw_version_GET() == 3453238486L);
            assert(pack.product_id_GET() == (char)923);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)100, (char)74, (char)179, (char)110, (char)80, (char)80, (char)52, (char)108}));
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)136, (char)39, (char)218, (char)108, (char)187, (char)202, (char)172, (char)152}));
            assert(pack.capabilities_GET() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.uid2_SET(new char[] {(char)217, (char)159, (char)202, (char)98, (char)147, (char)1, (char)21, (char)143, (char)101, (char)77, (char)166, (char)211, (char)185, (char)237, (char)152, (char)53, (char)249, (char)109}, 0, PH) ;
        p148.vendor_id_SET((char)53754) ;
        p148.os_custom_version_SET(new char[] {(char)136, (char)39, (char)218, (char)108, (char)187, (char)202, (char)172, (char)152}, 0) ;
        p148.middleware_sw_version_SET(2556681066L) ;
        p148.flight_sw_version_SET(3453238486L) ;
        p148.os_sw_version_SET(362080044L) ;
        p148.board_version_SET(2958911656L) ;
        p148.product_id_SET((char)923) ;
        p148.uid_SET(7395766475926747666L) ;
        p148.middleware_custom_version_SET(new char[] {(char)221, (char)139, (char)38, (char)7, (char)171, (char)157, (char)125, (char)71}, 0) ;
        p148.flight_custom_version_SET(new char[] {(char)100, (char)74, (char)179, (char)110, (char)80, (char)80, (char)52, (char)108}, 0) ;
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {2.3964935E38F, 1.4094347E38F, -2.5532607E38F, -2.4913123E38F}));
            assert(pack.y_TRY(ph) == -1.7524878E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.size_x_GET() == 2.1805189E38F);
            assert(pack.angle_y_GET() == 1.2314678E37F);
            assert(pack.position_valid_TRY(ph) == (char)223);
            assert(pack.x_TRY(ph) == 2.9904054E38F);
            assert(pack.angle_x_GET() == -1.5286179E38F);
            assert(pack.target_num_GET() == (char)213);
            assert(pack.time_usec_GET() == 3214691597183853606L);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
            assert(pack.z_TRY(ph) == 1.4754546E38F);
            assert(pack.distance_GET() == 2.2592404E37F);
            assert(pack.size_y_GET() == -3.0627221E38F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.x_SET(2.9904054E38F, PH) ;
        p149.y_SET(-1.7524878E38F, PH) ;
        p149.distance_SET(2.2592404E37F) ;
        p149.q_SET(new float[] {2.3964935E38F, 1.4094347E38F, -2.5532607E38F, -2.4913123E38F}, 0, PH) ;
        p149.time_usec_SET(3214691597183853606L) ;
        p149.z_SET(1.4754546E38F, PH) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p149.size_x_SET(2.1805189E38F) ;
        p149.angle_x_SET(-1.5286179E38F) ;
        p149.size_y_SET(-3.0627221E38F) ;
        p149.target_num_SET((char)213) ;
        p149.angle_y_SET(1.2314678E37F) ;
        p149.position_valid_SET((char)223, PH) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_SENSOR_OFFSETS.add((src, ph, pack) ->
        {
            assert(pack.gyro_cal_x_GET() == -5.3570027E37F);
            assert(pack.raw_temp_GET() == -2073724113);
            assert(pack.accel_cal_y_GET() == -5.5459486E37F);
            assert(pack.accel_cal_x_GET() == -2.9636487E38F);
            assert(pack.mag_ofs_x_GET() == (short) -20362);
            assert(pack.gyro_cal_z_GET() == 1.0743122E38F);
            assert(pack.mag_ofs_z_GET() == (short)25802);
            assert(pack.mag_ofs_y_GET() == (short) -9180);
            assert(pack.raw_press_GET() == 356559235);
            assert(pack.accel_cal_z_GET() == -6.667065E37F);
            assert(pack.gyro_cal_y_GET() == -7.1686113E37F);
            assert(pack.mag_declination_GET() == 1.6377227E38F);
        });
        GroundControl.SENSOR_OFFSETS p150 = CommunicationChannel.new_SENSOR_OFFSETS();
        PH.setPack(p150);
        p150.mag_declination_SET(1.6377227E38F) ;
        p150.gyro_cal_z_SET(1.0743122E38F) ;
        p150.gyro_cal_y_SET(-7.1686113E37F) ;
        p150.mag_ofs_z_SET((short)25802) ;
        p150.accel_cal_z_SET(-6.667065E37F) ;
        p150.mag_ofs_y_SET((short) -9180) ;
        p150.gyro_cal_x_SET(-5.3570027E37F) ;
        p150.accel_cal_x_SET(-2.9636487E38F) ;
        p150.raw_temp_SET(-2073724113) ;
        p150.mag_ofs_x_SET((short) -20362) ;
        p150.accel_cal_y_SET(-5.5459486E37F) ;
        p150.raw_press_SET(356559235) ;
        CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_MAG_OFFSETS.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)162);
            assert(pack.mag_ofs_y_GET() == (short) -8532);
            assert(pack.mag_ofs_x_GET() == (short)22755);
            assert(pack.target_system_GET() == (char)172);
            assert(pack.mag_ofs_z_GET() == (short) -26693);
        });
        GroundControl.SET_MAG_OFFSETS p151 = CommunicationChannel.new_SET_MAG_OFFSETS();
        PH.setPack(p151);
        p151.mag_ofs_y_SET((short) -8532) ;
        p151.target_component_SET((char)162) ;
        p151.mag_ofs_x_SET((short)22755) ;
        p151.mag_ofs_z_SET((short) -26693) ;
        p151.target_system_SET((char)172) ;
        CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MEMINFO.add((src, ph, pack) ->
        {
            assert(pack.freemem32_TRY(ph) == 1029772119L);
            assert(pack.freemem_GET() == (char)14052);
            assert(pack.brkval_GET() == (char)12194);
        });
        GroundControl.MEMINFO p152 = CommunicationChannel.new_MEMINFO();
        PH.setPack(p152);
        p152.freemem_SET((char)14052) ;
        p152.freemem32_SET(1029772119L, PH) ;
        p152.brkval_SET((char)12194) ;
        CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AP_ADC.add((src, ph, pack) ->
        {
            assert(pack.adc1_GET() == (char)64143);
            assert(pack.adc2_GET() == (char)25297);
            assert(pack.adc4_GET() == (char)65442);
            assert(pack.adc6_GET() == (char)41508);
            assert(pack.adc5_GET() == (char)50672);
            assert(pack.adc3_GET() == (char)55908);
        });
        GroundControl.AP_ADC p153 = CommunicationChannel.new_AP_ADC();
        PH.setPack(p153);
        p153.adc5_SET((char)50672) ;
        p153.adc6_SET((char)41508) ;
        p153.adc4_SET((char)65442) ;
        p153.adc3_SET((char)55908) ;
        p153.adc1_SET((char)64143) ;
        p153.adc2_SET((char)25297) ;
        CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DIGICAM_CONFIGURE.add((src, ph, pack) ->
        {
            assert(pack.aperture_GET() == (char)155);
            assert(pack.engine_cut_off_GET() == (char)74);
            assert(pack.target_system_GET() == (char)172);
            assert(pack.mode_GET() == (char)178);
            assert(pack.extra_param_GET() == (char)144);
            assert(pack.shutter_speed_GET() == (char)15227);
            assert(pack.extra_value_GET() == -2.3850114E38F);
            assert(pack.exposure_type_GET() == (char)199);
            assert(pack.target_component_GET() == (char)240);
            assert(pack.command_id_GET() == (char)238);
            assert(pack.iso_GET() == (char)159);
        });
        GroundControl.DIGICAM_CONFIGURE p154 = CommunicationChannel.new_DIGICAM_CONFIGURE();
        PH.setPack(p154);
        p154.extra_value_SET(-2.3850114E38F) ;
        p154.iso_SET((char)159) ;
        p154.mode_SET((char)178) ;
        p154.command_id_SET((char)238) ;
        p154.exposure_type_SET((char)199) ;
        p154.extra_param_SET((char)144) ;
        p154.aperture_SET((char)155) ;
        p154.shutter_speed_SET((char)15227) ;
        p154.target_component_SET((char)240) ;
        p154.target_system_SET((char)172) ;
        p154.engine_cut_off_SET((char)74) ;
        CommunicationChannel.instance.send(p154);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DIGICAM_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.zoom_step_GET() == (byte) - 23);
            assert(pack.shot_GET() == (char)207);
            assert(pack.extra_value_GET() == 7.2247327E37F);
            assert(pack.target_system_GET() == (char)241);
            assert(pack.command_id_GET() == (char)185);
            assert(pack.focus_lock_GET() == (char)220);
            assert(pack.session_GET() == (char)207);
            assert(pack.target_component_GET() == (char)26);
            assert(pack.zoom_pos_GET() == (char)175);
            assert(pack.extra_param_GET() == (char)98);
        });
        GroundControl.DIGICAM_CONTROL p155 = CommunicationChannel.new_DIGICAM_CONTROL();
        PH.setPack(p155);
        p155.command_id_SET((char)185) ;
        p155.focus_lock_SET((char)220) ;
        p155.target_component_SET((char)26) ;
        p155.shot_SET((char)207) ;
        p155.target_system_SET((char)241) ;
        p155.zoom_step_SET((byte) - 23) ;
        p155.session_SET((char)207) ;
        p155.extra_param_SET((char)98) ;
        p155.extra_value_SET(7.2247327E37F) ;
        p155.zoom_pos_SET((char)175) ;
        CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_CONFIGURE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)141);
            assert(pack.stab_roll_GET() == (char)47);
            assert(pack.stab_pitch_GET() == (char)58);
            assert(pack.stab_yaw_GET() == (char)189);
            assert(pack.target_component_GET() == (char)203);
            assert(pack.mount_mode_GET() == MAV_MOUNT_MODE.MAV_MOUNT_MODE_GPS_POINT);
        });
        GroundControl.MOUNT_CONFIGURE p156 = CommunicationChannel.new_MOUNT_CONFIGURE();
        PH.setPack(p156);
        p156.stab_pitch_SET((char)58) ;
        p156.mount_mode_SET(MAV_MOUNT_MODE.MAV_MOUNT_MODE_GPS_POINT) ;
        p156.stab_roll_SET((char)47) ;
        p156.target_component_SET((char)203) ;
        p156.target_system_SET((char)141) ;
        p156.stab_yaw_SET((char)189) ;
        CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)252);
            assert(pack.input_b_GET() == 1301805);
            assert(pack.input_a_GET() == 196566035);
            assert(pack.input_c_GET() == 1118778687);
            assert(pack.target_system_GET() == (char)205);
            assert(pack.save_position_GET() == (char)128);
        });
        GroundControl.MOUNT_CONTROL p157 = CommunicationChannel.new_MOUNT_CONTROL();
        PH.setPack(p157);
        p157.target_component_SET((char)252) ;
        p157.target_system_SET((char)205) ;
        p157.input_c_SET(1118778687) ;
        p157.input_a_SET(196566035) ;
        p157.input_b_SET(1301805) ;
        p157.save_position_SET((char)128) ;
        CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pointing_a_GET() == -1093752661);
            assert(pack.target_system_GET() == (char)233);
            assert(pack.pointing_b_GET() == -1016265932);
            assert(pack.target_component_GET() == (char)232);
            assert(pack.pointing_c_GET() == 1895902726);
        });
        GroundControl.MOUNT_STATUS p158 = CommunicationChannel.new_MOUNT_STATUS();
        PH.setPack(p158);
        p158.pointing_a_SET(-1093752661) ;
        p158.pointing_b_SET(-1016265932) ;
        p158.target_component_SET((char)232) ;
        p158.target_system_SET((char)233) ;
        p158.pointing_c_SET(1895902726) ;
        CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FENCE_POINT.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)182);
            assert(pack.target_system_GET() == (char)200);
            assert(pack.target_component_GET() == (char)53);
            assert(pack.lng_GET() == -7.0218963E36F);
            assert(pack.idx_GET() == (char)74);
            assert(pack.lat_GET() == -3.2284483E38F);
        });
        GroundControl.FENCE_POINT p160 = CommunicationChannel.new_FENCE_POINT();
        PH.setPack(p160);
        p160.count_SET((char)182) ;
        p160.idx_SET((char)74) ;
        p160.lng_SET(-7.0218963E36F) ;
        p160.lat_SET(-3.2284483E38F) ;
        p160.target_system_SET((char)200) ;
        p160.target_component_SET((char)53) ;
        CommunicationChannel.instance.send(p160);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FENCE_FETCH_POINT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)176);
            assert(pack.idx_GET() == (char)124);
            assert(pack.target_component_GET() == (char)143);
        });
        GroundControl.FENCE_FETCH_POINT p161 = CommunicationChannel.new_FENCE_FETCH_POINT();
        PH.setPack(p161);
        p161.idx_SET((char)124) ;
        p161.target_component_SET((char)143) ;
        p161.target_system_SET((char)176) ;
        CommunicationChannel.instance.send(p161);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FENCE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.breach_count_GET() == (char)4661);
            assert(pack.breach_time_GET() == 3344578881L);
            assert(pack.breach_status_GET() == (char)137);
            assert(pack.breach_type_GET() == FENCE_BREACH.FENCE_BREACH_NONE);
        });
        GroundControl.FENCE_STATUS p162 = CommunicationChannel.new_FENCE_STATUS();
        PH.setPack(p162);
        p162.breach_type_SET(FENCE_BREACH.FENCE_BREACH_NONE) ;
        p162.breach_time_SET(3344578881L) ;
        p162.breach_count_SET((char)4661) ;
        p162.breach_status_SET((char)137) ;
        CommunicationChannel.instance.send(p162);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AHRS.add((src, ph, pack) ->
        {
            assert(pack.error_yaw_GET() == 3.2226783E38F);
            assert(pack.omegaIy_GET() == 1.0487581E38F);
            assert(pack.renorm_val_GET() == 1.2591557E38F);
            assert(pack.accel_weight_GET() == -3.416425E37F);
            assert(pack.error_rp_GET() == 3.2366024E38F);
            assert(pack.omegaIx_GET() == -1.1639753E38F);
            assert(pack.omegaIz_GET() == -2.6682905E38F);
        });
        GroundControl.AHRS p163 = CommunicationChannel.new_AHRS();
        PH.setPack(p163);
        p163.accel_weight_SET(-3.416425E37F) ;
        p163.omegaIx_SET(-1.1639753E38F) ;
        p163.error_yaw_SET(3.2226783E38F) ;
        p163.renorm_val_SET(1.2591557E38F) ;
        p163.error_rp_SET(3.2366024E38F) ;
        p163.omegaIz_SET(-2.6682905E38F) ;
        p163.omegaIy_SET(1.0487581E38F) ;
        CommunicationChannel.instance.send(p163);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SIMSTATE.add((src, ph, pack) ->
        {
            assert(pack.lng_GET() == 1146329606);
            assert(pack.zgyro_GET() == -9.684465E37F);
            assert(pack.yaw_GET() == 3.3702083E38F);
            assert(pack.zacc_GET() == -1.6026657E38F);
            assert(pack.yacc_GET() == -1.3536413E37F);
            assert(pack.xacc_GET() == -3.9973455E37F);
            assert(pack.ygyro_GET() == -3.018921E38F);
            assert(pack.lat_GET() == 2008279376);
            assert(pack.xgyro_GET() == 4.113813E37F);
            assert(pack.roll_GET() == -4.505599E37F);
            assert(pack.pitch_GET() == -1.944745E38F);
        });
        GroundControl.SIMSTATE p164 = CommunicationChannel.new_SIMSTATE();
        PH.setPack(p164);
        p164.roll_SET(-4.505599E37F) ;
        p164.lat_SET(2008279376) ;
        p164.xacc_SET(-3.9973455E37F) ;
        p164.zacc_SET(-1.6026657E38F) ;
        p164.yaw_SET(3.3702083E38F) ;
        p164.pitch_SET(-1.944745E38F) ;
        p164.zgyro_SET(-9.684465E37F) ;
        p164.lng_SET(1146329606) ;
        p164.ygyro_SET(-3.018921E38F) ;
        p164.xgyro_SET(4.113813E37F) ;
        p164.yacc_SET(-1.3536413E37F) ;
        CommunicationChannel.instance.send(p164);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HWSTATUS.add((src, ph, pack) ->
        {
            assert(pack.Vcc_GET() == (char)39681);
            assert(pack.I2Cerr_GET() == (char)84);
        });
        GroundControl.HWSTATUS p165 = CommunicationChannel.new_HWSTATUS();
        PH.setPack(p165);
        p165.I2Cerr_SET((char)84) ;
        p165.Vcc_SET((char)39681) ;
        CommunicationChannel.instance.send(p165);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RADIO.add((src, ph, pack) ->
        {
            assert(pack.rssi_GET() == (char)230);
            assert(pack.remrssi_GET() == (char)132);
            assert(pack.remnoise_GET() == (char)171);
            assert(pack.txbuf_GET() == (char)23);
            assert(pack.fixed__GET() == (char)26202);
            assert(pack.rxerrors_GET() == (char)52424);
            assert(pack.noise_GET() == (char)91);
        });
        GroundControl.RADIO p166 = CommunicationChannel.new_RADIO();
        PH.setPack(p166);
        p166.rxerrors_SET((char)52424) ;
        p166.fixed__SET((char)26202) ;
        p166.rssi_SET((char)230) ;
        p166.remrssi_SET((char)132) ;
        p166.remnoise_SET((char)171) ;
        p166.txbuf_SET((char)23) ;
        p166.noise_SET((char)91) ;
        CommunicationChannel.instance.send(p166);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LIMITS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.mods_required_GET() == LIMIT_MODULE.LIMIT_GEOFENCE);
            assert(pack.breach_count_GET() == (char)36558);
            assert(pack.limits_state_GET() == LIMITS_STATE.LIMITS_RECOVERED);
            assert(pack.last_action_GET() == 112799213L);
            assert(pack.last_recovery_GET() == 3480168058L);
            assert(pack.last_trigger_GET() == 3461010751L);
            assert(pack.mods_enabled_GET() == LIMIT_MODULE.LIMIT_ALTITUDE);
            assert(pack.last_clear_GET() == 201004709L);
            assert(pack.mods_triggered_GET() == LIMIT_MODULE.LIMIT_GPSLOCK);
        });
        GroundControl.LIMITS_STATUS p167 = CommunicationChannel.new_LIMITS_STATUS();
        PH.setPack(p167);
        p167.mods_required_SET(LIMIT_MODULE.LIMIT_GEOFENCE) ;
        p167.last_action_SET(112799213L) ;
        p167.limits_state_SET(LIMITS_STATE.LIMITS_RECOVERED) ;
        p167.breach_count_SET((char)36558) ;
        p167.mods_enabled_SET(LIMIT_MODULE.LIMIT_ALTITUDE) ;
        p167.mods_triggered_SET(LIMIT_MODULE.LIMIT_GPSLOCK) ;
        p167.last_trigger_SET(3461010751L) ;
        p167.last_clear_SET(201004709L) ;
        p167.last_recovery_SET(3480168058L) ;
        CommunicationChannel.instance.send(p167);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIND.add((src, ph, pack) ->
        {
            assert(pack.direction_GET() == 1.2461038E38F);
            assert(pack.speed_GET() == -6.7713494E37F);
            assert(pack.speed_z_GET() == -2.757441E38F);
        });
        GroundControl.WIND p168 = CommunicationChannel.new_WIND();
        PH.setPack(p168);
        p168.direction_SET(1.2461038E38F) ;
        p168.speed_z_SET(-2.757441E38F) ;
        p168.speed_SET(-6.7713494E37F) ;
        CommunicationChannel.instance.send(p168);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DATA16.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)197);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)138, (char)237, (char)140, (char)168, (char)42, (char)236, (char)109, (char)18, (char)24, (char)207, (char)252, (char)171, (char)123, (char)205, (char)146, (char)173}));
            assert(pack.len_GET() == (char)41);
        });
        GroundControl.DATA16 p169 = CommunicationChannel.new_DATA16();
        PH.setPack(p169);
        p169.len_SET((char)41) ;
        p169.data__SET(new char[] {(char)138, (char)237, (char)140, (char)168, (char)42, (char)236, (char)109, (char)18, (char)24, (char)207, (char)252, (char)171, (char)123, (char)205, (char)146, (char)173}, 0) ;
        p169.type_SET((char)197) ;
        CommunicationChannel.instance.send(p169);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DATA32.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)202);
            assert(pack.len_GET() == (char)159);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)251, (char)151, (char)19, (char)193, (char)239, (char)183, (char)64, (char)52, (char)122, (char)155, (char)62, (char)231, (char)72, (char)15, (char)71, (char)60, (char)13, (char)76, (char)114, (char)35, (char)202, (char)180, (char)78, (char)213, (char)7, (char)185, (char)28, (char)213, (char)60, (char)238, (char)99, (char)19}));
        });
        GroundControl.DATA32 p170 = CommunicationChannel.new_DATA32();
        PH.setPack(p170);
        p170.data__SET(new char[] {(char)251, (char)151, (char)19, (char)193, (char)239, (char)183, (char)64, (char)52, (char)122, (char)155, (char)62, (char)231, (char)72, (char)15, (char)71, (char)60, (char)13, (char)76, (char)114, (char)35, (char)202, (char)180, (char)78, (char)213, (char)7, (char)185, (char)28, (char)213, (char)60, (char)238, (char)99, (char)19}, 0) ;
        p170.type_SET((char)202) ;
        p170.len_SET((char)159) ;
        CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DATA64.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)100);
            assert(pack.type_GET() == (char)13);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)209, (char)11, (char)152, (char)197, (char)173, (char)169, (char)33, (char)196, (char)31, (char)72, (char)222, (char)120, (char)158, (char)155, (char)198, (char)114, (char)49, (char)79, (char)29, (char)78, (char)89, (char)161, (char)128, (char)63, (char)190, (char)59, (char)221, (char)54, (char)233, (char)154, (char)35, (char)225, (char)107, (char)72, (char)103, (char)168, (char)60, (char)76, (char)178, (char)202, (char)184, (char)220, (char)247, (char)122, (char)150, (char)203, (char)135, (char)196, (char)52, (char)78, (char)199, (char)61, (char)108, (char)104, (char)15, (char)1, (char)219, (char)92, (char)74, (char)132, (char)56, (char)133, (char)158, (char)3}));
        });
        GroundControl.DATA64 p171 = CommunicationChannel.new_DATA64();
        PH.setPack(p171);
        p171.type_SET((char)13) ;
        p171.data__SET(new char[] {(char)209, (char)11, (char)152, (char)197, (char)173, (char)169, (char)33, (char)196, (char)31, (char)72, (char)222, (char)120, (char)158, (char)155, (char)198, (char)114, (char)49, (char)79, (char)29, (char)78, (char)89, (char)161, (char)128, (char)63, (char)190, (char)59, (char)221, (char)54, (char)233, (char)154, (char)35, (char)225, (char)107, (char)72, (char)103, (char)168, (char)60, (char)76, (char)178, (char)202, (char)184, (char)220, (char)247, (char)122, (char)150, (char)203, (char)135, (char)196, (char)52, (char)78, (char)199, (char)61, (char)108, (char)104, (char)15, (char)1, (char)219, (char)92, (char)74, (char)132, (char)56, (char)133, (char)158, (char)3}, 0) ;
        p171.len_SET((char)100) ;
        CommunicationChannel.instance.send(p171);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DATA96.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)0);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)147, (char)79, (char)77, (char)90, (char)197, (char)101, (char)114, (char)20, (char)21, (char)77, (char)203, (char)48, (char)128, (char)237, (char)134, (char)211, (char)118, (char)227, (char)213, (char)191, (char)76, (char)153, (char)107, (char)241, (char)44, (char)235, (char)136, (char)138, (char)24, (char)96, (char)231, (char)175, (char)212, (char)86, (char)26, (char)210, (char)216, (char)140, (char)24, (char)234, (char)169, (char)198, (char)166, (char)108, (char)162, (char)10, (char)255, (char)208, (char)177, (char)216, (char)58, (char)209, (char)75, (char)201, (char)22, (char)224, (char)32, (char)33, (char)237, (char)105, (char)219, (char)247, (char)232, (char)225, (char)177, (char)195, (char)143, (char)125, (char)221, (char)15, (char)253, (char)247, (char)47, (char)8, (char)190, (char)241, (char)106, (char)80, (char)77, (char)240, (char)236, (char)204, (char)164, (char)88, (char)223, (char)4, (char)243, (char)128, (char)166, (char)149, (char)45, (char)34, (char)12, (char)152, (char)175, (char)221}));
            assert(pack.type_GET() == (char)153);
        });
        GroundControl.DATA96 p172 = CommunicationChannel.new_DATA96();
        PH.setPack(p172);
        p172.type_SET((char)153) ;
        p172.len_SET((char)0) ;
        p172.data__SET(new char[] {(char)147, (char)79, (char)77, (char)90, (char)197, (char)101, (char)114, (char)20, (char)21, (char)77, (char)203, (char)48, (char)128, (char)237, (char)134, (char)211, (char)118, (char)227, (char)213, (char)191, (char)76, (char)153, (char)107, (char)241, (char)44, (char)235, (char)136, (char)138, (char)24, (char)96, (char)231, (char)175, (char)212, (char)86, (char)26, (char)210, (char)216, (char)140, (char)24, (char)234, (char)169, (char)198, (char)166, (char)108, (char)162, (char)10, (char)255, (char)208, (char)177, (char)216, (char)58, (char)209, (char)75, (char)201, (char)22, (char)224, (char)32, (char)33, (char)237, (char)105, (char)219, (char)247, (char)232, (char)225, (char)177, (char)195, (char)143, (char)125, (char)221, (char)15, (char)253, (char)247, (char)47, (char)8, (char)190, (char)241, (char)106, (char)80, (char)77, (char)240, (char)236, (char)204, (char)164, (char)88, (char)223, (char)4, (char)243, (char)128, (char)166, (char)149, (char)45, (char)34, (char)12, (char)152, (char)175, (char)221}, 0) ;
        CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RANGEFINDER.add((src, ph, pack) ->
        {
            assert(pack.voltage_GET() == 1.3767574E38F);
            assert(pack.distance_GET() == -2.4549202E38F);
        });
        GroundControl.RANGEFINDER p173 = CommunicationChannel.new_RANGEFINDER();
        PH.setPack(p173);
        p173.voltage_SET(1.3767574E38F) ;
        p173.distance_SET(-2.4549202E38F) ;
        CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AIRSPEED_AUTOCAL.add((src, ph, pack) ->
        {
            assert(pack.state_x_GET() == -1.830453E38F);
            assert(pack.ratio_GET() == -6.143362E37F);
            assert(pack.vz_GET() == -2.6917139E38F);
            assert(pack.vy_GET() == -1.1320972E38F);
            assert(pack.EAS2TAS_GET() == 1.8450725E38F);
            assert(pack.diff_pressure_GET() == -2.654471E38F);
            assert(pack.Pax_GET() == -8.752449E37F);
            assert(pack.state_y_GET() == 9.681422E37F);
            assert(pack.Pby_GET() == 2.0639727E38F);
            assert(pack.state_z_GET() == -1.4927512E37F);
            assert(pack.vx_GET() == -3.0549562E38F);
            assert(pack.Pcz_GET() == 1.452948E38F);
        });
        GroundControl.AIRSPEED_AUTOCAL p174 = CommunicationChannel.new_AIRSPEED_AUTOCAL();
        PH.setPack(p174);
        p174.Pcz_SET(1.452948E38F) ;
        p174.Pax_SET(-8.752449E37F) ;
        p174.EAS2TAS_SET(1.8450725E38F) ;
        p174.diff_pressure_SET(-2.654471E38F) ;
        p174.state_z_SET(-1.4927512E37F) ;
        p174.vz_SET(-2.6917139E38F) ;
        p174.vy_SET(-1.1320972E38F) ;
        p174.state_x_SET(-1.830453E38F) ;
        p174.vx_SET(-3.0549562E38F) ;
        p174.ratio_SET(-6.143362E37F) ;
        p174.state_y_SET(9.681422E37F) ;
        p174.Pby_SET(2.0639727E38F) ;
        CommunicationChannel.instance.send(p174);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RALLY_POINT.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1817331877);
            assert(pack.alt_GET() == (short)26346);
            assert(pack.target_system_GET() == (char)65);
            assert(pack.target_component_GET() == (char)135);
            assert(pack.land_dir_GET() == (char)44869);
            assert(pack.break_alt_GET() == (short) -16846);
            assert(pack.count_GET() == (char)159);
            assert(pack.idx_GET() == (char)182);
            assert(pack.flags_GET() == RALLY_FLAGS.LAND_IMMEDIATELY);
            assert(pack.lng_GET() == -1649036668);
        });
        GroundControl.RALLY_POINT p175 = CommunicationChannel.new_RALLY_POINT();
        PH.setPack(p175);
        p175.land_dir_SET((char)44869) ;
        p175.lat_SET(-1817331877) ;
        p175.count_SET((char)159) ;
        p175.alt_SET((short)26346) ;
        p175.flags_SET(RALLY_FLAGS.LAND_IMMEDIATELY) ;
        p175.target_system_SET((char)65) ;
        p175.idx_SET((char)182) ;
        p175.target_component_SET((char)135) ;
        p175.break_alt_SET((short) -16846) ;
        p175.lng_SET(-1649036668) ;
        CommunicationChannel.instance.send(p175);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RALLY_FETCH_POINT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)46);
            assert(pack.target_system_GET() == (char)78);
            assert(pack.idx_GET() == (char)87);
        });
        GroundControl.RALLY_FETCH_POINT p176 = CommunicationChannel.new_RALLY_FETCH_POINT();
        PH.setPack(p176);
        p176.target_system_SET((char)78) ;
        p176.target_component_SET((char)46) ;
        p176.idx_SET((char)87) ;
        CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_COMPASSMOT_STATUS.add((src, ph, pack) ->
        {
            assert(pack.CompensationY_GET() == -1.7479926E38F);
            assert(pack.throttle_GET() == (char)60254);
            assert(pack.CompensationX_GET() == -1.5846186E38F);
            assert(pack.interference_GET() == (char)19061);
            assert(pack.current_GET() == -2.0330043E38F);
            assert(pack.CompensationZ_GET() == 1.1035502E37F);
        });
        GroundControl.COMPASSMOT_STATUS p177 = CommunicationChannel.new_COMPASSMOT_STATUS();
        PH.setPack(p177);
        p177.CompensationY_SET(-1.7479926E38F) ;
        p177.CompensationX_SET(-1.5846186E38F) ;
        p177.interference_SET((char)19061) ;
        p177.current_SET(-2.0330043E38F) ;
        p177.CompensationZ_SET(1.1035502E37F) ;
        p177.throttle_SET((char)60254) ;
        CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AHRS2.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == -2.4134844E38F);
            assert(pack.roll_GET() == 2.5454477E38F);
            assert(pack.lat_GET() == 865912023);
            assert(pack.pitch_GET() == -1.6384054E38F);
            assert(pack.yaw_GET() == -3.3509212E38F);
            assert(pack.lng_GET() == -96134938);
        });
        GroundControl.AHRS2 p178 = CommunicationChannel.new_AHRS2();
        PH.setPack(p178);
        p178.pitch_SET(-1.6384054E38F) ;
        p178.yaw_SET(-3.3509212E38F) ;
        p178.altitude_SET(-2.4134844E38F) ;
        p178.lng_SET(-96134938) ;
        p178.roll_SET(2.5454477E38F) ;
        p178.lat_SET(865912023) ;
        CommunicationChannel.instance.send(p178);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_STATUS.add((src, ph, pack) ->
        {
            assert(pack.event_id_GET() == CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_ERROR);
            assert(pack.p2_GET() == -7.1318454E37F);
            assert(pack.target_system_GET() == (char)83);
            assert(pack.img_idx_GET() == (char)62672);
            assert(pack.cam_idx_GET() == (char)124);
            assert(pack.p4_GET() == -8.2059556E37F);
            assert(pack.p3_GET() == -2.4225148E38F);
            assert(pack.time_usec_GET() == 3887956839759409710L);
            assert(pack.p1_GET() == 2.2000413E38F);
        });
        GroundControl.CAMERA_STATUS p179 = CommunicationChannel.new_CAMERA_STATUS();
        PH.setPack(p179);
        p179.p4_SET(-8.2059556E37F) ;
        p179.img_idx_SET((char)62672) ;
        p179.cam_idx_SET((char)124) ;
        p179.target_system_SET((char)83) ;
        p179.p3_SET(-2.4225148E38F) ;
        p179.p2_SET(-7.1318454E37F) ;
        p179.event_id_SET(CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_ERROR) ;
        p179.p1_SET(2.2000413E38F) ;
        p179.time_usec_SET(3887956839759409710L) ;
        CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_FEEDBACK.add((src, ph, pack) ->
        {
            assert(pack.img_idx_GET() == (char)15419);
            assert(pack.foc_len_GET() == 1.7024712E38F);
            assert(pack.yaw_GET() == 3.182103E38F);
            assert(pack.pitch_GET() == 2.885314E38F);
            assert(pack.lat_GET() == -1483364569);
            assert(pack.time_usec_GET() == 6317601084104946330L);
            assert(pack.roll_GET() == 1.2088592E38F);
            assert(pack.lng_GET() == -705178065);
            assert(pack.target_system_GET() == (char)112);
            assert(pack.flags_GET() == CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_OPENLOOP);
            assert(pack.alt_msl_GET() == -9.841552E37F);
            assert(pack.cam_idx_GET() == (char)167);
            assert(pack.alt_rel_GET() == -1.1252869E38F);
        });
        GroundControl.CAMERA_FEEDBACK p180 = CommunicationChannel.new_CAMERA_FEEDBACK();
        PH.setPack(p180);
        p180.img_idx_SET((char)15419) ;
        p180.time_usec_SET(6317601084104946330L) ;
        p180.alt_rel_SET(-1.1252869E38F) ;
        p180.yaw_SET(3.182103E38F) ;
        p180.cam_idx_SET((char)167) ;
        p180.target_system_SET((char)112) ;
        p180.foc_len_SET(1.7024712E38F) ;
        p180.alt_msl_SET(-9.841552E37F) ;
        p180.pitch_SET(2.885314E38F) ;
        p180.flags_SET(CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_OPENLOOP) ;
        p180.lat_SET(-1483364569) ;
        p180.roll_SET(1.2088592E38F) ;
        p180.lng_SET(-705178065) ;
        CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BATTERY2.add((src, ph, pack) ->
        {
            assert(pack.voltage_GET() == (char)56586);
            assert(pack.current_battery_GET() == (short) -20225);
        });
        GroundControl.BATTERY2 p181 = CommunicationChannel.new_BATTERY2();
        PH.setPack(p181);
        p181.current_battery_SET((short) -20225) ;
        p181.voltage_SET((char)56586) ;
        CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AHRS3.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == -1.8935544E37F);
            assert(pack.roll_GET() == -2.2838853E38F);
            assert(pack.v3_GET() == -7.679047E37F);
            assert(pack.yaw_GET() == -2.647159E38F);
            assert(pack.v4_GET() == -2.3157224E38F);
            assert(pack.lng_GET() == 1328629113);
            assert(pack.v1_GET() == 5.0850004E37F);
            assert(pack.pitch_GET() == 2.9547688E38F);
            assert(pack.v2_GET() == 2.4973382E38F);
            assert(pack.lat_GET() == 372758602);
        });
        GroundControl.AHRS3 p182 = CommunicationChannel.new_AHRS3();
        PH.setPack(p182);
        p182.v1_SET(5.0850004E37F) ;
        p182.lng_SET(1328629113) ;
        p182.pitch_SET(2.9547688E38F) ;
        p182.lat_SET(372758602) ;
        p182.v3_SET(-7.679047E37F) ;
        p182.v2_SET(2.4973382E38F) ;
        p182.roll_SET(-2.2838853E38F) ;
        p182.v4_SET(-2.3157224E38F) ;
        p182.altitude_SET(-1.8935544E37F) ;
        p182.yaw_SET(-2.647159E38F) ;
        CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)105);
            assert(pack.target_system_GET() == (char)156);
        });
        GroundControl.AUTOPILOT_VERSION_REQUEST p183 = CommunicationChannel.new_AUTOPILOT_VERSION_REQUEST();
        PH.setPack(p183);
        p183.target_component_SET((char)105) ;
        p183.target_system_SET((char)156) ;
        CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_REMOTE_LOG_DATA_BLOCK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)186);
            assert(pack.seqno_GET() == MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP);
            assert(pack.target_component_GET() == (char)105);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)177, (char)141, (char)241, (char)10, (char)106, (char)232, (char)172, (char)79, (char)148, (char)46, (char)209, (char)219, (char)2, (char)109, (char)171, (char)239, (char)245, (char)12, (char)233, (char)167, (char)30, (char)75, (char)230, (char)236, (char)99, (char)170, (char)135, (char)209, (char)222, (char)229, (char)31, (char)92, (char)132, (char)116, (char)249, (char)43, (char)125, (char)113, (char)13, (char)162, (char)210, (char)205, (char)132, (char)108, (char)112, (char)86, (char)119, (char)58, (char)180, (char)7, (char)112, (char)245, (char)214, (char)2, (char)86, (char)245, (char)19, (char)176, (char)43, (char)252, (char)105, (char)1, (char)45, (char)51, (char)208, (char)239, (char)4, (char)79, (char)73, (char)112, (char)198, (char)136, (char)55, (char)252, (char)152, (char)86, (char)91, (char)202, (char)62, (char)229, (char)198, (char)25, (char)122, (char)31, (char)44, (char)0, (char)234, (char)30, (char)193, (char)12, (char)207, (char)251, (char)41, (char)62, (char)132, (char)203, (char)37, (char)174, (char)91, (char)109, (char)162, (char)9, (char)193, (char)180, (char)134, (char)42, (char)253, (char)63, (char)228, (char)248, (char)82, (char)235, (char)88, (char)199, (char)32, (char)195, (char)122, (char)226, (char)57, (char)180, (char)181, (char)160, (char)83, (char)104, (char)83, (char)58, (char)85, (char)76, (char)249, (char)63, (char)106, (char)231, (char)229, (char)144, (char)12, (char)137, (char)21, (char)73, (char)222, (char)61, (char)254, (char)81, (char)74, (char)2, (char)9, (char)134, (char)191, (char)175, (char)36, (char)254, (char)195, (char)56, (char)170, (char)170, (char)214, (char)253, (char)87, (char)0, (char)84, (char)88, (char)25, (char)178, (char)125, (char)166, (char)21, (char)230, (char)57, (char)13, (char)6, (char)198, (char)64, (char)130, (char)82, (char)242, (char)178, (char)10, (char)46, (char)225, (char)225, (char)197, (char)219, (char)43, (char)70, (char)196, (char)230, (char)195, (char)96, (char)166, (char)232, (char)212, (char)74, (char)175, (char)167, (char)97, (char)240, (char)9, (char)232, (char)190, (char)41, (char)223}));
        });
        GroundControl.REMOTE_LOG_DATA_BLOCK p184 = CommunicationChannel.new_REMOTE_LOG_DATA_BLOCK();
        PH.setPack(p184);
        p184.data__SET(new char[] {(char)177, (char)141, (char)241, (char)10, (char)106, (char)232, (char)172, (char)79, (char)148, (char)46, (char)209, (char)219, (char)2, (char)109, (char)171, (char)239, (char)245, (char)12, (char)233, (char)167, (char)30, (char)75, (char)230, (char)236, (char)99, (char)170, (char)135, (char)209, (char)222, (char)229, (char)31, (char)92, (char)132, (char)116, (char)249, (char)43, (char)125, (char)113, (char)13, (char)162, (char)210, (char)205, (char)132, (char)108, (char)112, (char)86, (char)119, (char)58, (char)180, (char)7, (char)112, (char)245, (char)214, (char)2, (char)86, (char)245, (char)19, (char)176, (char)43, (char)252, (char)105, (char)1, (char)45, (char)51, (char)208, (char)239, (char)4, (char)79, (char)73, (char)112, (char)198, (char)136, (char)55, (char)252, (char)152, (char)86, (char)91, (char)202, (char)62, (char)229, (char)198, (char)25, (char)122, (char)31, (char)44, (char)0, (char)234, (char)30, (char)193, (char)12, (char)207, (char)251, (char)41, (char)62, (char)132, (char)203, (char)37, (char)174, (char)91, (char)109, (char)162, (char)9, (char)193, (char)180, (char)134, (char)42, (char)253, (char)63, (char)228, (char)248, (char)82, (char)235, (char)88, (char)199, (char)32, (char)195, (char)122, (char)226, (char)57, (char)180, (char)181, (char)160, (char)83, (char)104, (char)83, (char)58, (char)85, (char)76, (char)249, (char)63, (char)106, (char)231, (char)229, (char)144, (char)12, (char)137, (char)21, (char)73, (char)222, (char)61, (char)254, (char)81, (char)74, (char)2, (char)9, (char)134, (char)191, (char)175, (char)36, (char)254, (char)195, (char)56, (char)170, (char)170, (char)214, (char)253, (char)87, (char)0, (char)84, (char)88, (char)25, (char)178, (char)125, (char)166, (char)21, (char)230, (char)57, (char)13, (char)6, (char)198, (char)64, (char)130, (char)82, (char)242, (char)178, (char)10, (char)46, (char)225, (char)225, (char)197, (char)219, (char)43, (char)70, (char)196, (char)230, (char)195, (char)96, (char)166, (char)232, (char)212, (char)74, (char)175, (char)167, (char)97, (char)240, (char)9, (char)232, (char)190, (char)41, (char)223}, 0) ;
        p184.target_system_SET((char)186) ;
        p184.seqno_SET(MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP) ;
        p184.target_component_SET((char)105) ;
        CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_REMOTE_LOG_BLOCK_STATUS.add((src, ph, pack) ->
        {
            assert(pack.status_GET() == MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_ACK);
            assert(pack.target_system_GET() == (char)142);
            assert(pack.seqno_GET() == 231170306L);
            assert(pack.target_component_GET() == (char)76);
        });
        GroundControl.REMOTE_LOG_BLOCK_STATUS p185 = CommunicationChannel.new_REMOTE_LOG_BLOCK_STATUS();
        PH.setPack(p185);
        p185.seqno_SET(231170306L) ;
        p185.target_system_SET((char)142) ;
        p185.status_SET(MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_ACK) ;
        p185.target_component_SET((char)76) ;
        CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LED_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.instance_GET() == (char)107);
            assert(Arrays.equals(pack.custom_bytes_GET(),  new char[] {(char)109, (char)105, (char)88, (char)124, (char)175, (char)95, (char)247, (char)180, (char)108, (char)181, (char)71, (char)184, (char)13, (char)51, (char)137, (char)199, (char)119, (char)42, (char)190, (char)165, (char)251, (char)148, (char)212, (char)223}));
            assert(pack.pattern_GET() == (char)105);
            assert(pack.target_system_GET() == (char)222);
            assert(pack.target_component_GET() == (char)231);
            assert(pack.custom_len_GET() == (char)246);
        });
        GroundControl.LED_CONTROL p186 = CommunicationChannel.new_LED_CONTROL();
        PH.setPack(p186);
        p186.pattern_SET((char)105) ;
        p186.target_component_SET((char)231) ;
        p186.custom_len_SET((char)246) ;
        p186.custom_bytes_SET(new char[] {(char)109, (char)105, (char)88, (char)124, (char)175, (char)95, (char)247, (char)180, (char)108, (char)181, (char)71, (char)184, (char)13, (char)51, (char)137, (char)199, (char)119, (char)42, (char)190, (char)165, (char)251, (char)148, (char)212, (char)223}, 0) ;
        p186.instance_SET((char)107) ;
        p186.target_system_SET((char)222) ;
        CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MAG_CAL_PROGRESS.add((src, ph, pack) ->
        {
            assert(pack.direction_y_GET() == -1.7885485E38F);
            assert(pack.cal_status_GET() == MAG_CAL_STATUS.MAG_CAL_NOT_STARTED);
            assert(pack.cal_mask_GET() == (char)202);
            assert(pack.direction_x_GET() == -2.2650133E38F);
            assert(pack.compass_id_GET() == (char)249);
            assert(pack.attempt_GET() == (char)85);
            assert(Arrays.equals(pack.completion_mask_GET(),  new char[] {(char)219, (char)94, (char)125, (char)123, (char)78, (char)45, (char)202, (char)251, (char)36, (char)212}));
            assert(pack.completion_pct_GET() == (char)222);
            assert(pack.direction_z_GET() == 6.749772E37F);
        });
        GroundControl.MAG_CAL_PROGRESS p191 = CommunicationChannel.new_MAG_CAL_PROGRESS();
        PH.setPack(p191);
        p191.completion_pct_SET((char)222) ;
        p191.cal_status_SET(MAG_CAL_STATUS.MAG_CAL_NOT_STARTED) ;
        p191.direction_z_SET(6.749772E37F) ;
        p191.direction_x_SET(-2.2650133E38F) ;
        p191.direction_y_SET(-1.7885485E38F) ;
        p191.attempt_SET((char)85) ;
        p191.cal_mask_SET((char)202) ;
        p191.completion_mask_SET(new char[] {(char)219, (char)94, (char)125, (char)123, (char)78, (char)45, (char)202, (char)251, (char)36, (char)212}, 0) ;
        p191.compass_id_SET((char)249) ;
        CommunicationChannel.instance.send(p191);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MAG_CAL_REPORT.add((src, ph, pack) ->
        {
            assert(pack.autosaved_GET() == (char)172);
            assert(pack.ofs_z_GET() == 2.4458745E37F);
            assert(pack.diag_y_GET() == -2.8831372E38F);
            assert(pack.offdiag_y_GET() == -2.9404975E36F);
            assert(pack.diag_z_GET() == -1.2577372E38F);
            assert(pack.ofs_x_GET() == 3.6939458E37F);
            assert(pack.cal_status_GET() == MAG_CAL_STATUS.MAG_CAL_FAILED);
            assert(pack.offdiag_z_GET() == -1.310846E38F);
            assert(pack.compass_id_GET() == (char)132);
            assert(pack.cal_mask_GET() == (char)169);
            assert(pack.diag_x_GET() == 4.514403E37F);
            assert(pack.fitness_GET() == 8.486416E37F);
            assert(pack.ofs_y_GET() == -5.66697E37F);
            assert(pack.offdiag_x_GET() == -2.8192079E38F);
        });
        GroundControl.MAG_CAL_REPORT p192 = CommunicationChannel.new_MAG_CAL_REPORT();
        PH.setPack(p192);
        p192.diag_x_SET(4.514403E37F) ;
        p192.autosaved_SET((char)172) ;
        p192.cal_status_SET(MAG_CAL_STATUS.MAG_CAL_FAILED) ;
        p192.diag_y_SET(-2.8831372E38F) ;
        p192.diag_z_SET(-1.2577372E38F) ;
        p192.cal_mask_SET((char)169) ;
        p192.ofs_z_SET(2.4458745E37F) ;
        p192.offdiag_z_SET(-1.310846E38F) ;
        p192.ofs_y_SET(-5.66697E37F) ;
        p192.offdiag_x_SET(-2.8192079E38F) ;
        p192.offdiag_y_SET(-2.9404975E36F) ;
        p192.ofs_x_SET(3.6939458E37F) ;
        p192.compass_id_SET((char)132) ;
        p192.fitness_SET(8.486416E37F) ;
        CommunicationChannel.instance.send(p192);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_EKF_STATUS_REPORT.add((src, ph, pack) ->
        {
            assert(pack.pos_horiz_variance_GET() == 3.0568208E38F);
            assert(pack.pos_vert_variance_GET() == 7.238348E37F);
            assert(pack.velocity_variance_GET() == 3.5761724E37F);
            assert(pack.compass_variance_GET() == 7.1031843E37F);
            assert(pack.flags_GET() == EKF_STATUS_FLAGS.EKF_VELOCITY_HORIZ);
            assert(pack.terrain_alt_variance_GET() == -2.9101915E38F);
        });
        GroundControl.EKF_STATUS_REPORT p193 = CommunicationChannel.new_EKF_STATUS_REPORT();
        PH.setPack(p193);
        p193.compass_variance_SET(7.1031843E37F) ;
        p193.terrain_alt_variance_SET(-2.9101915E38F) ;
        p193.flags_SET(EKF_STATUS_FLAGS.EKF_VELOCITY_HORIZ) ;
        p193.pos_vert_variance_SET(7.238348E37F) ;
        p193.pos_horiz_variance_SET(3.0568208E38F) ;
        p193.velocity_variance_SET(3.5761724E37F) ;
        CommunicationChannel.instance.send(p193);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PID_TUNING.add((src, ph, pack) ->
        {
            assert(pack.achieved_GET() == -1.4193386E38F);
            assert(pack.P_GET() == 8.233784E37F);
            assert(pack.I_GET() == 3.7550098E37F);
            assert(pack.D_GET() == -3.1447148E38F);
            assert(pack.FF_GET() == -2.6371343E38F);
            assert(pack.axis_GET() == PID_TUNING_AXIS.PID_TUNING_ACCZ);
            assert(pack.desired_GET() == 1.8057478E38F);
        });
        GroundControl.PID_TUNING p194 = CommunicationChannel.new_PID_TUNING();
        PH.setPack(p194);
        p194.D_SET(-3.1447148E38F) ;
        p194.FF_SET(-2.6371343E38F) ;
        p194.P_SET(8.233784E37F) ;
        p194.achieved_SET(-1.4193386E38F) ;
        p194.desired_SET(1.8057478E38F) ;
        p194.axis_SET(PID_TUNING_AXIS.PID_TUNING_ACCZ) ;
        p194.I_SET(3.7550098E37F) ;
        CommunicationChannel.instance.send(p194);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GIMBAL_REPORT.add((src, ph, pack) ->
        {
            assert(pack.joint_roll_GET() == 8.943514E37F);
            assert(pack.delta_time_GET() == 2.1742102E38F);
            assert(pack.delta_velocity_y_GET() == -3.0624295E38F);
            assert(pack.delta_velocity_z_GET() == 1.4745863E37F);
            assert(pack.target_system_GET() == (char)219);
            assert(pack.delta_angle_x_GET() == -3.2073292E38F);
            assert(pack.delta_angle_z_GET() == -1.2747729E38F);
            assert(pack.target_component_GET() == (char)10);
            assert(pack.joint_el_GET() == 1.295841E38F);
            assert(pack.joint_az_GET() == 2.5421984E38F);
            assert(pack.delta_angle_y_GET() == -2.2799536E38F);
            assert(pack.delta_velocity_x_GET() == -8.473646E37F);
        });
        GroundControl.GIMBAL_REPORT p200 = CommunicationChannel.new_GIMBAL_REPORT();
        PH.setPack(p200);
        p200.delta_angle_z_SET(-1.2747729E38F) ;
        p200.delta_velocity_z_SET(1.4745863E37F) ;
        p200.target_system_SET((char)219) ;
        p200.joint_az_SET(2.5421984E38F) ;
        p200.delta_angle_y_SET(-2.2799536E38F) ;
        p200.delta_angle_x_SET(-3.2073292E38F) ;
        p200.delta_velocity_x_SET(-8.473646E37F) ;
        p200.delta_velocity_y_SET(-3.0624295E38F) ;
        p200.delta_time_SET(2.1742102E38F) ;
        p200.joint_roll_SET(8.943514E37F) ;
        p200.joint_el_SET(1.295841E38F) ;
        p200.target_component_SET((char)10) ;
        CommunicationChannel.instance.send(p200);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GIMBAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)90);
            assert(pack.demanded_rate_z_GET() == -3.2932234E38F);
            assert(pack.demanded_rate_y_GET() == -1.9435625E38F);
            assert(pack.demanded_rate_x_GET() == -1.1437712E38F);
            assert(pack.target_system_GET() == (char)81);
        });
        GroundControl.GIMBAL_CONTROL p201 = CommunicationChannel.new_GIMBAL_CONTROL();
        PH.setPack(p201);
        p201.demanded_rate_x_SET(-1.1437712E38F) ;
        p201.demanded_rate_y_SET(-1.9435625E38F) ;
        p201.target_component_SET((char)90) ;
        p201.target_system_SET((char)81) ;
        p201.demanded_rate_z_SET(-3.2932234E38F) ;
        CommunicationChannel.instance.send(p201);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GIMBAL_TORQUE_CMD_REPORT.add((src, ph, pack) ->
        {
            assert(pack.el_torque_cmd_GET() == (short) -16468);
            assert(pack.az_torque_cmd_GET() == (short)24641);
            assert(pack.target_component_GET() == (char)99);
            assert(pack.target_system_GET() == (char)101);
            assert(pack.rl_torque_cmd_GET() == (short)23557);
        });
        GroundControl.GIMBAL_TORQUE_CMD_REPORT p214 = CommunicationChannel.new_GIMBAL_TORQUE_CMD_REPORT();
        PH.setPack(p214);
        p214.rl_torque_cmd_SET((short)23557) ;
        p214.az_torque_cmd_SET((short)24641) ;
        p214.target_component_SET((char)99) ;
        p214.el_torque_cmd_SET((short) -16468) ;
        p214.target_system_SET((char)101) ;
        CommunicationChannel.instance.send(p214);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GOPRO_HEARTBEAT.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING);
            assert(pack.status_GET() == GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_CONNECTED);
            assert(pack.capture_mode_GET() == GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_PHOTO);
        });
        GroundControl.GOPRO_HEARTBEAT p215 = CommunicationChannel.new_GOPRO_HEARTBEAT();
        PH.setPack(p215);
        p215.status_SET(GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_CONNECTED) ;
        p215.capture_mode_SET(GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_PHOTO) ;
        p215.flags_SET(GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING) ;
        CommunicationChannel.instance.send(p215);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GOPRO_GET_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)87);
            assert(pack.target_system_GET() == (char)37);
            assert(pack.cmd_id_GET() == GOPRO_COMMAND.GOPRO_COMMAND_LOW_LIGHT);
        });
        GroundControl.GOPRO_GET_REQUEST p216 = CommunicationChannel.new_GOPRO_GET_REQUEST();
        PH.setPack(p216);
        p216.target_system_SET((char)37) ;
        p216.target_component_SET((char)87) ;
        p216.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_LOW_LIGHT) ;
        CommunicationChannel.instance.send(p216);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GOPRO_GET_RESPONSE.add((src, ph, pack) ->
        {
            assert(pack.status_GET() == GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED);
            assert(Arrays.equals(pack.value_GET(),  new char[] {(char)127, (char)141, (char)171, (char)52}));
            assert(pack.cmd_id_GET() == GOPRO_COMMAND.GOPRO_COMMAND_VIDEO_SETTINGS);
        });
        GroundControl.GOPRO_GET_RESPONSE p217 = CommunicationChannel.new_GOPRO_GET_RESPONSE();
        PH.setPack(p217);
        p217.value_SET(new char[] {(char)127, (char)141, (char)171, (char)52}, 0) ;
        p217.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_VIDEO_SETTINGS) ;
        p217.status_SET(GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED) ;
        CommunicationChannel.instance.send(p217);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GOPRO_SET_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.value_GET(),  new char[] {(char)99, (char)28, (char)6, (char)143}));
            assert(pack.target_system_GET() == (char)26);
            assert(pack.cmd_id_GET() == GOPRO_COMMAND.GOPRO_COMMAND_CAPTURE_MODE);
            assert(pack.target_component_GET() == (char)194);
        });
        GroundControl.GOPRO_SET_REQUEST p218 = CommunicationChannel.new_GOPRO_SET_REQUEST();
        PH.setPack(p218);
        p218.target_component_SET((char)194) ;
        p218.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_CAPTURE_MODE) ;
        p218.target_system_SET((char)26) ;
        p218.value_SET(new char[] {(char)99, (char)28, (char)6, (char)143}, 0) ;
        CommunicationChannel.instance.send(p218);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GOPRO_SET_RESPONSE.add((src, ph, pack) ->
        {
            assert(pack.cmd_id_GET() == GOPRO_COMMAND.GOPRO_COMMAND_LOW_LIGHT);
            assert(pack.status_GET() == GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS);
        });
        GroundControl.GOPRO_SET_RESPONSE p219 = CommunicationChannel.new_GOPRO_SET_RESPONSE();
        PH.setPack(p219);
        p219.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_LOW_LIGHT) ;
        p219.status_SET(GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS) ;
        CommunicationChannel.instance.send(p219);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RPM.add((src, ph, pack) ->
        {
            assert(pack.rpm2_GET() == -2.9090882E38F);
            assert(pack.rpm1_GET() == -1.0203173E38F);
        });
        GroundControl.RPM p226 = CommunicationChannel.new_RPM();
        PH.setPack(p226);
        p226.rpm1_SET(-1.0203173E38F) ;
        p226.rpm2_SET(-2.9090882E38F) ;
        CommunicationChannel.instance.send(p226);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        CommunicationChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH);
            assert(pack.pos_vert_accuracy_GET() == 1.7149388E38F);
            assert(pack.pos_vert_ratio_GET() == 2.549155E38F);
            assert(pack.pos_horiz_accuracy_GET() == -3.313287E38F);
            assert(pack.time_usec_GET() == 6725087303567657878L);
            assert(pack.pos_horiz_ratio_GET() == -7.185308E37F);
            assert(pack.hagl_ratio_GET() == 1.4232295E38F);
            assert(pack.vel_ratio_GET() == 1.1835154E38F);
            assert(pack.tas_ratio_GET() == -2.9762242E38F);
            assert(pack.mag_ratio_GET() == 1.4642851E38F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.pos_horiz_ratio_SET(-7.185308E37F) ;
        p230.vel_ratio_SET(1.1835154E38F) ;
        p230.pos_vert_ratio_SET(2.549155E38F) ;
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH) ;
        p230.mag_ratio_SET(1.4642851E38F) ;
        p230.pos_vert_accuracy_SET(1.7149388E38F) ;
        p230.time_usec_SET(6725087303567657878L) ;
        p230.tas_ratio_SET(-2.9762242E38F) ;
        p230.hagl_ratio_SET(1.4232295E38F) ;
        p230.pos_horiz_accuracy_SET(-3.313287E38F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_x_GET() == -2.371905E37F);
            assert(pack.wind_alt_GET() == 5.42058E37F);
            assert(pack.horiz_accuracy_GET() == -3.0254826E38F);
            assert(pack.wind_y_GET() == -8.3402795E37F);
            assert(pack.wind_z_GET() == 1.8297167E38F);
            assert(pack.var_horiz_GET() == 1.0919214E38F);
            assert(pack.vert_accuracy_GET() == -1.1937907E38F);
            assert(pack.var_vert_GET() == -1.492066E38F);
            assert(pack.time_usec_GET() == 4219139216960762691L);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.var_vert_SET(-1.492066E38F) ;
        p231.wind_z_SET(1.8297167E38F) ;
        p231.wind_alt_SET(5.42058E37F) ;
        p231.horiz_accuracy_SET(-3.0254826E38F) ;
        p231.wind_y_SET(-8.3402795E37F) ;
        p231.wind_x_SET(-2.371905E37F) ;
        p231.time_usec_SET(4219139216960762691L) ;
        p231.var_horiz_SET(1.0919214E38F) ;
        p231.vert_accuracy_SET(-1.1937907E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.ignore_flags_GET() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT);
            assert(pack.ve_GET() == 7.337574E37F);
            assert(pack.alt_GET() == -2.5469465E38F);
            assert(pack.fix_type_GET() == (char)42);
            assert(pack.gps_id_GET() == (char)168);
            assert(pack.vert_accuracy_GET() == 2.4500297E38F);
            assert(pack.time_week_ms_GET() == 1515018576L);
            assert(pack.time_week_GET() == (char)15907);
            assert(pack.lon_GET() == 1462577157);
            assert(pack.lat_GET() == -377674923);
            assert(pack.vdop_GET() == -2.7867019E38F);
            assert(pack.vd_GET() == -2.819485E38F);
            assert(pack.vn_GET() == -2.7531765E38F);
            assert(pack.time_usec_GET() == 4966534559822661666L);
            assert(pack.hdop_GET() == -7.3318533E37F);
            assert(pack.satellites_visible_GET() == (char)146);
            assert(pack.speed_accuracy_GET() == -1.7651217E38F);
            assert(pack.horiz_accuracy_GET() == -4.497249E37F);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.alt_SET(-2.5469465E38F) ;
        p232.lat_SET(-377674923) ;
        p232.speed_accuracy_SET(-1.7651217E38F) ;
        p232.lon_SET(1462577157) ;
        p232.ve_SET(7.337574E37F) ;
        p232.time_usec_SET(4966534559822661666L) ;
        p232.fix_type_SET((char)42) ;
        p232.vn_SET(-2.7531765E38F) ;
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT) ;
        p232.horiz_accuracy_SET(-4.497249E37F) ;
        p232.vert_accuracy_SET(2.4500297E38F) ;
        p232.time_week_ms_SET(1515018576L) ;
        p232.vd_SET(-2.819485E38F) ;
        p232.satellites_visible_SET((char)146) ;
        p232.time_week_SET((char)15907) ;
        p232.vdop_SET(-2.7867019E38F) ;
        p232.hdop_SET(-7.3318533E37F) ;
        p232.gps_id_SET((char)168) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)240, (char)51, (char)197, (char)131, (char)190, (char)218, (char)149, (char)249, (char)9, (char)11, (char)163, (char)168, (char)101, (char)224, (char)203, (char)74, (char)192, (char)84, (char)153, (char)88, (char)220, (char)219, (char)116, (char)186, (char)193, (char)112, (char)149, (char)42, (char)50, (char)193, (char)26, (char)64, (char)164, (char)33, (char)106, (char)223, (char)179, (char)118, (char)166, (char)226, (char)192, (char)171, (char)66, (char)81, (char)0, (char)159, (char)251, (char)194, (char)233, (char)59, (char)189, (char)96, (char)230, (char)227, (char)202, (char)229, (char)197, (char)83, (char)174, (char)219, (char)199, (char)241, (char)151, (char)159, (char)252, (char)60, (char)154, (char)204, (char)127, (char)0, (char)19, (char)130, (char)60, (char)28, (char)12, (char)222, (char)126, (char)70, (char)189, (char)111, (char)130, (char)45, (char)6, (char)25, (char)179, (char)122, (char)122, (char)154, (char)30, (char)113, (char)12, (char)195, (char)20, (char)174, (char)141, (char)199, (char)204, (char)2, (char)60, (char)28, (char)200, (char)225, (char)42, (char)158, (char)188, (char)45, (char)136, (char)125, (char)181, (char)128, (char)6, (char)220, (char)29, (char)87, (char)197, (char)138, (char)22, (char)243, (char)112, (char)127, (char)188, (char)214, (char)126, (char)58, (char)146, (char)114, (char)130, (char)62, (char)45, (char)57, (char)239, (char)90, (char)172, (char)222, (char)16, (char)247, (char)166, (char)107, (char)91, (char)94, (char)197, (char)109, (char)156, (char)57, (char)111, (char)75, (char)62, (char)69, (char)225, (char)12, (char)51, (char)124, (char)104, (char)252, (char)116, (char)176, (char)28, (char)139, (char)51, (char)233, (char)187, (char)239, (char)242, (char)247, (char)238, (char)228, (char)70, (char)46, (char)184, (char)219, (char)248, (char)84, (char)58, (char)97, (char)122, (char)166, (char)175, (char)69, (char)248, (char)23}));
            assert(pack.flags_GET() == (char)13);
            assert(pack.len_GET() == (char)161);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)13) ;
        p233.len_SET((char)161) ;
        p233.data__SET(new char[] {(char)240, (char)51, (char)197, (char)131, (char)190, (char)218, (char)149, (char)249, (char)9, (char)11, (char)163, (char)168, (char)101, (char)224, (char)203, (char)74, (char)192, (char)84, (char)153, (char)88, (char)220, (char)219, (char)116, (char)186, (char)193, (char)112, (char)149, (char)42, (char)50, (char)193, (char)26, (char)64, (char)164, (char)33, (char)106, (char)223, (char)179, (char)118, (char)166, (char)226, (char)192, (char)171, (char)66, (char)81, (char)0, (char)159, (char)251, (char)194, (char)233, (char)59, (char)189, (char)96, (char)230, (char)227, (char)202, (char)229, (char)197, (char)83, (char)174, (char)219, (char)199, (char)241, (char)151, (char)159, (char)252, (char)60, (char)154, (char)204, (char)127, (char)0, (char)19, (char)130, (char)60, (char)28, (char)12, (char)222, (char)126, (char)70, (char)189, (char)111, (char)130, (char)45, (char)6, (char)25, (char)179, (char)122, (char)122, (char)154, (char)30, (char)113, (char)12, (char)195, (char)20, (char)174, (char)141, (char)199, (char)204, (char)2, (char)60, (char)28, (char)200, (char)225, (char)42, (char)158, (char)188, (char)45, (char)136, (char)125, (char)181, (char)128, (char)6, (char)220, (char)29, (char)87, (char)197, (char)138, (char)22, (char)243, (char)112, (char)127, (char)188, (char)214, (char)126, (char)58, (char)146, (char)114, (char)130, (char)62, (char)45, (char)57, (char)239, (char)90, (char)172, (char)222, (char)16, (char)247, (char)166, (char)107, (char)91, (char)94, (char)197, (char)109, (char)156, (char)57, (char)111, (char)75, (char)62, (char)69, (char)225, (char)12, (char)51, (char)124, (char)104, (char)252, (char)116, (char)176, (char)28, (char)139, (char)51, (char)233, (char)187, (char)239, (char)242, (char)247, (char)238, (char)228, (char)70, (char)46, (char)184, (char)219, (char)248, (char)84, (char)58, (char)97, (char)122, (char)166, (char)175, (char)69, (char)248, (char)23}, 0) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
            assert(pack.airspeed_sp_GET() == (char)46);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
            assert(pack.climb_rate_GET() == (byte)62);
            assert(pack.heading_GET() == (char)26216);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
            assert(pack.longitude_GET() == 726550397);
            assert(pack.airspeed_GET() == (char)115);
            assert(pack.wp_num_GET() == (char)48);
            assert(pack.altitude_sp_GET() == (short)17390);
            assert(pack.wp_distance_GET() == (char)60972);
            assert(pack.throttle_GET() == (byte)28);
            assert(pack.altitude_amsl_GET() == (short) -5436);
            assert(pack.pitch_GET() == (short)15615);
            assert(pack.custom_mode_GET() == 4244207541L);
            assert(pack.temperature_GET() == (byte) - 96);
            assert(pack.temperature_air_GET() == (byte) - 93);
            assert(pack.latitude_GET() == -568395411);
            assert(pack.failsafe_GET() == (char)150);
            assert(pack.roll_GET() == (short)30381);
            assert(pack.heading_sp_GET() == (short) -5849);
            assert(pack.groundspeed_GET() == (char)16);
            assert(pack.battery_remaining_GET() == (char)229);
            assert(pack.gps_nsat_GET() == (char)13);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS) ;
        p234.throttle_SET((byte)28) ;
        p234.latitude_SET(-568395411) ;
        p234.airspeed_sp_SET((char)46) ;
        p234.failsafe_SET((char)150) ;
        p234.longitude_SET(726550397) ;
        p234.climb_rate_SET((byte)62) ;
        p234.altitude_amsl_SET((short) -5436) ;
        p234.temperature_air_SET((byte) - 93) ;
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED) ;
        p234.altitude_sp_SET((short)17390) ;
        p234.wp_distance_SET((char)60972) ;
        p234.pitch_SET((short)15615) ;
        p234.roll_SET((short)30381) ;
        p234.wp_num_SET((char)48) ;
        p234.temperature_SET((byte) - 96) ;
        p234.custom_mode_SET(4244207541L) ;
        p234.groundspeed_SET((char)16) ;
        p234.heading_sp_SET((short) -5849) ;
        p234.battery_remaining_SET((char)229) ;
        p234.gps_nsat_SET((char)13) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED) ;
        p234.heading_SET((char)26216) ;
        p234.airspeed_SET((char)115) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.clipping_2_GET() == 3015143154L);
            assert(pack.vibration_y_GET() == 3.980854E37F);
            assert(pack.time_usec_GET() == 4815058060503115967L);
            assert(pack.vibration_z_GET() == -2.1660646E38F);
            assert(pack.clipping_0_GET() == 3581628006L);
            assert(pack.vibration_x_GET() == 9.153653E37F);
            assert(pack.clipping_1_GET() == 3840049319L);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.clipping_1_SET(3840049319L) ;
        p241.vibration_y_SET(3.980854E37F) ;
        p241.clipping_2_SET(3015143154L) ;
        p241.clipping_0_SET(3581628006L) ;
        p241.vibration_z_SET(-2.1660646E38F) ;
        p241.time_usec_SET(4815058060503115967L) ;
        p241.vibration_x_SET(9.153653E37F) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.7005651E38F, 2.453629E38F, -3.2080403E38F, 3.3095078E38F}));
            assert(pack.x_GET() == -2.929456E37F);
            assert(pack.latitude_GET() == 1031272852);
            assert(pack.approach_x_GET() == 2.001163E38F);
            assert(pack.time_usec_TRY(ph) == 2910079478678184640L);
            assert(pack.altitude_GET() == 158779588);
            assert(pack.longitude_GET() == -1356658501);
            assert(pack.y_GET() == -7.3550817E37F);
            assert(pack.z_GET() == 2.8760625E38F);
            assert(pack.approach_y_GET() == -4.9082503E37F);
            assert(pack.approach_z_GET() == 1.8132024E38F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.altitude_SET(158779588) ;
        p242.longitude_SET(-1356658501) ;
        p242.y_SET(-7.3550817E37F) ;
        p242.latitude_SET(1031272852) ;
        p242.approach_y_SET(-4.9082503E37F) ;
        p242.x_SET(-2.929456E37F) ;
        p242.approach_x_SET(2.001163E38F) ;
        p242.z_SET(2.8760625E38F) ;
        p242.approach_z_SET(1.8132024E38F) ;
        p242.q_SET(new float[] {1.7005651E38F, 2.453629E38F, -3.2080403E38F, 3.3095078E38F}, 0) ;
        p242.time_usec_SET(2910079478678184640L, PH) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)61);
            assert(pack.time_usec_TRY(ph) == 5510892271019763533L);
            assert(pack.y_GET() == 1.5174374E38F);
            assert(pack.x_GET() == -2.8989386E38F);
            assert(pack.approach_y_GET() == -3.2240468E38F);
            assert(pack.latitude_GET() == -1372571431);
            assert(pack.longitude_GET() == -1856058629);
            assert(pack.altitude_GET() == -1414153547);
            assert(pack.z_GET() == -1.9756493E38F);
            assert(pack.approach_x_GET() == 9.631417E37F);
            assert(pack.approach_z_GET() == 9.984047E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.3888587E38F, -1.0359905E38F, -2.814521E38F, -2.1256438E38F}));
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.approach_y_SET(-3.2240468E38F) ;
        p243.x_SET(-2.8989386E38F) ;
        p243.longitude_SET(-1856058629) ;
        p243.target_system_SET((char)61) ;
        p243.q_SET(new float[] {-1.3888587E38F, -1.0359905E38F, -2.814521E38F, -2.1256438E38F}, 0) ;
        p243.time_usec_SET(5510892271019763533L, PH) ;
        p243.y_SET(1.5174374E38F) ;
        p243.approach_x_SET(9.631417E37F) ;
        p243.altitude_SET(-1414153547) ;
        p243.z_SET(-1.9756493E38F) ;
        p243.latitude_SET(-1372571431) ;
        p243.approach_z_SET(9.984047E37F) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)35820);
            assert(pack.interval_us_GET() == 51669620);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(51669620) ;
        p244.message_id_SET((char)35820) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED) ;
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 365295893);
            assert(pack.squawk_GET() == (char)16696);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT);
            assert(pack.tslc_GET() == (char)149);
            assert(pack.flags_GET() == ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK);
            assert(pack.ICAO_address_GET() == 1988384692L);
            assert(pack.lon_GET() == -1962530787);
            assert(pack.hor_velocity_GET() == (char)39869);
            assert(pack.ver_velocity_GET() == (short) -28128);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            assert(pack.heading_GET() == (char)38076);
            assert(pack.callsign_LEN(ph) == 2);
            assert(pack.callsign_TRY(ph).equals("ya"));
            assert(pack.altitude_GET() == 1189855165);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.heading_SET((char)38076) ;
        p246.lon_SET(-1962530787) ;
        p246.ICAO_address_SET(1988384692L) ;
        p246.altitude_SET(1189855165) ;
        p246.ver_velocity_SET((short) -28128) ;
        p246.squawk_SET((char)16696) ;
        p246.tslc_SET((char)149) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT) ;
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK) ;
        p246.hor_velocity_SET((char)39869) ;
        p246.callsign_SET("ya", PH) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.lat_SET(365295893) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.altitude_minimum_delta_GET() == -2.607035E38F);
            assert(pack.id_GET() == 672573658L);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
            assert(pack.time_to_minimum_delta_GET() == 1.1550237E38F);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
            assert(pack.horizontal_minimum_delta_GET() == 1.4301666E38F);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.id_SET(672573658L) ;
        p247.time_to_minimum_delta_SET(1.1550237E38F) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE) ;
        p247.horizontal_minimum_delta_SET(1.4301666E38F) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE) ;
        p247.altitude_minimum_delta_SET(-2.607035E38F) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)145, (char)80, (char)5, (char)42, (char)111, (char)221, (char)25, (char)19, (char)176, (char)54, (char)81, (char)88, (char)114, (char)114, (char)54, (char)151, (char)139, (char)141, (char)166, (char)143, (char)181, (char)216, (char)13, (char)101, (char)81, (char)64, (char)95, (char)108, (char)202, (char)83, (char)183, (char)185, (char)26, (char)9, (char)227, (char)127, (char)180, (char)248, (char)154, (char)242, (char)37, (char)28, (char)158, (char)33, (char)229, (char)5, (char)53, (char)21, (char)215, (char)213, (char)73, (char)3, (char)21, (char)123, (char)30, (char)225, (char)13, (char)247, (char)229, (char)101, (char)19, (char)3, (char)211, (char)172, (char)63, (char)83, (char)11, (char)16, (char)157, (char)179, (char)141, (char)89, (char)205, (char)225, (char)233, (char)222, (char)107, (char)137, (char)186, (char)19, (char)224, (char)120, (char)35, (char)129, (char)93, (char)223, (char)129, (char)43, (char)72, (char)141, (char)38, (char)226, (char)255, (char)180, (char)104, (char)91, (char)35, (char)188, (char)160, (char)169, (char)74, (char)24, (char)110, (char)185, (char)183, (char)45, (char)59, (char)166, (char)5, (char)5, (char)231, (char)19, (char)158, (char)41, (char)202, (char)13, (char)163, (char)132, (char)103, (char)101, (char)185, (char)35, (char)177, (char)110, (char)163, (char)139, (char)37, (char)116, (char)222, (char)165, (char)202, (char)15, (char)170, (char)217, (char)101, (char)214, (char)9, (char)84, (char)1, (char)135, (char)244, (char)39, (char)147, (char)70, (char)183, (char)240, (char)102, (char)219, (char)244, (char)169, (char)56, (char)218, (char)239, (char)16, (char)160, (char)115, (char)141, (char)73, (char)16, (char)132, (char)147, (char)35, (char)62, (char)192, (char)22, (char)113, (char)139, (char)50, (char)62, (char)125, (char)47, (char)123, (char)191, (char)250, (char)19, (char)125, (char)29, (char)123, (char)143, (char)128, (char)52, (char)219, (char)136, (char)11, (char)18, (char)238, (char)193, (char)50, (char)119, (char)28, (char)91, (char)230, (char)189, (char)49, (char)163, (char)77, (char)99, (char)52, (char)27, (char)148, (char)17, (char)174, (char)151, (char)185, (char)238, (char)202, (char)147, (char)92, (char)41, (char)46, (char)81, (char)173, (char)103, (char)169, (char)111, (char)79, (char)151, (char)165, (char)187, (char)64, (char)55, (char)203, (char)58, (char)169, (char)219, (char)75, (char)1, (char)143, (char)71, (char)178, (char)196, (char)127, (char)84, (char)227, (char)131, (char)230, (char)230, (char)71, (char)188, (char)159, (char)159, (char)222, (char)5, (char)36, (char)131, (char)108, (char)49, (char)149, (char)36}));
            assert(pack.target_component_GET() == (char)208);
            assert(pack.target_network_GET() == (char)40);
            assert(pack.target_system_GET() == (char)134);
            assert(pack.message_type_GET() == (char)41437);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.payload_SET(new char[] {(char)145, (char)80, (char)5, (char)42, (char)111, (char)221, (char)25, (char)19, (char)176, (char)54, (char)81, (char)88, (char)114, (char)114, (char)54, (char)151, (char)139, (char)141, (char)166, (char)143, (char)181, (char)216, (char)13, (char)101, (char)81, (char)64, (char)95, (char)108, (char)202, (char)83, (char)183, (char)185, (char)26, (char)9, (char)227, (char)127, (char)180, (char)248, (char)154, (char)242, (char)37, (char)28, (char)158, (char)33, (char)229, (char)5, (char)53, (char)21, (char)215, (char)213, (char)73, (char)3, (char)21, (char)123, (char)30, (char)225, (char)13, (char)247, (char)229, (char)101, (char)19, (char)3, (char)211, (char)172, (char)63, (char)83, (char)11, (char)16, (char)157, (char)179, (char)141, (char)89, (char)205, (char)225, (char)233, (char)222, (char)107, (char)137, (char)186, (char)19, (char)224, (char)120, (char)35, (char)129, (char)93, (char)223, (char)129, (char)43, (char)72, (char)141, (char)38, (char)226, (char)255, (char)180, (char)104, (char)91, (char)35, (char)188, (char)160, (char)169, (char)74, (char)24, (char)110, (char)185, (char)183, (char)45, (char)59, (char)166, (char)5, (char)5, (char)231, (char)19, (char)158, (char)41, (char)202, (char)13, (char)163, (char)132, (char)103, (char)101, (char)185, (char)35, (char)177, (char)110, (char)163, (char)139, (char)37, (char)116, (char)222, (char)165, (char)202, (char)15, (char)170, (char)217, (char)101, (char)214, (char)9, (char)84, (char)1, (char)135, (char)244, (char)39, (char)147, (char)70, (char)183, (char)240, (char)102, (char)219, (char)244, (char)169, (char)56, (char)218, (char)239, (char)16, (char)160, (char)115, (char)141, (char)73, (char)16, (char)132, (char)147, (char)35, (char)62, (char)192, (char)22, (char)113, (char)139, (char)50, (char)62, (char)125, (char)47, (char)123, (char)191, (char)250, (char)19, (char)125, (char)29, (char)123, (char)143, (char)128, (char)52, (char)219, (char)136, (char)11, (char)18, (char)238, (char)193, (char)50, (char)119, (char)28, (char)91, (char)230, (char)189, (char)49, (char)163, (char)77, (char)99, (char)52, (char)27, (char)148, (char)17, (char)174, (char)151, (char)185, (char)238, (char)202, (char)147, (char)92, (char)41, (char)46, (char)81, (char)173, (char)103, (char)169, (char)111, (char)79, (char)151, (char)165, (char)187, (char)64, (char)55, (char)203, (char)58, (char)169, (char)219, (char)75, (char)1, (char)143, (char)71, (char)178, (char)196, (char)127, (char)84, (char)227, (char)131, (char)230, (char)230, (char)71, (char)188, (char)159, (char)159, (char)222, (char)5, (char)36, (char)131, (char)108, (char)49, (char)149, (char)36}, 0) ;
        p248.target_system_SET((char)134) ;
        p248.target_network_SET((char)40) ;
        p248.target_component_SET((char)208) ;
        p248.message_type_SET((char)41437) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.address_GET() == (char)8795);
            assert(pack.ver_GET() == (char)252);
            assert(pack.type_GET() == (char)79);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)5, (byte)57, (byte) - 26, (byte) - 65, (byte)57, (byte)21, (byte)114, (byte) - 124, (byte)52, (byte) - 83, (byte) - 82, (byte)86, (byte) - 29, (byte)8, (byte)37, (byte)9, (byte)24, (byte)112, (byte)48, (byte) - 36, (byte) - 108, (byte) - 119, (byte) - 65, (byte)21, (byte)11, (byte) - 33, (byte) - 77, (byte)96, (byte) - 26, (byte) - 16, (byte)38, (byte)14}));
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.type_SET((char)79) ;
        p249.ver_SET((char)252) ;
        p249.address_SET((char)8795) ;
        p249.value_SET(new byte[] {(byte)5, (byte)57, (byte) - 26, (byte) - 65, (byte)57, (byte)21, (byte)114, (byte) - 124, (byte)52, (byte) - 83, (byte) - 82, (byte)86, (byte) - 29, (byte)8, (byte)37, (byte)9, (byte)24, (byte)112, (byte)48, (byte) - 36, (byte) - 108, (byte) - 119, (byte) - 65, (byte)21, (byte)11, (byte) - 33, (byte) - 77, (byte)96, (byte) - 26, (byte) - 16, (byte)38, (byte)14}, 0) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 1967578284579733838L);
            assert(pack.z_GET() == -2.036908E38F);
            assert(pack.name_LEN(ph) == 7);
            assert(pack.name_TRY(ph).equals("ozEgbbf"));
            assert(pack.x_GET() == 1.3550924E38F);
            assert(pack.y_GET() == 7.739882E37F);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.y_SET(7.739882E37F) ;
        p250.time_usec_SET(1967578284579733838L) ;
        p250.x_SET(1.3550924E38F) ;
        p250.name_SET("ozEgbbf", PH) ;
        p250.z_SET(-2.036908E38F) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == 2.831682E38F);
            assert(pack.name_LEN(ph) == 8);
            assert(pack.name_TRY(ph).equals("mnlftsRw"));
            assert(pack.time_boot_ms_GET() == 2490040976L);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.name_SET("mnlftsRw", PH) ;
        p251.value_SET(2.831682E38F) ;
        p251.time_boot_ms_SET(2490040976L) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 3);
            assert(pack.name_TRY(ph).equals("pKs"));
            assert(pack.time_boot_ms_GET() == 4027232396L);
            assert(pack.value_GET() == 918446874);
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(4027232396L) ;
        p252.value_SET(918446874) ;
        p252.name_SET("pKs", PH) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_INFO);
            assert(pack.text_LEN(ph) == 39);
            assert(pack.text_TRY(ph).equals("cpvmljnjkklnlbuaxLSlkmQpcxzTxtdqmmjwiHt"));
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("cpvmljnjkklnlbuaxLSlkmQpcxzTxtdqmmjwiHt", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_INFO) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.ind_GET() == (char)223);
            assert(pack.value_GET() == 9.103282E37F);
            assert(pack.time_boot_ms_GET() == 2206021448L);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.value_SET(9.103282E37F) ;
        p254.ind_SET((char)223) ;
        p254.time_boot_ms_SET(2206021448L) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.initial_timestamp_GET() == 2609526795328824711L);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)0, (char)74, (char)66, (char)234, (char)5, (char)229, (char)232, (char)232, (char)49, (char)98, (char)132, (char)14, (char)65, (char)129, (char)139, (char)123, (char)19, (char)254, (char)192, (char)220, (char)91, (char)133, (char)255, (char)1, (char)166, (char)38, (char)103, (char)85, (char)145, (char)54, (char)78, (char)68}));
            assert(pack.target_component_GET() == (char)132);
            assert(pack.target_system_GET() == (char)89);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)89) ;
        p256.initial_timestamp_SET(2609526795328824711L) ;
        p256.target_component_SET((char)132) ;
        p256.secret_key_SET(new char[] {(char)0, (char)74, (char)66, (char)234, (char)5, (char)229, (char)232, (char)232, (char)49, (char)98, (char)132, (char)14, (char)65, (char)129, (char)139, (char)123, (char)19, (char)254, (char)192, (char)220, (char)91, (char)133, (char)255, (char)1, (char)166, (char)38, (char)103, (char)85, (char)145, (char)54, (char)78, (char)68}, 0) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1435218874L);
            assert(pack.last_change_ms_GET() == 278119190L);
            assert(pack.state_GET() == (char)176);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.state_SET((char)176) ;
        p257.last_change_ms_SET(278119190L) ;
        p257.time_boot_ms_SET(1435218874L) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)22);
            assert(pack.tune_LEN(ph) == 13);
            assert(pack.tune_TRY(ph).equals("notzlfbcrwLvb"));
            assert(pack.target_component_GET() == (char)146);
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)22) ;
        p258.tune_SET("notzlfbcrwLvb", PH) ;
        p258.target_component_SET((char)146) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.sensor_size_h_GET() == 2.7975135E37F);
            assert(pack.sensor_size_v_GET() == 2.036177E38F);
            assert(pack.flags_GET() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
            assert(pack.cam_definition_uri_LEN(ph) == 42);
            assert(pack.cam_definition_uri_TRY(ph).equals("zmosliAosvmgkxmusvucmdisfAfllvadfqknprcjwc"));
            assert(pack.time_boot_ms_GET() == 2759001281L);
            assert(pack.focal_length_GET() == -1.85735E38F);
            assert(pack.firmware_version_GET() == 1723432785L);
            assert(pack.resolution_v_GET() == (char)44360);
            assert(pack.lens_id_GET() == (char)46);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)162, (char)39, (char)139, (char)141, (char)131, (char)46, (char)57, (char)235, (char)102, (char)225, (char)174, (char)220, (char)56, (char)73, (char)213, (char)80, (char)121, (char)224, (char)7, (char)150, (char)142, (char)92, (char)77, (char)176, (char)33, (char)235, (char)207, (char)241, (char)7, (char)40, (char)65, (char)128}));
            assert(pack.cam_definition_version_GET() == (char)65470);
            assert(pack.resolution_h_GET() == (char)44131);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)9, (char)241, (char)111, (char)143, (char)237, (char)23, (char)250, (char)43, (char)21, (char)223, (char)211, (char)251, (char)30, (char)131, (char)123, (char)136, (char)149, (char)0, (char)31, (char)140, (char)1, (char)42, (char)89, (char)113, (char)116, (char)211, (char)39, (char)166, (char)190, (char)141, (char)236, (char)167}));
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.firmware_version_SET(1723432785L) ;
        p259.time_boot_ms_SET(2759001281L) ;
        p259.resolution_h_SET((char)44131) ;
        p259.sensor_size_h_SET(2.7975135E37F) ;
        p259.model_name_SET(new char[] {(char)162, (char)39, (char)139, (char)141, (char)131, (char)46, (char)57, (char)235, (char)102, (char)225, (char)174, (char)220, (char)56, (char)73, (char)213, (char)80, (char)121, (char)224, (char)7, (char)150, (char)142, (char)92, (char)77, (char)176, (char)33, (char)235, (char)207, (char)241, (char)7, (char)40, (char)65, (char)128}, 0) ;
        p259.cam_definition_uri_SET("zmosliAosvmgkxmusvucmdisfAfllvadfqknprcjwc", PH) ;
        p259.lens_id_SET((char)46) ;
        p259.focal_length_SET(-1.85735E38F) ;
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE) ;
        p259.sensor_size_v_SET(2.036177E38F) ;
        p259.vendor_name_SET(new char[] {(char)9, (char)241, (char)111, (char)143, (char)237, (char)23, (char)250, (char)43, (char)21, (char)223, (char)211, (char)251, (char)30, (char)131, (char)123, (char)136, (char)149, (char)0, (char)31, (char)140, (char)1, (char)42, (char)89, (char)113, (char)116, (char)211, (char)39, (char)166, (char)190, (char)141, (char)236, (char)167}, 0) ;
        p259.cam_definition_version_SET((char)65470) ;
        p259.resolution_v_SET((char)44360) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1832780036L);
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_VIDEO);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_VIDEO) ;
        p260.time_boot_ms_SET(1832780036L) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.storage_count_GET() == (char)154);
            assert(pack.write_speed_GET() == -6.970707E37F);
            assert(pack.used_capacity_GET() == 1.4993743E38F);
            assert(pack.available_capacity_GET() == 9.905468E37F);
            assert(pack.time_boot_ms_GET() == 4273930300L);
            assert(pack.read_speed_GET() == 1.8219792E38F);
            assert(pack.storage_id_GET() == (char)76);
            assert(pack.status_GET() == (char)167);
            assert(pack.total_capacity_GET() == 5.218491E37F);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.status_SET((char)167) ;
        p261.storage_count_SET((char)154) ;
        p261.available_capacity_SET(9.905468E37F) ;
        p261.total_capacity_SET(5.218491E37F) ;
        p261.read_speed_SET(1.8219792E38F) ;
        p261.write_speed_SET(-6.970707E37F) ;
        p261.time_boot_ms_SET(4273930300L) ;
        p261.storage_id_SET((char)76) ;
        p261.used_capacity_SET(1.4993743E38F) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.available_capacity_GET() == 1.1267824E37F);
            assert(pack.time_boot_ms_GET() == 2529310082L);
            assert(pack.image_interval_GET() == -3.2359694E38F);
            assert(pack.image_status_GET() == (char)55);
            assert(pack.video_status_GET() == (char)60);
            assert(pack.recording_time_ms_GET() == 3073895627L);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.image_status_SET((char)55) ;
        p262.video_status_SET((char)60) ;
        p262.available_capacity_SET(1.1267824E37F) ;
        p262.recording_time_ms_SET(3073895627L) ;
        p262.time_boot_ms_SET(2529310082L) ;
        p262.image_interval_SET(-3.2359694E38F) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -1568713193);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.9899726E38F, -6.023413E37F, -3.5176442E37F, 2.3478662E38F}));
            assert(pack.lat_GET() == -1425308414);
            assert(pack.lon_GET() == -437445664);
            assert(pack.time_utc_GET() == 4369172056989484679L);
            assert(pack.time_boot_ms_GET() == 2254826319L);
            assert(pack.capture_result_GET() == (byte)8);
            assert(pack.image_index_GET() == -18470341);
            assert(pack.camera_id_GET() == (char)73);
            assert(pack.file_url_LEN(ph) == 88);
            assert(pack.file_url_TRY(ph).equals("kprupieQvficvratynnfjttzxajvklwykfiopzsszEbdrkbphygdzfVpumxhzmvPqsrgrFlusckizcamzrvlqFoe"));
            assert(pack.relative_alt_GET() == -1547532436);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.alt_SET(-1568713193) ;
        p263.relative_alt_SET(-1547532436) ;
        p263.lat_SET(-1425308414) ;
        p263.lon_SET(-437445664) ;
        p263.capture_result_SET((byte)8) ;
        p263.q_SET(new float[] {2.9899726E38F, -6.023413E37F, -3.5176442E37F, 2.3478662E38F}, 0) ;
        p263.file_url_SET("kprupieQvficvratynnfjttzxajvklwykfiopzsszEbdrkbphygdzfVpumxhzmvPqsrgrFlusckizcamzrvlqFoe", PH) ;
        p263.image_index_SET(-18470341) ;
        p263.time_boot_ms_SET(2254826319L) ;
        p263.time_utc_SET(4369172056989484679L) ;
        p263.camera_id_SET((char)73) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.flight_uuid_GET() == 8735980147481828212L);
            assert(pack.arming_time_utc_GET() == 6889316438577757134L);
            assert(pack.time_boot_ms_GET() == 3205328033L);
            assert(pack.takeoff_time_utc_GET() == 7796877453273521726L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.time_boot_ms_SET(3205328033L) ;
        p264.arming_time_utc_SET(6889316438577757134L) ;
        p264.takeoff_time_utc_SET(7796877453273521726L) ;
        p264.flight_uuid_SET(8735980147481828212L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 2.7703695E38F);
            assert(pack.roll_GET() == -1.1502952E38F);
            assert(pack.time_boot_ms_GET() == 3207530057L);
            assert(pack.yaw_GET() == 3.3978137E38F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.pitch_SET(2.7703695E38F) ;
        p265.time_boot_ms_SET(3207530057L) ;
        p265.yaw_SET(3.3978137E38F) ;
        p265.roll_SET(-1.1502952E38F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)190, (char)13, (char)11, (char)126, (char)24, (char)73, (char)212, (char)216, (char)150, (char)49, (char)65, (char)216, (char)47, (char)20, (char)78, (char)105, (char)143, (char)82, (char)28, (char)118, (char)196, (char)200, (char)9, (char)237, (char)234, (char)163, (char)148, (char)58, (char)92, (char)178, (char)80, (char)26, (char)214, (char)75, (char)85, (char)67, (char)78, (char)39, (char)128, (char)197, (char)188, (char)123, (char)75, (char)3, (char)78, (char)73, (char)169, (char)145, (char)52, (char)98, (char)48, (char)40, (char)220, (char)0, (char)90, (char)252, (char)25, (char)241, (char)233, (char)138, (char)248, (char)23, (char)227, (char)65, (char)120, (char)89, (char)203, (char)7, (char)47, (char)233, (char)92, (char)231, (char)77, (char)165, (char)99, (char)110, (char)238, (char)130, (char)92, (char)247, (char)106, (char)73, (char)59, (char)140, (char)149, (char)154, (char)83, (char)180, (char)35, (char)217, (char)128, (char)54, (char)69, (char)200, (char)43, (char)102, (char)43, (char)214, (char)165, (char)79, (char)61, (char)7, (char)39, (char)81, (char)99, (char)45, (char)199, (char)212, (char)24, (char)226, (char)35, (char)46, (char)223, (char)174, (char)88, (char)220, (char)191, (char)207, (char)130, (char)182, (char)168, (char)187, (char)156, (char)246, (char)194, (char)188, (char)229, (char)5, (char)219, (char)210, (char)34, (char)89, (char)152, (char)103, (char)98, (char)155, (char)234, (char)56, (char)130, (char)118, (char)160, (char)143, (char)216, (char)1, (char)4, (char)155, (char)126, (char)175, (char)249, (char)43, (char)250, (char)148, (char)83, (char)86, (char)91, (char)26, (char)80, (char)235, (char)62, (char)111, (char)73, (char)162, (char)120, (char)153, (char)86, (char)116, (char)209, (char)225, (char)92, (char)101, (char)16, (char)20, (char)165, (char)128, (char)97, (char)168, (char)59, (char)117, (char)245, (char)60, (char)187, (char)148, (char)147, (char)221, (char)169, (char)135, (char)181, (char)78, (char)200, (char)46, (char)242, (char)106, (char)3, (char)79, (char)204, (char)43, (char)38, (char)26, (char)61, (char)231, (char)20, (char)224, (char)87, (char)78, (char)94, (char)33, (char)205, (char)115, (char)170, (char)23, (char)15, (char)4, (char)253, (char)94, (char)155, (char)12, (char)61, (char)12, (char)79, (char)247, (char)202, (char)254, (char)69, (char)105, (char)207, (char)179, (char)82, (char)193, (char)226, (char)67, (char)230, (char)7, (char)226, (char)106, (char)233, (char)108, (char)143, (char)44, (char)140, (char)28, (char)131, (char)199, (char)172, (char)71, (char)254, (char)197, (char)43, (char)199, (char)43}));
            assert(pack.sequence_GET() == (char)23291);
            assert(pack.target_component_GET() == (char)242);
            assert(pack.length_GET() == (char)2);
            assert(pack.first_message_offset_GET() == (char)151);
            assert(pack.target_system_GET() == (char)207);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.data__SET(new char[] {(char)190, (char)13, (char)11, (char)126, (char)24, (char)73, (char)212, (char)216, (char)150, (char)49, (char)65, (char)216, (char)47, (char)20, (char)78, (char)105, (char)143, (char)82, (char)28, (char)118, (char)196, (char)200, (char)9, (char)237, (char)234, (char)163, (char)148, (char)58, (char)92, (char)178, (char)80, (char)26, (char)214, (char)75, (char)85, (char)67, (char)78, (char)39, (char)128, (char)197, (char)188, (char)123, (char)75, (char)3, (char)78, (char)73, (char)169, (char)145, (char)52, (char)98, (char)48, (char)40, (char)220, (char)0, (char)90, (char)252, (char)25, (char)241, (char)233, (char)138, (char)248, (char)23, (char)227, (char)65, (char)120, (char)89, (char)203, (char)7, (char)47, (char)233, (char)92, (char)231, (char)77, (char)165, (char)99, (char)110, (char)238, (char)130, (char)92, (char)247, (char)106, (char)73, (char)59, (char)140, (char)149, (char)154, (char)83, (char)180, (char)35, (char)217, (char)128, (char)54, (char)69, (char)200, (char)43, (char)102, (char)43, (char)214, (char)165, (char)79, (char)61, (char)7, (char)39, (char)81, (char)99, (char)45, (char)199, (char)212, (char)24, (char)226, (char)35, (char)46, (char)223, (char)174, (char)88, (char)220, (char)191, (char)207, (char)130, (char)182, (char)168, (char)187, (char)156, (char)246, (char)194, (char)188, (char)229, (char)5, (char)219, (char)210, (char)34, (char)89, (char)152, (char)103, (char)98, (char)155, (char)234, (char)56, (char)130, (char)118, (char)160, (char)143, (char)216, (char)1, (char)4, (char)155, (char)126, (char)175, (char)249, (char)43, (char)250, (char)148, (char)83, (char)86, (char)91, (char)26, (char)80, (char)235, (char)62, (char)111, (char)73, (char)162, (char)120, (char)153, (char)86, (char)116, (char)209, (char)225, (char)92, (char)101, (char)16, (char)20, (char)165, (char)128, (char)97, (char)168, (char)59, (char)117, (char)245, (char)60, (char)187, (char)148, (char)147, (char)221, (char)169, (char)135, (char)181, (char)78, (char)200, (char)46, (char)242, (char)106, (char)3, (char)79, (char)204, (char)43, (char)38, (char)26, (char)61, (char)231, (char)20, (char)224, (char)87, (char)78, (char)94, (char)33, (char)205, (char)115, (char)170, (char)23, (char)15, (char)4, (char)253, (char)94, (char)155, (char)12, (char)61, (char)12, (char)79, (char)247, (char)202, (char)254, (char)69, (char)105, (char)207, (char)179, (char)82, (char)193, (char)226, (char)67, (char)230, (char)7, (char)226, (char)106, (char)233, (char)108, (char)143, (char)44, (char)140, (char)28, (char)131, (char)199, (char)172, (char)71, (char)254, (char)197, (char)43, (char)199, (char)43}, 0) ;
        p266.length_SET((char)2) ;
        p266.target_component_SET((char)242) ;
        p266.sequence_SET((char)23291) ;
        p266.first_message_offset_SET((char)151) ;
        p266.target_system_SET((char)207) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)22);
            assert(pack.first_message_offset_GET() == (char)155);
            assert(pack.length_GET() == (char)84);
            assert(pack.sequence_GET() == (char)16529);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)94, (char)187, (char)221, (char)73, (char)128, (char)22, (char)156, (char)255, (char)143, (char)192, (char)77, (char)45, (char)95, (char)67, (char)127, (char)141, (char)119, (char)11, (char)5, (char)108, (char)106, (char)69, (char)143, (char)24, (char)6, (char)102, (char)245, (char)157, (char)107, (char)139, (char)59, (char)97, (char)254, (char)30, (char)248, (char)230, (char)47, (char)183, (char)44, (char)6, (char)140, (char)168, (char)201, (char)154, (char)129, (char)104, (char)143, (char)133, (char)69, (char)64, (char)192, (char)192, (char)162, (char)38, (char)10, (char)253, (char)223, (char)221, (char)161, (char)120, (char)43, (char)138, (char)1, (char)19, (char)227, (char)102, (char)21, (char)240, (char)222, (char)75, (char)196, (char)21, (char)237, (char)97, (char)142, (char)66, (char)119, (char)178, (char)156, (char)88, (char)71, (char)133, (char)117, (char)71, (char)122, (char)152, (char)124, (char)199, (char)21, (char)9, (char)36, (char)80, (char)35, (char)73, (char)249, (char)34, (char)25, (char)18, (char)36, (char)59, (char)134, (char)145, (char)8, (char)222, (char)13, (char)64, (char)103, (char)5, (char)78, (char)81, (char)109, (char)216, (char)83, (char)7, (char)127, (char)131, (char)115, (char)204, (char)107, (char)116, (char)159, (char)60, (char)185, (char)70, (char)78, (char)102, (char)150, (char)57, (char)229, (char)154, (char)251, (char)148, (char)148, (char)8, (char)63, (char)134, (char)151, (char)25, (char)100, (char)83, (char)17, (char)172, (char)59, (char)55, (char)173, (char)103, (char)229, (char)35, (char)70, (char)76, (char)204, (char)142, (char)254, (char)28, (char)123, (char)2, (char)222, (char)87, (char)130, (char)194, (char)124, (char)26, (char)78, (char)232, (char)189, (char)158, (char)182, (char)168, (char)191, (char)164, (char)99, (char)56, (char)131, (char)189, (char)65, (char)101, (char)154, (char)235, (char)240, (char)38, (char)139, (char)113, (char)177, (char)104, (char)192, (char)15, (char)29, (char)34, (char)195, (char)40, (char)127, (char)99, (char)94, (char)87, (char)10, (char)171, (char)118, (char)157, (char)196, (char)28, (char)192, (char)168, (char)80, (char)107, (char)95, (char)23, (char)201, (char)91, (char)10, (char)221, (char)182, (char)141, (char)167, (char)252, (char)198, (char)15, (char)63, (char)150, (char)174, (char)183, (char)126, (char)223, (char)219, (char)209, (char)168, (char)132, (char)206, (char)61, (char)19, (char)1, (char)200, (char)82, (char)202, (char)173, (char)116, (char)53, (char)194, (char)78, (char)81, (char)217, (char)185, (char)46, (char)23, (char)240, (char)138, (char)121, (char)179, (char)48, (char)187}));
            assert(pack.target_system_GET() == (char)78);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_system_SET((char)78) ;
        p267.target_component_SET((char)22) ;
        p267.sequence_SET((char)16529) ;
        p267.length_SET((char)84) ;
        p267.first_message_offset_SET((char)155) ;
        p267.data__SET(new char[] {(char)94, (char)187, (char)221, (char)73, (char)128, (char)22, (char)156, (char)255, (char)143, (char)192, (char)77, (char)45, (char)95, (char)67, (char)127, (char)141, (char)119, (char)11, (char)5, (char)108, (char)106, (char)69, (char)143, (char)24, (char)6, (char)102, (char)245, (char)157, (char)107, (char)139, (char)59, (char)97, (char)254, (char)30, (char)248, (char)230, (char)47, (char)183, (char)44, (char)6, (char)140, (char)168, (char)201, (char)154, (char)129, (char)104, (char)143, (char)133, (char)69, (char)64, (char)192, (char)192, (char)162, (char)38, (char)10, (char)253, (char)223, (char)221, (char)161, (char)120, (char)43, (char)138, (char)1, (char)19, (char)227, (char)102, (char)21, (char)240, (char)222, (char)75, (char)196, (char)21, (char)237, (char)97, (char)142, (char)66, (char)119, (char)178, (char)156, (char)88, (char)71, (char)133, (char)117, (char)71, (char)122, (char)152, (char)124, (char)199, (char)21, (char)9, (char)36, (char)80, (char)35, (char)73, (char)249, (char)34, (char)25, (char)18, (char)36, (char)59, (char)134, (char)145, (char)8, (char)222, (char)13, (char)64, (char)103, (char)5, (char)78, (char)81, (char)109, (char)216, (char)83, (char)7, (char)127, (char)131, (char)115, (char)204, (char)107, (char)116, (char)159, (char)60, (char)185, (char)70, (char)78, (char)102, (char)150, (char)57, (char)229, (char)154, (char)251, (char)148, (char)148, (char)8, (char)63, (char)134, (char)151, (char)25, (char)100, (char)83, (char)17, (char)172, (char)59, (char)55, (char)173, (char)103, (char)229, (char)35, (char)70, (char)76, (char)204, (char)142, (char)254, (char)28, (char)123, (char)2, (char)222, (char)87, (char)130, (char)194, (char)124, (char)26, (char)78, (char)232, (char)189, (char)158, (char)182, (char)168, (char)191, (char)164, (char)99, (char)56, (char)131, (char)189, (char)65, (char)101, (char)154, (char)235, (char)240, (char)38, (char)139, (char)113, (char)177, (char)104, (char)192, (char)15, (char)29, (char)34, (char)195, (char)40, (char)127, (char)99, (char)94, (char)87, (char)10, (char)171, (char)118, (char)157, (char)196, (char)28, (char)192, (char)168, (char)80, (char)107, (char)95, (char)23, (char)201, (char)91, (char)10, (char)221, (char)182, (char)141, (char)167, (char)252, (char)198, (char)15, (char)63, (char)150, (char)174, (char)183, (char)126, (char)223, (char)219, (char)209, (char)168, (char)132, (char)206, (char)61, (char)19, (char)1, (char)200, (char)82, (char)202, (char)173, (char)116, (char)53, (char)194, (char)78, (char)81, (char)217, (char)185, (char)46, (char)23, (char)240, (char)138, (char)121, (char)179, (char)48, (char)187}, 0) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)43659);
            assert(pack.target_system_GET() == (char)4);
            assert(pack.target_component_GET() == (char)69);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_component_SET((char)69) ;
        p268.target_system_SET((char)4) ;
        p268.sequence_SET((char)43659) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.bitrate_GET() == 2111028246L);
            assert(pack.camera_id_GET() == (char)252);
            assert(pack.framerate_GET() == -3.9048552E37F);
            assert(pack.resolution_h_GET() == (char)47770);
            assert(pack.status_GET() == (char)230);
            assert(pack.resolution_v_GET() == (char)45143);
            assert(pack.uri_LEN(ph) == 207);
            assert(pack.uri_TRY(ph).equals("nwmozufizkvtbswyuhjarkoldykkcwfpngeqEdvotopNFcftftmixayTyngtuecuEhibnfcrifRbfkvnFctrfucFsneaoeitgtfgolmXMlzhfqowvinjnuwqceyzPwcmavAnaqephxczxAouaechLfdqKlbbuJapflvmSvxwgcwsbeofbapefyuedaqvngxglzqgoCvTznwFpql"));
            assert(pack.rotation_GET() == (char)38719);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.resolution_h_SET((char)47770) ;
        p269.resolution_v_SET((char)45143) ;
        p269.bitrate_SET(2111028246L) ;
        p269.framerate_SET(-3.9048552E37F) ;
        p269.rotation_SET((char)38719) ;
        p269.uri_SET("nwmozufizkvtbswyuhjarkoldykkcwfpngeqEdvotopNFcftftmixayTyngtuecuEhibnfcrifRbfkvnFctrfucFsneaoeitgtfgolmXMlzhfqowvinjnuwqceyzPwcmavAnaqephxczxAouaechLfdqKlbbuJapflvmSvxwgcwsbeofbapefyuedaqvngxglzqgoCvTznwFpql", PH) ;
        p269.camera_id_SET((char)252) ;
        p269.status_SET((char)230) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.uri_LEN(ph) == 52);
            assert(pack.uri_TRY(ph).equals("cdXktaleonkdNhoscyjpvrgfhufpmqabrFkfsuypeyKYgsbfcqQa"));
            assert(pack.target_component_GET() == (char)240);
            assert(pack.resolution_h_GET() == (char)12420);
            assert(pack.framerate_GET() == 2.3382639E38F);
            assert(pack.rotation_GET() == (char)40917);
            assert(pack.bitrate_GET() == 459913119L);
            assert(pack.target_system_GET() == (char)26);
            assert(pack.resolution_v_GET() == (char)58561);
            assert(pack.camera_id_GET() == (char)101);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_component_SET((char)240) ;
        p270.resolution_h_SET((char)12420) ;
        p270.resolution_v_SET((char)58561) ;
        p270.framerate_SET(2.3382639E38F) ;
        p270.bitrate_SET(459913119L) ;
        p270.camera_id_SET((char)101) ;
        p270.uri_SET("cdXktaleonkdNhoscyjpvrgfhufpmqabrFkfsuypeyKYgsbfcqQa", PH) ;
        p270.target_system_SET((char)26) ;
        p270.rotation_SET((char)40917) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.password_LEN(ph) == 18);
            assert(pack.password_TRY(ph).equals("kozxxzvbXGsqdcgwbx"));
            assert(pack.ssid_LEN(ph) == 1);
            assert(pack.ssid_TRY(ph).equals("c"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("kozxxzvbXGsqdcgwbx", PH) ;
        p299.ssid_SET("c", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)243, (char)252, (char)0, (char)21, (char)196, (char)109, (char)200, (char)197}));
            assert(pack.min_version_GET() == (char)49221);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)141, (char)170, (char)173, (char)71, (char)121, (char)22, (char)69, (char)229}));
            assert(pack.version_GET() == (char)14413);
            assert(pack.max_version_GET() == (char)35417);
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.max_version_SET((char)35417) ;
        p300.min_version_SET((char)49221) ;
        p300.spec_version_hash_SET(new char[] {(char)141, (char)170, (char)173, (char)71, (char)121, (char)22, (char)69, (char)229}, 0) ;
        p300.version_SET((char)14413) ;
        p300.library_version_hash_SET(new char[] {(char)243, (char)252, (char)0, (char)21, (char)196, (char)109, (char)200, (char)197}, 0) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
            assert(pack.sub_mode_GET() == (char)22);
            assert(pack.vendor_specific_status_code_GET() == (char)34658);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
            assert(pack.time_usec_GET() == 3376436184875113930L);
            assert(pack.uptime_sec_GET() == 1701039978L);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.uptime_sec_SET(1701039978L) ;
        p310.sub_mode_SET((char)22) ;
        p310.time_usec_SET(3376436184875113930L) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK) ;
        p310.vendor_specific_status_code_SET((char)34658) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 30);
            assert(pack.name_TRY(ph).equals("iwaroVmoguhqqattsZrlzhpodrzjgh"));
            assert(pack.sw_version_major_GET() == (char)194);
            assert(pack.sw_version_minor_GET() == (char)76);
            assert(pack.uptime_sec_GET() == 1430199555L);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)40, (char)214, (char)117, (char)123, (char)29, (char)74, (char)245, (char)208, (char)223, (char)194, (char)208, (char)6, (char)7, (char)0, (char)133, (char)127}));
            assert(pack.hw_version_minor_GET() == (char)145);
            assert(pack.time_usec_GET() == 4220579084789880276L);
            assert(pack.hw_version_major_GET() == (char)197);
            assert(pack.sw_vcs_commit_GET() == 2645427987L);
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.uptime_sec_SET(1430199555L) ;
        p311.sw_version_minor_SET((char)76) ;
        p311.hw_version_minor_SET((char)145) ;
        p311.hw_version_major_SET((char)197) ;
        p311.name_SET("iwaroVmoguhqqattsZrlzhpodrzjgh", PH) ;
        p311.sw_vcs_commit_SET(2645427987L) ;
        p311.hw_unique_id_SET(new char[] {(char)40, (char)214, (char)117, (char)123, (char)29, (char)74, (char)245, (char)208, (char)223, (char)194, (char)208, (char)6, (char)7, (char)0, (char)133, (char)127}, 0) ;
        p311.time_usec_SET(4220579084789880276L) ;
        p311.sw_version_major_SET((char)194) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)40);
            assert(pack.target_system_GET() == (char)119);
            assert(pack.param_id_LEN(ph) == 10);
            assert(pack.param_id_TRY(ph).equals("keeaqebhcg"));
            assert(pack.param_index_GET() == (short) -1834);
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_component_SET((char)40) ;
        p320.param_index_SET((short) -1834) ;
        p320.target_system_SET((char)119) ;
        p320.param_id_SET("keeaqebhcg", PH) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)44);
            assert(pack.target_component_GET() == (char)88);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)44) ;
        p321.target_component_SET((char)88) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)9364);
            assert(pack.param_value_LEN(ph) == 128);
            assert(pack.param_value_TRY(ph).equals("QwgumpxcklfedbaubKjkiUmxvwalqjmnosUHuJNwvrhizgqlfokizgyynpaAlgekqqKtvRdxsnqebbmukbtsqnrstJHAuyyQbRbopbfaeulgcGBfkaawmmisxiNjkepe"));
            assert(pack.param_id_LEN(ph) == 14);
            assert(pack.param_id_TRY(ph).equals("gtrBbvpwjonuXe"));
            assert(pack.param_index_GET() == (char)9856);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("gtrBbvpwjonuXe", PH) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16) ;
        p322.param_count_SET((char)9364) ;
        p322.param_index_SET((char)9856) ;
        p322.param_value_SET("QwgumpxcklfedbaubKjkiUmxvwalqjmnosUHuJNwvrhizgqlfokizgyynpaAlgekqqKtvRdxsnqebbmukbtsqnrstJHAuyyQbRbopbfaeulgcGBfkaawmmisxiNjkepe", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 105);
            assert(pack.param_value_TRY(ph).equals("ebikfjntgklcomyatHksffkwsgdakaogbmdLtsfopymtvsliuanvmrymYwumdbrvwaiaqxKawqwamTyrsevFecptydxbnhycizfgifjzu"));
            assert(pack.target_component_GET() == (char)120);
            assert(pack.target_system_GET() == (char)83);
            assert(pack.param_id_LEN(ph) == 10);
            assert(pack.param_id_TRY(ph).equals("kedMgzfbox"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_component_SET((char)120) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8) ;
        p323.param_value_SET("ebikfjntgklcomyatHksffkwsgdakaogbmdLtsfopymtvsliuanvmrymYwumdbrvwaiaqxKawqwamTyrsevFecptydxbnhycizfgifjzu", PH) ;
        p323.target_system_SET((char)83) ;
        p323.param_id_SET("kedMgzfbox", PH) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_FAILED);
            assert(pack.param_value_LEN(ph) == 27);
            assert(pack.param_value_TRY(ph).equals("dlnkrujclsusehpjeccwbcNfJzf"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
            assert(pack.param_id_LEN(ph) == 16);
            assert(pack.param_id_TRY(ph).equals("hwgmrtmwysjfiyhq"));
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("hwgmrtmwysjfiyhq", PH) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_FAILED) ;
        p324.param_value_SET("dlnkrujclsusehpjeccwbcNfJzf", PH) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)11761, (char)49372, (char)18289, (char)20967, (char)46406, (char)10824, (char)30910, (char)47347, (char)27222, (char)35815, (char)13743, (char)32423, (char)22578, (char)5344, (char)18048, (char)59885, (char)31616, (char)42689, (char)25001, (char)61267, (char)57531, (char)41521, (char)29030, (char)51117, (char)4245, (char)23037, (char)22628, (char)37829, (char)21050, (char)23627, (char)11501, (char)39635, (char)53203, (char)55841, (char)7183, (char)31513, (char)15733, (char)52011, (char)6754, (char)58346, (char)28708, (char)43338, (char)29398, (char)51170, (char)314, (char)52514, (char)2644, (char)20688, (char)30268, (char)27663, (char)21410, (char)30548, (char)18710, (char)19464, (char)64364, (char)15258, (char)13870, (char)28610, (char)22291, (char)14737, (char)28547, (char)26910, (char)18746, (char)2381, (char)61642, (char)43258, (char)41831, (char)35554, (char)58868, (char)271, (char)63565, (char)7266}));
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
            assert(pack.max_distance_GET() == (char)42970);
            assert(pack.increment_GET() == (char)179);
            assert(pack.time_usec_GET() == 7875971137087362173L);
            assert(pack.min_distance_GET() == (char)44980);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.increment_SET((char)179) ;
        p330.max_distance_SET((char)42970) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER) ;
        p330.distances_SET(new char[] {(char)11761, (char)49372, (char)18289, (char)20967, (char)46406, (char)10824, (char)30910, (char)47347, (char)27222, (char)35815, (char)13743, (char)32423, (char)22578, (char)5344, (char)18048, (char)59885, (char)31616, (char)42689, (char)25001, (char)61267, (char)57531, (char)41521, (char)29030, (char)51117, (char)4245, (char)23037, (char)22628, (char)37829, (char)21050, (char)23627, (char)11501, (char)39635, (char)53203, (char)55841, (char)7183, (char)31513, (char)15733, (char)52011, (char)6754, (char)58346, (char)28708, (char)43338, (char)29398, (char)51170, (char)314, (char)52514, (char)2644, (char)20688, (char)30268, (char)27663, (char)21410, (char)30548, (char)18710, (char)19464, (char)64364, (char)15258, (char)13870, (char)28610, (char)22291, (char)14737, (char)28547, (char)26910, (char)18746, (char)2381, (char)61642, (char)43258, (char)41831, (char)35554, (char)58868, (char)271, (char)63565, (char)7266}, 0) ;
        p330.time_usec_SET(7875971137087362173L) ;
        p330.min_distance_SET((char)44980) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVIONIX_ADSB_OUT_CFG.add((src, ph, pack) ->
        {
            assert(pack.ICAO_GET() == 1551941223L);
            assert(pack.emitterType_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ROTOCRAFT);
            assert(pack.aircraftSize_GET() == UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25M_W28P5M);
            assert(pack.callsign_LEN(ph) == 5);
            assert(pack.callsign_TRY(ph).equals("iezra"));
            assert(pack.gpsOffsetLat_GET() == UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_0M);
            assert(pack.rfSelect_GET() == UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY);
            assert(pack.gpsOffsetLon_GET() == UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR);
            assert(pack.stallSpeed_GET() == (char)61923);
        });
        GroundControl.UAVIONIX_ADSB_OUT_CFG p10001 = CommunicationChannel.new_UAVIONIX_ADSB_OUT_CFG();
        PH.setPack(p10001);
        p10001.gpsOffsetLon_SET(UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR) ;
        p10001.aircraftSize_SET(UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25M_W28P5M) ;
        p10001.callsign_SET("iezra", PH) ;
        p10001.gpsOffsetLat_SET(UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_0M) ;
        p10001.rfSelect_SET(UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY) ;
        p10001.ICAO_SET(1551941223L) ;
        p10001.stallSpeed_SET((char)61923) ;
        p10001.emitterType_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ROTOCRAFT) ;
        CommunicationChannel.instance.send(p10001);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVIONIX_ADSB_OUT_DYNAMIC.add((src, ph, pack) ->
        {
            assert(pack.state_GET() == UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND);
            assert(pack.baroAltMSL_GET() == 1275552009);
            assert(pack.squawk_GET() == (char)39400);
            assert(pack.gpsFix_GET() == UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0);
            assert(pack.VelEW_GET() == (short) -24029);
            assert(pack.utcTime_GET() == 179680671L);
            assert(pack.accuracyHor_GET() == 1456769342L);
            assert(pack.numSats_GET() == (char)12);
            assert(pack.gpsAlt_GET() == 838276038);
            assert(pack.velVert_GET() == (short) -14980);
            assert(pack.accuracyVel_GET() == (char)48717);
            assert(pack.gpsLat_GET() == 1960195126);
            assert(pack.emergencyStatus_GET() == UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_LIFEGUARD_EMERGENCY);
            assert(pack.gpsLon_GET() == -418495882);
            assert(pack.accuracyVert_GET() == (char)63398);
            assert(pack.velNS_GET() == (short)9112);
        });
        GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC p10002 = CommunicationChannel.new_UAVIONIX_ADSB_OUT_DYNAMIC();
        PH.setPack(p10002);
        p10002.gpsFix_SET(UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0) ;
        p10002.gpsLat_SET(1960195126) ;
        p10002.state_SET(UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND) ;
        p10002.accuracyVert_SET((char)63398) ;
        p10002.accuracyHor_SET(1456769342L) ;
        p10002.gpsLon_SET(-418495882) ;
        p10002.squawk_SET((char)39400) ;
        p10002.utcTime_SET(179680671L) ;
        p10002.accuracyVel_SET((char)48717) ;
        p10002.velNS_SET((short)9112) ;
        p10002.baroAltMSL_SET(1275552009) ;
        p10002.emergencyStatus_SET(UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_LIFEGUARD_EMERGENCY) ;
        p10002.velVert_SET((short) -14980) ;
        p10002.VelEW_SET((short) -24029) ;
        p10002.numSats_SET((char)12) ;
        p10002.gpsAlt_SET(838276038) ;
        CommunicationChannel.instance.send(p10002);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT.add((src, ph, pack) ->
        {
            assert(pack.rfHealth_GET() == UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_RX);
        });
        GroundControl.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT p10003 = CommunicationChannel.new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();
        PH.setPack(p10003);
        p10003.rfHealth_SET(UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_RX) ;
        CommunicationChannel.instance.send(p10003);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEVICE_OP_READ.add((src, ph, pack) ->
        {
            assert(pack.busname_LEN(ph) == 7);
            assert(pack.busname_TRY(ph).equals("ouzhuwd"));
            assert(pack.target_component_GET() == (char)117);
            assert(pack.count_GET() == (char)196);
            assert(pack.target_system_GET() == (char)249);
            assert(pack.regstart_GET() == (char)74);
            assert(pack.bustype_GET() == DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C);
            assert(pack.address_GET() == (char)176);
            assert(pack.bus_GET() == (char)123);
            assert(pack.request_id_GET() == 2339226929L);
        });
        GroundControl.DEVICE_OP_READ p11000 = CommunicationChannel.new_DEVICE_OP_READ();
        PH.setPack(p11000);
        p11000.bus_SET((char)123) ;
        p11000.target_component_SET((char)117) ;
        p11000.request_id_SET(2339226929L) ;
        p11000.target_system_SET((char)249) ;
        p11000.regstart_SET((char)74) ;
        p11000.count_SET((char)196) ;
        p11000.address_SET((char)176) ;
        p11000.busname_SET("ouzhuwd", PH) ;
        p11000.bustype_SET(DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C) ;
        CommunicationChannel.instance.send(p11000);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEVICE_OP_READ_REPLY.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)115);
            assert(pack.regstart_GET() == (char)14);
            assert(pack.request_id_GET() == 2221596042L);
            assert(pack.result_GET() == (char)28);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)81, (char)68, (char)157, (char)241, (char)167, (char)237, (char)15, (char)99, (char)143, (char)82, (char)186, (char)53, (char)28, (char)166, (char)175, (char)56, (char)209, (char)172, (char)95, (char)75, (char)181, (char)178, (char)167, (char)206, (char)80, (char)153, (char)28, (char)115, (char)193, (char)116, (char)70, (char)69, (char)86, (char)3, (char)34, (char)87, (char)150, (char)172, (char)64, (char)214, (char)131, (char)228, (char)193, (char)117, (char)82, (char)71, (char)163, (char)14, (char)78, (char)231, (char)221, (char)145, (char)153, (char)37, (char)203, (char)250, (char)159, (char)25, (char)141, (char)252, (char)237, (char)42, (char)134, (char)248, (char)185, (char)93, (char)180, (char)147, (char)240, (char)218, (char)123, (char)215, (char)169, (char)107, (char)127, (char)207, (char)21, (char)182, (char)106, (char)23, (char)18, (char)54, (char)174, (char)120, (char)181, (char)123, (char)25, (char)253, (char)127, (char)37, (char)77, (char)247, (char)251, (char)91, (char)163, (char)224, (char)219, (char)40, (char)178, (char)54, (char)41, (char)84, (char)97, (char)191, (char)32, (char)117, (char)119, (char)212, (char)198, (char)62, (char)100, (char)78, (char)192, (char)116, (char)244, (char)136, (char)114, (char)121, (char)205, (char)76, (char)189, (char)36, (char)197, (char)244, (char)1, (char)221, (char)115, (char)90}));
        });
        GroundControl.DEVICE_OP_READ_REPLY p11001 = CommunicationChannel.new_DEVICE_OP_READ_REPLY();
        PH.setPack(p11001);
        p11001.data__SET(new char[] {(char)81, (char)68, (char)157, (char)241, (char)167, (char)237, (char)15, (char)99, (char)143, (char)82, (char)186, (char)53, (char)28, (char)166, (char)175, (char)56, (char)209, (char)172, (char)95, (char)75, (char)181, (char)178, (char)167, (char)206, (char)80, (char)153, (char)28, (char)115, (char)193, (char)116, (char)70, (char)69, (char)86, (char)3, (char)34, (char)87, (char)150, (char)172, (char)64, (char)214, (char)131, (char)228, (char)193, (char)117, (char)82, (char)71, (char)163, (char)14, (char)78, (char)231, (char)221, (char)145, (char)153, (char)37, (char)203, (char)250, (char)159, (char)25, (char)141, (char)252, (char)237, (char)42, (char)134, (char)248, (char)185, (char)93, (char)180, (char)147, (char)240, (char)218, (char)123, (char)215, (char)169, (char)107, (char)127, (char)207, (char)21, (char)182, (char)106, (char)23, (char)18, (char)54, (char)174, (char)120, (char)181, (char)123, (char)25, (char)253, (char)127, (char)37, (char)77, (char)247, (char)251, (char)91, (char)163, (char)224, (char)219, (char)40, (char)178, (char)54, (char)41, (char)84, (char)97, (char)191, (char)32, (char)117, (char)119, (char)212, (char)198, (char)62, (char)100, (char)78, (char)192, (char)116, (char)244, (char)136, (char)114, (char)121, (char)205, (char)76, (char)189, (char)36, (char)197, (char)244, (char)1, (char)221, (char)115, (char)90}, 0) ;
        p11001.count_SET((char)115) ;
        p11001.request_id_SET(2221596042L) ;
        p11001.regstart_SET((char)14) ;
        p11001.result_SET((char)28) ;
        CommunicationChannel.instance.send(p11001);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEVICE_OP_WRITE.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)155, (char)214, (char)133, (char)127, (char)48, (char)240, (char)245, (char)120, (char)48, (char)171, (char)7, (char)13, (char)22, (char)83, (char)164, (char)41, (char)53, (char)86, (char)95, (char)40, (char)18, (char)115, (char)113, (char)88, (char)124, (char)48, (char)150, (char)172, (char)156, (char)67, (char)167, (char)249, (char)237, (char)10, (char)176, (char)43, (char)43, (char)227, (char)73, (char)163, (char)229, (char)112, (char)16, (char)114, (char)6, (char)99, (char)1, (char)245, (char)30, (char)82, (char)146, (char)124, (char)231, (char)171, (char)255, (char)43, (char)162, (char)83, (char)24, (char)227, (char)57, (char)184, (char)212, (char)11, (char)77, (char)220, (char)64, (char)19, (char)6, (char)93, (char)226, (char)244, (char)173, (char)127, (char)228, (char)211, (char)29, (char)183, (char)247, (char)67, (char)102, (char)16, (char)117, (char)219, (char)205, (char)60, (char)24, (char)111, (char)1, (char)4, (char)233, (char)88, (char)91, (char)24, (char)153, (char)22, (char)173, (char)76, (char)9, (char)33, (char)6, (char)43, (char)25, (char)160, (char)77, (char)178, (char)136, (char)171, (char)27, (char)59, (char)171, (char)51, (char)198, (char)118, (char)69, (char)46, (char)223, (char)95, (char)245, (char)95, (char)237, (char)203, (char)40, (char)47, (char)30, (char)179, (char)124, (char)173}));
            assert(pack.regstart_GET() == (char)186);
            assert(pack.count_GET() == (char)67);
            assert(pack.request_id_GET() == 888654716L);
            assert(pack.busname_LEN(ph) == 18);
            assert(pack.busname_TRY(ph).equals("iGbkjeogkuwcjjGpcg"));
            assert(pack.target_component_GET() == (char)196);
            assert(pack.bus_GET() == (char)73);
            assert(pack.bustype_GET() == DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C);
            assert(pack.address_GET() == (char)49);
            assert(pack.target_system_GET() == (char)35);
        });
        GroundControl.DEVICE_OP_WRITE p11002 = CommunicationChannel.new_DEVICE_OP_WRITE();
        PH.setPack(p11002);
        p11002.request_id_SET(888654716L) ;
        p11002.busname_SET("iGbkjeogkuwcjjGpcg", PH) ;
        p11002.regstart_SET((char)186) ;
        p11002.target_system_SET((char)35) ;
        p11002.address_SET((char)49) ;
        p11002.target_component_SET((char)196) ;
        p11002.bustype_SET(DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C) ;
        p11002.bus_SET((char)73) ;
        p11002.count_SET((char)67) ;
        p11002.data__SET(new char[] {(char)155, (char)214, (char)133, (char)127, (char)48, (char)240, (char)245, (char)120, (char)48, (char)171, (char)7, (char)13, (char)22, (char)83, (char)164, (char)41, (char)53, (char)86, (char)95, (char)40, (char)18, (char)115, (char)113, (char)88, (char)124, (char)48, (char)150, (char)172, (char)156, (char)67, (char)167, (char)249, (char)237, (char)10, (char)176, (char)43, (char)43, (char)227, (char)73, (char)163, (char)229, (char)112, (char)16, (char)114, (char)6, (char)99, (char)1, (char)245, (char)30, (char)82, (char)146, (char)124, (char)231, (char)171, (char)255, (char)43, (char)162, (char)83, (char)24, (char)227, (char)57, (char)184, (char)212, (char)11, (char)77, (char)220, (char)64, (char)19, (char)6, (char)93, (char)226, (char)244, (char)173, (char)127, (char)228, (char)211, (char)29, (char)183, (char)247, (char)67, (char)102, (char)16, (char)117, (char)219, (char)205, (char)60, (char)24, (char)111, (char)1, (char)4, (char)233, (char)88, (char)91, (char)24, (char)153, (char)22, (char)173, (char)76, (char)9, (char)33, (char)6, (char)43, (char)25, (char)160, (char)77, (char)178, (char)136, (char)171, (char)27, (char)59, (char)171, (char)51, (char)198, (char)118, (char)69, (char)46, (char)223, (char)95, (char)245, (char)95, (char)237, (char)203, (char)40, (char)47, (char)30, (char)179, (char)124, (char)173}, 0) ;
        CommunicationChannel.instance.send(p11002);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEVICE_OP_WRITE_REPLY.add((src, ph, pack) ->
        {
            assert(pack.request_id_GET() == 2584961554L);
            assert(pack.result_GET() == (char)94);
        });
        GroundControl.DEVICE_OP_WRITE_REPLY p11003 = CommunicationChannel.new_DEVICE_OP_WRITE_REPLY();
        PH.setPack(p11003);
        p11003.result_SET((char)94) ;
        p11003.request_id_SET(2584961554L) ;
        CommunicationChannel.instance.send(p11003);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ADAP_TUNING.add((src, ph, pack) ->
        {
            assert(pack.f_GET() == -1.2082132E38F);
            assert(pack.sigma_GET() == -2.7371477E38F);
            assert(pack.achieved_GET() == 9.5412E37F);
            assert(pack.omega_GET() == 1.3936718E38F);
            assert(pack.error_GET() == 1.7295185E37F);
            assert(pack.theta_dot_GET() == -1.0685272E38F);
            assert(pack.theta_GET() == 3.6309702E37F);
            assert(pack.desired_GET() == 2.2427625E38F);
            assert(pack.sigma_dot_GET() == -3.3360694E38F);
            assert(pack.axis_GET() == PID_TUNING_AXIS.PID_TUNING_PITCH);
            assert(pack.f_dot_GET() == -9.721631E37F);
            assert(pack.omega_dot_GET() == -2.301767E37F);
            assert(pack.u_GET() == -2.6900057E38F);
        });
        GroundControl.ADAP_TUNING p11010 = CommunicationChannel.new_ADAP_TUNING();
        PH.setPack(p11010);
        p11010.theta_dot_SET(-1.0685272E38F) ;
        p11010.omega_dot_SET(-2.301767E37F) ;
        p11010.desired_SET(2.2427625E38F) ;
        p11010.axis_SET(PID_TUNING_AXIS.PID_TUNING_PITCH) ;
        p11010.omega_SET(1.3936718E38F) ;
        p11010.error_SET(1.7295185E37F) ;
        p11010.achieved_SET(9.5412E37F) ;
        p11010.f_SET(-1.2082132E38F) ;
        p11010.sigma_SET(-2.7371477E38F) ;
        p11010.f_dot_SET(-9.721631E37F) ;
        p11010.sigma_dot_SET(-3.3360694E38F) ;
        p11010.u_SET(-2.6900057E38F) ;
        p11010.theta_SET(3.6309702E37F) ;
        CommunicationChannel.instance.send(p11010);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VISION_POSITION_DELTA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.position_delta_GET(),  new float[] {1.7136293E38F, 2.3496444E37F, -1.0773692E38F}));
            assert(pack.confidence_GET() == -1.1376656E38F);
            assert(pack.time_usec_GET() == 4996347538154212918L);
            assert(Arrays.equals(pack.angle_delta_GET(),  new float[] {-2.0310115E38F, -1.0322981E38F, -3.3539771E38F}));
            assert(pack.time_delta_usec_GET() == 7886452558374293771L);
        });
        GroundControl.VISION_POSITION_DELTA p11011 = CommunicationChannel.new_VISION_POSITION_DELTA();
        PH.setPack(p11011);
        p11011.angle_delta_SET(new float[] {-2.0310115E38F, -1.0322981E38F, -3.3539771E38F}, 0) ;
        p11011.time_delta_usec_SET(7886452558374293771L) ;
        p11011.position_delta_SET(new float[] {1.7136293E38F, 2.3496444E37F, -1.0773692E38F}, 0) ;
        p11011.time_usec_SET(4996347538154212918L) ;
        p11011.confidence_SET(-1.1376656E38F) ;
        CommunicationChannel.instance.send(p11011);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
    }

}
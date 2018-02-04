
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
                case MAV_CMD.MAV_CMD_AQ_NAV_LEG_ORBIT:
                    id = 0;
                    break;
                case MAV_CMD.MAV_CMD_AQ_TELEMETRY:
                    id = 1;
                    break;
                case MAV_CMD.MAV_CMD_AQ_REQUEST_VERSION:
                    id = 2;
                    break;
                case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                    id = 3;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
                    id = 4;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
                    id = 5;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
                    id = 6;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                    id = 7;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND:
                    id = 8;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                    id = 9;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
                    id = 10;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
                    id = 11;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FOLLOW:
                    id = 12;
                    break;
                case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
                    id = 13;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
                    id = 14;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW:
                    id = 15;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
                    id = 16;
                    break;
                case MAV_CMD.MAV_CMD_NAV_ROI:
                    id = 17;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
                    id = 18;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
                    id = 19;
                    break;
                case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
                    id = 20;
                    break;
                case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
                    id = 21;
                    break;
                case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
                    id = 22;
                    break;
                case MAV_CMD.MAV_CMD_NAV_DELAY:
                    id = 23;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
                    id = 24;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAST:
                    id = 25;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DELAY:
                    id = 26;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
                    id = 27;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
                    id = 28;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_YAW:
                    id = 29;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_LAST:
                    id = 30;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_MODE:
                    id = 31;
                    break;
                case MAV_CMD.MAV_CMD_DO_JUMP:
                    id = 32;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
                    id = 33;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_HOME:
                    id = 34;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
                    id = 35;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_RELAY:
                    id = 36;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
                    id = 37;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_SERVO:
                    id = 38;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
                    id = 39;
                    break;
                case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
                    id = 40;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
                    id = 41;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAND_START:
                    id = 42;
                    break;
                case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
                    id = 43;
                    break;
                case MAV_CMD.MAV_CMD_DO_GO_AROUND:
                    id = 44;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPOSITION:
                    id = 45;
                    break;
                case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
                    id = 46;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
                    id = 47;
                    break;
                case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
                    id = 48;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_ROI:
                    id = 49;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
                    id = 50;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
                    id = 51;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
                    id = 52;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                    id = 53;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                    id = 54;
                    break;
                case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
                    id = 55;
                    break;
                case MAV_CMD.MAV_CMD_DO_PARACHUTE:
                    id = 56;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
                    id = 57;
                    break;
                case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
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
                case MAV_CMD.MAV_CMD_AQ_NAV_LEG_ORBIT:
                    id = 0;
                    break;
                case MAV_CMD.MAV_CMD_AQ_TELEMETRY:
                    id = 1;
                    break;
                case MAV_CMD.MAV_CMD_AQ_REQUEST_VERSION:
                    id = 2;
                    break;
                case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                    id = 3;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
                    id = 4;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
                    id = 5;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
                    id = 6;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                    id = 7;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND:
                    id = 8;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                    id = 9;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
                    id = 10;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
                    id = 11;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FOLLOW:
                    id = 12;
                    break;
                case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
                    id = 13;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
                    id = 14;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW:
                    id = 15;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
                    id = 16;
                    break;
                case MAV_CMD.MAV_CMD_NAV_ROI:
                    id = 17;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
                    id = 18;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
                    id = 19;
                    break;
                case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
                    id = 20;
                    break;
                case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
                    id = 21;
                    break;
                case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
                    id = 22;
                    break;
                case MAV_CMD.MAV_CMD_NAV_DELAY:
                    id = 23;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
                    id = 24;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAST:
                    id = 25;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DELAY:
                    id = 26;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
                    id = 27;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
                    id = 28;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_YAW:
                    id = 29;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_LAST:
                    id = 30;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_MODE:
                    id = 31;
                    break;
                case MAV_CMD.MAV_CMD_DO_JUMP:
                    id = 32;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
                    id = 33;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_HOME:
                    id = 34;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
                    id = 35;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_RELAY:
                    id = 36;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
                    id = 37;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_SERVO:
                    id = 38;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
                    id = 39;
                    break;
                case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
                    id = 40;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
                    id = 41;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAND_START:
                    id = 42;
                    break;
                case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
                    id = 43;
                    break;
                case MAV_CMD.MAV_CMD_DO_GO_AROUND:
                    id = 44;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPOSITION:
                    id = 45;
                    break;
                case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
                    id = 46;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
                    id = 47;
                    break;
                case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
                    id = 48;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_ROI:
                    id = 49;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
                    id = 50;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
                    id = 51;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
                    id = 52;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                    id = 53;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                    id = 54;
                    break;
                case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
                    id = 55;
                    break;
                case MAV_CMD.MAV_CMD_DO_PARACHUTE:
                    id = 56;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
                    id = 57;
                    break;
                case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
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
    public static class SET_ACTUATOR_CONTROL_TARGET extends GroundControl.SET_ACTUATOR_CONTROL_TARGET
    {
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        /**
        *Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
        *	this field to difference between instances*/
        public char group_mlx_GET()
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  9, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  10, 1)); }
        /**
        *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
        *	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
        *	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
        *	mixer to repurpose them as generic outputs*/
        public float[] controls_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 11, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        /**
        *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
        *	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
        *	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
        *	mixer to repurpose them as generic outputs*/
        public float[] controls_GET()
        {return controls_GET(new float[8], 0);}
    }
    public static class ACTUATOR_CONTROL_TARGET extends GroundControl.ACTUATOR_CONTROL_TARGET
    {
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        /**
        *Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
        *	this field to difference between instances*/
        public char group_mlx_GET()
        {  return (char)((char) get_bytes(data,  8, 1)); }
        /**
        *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
        *	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
        *	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
        *	mixer to repurpose them as generic outputs*/
        public float[] controls_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 9, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        /**
        *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
        *	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
        *	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
        *	mixer to repurpose them as generic outputs*/
        public float[] controls_GET()
        {return controls_GET(new float[8], 0);}
    }
    public static class ALTITUDE extends GroundControl.ALTITUDE
    {
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        /**
        *This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the
        *	local altitude change). The only guarantee on this field is that it will never be reset and is consistent
        *	within a flight. The recommended value for this field is the uncorrected barometric altitude at boot
        *	time. This altitude will also drift and vary between flights*/
        public float altitude_monotonic_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        /**
        *This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events
        *	like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints
        *	are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL
        *	by default and not the WGS84 altitude*/
        public float altitude_amsl_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        /**
        *This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
        *	to the coordinate origin (0, 0, 0). It is up-positive*/
        public float altitude_local_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float altitude_relative_GET()//This is the altitude above the home position. It resets on each change of the current home position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        /**
        *This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller
        *	than -1000 should be interpreted as unknown*/
        public float altitude_terrain_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        /**
        *This is not the altitude, but the clear space below the system according to the fused clearance estimate.
        *	It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving
        *	target. A negative value indicates no measurement available*/
        public float bottom_clearance_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
    }
    public static class RESOURCE_REQUEST extends GroundControl.RESOURCE_REQUEST
    {
        public char request_id_GET()//Request ID. This ID should be re-used when sending back URI contents
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char uri_type_GET()//The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
        {  return (char)((char) get_bytes(data,  1, 1)); }
        /**
        *The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
        *	on the URI type enum*/
        public char[] uri_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 2, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
        *	on the URI type enum*/
        public char[] uri_GET()
        {return uri_GET(new char[120], 0);} public char transfer_type_GET()//The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
        {  return (char)((char) get_bytes(data,  122, 1)); }
        /**
        *The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
        *	has a storage associated (e.g. MAVLink FTP)*/
        public char[] storage_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 123, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
        *	has a storage associated (e.g. MAVLink FTP)*/
        public char[] storage_GET()
        {return storage_GET(new char[120], 0);}
    }
    public static class SCALED_PRESSURE3 extends GroundControl.SCALED_PRESSURE3
    {
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public float press_abs_GET()//Absolute pressure (hectopascal)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float press_diff_GET()//Differential pressure 1 (hectopascal)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public short temperature_GET()//Temperature measurement (0.01 degrees celsius)
        {  return (short)((short) get_bytes(data,  12, 2)); }
    }
    public static class FOLLOW_TARGET extends GroundControl.FOLLOW_TARGET
    {
        public long timestamp_GET()//Timestamp in milliseconds since system boot
        {  return (get_bytes(data,  0, 8)); }
        public long custom_state_GET()//button states or switches of a tracker device
        {  return (get_bytes(data,  8, 8)); }
        public char est_capabilities_GET()//bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
        {  return (char)((char) get_bytes(data,  16, 1)); }
        public int lat_GET()//Latitude (WGS84), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  17, 4)); }
        public int lon_GET()//Longitude (WGS84), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  21, 4)); }
        public float alt_GET()//AMSL, in meters
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  25, 4))); }
        public float[] vel_GET(float[]  dst_ch, int pos)  //target velocity (0,0,0) for unknown
        {
            for(int BYTE = 29, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] vel_GET()//target velocity (0,0,0) for unknown
        {return vel_GET(new float[3], 0);} public float[] acc_GET(float[]  dst_ch, int pos)  //linear target acceleration (0,0,0) for unknown
        {
            for(int BYTE = 41, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] acc_GET()//linear target acceleration (0,0,0) for unknown
        {return acc_GET(new float[3], 0);} public float[] attitude_q_GET(float[]  dst_ch, int pos)  //(1 0 0 0 for unknown)
        {
            for(int BYTE = 53, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] attitude_q_GET()//(1 0 0 0 for unknown)
        {return attitude_q_GET(new float[4], 0);} public float[] rates_GET(float[]  dst_ch, int pos)  //(0 0 0 for unknown)
        {
            for(int BYTE = 69, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] rates_GET()//(0 0 0 for unknown)
        {return rates_GET(new float[3], 0);} public float[] position_cov_GET(float[]  dst_ch, int pos)  //eph epv
        {
            for(int BYTE = 81, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] position_cov_GET()//eph epv
        {return position_cov_GET(new float[3], 0);}
    }
    public static class CONTROL_SYSTEM_STATE extends GroundControl.CONTROL_SYSTEM_STATE
    {
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        public float x_acc_GET()//X acceleration in body frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float y_acc_GET()//Y acceleration in body frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float z_acc_GET()//Z acceleration in body frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float x_vel_GET()//X velocity in body frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float y_vel_GET()//Y velocity in body frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float z_vel_GET()//Z velocity in body frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public float x_pos_GET()//X position in local frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public float y_pos_GET()//Y position in local frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public float z_pos_GET()//Z position in local frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public float airspeed_GET()//Airspeed, set to -1 if unknown
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        public float[] vel_variance_GET(float[]  dst_ch, int pos)  //Variance of body velocity estimate
        {
            for(int BYTE = 48, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] vel_variance_GET()//Variance of body velocity estimate
        {return vel_variance_GET(new float[3], 0);} public float[] pos_variance_GET(float[]  dst_ch, int pos)  //Variance in local position
        {
            for(int BYTE = 60, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] pos_variance_GET()//Variance in local position
        {return pos_variance_GET(new float[3], 0);} public float[] q_GET(float[]  dst_ch, int pos)  //The attitude, represented as Quaternion
        {
            for(int BYTE = 72, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] q_GET()//The attitude, represented as Quaternion
        {return q_GET(new float[4], 0);} public float roll_rate_GET()//Angular rate in roll axis
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  88, 4))); }
        public float pitch_rate_GET()//Angular rate in pitch axis
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  92, 4))); }
        public float yaw_rate_GET()//Angular rate in yaw axis
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  96, 4))); }
    }
    public static class BATTERY_STATUS extends GroundControl.BATTERY_STATUS
    {
        /**
        *Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
        *	should have the UINT16_MAX value*/
        public char[] voltages_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 0, dst_max = pos + 10; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        /**
        *Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
        *	should have the UINT16_MAX value*/
        public char[] voltages_GET()
        {return voltages_GET(new char[10], 0);} public char id_GET()//Battery ID
        {  return (char)((char) get_bytes(data,  20, 1)); }
        public short temperature_GET()//Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
        {  return (short)((short) get_bytes(data,  21, 2)); }
        public short current_battery_GET()//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
        {  return (short)((short) get_bytes(data,  23, 2)); }
        public int current_consumed_GET()//Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimat
        {  return (int)((int) get_bytes(data,  25, 4)); }
        /**
        *Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide
        *	energy consumption estimat*/
        public int energy_consumed_GET()
        {  return (int)((int) get_bytes(data,  29, 4)); }
        public byte battery_remaining_GET()//Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
        {  return (byte)((byte) get_bytes(data,  33, 1)); }
        public @MAV_BATTERY_FUNCTION int battery_function_GET()//Function of the battery
        {  return  0 + (int)get_bits(data, 272, 3); }
        public @MAV_BATTERY_TYPE int type_GET()//Type (chemistry) of the battery
        {  return  0 + (int)get_bits(data, 275, 3); }
    }
    public static class AUTOPILOT_VERSION extends GroundControl.AUTOPILOT_VERSION
    {
        public char vendor_id_GET()//ID of the board vendor
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char product_id_GET()//ID of the product
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public long flight_sw_version_GET()//Firmware version number
        {  return (get_bytes(data,  4, 4)); }
        public long middleware_sw_version_GET()//Middleware version number
        {  return (get_bytes(data,  8, 4)); }
        public long os_sw_version_GET()//Operating system version number
        {  return (get_bytes(data,  12, 4)); }
        public long board_version_GET()//HW / board version (last 8 bytes should be silicon ID, if any)
        {  return (get_bytes(data,  16, 4)); }
        public long uid_GET()//UID if provided by hardware (see uid2)
        {  return (get_bytes(data,  20, 8)); }
        /**
        *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
        *	should allow to identify the commit using the main version number even for very large code bases*/
        public char[] flight_custom_version_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 28, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
        *	should allow to identify the commit using the main version number even for very large code bases*/
        public char[] flight_custom_version_GET()
        {return flight_custom_version_GET(new char[8], 0);}/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	should allow to identify the commit using the main version number even for very large code bases*/
        public char[] middleware_custom_version_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 36, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
        *	should allow to identify the commit using the main version number even for very large code bases*/
        public char[] middleware_custom_version_GET()
        {return middleware_custom_version_GET(new char[8], 0);}/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	should allow to identify the commit using the main version number even for very large code bases*/
        public char[] os_custom_version_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 44, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
        *	should allow to identify the commit using the main version number even for very large code bases*/
        public char[] os_custom_version_GET()
        {return os_custom_version_GET(new char[8], 0);} public @MAV_PROTOCOL_CAPABILITY int capabilities_GET()//bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
        {
            switch((int)get_bits(data, 416, 5))
            {
                case 0:
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT;
                case 1:
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT;
                case 2:
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT;
                case 3:
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT;
                case 4:
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION;
                case 5:
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP;
                case 6:
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;
                case 7:
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED;
                case 8:
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT;
                case 9:
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN;
                case 10:
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET;
                case 11:
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION;
                case 12:
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION;
                case 13:
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2;
                case 14:
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE;
                case 15:
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY;
                case 16:
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION;
            }
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        /**
        *UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
        *	use uid*/
        public char[]  uid2_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  421 && !try_visit_field(ph, 421)) return null;
            return uid2_GET(ph, new char[ph.items], 0);
        }
        /**
        *UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
        *	use uid*/
        public char[] uid2_GET(Bounds.Inside ph, char[]  dst_ch, int pos)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + 18; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public int uid2_LEN()
        {
            return 18;
        }
    }
    public static class LANDING_TARGET extends GroundControl.LANDING_TARGET
    {
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        public char target_num_GET()//The ID of the target if multiple targets are present
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public float angle_x_GET()//X-axis angular offset (in radians) of the target from the center of the image
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  9, 4))); }
        public float angle_y_GET()//Y-axis angular offset (in radians) of the target from the center of the image
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  13, 4))); }
        public float distance_GET()//Distance to the target from the vehicle in meters
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  17, 4))); }
        public float size_x_GET()//Size in radians of target along x-axis
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  21, 4))); }
        public float size_y_GET()//Size in radians of target along y-axis
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  25, 4))); }
        public @MAV_FRAME int frame_GET()//MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
        {  return  0 + (int)get_bits(data, 232, 4); }
        public @LANDING_TARGET_TYPE int type_GET()//LANDING_TARGET_TYPE enum specifying the type of landing target
        {  return  0 + (int)get_bits(data, 236, 3); }
        public float  x_TRY(Bounds.Inside ph)//X Position of the landing target on MAV_FRAME
        {
            if(ph.field_bit !=  239 && !try_visit_field(ph, 239)) return 0;
            return (float)(Float.intBitsToFloat((int) get_bytes(data,  ph.BYTE, 4)));
        }
        public float  y_TRY(Bounds.Inside ph)//Y Position of the landing target on MAV_FRAME
        {
            if(ph.field_bit !=  240 && !try_visit_field(ph, 240)) return 0;
            return (float)(Float.intBitsToFloat((int) get_bytes(data,  ph.BYTE, 4)));
        }
        public float  z_TRY(Bounds.Inside ph)//Z Position of the landing target on MAV_FRAME
        {
            if(ph.field_bit !=  241 && !try_visit_field(ph, 241)) return 0;
            return (float)(Float.intBitsToFloat((int) get_bytes(data,  ph.BYTE, 4)));
        }
        public float[]  q_TRY(Bounds.Inside ph)//Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        {
            if(ph.field_bit !=  242 && !try_visit_field(ph, 242)) return null;
            return q_GET(ph, new float[ph.items], 0);
        }
        public float[] q_GET(Bounds.Inside ph, float[]  dst_ch, int pos) //Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public int q_LEN()
        {
            return 4;
        }/**
*Boolean indicating known position (1) or default unkown position (0), for validation of positioning of
*	the landing targe*/
        public char  position_valid_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  243 && !try_visit_field(ph, 243)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 1));
        }
    }
    public static class AQ_TELEMETRY_F extends GroundControl.AQ_TELEMETRY_F
    {
        public char Index_GET()//Index of message
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public float value1_GET()//value1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  2, 4))); }
        public float value2_GET()//value2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  6, 4))); }
        public float value3_GET()//value3
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
        public float value4_GET()//value4
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public float value5_GET()//value5
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public float value6_GET()//value6
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        public float value7_GET()//value7
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  26, 4))); }
        public float value8_GET()//value8
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  30, 4))); }
        public float value9_GET()//value9
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  34, 4))); }
        public float value10_GET()//value10
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  38, 4))); }
        public float value11_GET()//value11
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  42, 4))); }
        public float value12_GET()//value12
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  46, 4))); }
        public float value13_GET()//value13
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  50, 4))); }
        public float value14_GET()//value14
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  54, 4))); }
        public float value15_GET()//value15
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  58, 4))); }
        public float value16_GET()//value16
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  62, 4))); }
        public float value17_GET()//value17
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  66, 4))); }
        public float value18_GET()//value18
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  70, 4))); }
        public float value19_GET()//value19
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  74, 4))); }
        public float value20_GET()//value20
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  78, 4))); }
    }
    public static class AQ_ESC_TELEMETRY extends GroundControl.AQ_ESC_TELEMETRY
    {
        public char[] status_age_GET(char[]  dst_ch, int pos)  //Age of each ESC telemetry reading in ms compared to boot time. A value of 0xFFFF means timeout/no data
        {
            for(int BYTE = 0, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] status_age_GET()//Age of each ESC telemetry reading in ms compared to boot time. A value of 0xFFFF means timeout/no data
        {return status_age_GET(new char[4], 0);} public long time_boot_ms_GET()//Timestamp of the component clock since boot time in ms.
        {  return (get_bytes(data,  8, 4)); }
        public long[] data0_GET(long[]  dst_ch, int pos)  //Data bits 1-32 for each ESC.
        {
            for(int BYTE = 12, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (get_bytes(data,  BYTE, 4));
            return dst_ch;
        }
        public long[] data0_GET()//Data bits 1-32 for each ESC.
        {return data0_GET(new long[4], 0);} public long[] data1_GET(long[]  dst_ch, int pos)  //Data bits 33-64 for each ESC.
        {
            for(int BYTE = 28, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (get_bytes(data,  BYTE, 4));
            return dst_ch;
        }
        public long[] data1_GET()//Data bits 33-64 for each ESC.
        {return data1_GET(new long[4], 0);} public char seq_GET()//Sequence number of message (first set of 4 motors is #1, next 4 is #2, etc).
        {  return (char)((char) get_bytes(data,  44, 1)); }
        public char num_motors_GET()//Total number of active ESCs/motors on the system.
        {  return (char)((char) get_bytes(data,  45, 1)); }
        public char num_in_seq_GET()//Number of active ESCs in this sequence (1 through this many array members will be populated with data
        {  return (char)((char) get_bytes(data,  46, 1)); }
        public char[] escid_GET(char[]  dst_ch, int pos)  //ESC/Motor ID
        {
            for(int BYTE = 47, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] escid_GET()//ESC/Motor ID
        {return escid_GET(new char[4], 0);} public char[] data_version_GET(char[]  dst_ch, int pos)  //Version of data structure (determines contents).
        {
            for(int BYTE = 51, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data_version_GET()//Version of data structure (determines contents).
        {return data_version_GET(new char[4], 0);}
    }
    public static class ESTIMATOR_STATUS extends GroundControl.ESTIMATOR_STATUS
    {
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        public float vel_ratio_GET()//Velocity innovation test ratio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float pos_horiz_ratio_GET()//Horizontal position innovation test ratio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float pos_vert_ratio_GET()//Vertical position innovation test ratio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float mag_ratio_GET()//Magnetometer innovation test ratio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float hagl_ratio_GET()//Height above terrain innovation test ratio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float tas_ratio_GET()//True airspeed innovation test ratio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public float pos_horiz_accuracy_GET()//Horizontal position 1-STD accuracy relative to the EKF local origin (m)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public float pos_vert_accuracy_GET()//Vertical position 1-STD accuracy relative to the EKF local origin (m)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public @ESTIMATOR_STATUS_FLAGS int flags_GET()//Integer bitmask indicating which EKF outputs are valid. See definition for ESTIMATOR_STATUS_FLAGS.
        {
            switch((int)get_bits(data, 320, 4))
            {
                case 0:
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE;
                case 1:
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ;
                case 2:
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT;
                case 3:
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL;
                case 4:
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS;
                case 5:
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS;
                case 6:
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL;
                case 7:
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE;
                case 8:
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL;
                case 9:
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS;
                case 10:
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH;
            }
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
    }
    public static class WIND_COV extends GroundControl.WIND_COV
    {
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        public float wind_x_GET()//Wind in X (NED) direction in m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float wind_y_GET()//Wind in Y (NED) direction in m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float wind_z_GET()//Wind in Z (NED) direction in m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float var_horiz_GET()//Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float var_vert_GET()//Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float wind_alt_GET()//AMSL altitude (m) this measurement was taken at
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public float horiz_accuracy_GET()//Horizontal speed 1-STD accuracy
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public float vert_accuracy_GET()//Vertical speed 1-STD accuracy
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
    }
    public static class GPS_INPUT extends GroundControl.GPS_INPUT
    {
        public char time_week_GET()//GPS week number
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public long time_week_ms_GET()//GPS time (milliseconds from start of GPS week)
        {  return (get_bytes(data,  2, 4)); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  6, 8)); }
        public char gps_id_GET()//ID of the GPS for multiple GPS inputs
        {  return (char)((char) get_bytes(data,  14, 1)); }
        public char fix_type_GET()//0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
        {  return (char)((char) get_bytes(data,  15, 1)); }
        public int lat_GET()//Latitude (WGS84), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  16, 4)); }
        public int lon_GET()//Longitude (WGS84), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  20, 4)); }
        public float alt_GET()//Altitude (AMSL, not WGS84), in m (positive for up)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float hdop_GET()//GPS HDOP horizontal dilution of position in m
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public float vdop_GET()//GPS VDOP vertical dilution of position in m
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public float vn_GET()//GPS velocity in m/s in NORTH direction in earth-fixed NED frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public float ve_GET()//GPS velocity in m/s in EAST direction in earth-fixed NED frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public float vd_GET()//GPS velocity in m/s in DOWN direction in earth-fixed NED frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        public float speed_accuracy_GET()//GPS speed accuracy in m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  48, 4))); }
        public float horiz_accuracy_GET()//GPS horizontal accuracy in m
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  52, 4))); }
        public float vert_accuracy_GET()//GPS vertical accuracy in m
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  56, 4))); }
        public char satellites_visible_GET()//Number of satellites visible.
        {  return (char)((char) get_bytes(data,  60, 1)); }
        public @GPS_INPUT_IGNORE_FLAGS int ignore_flags_GET()//Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided
        {
            switch((int)get_bits(data, 488, 4))
            {
                case 0:
                    return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT;
                case 1:
                    return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP;
                case 2:
                    return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP;
                case 3:
                    return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ;
                case 4:
                    return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT;
                case 5:
                    return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY;
                case 6:
                    return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY;
                case 7:
                    return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY;
            }
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
    }
    public static class GPS_RTCM_DATA extends GroundControl.GPS_RTCM_DATA
    {
        /**
        *LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for
        *	the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed
        *	on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer,
        *	while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered
        *	fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment
        *	with a non full payload is received. This management is used to ensure that normal GPS operation doesn't
        *	corrupt RTCM data, and to recover from a unreliable transport delivery order*/
        public char flags_GET()
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char len_GET()//data length
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char[] data__GET(char[]  dst_ch, int pos)  //RTCM message (may be fragmented)
        {
            for(int BYTE = 2, dst_max = pos + 180; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//RTCM message (may be fragmented)
        {return data__GET(new char[180], 0);}
    }
    public static class HIGH_LATENCY extends GroundControl.HIGH_LATENCY
    {
        public char heading_GET()//heading (centidegrees)
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char wp_distance_GET()//distance to target (meters)
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public long custom_mode_GET()//A bitfield for use for autopilot-specific flags.
        {  return (get_bytes(data,  4, 4)); }
        public short roll_GET()//roll (centidegrees)
        {  return (short)((short) get_bytes(data,  8, 2)); }
        public short pitch_GET()//pitch (centidegrees)
        {  return (short)((short) get_bytes(data,  10, 2)); }
        public byte throttle_GET()//throttle (percentage)
        {  return (byte)((byte) get_bytes(data,  12, 1)); }
        public short heading_sp_GET()//heading setpoint (centidegrees)
        {  return (short)((short) get_bytes(data,  13, 2)); }
        public int latitude_GET()//Latitude, expressed as degrees * 1E7
        {  return (int)((int) get_bytes(data,  15, 4)); }
        public int longitude_GET()//Longitude, expressed as degrees * 1E7
        {  return (int)((int) get_bytes(data,  19, 4)); }
        public short altitude_amsl_GET()//Altitude above mean sea level (meters)
        {  return (short)((short) get_bytes(data,  23, 2)); }
        public short altitude_sp_GET()//Altitude setpoint relative to the home position (meters)
        {  return (short)((short) get_bytes(data,  25, 2)); }
        public char airspeed_GET()//airspeed (m/s)
        {  return (char)((char) get_bytes(data,  27, 1)); }
        public char airspeed_sp_GET()//airspeed setpoint (m/s)
        {  return (char)((char) get_bytes(data,  28, 1)); }
        public char groundspeed_GET()//groundspeed (m/s)
        {  return (char)((char) get_bytes(data,  29, 1)); }
        public byte climb_rate_GET()//climb rate (m/s)
        {  return (byte)((byte) get_bytes(data,  30, 1)); }
        public char gps_nsat_GET()//Number of satellites visible. If unknown, set to 255
        {  return (char)((char) get_bytes(data,  31, 1)); }
        public char battery_remaining_GET()//Remaining battery (percentage)
        {  return (char)((char) get_bytes(data,  32, 1)); }
        public byte temperature_GET()//Autopilot temperature (degrees C)
        {  return (byte)((byte) get_bytes(data,  33, 1)); }
        public byte temperature_air_GET()//Air temperature (degrees C) from airspeed sensor
        {  return (byte)((byte) get_bytes(data,  34, 1)); }
        /**
        *failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS,
        *	bit3:GCS, bit4:fence*/
        public char failsafe_GET()
        {  return (char)((char) get_bytes(data,  35, 1)); }
        public char wp_num_GET()//current waypoint number
        {  return (char)((char) get_bytes(data,  36, 1)); }
        public @MAV_MODE_FLAG int base_mode_GET()//System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
        {
            switch((int)get_bits(data, 296, 4))
            {
                case 0:
                    return MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
                case 1:
                    return MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED;
                case 2:
                    return MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED;
                case 3:
                    return MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
                case 4:
                    return MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED;
                case 5:
                    return MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED;
                case 6:
                    return MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
                case 7:
                    return MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED;
            }
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public @MAV_LANDED_STATE int landed_state_GET()//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
        {  return  0 + (int)get_bits(data, 300, 3); }
        public @GPS_FIX_TYPE int gps_fix_type_GET()//See the GPS_FIX_TYPE enum.
        {  return  0 + (int)get_bits(data, 303, 4); }
    }
    public static class VIBRATION extends GroundControl.VIBRATION
    {
        public long clipping_0_GET()//first accelerometer clipping count
        {  return (get_bytes(data,  0, 4)); }
        public long clipping_1_GET()//second accelerometer clipping count
        {  return (get_bytes(data,  4, 4)); }
        public long clipping_2_GET()//third accelerometer clipping count
        {  return (get_bytes(data,  8, 4)); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  12, 8)); }
        public float vibration_x_GET()//Vibration levels on X-axis
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float vibration_y_GET()//Vibration levels on Y-axis
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float vibration_z_GET()//Vibration levels on Z-axis
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
    }
    public static class HOME_POSITION extends GroundControl.HOME_POSITION
    {
        public int latitude_GET()//Latitude (WGS84), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  0, 4)); }
        public int longitude_GET()//Longitude (WGS84, in degrees * 1E7
        {  return (int)((int) get_bytes(data,  4, 4)); }
        public int altitude_GET()//Altitude (AMSL), in meters * 1000 (positive for up)
        {  return (int)((int) get_bytes(data,  8, 4)); }
        public float x_GET()//Local X position of this position in the local coordinate frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float y_GET()//Local Y position of this position in the local coordinate frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float z_GET()//Local Z position of this position in the local coordinate frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        /**
        *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
        *	and slope of the groun*/
        public float[] q_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 24, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        /**
        *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
        *	and slope of the groun*/
        public float[] q_GET()
        {return q_GET(new float[4], 0);}/**
*Local X position of the end of the approach vector. Multicopters should set this position based on their
*	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	from the threshold / touchdown zone*/
        public float approach_x_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        /**
        *Local Y position of the end of the approach vector. Multicopters should set this position based on their
        *	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	from the threshold / touchdown zone*/
        public float approach_y_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        /**
        *Local Z position of the end of the approach vector. Multicopters should set this position based on their
        *	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	from the threshold / touchdown zone*/
        public float approach_z_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  48, 4))); }
        public long  time_usec_TRY(Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {
            if(ph.field_bit !=  416 && !try_visit_field(ph, 416)) return 0;
            return (get_bytes(data,  ph.BYTE, 8));
        }
    }
    public static class SET_HOME_POSITION extends GroundControl.SET_HOME_POSITION
    {
        public char target_system_GET()//System ID.
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public int latitude_GET()//Latitude (WGS84), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  1, 4)); }
        public int longitude_GET()//Longitude (WGS84, in degrees * 1E7
        {  return (int)((int) get_bytes(data,  5, 4)); }
        public int altitude_GET()//Altitude (AMSL), in meters * 1000 (positive for up)
        {  return (int)((int) get_bytes(data,  9, 4)); }
        public float x_GET()//Local X position of this position in the local coordinate frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  13, 4))); }
        public float y_GET()//Local Y position of this position in the local coordinate frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  17, 4))); }
        public float z_GET()//Local Z position of this position in the local coordinate frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  21, 4))); }
        /**
        *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
        *	and slope of the groun*/
        public float[] q_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 25, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        /**
        *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
        *	and slope of the groun*/
        public float[] q_GET()
        {return q_GET(new float[4], 0);}/**
*Local X position of the end of the approach vector. Multicopters should set this position based on their
*	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	from the threshold / touchdown zone*/
        public float approach_x_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  41, 4))); }
        /**
        *Local Y position of the end of the approach vector. Multicopters should set this position based on their
        *	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	from the threshold / touchdown zone*/
        public float approach_y_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  45, 4))); }
        /**
        *Local Z position of the end of the approach vector. Multicopters should set this position based on their
        *	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	from the threshold / touchdown zone*/
        public float approach_z_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  49, 4))); }
        public long  time_usec_TRY(Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {
            if(ph.field_bit !=  424 && !try_visit_field(ph, 424)) return 0;
            return (get_bytes(data,  ph.BYTE, 8));
        }
    }
    public static class MESSAGE_INTERVAL extends GroundControl.MESSAGE_INTERVAL
    {
        public char message_id_GET()//The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public int interval_us_GET()//0 indicates the interval at which it is sent.
        {  return (int)((int) get_bytes(data,  2, 4)); }
    }
    public static class EXTENDED_SYS_STATE extends GroundControl.EXTENDED_SYS_STATE
    {
        public @MAV_VTOL_STATE int vtol_state_GET()//The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration
        {  return  0 + (int)get_bits(data, 0, 3); }
        public @MAV_LANDED_STATE int landed_state_GET()//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
        {  return  0 + (int)get_bits(data, 3, 3); }
    }
    public static class ADSB_VEHICLE extends GroundControl.ADSB_VEHICLE
    {
        public char heading_GET()//Course over ground in centidegrees
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char hor_velocity_GET()//The horizontal velocity in centimeters/second
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public char squawk_GET()//Squawk code
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public long ICAO_address_GET()//ICAO address
        {  return (get_bytes(data,  6, 4)); }
        public int lat_GET()//Latitude, expressed as degrees * 1E7
        {  return (int)((int) get_bytes(data,  10, 4)); }
        public int lon_GET()//Longitude, expressed as degrees * 1E7
        {  return (int)((int) get_bytes(data,  14, 4)); }
        public int altitude_GET()//Altitude(ASL) in millimeters
        {  return (int)((int) get_bytes(data,  18, 4)); }
        public short ver_velocity_GET()//The vertical velocity in centimeters/second, positive is up
        {  return (short)((short) get_bytes(data,  22, 2)); }
        public char tslc_GET()//Time since last communication in seconds
        {  return (char)((char) get_bytes(data,  24, 1)); }
        public @ADSB_ALTITUDE_TYPE int altitude_type_GET()//Type from ADSB_ALTITUDE_TYPE enum
        {  return  0 + (int)get_bits(data, 200, 2); }
        public @ADSB_EMITTER_TYPE int emitter_type_GET()//Type from ADSB_EMITTER_TYPE enum
        {  return  0 + (int)get_bits(data, 202, 5); }
        public @ADSB_FLAGS int flags_GET()//Flags to indicate various statuses including valid data fields
        {
            switch((int)get_bits(data, 207, 3))
            {
                case 0:
                    return ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS;
                case 1:
                    return ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE;
                case 2:
                    return ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING;
                case 3:
                    return ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY;
                case 4:
                    return ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN;
                case 5:
                    return ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK;
                case 6:
                    return ADSB_FLAGS.ADSB_FLAGS_SIMULATED;
            }
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public String callsign_TRY(Bounds.Inside ph)//The callsign, 8+null
        {
            if(ph.field_bit !=  210 && !try_visit_field(ph, 210)  ||  !try_visit_item(ph, 0)) return null;
            return new String(callsign_GET(ph, new char[ph.items], 0));
        }
        public char[] callsign_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //The callsign, 8+null
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int callsign_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  210 && !try_visit_field(ph, 210)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class COLLISION extends GroundControl.COLLISION
    {
        public long id_GET()//Unique identifier, domain based on src field
        {  return (get_bytes(data,  0, 4)); }
        public float time_to_minimum_delta_GET()//Estimated time until collision occurs (seconds)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float altitude_minimum_delta_GET()//Closest vertical distance in meters between vehicle and object
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float horizontal_minimum_delta_GET()//Closest horizontal distance in meteres between vehicle and object
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public @MAV_COLLISION_SRC int src__GET()//Collision data source
        {  return  0 + (int)get_bits(data, 128, 2); }
        public @MAV_COLLISION_ACTION int action_GET()//Action that is being taken to avoid this collision
        {  return  0 + (int)get_bits(data, 130, 3); }
        public @MAV_COLLISION_THREAT_LEVEL int threat_level_GET()//How concerned the aircraft is about this collision
        {  return  0 + (int)get_bits(data, 133, 2); }
    }
    public static class V2_EXTENSION extends GroundControl.V2_EXTENSION
    {
        /**
        *A code that identifies the software component that understands this message (analogous to usb device classes
        *	or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension
        *	and the corresponding entry should be added to https:github.com/mavlink/mavlink/extension-message-ids.xml.
        *	 Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...).
        *	Message_types greater than 32767 are considered local experiments and should not be checked in to any
        *	widely distributed codebase*/
        public char message_type_GET()
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char target_network_GET()//Network ID (0 for broadcast)
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char target_system_GET()//System ID (0 for broadcast)
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public char target_component_GET()//Component ID (0 for broadcast)
        {  return (char)((char) get_bytes(data,  4, 1)); }
        /**
        *Variable length payload. The length is defined by the remaining message length when subtracting the header
        *	and other fields.  The entire content of this block is opaque unless you understand any the encoding
        *	message_type.  The particular encoding used can be extension specific and might not always be documented
        *	as part of the mavlink specification*/
        public char[] payload_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 5, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *Variable length payload. The length is defined by the remaining message length when subtracting the header
        *	and other fields.  The entire content of this block is opaque unless you understand any the encoding
        *	message_type.  The particular encoding used can be extension specific and might not always be documented
        *	as part of the mavlink specification*/
        public char[] payload_GET()
        {return payload_GET(new char[249], 0);}
    }
    public static class MEMORY_VECT extends GroundControl.MEMORY_VECT
    {
        public char address_GET()//Starting address of the debug variables
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char ver_GET()//Version code of the type variable. 0=unknown, type ignored and assumed short. 1=as below
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char type_GET()//Type code of the memory variables. for ver = 1: 0=16 x short, 1=16 x char, 2=16 x Q15, 3=16 x 1Q1
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public byte[] value_GET(byte[]  dst_ch, int pos)  //Memory contents at specified address
        {
            for(int BYTE = 4, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (byte)((byte) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public byte[] value_GET()//Memory contents at specified address
        {return value_GET(new byte[32], 0);}
    }
    public static class DEBUG_VECT extends GroundControl.DEBUG_VECT
    {
        public long time_usec_GET()//Timestamp
        {  return (get_bytes(data,  0, 8)); }
        public float x_GET()//x
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float y_GET()//y
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float z_GET()//z
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public String name_TRY(Bounds.Inside ph)//Name
        {
            if(ph.field_bit !=  160 && !try_visit_field(ph, 160)  ||  !try_visit_item(ph, 0)) return null;
            return new String(name_GET(ph, new char[ph.items], 0));
        }
        public char[] name_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Name
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int name_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  160 && !try_visit_field(ph, 160)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class NAMED_VALUE_FLOAT extends GroundControl.NAMED_VALUE_FLOAT
    {
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public float value_GET()//Floating point value
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public String name_TRY(Bounds.Inside ph)//Name of the debug variable
        {
            if(ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) return null;
            return new String(name_GET(ph, new char[ph.items], 0));
        }
        public char[] name_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Name of the debug variable
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int name_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class NAMED_VALUE_INT extends GroundControl.NAMED_VALUE_INT
    {
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public int value_GET()//Signed integer value
        {  return (int)((int) get_bytes(data,  4, 4)); }
        public String name_TRY(Bounds.Inside ph)//Name of the debug variable
        {
            if(ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) return null;
            return new String(name_GET(ph, new char[ph.items], 0));
        }
        public char[] name_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Name of the debug variable
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int name_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class STATUSTEXT extends GroundControl.STATUSTEXT
    {
        public @MAV_SEVERITY int severity_GET()//Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
        {  return  0 + (int)get_bits(data, 0, 4); }
        public String text_TRY(Bounds.Inside ph)//Status text message, without null termination character
        {
            if(ph.field_bit !=  4 && !try_visit_field(ph, 4)  ||  !try_visit_item(ph, 0)) return null;
            return new String(text_GET(ph, new char[ph.items], 0));
        }
        public char[] text_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Status text message, without null termination character
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int text_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  4 && !try_visit_field(ph, 4)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class DEBUG extends GroundControl.DEBUG
    {
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public char ind_GET()//index of debug variable
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public float value_GET()//DEBUG value
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  5, 4))); }
    }
    public static class SETUP_SIGNING extends GroundControl.SETUP_SIGNING
    {
        public long initial_timestamp_GET()//initial timestamp
        {  return (get_bytes(data,  0, 8)); }
        public char target_system_GET()//system id of the target
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public char target_component_GET()//component ID of the target
        {  return (char)((char) get_bytes(data,  9, 1)); }
        public char[] secret_key_GET(char[]  dst_ch, int pos)  //signing key
        {
            for(int BYTE = 10, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] secret_key_GET()//signing key
        {return secret_key_GET(new char[32], 0);}
    }
    public static class BUTTON_CHANGE extends GroundControl.BUTTON_CHANGE
    {
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public long last_change_ms_GET()//Time of last change of button state
        {  return (get_bytes(data,  4, 4)); }
        public char state_GET()//Bitmap state of buttons
        {  return (char)((char) get_bytes(data,  8, 1)); }
    }
    public static class PLAY_TUNE extends GroundControl.PLAY_TUNE
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public String tune_TRY(Bounds.Inside ph)//tune in board specific format
        {
            if(ph.field_bit !=  16 && !try_visit_field(ph, 16)  ||  !try_visit_item(ph, 0)) return null;
            return new String(tune_GET(ph, new char[ph.items], 0));
        }
        public char[] tune_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //tune in board specific format
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int tune_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  16 && !try_visit_field(ph, 16)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class CAMERA_INFORMATION extends GroundControl.CAMERA_INFORMATION
    {
        public char resolution_h_GET()//Image resolution in pixels horizontal
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char resolution_v_GET()//Image resolution in pixels vertical
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public char cam_definition_version_GET()//Camera definition version (iteration)
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  6, 4)); }
        public long firmware_version_GET()//0xff = Major)
        {  return (get_bytes(data,  10, 4)); }
        public char[] vendor_name_GET(char[]  dst_ch, int pos)  //Name of the camera vendor
        {
            for(int BYTE = 14, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] vendor_name_GET()//Name of the camera vendor
        {return vendor_name_GET(new char[32], 0);} public char[] model_name_GET(char[]  dst_ch, int pos)  //Name of the camera model
        {
            for(int BYTE = 46, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] model_name_GET()//Name of the camera model
        {return model_name_GET(new char[32], 0);} public float focal_length_GET()//Focal length in mm
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  78, 4))); }
        public float sensor_size_h_GET()//Image sensor size horizontal in mm
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  82, 4))); }
        public float sensor_size_v_GET()//Image sensor size vertical in mm
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  86, 4))); }
        public char lens_id_GET()//Reserved for a lens ID
        {  return (char)((char) get_bytes(data,  90, 1)); }
        public @CAMERA_CAP_FLAGS int flags_GET()//CAMERA_CAP_FLAGS enum flags (bitmap) describing camera capabilities.
        {
            switch((int)get_bits(data, 728, 3))
            {
                case 0:
                    return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO;
                case 1:
                    return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE;
                case 2:
                    return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES;
                case 3:
                    return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE;
                case 4:
                    return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE;
                case 5:
                    return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE;
            }
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public String cam_definition_uri_TRY(Bounds.Inside ph)//Camera definition URI (if any, otherwise only basic functions will be available).
        {
            if(ph.field_bit !=  731 && !try_visit_field(ph, 731)  ||  !try_visit_item(ph, 0)) return null;
            return new String(cam_definition_uri_GET(ph, new char[ph.items], 0));
        }
        public char[] cam_definition_uri_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Camera definition URI (if any, otherwise only basic functions will be available).
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int cam_definition_uri_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  731 && !try_visit_field(ph, 731)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class CAMERA_SETTINGS extends GroundControl.CAMERA_SETTINGS
    {
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public @CAMERA_MODE int mode_id_GET()//Camera mode (CAMERA_MODE)
        {  return  0 + (int)get_bits(data, 32, 2); }
    }
    public static class STORAGE_INFORMATION extends GroundControl.STORAGE_INFORMATION
    {
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public char storage_id_GET()//Storage ID (1 for first, 2 for second, etc.)
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public char storage_count_GET()//Number of storage devices
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public char status_GET()//Status of storage (0 not available, 1 unformatted, 2 formatted)
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public float total_capacity_GET()//Total capacity in MiB
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  7, 4))); }
        public float used_capacity_GET()//Used capacity in MiB
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  11, 4))); }
        public float available_capacity_GET()//Available capacity in MiB
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  15, 4))); }
        public float read_speed_GET()//Read speed in MiB/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  19, 4))); }
        public float write_speed_GET()//Write speed in MiB/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  23, 4))); }
    }
    public static class CAMERA_CAPTURE_STATUS extends GroundControl.CAMERA_CAPTURE_STATUS
    {
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public long recording_time_ms_GET()//Time in milliseconds since recording started
        {  return (get_bytes(data,  4, 4)); }
        /**
        *Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval
        *	set and capture in progress*/
        public char image_status_GET()
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public char video_status_GET()//Current status of video capturing (0: idle, 1: capture in progress)
        {  return (char)((char) get_bytes(data,  9, 1)); }
        public float image_interval_GET()//Image capture interval in seconds
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
        public float available_capacity_GET()//Available storage capacity in MiB
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
    }
    public static class CAMERA_IMAGE_CAPTURED extends GroundControl.CAMERA_IMAGE_CAPTURED
    {
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public long time_utc_GET()//Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
        {  return (get_bytes(data,  4, 8)); }
        public char camera_id_GET()//Camera ID (1 for first, 2 for second, etc.)
        {  return (char)((char) get_bytes(data,  12, 1)); }
        public int lat_GET()//Latitude, expressed as degrees * 1E7 where image was taken
        {  return (int)((int) get_bytes(data,  13, 4)); }
        public int lon_GET()//Longitude, expressed as degrees * 1E7 where capture was taken
        {  return (int)((int) get_bytes(data,  17, 4)); }
        public int alt_GET()//Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
        {  return (int)((int) get_bytes(data,  21, 4)); }
        public int relative_alt_GET()//Altitude above ground in meters, expressed as * 1E3 where image was taken
        {  return (int)((int) get_bytes(data,  25, 4)); }
        public float[] q_GET(float[]  dst_ch, int pos)  //Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
        {
            for(int BYTE = 29, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] q_GET()//Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
        {return q_GET(new float[4], 0);} public int image_index_GET()//Zero based index of this image (image count since armed -1)
        {  return (int)((int) get_bytes(data,  45, 4)); }
        public byte capture_result_GET()//Boolean indicating success (1) or failure (0) while capturing this image.
        {  return (byte)((byte) get_bytes(data,  49, 1)); }
        public String file_url_TRY(Bounds.Inside ph)//URL of image taken. Either local storage or http:foo.jpg if camera provides an HTTP interface.
        {
            if(ph.field_bit !=  402 && !try_visit_field(ph, 402)  ||  !try_visit_item(ph, 0)) return null;
            return new String(file_url_GET(ph, new char[ph.items], 0));
        }
        public char[] file_url_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //URL of image taken. Either local storage or http:foo.jpg if camera provides an HTTP interface.
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int file_url_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  402 && !try_visit_field(ph, 402)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
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

        static final Collection<OnReceive.Handler<SET_ACTUATOR_CONTROL_TARGET, Channel>> on_SET_ACTUATOR_CONTROL_TARGET = new OnReceive<>();
        static final Collection<OnReceive.Handler<ACTUATOR_CONTROL_TARGET, Channel>> on_ACTUATOR_CONTROL_TARGET = new OnReceive<>();
        static final Collection<OnReceive.Handler<ALTITUDE, Channel>> on_ALTITUDE = new OnReceive<>();
        static final Collection<OnReceive.Handler<RESOURCE_REQUEST, Channel>> on_RESOURCE_REQUEST = new OnReceive<>();
        static final Collection<OnReceive.Handler<SCALED_PRESSURE3, Channel>> on_SCALED_PRESSURE3 = new OnReceive<>();
        static final Collection<OnReceive.Handler<FOLLOW_TARGET, Channel>> on_FOLLOW_TARGET = new OnReceive<>();
        static final Collection<OnReceive.Handler<CONTROL_SYSTEM_STATE, Channel>> on_CONTROL_SYSTEM_STATE = new OnReceive<>();
        static final Collection<OnReceive.Handler<BATTERY_STATUS, Channel>> on_BATTERY_STATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<AUTOPILOT_VERSION, Channel>> on_AUTOPILOT_VERSION = new OnReceive<>();
        static final Collection<OnReceive.Handler<LANDING_TARGET, Channel>> on_LANDING_TARGET = new OnReceive<>();
        static final Collection<OnReceive.Handler<AQ_TELEMETRY_F, Channel>> on_AQ_TELEMETRY_F = new OnReceive<>();
        static final Collection<OnReceive.Handler<AQ_ESC_TELEMETRY, Channel>> on_AQ_ESC_TELEMETRY = new OnReceive<>();
        static final Collection<OnReceive.Handler<ESTIMATOR_STATUS, Channel>> on_ESTIMATOR_STATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<WIND_COV, Channel>> on_WIND_COV = new OnReceive<>();
        static final Collection<OnReceive.Handler<GPS_INPUT, Channel>> on_GPS_INPUT = new OnReceive<>();
        static final Collection<OnReceive.Handler<GPS_RTCM_DATA, Channel>> on_GPS_RTCM_DATA = new OnReceive<>();
        static final Collection<OnReceive.Handler<HIGH_LATENCY, Channel>> on_HIGH_LATENCY = new OnReceive<>();
        static final Collection<OnReceive.Handler<VIBRATION, Channel>> on_VIBRATION = new OnReceive<>();
        static final Collection<OnReceive.Handler<HOME_POSITION, Channel>> on_HOME_POSITION = new OnReceive<>();
        static final Collection<OnReceive.Handler<SET_HOME_POSITION, Channel>> on_SET_HOME_POSITION = new OnReceive<>();
        static final Collection<OnReceive.Handler<MESSAGE_INTERVAL, Channel>> on_MESSAGE_INTERVAL = new OnReceive<>();
        static final Collection<OnReceive.Handler<EXTENDED_SYS_STATE, Channel>> on_EXTENDED_SYS_STATE = new OnReceive<>();
        static final Collection<OnReceive.Handler<ADSB_VEHICLE, Channel>> on_ADSB_VEHICLE = new OnReceive<>();
        static final Collection<OnReceive.Handler<COLLISION, Channel>> on_COLLISION = new OnReceive<>();
        static final Collection<OnReceive.Handler<V2_EXTENSION, Channel>> on_V2_EXTENSION = new OnReceive<>();
        static final Collection<OnReceive.Handler<MEMORY_VECT, Channel>> on_MEMORY_VECT = new OnReceive<>();
        static final Collection<OnReceive.Handler<DEBUG_VECT, Channel>> on_DEBUG_VECT = new OnReceive<>();
        static final Collection<OnReceive.Handler<NAMED_VALUE_FLOAT, Channel>> on_NAMED_VALUE_FLOAT = new OnReceive<>();
        static final Collection<OnReceive.Handler<NAMED_VALUE_INT, Channel>> on_NAMED_VALUE_INT = new OnReceive<>();
        static final Collection<OnReceive.Handler<STATUSTEXT, Channel>> on_STATUSTEXT = new OnReceive<>();
        static final Collection<OnReceive.Handler<DEBUG, Channel>> on_DEBUG = new OnReceive<>();
        static final Collection<OnReceive.Handler<SETUP_SIGNING, Channel>> on_SETUP_SIGNING = new OnReceive<>();
        static final Collection<OnReceive.Handler<BUTTON_CHANGE, Channel>> on_BUTTON_CHANGE = new OnReceive<>();
        static final Collection<OnReceive.Handler<PLAY_TUNE, Channel>> on_PLAY_TUNE = new OnReceive<>();
        static final Collection<OnReceive.Handler<CAMERA_INFORMATION, Channel>> on_CAMERA_INFORMATION = new OnReceive<>();
        static final Collection<OnReceive.Handler<CAMERA_SETTINGS, Channel>> on_CAMERA_SETTINGS = new OnReceive<>();
        static final Collection<OnReceive.Handler<STORAGE_INFORMATION, Channel>> on_STORAGE_INFORMATION = new OnReceive<>();
        static final Collection<OnReceive.Handler<CAMERA_CAPTURE_STATUS, Channel>> on_CAMERA_CAPTURE_STATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<CAMERA_IMAGE_CAPTURED, Channel>> on_CAMERA_IMAGE_CAPTURED = new OnReceive<>();
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
                case 139:
                    if(pack == null) return new SET_ACTUATOR_CONTROL_TARGET();
                    ((OnReceive) on_SET_ACTUATOR_CONTROL_TARGET).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 140:
                    if(pack == null) return new ACTUATOR_CONTROL_TARGET();
                    ((OnReceive) on_ACTUATOR_CONTROL_TARGET).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 141:
                    if(pack == null) return new ALTITUDE();
                    ((OnReceive) on_ALTITUDE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 142:
                    if(pack == null) return new RESOURCE_REQUEST();
                    ((OnReceive) on_RESOURCE_REQUEST).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 143:
                    if(pack == null) return new SCALED_PRESSURE3();
                    ((OnReceive) on_SCALED_PRESSURE3).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 144:
                    if(pack == null) return new FOLLOW_TARGET();
                    ((OnReceive) on_FOLLOW_TARGET).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 146:
                    if(pack == null) return new CONTROL_SYSTEM_STATE();
                    ((OnReceive) on_CONTROL_SYSTEM_STATE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 147:
                    if(pack == null) return new BATTERY_STATUS();
                    ((OnReceive) on_BATTERY_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 148:
                    if(pack == null) return new AUTOPILOT_VERSION();
                    ((OnReceive) on_AUTOPILOT_VERSION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 149:
                    if(pack == null) return new LANDING_TARGET();
                    ((OnReceive) on_LANDING_TARGET).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 150:
                    if(pack == null) return new AQ_TELEMETRY_F();
                    ((OnReceive) on_AQ_TELEMETRY_F).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 152:
                    if(pack == null) return new AQ_ESC_TELEMETRY();
                    ((OnReceive) on_AQ_ESC_TELEMETRY).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 230:
                    if(pack == null) return new ESTIMATOR_STATUS();
                    ((OnReceive) on_ESTIMATOR_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 231:
                    if(pack == null) return new WIND_COV();
                    ((OnReceive) on_WIND_COV).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 232:
                    if(pack == null) return new GPS_INPUT();
                    ((OnReceive) on_GPS_INPUT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 233:
                    if(pack == null) return new GPS_RTCM_DATA();
                    ((OnReceive) on_GPS_RTCM_DATA).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 234:
                    if(pack == null) return new HIGH_LATENCY();
                    ((OnReceive) on_HIGH_LATENCY).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 241:
                    if(pack == null) return new VIBRATION();
                    ((OnReceive) on_VIBRATION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 242:
                    if(pack == null) return new HOME_POSITION();
                    ((OnReceive) on_HOME_POSITION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 243:
                    if(pack == null) return new SET_HOME_POSITION();
                    ((OnReceive) on_SET_HOME_POSITION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 244:
                    if(pack == null) return new MESSAGE_INTERVAL();
                    ((OnReceive) on_MESSAGE_INTERVAL).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 245:
                    if(pack == null) return new EXTENDED_SYS_STATE();
                    ((OnReceive) on_EXTENDED_SYS_STATE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 246:
                    if(pack == null) return new ADSB_VEHICLE();
                    ((OnReceive) on_ADSB_VEHICLE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 247:
                    if(pack == null) return new COLLISION();
                    ((OnReceive) on_COLLISION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 248:
                    if(pack == null) return new V2_EXTENSION();
                    ((OnReceive) on_V2_EXTENSION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 249:
                    if(pack == null) return new MEMORY_VECT();
                    ((OnReceive) on_MEMORY_VECT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 250:
                    if(pack == null) return new DEBUG_VECT();
                    ((OnReceive) on_DEBUG_VECT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 251:
                    if(pack == null) return new NAMED_VALUE_FLOAT();
                    ((OnReceive) on_NAMED_VALUE_FLOAT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 252:
                    if(pack == null) return new NAMED_VALUE_INT();
                    ((OnReceive) on_NAMED_VALUE_INT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 253:
                    if(pack == null) return new STATUSTEXT();
                    ((OnReceive) on_STATUSTEXT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 254:
                    if(pack == null) return new DEBUG();
                    ((OnReceive) on_DEBUG).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 256:
                    if(pack == null) return new SETUP_SIGNING();
                    ((OnReceive) on_SETUP_SIGNING).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 257:
                    if(pack == null) return new BUTTON_CHANGE();
                    ((OnReceive) on_BUTTON_CHANGE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 258:
                    if(pack == null) return new PLAY_TUNE();
                    ((OnReceive) on_PLAY_TUNE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 259:
                    if(pack == null) return new CAMERA_INFORMATION();
                    ((OnReceive) on_CAMERA_INFORMATION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 260:
                    if(pack == null) return new CAMERA_SETTINGS();
                    ((OnReceive) on_CAMERA_SETTINGS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 261:
                    if(pack == null) return new STORAGE_INFORMATION();
                    ((OnReceive) on_STORAGE_INFORMATION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 262:
                    if(pack == null) return new CAMERA_CAPTURE_STATUS();
                    ((OnReceive) on_CAMERA_CAPTURE_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 263:
                    if(pack == null) return new CAMERA_IMAGE_CAPTURED();
                    ((OnReceive) on_CAMERA_IMAGE_CAPTURED).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
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
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_VTOL_RESERVED5);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_STANDBY);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
            assert(pack.custom_mode_GET() == 4229789202L);
            assert(pack.mavlink_version_GET() == (char)196);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.type_SET(MAV_TYPE.MAV_TYPE_VTOL_RESERVED5) ;
        p0.custom_mode_SET(4229789202L) ;
        p0.mavlink_version_SET((char)196) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_STANDBY) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY) ;
        p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.errors_count2_GET() == (char)47109);
            assert(pack.voltage_battery_GET() == (char)35393);
            assert(pack.onboard_control_sensors_enabled_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG);
            assert(pack.errors_count4_GET() == (char)3058);
            assert(pack.onboard_control_sensors_present_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);
            assert(pack.drop_rate_comm_GET() == (char)14988);
            assert(pack.errors_count1_GET() == (char)51072);
            assert(pack.onboard_control_sensors_health_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO);
            assert(pack.current_battery_GET() == (short)10330);
            assert(pack.errors_count3_GET() == (char)16179);
            assert(pack.errors_comm_GET() == (char)33430);
            assert(pack.battery_remaining_GET() == (byte) - 114);
            assert(pack.load_GET() == (char)8905);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.current_battery_SET((short)10330) ;
        p1.errors_count4_SET((char)3058) ;
        p1.errors_count1_SET((char)51072) ;
        p1.battery_remaining_SET((byte) - 114) ;
        p1.errors_count2_SET((char)47109) ;
        p1.errors_comm_SET((char)33430) ;
        p1.drop_rate_comm_SET((char)14988) ;
        p1.load_SET((char)8905) ;
        p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO) ;
        p1.errors_count3_SET((char)16179) ;
        p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS) ;
        p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG) ;
        p1.voltage_battery_SET((char)35393) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 219487151177900419L);
            assert(pack.time_boot_ms_GET() == 3934735628L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(3934735628L) ;
        p2.time_unix_usec_SET(219487151177900419L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.afy_GET() == -1.8175799E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.vy_GET() == -7.3610924E37F);
            assert(pack.yaw_GET() == -2.8236836E38F);
            assert(pack.y_GET() == -7.8209813E37F);
            assert(pack.type_mask_GET() == (char)52269);
            assert(pack.z_GET() == -3.276474E38F);
            assert(pack.yaw_rate_GET() == -2.318856E38F);
            assert(pack.vx_GET() == 5.252768E37F);
            assert(pack.time_boot_ms_GET() == 2470660752L);
            assert(pack.x_GET() == 1.5205462E38F);
            assert(pack.vz_GET() == 1.4230658E38F);
            assert(pack.afx_GET() == -2.1532508E38F);
            assert(pack.afz_GET() == 3.10201E38F);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.afx_SET(-2.1532508E38F) ;
        p3.vz_SET(1.4230658E38F) ;
        p3.x_SET(1.5205462E38F) ;
        p3.type_mask_SET((char)52269) ;
        p3.vx_SET(5.252768E37F) ;
        p3.vy_SET(-7.3610924E37F) ;
        p3.time_boot_ms_SET(2470660752L) ;
        p3.yaw_rate_SET(-2.318856E38F) ;
        p3.y_SET(-7.8209813E37F) ;
        p3.afy_SET(-1.8175799E37F) ;
        p3.yaw_SET(-2.8236836E38F) ;
        p3.z_SET(-3.276474E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p3.afz_SET(3.10201E38F) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 3828718140L);
            assert(pack.target_component_GET() == (char)148);
            assert(pack.target_system_GET() == (char)231);
            assert(pack.time_usec_GET() == 3005173331382416232L);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.time_usec_SET(3005173331382416232L) ;
        p4.seq_SET(3828718140L) ;
        p4.target_system_SET((char)231) ;
        p4.target_component_SET((char)148) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)77);
            assert(pack.control_request_GET() == (char)70);
            assert(pack.version_GET() == (char)133);
            assert(pack.passkey_LEN(ph) == 12);
            assert(pack.passkey_TRY(ph).equals("fwshdyofufxk"));
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.target_system_SET((char)77) ;
        p5.passkey_SET("fwshdyofufxk", PH) ;
        p5.version_SET((char)133) ;
        p5.control_request_SET((char)70) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.ack_GET() == (char)37);
            assert(pack.control_request_GET() == (char)32);
            assert(pack.gcs_system_id_GET() == (char)195);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.ack_SET((char)37) ;
        p6.gcs_system_id_SET((char)195) ;
        p6.control_request_SET((char)32) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 23);
            assert(pack.key_TRY(ph).equals("gddllNppKpCgtyrjhwtrnin"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("gddllNppKpCgtyrjhwtrnin", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
            assert(pack.custom_mode_GET() == 195819283L);
            assert(pack.target_system_GET() == (char)165);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.custom_mode_SET(195819283L) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_GUIDED_DISARMED) ;
        p11.target_system_SET((char)165) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)12);
            assert(pack.param_index_GET() == (short)10846);
            assert(pack.param_id_LEN(ph) == 7);
            assert(pack.param_id_TRY(ph).equals("fnalgan"));
            assert(pack.target_component_GET() == (char)107);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_system_SET((char)12) ;
        p20.param_index_SET((short)10846) ;
        p20.param_id_SET("fnalgan", PH) ;
        p20.target_component_SET((char)107) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)222);
            assert(pack.target_component_GET() == (char)188);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)188) ;
        p21.target_system_SET((char)222) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("vlom"));
            assert(pack.param_index_GET() == (char)36223);
            assert(pack.param_value_GET() == -1.4947445E38F);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
            assert(pack.param_count_GET() == (char)21477);
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_index_SET((char)36223) ;
        p22.param_count_SET((char)21477) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32) ;
        p22.param_value_SET(-1.4947445E38F) ;
        p22.param_id_SET("vlom", PH) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_value_GET() == 7.287926E37F);
            assert(pack.target_component_GET() == (char)69);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
            assert(pack.param_id_LEN(ph) == 15);
            assert(pack.param_id_TRY(ph).equals("cogizbggzzpxyae"));
            assert(pack.target_system_GET() == (char)212);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.param_value_SET(7.287926E37F) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32) ;
        p23.target_system_SET((char)212) ;
        p23.target_component_SET((char)69) ;
        p23.param_id_SET("cogizbggzzpxyae", PH) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.eph_GET() == (char)16923);
            assert(pack.alt_GET() == 1723050100);
            assert(pack.vel_acc_TRY(ph) == 3962395574L);
            assert(pack.satellites_visible_GET() == (char)127);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
            assert(pack.cog_GET() == (char)63036);
            assert(pack.v_acc_TRY(ph) == 1910932740L);
            assert(pack.h_acc_TRY(ph) == 2705903098L);
            assert(pack.vel_GET() == (char)62056);
            assert(pack.epv_GET() == (char)26508);
            assert(pack.time_usec_GET() == 6991745847191459570L);
            assert(pack.lat_GET() == -1761781992);
            assert(pack.lon_GET() == -673805598);
            assert(pack.alt_ellipsoid_TRY(ph) == -1577974575);
            assert(pack.hdg_acc_TRY(ph) == 2741036783L);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.alt_ellipsoid_SET(-1577974575, PH) ;
        p24.epv_SET((char)26508) ;
        p24.h_acc_SET(2705903098L, PH) ;
        p24.lat_SET(-1761781992) ;
        p24.v_acc_SET(1910932740L, PH) ;
        p24.lon_SET(-673805598) ;
        p24.vel_SET((char)62056) ;
        p24.time_usec_SET(6991745847191459570L) ;
        p24.satellites_visible_SET((char)127) ;
        p24.alt_SET(1723050100) ;
        p24.hdg_acc_SET(2741036783L, PH) ;
        p24.vel_acc_SET(3962395574L, PH) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX) ;
        p24.eph_SET((char)16923) ;
        p24.cog_SET((char)63036) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)245, (char)129, (char)83, (char)195, (char)49, (char)228, (char)23, (char)52, (char)156, (char)95, (char)228, (char)2, (char)109, (char)60, (char)110, (char)101, (char)209, (char)54, (char)101, (char)5}));
            assert(pack.satellites_visible_GET() == (char)238);
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)23, (char)235, (char)191, (char)150, (char)75, (char)172, (char)73, (char)239, (char)176, (char)114, (char)142, (char)188, (char)166, (char)252, (char)232, (char)98, (char)184, (char)218, (char)194, (char)231}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)243, (char)69, (char)161, (char)209, (char)205, (char)203, (char)37, (char)3, (char)74, (char)227, (char)232, (char)242, (char)246, (char)190, (char)16, (char)184, (char)128, (char)131, (char)161, (char)137}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)161, (char)83, (char)45, (char)68, (char)131, (char)63, (char)209, (char)149, (char)84, (char)151, (char)150, (char)244, (char)237, (char)21, (char)100, (char)154, (char)238, (char)5, (char)180, (char)122}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)39, (char)60, (char)142, (char)63, (char)27, (char)168, (char)243, (char)120, (char)63, (char)67, (char)27, (char)133, (char)236, (char)9, (char)128, (char)66, (char)179, (char)126, (char)161, (char)13}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellites_visible_SET((char)238) ;
        p25.satellite_snr_SET(new char[] {(char)23, (char)235, (char)191, (char)150, (char)75, (char)172, (char)73, (char)239, (char)176, (char)114, (char)142, (char)188, (char)166, (char)252, (char)232, (char)98, (char)184, (char)218, (char)194, (char)231}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)39, (char)60, (char)142, (char)63, (char)27, (char)168, (char)243, (char)120, (char)63, (char)67, (char)27, (char)133, (char)236, (char)9, (char)128, (char)66, (char)179, (char)126, (char)161, (char)13}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)161, (char)83, (char)45, (char)68, (char)131, (char)63, (char)209, (char)149, (char)84, (char)151, (char)150, (char)244, (char)237, (char)21, (char)100, (char)154, (char)238, (char)5, (char)180, (char)122}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)243, (char)69, (char)161, (char)209, (char)205, (char)203, (char)37, (char)3, (char)74, (char)227, (char)232, (char)242, (char)246, (char)190, (char)16, (char)184, (char)128, (char)131, (char)161, (char)137}, 0) ;
        p25.satellite_used_SET(new char[] {(char)245, (char)129, (char)83, (char)195, (char)49, (char)228, (char)23, (char)52, (char)156, (char)95, (char)228, (char)2, (char)109, (char)60, (char)110, (char)101, (char)209, (char)54, (char)101, (char)5}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short) -29401);
            assert(pack.zmag_GET() == (short) -25658);
            assert(pack.ygyro_GET() == (short)17471);
            assert(pack.zgyro_GET() == (short)3053);
            assert(pack.time_boot_ms_GET() == 278532626L);
            assert(pack.zacc_GET() == (short) -27712);
            assert(pack.xmag_GET() == (short)19529);
            assert(pack.ymag_GET() == (short)1571);
            assert(pack.yacc_GET() == (short)19312);
            assert(pack.xgyro_GET() == (short) -14399);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.ygyro_SET((short)17471) ;
        p26.time_boot_ms_SET(278532626L) ;
        p26.ymag_SET((short)1571) ;
        p26.zmag_SET((short) -25658) ;
        p26.xacc_SET((short) -29401) ;
        p26.xmag_SET((short)19529) ;
        p26.yacc_SET((short)19312) ;
        p26.xgyro_SET((short) -14399) ;
        p26.zgyro_SET((short)3053) ;
        p26.zacc_SET((short) -27712) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short) -7334);
            assert(pack.ymag_GET() == (short) -23614);
            assert(pack.zmag_GET() == (short)12669);
            assert(pack.xmag_GET() == (short)22306);
            assert(pack.ygyro_GET() == (short)7076);
            assert(pack.zgyro_GET() == (short) -23456);
            assert(pack.zacc_GET() == (short) -32234);
            assert(pack.xacc_GET() == (short)29717);
            assert(pack.time_usec_GET() == 2625702490968058720L);
            assert(pack.xgyro_GET() == (short) -26524);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.zgyro_SET((short) -23456) ;
        p27.yacc_SET((short) -7334) ;
        p27.zacc_SET((short) -32234) ;
        p27.xgyro_SET((short) -26524) ;
        p27.zmag_SET((short)12669) ;
        p27.ygyro_SET((short)7076) ;
        p27.xmag_SET((short)22306) ;
        p27.time_usec_SET(2625702490968058720L) ;
        p27.xacc_SET((short)29717) ;
        p27.ymag_SET((short) -23614) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == (short) -2655);
            assert(pack.press_diff2_GET() == (short) -29550);
            assert(pack.press_diff1_GET() == (short) -2306);
            assert(pack.temperature_GET() == (short) -9466);
            assert(pack.time_usec_GET() == 6370155651294448429L);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.time_usec_SET(6370155651294448429L) ;
        p28.press_diff2_SET((short) -29550) ;
        p28.temperature_SET((short) -9466) ;
        p28.press_abs_SET((short) -2655) ;
        p28.press_diff1_SET((short) -2306) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -9.435614E37F);
            assert(pack.temperature_GET() == (short)26320);
            assert(pack.time_boot_ms_GET() == 3048081648L);
            assert(pack.press_diff_GET() == -1.206733E38F);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.temperature_SET((short)26320) ;
        p29.press_abs_SET(-9.435614E37F) ;
        p29.press_diff_SET(-1.206733E38F) ;
        p29.time_boot_ms_SET(3048081648L) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3368493300L);
            assert(pack.rollspeed_GET() == 8.526429E37F);
            assert(pack.yawspeed_GET() == 4.440231E37F);
            assert(pack.roll_GET() == 2.9495816E38F);
            assert(pack.pitch_GET() == -3.1302826E38F);
            assert(pack.pitchspeed_GET() == 3.1070126E38F);
            assert(pack.yaw_GET() == -2.3285824E37F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.yaw_SET(-2.3285824E37F) ;
        p30.time_boot_ms_SET(3368493300L) ;
        p30.roll_SET(2.9495816E38F) ;
        p30.pitch_SET(-3.1302826E38F) ;
        p30.pitchspeed_SET(3.1070126E38F) ;
        p30.rollspeed_SET(8.526429E37F) ;
        p30.yawspeed_SET(4.440231E37F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == -1.7607357E38F);
            assert(pack.q3_GET() == 1.3591574E38F);
            assert(pack.time_boot_ms_GET() == 992669265L);
            assert(pack.q2_GET() == 8.451207E37F);
            assert(pack.pitchspeed_GET() == 2.0985386E38F);
            assert(pack.rollspeed_GET() == 1.218497E37F);
            assert(pack.q4_GET() == -3.0239432E38F);
            assert(pack.q1_GET() == -1.0029911E38F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q1_SET(-1.0029911E38F) ;
        p31.time_boot_ms_SET(992669265L) ;
        p31.yawspeed_SET(-1.7607357E38F) ;
        p31.q3_SET(1.3591574E38F) ;
        p31.pitchspeed_SET(2.0985386E38F) ;
        p31.q2_SET(8.451207E37F) ;
        p31.rollspeed_SET(1.218497E37F) ;
        p31.q4_SET(-3.0239432E38F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2722534078L);
            assert(pack.z_GET() == -2.4758755E37F);
            assert(pack.vz_GET() == 3.3351853E38F);
            assert(pack.vy_GET() == 2.7909545E38F);
            assert(pack.vx_GET() == 1.6444504E38F);
            assert(pack.x_GET() == -2.2533883E38F);
            assert(pack.y_GET() == 7.641588E37F);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.x_SET(-2.2533883E38F) ;
        p32.z_SET(-2.4758755E37F) ;
        p32.time_boot_ms_SET(2722534078L) ;
        p32.vy_SET(2.7909545E38F) ;
        p32.y_SET(7.641588E37F) ;
        p32.vz_SET(3.3351853E38F) ;
        p32.vx_SET(1.6444504E38F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 78727497);
            assert(pack.time_boot_ms_GET() == 1294875146L);
            assert(pack.relative_alt_GET() == 794641568);
            assert(pack.hdg_GET() == (char)54115);
            assert(pack.vx_GET() == (short)523);
            assert(pack.lat_GET() == 1002160804);
            assert(pack.vy_GET() == (short) -11959);
            assert(pack.lon_GET() == 1918470644);
            assert(pack.vz_GET() == (short)24863);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.lat_SET(1002160804) ;
        p33.alt_SET(78727497) ;
        p33.lon_SET(1918470644) ;
        p33.vy_SET((short) -11959) ;
        p33.vx_SET((short)523) ;
        p33.time_boot_ms_SET(1294875146L) ;
        p33.relative_alt_SET(794641568) ;
        p33.hdg_SET((char)54115) ;
        p33.vz_SET((short)24863) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan6_scaled_GET() == (short)454);
            assert(pack.chan3_scaled_GET() == (short)4030);
            assert(pack.chan2_scaled_GET() == (short) -32106);
            assert(pack.chan8_scaled_GET() == (short)24197);
            assert(pack.chan1_scaled_GET() == (short)9098);
            assert(pack.time_boot_ms_GET() == 3217260943L);
            assert(pack.chan5_scaled_GET() == (short) -12954);
            assert(pack.port_GET() == (char)44);
            assert(pack.rssi_GET() == (char)25);
            assert(pack.chan4_scaled_GET() == (short) -4284);
            assert(pack.chan7_scaled_GET() == (short) -1311);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan2_scaled_SET((short) -32106) ;
        p34.chan1_scaled_SET((short)9098) ;
        p34.chan7_scaled_SET((short) -1311) ;
        p34.chan4_scaled_SET((short) -4284) ;
        p34.port_SET((char)44) ;
        p34.chan6_scaled_SET((short)454) ;
        p34.chan3_scaled_SET((short)4030) ;
        p34.chan8_scaled_SET((short)24197) ;
        p34.chan5_scaled_SET((short) -12954) ;
        p34.time_boot_ms_SET(3217260943L) ;
        p34.rssi_SET((char)25) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan8_raw_GET() == (char)42789);
            assert(pack.time_boot_ms_GET() == 2905591053L);
            assert(pack.chan1_raw_GET() == (char)43183);
            assert(pack.port_GET() == (char)166);
            assert(pack.chan7_raw_GET() == (char)606);
            assert(pack.chan5_raw_GET() == (char)43216);
            assert(pack.chan4_raw_GET() == (char)49480);
            assert(pack.chan2_raw_GET() == (char)40292);
            assert(pack.chan6_raw_GET() == (char)62605);
            assert(pack.rssi_GET() == (char)218);
            assert(pack.chan3_raw_GET() == (char)20189);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan5_raw_SET((char)43216) ;
        p35.chan3_raw_SET((char)20189) ;
        p35.rssi_SET((char)218) ;
        p35.time_boot_ms_SET(2905591053L) ;
        p35.chan6_raw_SET((char)62605) ;
        p35.chan1_raw_SET((char)43183) ;
        p35.chan8_raw_SET((char)42789) ;
        p35.chan2_raw_SET((char)40292) ;
        p35.chan7_raw_SET((char)606) ;
        p35.chan4_raw_SET((char)49480) ;
        p35.port_SET((char)166) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo16_raw_TRY(ph) == (char)8008);
            assert(pack.servo2_raw_GET() == (char)43037);
            assert(pack.port_GET() == (char)157);
            assert(pack.servo11_raw_TRY(ph) == (char)12487);
            assert(pack.servo10_raw_TRY(ph) == (char)7355);
            assert(pack.servo4_raw_GET() == (char)51377);
            assert(pack.servo9_raw_TRY(ph) == (char)47425);
            assert(pack.servo5_raw_GET() == (char)6131);
            assert(pack.servo3_raw_GET() == (char)61904);
            assert(pack.servo8_raw_GET() == (char)61938);
            assert(pack.servo15_raw_TRY(ph) == (char)11405);
            assert(pack.servo1_raw_GET() == (char)24740);
            assert(pack.time_usec_GET() == 2156730830L);
            assert(pack.servo7_raw_GET() == (char)14039);
            assert(pack.servo14_raw_TRY(ph) == (char)25625);
            assert(pack.servo13_raw_TRY(ph) == (char)61937);
            assert(pack.servo12_raw_TRY(ph) == (char)43385);
            assert(pack.servo6_raw_GET() == (char)5540);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo5_raw_SET((char)6131) ;
        p36.servo8_raw_SET((char)61938) ;
        p36.servo6_raw_SET((char)5540) ;
        p36.time_usec_SET(2156730830L) ;
        p36.servo11_raw_SET((char)12487, PH) ;
        p36.servo12_raw_SET((char)43385, PH) ;
        p36.servo1_raw_SET((char)24740) ;
        p36.servo9_raw_SET((char)47425, PH) ;
        p36.servo10_raw_SET((char)7355, PH) ;
        p36.servo4_raw_SET((char)51377) ;
        p36.port_SET((char)157) ;
        p36.servo15_raw_SET((char)11405, PH) ;
        p36.servo3_raw_SET((char)61904) ;
        p36.servo2_raw_SET((char)43037) ;
        p36.servo16_raw_SET((char)8008, PH) ;
        p36.servo7_raw_SET((char)14039) ;
        p36.servo13_raw_SET((char)61937, PH) ;
        p36.servo14_raw_SET((char)25625, PH) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_index_GET() == (short) -20850);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.start_index_GET() == (short) -187);
            assert(pack.target_component_GET() == (char)88);
            assert(pack.target_system_GET() == (char)65);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_component_SET((char)88) ;
        p37.end_index_SET((short) -20850) ;
        p37.start_index_SET((short) -187) ;
        p37.target_system_SET((char)65) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)192);
            assert(pack.end_index_GET() == (short)13965);
            assert(pack.target_system_GET() == (char)205);
            assert(pack.start_index_GET() == (short)19017);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_component_SET((char)192) ;
        p38.target_system_SET((char)205) ;
        p38.start_index_SET((short)19017) ;
        p38.end_index_SET((short)13965) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.current_GET() == (char)26);
            assert(pack.param2_GET() == -3.246967E38F);
            assert(pack.param3_GET() == 1.6700459E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.x_GET() == 1.3115712E38F);
            assert(pack.seq_GET() == (char)21995);
            assert(pack.param1_GET() == 2.9454991E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_FOLLOW);
            assert(pack.autocontinue_GET() == (char)184);
            assert(pack.y_GET() == -1.4682808E37F);
            assert(pack.target_system_GET() == (char)247);
            assert(pack.z_GET() == -3.00401E37F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_component_GET() == (char)180);
            assert(pack.param4_GET() == 1.5338307E38F);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.command_SET(MAV_CMD.MAV_CMD_DO_FOLLOW) ;
        p39.param3_SET(1.6700459E38F) ;
        p39.seq_SET((char)21995) ;
        p39.z_SET(-3.00401E37F) ;
        p39.autocontinue_SET((char)184) ;
        p39.param4_SET(1.5338307E38F) ;
        p39.param1_SET(2.9454991E38F) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p39.target_system_SET((char)247) ;
        p39.x_SET(1.3115712E38F) ;
        p39.param2_SET(-3.246967E38F) ;
        p39.current_SET((char)26) ;
        p39.target_component_SET((char)180) ;
        p39.y_SET(-1.4682808E37F) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)158);
            assert(pack.target_component_GET() == (char)162);
            assert(pack.seq_GET() == (char)47367);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_system_SET((char)158) ;
        p40.seq_SET((char)47367) ;
        p40.target_component_SET((char)162) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)221);
            assert(pack.seq_GET() == (char)5719);
            assert(pack.target_system_GET() == (char)10);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)10) ;
        p41.seq_SET((char)5719) ;
        p41.target_component_SET((char)221) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)26843);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)26843) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)62);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)176);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_component_SET((char)62) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p43.target_system_SET((char)176) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)41402);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_component_GET() == (char)157);
            assert(pack.target_system_GET() == (char)36);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.target_system_SET((char)36) ;
        p44.count_SET((char)41402) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p44.target_component_SET((char)157) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)173);
            assert(pack.target_component_GET() == (char)151);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)173) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p45.target_component_SET((char)151) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)61526);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)61526) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)57);
            assert(pack.target_component_GET() == (char)46);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.target_component_SET((char)46) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE) ;
        p47.target_system_SET((char)57) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)145);
            assert(pack.time_usec_TRY(ph) == 3265901216748487718L);
            assert(pack.altitude_GET() == 563159264);
            assert(pack.longitude_GET() == -735801860);
            assert(pack.latitude_GET() == 1887500091);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.time_usec_SET(3265901216748487718L, PH) ;
        p48.latitude_SET(1887500091) ;
        p48.altitude_SET(563159264) ;
        p48.longitude_SET(-735801860) ;
        p48.target_system_SET((char)145) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 1906589872066821413L);
            assert(pack.latitude_GET() == -357697262);
            assert(pack.longitude_GET() == -2107061667);
            assert(pack.altitude_GET() == 2020451810);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.latitude_SET(-357697262) ;
        p49.altitude_SET(2020451810) ;
        p49.time_usec_SET(1906589872066821413L, PH) ;
        p49.longitude_SET(-2107061667) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.param_value_min_GET() == 5.379197E37F);
            assert(pack.target_component_GET() == (char)208);
            assert(pack.param_id_LEN(ph) == 6);
            assert(pack.param_id_TRY(ph).equals("pcbhfj"));
            assert(pack.param_value0_GET() == 9.692265E37F);
            assert(pack.target_system_GET() == (char)10);
            assert(pack.param_index_GET() == (short) -13335);
            assert(pack.scale_GET() == -2.5740594E37F);
            assert(pack.param_value_max_GET() == -3.1791614E38F);
            assert(pack.parameter_rc_channel_index_GET() == (char)32);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.parameter_rc_channel_index_SET((char)32) ;
        p50.param_index_SET((short) -13335) ;
        p50.param_value_min_SET(5.379197E37F) ;
        p50.param_value_max_SET(-3.1791614E38F) ;
        p50.param_value0_SET(9.692265E37F) ;
        p50.target_system_SET((char)10) ;
        p50.target_component_SET((char)208) ;
        p50.scale_SET(-2.5740594E37F) ;
        p50.param_id_SET("pcbhfj", PH) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)231);
            assert(pack.seq_GET() == (char)51923);
            assert(pack.target_component_GET() == (char)16);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.seq_SET((char)51923) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p51.target_system_SET((char)231) ;
        p51.target_component_SET((char)16) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)44);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.p2y_GET() == -2.4741757E38F);
            assert(pack.p1y_GET() == 9.23473E37F);
            assert(pack.p2z_GET() == -3.050303E38F);
            assert(pack.target_component_GET() == (char)25);
            assert(pack.p1z_GET() == 1.1908851E38F);
            assert(pack.p2x_GET() == -1.7138567E38F);
            assert(pack.p1x_GET() == -1.5035955E38F);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p2y_SET(-2.4741757E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p54.p1y_SET(9.23473E37F) ;
        p54.p1x_SET(-1.5035955E38F) ;
        p54.target_component_SET((char)25) ;
        p54.p1z_SET(1.1908851E38F) ;
        p54.p2z_SET(-3.050303E38F) ;
        p54.target_system_SET((char)44) ;
        p54.p2x_SET(-1.7138567E38F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2z_GET() == 8.545271E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.p1y_GET() == -5.7137906E37F);
            assert(pack.p2x_GET() == -5.472071E37F);
            assert(pack.p2y_GET() == -1.9033512E38F);
            assert(pack.p1x_GET() == -2.761943E38F);
            assert(pack.p1z_GET() == 2.339984E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p2y_SET(-1.9033512E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p55.p1x_SET(-2.761943E38F) ;
        p55.p1y_SET(-5.7137906E37F) ;
        p55.p2z_SET(8.545271E37F) ;
        p55.p2x_SET(-5.472071E37F) ;
        p55.p1z_SET(2.339984E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.9356011E38F, 4.402487E37F, 6.3485073E37F, 1.6070465E38F}));
            assert(pack.time_usec_GET() == 4709209292955871890L);
            assert(pack.pitchspeed_GET() == -1.1393567E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.8953287E38F, -5.9778214E37F, -1.7332007E38F, 2.3721972E37F, -1.638817E37F, -2.6891447E38F, -7.0693674E37F, -3.0804127E38F, -3.3545197E38F}));
            assert(pack.yawspeed_GET() == 1.1859917E38F);
            assert(pack.rollspeed_GET() == 7.244595E37F);
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.covariance_SET(new float[] {-1.8953287E38F, -5.9778214E37F, -1.7332007E38F, 2.3721972E37F, -1.638817E37F, -2.6891447E38F, -7.0693674E37F, -3.0804127E38F, -3.3545197E38F}, 0) ;
        p61.pitchspeed_SET(-1.1393567E38F) ;
        p61.rollspeed_SET(7.244595E37F) ;
        p61.time_usec_SET(4709209292955871890L) ;
        p61.q_SET(new float[] {-2.9356011E38F, 4.402487E37F, 6.3485073E37F, 1.6070465E38F}, 0) ;
        p61.yawspeed_SET(1.1859917E38F) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.wp_dist_GET() == (char)22818);
            assert(pack.alt_error_GET() == 2.157972E38F);
            assert(pack.nav_pitch_GET() == -9.372392E37F);
            assert(pack.aspd_error_GET() == -1.5483632E37F);
            assert(pack.nav_roll_GET() == -1.8198234E38F);
            assert(pack.nav_bearing_GET() == (short) -7743);
            assert(pack.target_bearing_GET() == (short) -28645);
            assert(pack.xtrack_error_GET() == 6.2451916E36F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.nav_pitch_SET(-9.372392E37F) ;
        p62.wp_dist_SET((char)22818) ;
        p62.target_bearing_SET((short) -28645) ;
        p62.aspd_error_SET(-1.5483632E37F) ;
        p62.nav_bearing_SET((short) -7743) ;
        p62.nav_roll_SET(-1.8198234E38F) ;
        p62.xtrack_error_SET(6.2451916E36F) ;
        p62.alt_error_SET(2.157972E38F) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.8862382E38F, 1.5337227E38F, -2.1529246E38F, -2.44679E38F, 1.9281747E38F, -1.1781885E36F, 2.5445834E38F, 1.692966E38F, -3.2400878E38F, -1.199098E38F, 2.4222316E38F, -1.548177E38F, -2.6306095E37F, -2.7636402E38F, -2.5685305E38F, -2.8693592E38F, 3.3666871E38F, -2.5998285E38F, 2.7587267E38F, 2.616536E37F, -2.6389064E38F, -1.9697128E38F, 2.9765681E38F, -2.1299228E38F, -7.307942E37F, 3.8795136E37F, 2.8027765E38F, 1.1411219E38F, -1.8823632E38F, -3.0681964E38F, 1.761931E38F, -3.927462E37F, 1.6582409E38F, 2.5917323E38F, -2.9080824E38F, 2.088645E38F}));
            assert(pack.vx_GET() == -1.0166481E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
            assert(pack.time_usec_GET() == 7352938698694193620L);
            assert(pack.lat_GET() == 1680880364);
            assert(pack.relative_alt_GET() == 1311790041);
            assert(pack.alt_GET() == -454694410);
            assert(pack.lon_GET() == -999332381);
            assert(pack.vz_GET() == 1.6622527E38F);
            assert(pack.vy_GET() == 2.7433602E38F);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.lat_SET(1680880364) ;
        p63.vx_SET(-1.0166481E38F) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS) ;
        p63.vy_SET(2.7433602E38F) ;
        p63.lon_SET(-999332381) ;
        p63.time_usec_SET(7352938698694193620L) ;
        p63.relative_alt_SET(1311790041) ;
        p63.covariance_SET(new float[] {2.8862382E38F, 1.5337227E38F, -2.1529246E38F, -2.44679E38F, 1.9281747E38F, -1.1781885E36F, 2.5445834E38F, 1.692966E38F, -3.2400878E38F, -1.199098E38F, 2.4222316E38F, -1.548177E38F, -2.6306095E37F, -2.7636402E38F, -2.5685305E38F, -2.8693592E38F, 3.3666871E38F, -2.5998285E38F, 2.7587267E38F, 2.616536E37F, -2.6389064E38F, -1.9697128E38F, 2.9765681E38F, -2.1299228E38F, -7.307942E37F, 3.8795136E37F, 2.8027765E38F, 1.1411219E38F, -1.8823632E38F, -3.0681964E38F, 1.761931E38F, -3.927462E37F, 1.6582409E38F, 2.5917323E38F, -2.9080824E38F, 2.088645E38F}, 0) ;
        p63.vz_SET(1.6622527E38F) ;
        p63.alt_SET(-454694410) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {1.976406E38F, -4.880662E37F, -4.2070314E37F, -1.0612353E38F, -2.4810902E38F, -8.644612E37F, 7.8587714E36F, 2.6718032E38F, -2.9738598E38F, -2.4452416E38F, 2.4643953E38F, -5.995733E37F, -3.1514555E38F, 3.0870677E38F, -5.1083764E37F, -2.7763808E38F, -8.383842E37F, 1.4852484E38F, 1.1998223E38F, 1.2562378E38F, 1.3310351E38F, 1.8130208E37F, -8.585963E37F, -1.429483E38F, 3.943167E35F, -1.8553036E38F, 4.5654203E37F, -2.4229306E38F, -2.6704244E38F, -1.1559093E38F, -5.43473E37F, -1.3293136E37F, -2.9896937E38F, -1.4210061E38F, -1.184632E37F, -9.987234E37F, 1.105618E38F, 1.78031E38F, 3.737644E37F, 1.6934378E38F, 7.8364487E37F, 3.3849157E38F, 1.5308224E38F, -1.300364E38F, 2.0030147E38F}));
            assert(pack.time_usec_GET() == 6010277123273611652L);
            assert(pack.ax_GET() == 3.1987924E38F);
            assert(pack.vx_GET() == -1.0828287E38F);
            assert(pack.x_GET() == -2.795328E37F);
            assert(pack.ay_GET() == 3.0167843E38F);
            assert(pack.az_GET() == 2.8083535E38F);
            assert(pack.vy_GET() == -2.0742133E38F);
            assert(pack.y_GET() == -5.161769E37F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            assert(pack.z_GET() == 3.3976845E38F);
            assert(pack.vz_GET() == -6.553026E37F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.time_usec_SET(6010277123273611652L) ;
        p64.covariance_SET(new float[] {1.976406E38F, -4.880662E37F, -4.2070314E37F, -1.0612353E38F, -2.4810902E38F, -8.644612E37F, 7.8587714E36F, 2.6718032E38F, -2.9738598E38F, -2.4452416E38F, 2.4643953E38F, -5.995733E37F, -3.1514555E38F, 3.0870677E38F, -5.1083764E37F, -2.7763808E38F, -8.383842E37F, 1.4852484E38F, 1.1998223E38F, 1.2562378E38F, 1.3310351E38F, 1.8130208E37F, -8.585963E37F, -1.429483E38F, 3.943167E35F, -1.8553036E38F, 4.5654203E37F, -2.4229306E38F, -2.6704244E38F, -1.1559093E38F, -5.43473E37F, -1.3293136E37F, -2.9896937E38F, -1.4210061E38F, -1.184632E37F, -9.987234E37F, 1.105618E38F, 1.78031E38F, 3.737644E37F, 1.6934378E38F, 7.8364487E37F, 3.3849157E38F, 1.5308224E38F, -1.300364E38F, 2.0030147E38F}, 0) ;
        p64.ay_SET(3.0167843E38F) ;
        p64.ax_SET(3.1987924E38F) ;
        p64.z_SET(3.3976845E38F) ;
        p64.vy_SET(-2.0742133E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        p64.vz_SET(-6.553026E37F) ;
        p64.az_SET(2.8083535E38F) ;
        p64.vx_SET(-1.0828287E38F) ;
        p64.y_SET(-5.161769E37F) ;
        p64.x_SET(-2.795328E37F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan1_raw_GET() == (char)13705);
            assert(pack.chan15_raw_GET() == (char)3430);
            assert(pack.chan16_raw_GET() == (char)59427);
            assert(pack.chan9_raw_GET() == (char)19893);
            assert(pack.chancount_GET() == (char)179);
            assert(pack.chan7_raw_GET() == (char)46677);
            assert(pack.chan13_raw_GET() == (char)42537);
            assert(pack.rssi_GET() == (char)118);
            assert(pack.chan12_raw_GET() == (char)21904);
            assert(pack.chan17_raw_GET() == (char)1771);
            assert(pack.chan11_raw_GET() == (char)36310);
            assert(pack.chan14_raw_GET() == (char)21605);
            assert(pack.chan8_raw_GET() == (char)8818);
            assert(pack.chan4_raw_GET() == (char)34897);
            assert(pack.chan18_raw_GET() == (char)3541);
            assert(pack.time_boot_ms_GET() == 221799654L);
            assert(pack.chan10_raw_GET() == (char)10978);
            assert(pack.chan2_raw_GET() == (char)37717);
            assert(pack.chan5_raw_GET() == (char)55958);
            assert(pack.chan3_raw_GET() == (char)40040);
            assert(pack.chan6_raw_GET() == (char)49135);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan4_raw_SET((char)34897) ;
        p65.chan12_raw_SET((char)21904) ;
        p65.chan13_raw_SET((char)42537) ;
        p65.chan15_raw_SET((char)3430) ;
        p65.chan7_raw_SET((char)46677) ;
        p65.chan18_raw_SET((char)3541) ;
        p65.chan3_raw_SET((char)40040) ;
        p65.rssi_SET((char)118) ;
        p65.chan11_raw_SET((char)36310) ;
        p65.chan10_raw_SET((char)10978) ;
        p65.chan16_raw_SET((char)59427) ;
        p65.chan6_raw_SET((char)49135) ;
        p65.chancount_SET((char)179) ;
        p65.chan17_raw_SET((char)1771) ;
        p65.chan14_raw_SET((char)21605) ;
        p65.chan2_raw_SET((char)37717) ;
        p65.chan1_raw_SET((char)13705) ;
        p65.chan8_raw_SET((char)8818) ;
        p65.chan9_raw_SET((char)19893) ;
        p65.chan5_raw_SET((char)55958) ;
        p65.time_boot_ms_SET(221799654L) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.start_stop_GET() == (char)69);
            assert(pack.req_stream_id_GET() == (char)167);
            assert(pack.target_system_GET() == (char)67);
            assert(pack.target_component_GET() == (char)189);
            assert(pack.req_message_rate_GET() == (char)11032);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_component_SET((char)189) ;
        p66.req_stream_id_SET((char)167) ;
        p66.start_stop_SET((char)69) ;
        p66.req_message_rate_SET((char)11032) ;
        p66.target_system_SET((char)67) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.on_off_GET() == (char)37);
            assert(pack.message_rate_GET() == (char)45177);
            assert(pack.stream_id_GET() == (char)209);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.message_rate_SET((char)45177) ;
        p67.on_off_SET((char)37) ;
        p67.stream_id_SET((char)209) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == (short)25717);
            assert(pack.z_GET() == (short) -18072);
            assert(pack.buttons_GET() == (char)42355);
            assert(pack.y_GET() == (short) -17644);
            assert(pack.target_GET() == (char)189);
            assert(pack.r_GET() == (short) -32596);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.x_SET((short)25717) ;
        p69.z_SET((short) -18072) ;
        p69.buttons_SET((char)42355) ;
        p69.target_SET((char)189) ;
        p69.y_SET((short) -17644) ;
        p69.r_SET((short) -32596) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)105);
            assert(pack.chan1_raw_GET() == (char)52002);
            assert(pack.chan5_raw_GET() == (char)8420);
            assert(pack.chan3_raw_GET() == (char)53654);
            assert(pack.chan8_raw_GET() == (char)55123);
            assert(pack.target_system_GET() == (char)64);
            assert(pack.chan2_raw_GET() == (char)20213);
            assert(pack.chan6_raw_GET() == (char)4361);
            assert(pack.chan7_raw_GET() == (char)2672);
            assert(pack.chan4_raw_GET() == (char)61919);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan3_raw_SET((char)53654) ;
        p70.chan8_raw_SET((char)55123) ;
        p70.chan7_raw_SET((char)2672) ;
        p70.target_component_SET((char)105) ;
        p70.chan2_raw_SET((char)20213) ;
        p70.chan5_raw_SET((char)8420) ;
        p70.chan1_raw_SET((char)52002) ;
        p70.target_system_SET((char)64) ;
        p70.chan6_raw_SET((char)4361) ;
        p70.chan4_raw_SET((char)61919) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.param1_GET() == -1.9500237E37F);
            assert(pack.z_GET() == -2.6390565E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_SET_ROI);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.autocontinue_GET() == (char)34);
            assert(pack.current_GET() == (char)232);
            assert(pack.target_system_GET() == (char)136);
            assert(pack.target_component_GET() == (char)19);
            assert(pack.seq_GET() == (char)46926);
            assert(pack.y_GET() == 2143442203);
            assert(pack.param3_GET() == 9.096307E37F);
            assert(pack.param4_GET() == -1.2250998E38F);
            assert(pack.param2_GET() == -1.5324433E38F);
            assert(pack.x_GET() == -1333489531);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.z_SET(-2.6390565E38F) ;
        p73.param4_SET(-1.2250998E38F) ;
        p73.command_SET(MAV_CMD.MAV_CMD_DO_SET_ROI) ;
        p73.target_system_SET((char)136) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p73.param3_SET(9.096307E37F) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p73.seq_SET((char)46926) ;
        p73.autocontinue_SET((char)34) ;
        p73.x_SET(-1333489531) ;
        p73.param1_SET(-1.9500237E37F) ;
        p73.param2_SET(-1.5324433E38F) ;
        p73.current_SET((char)232) ;
        p73.y_SET(2143442203) ;
        p73.target_component_SET((char)19) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -2.486643E38F);
            assert(pack.airspeed_GET() == 9.776807E37F);
            assert(pack.throttle_GET() == (char)50325);
            assert(pack.heading_GET() == (short) -28489);
            assert(pack.climb_GET() == 2.9508987E38F);
            assert(pack.groundspeed_GET() == -1.6594356E38F);
        });
        GroundControl.VFR_HUD p74 = CommunicationChannel.new_VFR_HUD();
        PH.setPack(p74);
        p74.throttle_SET((char)50325) ;
        p74.heading_SET((short) -28489) ;
        p74.airspeed_SET(9.776807E37F) ;
        p74.alt_SET(-2.486643E38F) ;
        p74.groundspeed_SET(-1.6594356E38F) ;
        p74.climb_SET(2.9508987E38F) ;
        CommunicationChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.param4_GET() == -7.6866707E37F);
            assert(pack.autocontinue_GET() == (char)32);
            assert(pack.target_system_GET() == (char)117);
            assert(pack.x_GET() == 1089356821);
            assert(pack.param1_GET() == 2.836932E38F);
            assert(pack.z_GET() == 1.3766744E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_JUMP);
            assert(pack.param3_GET() == -1.7567558E38F);
            assert(pack.y_GET() == 734865848);
            assert(pack.target_component_GET() == (char)4);
            assert(pack.current_GET() == (char)28);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.param2_GET() == 3.337343E38F);
        });
        GroundControl.COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
        PH.setPack(p75);
        p75.param4_SET(-7.6866707E37F) ;
        p75.autocontinue_SET((char)32) ;
        p75.target_component_SET((char)4) ;
        p75.x_SET(1089356821) ;
        p75.current_SET((char)28) ;
        p75.target_system_SET((char)117) ;
        p75.y_SET(734865848) ;
        p75.z_SET(1.3766744E38F) ;
        p75.param3_SET(-1.7567558E38F) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p75.command_SET(MAV_CMD.MAV_CMD_DO_JUMP) ;
        p75.param2_SET(3.337343E38F) ;
        p75.param1_SET(2.836932E38F) ;
        CommunicationChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)37);
            assert(pack.param4_GET() == -2.0554389E38F);
            assert(pack.param2_GET() == 1.8955518E38F);
            assert(pack.param3_GET() == 3.3578734E38F);
            assert(pack.param1_GET() == -6.1770535E37F);
            assert(pack.param6_GET() == 2.065534E38F);
            assert(pack.target_system_GET() == (char)15);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING);
            assert(pack.param5_GET() == 2.2601087E38F);
            assert(pack.confirmation_GET() == (char)244);
            assert(pack.param7_GET() == -7.5550404E36F);
        });
        GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.param3_SET(3.3578734E38F) ;
        p76.param4_SET(-2.0554389E38F) ;
        p76.param1_SET(-6.1770535E37F) ;
        p76.confirmation_SET((char)244) ;
        p76.param7_SET(-7.5550404E36F) ;
        p76.target_component_SET((char)37) ;
        p76.param6_SET(2.065534E38F) ;
        p76.param2_SET(1.8955518E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING) ;
        p76.param5_SET(2.2601087E38F) ;
        p76.target_system_SET((char)15) ;
        CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_TRY(ph) == (char)217);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_OVERRIDE_GOTO);
            assert(pack.result_param2_TRY(ph) == 916071468);
            assert(pack.progress_TRY(ph) == (char)19);
            assert(pack.target_component_TRY(ph) == (char)56);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_DENIED);
        });
        GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.command_SET(MAV_CMD.MAV_CMD_OVERRIDE_GOTO) ;
        p77.target_system_SET((char)217, PH) ;
        p77.progress_SET((char)19, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_DENIED) ;
        p77.target_component_SET((char)56, PH) ;
        p77.result_param2_SET(916071468, PH) ;
        CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.mode_switch_GET() == (char)60);
            assert(pack.time_boot_ms_GET() == 668171114L);
            assert(pack.roll_GET() == -2.187744E38F);
            assert(pack.pitch_GET() == -3.2285367E38F);
            assert(pack.thrust_GET() == -1.5684457E38F);
            assert(pack.yaw_GET() == -2.4052542E38F);
            assert(pack.manual_override_switch_GET() == (char)233);
        });
        GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.pitch_SET(-3.2285367E38F) ;
        p81.manual_override_switch_SET((char)233) ;
        p81.roll_SET(-2.187744E38F) ;
        p81.yaw_SET(-2.4052542E38F) ;
        p81.time_boot_ms_SET(668171114L) ;
        p81.thrust_SET(-1.5684457E38F) ;
        p81.mode_switch_SET((char)60) ;
        CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)249);
            assert(pack.body_roll_rate_GET() == -1.1284985E38F);
            assert(pack.type_mask_GET() == (char)173);
            assert(pack.target_system_GET() == (char)254);
            assert(pack.body_pitch_rate_GET() == 1.966626E35F);
            assert(pack.body_yaw_rate_GET() == 1.842274E38F);
            assert(pack.time_boot_ms_GET() == 244384035L);
            assert(pack.thrust_GET() == 3.3970874E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.0447467E38F, 1.6651017E38F, 2.519477E38F, -3.0452643E38F}));
        });
        GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.body_pitch_rate_SET(1.966626E35F) ;
        p82.target_system_SET((char)254) ;
        p82.body_yaw_rate_SET(1.842274E38F) ;
        p82.target_component_SET((char)249) ;
        p82.time_boot_ms_SET(244384035L) ;
        p82.thrust_SET(3.3970874E38F) ;
        p82.q_SET(new float[] {3.0447467E38F, 1.6651017E38F, 2.519477E38F, -3.0452643E38F}, 0) ;
        p82.type_mask_SET((char)173) ;
        p82.body_roll_rate_SET(-1.1284985E38F) ;
        CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_yaw_rate_GET() == -1.2149083E38F);
            assert(pack.time_boot_ms_GET() == 4189692495L);
            assert(pack.body_pitch_rate_GET() == -1.3806161E37F);
            assert(pack.thrust_GET() == 2.6915697E38F);
            assert(pack.type_mask_GET() == (char)66);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.5368914E38F, -1.9855677E37F, -5.928158E37F, 5.990299E37F}));
            assert(pack.body_roll_rate_GET() == -1.3973368E38F);
        });
        GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.thrust_SET(2.6915697E38F) ;
        p83.body_yaw_rate_SET(-1.2149083E38F) ;
        p83.time_boot_ms_SET(4189692495L) ;
        p83.q_SET(new float[] {-1.5368914E38F, -1.9855677E37F, -5.928158E37F, 5.990299E37F}, 0) ;
        p83.type_mask_SET((char)66) ;
        p83.body_pitch_rate_SET(-1.3806161E37F) ;
        p83.body_roll_rate_SET(-1.3973368E38F) ;
        CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == 3.55365E37F);
            assert(pack.target_system_GET() == (char)214);
            assert(pack.time_boot_ms_GET() == 4098959794L);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.afz_GET() == 3.0728682E38F);
            assert(pack.yaw_GET() == -7.8591523E37F);
            assert(pack.yaw_rate_GET() == -1.5902826E38F);
            assert(pack.afx_GET() == 7.407978E37F);
            assert(pack.y_GET() == -2.043198E38F);
            assert(pack.vx_GET() == -2.803572E38F);
            assert(pack.x_GET() == 8.766276E37F);
            assert(pack.z_GET() == 2.1291747E38F);
            assert(pack.type_mask_GET() == (char)64980);
            assert(pack.vy_GET() == -1.5944123E38F);
            assert(pack.afy_GET() == 1.0653993E38F);
            assert(pack.target_component_GET() == (char)104);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.target_system_SET((char)214) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p84.vz_SET(3.55365E37F) ;
        p84.afy_SET(1.0653993E38F) ;
        p84.target_component_SET((char)104) ;
        p84.type_mask_SET((char)64980) ;
        p84.afz_SET(3.0728682E38F) ;
        p84.x_SET(8.766276E37F) ;
        p84.z_SET(2.1291747E38F) ;
        p84.yaw_rate_SET(-1.5902826E38F) ;
        p84.afx_SET(7.407978E37F) ;
        p84.time_boot_ms_SET(4098959794L) ;
        p84.vx_SET(-2.803572E38F) ;
        p84.y_SET(-2.043198E38F) ;
        p84.yaw_SET(-7.8591523E37F) ;
        p84.vy_SET(-1.5944123E38F) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.target_component_GET() == (char)123);
            assert(pack.target_system_GET() == (char)19);
            assert(pack.lat_int_GET() == -476874672);
            assert(pack.yaw_GET() == 1.9474829E38F);
            assert(pack.afy_GET() == -3.1223257E38F);
            assert(pack.afx_GET() == 3.0318383E38F);
            assert(pack.lon_int_GET() == 520237794);
            assert(pack.vz_GET() == 6.9069966E37F);
            assert(pack.alt_GET() == 2.456387E38F);
            assert(pack.afz_GET() == 3.0593194E38F);
            assert(pack.vx_GET() == 1.7769133E38F);
            assert(pack.vy_GET() == -2.1891431E38F);
            assert(pack.yaw_rate_GET() == -2.867256E37F);
            assert(pack.type_mask_GET() == (char)48426);
            assert(pack.time_boot_ms_GET() == 2293844899L);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.afx_SET(3.0318383E38F) ;
        p86.vx_SET(1.7769133E38F) ;
        p86.target_system_SET((char)19) ;
        p86.afz_SET(3.0593194E38F) ;
        p86.type_mask_SET((char)48426) ;
        p86.time_boot_ms_SET(2293844899L) ;
        p86.vz_SET(6.9069966E37F) ;
        p86.lat_int_SET(-476874672) ;
        p86.lon_int_SET(520237794) ;
        p86.afy_SET(-3.1223257E38F) ;
        p86.yaw_SET(1.9474829E38F) ;
        p86.target_component_SET((char)123) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p86.yaw_rate_SET(-2.867256E37F) ;
        p86.vy_SET(-2.1891431E38F) ;
        p86.alt_SET(2.456387E38F) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == -9.747954E37F);
            assert(pack.afy_GET() == -2.2675775E38F);
            assert(pack.yaw_rate_GET() == -9.846861E37F);
            assert(pack.vx_GET() == -1.3183807E38F);
            assert(pack.alt_GET() == -3.3277445E38F);
            assert(pack.yaw_GET() == -3.1331674E38F);
            assert(pack.lat_int_GET() == -1125025991);
            assert(pack.afx_GET() == -6.4294265E37F);
            assert(pack.time_boot_ms_GET() == 1735517742L);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.type_mask_GET() == (char)45918);
            assert(pack.vz_GET() == 5.0519446E37F);
            assert(pack.lon_int_GET() == -976660070);
            assert(pack.afz_GET() == -1.00791356E37F);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.lat_int_SET(-1125025991) ;
        p87.yaw_rate_SET(-9.846861E37F) ;
        p87.vz_SET(5.0519446E37F) ;
        p87.afy_SET(-2.2675775E38F) ;
        p87.afz_SET(-1.00791356E37F) ;
        p87.type_mask_SET((char)45918) ;
        p87.afx_SET(-6.4294265E37F) ;
        p87.lon_int_SET(-976660070) ;
        p87.yaw_SET(-3.1331674E38F) ;
        p87.alt_SET(-3.3277445E38F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p87.time_boot_ms_SET(1735517742L) ;
        p87.vy_SET(-9.747954E37F) ;
        p87.vx_SET(-1.3183807E38F) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 1.8537548E38F);
            assert(pack.z_GET() == -7.289548E37F);
            assert(pack.time_boot_ms_GET() == 2325842958L);
            assert(pack.y_GET() == 8.854665E37F);
            assert(pack.roll_GET() == -7.3411264E37F);
            assert(pack.x_GET() == -2.626541E38F);
            assert(pack.yaw_GET() == -1.9769543E38F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.z_SET(-7.289548E37F) ;
        p89.x_SET(-2.626541E38F) ;
        p89.roll_SET(-7.3411264E37F) ;
        p89.y_SET(8.854665E37F) ;
        p89.pitch_SET(1.8537548E38F) ;
        p89.time_boot_ms_SET(2325842958L) ;
        p89.yaw_SET(-1.9769543E38F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == (short)21251);
            assert(pack.alt_GET() == 1646013132);
            assert(pack.vz_GET() == (short)26102);
            assert(pack.lon_GET() == 746991189);
            assert(pack.yaw_GET() == 2.3342244E37F);
            assert(pack.time_usec_GET() == 5476436857210859891L);
            assert(pack.vy_GET() == (short)576);
            assert(pack.pitch_GET() == 1.7240896E38F);
            assert(pack.zacc_GET() == (short) -5206);
            assert(pack.yacc_GET() == (short) -1670);
            assert(pack.xacc_GET() == (short) -11220);
            assert(pack.yawspeed_GET() == -2.4492285E38F);
            assert(pack.pitchspeed_GET() == 2.329815E38F);
            assert(pack.rollspeed_GET() == 3.757094E36F);
            assert(pack.lat_GET() == 998485234);
            assert(pack.roll_GET() == 3.252665E38F);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.lon_SET(746991189) ;
        p90.vz_SET((short)26102) ;
        p90.lat_SET(998485234) ;
        p90.vy_SET((short)576) ;
        p90.xacc_SET((short) -11220) ;
        p90.roll_SET(3.252665E38F) ;
        p90.zacc_SET((short) -5206) ;
        p90.yawspeed_SET(-2.4492285E38F) ;
        p90.rollspeed_SET(3.757094E36F) ;
        p90.time_usec_SET(5476436857210859891L) ;
        p90.pitch_SET(1.7240896E38F) ;
        p90.pitchspeed_SET(2.329815E38F) ;
        p90.yaw_SET(2.3342244E37F) ;
        p90.yacc_SET((short) -1670) ;
        p90.alt_SET(1646013132) ;
        p90.vx_SET((short)21251) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.throttle_GET() == 1.6979876E38F);
            assert(pack.yaw_rudder_GET() == -2.7906657E38F);
            assert(pack.time_usec_GET() == 4545126453945480809L);
            assert(pack.pitch_elevator_GET() == -2.1301858E38F);
            assert(pack.aux2_GET() == 3.7766232E37F);
            assert(pack.aux3_GET() == -2.001661E38F);
            assert(pack.nav_mode_GET() == (char)108);
            assert(pack.aux4_GET() == 1.3219044E37F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_MANUAL_ARMED);
            assert(pack.aux1_GET() == 3.0987764E38F);
            assert(pack.roll_ailerons_GET() == -2.4562586E38F);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.aux3_SET(-2.001661E38F) ;
        p91.aux4_SET(1.3219044E37F) ;
        p91.aux2_SET(3.7766232E37F) ;
        p91.throttle_SET(1.6979876E38F) ;
        p91.nav_mode_SET((char)108) ;
        p91.aux1_SET(3.0987764E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_MANUAL_ARMED) ;
        p91.time_usec_SET(4545126453945480809L) ;
        p91.yaw_rudder_SET(-2.7906657E38F) ;
        p91.roll_ailerons_SET(-2.4562586E38F) ;
        p91.pitch_elevator_SET(-2.1301858E38F) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan9_raw_GET() == (char)30201);
            assert(pack.chan11_raw_GET() == (char)48098);
            assert(pack.rssi_GET() == (char)69);
            assert(pack.chan4_raw_GET() == (char)38841);
            assert(pack.chan8_raw_GET() == (char)35461);
            assert(pack.chan12_raw_GET() == (char)27097);
            assert(pack.chan10_raw_GET() == (char)65181);
            assert(pack.chan1_raw_GET() == (char)50851);
            assert(pack.chan7_raw_GET() == (char)28379);
            assert(pack.chan5_raw_GET() == (char)35609);
            assert(pack.time_usec_GET() == 554972814820273704L);
            assert(pack.chan6_raw_GET() == (char)49063);
            assert(pack.chan2_raw_GET() == (char)5814);
            assert(pack.chan3_raw_GET() == (char)37486);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan5_raw_SET((char)35609) ;
        p92.rssi_SET((char)69) ;
        p92.chan7_raw_SET((char)28379) ;
        p92.chan1_raw_SET((char)50851) ;
        p92.chan9_raw_SET((char)30201) ;
        p92.time_usec_SET(554972814820273704L) ;
        p92.chan11_raw_SET((char)48098) ;
        p92.chan2_raw_SET((char)5814) ;
        p92.chan10_raw_SET((char)65181) ;
        p92.chan4_raw_SET((char)38841) ;
        p92.chan8_raw_SET((char)35461) ;
        p92.chan12_raw_SET((char)27097) ;
        p92.chan6_raw_SET((char)49063) ;
        p92.chan3_raw_SET((char)37486) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_AUTO_ARMED);
            assert(pack.flags_GET() == 8714324058748267733L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.2172026E38F, -1.7679598E38F, -1.3615167E38F, 2.503426E38F, -1.7174305E38F, 1.1628955E38F, 1.9316943E38F, -2.0441693E38F, 2.8875691E38F, -2.3024587E38F, -1.526358E38F, 2.5846436E38F, -1.4308906E38F, -2.147611E38F, -2.2573997E38F, 7.32739E37F}));
            assert(pack.time_usec_GET() == 5673583987555008193L);
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED) ;
        p93.time_usec_SET(5673583987555008193L) ;
        p93.controls_SET(new float[] {1.2172026E38F, -1.7679598E38F, -1.3615167E38F, 2.503426E38F, -1.7174305E38F, 1.1628955E38F, 1.9316943E38F, -2.0441693E38F, 2.8875691E38F, -2.3024587E38F, -1.526358E38F, 2.5846436E38F, -1.4308906E38F, -2.147611E38F, -2.2573997E38F, 7.32739E37F}, 0) ;
        p93.flags_SET(8714324058748267733L) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.quality_GET() == (char)245);
            assert(pack.flow_y_GET() == (short) -10079);
            assert(pack.time_usec_GET() == 959987779290066809L);
            assert(pack.flow_rate_y_TRY(ph) == 7.734992E37F);
            assert(pack.ground_distance_GET() == -2.408274E38F);
            assert(pack.flow_comp_m_y_GET() == -2.4545062E38F);
            assert(pack.flow_rate_x_TRY(ph) == -3.1063202E38F);
            assert(pack.flow_comp_m_x_GET() == -2.5894538E38F);
            assert(pack.sensor_id_GET() == (char)186);
            assert(pack.flow_x_GET() == (short) -29435);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.flow_comp_m_x_SET(-2.5894538E38F) ;
        p100.sensor_id_SET((char)186) ;
        p100.quality_SET((char)245) ;
        p100.flow_comp_m_y_SET(-2.4545062E38F) ;
        p100.flow_rate_y_SET(7.734992E37F, PH) ;
        p100.flow_x_SET((short) -29435) ;
        p100.ground_distance_SET(-2.408274E38F) ;
        p100.flow_rate_x_SET(-3.1063202E38F, PH) ;
        p100.flow_y_SET((short) -10079) ;
        p100.time_usec_SET(959987779290066809L) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 3.354941E37F);
            assert(pack.x_GET() == 3.6795513E37F);
            assert(pack.y_GET() == 1.6227254E37F);
            assert(pack.usec_GET() == 886265465778632970L);
            assert(pack.z_GET() == 2.6608298E37F);
            assert(pack.yaw_GET() == -5.9623566E37F);
            assert(pack.roll_GET() == -2.5008475E38F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.pitch_SET(3.354941E37F) ;
        p101.x_SET(3.6795513E37F) ;
        p101.z_SET(2.6608298E37F) ;
        p101.roll_SET(-2.5008475E38F) ;
        p101.usec_SET(886265465778632970L) ;
        p101.y_SET(1.6227254E37F) ;
        p101.yaw_SET(-5.9623566E37F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == 2.7946443E38F);
            assert(pack.usec_GET() == 401483103245065368L);
            assert(pack.yaw_GET() == -3.0394556E38F);
            assert(pack.y_GET() == -1.3146114E38F);
            assert(pack.x_GET() == -1.8039351E37F);
            assert(pack.z_GET() == 7.1703267E37F);
            assert(pack.pitch_GET() == 9.761242E37F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.z_SET(7.1703267E37F) ;
        p102.yaw_SET(-3.0394556E38F) ;
        p102.roll_SET(2.7946443E38F) ;
        p102.y_SET(-1.3146114E38F) ;
        p102.usec_SET(401483103245065368L) ;
        p102.pitch_SET(9.761242E37F) ;
        p102.x_SET(-1.8039351E37F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 3.1727832E38F);
            assert(pack.usec_GET() == 4060630365942285656L);
            assert(pack.x_GET() == 2.1863918E37F);
            assert(pack.z_GET() == 1.9425583E38F);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.x_SET(2.1863918E37F) ;
        p103.usec_SET(4060630365942285656L) ;
        p103.z_SET(1.9425583E38F) ;
        p103.y_SET(3.1727832E38F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 792767562136345800L);
            assert(pack.y_GET() == 2.235956E38F);
            assert(pack.z_GET() == 1.4216328E38F);
            assert(pack.pitch_GET() == 1.4001468E38F);
            assert(pack.x_GET() == 7.795133E37F);
            assert(pack.yaw_GET() == 2.654037E38F);
            assert(pack.roll_GET() == 2.3419027E38F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.roll_SET(2.3419027E38F) ;
        p104.usec_SET(792767562136345800L) ;
        p104.y_SET(2.235956E38F) ;
        p104.z_SET(1.4216328E38F) ;
        p104.pitch_SET(1.4001468E38F) ;
        p104.yaw_SET(2.654037E38F) ;
        p104.x_SET(7.795133E37F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.abs_pressure_GET() == 2.392474E38F);
            assert(pack.diff_pressure_GET() == 1.8284531E38F);
            assert(pack.xacc_GET() == -2.9023847E37F);
            assert(pack.xgyro_GET() == 2.6405057E38F);
            assert(pack.fields_updated_GET() == (char)11922);
            assert(pack.time_usec_GET() == 8311990522636231285L);
            assert(pack.temperature_GET() == -9.939222E36F);
            assert(pack.zmag_GET() == 1.137694E38F);
            assert(pack.ymag_GET() == -1.4593723E38F);
            assert(pack.zacc_GET() == -2.7779255E38F);
            assert(pack.xmag_GET() == -3.2481839E38F);
            assert(pack.yacc_GET() == -4.921245E37F);
            assert(pack.zgyro_GET() == 2.9651502E38F);
            assert(pack.pressure_alt_GET() == 2.4194199E38F);
            assert(pack.ygyro_GET() == -1.572991E37F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.temperature_SET(-9.939222E36F) ;
        p105.ygyro_SET(-1.572991E37F) ;
        p105.zgyro_SET(2.9651502E38F) ;
        p105.zacc_SET(-2.7779255E38F) ;
        p105.abs_pressure_SET(2.392474E38F) ;
        p105.fields_updated_SET((char)11922) ;
        p105.zmag_SET(1.137694E38F) ;
        p105.time_usec_SET(8311990522636231285L) ;
        p105.xmag_SET(-3.2481839E38F) ;
        p105.diff_pressure_SET(1.8284531E38F) ;
        p105.pressure_alt_SET(2.4194199E38F) ;
        p105.xgyro_SET(2.6405057E38F) ;
        p105.ymag_SET(-1.4593723E38F) ;
        p105.yacc_SET(-4.921245E37F) ;
        p105.xacc_SET(-2.9023847E37F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integration_time_us_GET() == 1996771989L);
            assert(pack.time_delta_distance_us_GET() == 3923232073L);
            assert(pack.distance_GET() == 1.6814339E38F);
            assert(pack.temperature_GET() == (short) -25386);
            assert(pack.sensor_id_GET() == (char)21);
            assert(pack.integrated_zgyro_GET() == 6.842149E37F);
            assert(pack.time_usec_GET() == 4490395892730146468L);
            assert(pack.integrated_y_GET() == -2.1009315E38F);
            assert(pack.integrated_x_GET() == -2.7483722E38F);
            assert(pack.integrated_ygyro_GET() == -1.4569615E38F);
            assert(pack.quality_GET() == (char)203);
            assert(pack.integrated_xgyro_GET() == -1.7751893E38F);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integrated_y_SET(-2.1009315E38F) ;
        p106.quality_SET((char)203) ;
        p106.distance_SET(1.6814339E38F) ;
        p106.temperature_SET((short) -25386) ;
        p106.integrated_xgyro_SET(-1.7751893E38F) ;
        p106.time_delta_distance_us_SET(3923232073L) ;
        p106.integration_time_us_SET(1996771989L) ;
        p106.time_usec_SET(4490395892730146468L) ;
        p106.sensor_id_SET((char)21) ;
        p106.integrated_zgyro_SET(6.842149E37F) ;
        p106.integrated_x_SET(-2.7483722E38F) ;
        p106.integrated_ygyro_SET(-1.4569615E38F) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == 4.431815E37F);
            assert(pack.zgyro_GET() == -6.779178E37F);
            assert(pack.temperature_GET() == -9.661809E37F);
            assert(pack.zacc_GET() == -3.3563243E37F);
            assert(pack.abs_pressure_GET() == -2.9975158E38F);
            assert(pack.time_usec_GET() == 6357530402867049931L);
            assert(pack.xacc_GET() == -1.6374621E38F);
            assert(pack.pressure_alt_GET() == -1.3316374E38F);
            assert(pack.xgyro_GET() == -3.0400762E38F);
            assert(pack.zmag_GET() == 1.0394208E38F);
            assert(pack.fields_updated_GET() == 2613107610L);
            assert(pack.ygyro_GET() == 3.2033523E38F);
            assert(pack.xmag_GET() == 2.9729195E38F);
            assert(pack.ymag_GET() == -2.6870432E38F);
            assert(pack.diff_pressure_GET() == 1.6920788E38F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.zmag_SET(1.0394208E38F) ;
        p107.abs_pressure_SET(-2.9975158E38F) ;
        p107.pressure_alt_SET(-1.3316374E38F) ;
        p107.ygyro_SET(3.2033523E38F) ;
        p107.zgyro_SET(-6.779178E37F) ;
        p107.diff_pressure_SET(1.6920788E38F) ;
        p107.xacc_SET(-1.6374621E38F) ;
        p107.xgyro_SET(-3.0400762E38F) ;
        p107.temperature_SET(-9.661809E37F) ;
        p107.time_usec_SET(6357530402867049931L) ;
        p107.yacc_SET(4.431815E37F) ;
        p107.ymag_SET(-2.6870432E38F) ;
        p107.xmag_SET(2.9729195E38F) ;
        p107.zacc_SET(-3.3563243E37F) ;
        p107.fields_updated_SET(2613107610L) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 3.266278E38F);
            assert(pack.q2_GET() == -2.3922877E38F);
            assert(pack.xgyro_GET() == -8.2714643E37F);
            assert(pack.vd_GET() == -1.6668269E37F);
            assert(pack.pitch_GET() == 3.0187744E38F);
            assert(pack.vn_GET() == -2.3225754E38F);
            assert(pack.yacc_GET() == -4.9700417E37F);
            assert(pack.roll_GET() == 3.0882597E38F);
            assert(pack.xacc_GET() == 1.4807231E38F);
            assert(pack.ve_GET() == -1.9440207E38F);
            assert(pack.zgyro_GET() == 1.5757613E38F);
            assert(pack.q3_GET() == -1.9701759E38F);
            assert(pack.yaw_GET() == -3.3526953E38F);
            assert(pack.std_dev_horz_GET() == -2.6252435E38F);
            assert(pack.ygyro_GET() == 2.4273106E38F);
            assert(pack.q4_GET() == -6.792203E36F);
            assert(pack.q1_GET() == 3.1403416E36F);
            assert(pack.zacc_GET() == 2.9948899E38F);
            assert(pack.lat_GET() == 2.453341E38F);
            assert(pack.std_dev_vert_GET() == 1.8531798E38F);
            assert(pack.lon_GET() == 1.623334E38F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.yaw_SET(-3.3526953E38F) ;
        p108.zacc_SET(2.9948899E38F) ;
        p108.q4_SET(-6.792203E36F) ;
        p108.lon_SET(1.623334E38F) ;
        p108.q1_SET(3.1403416E36F) ;
        p108.vd_SET(-1.6668269E37F) ;
        p108.lat_SET(2.453341E38F) ;
        p108.xgyro_SET(-8.2714643E37F) ;
        p108.q3_SET(-1.9701759E38F) ;
        p108.pitch_SET(3.0187744E38F) ;
        p108.xacc_SET(1.4807231E38F) ;
        p108.zgyro_SET(1.5757613E38F) ;
        p108.roll_SET(3.0882597E38F) ;
        p108.std_dev_vert_SET(1.8531798E38F) ;
        p108.ve_SET(-1.9440207E38F) ;
        p108.yacc_SET(-4.9700417E37F) ;
        p108.q2_SET(-2.3922877E38F) ;
        p108.std_dev_horz_SET(-2.6252435E38F) ;
        p108.ygyro_SET(2.4273106E38F) ;
        p108.alt_SET(3.266278E38F) ;
        p108.vn_SET(-2.3225754E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.rssi_GET() == (char)83);
            assert(pack.txbuf_GET() == (char)254);
            assert(pack.remnoise_GET() == (char)81);
            assert(pack.fixed__GET() == (char)33938);
            assert(pack.rxerrors_GET() == (char)53489);
            assert(pack.remrssi_GET() == (char)202);
            assert(pack.noise_GET() == (char)119);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.remnoise_SET((char)81) ;
        p109.rssi_SET((char)83) ;
        p109.txbuf_SET((char)254) ;
        p109.remrssi_SET((char)202) ;
        p109.noise_SET((char)119) ;
        p109.rxerrors_SET((char)53489) ;
        p109.fixed__SET((char)33938) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)180);
            assert(pack.target_system_GET() == (char)233);
            assert(pack.target_component_GET() == (char)115);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)113, (char)96, (char)129, (char)205, (char)100, (char)136, (char)160, (char)21, (char)174, (char)110, (char)9, (char)211, (char)94, (char)211, (char)76, (char)235, (char)143, (char)226, (char)247, (char)34, (char)26, (char)68, (char)78, (char)22, (char)41, (char)30, (char)156, (char)87, (char)78, (char)212, (char)183, (char)229, (char)184, (char)97, (char)147, (char)17, (char)205, (char)9, (char)196, (char)42, (char)220, (char)26, (char)70, (char)149, (char)19, (char)223, (char)142, (char)195, (char)70, (char)55, (char)121, (char)52, (char)68, (char)189, (char)164, (char)193, (char)184, (char)90, (char)223, (char)255, (char)3, (char)223, (char)207, (char)9, (char)251, (char)99, (char)7, (char)106, (char)98, (char)157, (char)153, (char)179, (char)63, (char)43, (char)72, (char)237, (char)175, (char)55, (char)196, (char)37, (char)218, (char)39, (char)184, (char)121, (char)209, (char)171, (char)100, (char)91, (char)53, (char)50, (char)38, (char)181, (char)233, (char)222, (char)193, (char)179, (char)131, (char)222, (char)137, (char)172, (char)139, (char)7, (char)104, (char)178, (char)163, (char)161, (char)150, (char)3, (char)163, (char)69, (char)192, (char)1, (char)100, (char)48, (char)176, (char)65, (char)97, (char)85, (char)59, (char)29, (char)172, (char)187, (char)180, (char)19, (char)106, (char)237, (char)202, (char)242, (char)202, (char)40, (char)1, (char)104, (char)6, (char)50, (char)193, (char)51, (char)231, (char)189, (char)110, (char)173, (char)4, (char)111, (char)145, (char)193, (char)192, (char)183, (char)230, (char)74, (char)231, (char)52, (char)33, (char)173, (char)108, (char)86, (char)53, (char)175, (char)242, (char)211, (char)77, (char)107, (char)38, (char)39, (char)244, (char)205, (char)254, (char)241, (char)71, (char)94, (char)183, (char)42, (char)255, (char)233, (char)171, (char)43, (char)102, (char)74, (char)99, (char)102, (char)69, (char)223, (char)74, (char)13, (char)49, (char)120, (char)206, (char)14, (char)153, (char)5, (char)111, (char)13, (char)191, (char)69, (char)249, (char)213, (char)181, (char)185, (char)222, (char)241, (char)180, (char)12, (char)133, (char)234, (char)55, (char)198, (char)160, (char)25, (char)73, (char)27, (char)174, (char)166, (char)15, (char)92, (char)16, (char)217, (char)101, (char)179, (char)4, (char)157, (char)16, (char)70, (char)54, (char)106, (char)55, (char)66, (char)16, (char)87, (char)196, (char)239, (char)128, (char)190, (char)125, (char)138, (char)97, (char)81, (char)109, (char)67, (char)166, (char)221, (char)226, (char)64, (char)158, (char)64, (char)243, (char)220, (char)201, (char)188, (char)245, (char)61, (char)240, (char)179, (char)150}));
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_system_SET((char)233) ;
        p110.target_component_SET((char)115) ;
        p110.target_network_SET((char)180) ;
        p110.payload_SET(new char[] {(char)113, (char)96, (char)129, (char)205, (char)100, (char)136, (char)160, (char)21, (char)174, (char)110, (char)9, (char)211, (char)94, (char)211, (char)76, (char)235, (char)143, (char)226, (char)247, (char)34, (char)26, (char)68, (char)78, (char)22, (char)41, (char)30, (char)156, (char)87, (char)78, (char)212, (char)183, (char)229, (char)184, (char)97, (char)147, (char)17, (char)205, (char)9, (char)196, (char)42, (char)220, (char)26, (char)70, (char)149, (char)19, (char)223, (char)142, (char)195, (char)70, (char)55, (char)121, (char)52, (char)68, (char)189, (char)164, (char)193, (char)184, (char)90, (char)223, (char)255, (char)3, (char)223, (char)207, (char)9, (char)251, (char)99, (char)7, (char)106, (char)98, (char)157, (char)153, (char)179, (char)63, (char)43, (char)72, (char)237, (char)175, (char)55, (char)196, (char)37, (char)218, (char)39, (char)184, (char)121, (char)209, (char)171, (char)100, (char)91, (char)53, (char)50, (char)38, (char)181, (char)233, (char)222, (char)193, (char)179, (char)131, (char)222, (char)137, (char)172, (char)139, (char)7, (char)104, (char)178, (char)163, (char)161, (char)150, (char)3, (char)163, (char)69, (char)192, (char)1, (char)100, (char)48, (char)176, (char)65, (char)97, (char)85, (char)59, (char)29, (char)172, (char)187, (char)180, (char)19, (char)106, (char)237, (char)202, (char)242, (char)202, (char)40, (char)1, (char)104, (char)6, (char)50, (char)193, (char)51, (char)231, (char)189, (char)110, (char)173, (char)4, (char)111, (char)145, (char)193, (char)192, (char)183, (char)230, (char)74, (char)231, (char)52, (char)33, (char)173, (char)108, (char)86, (char)53, (char)175, (char)242, (char)211, (char)77, (char)107, (char)38, (char)39, (char)244, (char)205, (char)254, (char)241, (char)71, (char)94, (char)183, (char)42, (char)255, (char)233, (char)171, (char)43, (char)102, (char)74, (char)99, (char)102, (char)69, (char)223, (char)74, (char)13, (char)49, (char)120, (char)206, (char)14, (char)153, (char)5, (char)111, (char)13, (char)191, (char)69, (char)249, (char)213, (char)181, (char)185, (char)222, (char)241, (char)180, (char)12, (char)133, (char)234, (char)55, (char)198, (char)160, (char)25, (char)73, (char)27, (char)174, (char)166, (char)15, (char)92, (char)16, (char)217, (char)101, (char)179, (char)4, (char)157, (char)16, (char)70, (char)54, (char)106, (char)55, (char)66, (char)16, (char)87, (char)196, (char)239, (char)128, (char)190, (char)125, (char)138, (char)97, (char)81, (char)109, (char)67, (char)166, (char)221, (char)226, (char)64, (char)158, (char)64, (char)243, (char)220, (char)201, (char)188, (char)245, (char)61, (char)240, (char)179, (char)150}, 0) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.ts1_GET() == 5882999285390469890L);
            assert(pack.tc1_GET() == -8963752896645475559L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(5882999285390469890L) ;
        p111.tc1_SET(-8963752896645475559L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 946710170707917774L);
            assert(pack.seq_GET() == 3137395481L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(3137395481L) ;
        p112.time_usec_SET(946710170707917774L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 8537523437100865587L);
            assert(pack.epv_GET() == (char)55495);
            assert(pack.fix_type_GET() == (char)109);
            assert(pack.eph_GET() == (char)64655);
            assert(pack.vel_GET() == (char)53403);
            assert(pack.lon_GET() == 1329912159);
            assert(pack.alt_GET() == 47112422);
            assert(pack.vn_GET() == (short) -18065);
            assert(pack.lat_GET() == -316835062);
            assert(pack.cog_GET() == (char)37230);
            assert(pack.ve_GET() == (short) -1761);
            assert(pack.satellites_visible_GET() == (char)144);
            assert(pack.vd_GET() == (short)4596);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.vn_SET((short) -18065) ;
        p113.lon_SET(1329912159) ;
        p113.epv_SET((char)55495) ;
        p113.satellites_visible_SET((char)144) ;
        p113.lat_SET(-316835062) ;
        p113.vel_SET((char)53403) ;
        p113.vd_SET((short)4596) ;
        p113.fix_type_SET((char)109) ;
        p113.cog_SET((char)37230) ;
        p113.time_usec_SET(8537523437100865587L) ;
        p113.alt_SET(47112422) ;
        p113.ve_SET((short) -1761) ;
        p113.eph_SET((char)64655) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.time_delta_distance_us_GET() == 4033824793L);
            assert(pack.integrated_y_GET() == -1.1224611E38F);
            assert(pack.distance_GET() == -1.4941876E38F);
            assert(pack.integrated_ygyro_GET() == -1.4033434E38F);
            assert(pack.temperature_GET() == (short)28926);
            assert(pack.integrated_x_GET() == 1.5753721E38F);
            assert(pack.quality_GET() == (char)56);
            assert(pack.sensor_id_GET() == (char)243);
            assert(pack.integration_time_us_GET() == 330489322L);
            assert(pack.integrated_xgyro_GET() == -1.496038E38F);
            assert(pack.time_usec_GET() == 973268764830508423L);
            assert(pack.integrated_zgyro_GET() == -1.065129E38F);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.integrated_ygyro_SET(-1.4033434E38F) ;
        p114.integrated_x_SET(1.5753721E38F) ;
        p114.integrated_y_SET(-1.1224611E38F) ;
        p114.temperature_SET((short)28926) ;
        p114.sensor_id_SET((char)243) ;
        p114.quality_SET((char)56) ;
        p114.time_delta_distance_us_SET(4033824793L) ;
        p114.distance_SET(-1.4941876E38F) ;
        p114.integration_time_us_SET(330489322L) ;
        p114.integrated_xgyro_SET(-1.496038E38F) ;
        p114.integrated_zgyro_SET(-1.065129E38F) ;
        p114.time_usec_SET(973268764830508423L) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == -2.4516632E38F);
            assert(pack.alt_GET() == -793696953);
            assert(pack.vz_GET() == (short)21956);
            assert(pack.zacc_GET() == (short) -28752);
            assert(pack.lon_GET() == 653145412);
            assert(pack.true_airspeed_GET() == (char)53027);
            assert(pack.ind_airspeed_GET() == (char)29916);
            assert(pack.vx_GET() == (short) -19150);
            assert(pack.yawspeed_GET() == -2.4651636E38F);
            assert(pack.yacc_GET() == (short) -2935);
            assert(pack.pitchspeed_GET() == -2.0257294E38F);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {-1.1465013E38F, 3.2133983E38F, -4.584542E37F, -1.9941248E37F}));
            assert(pack.lat_GET() == -75531001);
            assert(pack.time_usec_GET() == 6252776209459685083L);
            assert(pack.vy_GET() == (short)2817);
            assert(pack.xacc_GET() == (short) -29668);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.time_usec_SET(6252776209459685083L) ;
        p115.vz_SET((short)21956) ;
        p115.lat_SET(-75531001) ;
        p115.yawspeed_SET(-2.4651636E38F) ;
        p115.lon_SET(653145412) ;
        p115.rollspeed_SET(-2.4516632E38F) ;
        p115.attitude_quaternion_SET(new float[] {-1.1465013E38F, 3.2133983E38F, -4.584542E37F, -1.9941248E37F}, 0) ;
        p115.pitchspeed_SET(-2.0257294E38F) ;
        p115.vy_SET((short)2817) ;
        p115.true_airspeed_SET((char)53027) ;
        p115.yacc_SET((short) -2935) ;
        p115.vx_SET((short) -19150) ;
        p115.xacc_SET((short) -29668) ;
        p115.zacc_SET((short) -28752) ;
        p115.alt_SET(-793696953) ;
        p115.ind_airspeed_SET((char)29916) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short) -14989);
            assert(pack.ygyro_GET() == (short) -28123);
            assert(pack.zacc_GET() == (short) -11814);
            assert(pack.zgyro_GET() == (short) -24983);
            assert(pack.xmag_GET() == (short)10205);
            assert(pack.time_boot_ms_GET() == 602237874L);
            assert(pack.xgyro_GET() == (short) -142);
            assert(pack.ymag_GET() == (short)3676);
            assert(pack.xacc_GET() == (short) -23864);
            assert(pack.zmag_GET() == (short)15274);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.xmag_SET((short)10205) ;
        p116.xacc_SET((short) -23864) ;
        p116.zmag_SET((short)15274) ;
        p116.ygyro_SET((short) -28123) ;
        p116.yacc_SET((short) -14989) ;
        p116.zacc_SET((short) -11814) ;
        p116.ymag_SET((short)3676) ;
        p116.zgyro_SET((short) -24983) ;
        p116.xgyro_SET((short) -142) ;
        p116.time_boot_ms_SET(602237874L) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)124);
            assert(pack.start_GET() == (char)45686);
            assert(pack.end_GET() == (char)25326);
            assert(pack.target_system_GET() == (char)94);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.start_SET((char)45686) ;
        p117.end_SET((char)25326) ;
        p117.target_system_SET((char)94) ;
        p117.target_component_SET((char)124) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)43720);
            assert(pack.size_GET() == 1349049226L);
            assert(pack.last_log_num_GET() == (char)54495);
            assert(pack.time_utc_GET() == 1630582128L);
            assert(pack.num_logs_GET() == (char)33163);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.last_log_num_SET((char)54495) ;
        p118.id_SET((char)43720) ;
        p118.time_utc_SET(1630582128L) ;
        p118.num_logs_SET((char)33163) ;
        p118.size_SET(1349049226L) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)35);
            assert(pack.target_system_GET() == (char)84);
            assert(pack.ofs_GET() == 3381361523L);
            assert(pack.count_GET() == 2068868890L);
            assert(pack.id_GET() == (char)20462);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.count_SET(2068868890L) ;
        p119.target_component_SET((char)35) ;
        p119.id_SET((char)20462) ;
        p119.target_system_SET((char)84) ;
        p119.ofs_SET(3381361523L) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)235);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)158, (char)30, (char)206, (char)11, (char)240, (char)25, (char)143, (char)75, (char)63, (char)193, (char)239, (char)135, (char)158, (char)80, (char)135, (char)92, (char)178, (char)107, (char)250, (char)61, (char)49, (char)240, (char)102, (char)231, (char)211, (char)99, (char)147, (char)47, (char)148, (char)41, (char)180, (char)234, (char)122, (char)103, (char)65, (char)188, (char)6, (char)234, (char)124, (char)132, (char)227, (char)127, (char)157, (char)107, (char)60, (char)110, (char)210, (char)38, (char)48, (char)183, (char)188, (char)74, (char)212, (char)219, (char)122, (char)164, (char)250, (char)109, (char)104, (char)22, (char)64, (char)111, (char)53, (char)65, (char)121, (char)23, (char)246, (char)34, (char)138, (char)32, (char)183, (char)209, (char)198, (char)78, (char)64, (char)2, (char)18, (char)150, (char)215, (char)13, (char)73, (char)171, (char)16, (char)22, (char)11, (char)126, (char)3, (char)129, (char)228, (char)183}));
            assert(pack.id_GET() == (char)11534);
            assert(pack.ofs_GET() == 2503475646L);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)11534) ;
        p120.data__SET(new char[] {(char)158, (char)30, (char)206, (char)11, (char)240, (char)25, (char)143, (char)75, (char)63, (char)193, (char)239, (char)135, (char)158, (char)80, (char)135, (char)92, (char)178, (char)107, (char)250, (char)61, (char)49, (char)240, (char)102, (char)231, (char)211, (char)99, (char)147, (char)47, (char)148, (char)41, (char)180, (char)234, (char)122, (char)103, (char)65, (char)188, (char)6, (char)234, (char)124, (char)132, (char)227, (char)127, (char)157, (char)107, (char)60, (char)110, (char)210, (char)38, (char)48, (char)183, (char)188, (char)74, (char)212, (char)219, (char)122, (char)164, (char)250, (char)109, (char)104, (char)22, (char)64, (char)111, (char)53, (char)65, (char)121, (char)23, (char)246, (char)34, (char)138, (char)32, (char)183, (char)209, (char)198, (char)78, (char)64, (char)2, (char)18, (char)150, (char)215, (char)13, (char)73, (char)171, (char)16, (char)22, (char)11, (char)126, (char)3, (char)129, (char)228, (char)183}, 0) ;
        p120.ofs_SET(2503475646L) ;
        p120.count_SET((char)235) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)120);
            assert(pack.target_component_GET() == (char)143);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_component_SET((char)143) ;
        p121.target_system_SET((char)120) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)105);
            assert(pack.target_system_GET() == (char)218);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)218) ;
        p122.target_component_SET((char)105) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)205);
            assert(pack.target_component_GET() == (char)246);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)220, (char)120, (char)188, (char)59, (char)10, (char)121, (char)153, (char)44, (char)164, (char)170, (char)174, (char)198, (char)137, (char)130, (char)220, (char)251, (char)176, (char)112, (char)217, (char)250, (char)94, (char)8, (char)31, (char)214, (char)101, (char)22, (char)215, (char)244, (char)137, (char)81, (char)2, (char)87, (char)50, (char)246, (char)19, (char)99, (char)191, (char)104, (char)152, (char)65, (char)166, (char)129, (char)90, (char)222, (char)30, (char)147, (char)200, (char)80, (char)93, (char)238, (char)50, (char)208, (char)70, (char)80, (char)90, (char)215, (char)76, (char)229, (char)171, (char)30, (char)129, (char)5, (char)10, (char)184, (char)18, (char)56, (char)121, (char)205, (char)10, (char)222, (char)43, (char)182, (char)113, (char)30, (char)127, (char)49, (char)132, (char)178, (char)192, (char)161, (char)183, (char)110, (char)155, (char)235, (char)100, (char)126, (char)198, (char)221, (char)63, (char)161, (char)48, (char)95, (char)119, (char)177, (char)41, (char)240, (char)149, (char)3, (char)193, (char)165, (char)221, (char)241, (char)65, (char)149, (char)17, (char)14, (char)32, (char)36, (char)55, (char)165}));
            assert(pack.len_GET() == (char)88);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.len_SET((char)88) ;
        p123.target_system_SET((char)205) ;
        p123.target_component_SET((char)246) ;
        p123.data__SET(new char[] {(char)220, (char)120, (char)188, (char)59, (char)10, (char)121, (char)153, (char)44, (char)164, (char)170, (char)174, (char)198, (char)137, (char)130, (char)220, (char)251, (char)176, (char)112, (char)217, (char)250, (char)94, (char)8, (char)31, (char)214, (char)101, (char)22, (char)215, (char)244, (char)137, (char)81, (char)2, (char)87, (char)50, (char)246, (char)19, (char)99, (char)191, (char)104, (char)152, (char)65, (char)166, (char)129, (char)90, (char)222, (char)30, (char)147, (char)200, (char)80, (char)93, (char)238, (char)50, (char)208, (char)70, (char)80, (char)90, (char)215, (char)76, (char)229, (char)171, (char)30, (char)129, (char)5, (char)10, (char)184, (char)18, (char)56, (char)121, (char)205, (char)10, (char)222, (char)43, (char)182, (char)113, (char)30, (char)127, (char)49, (char)132, (char)178, (char)192, (char)161, (char)183, (char)110, (char)155, (char)235, (char)100, (char)126, (char)198, (char)221, (char)63, (char)161, (char)48, (char)95, (char)119, (char)177, (char)41, (char)240, (char)149, (char)3, (char)193, (char)165, (char)221, (char)241, (char)65, (char)149, (char)17, (char)14, (char)32, (char)36, (char)55, (char)165}, 0) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.cog_GET() == (char)50287);
            assert(pack.lon_GET() == -20303927);
            assert(pack.dgps_numch_GET() == (char)200);
            assert(pack.dgps_age_GET() == 603623504L);
            assert(pack.time_usec_GET() == 2474705468183667136L);
            assert(pack.lat_GET() == -1491126158);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.eph_GET() == (char)2702);
            assert(pack.alt_GET() == -1374004404);
            assert(pack.epv_GET() == (char)20668);
            assert(pack.vel_GET() == (char)14339);
            assert(pack.satellites_visible_GET() == (char)232);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.lon_SET(-20303927) ;
        p124.time_usec_SET(2474705468183667136L) ;
        p124.epv_SET((char)20668) ;
        p124.eph_SET((char)2702) ;
        p124.dgps_numch_SET((char)200) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p124.lat_SET(-1491126158) ;
        p124.vel_SET((char)14339) ;
        p124.dgps_age_SET(603623504L) ;
        p124.alt_SET(-1374004404) ;
        p124.cog_SET((char)50287) ;
        p124.satellites_visible_SET((char)232) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT);
            assert(pack.Vcc_GET() == (char)17605);
            assert(pack.Vservo_GET() == (char)315);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)17605) ;
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT) ;
        p125.Vservo_SET((char)315) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2);
            assert(pack.baudrate_GET() == 2224047385L);
            assert(pack.flags_GET() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND);
            assert(pack.timeout_GET() == (char)10530);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)155, (char)193, (char)168, (char)75, (char)177, (char)63, (char)109, (char)10, (char)203, (char)213, (char)233, (char)30, (char)82, (char)4, (char)7, (char)1, (char)156, (char)53, (char)130, (char)201, (char)4, (char)58, (char)100, (char)17, (char)213, (char)166, (char)137, (char)196, (char)81, (char)177, (char)201, (char)98, (char)16, (char)0, (char)9, (char)154, (char)56, (char)252, (char)194, (char)95, (char)32, (char)9, (char)145, (char)22, (char)47, (char)36, (char)153, (char)28, (char)105, (char)235, (char)18, (char)170, (char)116, (char)240, (char)116, (char)181, (char)56, (char)220, (char)18, (char)172, (char)11, (char)61, (char)147, (char)145, (char)46, (char)244, (char)131, (char)52, (char)139, (char)116}));
            assert(pack.count_GET() == (char)24);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.count_SET((char)24) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2) ;
        p126.timeout_SET((char)10530) ;
        p126.baudrate_SET(2224047385L) ;
        p126.data__SET(new char[] {(char)155, (char)193, (char)168, (char)75, (char)177, (char)63, (char)109, (char)10, (char)203, (char)213, (char)233, (char)30, (char)82, (char)4, (char)7, (char)1, (char)156, (char)53, (char)130, (char)201, (char)4, (char)58, (char)100, (char)17, (char)213, (char)166, (char)137, (char)196, (char)81, (char)177, (char)201, (char)98, (char)16, (char)0, (char)9, (char)154, (char)56, (char)252, (char)194, (char)95, (char)32, (char)9, (char)145, (char)22, (char)47, (char)36, (char)153, (char)28, (char)105, (char)235, (char)18, (char)170, (char)116, (char)240, (char)116, (char)181, (char)56, (char)220, (char)18, (char)172, (char)11, (char)61, (char)147, (char)145, (char)46, (char)244, (char)131, (char)52, (char)139, (char)116}, 0) ;
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.wn_GET() == (char)58043);
            assert(pack.nsats_GET() == (char)75);
            assert(pack.baseline_c_mm_GET() == -238711323);
            assert(pack.rtk_receiver_id_GET() == (char)144);
            assert(pack.time_last_baseline_ms_GET() == 1095958223L);
            assert(pack.baseline_coords_type_GET() == (char)26);
            assert(pack.baseline_b_mm_GET() == -523823644);
            assert(pack.iar_num_hypotheses_GET() == -69439549);
            assert(pack.baseline_a_mm_GET() == 514806395);
            assert(pack.rtk_rate_GET() == (char)214);
            assert(pack.tow_GET() == 3790849871L);
            assert(pack.rtk_health_GET() == (char)187);
            assert(pack.accuracy_GET() == 146432868L);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.nsats_SET((char)75) ;
        p127.time_last_baseline_ms_SET(1095958223L) ;
        p127.wn_SET((char)58043) ;
        p127.rtk_rate_SET((char)214) ;
        p127.baseline_c_mm_SET(-238711323) ;
        p127.baseline_b_mm_SET(-523823644) ;
        p127.baseline_coords_type_SET((char)26) ;
        p127.rtk_health_SET((char)187) ;
        p127.iar_num_hypotheses_SET(-69439549) ;
        p127.baseline_a_mm_SET(514806395) ;
        p127.accuracy_SET(146432868L) ;
        p127.tow_SET(3790849871L) ;
        p127.rtk_receiver_id_SET((char)144) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.rtk_health_GET() == (char)50);
            assert(pack.tow_GET() == 3764379633L);
            assert(pack.wn_GET() == (char)30189);
            assert(pack.baseline_coords_type_GET() == (char)213);
            assert(pack.baseline_b_mm_GET() == -1157255062);
            assert(pack.time_last_baseline_ms_GET() == 1975785395L);
            assert(pack.baseline_c_mm_GET() == 323721462);
            assert(pack.baseline_a_mm_GET() == -559384715);
            assert(pack.nsats_GET() == (char)6);
            assert(pack.accuracy_GET() == 2780650622L);
            assert(pack.iar_num_hypotheses_GET() == -1348801373);
            assert(pack.rtk_rate_GET() == (char)69);
            assert(pack.rtk_receiver_id_GET() == (char)12);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.baseline_coords_type_SET((char)213) ;
        p128.rtk_rate_SET((char)69) ;
        p128.tow_SET(3764379633L) ;
        p128.accuracy_SET(2780650622L) ;
        p128.iar_num_hypotheses_SET(-1348801373) ;
        p128.rtk_receiver_id_SET((char)12) ;
        p128.baseline_b_mm_SET(-1157255062) ;
        p128.rtk_health_SET((char)50) ;
        p128.wn_SET((char)30189) ;
        p128.time_last_baseline_ms_SET(1975785395L) ;
        p128.baseline_a_mm_SET(-559384715) ;
        p128.nsats_SET((char)6) ;
        p128.baseline_c_mm_SET(323721462) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short) -28914);
            assert(pack.zacc_GET() == (short) -5066);
            assert(pack.xgyro_GET() == (short)22726);
            assert(pack.xmag_GET() == (short)29796);
            assert(pack.time_boot_ms_GET() == 3847723101L);
            assert(pack.xacc_GET() == (short)10329);
            assert(pack.zgyro_GET() == (short) -2491);
            assert(pack.ymag_GET() == (short)23755);
            assert(pack.ygyro_GET() == (short) -4816);
            assert(pack.zmag_GET() == (short) -4387);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.xmag_SET((short)29796) ;
        p129.zacc_SET((short) -5066) ;
        p129.yacc_SET((short) -28914) ;
        p129.xacc_SET((short)10329) ;
        p129.zgyro_SET((short) -2491) ;
        p129.zmag_SET((short) -4387) ;
        p129.ygyro_SET((short) -4816) ;
        p129.ymag_SET((short)23755) ;
        p129.time_boot_ms_SET(3847723101L) ;
        p129.xgyro_SET((short)22726) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.packets_GET() == (char)2050);
            assert(pack.type_GET() == (char)83);
            assert(pack.width_GET() == (char)56924);
            assert(pack.size_GET() == 3166946911L);
            assert(pack.payload_GET() == (char)40);
            assert(pack.height_GET() == (char)20389);
            assert(pack.jpg_quality_GET() == (char)77);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.packets_SET((char)2050) ;
        p130.jpg_quality_SET((char)77) ;
        p130.size_SET(3166946911L) ;
        p130.height_SET((char)20389) ;
        p130.payload_SET((char)40) ;
        p130.type_SET((char)83) ;
        p130.width_SET((char)56924) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)64721);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)217, (char)142, (char)145, (char)42, (char)68, (char)109, (char)116, (char)227, (char)21, (char)188, (char)249, (char)32, (char)202, (char)111, (char)65, (char)247, (char)176, (char)17, (char)191, (char)179, (char)103, (char)239, (char)130, (char)90, (char)78, (char)167, (char)221, (char)71, (char)253, (char)202, (char)246, (char)141, (char)12, (char)164, (char)149, (char)76, (char)56, (char)196, (char)100, (char)77, (char)188, (char)43, (char)109, (char)14, (char)107, (char)124, (char)46, (char)213, (char)162, (char)235, (char)15, (char)66, (char)188, (char)61, (char)44, (char)0, (char)252, (char)127, (char)193, (char)188, (char)107, (char)200, (char)34, (char)240, (char)241, (char)35, (char)227, (char)51, (char)174, (char)54, (char)90, (char)98, (char)191, (char)234, (char)21, (char)47, (char)24, (char)129, (char)70, (char)125, (char)184, (char)209, (char)0, (char)75, (char)182, (char)202, (char)131, (char)248, (char)1, (char)238, (char)50, (char)231, (char)217, (char)55, (char)33, (char)31, (char)117, (char)55, (char)74, (char)11, (char)189, (char)214, (char)62, (char)182, (char)253, (char)67, (char)34, (char)147, (char)100, (char)27, (char)253, (char)22, (char)139, (char)247, (char)169, (char)157, (char)238, (char)245, (char)38, (char)230, (char)159, (char)64, (char)140, (char)20, (char)115, (char)225, (char)58, (char)50, (char)110, (char)42, (char)114, (char)181, (char)71, (char)247, (char)19, (char)62, (char)137, (char)210, (char)53, (char)221, (char)120, (char)156, (char)154, (char)162, (char)143, (char)62, (char)32, (char)129, (char)7, (char)35, (char)2, (char)217, (char)167, (char)153, (char)22, (char)243, (char)203, (char)57, (char)102, (char)130, (char)220, (char)237, (char)32, (char)203, (char)1, (char)11, (char)156, (char)55, (char)70, (char)104, (char)167, (char)193, (char)161, (char)218, (char)182, (char)173, (char)165, (char)52, (char)188, (char)241, (char)11, (char)161, (char)90, (char)210, (char)31, (char)241, (char)64, (char)36, (char)177, (char)99, (char)220, (char)234, (char)89, (char)221, (char)146, (char)55, (char)191, (char)141, (char)5, (char)204, (char)137, (char)211, (char)46, (char)44, (char)66, (char)7, (char)34, (char)91, (char)97, (char)99, (char)90, (char)87, (char)91, (char)164, (char)149, (char)173, (char)47, (char)123, (char)44, (char)49, (char)102, (char)211, (char)175, (char)175, (char)143, (char)151, (char)72, (char)209, (char)3, (char)153, (char)53, (char)179, (char)104, (char)246, (char)30, (char)210, (char)176, (char)8, (char)3, (char)21, (char)28, (char)85, (char)253, (char)192, (char)14, (char)105, (char)124, (char)173, (char)85, (char)196, (char)93, (char)197, (char)58}));
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)64721) ;
        p131.data__SET(new char[] {(char)217, (char)142, (char)145, (char)42, (char)68, (char)109, (char)116, (char)227, (char)21, (char)188, (char)249, (char)32, (char)202, (char)111, (char)65, (char)247, (char)176, (char)17, (char)191, (char)179, (char)103, (char)239, (char)130, (char)90, (char)78, (char)167, (char)221, (char)71, (char)253, (char)202, (char)246, (char)141, (char)12, (char)164, (char)149, (char)76, (char)56, (char)196, (char)100, (char)77, (char)188, (char)43, (char)109, (char)14, (char)107, (char)124, (char)46, (char)213, (char)162, (char)235, (char)15, (char)66, (char)188, (char)61, (char)44, (char)0, (char)252, (char)127, (char)193, (char)188, (char)107, (char)200, (char)34, (char)240, (char)241, (char)35, (char)227, (char)51, (char)174, (char)54, (char)90, (char)98, (char)191, (char)234, (char)21, (char)47, (char)24, (char)129, (char)70, (char)125, (char)184, (char)209, (char)0, (char)75, (char)182, (char)202, (char)131, (char)248, (char)1, (char)238, (char)50, (char)231, (char)217, (char)55, (char)33, (char)31, (char)117, (char)55, (char)74, (char)11, (char)189, (char)214, (char)62, (char)182, (char)253, (char)67, (char)34, (char)147, (char)100, (char)27, (char)253, (char)22, (char)139, (char)247, (char)169, (char)157, (char)238, (char)245, (char)38, (char)230, (char)159, (char)64, (char)140, (char)20, (char)115, (char)225, (char)58, (char)50, (char)110, (char)42, (char)114, (char)181, (char)71, (char)247, (char)19, (char)62, (char)137, (char)210, (char)53, (char)221, (char)120, (char)156, (char)154, (char)162, (char)143, (char)62, (char)32, (char)129, (char)7, (char)35, (char)2, (char)217, (char)167, (char)153, (char)22, (char)243, (char)203, (char)57, (char)102, (char)130, (char)220, (char)237, (char)32, (char)203, (char)1, (char)11, (char)156, (char)55, (char)70, (char)104, (char)167, (char)193, (char)161, (char)218, (char)182, (char)173, (char)165, (char)52, (char)188, (char)241, (char)11, (char)161, (char)90, (char)210, (char)31, (char)241, (char)64, (char)36, (char)177, (char)99, (char)220, (char)234, (char)89, (char)221, (char)146, (char)55, (char)191, (char)141, (char)5, (char)204, (char)137, (char)211, (char)46, (char)44, (char)66, (char)7, (char)34, (char)91, (char)97, (char)99, (char)90, (char)87, (char)91, (char)164, (char)149, (char)173, (char)47, (char)123, (char)44, (char)49, (char)102, (char)211, (char)175, (char)175, (char)143, (char)151, (char)72, (char)209, (char)3, (char)153, (char)53, (char)179, (char)104, (char)246, (char)30, (char)210, (char)176, (char)8, (char)3, (char)21, (char)28, (char)85, (char)253, (char)192, (char)14, (char)105, (char)124, (char)173, (char)85, (char)196, (char)93, (char)197, (char)58}, 0) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.covariance_GET() == (char)40);
            assert(pack.max_distance_GET() == (char)58678);
            assert(pack.id_GET() == (char)100);
            assert(pack.current_distance_GET() == (char)3166);
            assert(pack.time_boot_ms_GET() == 1675568797L);
            assert(pack.min_distance_GET() == (char)14488);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_180);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.covariance_SET((char)40) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_180) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER) ;
        p132.max_distance_SET((char)58678) ;
        p132.min_distance_SET((char)14488) ;
        p132.time_boot_ms_SET(1675568797L) ;
        p132.current_distance_SET((char)3166) ;
        p132.id_SET((char)100) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -1214608913);
            assert(pack.lat_GET() == 789695992);
            assert(pack.grid_spacing_GET() == (char)64613);
            assert(pack.mask_GET() == 2826026590920731996L);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.grid_spacing_SET((char)64613) ;
        p133.lat_SET(789695992) ;
        p133.lon_SET(-1214608913) ;
        p133.mask_SET(2826026590920731996L) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.grid_spacing_GET() == (char)54468);
            assert(pack.lat_GET() == -1012536311);
            assert(pack.gridbit_GET() == (char)31);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -18948, (short)27329, (short)5777, (short) -6984, (short)13594, (short) -7620, (short)30199, (short)1344, (short)20439, (short)17271, (short) -10831, (short) -7226, (short)4183, (short)156, (short) -25523, (short)13257}));
            assert(pack.lon_GET() == 854602397);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.data__SET(new short[] {(short) -18948, (short)27329, (short)5777, (short) -6984, (short)13594, (short) -7620, (short)30199, (short)1344, (short)20439, (short)17271, (short) -10831, (short) -7226, (short)4183, (short)156, (short) -25523, (short)13257}, 0) ;
        p134.gridbit_SET((char)31) ;
        p134.lon_SET(854602397) ;
        p134.lat_SET(-1012536311) ;
        p134.grid_spacing_SET((char)54468) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1720834789);
            assert(pack.lon_GET() == -1741539085);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(-1720834789) ;
        p135.lon_SET(-1741539085) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1494242750);
            assert(pack.pending_GET() == (char)25888);
            assert(pack.spacing_GET() == (char)59752);
            assert(pack.lon_GET() == -1595694020);
            assert(pack.terrain_height_GET() == -3.3821451E38F);
            assert(pack.loaded_GET() == (char)4547);
            assert(pack.current_height_GET() == 2.551028E38F);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lat_SET(-1494242750) ;
        p136.current_height_SET(2.551028E38F) ;
        p136.lon_SET(-1595694020) ;
        p136.spacing_SET((char)59752) ;
        p136.pending_SET((char)25888) ;
        p136.loaded_SET((char)4547) ;
        p136.terrain_height_SET(-3.3821451E38F) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == 7.8488843E37F);
            assert(pack.press_abs_GET() == -2.2717343E38F);
            assert(pack.temperature_GET() == (short) -30524);
            assert(pack.time_boot_ms_GET() == 315169665L);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.press_diff_SET(7.8488843E37F) ;
        p137.temperature_SET((short) -30524) ;
        p137.press_abs_SET(-2.2717343E38F) ;
        p137.time_boot_ms_SET(315169665L) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.6499233E38F, -3.18302E38F, -1.0269433E38F, 2.9575418E38F}));
            assert(pack.y_GET() == 6.9760886E37F);
            assert(pack.time_usec_GET() == 7360806333562946055L);
            assert(pack.x_GET() == -2.5804693E38F);
            assert(pack.z_GET() == 2.3186625E37F);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.x_SET(-2.5804693E38F) ;
        p138.z_SET(2.3186625E37F) ;
        p138.q_SET(new float[] {-1.6499233E38F, -3.18302E38F, -1.0269433E38F, 2.9575418E38F}, 0) ;
        p138.y_SET(6.9760886E37F) ;
        p138.time_usec_SET(7360806333562946055L) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-2.0693794E38F, 1.8190555E38F, -1.916215E38F, -2.4963367E38F, -1.9637248E38F, -1.0011147E38F, -7.229763E37F, -3.0799656E38F}));
            assert(pack.target_system_GET() == (char)139);
            assert(pack.time_usec_GET() == 5175778968312934249L);
            assert(pack.target_component_GET() == (char)220);
            assert(pack.group_mlx_GET() == (char)33);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.target_component_SET((char)220) ;
        p139.group_mlx_SET((char)33) ;
        p139.time_usec_SET(5175778968312934249L) ;
        p139.target_system_SET((char)139) ;
        p139.controls_SET(new float[] {-2.0693794E38F, 1.8190555E38F, -1.916215E38F, -2.4963367E38F, -1.9637248E38F, -1.0011147E38F, -7.229763E37F, -3.0799656E38F}, 0) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.4942899E38F, 3.352465E38F, -1.9271437E38F, 1.1422289E37F, 5.787111E37F, 3.1070915E38F, -2.094637E38F, 7.514821E37F}));
            assert(pack.group_mlx_GET() == (char)105);
            assert(pack.time_usec_GET() == 901623473891179931L);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.group_mlx_SET((char)105) ;
        p140.time_usec_SET(901623473891179931L) ;
        p140.controls_SET(new float[] {1.4942899E38F, 3.352465E38F, -1.9271437E38F, 1.1422289E37F, 5.787111E37F, 3.1070915E38F, -2.094637E38F, 7.514821E37F}, 0) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_amsl_GET() == 1.02929153E37F);
            assert(pack.altitude_local_GET() == 3.85113E37F);
            assert(pack.time_usec_GET() == 6751003256593142134L);
            assert(pack.altitude_terrain_GET() == 7.4951236E37F);
            assert(pack.bottom_clearance_GET() == -2.4464333E37F);
            assert(pack.altitude_relative_GET() == 4.0716022E37F);
            assert(pack.altitude_monotonic_GET() == -1.1909347E38F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_relative_SET(4.0716022E37F) ;
        p141.bottom_clearance_SET(-2.4464333E37F) ;
        p141.altitude_amsl_SET(1.02929153E37F) ;
        p141.altitude_local_SET(3.85113E37F) ;
        p141.time_usec_SET(6751003256593142134L) ;
        p141.altitude_terrain_SET(7.4951236E37F) ;
        p141.altitude_monotonic_SET(-1.1909347E38F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)182, (char)76, (char)116, (char)125, (char)189, (char)82, (char)203, (char)245, (char)14, (char)175, (char)238, (char)92, (char)146, (char)41, (char)127, (char)209, (char)57, (char)222, (char)222, (char)67, (char)69, (char)156, (char)235, (char)93, (char)85, (char)93, (char)164, (char)124, (char)96, (char)192, (char)229, (char)64, (char)47, (char)226, (char)149, (char)9, (char)101, (char)78, (char)184, (char)62, (char)206, (char)203, (char)126, (char)198, (char)215, (char)197, (char)195, (char)52, (char)9, (char)177, (char)151, (char)203, (char)220, (char)182, (char)252, (char)149, (char)249, (char)77, (char)18, (char)219, (char)206, (char)35, (char)151, (char)149, (char)209, (char)93, (char)222, (char)221, (char)130, (char)252, (char)221, (char)214, (char)97, (char)40, (char)104, (char)76, (char)181, (char)72, (char)136, (char)156, (char)87, (char)25, (char)162, (char)15, (char)56, (char)180, (char)20, (char)150, (char)110, (char)228, (char)227, (char)76, (char)68, (char)162, (char)170, (char)141, (char)50, (char)140, (char)76, (char)207, (char)129, (char)92, (char)187, (char)199, (char)168, (char)134, (char)161, (char)37, (char)89, (char)250, (char)205, (char)167, (char)163, (char)240, (char)159, (char)74, (char)165, (char)193, (char)192, (char)216}));
            assert(pack.transfer_type_GET() == (char)106);
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)131, (char)131, (char)67, (char)168, (char)96, (char)214, (char)41, (char)123, (char)195, (char)21, (char)77, (char)231, (char)176, (char)191, (char)254, (char)100, (char)140, (char)231, (char)239, (char)124, (char)118, (char)14, (char)170, (char)92, (char)52, (char)139, (char)90, (char)249, (char)188, (char)217, (char)153, (char)71, (char)65, (char)102, (char)223, (char)85, (char)23, (char)183, (char)54, (char)97, (char)113, (char)161, (char)201, (char)65, (char)115, (char)25, (char)26, (char)134, (char)241, (char)24, (char)117, (char)116, (char)64, (char)81, (char)170, (char)1, (char)101, (char)95, (char)26, (char)4, (char)161, (char)50, (char)53, (char)162, (char)105, (char)74, (char)75, (char)126, (char)9, (char)188, (char)89, (char)37, (char)225, (char)102, (char)230, (char)157, (char)219, (char)89, (char)108, (char)95, (char)210, (char)212, (char)113, (char)11, (char)135, (char)203, (char)188, (char)197, (char)118, (char)237, (char)192, (char)83, (char)56, (char)8, (char)195, (char)25, (char)213, (char)86, (char)150, (char)59, (char)90, (char)200, (char)135, (char)186, (char)194, (char)243, (char)171, (char)119, (char)207, (char)120, (char)224, (char)41, (char)130, (char)162, (char)112, (char)112, (char)172, (char)177, (char)7, (char)139}));
            assert(pack.request_id_GET() == (char)14);
            assert(pack.uri_type_GET() == (char)55);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.uri_type_SET((char)55) ;
        p142.transfer_type_SET((char)106) ;
        p142.storage_SET(new char[] {(char)182, (char)76, (char)116, (char)125, (char)189, (char)82, (char)203, (char)245, (char)14, (char)175, (char)238, (char)92, (char)146, (char)41, (char)127, (char)209, (char)57, (char)222, (char)222, (char)67, (char)69, (char)156, (char)235, (char)93, (char)85, (char)93, (char)164, (char)124, (char)96, (char)192, (char)229, (char)64, (char)47, (char)226, (char)149, (char)9, (char)101, (char)78, (char)184, (char)62, (char)206, (char)203, (char)126, (char)198, (char)215, (char)197, (char)195, (char)52, (char)9, (char)177, (char)151, (char)203, (char)220, (char)182, (char)252, (char)149, (char)249, (char)77, (char)18, (char)219, (char)206, (char)35, (char)151, (char)149, (char)209, (char)93, (char)222, (char)221, (char)130, (char)252, (char)221, (char)214, (char)97, (char)40, (char)104, (char)76, (char)181, (char)72, (char)136, (char)156, (char)87, (char)25, (char)162, (char)15, (char)56, (char)180, (char)20, (char)150, (char)110, (char)228, (char)227, (char)76, (char)68, (char)162, (char)170, (char)141, (char)50, (char)140, (char)76, (char)207, (char)129, (char)92, (char)187, (char)199, (char)168, (char)134, (char)161, (char)37, (char)89, (char)250, (char)205, (char)167, (char)163, (char)240, (char)159, (char)74, (char)165, (char)193, (char)192, (char)216}, 0) ;
        p142.request_id_SET((char)14) ;
        p142.uri_SET(new char[] {(char)131, (char)131, (char)67, (char)168, (char)96, (char)214, (char)41, (char)123, (char)195, (char)21, (char)77, (char)231, (char)176, (char)191, (char)254, (char)100, (char)140, (char)231, (char)239, (char)124, (char)118, (char)14, (char)170, (char)92, (char)52, (char)139, (char)90, (char)249, (char)188, (char)217, (char)153, (char)71, (char)65, (char)102, (char)223, (char)85, (char)23, (char)183, (char)54, (char)97, (char)113, (char)161, (char)201, (char)65, (char)115, (char)25, (char)26, (char)134, (char)241, (char)24, (char)117, (char)116, (char)64, (char)81, (char)170, (char)1, (char)101, (char)95, (char)26, (char)4, (char)161, (char)50, (char)53, (char)162, (char)105, (char)74, (char)75, (char)126, (char)9, (char)188, (char)89, (char)37, (char)225, (char)102, (char)230, (char)157, (char)219, (char)89, (char)108, (char)95, (char)210, (char)212, (char)113, (char)11, (char)135, (char)203, (char)188, (char)197, (char)118, (char)237, (char)192, (char)83, (char)56, (char)8, (char)195, (char)25, (char)213, (char)86, (char)150, (char)59, (char)90, (char)200, (char)135, (char)186, (char)194, (char)243, (char)171, (char)119, (char)207, (char)120, (char)224, (char)41, (char)130, (char)162, (char)112, (char)112, (char)172, (char)177, (char)7, (char)139}, 0) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == -1.5703509E38F);
            assert(pack.time_boot_ms_GET() == 2143693706L);
            assert(pack.press_abs_GET() == -1.9761257E38F);
            assert(pack.temperature_GET() == (short)19805);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.press_diff_SET(-1.5703509E38F) ;
        p143.time_boot_ms_SET(2143693706L) ;
        p143.press_abs_SET(-1.9761257E38F) ;
        p143.temperature_SET((short)19805) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.vel_GET(),  new float[] {-2.3159102E38F, -1.0949371E37F, 1.3367678E38F}));
            assert(pack.lat_GET() == -188676116);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-2.3214216E38F, -1.628166E38F, -2.3357357E38F}));
            assert(pack.lon_GET() == 96746077);
            assert(pack.timestamp_GET() == 4849615239892052837L);
            assert(pack.est_capabilities_GET() == (char)20);
            assert(pack.custom_state_GET() == 1355839187223721052L);
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-8.2088586E37F, -2.7963819E38F, -1.7847222E38F}));
            assert(pack.alt_GET() == -9.666262E37F);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-2.200401E38F, -1.3398135E38F, 3.1836568E38F}));
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-2.4608603E38F, 3.2158706E38F, -1.9622207E38F, 2.9863336E37F}));
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.est_capabilities_SET((char)20) ;
        p144.attitude_q_SET(new float[] {-2.4608603E38F, 3.2158706E38F, -1.9622207E38F, 2.9863336E37F}, 0) ;
        p144.lat_SET(-188676116) ;
        p144.custom_state_SET(1355839187223721052L) ;
        p144.rates_SET(new float[] {-8.2088586E37F, -2.7963819E38F, -1.7847222E38F}, 0) ;
        p144.lon_SET(96746077) ;
        p144.vel_SET(new float[] {-2.3159102E38F, -1.0949371E37F, 1.3367678E38F}, 0) ;
        p144.timestamp_SET(4849615239892052837L) ;
        p144.alt_SET(-9.666262E37F) ;
        p144.position_cov_SET(new float[] {-2.200401E38F, -1.3398135E38F, 3.1836568E38F}, 0) ;
        p144.acc_SET(new float[] {-2.3214216E38F, -1.628166E38F, -2.3357357E38F}, 0) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.x_vel_GET() == 6.10558E37F);
            assert(pack.y_acc_GET() == -2.1960272E37F);
            assert(pack.x_acc_GET() == 3.2156184E38F);
            assert(pack.z_acc_GET() == -2.9642385E38F);
            assert(pack.airspeed_GET() == -2.9757234E38F);
            assert(pack.yaw_rate_GET() == -2.1800562E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {-3.0727546E37F, -6.03302E37F, 3.8741978E37F}));
            assert(pack.z_vel_GET() == 3.0162188E38F);
            assert(pack.y_vel_GET() == 2.7387236E36F);
            assert(pack.pitch_rate_GET() == 3.1371373E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {2.7252853E38F, -3.5649524E37F, -3.3188677E37F}));
            assert(pack.y_pos_GET() == -2.640754E38F);
            assert(pack.x_pos_GET() == -1.5920077E38F);
            assert(pack.time_usec_GET() == 453913811275602733L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.9800006E37F, 8.433876E37F, 2.2302122E38F, -5.731012E37F}));
            assert(pack.roll_rate_GET() == 1.1317827E38F);
            assert(pack.z_pos_GET() == -1.6501886E38F);
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.z_pos_SET(-1.6501886E38F) ;
        p146.q_SET(new float[] {2.9800006E37F, 8.433876E37F, 2.2302122E38F, -5.731012E37F}, 0) ;
        p146.x_vel_SET(6.10558E37F) ;
        p146.y_acc_SET(-2.1960272E37F) ;
        p146.vel_variance_SET(new float[] {-3.0727546E37F, -6.03302E37F, 3.8741978E37F}, 0) ;
        p146.roll_rate_SET(1.1317827E38F) ;
        p146.y_pos_SET(-2.640754E38F) ;
        p146.pitch_rate_SET(3.1371373E38F) ;
        p146.x_acc_SET(3.2156184E38F) ;
        p146.time_usec_SET(453913811275602733L) ;
        p146.airspeed_SET(-2.9757234E38F) ;
        p146.z_acc_SET(-2.9642385E38F) ;
        p146.y_vel_SET(2.7387236E36F) ;
        p146.z_vel_SET(3.0162188E38F) ;
        p146.yaw_rate_SET(-2.1800562E38F) ;
        p146.x_pos_SET(-1.5920077E38F) ;
        p146.pos_variance_SET(new float[] {2.7252853E38F, -3.5649524E37F, -3.3188677E37F}, 0) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)56028, (char)49742, (char)29744, (char)53327, (char)39035, (char)38513, (char)18043, (char)54686, (char)58979, (char)21827}));
            assert(pack.battery_remaining_GET() == (byte)108);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
            assert(pack.energy_consumed_GET() == -1746605097);
            assert(pack.current_consumed_GET() == -223585444);
            assert(pack.current_battery_GET() == (short)17020);
            assert(pack.id_GET() == (char)174);
            assert(pack.temperature_GET() == (short)5451);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.temperature_SET((short)5451) ;
        p147.battery_remaining_SET((byte)108) ;
        p147.voltages_SET(new char[] {(char)56028, (char)49742, (char)29744, (char)53327, (char)39035, (char)38513, (char)18043, (char)54686, (char)58979, (char)21827}, 0) ;
        p147.id_SET((char)174) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION) ;
        p147.energy_consumed_SET(-1746605097) ;
        p147.current_battery_SET((short)17020) ;
        p147.current_consumed_SET(-223585444) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)170, (char)23, (char)58, (char)243, (char)40, (char)9, (char)10, (char)137}));
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)194, (char)71, (char)54, (char)250, (char)105, (char)101, (char)152, (char)136}));
            assert(pack.os_sw_version_GET() == 4104927636L);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)196, (char)222, (char)252, (char)85, (char)149, (char)141, (char)239, (char)193}));
            assert(pack.product_id_GET() == (char)10619);
            assert(pack.flight_sw_version_GET() == 3497034312L);
            assert(pack.vendor_id_GET() == (char)8608);
            assert(pack.middleware_sw_version_GET() == 565492167L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)193, (char)118, (char)104, (char)219, (char)6, (char)111, (char)5, (char)209, (char)249, (char)241, (char)103, (char)249, (char)128, (char)206, (char)61, (char)236, (char)38, (char)135}));
            assert(pack.uid_GET() == 6423493263864426665L);
            assert(pack.capabilities_GET() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE);
            assert(pack.board_version_GET() == 667689557L);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE) ;
        p148.middleware_sw_version_SET(565492167L) ;
        p148.uid_SET(6423493263864426665L) ;
        p148.flight_custom_version_SET(new char[] {(char)170, (char)23, (char)58, (char)243, (char)40, (char)9, (char)10, (char)137}, 0) ;
        p148.os_sw_version_SET(4104927636L) ;
        p148.product_id_SET((char)10619) ;
        p148.vendor_id_SET((char)8608) ;
        p148.middleware_custom_version_SET(new char[] {(char)194, (char)71, (char)54, (char)250, (char)105, (char)101, (char)152, (char)136}, 0) ;
        p148.flight_sw_version_SET(3497034312L) ;
        p148.os_custom_version_SET(new char[] {(char)196, (char)222, (char)252, (char)85, (char)149, (char)141, (char)239, (char)193}, 0) ;
        p148.uid2_SET(new char[] {(char)193, (char)118, (char)104, (char)219, (char)6, (char)111, (char)5, (char)209, (char)249, (char)241, (char)103, (char)249, (char)128, (char)206, (char)61, (char)236, (char)38, (char)135}, 0, PH) ;
        p148.board_version_SET(667689557L) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.distance_GET() == -1.7848557E38F);
            assert(pack.angle_x_GET() == 3.1234556E38F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {-2.5290268E38F, -6.475453E37F, -6.1300667E37F, -1.994492E38F}));
            assert(pack.size_x_GET() == -2.7283642E38F);
            assert(pack.angle_y_GET() == 7.380077E37F);
            assert(pack.time_usec_GET() == 4042213549644156054L);
            assert(pack.x_TRY(ph) == -1.6643608E38F);
            assert(pack.z_TRY(ph) == 2.2774413E37F);
            assert(pack.target_num_GET() == (char)234);
            assert(pack.size_y_GET() == -2.8583619E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.position_valid_TRY(ph) == (char)134);
            assert(pack.y_TRY(ph) == -8.4732033E37F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p149.size_x_SET(-2.7283642E38F) ;
        p149.y_SET(-8.4732033E37F, PH) ;
        p149.target_num_SET((char)234) ;
        p149.angle_y_SET(7.380077E37F) ;
        p149.size_y_SET(-2.8583619E38F) ;
        p149.angle_x_SET(3.1234556E38F) ;
        p149.time_usec_SET(4042213549644156054L) ;
        p149.q_SET(new float[] {-2.5290268E38F, -6.475453E37F, -6.1300667E37F, -1.994492E38F}, 0, PH) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON) ;
        p149.position_valid_SET((char)134, PH) ;
        p149.distance_SET(-1.7848557E38F) ;
        p149.z_SET(2.2774413E37F, PH) ;
        p149.x_SET(-1.6643608E38F, PH) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AQ_TELEMETRY_F.add((src, ph, pack) ->
        {
            assert(pack.value3_GET() == -1.8379175E38F);
            assert(pack.value1_GET() == -3.1596658E37F);
            assert(pack.value11_GET() == -1.74953E38F);
            assert(pack.value9_GET() == 6.3094474E37F);
            assert(pack.value19_GET() == -2.737315E37F);
            assert(pack.value15_GET() == -6.074886E36F);
            assert(pack.Index_GET() == (char)58524);
            assert(pack.value18_GET() == -3.156908E38F);
            assert(pack.value2_GET() == -1.3442511E38F);
            assert(pack.value16_GET() == 1.0850965E38F);
            assert(pack.value14_GET() == 1.05518355E37F);
            assert(pack.value6_GET() == 1.3905131E38F);
            assert(pack.value4_GET() == -1.16505E38F);
            assert(pack.value7_GET() == -1.019339E38F);
            assert(pack.value20_GET() == 9.997079E37F);
            assert(pack.value8_GET() == 1.7084567E38F);
            assert(pack.value5_GET() == -2.1766845E38F);
            assert(pack.value13_GET() == -1.4937035E38F);
            assert(pack.value12_GET() == -2.515545E37F);
            assert(pack.value17_GET() == -2.286623E38F);
            assert(pack.value10_GET() == -3.2946219E38F);
        });
        GroundControl.AQ_TELEMETRY_F p150 = CommunicationChannel.new_AQ_TELEMETRY_F();
        PH.setPack(p150);
        p150.value7_SET(-1.019339E38F) ;
        p150.value8_SET(1.7084567E38F) ;
        p150.value14_SET(1.05518355E37F) ;
        p150.value2_SET(-1.3442511E38F) ;
        p150.value3_SET(-1.8379175E38F) ;
        p150.value12_SET(-2.515545E37F) ;
        p150.value17_SET(-2.286623E38F) ;
        p150.Index_SET((char)58524) ;
        p150.value15_SET(-6.074886E36F) ;
        p150.value5_SET(-2.1766845E38F) ;
        p150.value13_SET(-1.4937035E38F) ;
        p150.value1_SET(-3.1596658E37F) ;
        p150.value10_SET(-3.2946219E38F) ;
        p150.value6_SET(1.3905131E38F) ;
        p150.value19_SET(-2.737315E37F) ;
        p150.value4_SET(-1.16505E38F) ;
        p150.value11_SET(-1.74953E38F) ;
        p150.value9_SET(6.3094474E37F) ;
        p150.value18_SET(-3.156908E38F) ;
        p150.value20_SET(9.997079E37F) ;
        p150.value16_SET(1.0850965E38F) ;
        CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AQ_ESC_TELEMETRY.add((src, ph, pack) ->
        {
            assert(pack.num_in_seq_GET() == (char)55);
            assert(Arrays.equals(pack.data1_GET(),  new long[] {2397123797L, 2475768098L, 222062884L, 2501058016L}));
            assert(pack.seq_GET() == (char)28);
            assert(pack.num_motors_GET() == (char)96);
            assert(Arrays.equals(pack.status_age_GET(),  new char[] {(char)44063, (char)56888, (char)13571, (char)61322}));
            assert(Arrays.equals(pack.data_version_GET(),  new char[] {(char)21, (char)111, (char)177, (char)55}));
            assert(Arrays.equals(pack.data0_GET(),  new long[] {2996675296L, 3253680671L, 3177829877L, 3047178857L}));
            assert(Arrays.equals(pack.escid_GET(),  new char[] {(char)22, (char)203, (char)197, (char)106}));
            assert(pack.time_boot_ms_GET() == 2967962376L);
        });
        GroundControl.AQ_ESC_TELEMETRY p152 = CommunicationChannel.new_AQ_ESC_TELEMETRY();
        PH.setPack(p152);
        p152.escid_SET(new char[] {(char)22, (char)203, (char)197, (char)106}, 0) ;
        p152.time_boot_ms_SET(2967962376L) ;
        p152.data1_SET(new long[] {2397123797L, 2475768098L, 222062884L, 2501058016L}, 0) ;
        p152.data0_SET(new long[] {2996675296L, 3253680671L, 3177829877L, 3047178857L}, 0) ;
        p152.data_version_SET(new char[] {(char)21, (char)111, (char)177, (char)55}, 0) ;
        p152.num_in_seq_SET((char)55) ;
        p152.seq_SET((char)28) ;
        p152.status_age_SET(new char[] {(char)44063, (char)56888, (char)13571, (char)61322}, 0) ;
        p152.num_motors_SET((char)96) ;
        CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 8839479083668820218L);
            assert(pack.pos_horiz_ratio_GET() == -1.2363559E38F);
            assert(pack.flags_GET() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL);
            assert(pack.tas_ratio_GET() == 1.3500873E38F);
            assert(pack.pos_horiz_accuracy_GET() == 1.9095463E38F);
            assert(pack.vel_ratio_GET() == 2.6280764E38F);
            assert(pack.hagl_ratio_GET() == 2.0495643E38F);
            assert(pack.pos_vert_accuracy_GET() == 3.0219202E38F);
            assert(pack.mag_ratio_GET() == 1.0158135E38F);
            assert(pack.pos_vert_ratio_GET() == -2.8034448E38F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.hagl_ratio_SET(2.0495643E38F) ;
        p230.mag_ratio_SET(1.0158135E38F) ;
        p230.pos_horiz_accuracy_SET(1.9095463E38F) ;
        p230.pos_vert_accuracy_SET(3.0219202E38F) ;
        p230.time_usec_SET(8839479083668820218L) ;
        p230.pos_horiz_ratio_SET(-1.2363559E38F) ;
        p230.pos_vert_ratio_SET(-2.8034448E38F) ;
        p230.vel_ratio_SET(2.6280764E38F) ;
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL) ;
        p230.tas_ratio_SET(1.3500873E38F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_z_GET() == 1.1407085E38F);
            assert(pack.var_vert_GET() == -1.7417325E38F);
            assert(pack.wind_y_GET() == 2.8334206E38F);
            assert(pack.horiz_accuracy_GET() == -1.5162873E38F);
            assert(pack.wind_alt_GET() == 1.2234129E35F);
            assert(pack.vert_accuracy_GET() == -1.5382586E38F);
            assert(pack.var_horiz_GET() == -1.6794169E38F);
            assert(pack.wind_x_GET() == -2.7641044E38F);
            assert(pack.time_usec_GET() == 6119007485921227301L);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.horiz_accuracy_SET(-1.5162873E38F) ;
        p231.wind_alt_SET(1.2234129E35F) ;
        p231.var_vert_SET(-1.7417325E38F) ;
        p231.var_horiz_SET(-1.6794169E38F) ;
        p231.wind_x_SET(-2.7641044E38F) ;
        p231.wind_y_SET(2.8334206E38F) ;
        p231.vert_accuracy_SET(-1.5382586E38F) ;
        p231.time_usec_SET(6119007485921227301L) ;
        p231.wind_z_SET(1.1407085E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.vd_GET() == -1.9766549E38F);
            assert(pack.satellites_visible_GET() == (char)23);
            assert(pack.hdop_GET() == -3.0916664E38F);
            assert(pack.ignore_flags_GET() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY);
            assert(pack.vn_GET() == 1.164368E38F);
            assert(pack.time_week_GET() == (char)17890);
            assert(pack.fix_type_GET() == (char)135);
            assert(pack.vert_accuracy_GET() == -1.2887671E37F);
            assert(pack.lon_GET() == -468456275);
            assert(pack.vdop_GET() == -1.9216943E37F);
            assert(pack.time_week_ms_GET() == 3991684068L);
            assert(pack.lat_GET() == 1112670752);
            assert(pack.speed_accuracy_GET() == -8.79091E37F);
            assert(pack.alt_GET() == -6.097231E37F);
            assert(pack.ve_GET() == 5.320285E36F);
            assert(pack.gps_id_GET() == (char)246);
            assert(pack.horiz_accuracy_GET() == 1.8110062E38F);
            assert(pack.time_usec_GET() == 3073361857201021632L);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.hdop_SET(-3.0916664E38F) ;
        p232.satellites_visible_SET((char)23) ;
        p232.gps_id_SET((char)246) ;
        p232.lon_SET(-468456275) ;
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY) ;
        p232.horiz_accuracy_SET(1.8110062E38F) ;
        p232.speed_accuracy_SET(-8.79091E37F) ;
        p232.alt_SET(-6.097231E37F) ;
        p232.time_week_ms_SET(3991684068L) ;
        p232.time_week_SET((char)17890) ;
        p232.ve_SET(5.320285E36F) ;
        p232.vert_accuracy_SET(-1.2887671E37F) ;
        p232.fix_type_SET((char)135) ;
        p232.vd_SET(-1.9766549E38F) ;
        p232.lat_SET(1112670752) ;
        p232.vdop_SET(-1.9216943E37F) ;
        p232.time_usec_SET(3073361857201021632L) ;
        p232.vn_SET(1.164368E38F) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (char)99);
            assert(pack.len_GET() == (char)11);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)27, (char)79, (char)239, (char)224, (char)172, (char)123, (char)136, (char)213, (char)155, (char)93, (char)215, (char)168, (char)69, (char)1, (char)134, (char)65, (char)152, (char)39, (char)156, (char)90, (char)242, (char)87, (char)49, (char)159, (char)134, (char)77, (char)217, (char)93, (char)238, (char)152, (char)120, (char)170, (char)142, (char)96, (char)90, (char)124, (char)93, (char)64, (char)202, (char)42, (char)164, (char)181, (char)130, (char)108, (char)150, (char)114, (char)229, (char)139, (char)91, (char)246, (char)254, (char)214, (char)213, (char)241, (char)11, (char)14, (char)75, (char)218, (char)214, (char)71, (char)20, (char)32, (char)8, (char)76, (char)214, (char)118, (char)100, (char)100, (char)102, (char)97, (char)23, (char)130, (char)126, (char)46, (char)65, (char)4, (char)25, (char)26, (char)30, (char)218, (char)222, (char)123, (char)6, (char)79, (char)120, (char)24, (char)249, (char)194, (char)179, (char)194, (char)215, (char)1, (char)104, (char)131, (char)125, (char)179, (char)60, (char)254, (char)0, (char)86, (char)188, (char)88, (char)237, (char)76, (char)5, (char)247, (char)240, (char)251, (char)54, (char)21, (char)75, (char)193, (char)207, (char)2, (char)194, (char)61, (char)96, (char)180, (char)227, (char)166, (char)2, (char)197, (char)106, (char)88, (char)171, (char)201, (char)247, (char)170, (char)134, (char)87, (char)162, (char)181, (char)86, (char)81, (char)14, (char)118, (char)253, (char)201, (char)121, (char)90, (char)43, (char)129, (char)72, (char)225, (char)142, (char)241, (char)23, (char)132, (char)169, (char)19, (char)193, (char)148, (char)17, (char)124, (char)17, (char)9, (char)6, (char)58, (char)54, (char)52, (char)214, (char)175, (char)127, (char)50, (char)123, (char)188, (char)29, (char)52, (char)245, (char)45, (char)151, (char)56, (char)222, (char)26, (char)60, (char)144, (char)123, (char)26, (char)253, (char)81}));
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.len_SET((char)11) ;
        p233.flags_SET((char)99) ;
        p233.data__SET(new char[] {(char)27, (char)79, (char)239, (char)224, (char)172, (char)123, (char)136, (char)213, (char)155, (char)93, (char)215, (char)168, (char)69, (char)1, (char)134, (char)65, (char)152, (char)39, (char)156, (char)90, (char)242, (char)87, (char)49, (char)159, (char)134, (char)77, (char)217, (char)93, (char)238, (char)152, (char)120, (char)170, (char)142, (char)96, (char)90, (char)124, (char)93, (char)64, (char)202, (char)42, (char)164, (char)181, (char)130, (char)108, (char)150, (char)114, (char)229, (char)139, (char)91, (char)246, (char)254, (char)214, (char)213, (char)241, (char)11, (char)14, (char)75, (char)218, (char)214, (char)71, (char)20, (char)32, (char)8, (char)76, (char)214, (char)118, (char)100, (char)100, (char)102, (char)97, (char)23, (char)130, (char)126, (char)46, (char)65, (char)4, (char)25, (char)26, (char)30, (char)218, (char)222, (char)123, (char)6, (char)79, (char)120, (char)24, (char)249, (char)194, (char)179, (char)194, (char)215, (char)1, (char)104, (char)131, (char)125, (char)179, (char)60, (char)254, (char)0, (char)86, (char)188, (char)88, (char)237, (char)76, (char)5, (char)247, (char)240, (char)251, (char)54, (char)21, (char)75, (char)193, (char)207, (char)2, (char)194, (char)61, (char)96, (char)180, (char)227, (char)166, (char)2, (char)197, (char)106, (char)88, (char)171, (char)201, (char)247, (char)170, (char)134, (char)87, (char)162, (char)181, (char)86, (char)81, (char)14, (char)118, (char)253, (char)201, (char)121, (char)90, (char)43, (char)129, (char)72, (char)225, (char)142, (char)241, (char)23, (char)132, (char)169, (char)19, (char)193, (char)148, (char)17, (char)124, (char)17, (char)9, (char)6, (char)58, (char)54, (char)52, (char)214, (char)175, (char)127, (char)50, (char)123, (char)188, (char)29, (char)52, (char)245, (char)45, (char)151, (char)56, (char)222, (char)26, (char)60, (char)144, (char)123, (char)26, (char)253, (char)81}, 0) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.airspeed_sp_GET() == (char)28);
            assert(pack.temperature_air_GET() == (byte)79);
            assert(pack.climb_rate_GET() == (byte) - 93);
            assert(pack.wp_distance_GET() == (char)19950);
            assert(pack.roll_GET() == (short) -24139);
            assert(pack.pitch_GET() == (short) -32634);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED);
            assert(pack.longitude_GET() == 1314240104);
            assert(pack.altitude_sp_GET() == (short)29272);
            assert(pack.custom_mode_GET() == 2603980866L);
            assert(pack.heading_sp_GET() == (short)30699);
            assert(pack.gps_nsat_GET() == (char)90);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            assert(pack.battery_remaining_GET() == (char)25);
            assert(pack.throttle_GET() == (byte) - 84);
            assert(pack.altitude_amsl_GET() == (short) -24147);
            assert(pack.wp_num_GET() == (char)184);
            assert(pack.groundspeed_GET() == (char)112);
            assert(pack.latitude_GET() == 1464266993);
            assert(pack.failsafe_GET() == (char)55);
            assert(pack.temperature_GET() == (byte)0);
            assert(pack.airspeed_GET() == (char)123);
            assert(pack.heading_GET() == (char)25598);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        p234.gps_nsat_SET((char)90) ;
        p234.temperature_SET((byte)0) ;
        p234.airspeed_SET((char)123) ;
        p234.roll_SET((short) -24139) ;
        p234.heading_SET((char)25598) ;
        p234.airspeed_sp_SET((char)28) ;
        p234.wp_num_SET((char)184) ;
        p234.altitude_amsl_SET((short) -24147) ;
        p234.groundspeed_SET((char)112) ;
        p234.temperature_air_SET((byte)79) ;
        p234.altitude_sp_SET((short)29272) ;
        p234.failsafe_SET((char)55) ;
        p234.climb_rate_SET((byte) - 93) ;
        p234.throttle_SET((byte) - 84) ;
        p234.wp_distance_SET((char)19950) ;
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED) ;
        p234.longitude_SET(1314240104) ;
        p234.pitch_SET((short) -32634) ;
        p234.latitude_SET(1464266993) ;
        p234.custom_mode_SET(2603980866L) ;
        p234.battery_remaining_SET((char)25) ;
        p234.heading_sp_SET((short)30699) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.clipping_2_GET() == 4178604459L);
            assert(pack.clipping_0_GET() == 4007714180L);
            assert(pack.time_usec_GET() == 3795376356780104993L);
            assert(pack.vibration_z_GET() == 2.7130627E38F);
            assert(pack.clipping_1_GET() == 45303043L);
            assert(pack.vibration_y_GET() == -1.6836726E38F);
            assert(pack.vibration_x_GET() == -3.3926094E38F);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.clipping_2_SET(4178604459L) ;
        p241.vibration_x_SET(-3.3926094E38F) ;
        p241.vibration_y_SET(-1.6836726E38F) ;
        p241.time_usec_SET(3795376356780104993L) ;
        p241.clipping_1_SET(45303043L) ;
        p241.vibration_z_SET(2.7130627E38F) ;
        p241.clipping_0_SET(4007714180L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == 177552818);
            assert(pack.approach_y_GET() == -6.055277E37F);
            assert(pack.approach_x_GET() == -2.5732283E38F);
            assert(pack.time_usec_TRY(ph) == 7169161521654038997L);
            assert(pack.latitude_GET() == 484721868);
            assert(pack.x_GET() == -1.1419412E38F);
            assert(pack.approach_z_GET() == 3.2828396E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.2092747E38F, -3.1444996E38F, -1.5036372E38F, 8.387808E37F}));
            assert(pack.longitude_GET() == 21292898);
            assert(pack.y_GET() == 7.383795E37F);
            assert(pack.z_GET() == 1.2822573E38F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.latitude_SET(484721868) ;
        p242.approach_x_SET(-2.5732283E38F) ;
        p242.y_SET(7.383795E37F) ;
        p242.time_usec_SET(7169161521654038997L, PH) ;
        p242.approach_y_SET(-6.055277E37F) ;
        p242.z_SET(1.2822573E38F) ;
        p242.longitude_SET(21292898) ;
        p242.x_SET(-1.1419412E38F) ;
        p242.q_SET(new float[] {-3.2092747E38F, -3.1444996E38F, -1.5036372E38F, 8.387808E37F}, 0) ;
        p242.altitude_SET(177552818) ;
        p242.approach_z_SET(3.2828396E38F) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -2.7557053E38F);
            assert(pack.approach_x_GET() == -2.1724749E38F);
            assert(pack.latitude_GET() == -2020558479);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.019804E38F, 3.3138925E37F, -2.167104E38F, -2.2346715E38F}));
            assert(pack.longitude_GET() == -1978848015);
            assert(pack.approach_z_GET() == 2.751068E38F);
            assert(pack.altitude_GET() == -2062408676);
            assert(pack.target_system_GET() == (char)246);
            assert(pack.time_usec_TRY(ph) == 4115491891949291200L);
            assert(pack.z_GET() == 8.443826E37F);
            assert(pack.x_GET() == 1.9726515E38F);
            assert(pack.approach_y_GET() == -3.0616038E38F);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.latitude_SET(-2020558479) ;
        p243.approach_z_SET(2.751068E38F) ;
        p243.target_system_SET((char)246) ;
        p243.altitude_SET(-2062408676) ;
        p243.approach_y_SET(-3.0616038E38F) ;
        p243.y_SET(-2.7557053E38F) ;
        p243.approach_x_SET(-2.1724749E38F) ;
        p243.z_SET(8.443826E37F) ;
        p243.time_usec_SET(4115491891949291200L, PH) ;
        p243.x_SET(1.9726515E38F) ;
        p243.longitude_SET(-1978848015) ;
        p243.q_SET(new float[] {-2.019804E38F, 3.3138925E37F, -2.167104E38F, -2.2346715E38F}, 0) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)28395);
            assert(pack.interval_us_GET() == 1139393055);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(1139393055) ;
        p244.message_id_SET((char)28395) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == 1971100818);
            assert(pack.heading_GET() == (char)64903);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            assert(pack.ver_velocity_GET() == (short) -11356);
            assert(pack.squawk_GET() == (char)57229);
            assert(pack.lon_GET() == 912984384);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_EMERGENCY_SURFACE);
            assert(pack.ICAO_address_GET() == 88633048L);
            assert(pack.tslc_GET() == (char)255);
            assert(pack.hor_velocity_GET() == (char)30821);
            assert(pack.flags_GET() == ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS);
            assert(pack.lat_GET() == -2081775056);
            assert(pack.callsign_LEN(ph) == 1);
            assert(pack.callsign_TRY(ph).equals("y"));
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_EMERGENCY_SURFACE) ;
        p246.hor_velocity_SET((char)30821) ;
        p246.tslc_SET((char)255) ;
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.lat_SET(-2081775056) ;
        p246.squawk_SET((char)57229) ;
        p246.ver_velocity_SET((short) -11356) ;
        p246.lon_SET(912984384) ;
        p246.altitude_SET(1971100818) ;
        p246.ICAO_address_SET(88633048L) ;
        p246.callsign_SET("y", PH) ;
        p246.heading_SET((char)64903) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == 2622190684L);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
            assert(pack.altitude_minimum_delta_GET() == -9.328923E37F);
            assert(pack.horizontal_minimum_delta_GET() == 3.235379E38F);
            assert(pack.time_to_minimum_delta_GET() == -6.320663E37F);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.time_to_minimum_delta_SET(-6.320663E37F) ;
        p247.altitude_minimum_delta_SET(-9.328923E37F) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE) ;
        p247.horizontal_minimum_delta_SET(3.235379E38F) ;
        p247.id_SET(2622190684L) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)250, (char)28, (char)227, (char)160, (char)248, (char)167, (char)38, (char)194, (char)107, (char)204, (char)101, (char)239, (char)198, (char)140, (char)242, (char)75, (char)143, (char)169, (char)122, (char)216, (char)248, (char)223, (char)14, (char)175, (char)203, (char)13, (char)171, (char)4, (char)86, (char)219, (char)3, (char)113, (char)122, (char)158, (char)206, (char)19, (char)97, (char)51, (char)137, (char)160, (char)19, (char)253, (char)206, (char)31, (char)189, (char)90, (char)120, (char)115, (char)41, (char)175, (char)217, (char)84, (char)199, (char)218, (char)3, (char)111, (char)251, (char)142, (char)231, (char)253, (char)228, (char)194, (char)1, (char)169, (char)193, (char)240, (char)55, (char)220, (char)194, (char)68, (char)102, (char)89, (char)56, (char)250, (char)66, (char)99, (char)120, (char)136, (char)13, (char)92, (char)228, (char)151, (char)27, (char)71, (char)237, (char)129, (char)61, (char)84, (char)98, (char)139, (char)59, (char)165, (char)7, (char)105, (char)163, (char)211, (char)212, (char)2, (char)135, (char)95, (char)87, (char)232, (char)107, (char)243, (char)15, (char)222, (char)0, (char)145, (char)148, (char)195, (char)195, (char)106, (char)210, (char)201, (char)81, (char)105, (char)133, (char)139, (char)178, (char)147, (char)62, (char)255, (char)145, (char)71, (char)4, (char)57, (char)172, (char)152, (char)69, (char)255, (char)58, (char)96, (char)154, (char)96, (char)16, (char)147, (char)0, (char)83, (char)168, (char)115, (char)155, (char)119, (char)214, (char)98, (char)14, (char)20, (char)213, (char)161, (char)21, (char)169, (char)93, (char)179, (char)240, (char)196, (char)255, (char)181, (char)87, (char)232, (char)49, (char)42, (char)187, (char)251, (char)247, (char)62, (char)201, (char)151, (char)14, (char)14, (char)65, (char)226, (char)31, (char)200, (char)106, (char)89, (char)81, (char)106, (char)39, (char)184, (char)79, (char)159, (char)238, (char)236, (char)212, (char)11, (char)220, (char)126, (char)178, (char)211, (char)49, (char)142, (char)171, (char)198, (char)246, (char)169, (char)83, (char)179, (char)87, (char)28, (char)237, (char)201, (char)131, (char)121, (char)56, (char)146, (char)126, (char)142, (char)181, (char)57, (char)233, (char)206, (char)63, (char)230, (char)196, (char)52, (char)85, (char)50, (char)209, (char)69, (char)202, (char)239, (char)207, (char)139, (char)12, (char)70, (char)12, (char)158, (char)54, (char)43, (char)145, (char)247, (char)193, (char)141, (char)27, (char)113, (char)121, (char)197, (char)238, (char)190, (char)219, (char)17, (char)111, (char)70, (char)217, (char)91, (char)83, (char)60, (char)147, (char)105, (char)187}));
            assert(pack.target_component_GET() == (char)97);
            assert(pack.message_type_GET() == (char)40038);
            assert(pack.target_system_GET() == (char)104);
            assert(pack.target_network_GET() == (char)211);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_component_SET((char)97) ;
        p248.target_network_SET((char)211) ;
        p248.payload_SET(new char[] {(char)250, (char)28, (char)227, (char)160, (char)248, (char)167, (char)38, (char)194, (char)107, (char)204, (char)101, (char)239, (char)198, (char)140, (char)242, (char)75, (char)143, (char)169, (char)122, (char)216, (char)248, (char)223, (char)14, (char)175, (char)203, (char)13, (char)171, (char)4, (char)86, (char)219, (char)3, (char)113, (char)122, (char)158, (char)206, (char)19, (char)97, (char)51, (char)137, (char)160, (char)19, (char)253, (char)206, (char)31, (char)189, (char)90, (char)120, (char)115, (char)41, (char)175, (char)217, (char)84, (char)199, (char)218, (char)3, (char)111, (char)251, (char)142, (char)231, (char)253, (char)228, (char)194, (char)1, (char)169, (char)193, (char)240, (char)55, (char)220, (char)194, (char)68, (char)102, (char)89, (char)56, (char)250, (char)66, (char)99, (char)120, (char)136, (char)13, (char)92, (char)228, (char)151, (char)27, (char)71, (char)237, (char)129, (char)61, (char)84, (char)98, (char)139, (char)59, (char)165, (char)7, (char)105, (char)163, (char)211, (char)212, (char)2, (char)135, (char)95, (char)87, (char)232, (char)107, (char)243, (char)15, (char)222, (char)0, (char)145, (char)148, (char)195, (char)195, (char)106, (char)210, (char)201, (char)81, (char)105, (char)133, (char)139, (char)178, (char)147, (char)62, (char)255, (char)145, (char)71, (char)4, (char)57, (char)172, (char)152, (char)69, (char)255, (char)58, (char)96, (char)154, (char)96, (char)16, (char)147, (char)0, (char)83, (char)168, (char)115, (char)155, (char)119, (char)214, (char)98, (char)14, (char)20, (char)213, (char)161, (char)21, (char)169, (char)93, (char)179, (char)240, (char)196, (char)255, (char)181, (char)87, (char)232, (char)49, (char)42, (char)187, (char)251, (char)247, (char)62, (char)201, (char)151, (char)14, (char)14, (char)65, (char)226, (char)31, (char)200, (char)106, (char)89, (char)81, (char)106, (char)39, (char)184, (char)79, (char)159, (char)238, (char)236, (char)212, (char)11, (char)220, (char)126, (char)178, (char)211, (char)49, (char)142, (char)171, (char)198, (char)246, (char)169, (char)83, (char)179, (char)87, (char)28, (char)237, (char)201, (char)131, (char)121, (char)56, (char)146, (char)126, (char)142, (char)181, (char)57, (char)233, (char)206, (char)63, (char)230, (char)196, (char)52, (char)85, (char)50, (char)209, (char)69, (char)202, (char)239, (char)207, (char)139, (char)12, (char)70, (char)12, (char)158, (char)54, (char)43, (char)145, (char)247, (char)193, (char)141, (char)27, (char)113, (char)121, (char)197, (char)238, (char)190, (char)219, (char)17, (char)111, (char)70, (char)217, (char)91, (char)83, (char)60, (char)147, (char)105, (char)187}, 0) ;
        p248.target_system_SET((char)104) ;
        p248.message_type_SET((char)40038) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)90, (byte) - 14, (byte) - 104, (byte) - 35, (byte)113, (byte) - 52, (byte) - 23, (byte)75, (byte)100, (byte)7, (byte) - 80, (byte) - 61, (byte)108, (byte) - 68, (byte) - 105, (byte)120, (byte)62, (byte)25, (byte) - 25, (byte) - 63, (byte) - 60, (byte) - 27, (byte) - 49, (byte) - 66, (byte) - 52, (byte)30, (byte) - 76, (byte) - 122, (byte)103, (byte) - 124, (byte) - 33, (byte) - 89}));
            assert(pack.address_GET() == (char)40680);
            assert(pack.ver_GET() == (char)9);
            assert(pack.type_GET() == (char)96);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.value_SET(new byte[] {(byte)90, (byte) - 14, (byte) - 104, (byte) - 35, (byte)113, (byte) - 52, (byte) - 23, (byte)75, (byte)100, (byte)7, (byte) - 80, (byte) - 61, (byte)108, (byte) - 68, (byte) - 105, (byte)120, (byte)62, (byte)25, (byte) - 25, (byte) - 63, (byte) - 60, (byte) - 27, (byte) - 49, (byte) - 66, (byte) - 52, (byte)30, (byte) - 76, (byte) - 122, (byte)103, (byte) - 124, (byte) - 33, (byte) - 89}, 0) ;
        p249.ver_SET((char)9) ;
        p249.type_SET((char)96) ;
        p249.address_SET((char)40680) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 6);
            assert(pack.name_TRY(ph).equals("BDrzvu"));
            assert(pack.x_GET() == 1.3653847E38F);
            assert(pack.y_GET() == -1.3774879E38F);
            assert(pack.time_usec_GET() == 4365141780076282525L);
            assert(pack.z_GET() == 1.7013209E38F);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("BDrzvu", PH) ;
        p250.x_SET(1.3653847E38F) ;
        p250.z_SET(1.7013209E38F) ;
        p250.time_usec_SET(4365141780076282525L) ;
        p250.y_SET(-1.3774879E38F) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == -1.5089088E38F);
            assert(pack.name_LEN(ph) == 4);
            assert(pack.name_TRY(ph).equals("Jaxy"));
            assert(pack.time_boot_ms_GET() == 3910981332L);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(3910981332L) ;
        p251.value_SET(-1.5089088E38F) ;
        p251.name_SET("Jaxy", PH) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 3);
            assert(pack.name_TRY(ph).equals("anp"));
            assert(pack.time_boot_ms_GET() == 3934543814L);
            assert(pack.value_GET() == 1416709826);
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.name_SET("anp", PH) ;
        p252.time_boot_ms_SET(3934543814L) ;
        p252.value_SET(1416709826) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 1);
            assert(pack.text_TRY(ph).equals("a"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_WARNING);
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("a", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_WARNING) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2498137283L);
            assert(pack.ind_GET() == (char)86);
            assert(pack.value_GET() == -3.127335E38F);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.ind_SET((char)86) ;
        p254.time_boot_ms_SET(2498137283L) ;
        p254.value_SET(-3.127335E38F) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)180, (char)149, (char)197, (char)127, (char)127, (char)1, (char)79, (char)88, (char)38, (char)13, (char)40, (char)235, (char)70, (char)117, (char)23, (char)248, (char)219, (char)22, (char)173, (char)180, (char)7, (char)88, (char)0, (char)191, (char)57, (char)106, (char)147, (char)206, (char)6, (char)79, (char)108, (char)242}));
            assert(pack.initial_timestamp_GET() == 7262146750700989940L);
            assert(pack.target_component_GET() == (char)31);
            assert(pack.target_system_GET() == (char)126);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.initial_timestamp_SET(7262146750700989940L) ;
        p256.target_system_SET((char)126) ;
        p256.target_component_SET((char)31) ;
        p256.secret_key_SET(new char[] {(char)180, (char)149, (char)197, (char)127, (char)127, (char)1, (char)79, (char)88, (char)38, (char)13, (char)40, (char)235, (char)70, (char)117, (char)23, (char)248, (char)219, (char)22, (char)173, (char)180, (char)7, (char)88, (char)0, (char)191, (char)57, (char)106, (char)147, (char)206, (char)6, (char)79, (char)108, (char)242}, 0) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.last_change_ms_GET() == 3260378030L);
            assert(pack.state_GET() == (char)80);
            assert(pack.time_boot_ms_GET() == 1881060766L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(1881060766L) ;
        p257.last_change_ms_SET(3260378030L) ;
        p257.state_SET((char)80) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)208);
            assert(pack.target_component_GET() == (char)42);
            assert(pack.tune_LEN(ph) == 21);
            assert(pack.tune_TRY(ph).equals("kammbymxftTidjvvmeGnv"));
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)208) ;
        p258.target_component_SET((char)42) ;
        p258.tune_SET("kammbymxftTidjvvmeGnv", PH) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)62, (char)192, (char)205, (char)150, (char)154, (char)160, (char)69, (char)250, (char)103, (char)54, (char)118, (char)142, (char)23, (char)35, (char)247, (char)115, (char)161, (char)52, (char)251, (char)105, (char)190, (char)40, (char)165, (char)176, (char)161, (char)72, (char)219, (char)168, (char)36, (char)125, (char)174, (char)114}));
            assert(pack.lens_id_GET() == (char)87);
            assert(pack.sensor_size_h_GET() == 8.837346E37F);
            assert(pack.flags_GET() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
            assert(pack.cam_definition_uri_LEN(ph) == 12);
            assert(pack.cam_definition_uri_TRY(ph).equals("nFkltxsjuxdo"));
            assert(pack.cam_definition_version_GET() == (char)63522);
            assert(pack.sensor_size_v_GET() == 2.3478327E38F);
            assert(pack.firmware_version_GET() == 3907562448L);
            assert(pack.resolution_h_GET() == (char)42394);
            assert(pack.focal_length_GET() == -6.963313E37F);
            assert(pack.resolution_v_GET() == (char)14126);
            assert(pack.time_boot_ms_GET() == 2944684248L);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)1, (char)137, (char)179, (char)205, (char)193, (char)19, (char)98, (char)239, (char)210, (char)74, (char)153, (char)36, (char)227, (char)152, (char)164, (char)64, (char)24, (char)30, (char)170, (char)2, (char)204, (char)255, (char)183, (char)165, (char)67, (char)227, (char)45, (char)14, (char)143, (char)166, (char)45, (char)126}));
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.cam_definition_uri_SET("nFkltxsjuxdo", PH) ;
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE) ;
        p259.vendor_name_SET(new char[] {(char)1, (char)137, (char)179, (char)205, (char)193, (char)19, (char)98, (char)239, (char)210, (char)74, (char)153, (char)36, (char)227, (char)152, (char)164, (char)64, (char)24, (char)30, (char)170, (char)2, (char)204, (char)255, (char)183, (char)165, (char)67, (char)227, (char)45, (char)14, (char)143, (char)166, (char)45, (char)126}, 0) ;
        p259.model_name_SET(new char[] {(char)62, (char)192, (char)205, (char)150, (char)154, (char)160, (char)69, (char)250, (char)103, (char)54, (char)118, (char)142, (char)23, (char)35, (char)247, (char)115, (char)161, (char)52, (char)251, (char)105, (char)190, (char)40, (char)165, (char)176, (char)161, (char)72, (char)219, (char)168, (char)36, (char)125, (char)174, (char)114}, 0) ;
        p259.sensor_size_v_SET(2.3478327E38F) ;
        p259.time_boot_ms_SET(2944684248L) ;
        p259.firmware_version_SET(3907562448L) ;
        p259.cam_definition_version_SET((char)63522) ;
        p259.focal_length_SET(-6.963313E37F) ;
        p259.resolution_v_SET((char)14126) ;
        p259.resolution_h_SET((char)42394) ;
        p259.lens_id_SET((char)87) ;
        p259.sensor_size_h_SET(8.837346E37F) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2814204853L);
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_VIDEO);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_VIDEO) ;
        p260.time_boot_ms_SET(2814204853L) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.total_capacity_GET() == 1.2217811E38F);
            assert(pack.storage_id_GET() == (char)101);
            assert(pack.used_capacity_GET() == 1.4562197E38F);
            assert(pack.storage_count_GET() == (char)43);
            assert(pack.time_boot_ms_GET() == 2626273599L);
            assert(pack.available_capacity_GET() == 5.105708E37F);
            assert(pack.write_speed_GET() == 3.047544E38F);
            assert(pack.read_speed_GET() == 1.2101661E38F);
            assert(pack.status_GET() == (char)163);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.status_SET((char)163) ;
        p261.storage_count_SET((char)43) ;
        p261.used_capacity_SET(1.4562197E38F) ;
        p261.read_speed_SET(1.2101661E38F) ;
        p261.time_boot_ms_SET(2626273599L) ;
        p261.available_capacity_SET(5.105708E37F) ;
        p261.total_capacity_SET(1.2217811E38F) ;
        p261.write_speed_SET(3.047544E38F) ;
        p261.storage_id_SET((char)101) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.image_status_GET() == (char)109);
            assert(pack.video_status_GET() == (char)36);
            assert(pack.available_capacity_GET() == 2.186873E38F);
            assert(pack.time_boot_ms_GET() == 2588053738L);
            assert(pack.recording_time_ms_GET() == 870062327L);
            assert(pack.image_interval_GET() == 2.7783287E38F);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.video_status_SET((char)36) ;
        p262.available_capacity_SET(2.186873E38F) ;
        p262.image_interval_SET(2.7783287E38F) ;
        p262.image_status_SET((char)109) ;
        p262.time_boot_ms_SET(2588053738L) ;
        p262.recording_time_ms_SET(870062327L) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.image_index_GET() == 180461018);
            assert(pack.capture_result_GET() == (byte)13);
            assert(pack.relative_alt_GET() == -1304356198);
            assert(pack.time_boot_ms_GET() == 3265306975L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {4.2116013E37F, -1.1981603E38F, 5.520484E36F, 2.458675E38F}));
            assert(pack.file_url_LEN(ph) == 114);
            assert(pack.file_url_TRY(ph).equals("gRyqUavvysmokyvzaphMFvhloAlqzTnwcobydmbOwhdwyqgqcxugeptthcryixakleFfgsybyjioWOsfhJvyjlmudelsgiolisHkflfzacsyJsuixz"));
            assert(pack.camera_id_GET() == (char)8);
            assert(pack.lon_GET() == -448568936);
            assert(pack.lat_GET() == 750263546);
            assert(pack.alt_GET() == -1235972410);
            assert(pack.time_utc_GET() == 1859391813769185657L);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.file_url_SET("gRyqUavvysmokyvzaphMFvhloAlqzTnwcobydmbOwhdwyqgqcxugeptthcryixakleFfgsybyjioWOsfhJvyjlmudelsgiolisHkflfzacsyJsuixz", PH) ;
        p263.q_SET(new float[] {4.2116013E37F, -1.1981603E38F, 5.520484E36F, 2.458675E38F}, 0) ;
        p263.time_boot_ms_SET(3265306975L) ;
        p263.relative_alt_SET(-1304356198) ;
        p263.lat_SET(750263546) ;
        p263.time_utc_SET(1859391813769185657L) ;
        p263.lon_SET(-448568936) ;
        p263.capture_result_SET((byte)13) ;
        p263.image_index_SET(180461018) ;
        p263.alt_SET(-1235972410) ;
        p263.camera_id_SET((char)8) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.arming_time_utc_GET() == 5329447069464822468L);
            assert(pack.time_boot_ms_GET() == 848607582L);
            assert(pack.flight_uuid_GET() == 3543569355698946268L);
            assert(pack.takeoff_time_utc_GET() == 8483415677089723579L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.time_boot_ms_SET(848607582L) ;
        p264.flight_uuid_SET(3543569355698946268L) ;
        p264.arming_time_utc_SET(5329447069464822468L) ;
        p264.takeoff_time_utc_SET(8483415677089723579L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -1.5908314E38F);
            assert(pack.time_boot_ms_GET() == 3045461330L);
            assert(pack.pitch_GET() == 2.7767128E38F);
            assert(pack.roll_GET() == 1.9426776E38F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.yaw_SET(-1.5908314E38F) ;
        p265.pitch_SET(2.7767128E38F) ;
        p265.roll_SET(1.9426776E38F) ;
        p265.time_boot_ms_SET(3045461330L) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)181);
            assert(pack.first_message_offset_GET() == (char)151);
            assert(pack.length_GET() == (char)171);
            assert(pack.target_component_GET() == (char)40);
            assert(pack.sequence_GET() == (char)41732);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)0, (char)147, (char)251, (char)125, (char)190, (char)225, (char)203, (char)148, (char)38, (char)32, (char)125, (char)90, (char)35, (char)102, (char)213, (char)39, (char)52, (char)84, (char)184, (char)233, (char)3, (char)198, (char)228, (char)131, (char)130, (char)231, (char)144, (char)57, (char)95, (char)139, (char)178, (char)29, (char)95, (char)182, (char)190, (char)108, (char)0, (char)208, (char)120, (char)52, (char)177, (char)218, (char)13, (char)228, (char)106, (char)245, (char)156, (char)163, (char)233, (char)57, (char)192, (char)150, (char)183, (char)15, (char)1, (char)225, (char)207, (char)138, (char)179, (char)91, (char)42, (char)4, (char)20, (char)85, (char)175, (char)211, (char)139, (char)86, (char)108, (char)167, (char)227, (char)114, (char)191, (char)87, (char)39, (char)6, (char)18, (char)24, (char)40, (char)113, (char)92, (char)30, (char)238, (char)148, (char)1, (char)229, (char)137, (char)30, (char)64, (char)182, (char)91, (char)145, (char)87, (char)214, (char)226, (char)92, (char)194, (char)206, (char)68, (char)113, (char)252, (char)135, (char)223, (char)179, (char)243, (char)171, (char)5, (char)144, (char)119, (char)120, (char)18, (char)207, (char)83, (char)15, (char)200, (char)146, (char)4, (char)223, (char)77, (char)87, (char)243, (char)40, (char)135, (char)184, (char)137, (char)248, (char)229, (char)144, (char)27, (char)78, (char)111, (char)155, (char)110, (char)202, (char)91, (char)98, (char)57, (char)185, (char)204, (char)208, (char)253, (char)47, (char)24, (char)53, (char)239, (char)152, (char)202, (char)83, (char)183, (char)172, (char)125, (char)233, (char)36, (char)197, (char)113, (char)217, (char)204, (char)0, (char)82, (char)196, (char)28, (char)189, (char)193, (char)122, (char)111, (char)23, (char)17, (char)79, (char)170, (char)253, (char)114, (char)19, (char)39, (char)222, (char)230, (char)63, (char)147, (char)82, (char)23, (char)65, (char)162, (char)203, (char)221, (char)166, (char)141, (char)51, (char)119, (char)74, (char)149, (char)47, (char)126, (char)166, (char)32, (char)101, (char)147, (char)44, (char)76, (char)8, (char)204, (char)215, (char)237, (char)78, (char)84, (char)189, (char)160, (char)84, (char)26, (char)4, (char)34, (char)166, (char)75, (char)183, (char)112, (char)220, (char)182, (char)62, (char)203, (char)199, (char)198, (char)209, (char)204, (char)225, (char)36, (char)165, (char)169, (char)42, (char)132, (char)248, (char)129, (char)214, (char)111, (char)54, (char)145, (char)189, (char)249, (char)182, (char)172, (char)213, (char)121, (char)187, (char)220, (char)203, (char)250, (char)26, (char)49, (char)93, (char)87, (char)72, (char)117}));
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.length_SET((char)171) ;
        p266.first_message_offset_SET((char)151) ;
        p266.data__SET(new char[] {(char)0, (char)147, (char)251, (char)125, (char)190, (char)225, (char)203, (char)148, (char)38, (char)32, (char)125, (char)90, (char)35, (char)102, (char)213, (char)39, (char)52, (char)84, (char)184, (char)233, (char)3, (char)198, (char)228, (char)131, (char)130, (char)231, (char)144, (char)57, (char)95, (char)139, (char)178, (char)29, (char)95, (char)182, (char)190, (char)108, (char)0, (char)208, (char)120, (char)52, (char)177, (char)218, (char)13, (char)228, (char)106, (char)245, (char)156, (char)163, (char)233, (char)57, (char)192, (char)150, (char)183, (char)15, (char)1, (char)225, (char)207, (char)138, (char)179, (char)91, (char)42, (char)4, (char)20, (char)85, (char)175, (char)211, (char)139, (char)86, (char)108, (char)167, (char)227, (char)114, (char)191, (char)87, (char)39, (char)6, (char)18, (char)24, (char)40, (char)113, (char)92, (char)30, (char)238, (char)148, (char)1, (char)229, (char)137, (char)30, (char)64, (char)182, (char)91, (char)145, (char)87, (char)214, (char)226, (char)92, (char)194, (char)206, (char)68, (char)113, (char)252, (char)135, (char)223, (char)179, (char)243, (char)171, (char)5, (char)144, (char)119, (char)120, (char)18, (char)207, (char)83, (char)15, (char)200, (char)146, (char)4, (char)223, (char)77, (char)87, (char)243, (char)40, (char)135, (char)184, (char)137, (char)248, (char)229, (char)144, (char)27, (char)78, (char)111, (char)155, (char)110, (char)202, (char)91, (char)98, (char)57, (char)185, (char)204, (char)208, (char)253, (char)47, (char)24, (char)53, (char)239, (char)152, (char)202, (char)83, (char)183, (char)172, (char)125, (char)233, (char)36, (char)197, (char)113, (char)217, (char)204, (char)0, (char)82, (char)196, (char)28, (char)189, (char)193, (char)122, (char)111, (char)23, (char)17, (char)79, (char)170, (char)253, (char)114, (char)19, (char)39, (char)222, (char)230, (char)63, (char)147, (char)82, (char)23, (char)65, (char)162, (char)203, (char)221, (char)166, (char)141, (char)51, (char)119, (char)74, (char)149, (char)47, (char)126, (char)166, (char)32, (char)101, (char)147, (char)44, (char)76, (char)8, (char)204, (char)215, (char)237, (char)78, (char)84, (char)189, (char)160, (char)84, (char)26, (char)4, (char)34, (char)166, (char)75, (char)183, (char)112, (char)220, (char)182, (char)62, (char)203, (char)199, (char)198, (char)209, (char)204, (char)225, (char)36, (char)165, (char)169, (char)42, (char)132, (char)248, (char)129, (char)214, (char)111, (char)54, (char)145, (char)189, (char)249, (char)182, (char)172, (char)213, (char)121, (char)187, (char)220, (char)203, (char)250, (char)26, (char)49, (char)93, (char)87, (char)72, (char)117}, 0) ;
        p266.sequence_SET((char)41732) ;
        p266.target_system_SET((char)181) ;
        p266.target_component_SET((char)40) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.length_GET() == (char)235);
            assert(pack.target_system_GET() == (char)238);
            assert(pack.target_component_GET() == (char)72);
            assert(pack.sequence_GET() == (char)65341);
            assert(pack.first_message_offset_GET() == (char)29);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)85, (char)204, (char)100, (char)128, (char)221, (char)91, (char)161, (char)33, (char)216, (char)123, (char)217, (char)236, (char)61, (char)135, (char)135, (char)65, (char)92, (char)61, (char)188, (char)14, (char)46, (char)76, (char)220, (char)143, (char)60, (char)104, (char)117, (char)244, (char)32, (char)226, (char)217, (char)161, (char)115, (char)89, (char)44, (char)136, (char)53, (char)36, (char)11, (char)108, (char)62, (char)192, (char)235, (char)97, (char)250, (char)192, (char)55, (char)37, (char)13, (char)83, (char)215, (char)33, (char)41, (char)152, (char)226, (char)12, (char)16, (char)76, (char)198, (char)78, (char)184, (char)153, (char)141, (char)224, (char)239, (char)116, (char)69, (char)130, (char)248, (char)167, (char)15, (char)153, (char)254, (char)110, (char)54, (char)93, (char)65, (char)32, (char)30, (char)0, (char)34, (char)130, (char)201, (char)15, (char)223, (char)4, (char)203, (char)68, (char)22, (char)185, (char)98, (char)196, (char)49, (char)177, (char)164, (char)137, (char)23, (char)116, (char)69, (char)135, (char)248, (char)215, (char)140, (char)5, (char)163, (char)78, (char)254, (char)49, (char)224, (char)96, (char)78, (char)137, (char)48, (char)137, (char)245, (char)218, (char)107, (char)17, (char)2, (char)151, (char)250, (char)57, (char)164, (char)94, (char)80, (char)236, (char)184, (char)178, (char)227, (char)8, (char)76, (char)25, (char)199, (char)105, (char)159, (char)235, (char)163, (char)150, (char)148, (char)177, (char)45, (char)244, (char)18, (char)94, (char)84, (char)251, (char)214, (char)65, (char)75, (char)64, (char)176, (char)255, (char)93, (char)190, (char)210, (char)150, (char)236, (char)5, (char)197, (char)245, (char)206, (char)81, (char)208, (char)222, (char)8, (char)104, (char)204, (char)45, (char)37, (char)227, (char)241, (char)52, (char)182, (char)78, (char)92, (char)104, (char)42, (char)108, (char)82, (char)129, (char)64, (char)132, (char)151, (char)21, (char)219, (char)59, (char)238, (char)178, (char)100, (char)167, (char)87, (char)200, (char)165, (char)7, (char)11, (char)120, (char)205, (char)146, (char)115, (char)226, (char)122, (char)90, (char)57, (char)110, (char)33, (char)212, (char)155, (char)64, (char)35, (char)238, (char)32, (char)119, (char)196, (char)199, (char)154, (char)113, (char)24, (char)110, (char)166, (char)49, (char)89, (char)118, (char)70, (char)163, (char)108, (char)193, (char)33, (char)178, (char)117, (char)53, (char)16, (char)53, (char)136, (char)116, (char)93, (char)48, (char)172, (char)70, (char)182, (char)82, (char)215, (char)17, (char)240, (char)143, (char)213, (char)185, (char)144, (char)65, (char)179}));
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.data__SET(new char[] {(char)85, (char)204, (char)100, (char)128, (char)221, (char)91, (char)161, (char)33, (char)216, (char)123, (char)217, (char)236, (char)61, (char)135, (char)135, (char)65, (char)92, (char)61, (char)188, (char)14, (char)46, (char)76, (char)220, (char)143, (char)60, (char)104, (char)117, (char)244, (char)32, (char)226, (char)217, (char)161, (char)115, (char)89, (char)44, (char)136, (char)53, (char)36, (char)11, (char)108, (char)62, (char)192, (char)235, (char)97, (char)250, (char)192, (char)55, (char)37, (char)13, (char)83, (char)215, (char)33, (char)41, (char)152, (char)226, (char)12, (char)16, (char)76, (char)198, (char)78, (char)184, (char)153, (char)141, (char)224, (char)239, (char)116, (char)69, (char)130, (char)248, (char)167, (char)15, (char)153, (char)254, (char)110, (char)54, (char)93, (char)65, (char)32, (char)30, (char)0, (char)34, (char)130, (char)201, (char)15, (char)223, (char)4, (char)203, (char)68, (char)22, (char)185, (char)98, (char)196, (char)49, (char)177, (char)164, (char)137, (char)23, (char)116, (char)69, (char)135, (char)248, (char)215, (char)140, (char)5, (char)163, (char)78, (char)254, (char)49, (char)224, (char)96, (char)78, (char)137, (char)48, (char)137, (char)245, (char)218, (char)107, (char)17, (char)2, (char)151, (char)250, (char)57, (char)164, (char)94, (char)80, (char)236, (char)184, (char)178, (char)227, (char)8, (char)76, (char)25, (char)199, (char)105, (char)159, (char)235, (char)163, (char)150, (char)148, (char)177, (char)45, (char)244, (char)18, (char)94, (char)84, (char)251, (char)214, (char)65, (char)75, (char)64, (char)176, (char)255, (char)93, (char)190, (char)210, (char)150, (char)236, (char)5, (char)197, (char)245, (char)206, (char)81, (char)208, (char)222, (char)8, (char)104, (char)204, (char)45, (char)37, (char)227, (char)241, (char)52, (char)182, (char)78, (char)92, (char)104, (char)42, (char)108, (char)82, (char)129, (char)64, (char)132, (char)151, (char)21, (char)219, (char)59, (char)238, (char)178, (char)100, (char)167, (char)87, (char)200, (char)165, (char)7, (char)11, (char)120, (char)205, (char)146, (char)115, (char)226, (char)122, (char)90, (char)57, (char)110, (char)33, (char)212, (char)155, (char)64, (char)35, (char)238, (char)32, (char)119, (char)196, (char)199, (char)154, (char)113, (char)24, (char)110, (char)166, (char)49, (char)89, (char)118, (char)70, (char)163, (char)108, (char)193, (char)33, (char)178, (char)117, (char)53, (char)16, (char)53, (char)136, (char)116, (char)93, (char)48, (char)172, (char)70, (char)182, (char)82, (char)215, (char)17, (char)240, (char)143, (char)213, (char)185, (char)144, (char)65, (char)179}, 0) ;
        p267.target_component_SET((char)72) ;
        p267.sequence_SET((char)65341) ;
        p267.first_message_offset_SET((char)29) ;
        p267.length_SET((char)235) ;
        p267.target_system_SET((char)238) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)68);
            assert(pack.target_component_GET() == (char)200);
            assert(pack.sequence_GET() == (char)17382);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)68) ;
        p268.sequence_SET((char)17382) ;
        p268.target_component_SET((char)200) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.framerate_GET() == -2.5172742E38F);
            assert(pack.resolution_h_GET() == (char)5242);
            assert(pack.resolution_v_GET() == (char)23074);
            assert(pack.status_GET() == (char)28);
            assert(pack.bitrate_GET() == 1568655982L);
            assert(pack.camera_id_GET() == (char)147);
            assert(pack.uri_LEN(ph) == 45);
            assert(pack.uri_TRY(ph).equals("jxvppkoondbrxiaceezkggpJdcctkjCivaouemjljcutc"));
            assert(pack.rotation_GET() == (char)47853);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.bitrate_SET(1568655982L) ;
        p269.framerate_SET(-2.5172742E38F) ;
        p269.uri_SET("jxvppkoondbrxiaceezkggpJdcctkjCivaouemjljcutc", PH) ;
        p269.status_SET((char)28) ;
        p269.resolution_h_SET((char)5242) ;
        p269.rotation_SET((char)47853) ;
        p269.resolution_v_SET((char)23074) ;
        p269.camera_id_SET((char)147) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.framerate_GET() == 1.287115E38F);
            assert(pack.rotation_GET() == (char)42829);
            assert(pack.camera_id_GET() == (char)200);
            assert(pack.bitrate_GET() == 3757415300L);
            assert(pack.resolution_h_GET() == (char)13757);
            assert(pack.resolution_v_GET() == (char)27847);
            assert(pack.uri_LEN(ph) == 185);
            assert(pack.uri_TRY(ph).equals("umpkvylpeotoawhxqnpozphqbryglibKtewexzcqrDziaadrxpMijjrnrvpqsqYpyeXvdlxlMHntZprapmrwigrourvqvoddmAylhLvtoTeplNEtmpGoiHgmlcxdznmpdscwnedqOobhxlwJeffzdhwwllzkcRktnssiesmigiiSuseDKrbSvTCrc"));
            assert(pack.target_system_GET() == (char)219);
            assert(pack.target_component_GET() == (char)38);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.bitrate_SET(3757415300L) ;
        p270.target_system_SET((char)219) ;
        p270.resolution_v_SET((char)27847) ;
        p270.resolution_h_SET((char)13757) ;
        p270.target_component_SET((char)38) ;
        p270.framerate_SET(1.287115E38F) ;
        p270.rotation_SET((char)42829) ;
        p270.camera_id_SET((char)200) ;
        p270.uri_SET("umpkvylpeotoawhxqnpozphqbryglibKtewexzcqrDziaadrxpMijjrnrvpqsqYpyeXvdlxlMHntZprapmrwigrourvqvoddmAylhLvtoTeplNEtmpGoiHgmlcxdznmpdscwnedqOobhxlwJeffzdhwwllzkcRktnssiesmigiiSuseDKrbSvTCrc", PH) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 7);
            assert(pack.ssid_TRY(ph).equals("lxfypdp"));
            assert(pack.password_LEN(ph) == 17);
            assert(pack.password_TRY(ph).equals("oKbfysflpwcbesMgc"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("oKbfysflpwcbesMgc", PH) ;
        p299.ssid_SET("lxfypdp", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)107, (char)4, (char)180, (char)239, (char)174, (char)168, (char)64, (char)248}));
            assert(pack.min_version_GET() == (char)17777);
            assert(pack.max_version_GET() == (char)22569);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)104, (char)37, (char)83, (char)206, (char)210, (char)186, (char)231, (char)144}));
            assert(pack.version_GET() == (char)58480);
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)58480) ;
        p300.min_version_SET((char)17777) ;
        p300.max_version_SET((char)22569) ;
        p300.library_version_hash_SET(new char[] {(char)104, (char)37, (char)83, (char)206, (char)210, (char)186, (char)231, (char)144}, 0) ;
        p300.spec_version_hash_SET(new char[] {(char)107, (char)4, (char)180, (char)239, (char)174, (char)168, (char)64, (char)248}, 0) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
            assert(pack.time_usec_GET() == 2634397257992885210L);
            assert(pack.vendor_specific_status_code_GET() == (char)58969);
            assert(pack.sub_mode_GET() == (char)14);
            assert(pack.uptime_sec_GET() == 4093933951L);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK) ;
        p310.sub_mode_SET((char)14) ;
        p310.time_usec_SET(2634397257992885210L) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE) ;
        p310.vendor_specific_status_code_SET((char)58969) ;
        p310.uptime_sec_SET(4093933951L) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.uptime_sec_GET() == 1502435427L);
            assert(pack.hw_version_major_GET() == (char)178);
            assert(pack.hw_version_minor_GET() == (char)112);
            assert(pack.name_LEN(ph) == 19);
            assert(pack.name_TRY(ph).equals("ikuamqhrmgPtowkjiju"));
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)23, (char)102, (char)193, (char)250, (char)127, (char)113, (char)18, (char)108, (char)22, (char)190, (char)97, (char)127, (char)225, (char)176, (char)15, (char)62}));
            assert(pack.sw_version_minor_GET() == (char)187);
            assert(pack.sw_version_major_GET() == (char)196);
            assert(pack.sw_vcs_commit_GET() == 3023891498L);
            assert(pack.time_usec_GET() == 7823745127393191266L);
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.time_usec_SET(7823745127393191266L) ;
        p311.hw_unique_id_SET(new char[] {(char)23, (char)102, (char)193, (char)250, (char)127, (char)113, (char)18, (char)108, (char)22, (char)190, (char)97, (char)127, (char)225, (char)176, (char)15, (char)62}, 0) ;
        p311.hw_version_minor_SET((char)112) ;
        p311.hw_version_major_SET((char)178) ;
        p311.sw_vcs_commit_SET(3023891498L) ;
        p311.name_SET("ikuamqhrmgPtowkjiju", PH) ;
        p311.sw_version_major_SET((char)196) ;
        p311.sw_version_minor_SET((char)187) ;
        p311.uptime_sec_SET(1502435427L) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 11);
            assert(pack.param_id_TRY(ph).equals("cmydKQabrjk"));
            assert(pack.target_component_GET() == (char)58);
            assert(pack.target_system_GET() == (char)176);
            assert(pack.param_index_GET() == (short) -4874);
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.param_id_SET("cmydKQabrjk", PH) ;
        p320.target_component_SET((char)58) ;
        p320.target_system_SET((char)176) ;
        p320.param_index_SET((short) -4874) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)160);
            assert(pack.target_system_GET() == (char)11);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)160) ;
        p321.target_system_SET((char)11) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 29);
            assert(pack.param_value_TRY(ph).equals("nKqdygtylvqpjvpscxvqslsIoehms"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("boevlalhq"));
            assert(pack.param_index_GET() == (char)17664);
            assert(pack.param_count_GET() == (char)9523);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_index_SET((char)17664) ;
        p322.param_count_SET((char)9523) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64) ;
        p322.param_id_SET("boevlalhq", PH) ;
        p322.param_value_SET("nKqdygtylvqpjvpscxvqslsIoehms", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)202);
            assert(pack.target_component_GET() == (char)249);
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("cm"));
            assert(pack.param_value_LEN(ph) == 22);
            assert(pack.param_value_TRY(ph).equals("vjyvszwZijsSoplpncaiit"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)202) ;
        p323.param_value_SET("vjyvszwZijsSoplpncaiit", PH) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16) ;
        p323.target_component_SET((char)249) ;
        p323.param_id_SET("cm", PH) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 52);
            assert(pack.param_value_TRY(ph).equals("lyuvqqdnbivnhlcYunczctxhwXmxifehfnDajuFbioysrbsbtyca"));
            assert(pack.param_id_LEN(ph) == 13);
            assert(pack.param_id_TRY(ph).equals("uhGSdywgejspn"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_ACCEPTED);
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32) ;
        p324.param_id_SET("uhGSdywgejspn", PH) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_ACCEPTED) ;
        p324.param_value_SET("lyuvqqdnbivnhlcYunczctxhwXmxifehfnDajuFbioysrbsbtyca", PH) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 1839072768538312585L);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
            assert(pack.max_distance_GET() == (char)28197);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)31374, (char)35195, (char)19722, (char)34967, (char)3986, (char)27726, (char)31561, (char)39324, (char)19315, (char)52118, (char)44557, (char)36342, (char)16868, (char)18014, (char)38783, (char)38784, (char)59112, (char)11099, (char)20191, (char)40649, (char)57450, (char)49625, (char)40666, (char)54195, (char)26377, (char)15953, (char)29865, (char)32227, (char)17985, (char)17649, (char)9453, (char)24918, (char)37601, (char)18930, (char)14088, (char)30707, (char)63509, (char)52657, (char)9037, (char)24754, (char)32782, (char)13166, (char)36432, (char)8144, (char)35404, (char)24511, (char)57541, (char)61029, (char)33788, (char)64442, (char)19294, (char)39627, (char)23233, (char)28270, (char)14066, (char)18159, (char)46798, (char)12191, (char)13565, (char)46579, (char)49121, (char)29497, (char)46517, (char)62991, (char)38540, (char)61781, (char)1259, (char)5882, (char)16380, (char)8859, (char)48375, (char)42202}));
            assert(pack.increment_GET() == (char)228);
            assert(pack.min_distance_GET() == (char)43907);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.increment_SET((char)228) ;
        p330.max_distance_SET((char)28197) ;
        p330.min_distance_SET((char)43907) ;
        p330.time_usec_SET(1839072768538312585L) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND) ;
        p330.distances_SET(new char[] {(char)31374, (char)35195, (char)19722, (char)34967, (char)3986, (char)27726, (char)31561, (char)39324, (char)19315, (char)52118, (char)44557, (char)36342, (char)16868, (char)18014, (char)38783, (char)38784, (char)59112, (char)11099, (char)20191, (char)40649, (char)57450, (char)49625, (char)40666, (char)54195, (char)26377, (char)15953, (char)29865, (char)32227, (char)17985, (char)17649, (char)9453, (char)24918, (char)37601, (char)18930, (char)14088, (char)30707, (char)63509, (char)52657, (char)9037, (char)24754, (char)32782, (char)13166, (char)36432, (char)8144, (char)35404, (char)24511, (char)57541, (char)61029, (char)33788, (char)64442, (char)19294, (char)39627, (char)23233, (char)28270, (char)14066, (char)18159, (char)46798, (char)12191, (char)13565, (char)46579, (char)49121, (char)29497, (char)46517, (char)62991, (char)38540, (char)61781, (char)1259, (char)5882, (char)16380, (char)8859, (char)48375, (char)42202}, 0) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
    }

}
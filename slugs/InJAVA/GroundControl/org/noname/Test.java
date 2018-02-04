
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
                case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
                    id = 17;
                    break;
                case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
                    id = 18;
                    break;
                case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
                    id = 19;
                    break;
                case MAV_CMD.MAV_CMD_NAV_DELAY:
                    id = 20;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
                    id = 21;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAST:
                    id = 22;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DELAY:
                    id = 23;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
                    id = 24;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
                    id = 25;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_YAW:
                    id = 26;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_LAST:
                    id = 27;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_MODE:
                    id = 28;
                    break;
                case MAV_CMD.MAV_CMD_DO_JUMP:
                    id = 29;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
                    id = 30;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_HOME:
                    id = 31;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
                    id = 32;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_RELAY:
                    id = 33;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
                    id = 34;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_SERVO:
                    id = 35;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
                    id = 36;
                    break;
                case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
                    id = 37;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
                    id = 38;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAND_START:
                    id = 39;
                    break;
                case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
                    id = 40;
                    break;
                case MAV_CMD.MAV_CMD_DO_GO_AROUND:
                    id = 41;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPOSITION:
                    id = 42;
                    break;
                case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
                    id = 43;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
                    id = 44;
                    break;
                case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
                    id = 45;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_ROI:
                    id = 46;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
                    id = 47;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
                    id = 48;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
                    id = 49;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                    id = 50;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                    id = 51;
                    break;
                case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
                    id = 52;
                    break;
                case MAV_CMD.MAV_CMD_DO_PARACHUTE:
                    id = 53;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
                    id = 54;
                    break;
                case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
                    id = 55;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                    id = 56;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                    id = 57;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                    id = 58;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                    id = 59;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                    id = 60;
                    break;
                case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                    id = 61;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAST:
                    id = 62;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                    id = 63;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                    id = 64;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                    id = 65;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                    id = 66;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                    id = 67;
                    break;
                case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                    id = 68;
                    break;
                case MAV_CMD.MAV_CMD_MISSION_START:
                    id = 69;
                    break;
                case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                    id = 70;
                    break;
                case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                    id = 71;
                    break;
                case MAV_CMD.MAV_CMD_START_RX_PAIR:
                    id = 72;
                    break;
                case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                    id = 73;
                    break;
                case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                    id = 74;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                    id = 75;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                    id = 76;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                    id = 77;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                    id = 78;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                    id = 79;
                    break;
                case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                    id = 80;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                    id = 81;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                    id = 82;
                    break;
                case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                    id = 83;
                    break;
                case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                    id = 84;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                    id = 85;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                    id = 86;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                    id = 87;
                    break;
                case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                    id = 88;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                    id = 89;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                    id = 90;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                    id = 91;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                    id = 92;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                    id = 93;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_START:
                    id = 94;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_STOP:
                    id = 95;
                    break;
                case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                    id = 96;
                    break;
                case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                    id = 97;
                    break;
                case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                    id = 98;
                    break;
                case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                    id = 99;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                    id = 100;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                    id = 101;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_GATE:
                    id = 102;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                    id = 103;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                    id = 104;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                    id = 105;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                    id = 106;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                    id = 107;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                    id = 108;
                    break;
                case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                    id = 109;
                    break;
                case MAV_CMD.MAV_CMD_DO_NOTHING:
                    id = 110;
                    break;
                case MAV_CMD.MAV_CMD_RETURN_TO_BASE:
                    id = 111;
                    break;
                case MAV_CMD.MAV_CMD_STOP_RETURN_TO_BASE:
                    id = 112;
                    break;
                case MAV_CMD.MAV_CMD_TURN_LIGHT:
                    id = 113;
                    break;
                case MAV_CMD.MAV_CMD_GET_MID_LEVEL_COMMANDS:
                    id = 114;
                    break;
                case MAV_CMD.MAV_CMD_MIDLEVEL_STORAGE:
                    id = 115;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                    id = 116;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                    id = 117;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                    id = 118;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                    id = 119;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                    id = 120;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                    id = 121;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                    id = 122;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                    id = 123;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                    id = 124;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                    id = 125;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                    id = 126;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                    id = 127;
                    break;
                case MAV_CMD.MAV_CMD_USER_1:
                    id = 128;
                    break;
                case MAV_CMD.MAV_CMD_USER_2:
                    id = 129;
                    break;
                case MAV_CMD.MAV_CMD_USER_3:
                    id = 130;
                    break;
                case MAV_CMD.MAV_CMD_USER_4:
                    id = 131;
                    break;
                case MAV_CMD.MAV_CMD_USER_5:
                    id = 132;
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
                case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
                    id = 17;
                    break;
                case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
                    id = 18;
                    break;
                case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
                    id = 19;
                    break;
                case MAV_CMD.MAV_CMD_NAV_DELAY:
                    id = 20;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
                    id = 21;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAST:
                    id = 22;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DELAY:
                    id = 23;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
                    id = 24;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
                    id = 25;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_YAW:
                    id = 26;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_LAST:
                    id = 27;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_MODE:
                    id = 28;
                    break;
                case MAV_CMD.MAV_CMD_DO_JUMP:
                    id = 29;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
                    id = 30;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_HOME:
                    id = 31;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
                    id = 32;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_RELAY:
                    id = 33;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
                    id = 34;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_SERVO:
                    id = 35;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
                    id = 36;
                    break;
                case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
                    id = 37;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
                    id = 38;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAND_START:
                    id = 39;
                    break;
                case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
                    id = 40;
                    break;
                case MAV_CMD.MAV_CMD_DO_GO_AROUND:
                    id = 41;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPOSITION:
                    id = 42;
                    break;
                case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
                    id = 43;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
                    id = 44;
                    break;
                case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
                    id = 45;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_ROI:
                    id = 46;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
                    id = 47;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
                    id = 48;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
                    id = 49;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                    id = 50;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                    id = 51;
                    break;
                case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
                    id = 52;
                    break;
                case MAV_CMD.MAV_CMD_DO_PARACHUTE:
                    id = 53;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
                    id = 54;
                    break;
                case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
                    id = 55;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                    id = 56;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                    id = 57;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                    id = 58;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                    id = 59;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                    id = 60;
                    break;
                case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                    id = 61;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAST:
                    id = 62;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                    id = 63;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                    id = 64;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                    id = 65;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                    id = 66;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                    id = 67;
                    break;
                case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                    id = 68;
                    break;
                case MAV_CMD.MAV_CMD_MISSION_START:
                    id = 69;
                    break;
                case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                    id = 70;
                    break;
                case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                    id = 71;
                    break;
                case MAV_CMD.MAV_CMD_START_RX_PAIR:
                    id = 72;
                    break;
                case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                    id = 73;
                    break;
                case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                    id = 74;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                    id = 75;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                    id = 76;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                    id = 77;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                    id = 78;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                    id = 79;
                    break;
                case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                    id = 80;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                    id = 81;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                    id = 82;
                    break;
                case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                    id = 83;
                    break;
                case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                    id = 84;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                    id = 85;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                    id = 86;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                    id = 87;
                    break;
                case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                    id = 88;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                    id = 89;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                    id = 90;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                    id = 91;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                    id = 92;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                    id = 93;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_START:
                    id = 94;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_STOP:
                    id = 95;
                    break;
                case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                    id = 96;
                    break;
                case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                    id = 97;
                    break;
                case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                    id = 98;
                    break;
                case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                    id = 99;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                    id = 100;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                    id = 101;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_GATE:
                    id = 102;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                    id = 103;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                    id = 104;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                    id = 105;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                    id = 106;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                    id = 107;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                    id = 108;
                    break;
                case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                    id = 109;
                    break;
                case MAV_CMD.MAV_CMD_DO_NOTHING:
                    id = 110;
                    break;
                case MAV_CMD.MAV_CMD_RETURN_TO_BASE:
                    id = 111;
                    break;
                case MAV_CMD.MAV_CMD_STOP_RETURN_TO_BASE:
                    id = 112;
                    break;
                case MAV_CMD.MAV_CMD_TURN_LIGHT:
                    id = 113;
                    break;
                case MAV_CMD.MAV_CMD_GET_MID_LEVEL_COMMANDS:
                    id = 114;
                    break;
                case MAV_CMD.MAV_CMD_MIDLEVEL_STORAGE:
                    id = 115;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                    id = 116;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                    id = 117;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                    id = 118;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                    id = 119;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                    id = 120;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                    id = 121;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                    id = 122;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                    id = 123;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                    id = 124;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                    id = 125;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                    id = 126;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                    id = 127;
                    break;
                case MAV_CMD.MAV_CMD_USER_1:
                    id = 128;
                    break;
                case MAV_CMD.MAV_CMD_USER_2:
                    id = 129;
                    break;
                case MAV_CMD.MAV_CMD_USER_3:
                    id = 130;
                    break;
                case MAV_CMD.MAV_CMD_USER_4:
                    id = 131;
                    break;
                case MAV_CMD.MAV_CMD_USER_5:
                    id = 132;
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
                case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
                    id = 17;
                    break;
                case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
                    id = 18;
                    break;
                case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
                    id = 19;
                    break;
                case MAV_CMD.MAV_CMD_NAV_DELAY:
                    id = 20;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
                    id = 21;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAST:
                    id = 22;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DELAY:
                    id = 23;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
                    id = 24;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
                    id = 25;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_YAW:
                    id = 26;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_LAST:
                    id = 27;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_MODE:
                    id = 28;
                    break;
                case MAV_CMD.MAV_CMD_DO_JUMP:
                    id = 29;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
                    id = 30;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_HOME:
                    id = 31;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
                    id = 32;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_RELAY:
                    id = 33;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
                    id = 34;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_SERVO:
                    id = 35;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
                    id = 36;
                    break;
                case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
                    id = 37;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
                    id = 38;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAND_START:
                    id = 39;
                    break;
                case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
                    id = 40;
                    break;
                case MAV_CMD.MAV_CMD_DO_GO_AROUND:
                    id = 41;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPOSITION:
                    id = 42;
                    break;
                case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
                    id = 43;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
                    id = 44;
                    break;
                case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
                    id = 45;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_ROI:
                    id = 46;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
                    id = 47;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
                    id = 48;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
                    id = 49;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                    id = 50;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                    id = 51;
                    break;
                case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
                    id = 52;
                    break;
                case MAV_CMD.MAV_CMD_DO_PARACHUTE:
                    id = 53;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
                    id = 54;
                    break;
                case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
                    id = 55;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                    id = 56;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                    id = 57;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                    id = 58;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                    id = 59;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                    id = 60;
                    break;
                case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                    id = 61;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAST:
                    id = 62;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                    id = 63;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                    id = 64;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                    id = 65;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                    id = 66;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                    id = 67;
                    break;
                case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                    id = 68;
                    break;
                case MAV_CMD.MAV_CMD_MISSION_START:
                    id = 69;
                    break;
                case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                    id = 70;
                    break;
                case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                    id = 71;
                    break;
                case MAV_CMD.MAV_CMD_START_RX_PAIR:
                    id = 72;
                    break;
                case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                    id = 73;
                    break;
                case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                    id = 74;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                    id = 75;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                    id = 76;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                    id = 77;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                    id = 78;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                    id = 79;
                    break;
                case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                    id = 80;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                    id = 81;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                    id = 82;
                    break;
                case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                    id = 83;
                    break;
                case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                    id = 84;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                    id = 85;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                    id = 86;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                    id = 87;
                    break;
                case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                    id = 88;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                    id = 89;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                    id = 90;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                    id = 91;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                    id = 92;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                    id = 93;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_START:
                    id = 94;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_STOP:
                    id = 95;
                    break;
                case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                    id = 96;
                    break;
                case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                    id = 97;
                    break;
                case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                    id = 98;
                    break;
                case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                    id = 99;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                    id = 100;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                    id = 101;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_GATE:
                    id = 102;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                    id = 103;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                    id = 104;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                    id = 105;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                    id = 106;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                    id = 107;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                    id = 108;
                    break;
                case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                    id = 109;
                    break;
                case MAV_CMD.MAV_CMD_DO_NOTHING:
                    id = 110;
                    break;
                case MAV_CMD.MAV_CMD_RETURN_TO_BASE:
                    id = 111;
                    break;
                case MAV_CMD.MAV_CMD_STOP_RETURN_TO_BASE:
                    id = 112;
                    break;
                case MAV_CMD.MAV_CMD_TURN_LIGHT:
                    id = 113;
                    break;
                case MAV_CMD.MAV_CMD_GET_MID_LEVEL_COMMANDS:
                    id = 114;
                    break;
                case MAV_CMD.MAV_CMD_MIDLEVEL_STORAGE:
                    id = 115;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                    id = 116;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                    id = 117;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                    id = 118;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                    id = 119;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                    id = 120;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                    id = 121;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                    id = 122;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                    id = 123;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                    id = 124;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                    id = 125;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                    id = 126;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                    id = 127;
                    break;
                case MAV_CMD.MAV_CMD_USER_1:
                    id = 128;
                    break;
                case MAV_CMD.MAV_CMD_USER_2:
                    id = 129;
                    break;
                case MAV_CMD.MAV_CMD_USER_3:
                    id = 130;
                    break;
                case MAV_CMD.MAV_CMD_USER_4:
                    id = 131;
                    break;
                case MAV_CMD.MAV_CMD_USER_5:
                    id = 132;
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
                case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
                    id = 17;
                    break;
                case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
                    id = 18;
                    break;
                case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
                    id = 19;
                    break;
                case MAV_CMD.MAV_CMD_NAV_DELAY:
                    id = 20;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
                    id = 21;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAST:
                    id = 22;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DELAY:
                    id = 23;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
                    id = 24;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
                    id = 25;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_YAW:
                    id = 26;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_LAST:
                    id = 27;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_MODE:
                    id = 28;
                    break;
                case MAV_CMD.MAV_CMD_DO_JUMP:
                    id = 29;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
                    id = 30;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_HOME:
                    id = 31;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
                    id = 32;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_RELAY:
                    id = 33;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
                    id = 34;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_SERVO:
                    id = 35;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
                    id = 36;
                    break;
                case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
                    id = 37;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
                    id = 38;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAND_START:
                    id = 39;
                    break;
                case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
                    id = 40;
                    break;
                case MAV_CMD.MAV_CMD_DO_GO_AROUND:
                    id = 41;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPOSITION:
                    id = 42;
                    break;
                case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
                    id = 43;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
                    id = 44;
                    break;
                case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
                    id = 45;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_ROI:
                    id = 46;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
                    id = 47;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
                    id = 48;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
                    id = 49;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                    id = 50;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                    id = 51;
                    break;
                case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
                    id = 52;
                    break;
                case MAV_CMD.MAV_CMD_DO_PARACHUTE:
                    id = 53;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
                    id = 54;
                    break;
                case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
                    id = 55;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                    id = 56;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                    id = 57;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                    id = 58;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                    id = 59;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                    id = 60;
                    break;
                case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                    id = 61;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAST:
                    id = 62;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                    id = 63;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                    id = 64;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                    id = 65;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                    id = 66;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                    id = 67;
                    break;
                case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                    id = 68;
                    break;
                case MAV_CMD.MAV_CMD_MISSION_START:
                    id = 69;
                    break;
                case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                    id = 70;
                    break;
                case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                    id = 71;
                    break;
                case MAV_CMD.MAV_CMD_START_RX_PAIR:
                    id = 72;
                    break;
                case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                    id = 73;
                    break;
                case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                    id = 74;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                    id = 75;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                    id = 76;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                    id = 77;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                    id = 78;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                    id = 79;
                    break;
                case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                    id = 80;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                    id = 81;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                    id = 82;
                    break;
                case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                    id = 83;
                    break;
                case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                    id = 84;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                    id = 85;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                    id = 86;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                    id = 87;
                    break;
                case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                    id = 88;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                    id = 89;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                    id = 90;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                    id = 91;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                    id = 92;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                    id = 93;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_START:
                    id = 94;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_STOP:
                    id = 95;
                    break;
                case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                    id = 96;
                    break;
                case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                    id = 97;
                    break;
                case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                    id = 98;
                    break;
                case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                    id = 99;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                    id = 100;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                    id = 101;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_GATE:
                    id = 102;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                    id = 103;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                    id = 104;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                    id = 105;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                    id = 106;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                    id = 107;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                    id = 108;
                    break;
                case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                    id = 109;
                    break;
                case MAV_CMD.MAV_CMD_DO_NOTHING:
                    id = 110;
                    break;
                case MAV_CMD.MAV_CMD_RETURN_TO_BASE:
                    id = 111;
                    break;
                case MAV_CMD.MAV_CMD_STOP_RETURN_TO_BASE:
                    id = 112;
                    break;
                case MAV_CMD.MAV_CMD_TURN_LIGHT:
                    id = 113;
                    break;
                case MAV_CMD.MAV_CMD_GET_MID_LEVEL_COMMANDS:
                    id = 114;
                    break;
                case MAV_CMD.MAV_CMD_MIDLEVEL_STORAGE:
                    id = 115;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                    id = 116;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                    id = 117;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                    id = 118;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                    id = 119;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                    id = 120;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                    id = 121;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                    id = 122;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                    id = 123;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                    id = 124;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                    id = 125;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                    id = 126;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                    id = 127;
                    break;
                case MAV_CMD.MAV_CMD_USER_1:
                    id = 128;
                    break;
                case MAV_CMD.MAV_CMD_USER_2:
                    id = 129;
                    break;
                case MAV_CMD.MAV_CMD_USER_3:
                    id = 130;
                    break;
                case MAV_CMD.MAV_CMD_USER_4:
                    id = 131;
                    break;
                case MAV_CMD.MAV_CMD_USER_5:
                    id = 132;
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
                case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
                    id = 17;
                    break;
                case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
                    id = 18;
                    break;
                case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
                    id = 19;
                    break;
                case MAV_CMD.MAV_CMD_NAV_DELAY:
                    id = 20;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
                    id = 21;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAST:
                    id = 22;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DELAY:
                    id = 23;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
                    id = 24;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
                    id = 25;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_YAW:
                    id = 26;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_LAST:
                    id = 27;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_MODE:
                    id = 28;
                    break;
                case MAV_CMD.MAV_CMD_DO_JUMP:
                    id = 29;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
                    id = 30;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_HOME:
                    id = 31;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
                    id = 32;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_RELAY:
                    id = 33;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
                    id = 34;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_SERVO:
                    id = 35;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
                    id = 36;
                    break;
                case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
                    id = 37;
                    break;
                case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
                    id = 38;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAND_START:
                    id = 39;
                    break;
                case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
                    id = 40;
                    break;
                case MAV_CMD.MAV_CMD_DO_GO_AROUND:
                    id = 41;
                    break;
                case MAV_CMD.MAV_CMD_DO_REPOSITION:
                    id = 42;
                    break;
                case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
                    id = 43;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
                    id = 44;
                    break;
                case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
                    id = 45;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_ROI:
                    id = 46;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
                    id = 47;
                    break;
                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
                    id = 48;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
                    id = 49;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                    id = 50;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                    id = 51;
                    break;
                case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
                    id = 52;
                    break;
                case MAV_CMD.MAV_CMD_DO_PARACHUTE:
                    id = 53;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
                    id = 54;
                    break;
                case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
                    id = 55;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                    id = 56;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                    id = 57;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                    id = 58;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                    id = 59;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                    id = 60;
                    break;
                case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                    id = 61;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAST:
                    id = 62;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                    id = 63;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                    id = 64;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                    id = 65;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                    id = 66;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                    id = 67;
                    break;
                case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                    id = 68;
                    break;
                case MAV_CMD.MAV_CMD_MISSION_START:
                    id = 69;
                    break;
                case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                    id = 70;
                    break;
                case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                    id = 71;
                    break;
                case MAV_CMD.MAV_CMD_START_RX_PAIR:
                    id = 72;
                    break;
                case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                    id = 73;
                    break;
                case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                    id = 74;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                    id = 75;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                    id = 76;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                    id = 77;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                    id = 78;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                    id = 79;
                    break;
                case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                    id = 80;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                    id = 81;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                    id = 82;
                    break;
                case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                    id = 83;
                    break;
                case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                    id = 84;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                    id = 85;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                    id = 86;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                    id = 87;
                    break;
                case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                    id = 88;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                    id = 89;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                    id = 90;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                    id = 91;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                    id = 92;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                    id = 93;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_START:
                    id = 94;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_STOP:
                    id = 95;
                    break;
                case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                    id = 96;
                    break;
                case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                    id = 97;
                    break;
                case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                    id = 98;
                    break;
                case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                    id = 99;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                    id = 100;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                    id = 101;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_GATE:
                    id = 102;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                    id = 103;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                    id = 104;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                    id = 105;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                    id = 106;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                    id = 107;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                    id = 108;
                    break;
                case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                    id = 109;
                    break;
                case MAV_CMD.MAV_CMD_DO_NOTHING:
                    id = 110;
                    break;
                case MAV_CMD.MAV_CMD_RETURN_TO_BASE:
                    id = 111;
                    break;
                case MAV_CMD.MAV_CMD_STOP_RETURN_TO_BASE:
                    id = 112;
                    break;
                case MAV_CMD.MAV_CMD_TURN_LIGHT:
                    id = 113;
                    break;
                case MAV_CMD.MAV_CMD_GET_MID_LEVEL_COMMANDS:
                    id = 114;
                    break;
                case MAV_CMD.MAV_CMD_MIDLEVEL_STORAGE:
                    id = 115;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                    id = 116;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                    id = 117;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                    id = 118;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                    id = 119;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                    id = 120;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                    id = 121;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                    id = 122;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                    id = 123;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                    id = 124;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                    id = 125;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                    id = 126;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                    id = 127;
                    break;
                case MAV_CMD.MAV_CMD_USER_1:
                    id = 128;
                    break;
                case MAV_CMD.MAV_CMD_USER_2:
                    id = 129;
                    break;
                case MAV_CMD.MAV_CMD_USER_3:
                    id = 130;
                    break;
                case MAV_CMD.MAV_CMD_USER_4:
                    id = 131;
                    break;
                case MAV_CMD.MAV_CMD_USER_5:
                    id = 132;
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
    public static class CPU_LOAD extends GroundControl.CPU_LOAD
    {
        public char batVolt_GET()//Battery Voltage in millivolts
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char sensLoad_GET()//Sensor DSC Load
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char ctrlLoad_GET()//Control DSC Load
        {  return (char)((char) get_bytes(data,  3, 1)); }
    }
    public static class SENSOR_BIAS extends GroundControl.SENSOR_BIAS
    {
        public float axBias_GET()//Accelerometer X bias (m/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float ayBias_GET()//Accelerometer Y bias (m/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float azBias_GET()//Accelerometer Z bias (m/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float gxBias_GET()//Gyro X bias (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float gyBias_GET()//Gyro Y bias (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float gzBias_GET()//Gyro Z bias (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
    }
    public static class DIAGNOSTIC extends GroundControl.DIAGNOSTIC
    {
        public float diagFl1_GET()//Diagnostic float 1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float diagFl2_GET()//Diagnostic float 2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float diagFl3_GET()//Diagnostic float 3
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public short diagSh1_GET()//Diagnostic short 1
        {  return (short)((short) get_bytes(data,  12, 2)); }
        public short diagSh2_GET()//Diagnostic short 2
        {  return (short)((short) get_bytes(data,  14, 2)); }
        public short diagSh3_GET()//Diagnostic short 3
        {  return (short)((short) get_bytes(data,  16, 2)); }
    }
    public static class SLUGS_NAVIGATION extends GroundControl.SLUGS_NAVIGATION
    {
        public char h_c_GET()//Commanded altitude in 0.1 m
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public float u_m_GET()//Measured Airspeed prior to the nav filter in m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  2, 4))); }
        public float phi_c_GET()//Commanded Roll
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  6, 4))); }
        public float theta_c_GET()//Commanded Pitch
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
        public float psiDot_c_GET()//Commanded Turn rate
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public float ay_body_GET()//Y component of the body acceleration
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public float totalDist_GET()//Total Distance to Run on this leg of Navigation
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        public float dist2Go_GET()//Remaining distance to Run on this leg of Navigation
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  26, 4))); }
        public char fromWP_GET()//Origin WP
        {  return (char)((char) get_bytes(data,  30, 1)); }
        public char toWP_GET()//Destination WP
        {  return (char)((char) get_bytes(data,  31, 1)); }
    }
    public static class DATA_LOG extends GroundControl.DATA_LOG
    {
        public float fl_1_GET()//Log value 1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float fl_2_GET()//Log value 2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float fl_3_GET()//Log value 3
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float fl_4_GET()//Log value 4
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float fl_5_GET()//Log value 5
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float fl_6_GET()//Log value 6
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
    }
    public static class GPS_DATE_TIME extends GroundControl.GPS_DATE_TIME
    {
        public char year_GET()//Year reported by Gps
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char month_GET()//Month reported by Gps
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char day_GET()//Day reported by Gps
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char hour_GET()//Hour reported by Gps
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public char min_GET()//Min reported by Gps
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public char sec_GET()//Sec reported by Gps
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public char clockStat_GET()//Clock Status. See table 47 page 211 OEMStar Manual
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public char visSat_GET()//Visible satellites reported by Gps
        {  return (char)((char) get_bytes(data,  7, 1)); }
        public char useSat_GET()//Used satellites in Solution
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public char GppGl_GET()//GPS+GLONASS satellites in Solution
        {  return (char)((char) get_bytes(data,  9, 1)); }
        public char sigUsedMask_GET()//GPS and GLONASS usage mask (bit 0 GPS_used? bit_4 GLONASS_used?)
        {  return (char)((char) get_bytes(data,  10, 1)); }
        public char percentUsed_GET()//Percent used GPS
        {  return (char)((char) get_bytes(data,  11, 1)); }
    }
    public static class MID_LVL_CMDS extends GroundControl.MID_LVL_CMDS
    {
        public char target_GET()//The system setting the commands
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public float hCommand_GET()//Commanded Altitude in meters
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  1, 4))); }
        public float uCommand_GET()//Commanded Airspeed in m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  5, 4))); }
        public float rCommand_GET()//Commanded Turnrate in rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  9, 4))); }
    }
    public static class CTRL_SRFC_PT extends GroundControl.CTRL_SRFC_PT
    {
        public char bitfieldPt_GET()//Bitfield containing the passthrough configuration, see CONTROL_SURFACE_FLAG ENUM.
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char target_GET()//The system setting the commands
        {  return (char)((char) get_bytes(data,  2, 1)); }
    }
    public static class SLUGS_CAMERA_ORDER extends GroundControl.SLUGS_CAMERA_ORDER
    {
        public char target_GET()//The system reporting the action
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public byte pan_GET()//Order the mount to pan: -1 left, 0 No pan motion, +1 right
        {  return (byte)((byte) get_bytes(data,  1, 1)); }
        public byte tilt_GET()//Order the mount to tilt: -1 down, 0 No tilt motion, +1 up
        {  return (byte)((byte) get_bytes(data,  2, 1)); }
        public byte zoom_GET()//Order the zoom values 0 to 10
        {  return (byte)((byte) get_bytes(data,  3, 1)); }
        /**
        *Orders the camera mount to move home. The other fields are ignored when this field is set. 1: move home,
        *	0 ignore*/
        public byte moveHome_GET()
        {  return (byte)((byte) get_bytes(data,  4, 1)); }
    }
    public static class CONTROL_SURFACE extends GroundControl.CONTROL_SURFACE
    {
        public char target_GET()//The system setting the commands
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char idSurface_GET()//ID control surface send 0: throttle 1: aileron 2: elevator 3: rudder
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public float mControl_GET()//Pending
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  2, 4))); }
        public float bControl_GET()//Order to origin
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  6, 4))); }
    }
    public static class SLUGS_MOBILE_LOCATION extends GroundControl.SLUGS_MOBILE_LOCATION
    {
        public char target_GET()//The system reporting the action
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public float latitude_GET()//Mobile Latitude
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  1, 4))); }
        public float longitude_GET()//Mobile Longitude
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  5, 4))); }
    }
    public static class SLUGS_CONFIGURATION_CAMERA extends GroundControl.SLUGS_CONFIGURATION_CAMERA
    {
        public char target_GET()//The system setting the commands
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char idOrder_GET()//ID 0: brightness 1: aperture 2: iris 3: ICR 4: backlight
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char order_GET()//1: up/on 2: down/off 3: auto/reset/no action
        {  return (char)((char) get_bytes(data,  2, 1)); }
    }
    public static class ISR_LOCATION extends GroundControl.ISR_LOCATION
    {
        public char target_GET()//The system reporting the action
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public float latitude_GET()//ISR Latitude
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  1, 4))); }
        public float longitude_GET()//ISR Longitude
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  5, 4))); }
        public float height_GET()//ISR Height
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  9, 4))); }
        public char option1_GET()//Option 1
        {  return (char)((char) get_bytes(data,  13, 1)); }
        public char option2_GET()//Option 2
        {  return (char)((char) get_bytes(data,  14, 1)); }
        public char option3_GET()//Option 3
        {  return (char)((char) get_bytes(data,  15, 1)); }
    }
    public static class VOLT_SENSOR extends GroundControl.VOLT_SENSOR
    {
        public char voltage_GET()//Voltage in uS of PWM. 0 uS = 0V, 20 uS = 21.5V
        {  return (char)((char) get_bytes(data,  0, 2)); }
        /**
        *Depends on the value of r2Type (0) Current consumption in uS of PWM, 20 uS = 90Amp (1) Distance in cm
        *	(2) Distance in cm (3) Absolute valu*/
        public char reading2_GET()
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public char r2Type_GET()//It is the value of reading 2: 0 - Current, 1 - Foreward Sonar, 2 - Back Sonar, 3 - RPM
        {  return (char)((char) get_bytes(data,  4, 1)); }
    }
    public static class PTZ_STATUS extends GroundControl.PTZ_STATUS
    {
        public char zoom_GET()//The actual Zoom Value
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public short pan_GET()//The Pan value in 10ths of degree
        {  return (short)((short) get_bytes(data,  1, 2)); }
        public short tilt_GET()//The Tilt value in 10ths of degree
        {  return (short)((short) get_bytes(data,  3, 2)); }
    }
    public static class UAV_STATUS extends GroundControl.UAV_STATUS
    {
        public char target_GET()//The ID system reporting the action
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public float latitude_GET()//Latitude UAV
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  1, 4))); }
        public float longitude_GET()//Longitude UAV
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  5, 4))); }
        public float altitude_GET()//Altitude UAV
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  9, 4))); }
        public float speed_GET()//Speed UAV
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  13, 4))); }
        public float course_GET()//Course UAV
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  17, 4))); }
    }
    public static class STATUS_GPS extends GroundControl.STATUS_GPS
    {
        public char csFails_GET()//Number of times checksum has failed
        {  return (char)((char) get_bytes(data,  0, 2)); }
        /**
        *The quality indicator, 0=fix not available or invalid, 1=GPS fix, 2=C/A differential GPS, 6=Dead reckoning
        *	mode, 7=Manual input mode (fixed position), 8=Simulator mode, 9= WAAS*/
        public char gpsQuality_GET()
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char msgsType_GET()//Indicates if GN, GL or GP messages are being received
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public char posStatus_GET()//A = data valid, V = data invalid
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public float magVar_GET()//Magnetic variation, degrees
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  5, 4))); }
        /**
        *Magnetic variation direction E/W. Easterly variation (E) subtracts from True course and Westerly variation
        *	(W) adds to True cours*/
        public byte magDir_GET()
        {  return (byte)((byte) get_bytes(data,  9, 1)); }
        /**
        *Positioning system mode indicator. A - Autonomous;D-Differential; E-Estimated (dead reckoning) mode;M-Manual
        *	input; N-Data not vali*/
        public char modeInd_GET()
        {  return (char)((char) get_bytes(data,  10, 1)); }
    }
    public static class NOVATEL_DIAG extends GroundControl.NOVATEL_DIAG
    {
        public char csFails_GET()//Times the CRC has failed since boot
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public long receiverStatus_GET()//Status Bitfield. See table 69 page 350 Novatel OEMstar Manual
        {  return (get_bytes(data,  2, 4)); }
        public char timeStatus_GET()//The Time Status. See Table 8 page 27 Novatel OEMStar Manual
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public char solStatus_GET()//solution Status. See table 44 page 197
        {  return (char)((char) get_bytes(data,  7, 1)); }
        public char posType_GET()//position type. See table 43 page 196
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public char velType_GET()//velocity type. See table 43 page 196
        {  return (char)((char) get_bytes(data,  9, 1)); }
        public float posSolAge_GET()//Age of the position solution in seconds
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
    }
    public static class SENSOR_DIAG extends GroundControl.SENSOR_DIAG
    {
        public float float1_GET()//Float field 1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float float2_GET()//Float field 2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public short int1_GET()//Int 16 field 1
        {  return (short)((short) get_bytes(data,  8, 2)); }
        public byte char1_GET()//Int 8 field 1
        {  return (byte)((byte) get_bytes(data,  10, 1)); }
    }
    public static class BOOT extends GroundControl.BOOT
    {
        public long version_GET()//The onboard software version
        {  return (get_bytes(data,  0, 4)); }
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

        static final Collection<OnReceive.Handler<CPU_LOAD, Channel>> on_CPU_LOAD = new OnReceive<>();
        static final Collection<OnReceive.Handler<SENSOR_BIAS, Channel>> on_SENSOR_BIAS = new OnReceive<>();
        static final Collection<OnReceive.Handler<DIAGNOSTIC, Channel>> on_DIAGNOSTIC = new OnReceive<>();
        static final Collection<OnReceive.Handler<SLUGS_NAVIGATION, Channel>> on_SLUGS_NAVIGATION = new OnReceive<>();
        static final Collection<OnReceive.Handler<DATA_LOG, Channel>> on_DATA_LOG = new OnReceive<>();
        static final Collection<OnReceive.Handler<GPS_DATE_TIME, Channel>> on_GPS_DATE_TIME = new OnReceive<>();
        static final Collection<OnReceive.Handler<MID_LVL_CMDS, Channel>> on_MID_LVL_CMDS = new OnReceive<>();
        static final Collection<OnReceive.Handler<CTRL_SRFC_PT, Channel>> on_CTRL_SRFC_PT = new OnReceive<>();
        static final Collection<OnReceive.Handler<SLUGS_CAMERA_ORDER, Channel>> on_SLUGS_CAMERA_ORDER = new OnReceive<>();
        static final Collection<OnReceive.Handler<CONTROL_SURFACE, Channel>> on_CONTROL_SURFACE = new OnReceive<>();
        static final Collection<OnReceive.Handler<SLUGS_MOBILE_LOCATION, Channel>> on_SLUGS_MOBILE_LOCATION = new OnReceive<>();
        static final Collection<OnReceive.Handler<SLUGS_CONFIGURATION_CAMERA, Channel>> on_SLUGS_CONFIGURATION_CAMERA = new OnReceive<>();
        static final Collection<OnReceive.Handler<ISR_LOCATION, Channel>> on_ISR_LOCATION = new OnReceive<>();
        static final Collection<OnReceive.Handler<VOLT_SENSOR, Channel>> on_VOLT_SENSOR = new OnReceive<>();
        static final Collection<OnReceive.Handler<PTZ_STATUS, Channel>> on_PTZ_STATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<UAV_STATUS, Channel>> on_UAV_STATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<STATUS_GPS, Channel>> on_STATUS_GPS = new OnReceive<>();
        static final Collection<OnReceive.Handler<NOVATEL_DIAG, Channel>> on_NOVATEL_DIAG = new OnReceive<>();
        static final Collection<OnReceive.Handler<SENSOR_DIAG, Channel>> on_SENSOR_DIAG = new OnReceive<>();
        static final Collection<OnReceive.Handler<BOOT, Channel>> on_BOOT = new OnReceive<>();
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
                case 170:
                    if(pack == null) return new CPU_LOAD();
                    ((OnReceive) on_CPU_LOAD).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 172:
                    if(pack == null) return new SENSOR_BIAS();
                    ((OnReceive) on_SENSOR_BIAS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 173:
                    if(pack == null) return new DIAGNOSTIC();
                    ((OnReceive) on_DIAGNOSTIC).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 176:
                    if(pack == null) return new SLUGS_NAVIGATION();
                    ((OnReceive) on_SLUGS_NAVIGATION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 177:
                    if(pack == null) return new DATA_LOG();
                    ((OnReceive) on_DATA_LOG).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 179:
                    if(pack == null) return new GPS_DATE_TIME();
                    ((OnReceive) on_GPS_DATE_TIME).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 180:
                    if(pack == null) return new MID_LVL_CMDS();
                    ((OnReceive) on_MID_LVL_CMDS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 181:
                    if(pack == null) return new CTRL_SRFC_PT();
                    ((OnReceive) on_CTRL_SRFC_PT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 184:
                    if(pack == null) return new SLUGS_CAMERA_ORDER();
                    ((OnReceive) on_SLUGS_CAMERA_ORDER).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 185:
                    if(pack == null) return new CONTROL_SURFACE();
                    ((OnReceive) on_CONTROL_SURFACE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 186:
                    if(pack == null) return new SLUGS_MOBILE_LOCATION();
                    ((OnReceive) on_SLUGS_MOBILE_LOCATION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 188:
                    if(pack == null) return new SLUGS_CONFIGURATION_CAMERA();
                    ((OnReceive) on_SLUGS_CONFIGURATION_CAMERA).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 189:
                    if(pack == null) return new ISR_LOCATION();
                    ((OnReceive) on_ISR_LOCATION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 191:
                    if(pack == null) return new VOLT_SENSOR();
                    ((OnReceive) on_VOLT_SENSOR).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 192:
                    if(pack == null) return new PTZ_STATUS();
                    ((OnReceive) on_PTZ_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 193:
                    if(pack == null) return new UAV_STATUS();
                    ((OnReceive) on_UAV_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 194:
                    if(pack == null) return new STATUS_GPS();
                    ((OnReceive) on_STATUS_GPS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 195:
                    if(pack == null) return new NOVATEL_DIAG();
                    ((OnReceive) on_NOVATEL_DIAG).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 196:
                    if(pack == null) return new SENSOR_DIAG();
                    ((OnReceive) on_SENSOR_DIAG).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 197:
                    if(pack == null) return new BOOT();
                    ((OnReceive) on_BOOT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
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
            assert(pack.mavlink_version_GET() == (char)249);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_SUBMARINE);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_SLUGS);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_ACTIVE);
            assert(pack.custom_mode_GET() == 2584613292L);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.system_status_SET(MAV_STATE.MAV_STATE_ACTIVE) ;
        p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED) ;
        p0.mavlink_version_SET((char)249) ;
        p0.custom_mode_SET(2584613292L) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_SUBMARINE) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_SLUGS) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.onboard_control_sensors_enabled_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG);
            assert(pack.drop_rate_comm_GET() == (char)50382);
            assert(pack.errors_count2_GET() == (char)43789);
            assert(pack.errors_count3_GET() == (char)6463);
            assert(pack.current_battery_GET() == (short)11593);
            assert(pack.onboard_control_sensors_health_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2);
            assert(pack.voltage_battery_GET() == (char)60066);
            assert(pack.errors_count1_GET() == (char)4617);
            assert(pack.errors_comm_GET() == (char)53907);
            assert(pack.battery_remaining_GET() == (byte) - 29);
            assert(pack.onboard_control_sensors_present_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION);
            assert(pack.errors_count4_GET() == (char)38053);
            assert(pack.load_GET() == (char)25278);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG) ;
        p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2) ;
        p1.errors_count1_SET((char)4617) ;
        p1.errors_comm_SET((char)53907) ;
        p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION) ;
        p1.errors_count3_SET((char)6463) ;
        p1.voltage_battery_SET((char)60066) ;
        p1.current_battery_SET((short)11593) ;
        p1.battery_remaining_SET((byte) - 29) ;
        p1.errors_count2_SET((char)43789) ;
        p1.drop_rate_comm_SET((char)50382) ;
        p1.load_SET((char)25278) ;
        p1.errors_count4_SET((char)38053) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 5131606190029096161L);
            assert(pack.time_boot_ms_GET() == 3475521252L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(3475521252L) ;
        p2.time_unix_usec_SET(5131606190029096161L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == -2.1016978E38F);
            assert(pack.afx_GET() == -1.8253063E38F);
            assert(pack.yaw_rate_GET() == -1.2184336E38F);
            assert(pack.afz_GET() == -1.770666E38F);
            assert(pack.yaw_GET() == 3.1530945E38F);
            assert(pack.afy_GET() == 2.333605E38F);
            assert(pack.time_boot_ms_GET() == 187180571L);
            assert(pack.y_GET() == -1.2411267E38F);
            assert(pack.z_GET() == -4.579745E37F);
            assert(pack.vz_GET() == -1.6299426E38F);
            assert(pack.x_GET() == -2.5298412E38F);
            assert(pack.type_mask_GET() == (char)48095);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.vx_GET() == -2.2374082E38F);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.yaw_SET(3.1530945E38F) ;
        p3.afy_SET(2.333605E38F) ;
        p3.afx_SET(-1.8253063E38F) ;
        p3.x_SET(-2.5298412E38F) ;
        p3.vz_SET(-1.6299426E38F) ;
        p3.afz_SET(-1.770666E38F) ;
        p3.time_boot_ms_SET(187180571L) ;
        p3.vx_SET(-2.2374082E38F) ;
        p3.y_SET(-1.2411267E38F) ;
        p3.vy_SET(-2.1016978E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p3.yaw_rate_SET(-1.2184336E38F) ;
        p3.type_mask_SET((char)48095) ;
        p3.z_SET(-4.579745E37F) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 2230880162L);
            assert(pack.target_component_GET() == (char)1);
            assert(pack.time_usec_GET() == 3821363073181350434L);
            assert(pack.target_system_GET() == (char)35);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.time_usec_SET(3821363073181350434L) ;
        p4.target_system_SET((char)35) ;
        p4.target_component_SET((char)1) ;
        p4.seq_SET(2230880162L) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.passkey_LEN(ph) == 4);
            assert(pack.passkey_TRY(ph).equals("kdxo"));
            assert(pack.version_GET() == (char)184);
            assert(pack.target_system_GET() == (char)84);
            assert(pack.control_request_GET() == (char)67);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.passkey_SET("kdxo", PH) ;
        p5.version_SET((char)184) ;
        p5.control_request_SET((char)67) ;
        p5.target_system_SET((char)84) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.gcs_system_id_GET() == (char)190);
            assert(pack.control_request_GET() == (char)235);
            assert(pack.ack_GET() == (char)244);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.ack_SET((char)244) ;
        p6.control_request_SET((char)235) ;
        p6.gcs_system_id_SET((char)190) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 27);
            assert(pack.key_TRY(ph).equals("dmwercuiafplNCthzbnhpcouGge"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("dmwercuiafplNCthzbnhpcouGge", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_MANUAL_ARMED);
            assert(pack.custom_mode_GET() == 2564466669L);
            assert(pack.target_system_GET() == (char)170);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_MANUAL_ARMED) ;
        p11.target_system_SET((char)170) ;
        p11.custom_mode_SET(2564466669L) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)60);
            assert(pack.param_index_GET() == (short) -11957);
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("tskonxkz"));
            assert(pack.target_system_GET() == (char)146);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_system_SET((char)146) ;
        p20.param_index_SET((short) -11957) ;
        p20.target_component_SET((char)60) ;
        p20.param_id_SET("tskonxkz", PH) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)2);
            assert(pack.target_system_GET() == (char)35);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)2) ;
        p21.target_system_SET((char)35) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)5122);
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("fiwoopuqp"));
            assert(pack.param_index_GET() == (char)19220);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
            assert(pack.param_value_GET() == 2.265318E38F);
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_id_SET("fiwoopuqp", PH) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16) ;
        p22.param_count_SET((char)5122) ;
        p22.param_value_SET(2.265318E38F) ;
        p22.param_index_SET((char)19220) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
            assert(pack.target_component_GET() == (char)237);
            assert(pack.target_system_GET() == (char)60);
            assert(pack.param_value_GET() == 2.8961656E38F);
            assert(pack.param_id_LEN(ph) == 6);
            assert(pack.param_id_TRY(ph).equals("yosqqm"));
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.target_component_SET((char)237) ;
        p23.param_value_SET(2.8961656E38F) ;
        p23.param_id_SET("yosqqm", PH) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32) ;
        p23.target_system_SET((char)60) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.hdg_acc_TRY(ph) == 3106333903L);
            assert(pack.vel_acc_TRY(ph) == 1944824034L);
            assert(pack.lon_GET() == -1748521676);
            assert(pack.satellites_visible_GET() == (char)194);
            assert(pack.h_acc_TRY(ph) == 2670328766L);
            assert(pack.time_usec_GET() == 5092055866833157387L);
            assert(pack.lat_GET() == -1396337755);
            assert(pack.eph_GET() == (char)15486);
            assert(pack.vel_GET() == (char)55981);
            assert(pack.epv_GET() == (char)57550);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.alt_GET() == -1030536667);
            assert(pack.v_acc_TRY(ph) == 2940016923L);
            assert(pack.cog_GET() == (char)58746);
            assert(pack.alt_ellipsoid_TRY(ph) == -1306713250);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.h_acc_SET(2670328766L, PH) ;
        p24.v_acc_SET(2940016923L, PH) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p24.satellites_visible_SET((char)194) ;
        p24.lon_SET(-1748521676) ;
        p24.alt_SET(-1030536667) ;
        p24.cog_SET((char)58746) ;
        p24.eph_SET((char)15486) ;
        p24.alt_ellipsoid_SET(-1306713250, PH) ;
        p24.time_usec_SET(5092055866833157387L) ;
        p24.epv_SET((char)57550) ;
        p24.hdg_acc_SET(3106333903L, PH) ;
        p24.vel_acc_SET(1944824034L, PH) ;
        p24.lat_SET(-1396337755) ;
        p24.vel_SET((char)55981) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.satellites_visible_GET() == (char)178);
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)89, (char)72, (char)133, (char)18, (char)185, (char)104, (char)124, (char)145, (char)138, (char)206, (char)22, (char)239, (char)182, (char)36, (char)173, (char)144, (char)48, (char)46, (char)85, (char)21}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)186, (char)38, (char)5, (char)132, (char)193, (char)119, (char)42, (char)231, (char)88, (char)13, (char)123, (char)19, (char)145, (char)222, (char)100, (char)81, (char)25, (char)145, (char)178, (char)181}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)92, (char)69, (char)95, (char)0, (char)161, (char)172, (char)243, (char)83, (char)63, (char)214, (char)192, (char)223, (char)184, (char)243, (char)30, (char)163, (char)166, (char)47, (char)203, (char)220}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)193, (char)222, (char)232, (char)32, (char)185, (char)179, (char)143, (char)22, (char)94, (char)122, (char)27, (char)97, (char)97, (char)96, (char)192, (char)205, (char)170, (char)143, (char)163, (char)234}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)243, (char)240, (char)112, (char)198, (char)128, (char)182, (char)60, (char)142, (char)214, (char)148, (char)92, (char)3, (char)217, (char)72, (char)247, (char)56, (char)186, (char)151, (char)168, (char)43}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_azimuth_SET(new char[] {(char)193, (char)222, (char)232, (char)32, (char)185, (char)179, (char)143, (char)22, (char)94, (char)122, (char)27, (char)97, (char)97, (char)96, (char)192, (char)205, (char)170, (char)143, (char)163, (char)234}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)243, (char)240, (char)112, (char)198, (char)128, (char)182, (char)60, (char)142, (char)214, (char)148, (char)92, (char)3, (char)217, (char)72, (char)247, (char)56, (char)186, (char)151, (char)168, (char)43}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)89, (char)72, (char)133, (char)18, (char)185, (char)104, (char)124, (char)145, (char)138, (char)206, (char)22, (char)239, (char)182, (char)36, (char)173, (char)144, (char)48, (char)46, (char)85, (char)21}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)92, (char)69, (char)95, (char)0, (char)161, (char)172, (char)243, (char)83, (char)63, (char)214, (char)192, (char)223, (char)184, (char)243, (char)30, (char)163, (char)166, (char)47, (char)203, (char)220}, 0) ;
        p25.satellites_visible_SET((char)178) ;
        p25.satellite_used_SET(new char[] {(char)186, (char)38, (char)5, (char)132, (char)193, (char)119, (char)42, (char)231, (char)88, (char)13, (char)123, (char)19, (char)145, (char)222, (char)100, (char)81, (char)25, (char)145, (char)178, (char)181}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3650305415L);
            assert(pack.zgyro_GET() == (short) -10521);
            assert(pack.xgyro_GET() == (short)20109);
            assert(pack.yacc_GET() == (short)22399);
            assert(pack.zmag_GET() == (short) -30537);
            assert(pack.zacc_GET() == (short) -15524);
            assert(pack.xmag_GET() == (short)24924);
            assert(pack.ymag_GET() == (short)8870);
            assert(pack.xacc_GET() == (short) -18196);
            assert(pack.ygyro_GET() == (short) -27874);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.ymag_SET((short)8870) ;
        p26.xmag_SET((short)24924) ;
        p26.yacc_SET((short)22399) ;
        p26.ygyro_SET((short) -27874) ;
        p26.zgyro_SET((short) -10521) ;
        p26.xgyro_SET((short)20109) ;
        p26.xacc_SET((short) -18196) ;
        p26.time_boot_ms_SET(3650305415L) ;
        p26.zacc_SET((short) -15524) ;
        p26.zmag_SET((short) -30537) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == (short)11703);
            assert(pack.ygyro_GET() == (short) -32691);
            assert(pack.yacc_GET() == (short) -14266);
            assert(pack.zmag_GET() == (short)1529);
            assert(pack.zgyro_GET() == (short)12708);
            assert(pack.ymag_GET() == (short) -7899);
            assert(pack.xacc_GET() == (short) -25067);
            assert(pack.time_usec_GET() == 5649012384278733648L);
            assert(pack.xgyro_GET() == (short)9600);
            assert(pack.xmag_GET() == (short) -11266);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.ymag_SET((short) -7899) ;
        p27.zgyro_SET((short)12708) ;
        p27.xgyro_SET((short)9600) ;
        p27.yacc_SET((short) -14266) ;
        p27.time_usec_SET(5649012384278733648L) ;
        p27.zmag_SET((short)1529) ;
        p27.ygyro_SET((short) -32691) ;
        p27.xacc_SET((short) -25067) ;
        p27.xmag_SET((short) -11266) ;
        p27.zacc_SET((short)11703) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == (short)7769);
            assert(pack.time_usec_GET() == 2369599400648319924L);
            assert(pack.press_diff2_GET() == (short)3281);
            assert(pack.press_diff1_GET() == (short)32633);
            assert(pack.temperature_GET() == (short)23229);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_diff2_SET((short)3281) ;
        p28.press_abs_SET((short)7769) ;
        p28.temperature_SET((short)23229) ;
        p28.press_diff1_SET((short)32633) ;
        p28.time_usec_SET(2369599400648319924L) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -1.7612883E38F);
            assert(pack.press_diff_GET() == 1.3117213E38F);
            assert(pack.temperature_GET() == (short) -26741);
            assert(pack.time_boot_ms_GET() == 2679378453L);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.time_boot_ms_SET(2679378453L) ;
        p29.press_diff_SET(1.3117213E38F) ;
        p29.temperature_SET((short) -26741) ;
        p29.press_abs_SET(-1.7612883E38F) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.pitchspeed_GET() == 1.7082365E38F);
            assert(pack.pitch_GET() == -2.0622329E38F);
            assert(pack.roll_GET() == 2.7002803E38F);
            assert(pack.yaw_GET() == 7.2147746E37F);
            assert(pack.time_boot_ms_GET() == 3971503004L);
            assert(pack.rollspeed_GET() == -2.2029857E38F);
            assert(pack.yawspeed_GET() == -3.035693E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.yaw_SET(7.2147746E37F) ;
        p30.pitch_SET(-2.0622329E38F) ;
        p30.roll_SET(2.7002803E38F) ;
        p30.time_boot_ms_SET(3971503004L) ;
        p30.pitchspeed_SET(1.7082365E38F) ;
        p30.yawspeed_SET(-3.035693E38F) ;
        p30.rollspeed_SET(-2.2029857E38F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.pitchspeed_GET() == -7.7133035E37F);
            assert(pack.time_boot_ms_GET() == 171426216L);
            assert(pack.rollspeed_GET() == -6.8604135E37F);
            assert(pack.q4_GET() == 2.8225033E36F);
            assert(pack.q3_GET() == 2.2476628E38F);
            assert(pack.q1_GET() == -1.8681868E38F);
            assert(pack.yawspeed_GET() == -3.112112E38F);
            assert(pack.q2_GET() == -2.4294217E38F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.yawspeed_SET(-3.112112E38F) ;
        p31.time_boot_ms_SET(171426216L) ;
        p31.rollspeed_SET(-6.8604135E37F) ;
        p31.pitchspeed_SET(-7.7133035E37F) ;
        p31.q1_SET(-1.8681868E38F) ;
        p31.q2_SET(-2.4294217E38F) ;
        p31.q4_SET(2.8225033E36F) ;
        p31.q3_SET(2.2476628E38F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == 1.5730541E38F);
            assert(pack.time_boot_ms_GET() == 3107212193L);
            assert(pack.z_GET() == -3.354022E38F);
            assert(pack.vx_GET() == 2.1306976E38F);
            assert(pack.vz_GET() == -1.2798651E38F);
            assert(pack.x_GET() == -2.0964028E38F);
            assert(pack.y_GET() == 2.7751748E38F);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.vz_SET(-1.2798651E38F) ;
        p32.x_SET(-2.0964028E38F) ;
        p32.y_SET(2.7751748E38F) ;
        p32.z_SET(-3.354022E38F) ;
        p32.vy_SET(1.5730541E38F) ;
        p32.vx_SET(2.1306976E38F) ;
        p32.time_boot_ms_SET(3107212193L) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -1094074352);
            assert(pack.time_boot_ms_GET() == 192422364L);
            assert(pack.lat_GET() == -497608065);
            assert(pack.vz_GET() == (short) -3957);
            assert(pack.alt_GET() == 526849566);
            assert(pack.hdg_GET() == (char)30266);
            assert(pack.relative_alt_GET() == -1125183410);
            assert(pack.vy_GET() == (short) -7044);
            assert(pack.vx_GET() == (short)30820);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.vy_SET((short) -7044) ;
        p33.vx_SET((short)30820) ;
        p33.lon_SET(-1094074352) ;
        p33.relative_alt_SET(-1125183410) ;
        p33.hdg_SET((char)30266) ;
        p33.time_boot_ms_SET(192422364L) ;
        p33.alt_SET(526849566) ;
        p33.vz_SET((short) -3957) ;
        p33.lat_SET(-497608065) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.port_GET() == (char)85);
            assert(pack.rssi_GET() == (char)188);
            assert(pack.chan6_scaled_GET() == (short)6849);
            assert(pack.chan1_scaled_GET() == (short)13911);
            assert(pack.chan5_scaled_GET() == (short) -1161);
            assert(pack.time_boot_ms_GET() == 2343978569L);
            assert(pack.chan4_scaled_GET() == (short)25198);
            assert(pack.chan3_scaled_GET() == (short)24847);
            assert(pack.chan8_scaled_GET() == (short) -8079);
            assert(pack.chan2_scaled_GET() == (short) -6606);
            assert(pack.chan7_scaled_GET() == (short) -3323);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.port_SET((char)85) ;
        p34.chan8_scaled_SET((short) -8079) ;
        p34.chan1_scaled_SET((short)13911) ;
        p34.chan3_scaled_SET((short)24847) ;
        p34.chan4_scaled_SET((short)25198) ;
        p34.chan6_scaled_SET((short)6849) ;
        p34.chan5_scaled_SET((short) -1161) ;
        p34.rssi_SET((char)188) ;
        p34.time_boot_ms_SET(2343978569L) ;
        p34.chan2_scaled_SET((short) -6606) ;
        p34.chan7_scaled_SET((short) -3323) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan7_raw_GET() == (char)16038);
            assert(pack.chan5_raw_GET() == (char)32039);
            assert(pack.chan3_raw_GET() == (char)54782);
            assert(pack.chan6_raw_GET() == (char)19814);
            assert(pack.chan8_raw_GET() == (char)29148);
            assert(pack.time_boot_ms_GET() == 2125888782L);
            assert(pack.rssi_GET() == (char)203);
            assert(pack.chan1_raw_GET() == (char)8018);
            assert(pack.chan4_raw_GET() == (char)36124);
            assert(pack.port_GET() == (char)134);
            assert(pack.chan2_raw_GET() == (char)47259);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.time_boot_ms_SET(2125888782L) ;
        p35.chan6_raw_SET((char)19814) ;
        p35.chan5_raw_SET((char)32039) ;
        p35.chan7_raw_SET((char)16038) ;
        p35.chan1_raw_SET((char)8018) ;
        p35.port_SET((char)134) ;
        p35.chan3_raw_SET((char)54782) ;
        p35.chan2_raw_SET((char)47259) ;
        p35.chan4_raw_SET((char)36124) ;
        p35.rssi_SET((char)203) ;
        p35.chan8_raw_SET((char)29148) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo9_raw_TRY(ph) == (char)28878);
            assert(pack.servo11_raw_TRY(ph) == (char)44730);
            assert(pack.servo3_raw_GET() == (char)25016);
            assert(pack.servo7_raw_GET() == (char)34391);
            assert(pack.servo10_raw_TRY(ph) == (char)38153);
            assert(pack.servo15_raw_TRY(ph) == (char)2431);
            assert(pack.servo12_raw_TRY(ph) == (char)23923);
            assert(pack.servo14_raw_TRY(ph) == (char)26945);
            assert(pack.servo4_raw_GET() == (char)38216);
            assert(pack.servo6_raw_GET() == (char)20672);
            assert(pack.port_GET() == (char)87);
            assert(pack.servo1_raw_GET() == (char)38157);
            assert(pack.servo13_raw_TRY(ph) == (char)12647);
            assert(pack.time_usec_GET() == 4148467220L);
            assert(pack.servo5_raw_GET() == (char)11725);
            assert(pack.servo8_raw_GET() == (char)20948);
            assert(pack.servo2_raw_GET() == (char)57641);
            assert(pack.servo16_raw_TRY(ph) == (char)49373);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo4_raw_SET((char)38216) ;
        p36.servo1_raw_SET((char)38157) ;
        p36.servo9_raw_SET((char)28878, PH) ;
        p36.servo6_raw_SET((char)20672) ;
        p36.servo15_raw_SET((char)2431, PH) ;
        p36.servo10_raw_SET((char)38153, PH) ;
        p36.servo11_raw_SET((char)44730, PH) ;
        p36.servo7_raw_SET((char)34391) ;
        p36.time_usec_SET(4148467220L) ;
        p36.servo2_raw_SET((char)57641) ;
        p36.servo8_raw_SET((char)20948) ;
        p36.servo12_raw_SET((char)23923, PH) ;
        p36.servo5_raw_SET((char)11725) ;
        p36.servo3_raw_SET((char)25016) ;
        p36.servo16_raw_SET((char)49373, PH) ;
        p36.port_SET((char)87) ;
        p36.servo14_raw_SET((char)26945, PH) ;
        p36.servo13_raw_SET((char)12647, PH) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_index_GET() == (short) -23969);
            assert(pack.target_component_GET() == (char)172);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.start_index_GET() == (short)25945);
            assert(pack.target_system_GET() == (char)85);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.end_index_SET((short) -23969) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p37.target_component_SET((char)172) ;
        p37.start_index_SET((short)25945) ;
        p37.target_system_SET((char)85) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)110);
            assert(pack.end_index_GET() == (short) -25113);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_component_GET() == (char)176);
            assert(pack.start_index_GET() == (short) -19521);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.end_index_SET((short) -25113) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p38.start_index_SET((short) -19521) ;
        p38.target_system_SET((char)110) ;
        p38.target_component_SET((char)176) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.param2_GET() == -2.066455E38F);
            assert(pack.param4_GET() == -3.3621885E38F);
            assert(pack.current_GET() == (char)63);
            assert(pack.x_GET() == 7.902708E37F);
            assert(pack.target_system_GET() == (char)120);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.z_GET() == 1.0723731E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.y_GET() == 2.1261482E38F);
            assert(pack.seq_GET() == (char)11726);
            assert(pack.param1_GET() == 3.0003785E38F);
            assert(pack.target_component_GET() == (char)10);
            assert(pack.autocontinue_GET() == (char)2);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_FOLLOW);
            assert(pack.param3_GET() == 1.1102251E38F);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.target_system_SET((char)120) ;
        p39.autocontinue_SET((char)2) ;
        p39.param4_SET(-3.3621885E38F) ;
        p39.y_SET(2.1261482E38F) ;
        p39.current_SET((char)63) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p39.target_component_SET((char)10) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p39.command_SET(MAV_CMD.MAV_CMD_NAV_FOLLOW) ;
        p39.z_SET(1.0723731E38F) ;
        p39.x_SET(7.902708E37F) ;
        p39.param3_SET(1.1102251E38F) ;
        p39.param1_SET(3.0003785E38F) ;
        p39.seq_SET((char)11726) ;
        p39.param2_SET(-2.066455E38F) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)21);
            assert(pack.seq_GET() == (char)56411);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)54);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p40.target_system_SET((char)54) ;
        p40.target_component_SET((char)21) ;
        p40.seq_SET((char)56411) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)64250);
            assert(pack.target_component_GET() == (char)42);
            assert(pack.target_system_GET() == (char)181);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.seq_SET((char)64250) ;
        p41.target_component_SET((char)42) ;
        p41.target_system_SET((char)181) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)34812);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)34812) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)104);
            assert(pack.target_component_GET() == (char)74);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_component_SET((char)74) ;
        p43.target_system_SET((char)104) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)177);
            assert(pack.target_system_GET() == (char)38);
            assert(pack.count_GET() == (char)5842);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.count_SET((char)5842) ;
        p44.target_component_SET((char)177) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p44.target_system_SET((char)38) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)244);
            assert(pack.target_system_GET() == (char)202);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p45.target_component_SET((char)244) ;
        p45.target_system_SET((char)202) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)10076);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)10076) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)155);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)176);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED) ;
        p47.target_system_SET((char)176) ;
        p47.target_component_SET((char)155) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == 164941114);
            assert(pack.target_system_GET() == (char)127);
            assert(pack.latitude_GET() == -1362509915);
            assert(pack.longitude_GET() == 377131951);
            assert(pack.time_usec_TRY(ph) == 3872311856839089057L);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.target_system_SET((char)127) ;
        p48.latitude_SET(-1362509915) ;
        p48.longitude_SET(377131951) ;
        p48.time_usec_SET(3872311856839089057L, PH) ;
        p48.altitude_SET(164941114) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == 998171834);
            assert(pack.time_usec_TRY(ph) == 7551830856885141132L);
            assert(pack.latitude_GET() == -1157634618);
            assert(pack.longitude_GET() == -991572216);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.latitude_SET(-1157634618) ;
        p49.altitude_SET(998171834) ;
        p49.longitude_SET(-991572216) ;
        p49.time_usec_SET(7551830856885141132L, PH) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("f"));
            assert(pack.target_system_GET() == (char)53);
            assert(pack.param_value_max_GET() == -2.1128385E38F);
            assert(pack.scale_GET() == -2.6488072E38F);
            assert(pack.param_index_GET() == (short)23020);
            assert(pack.param_value_min_GET() == -2.437287E38F);
            assert(pack.param_value0_GET() == -1.2690001E38F);
            assert(pack.target_component_GET() == (char)239);
            assert(pack.parameter_rc_channel_index_GET() == (char)48);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.target_component_SET((char)239) ;
        p50.target_system_SET((char)53) ;
        p50.param_id_SET("f", PH) ;
        p50.parameter_rc_channel_index_SET((char)48) ;
        p50.param_value_min_SET(-2.437287E38F) ;
        p50.param_value_max_SET(-2.1128385E38F) ;
        p50.scale_SET(-2.6488072E38F) ;
        p50.param_value0_SET(-1.2690001E38F) ;
        p50.param_index_SET((short)23020) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)129);
            assert(pack.target_component_GET() == (char)36);
            assert(pack.seq_GET() == (char)54139);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_system_SET((char)129) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p51.target_component_SET((char)36) ;
        p51.seq_SET((char)54139) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2z_GET() == 1.1334384E38F);
            assert(pack.p1z_GET() == 3.1154185E38F);
            assert(pack.p1y_GET() == 3.0966984E38F);
            assert(pack.target_system_GET() == (char)140);
            assert(pack.p2y_GET() == 9.033395E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.p2x_GET() == -2.459666E38F);
            assert(pack.target_component_GET() == (char)75);
            assert(pack.p1x_GET() == -2.4883561E38F);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p2y_SET(9.033395E37F) ;
        p54.target_component_SET((char)75) ;
        p54.p1x_SET(-2.4883561E38F) ;
        p54.p1y_SET(3.0966984E38F) ;
        p54.p2z_SET(1.1334384E38F) ;
        p54.target_system_SET((char)140) ;
        p54.p1z_SET(3.1154185E38F) ;
        p54.p2x_SET(-2.459666E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2y_GET() == 9.890447E37F);
            assert(pack.p1z_GET() == -3.3524272E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.p2x_GET() == -2.587199E38F);
            assert(pack.p1y_GET() == 1.2335112E38F);
            assert(pack.p1x_GET() == 3.2926723E38F);
            assert(pack.p2z_GET() == 3.012918E37F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p2x_SET(-2.587199E38F) ;
        p55.p2z_SET(3.012918E37F) ;
        p55.p2y_SET(9.890447E37F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p55.p1x_SET(3.2926723E38F) ;
        p55.p1y_SET(1.2335112E38F) ;
        p55.p1z_SET(-3.3524272E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {4.0228882E37F, 1.9119587E38F, -6.625083E37F, 2.3912217E38F}));
            assert(pack.pitchspeed_GET() == -1.4742856E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.3969048E38F, 1.3052225E37F, 2.287793E38F, 2.8929507E38F, 7.8509876E37F, -2.69313E38F, 1.5019661E38F, 3.333199E38F, 3.77487E37F}));
            assert(pack.yawspeed_GET() == 8.321852E37F);
            assert(pack.rollspeed_GET() == 1.880943E38F);
            assert(pack.time_usec_GET() == 1131552303351575211L);
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.yawspeed_SET(8.321852E37F) ;
        p61.covariance_SET(new float[] {2.3969048E38F, 1.3052225E37F, 2.287793E38F, 2.8929507E38F, 7.8509876E37F, -2.69313E38F, 1.5019661E38F, 3.333199E38F, 3.77487E37F}, 0) ;
        p61.rollspeed_SET(1.880943E38F) ;
        p61.time_usec_SET(1131552303351575211L) ;
        p61.q_SET(new float[] {4.0228882E37F, 1.9119587E38F, -6.625083E37F, 2.3912217E38F}, 0) ;
        p61.pitchspeed_SET(-1.4742856E38F) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.target_bearing_GET() == (short)13935);
            assert(pack.wp_dist_GET() == (char)42979);
            assert(pack.nav_pitch_GET() == -1.1111889E38F);
            assert(pack.alt_error_GET() == -1.798237E38F);
            assert(pack.nav_bearing_GET() == (short) -22686);
            assert(pack.nav_roll_GET() == 3.2180004E38F);
            assert(pack.aspd_error_GET() == -2.3557207E38F);
            assert(pack.xtrack_error_GET() == -1.5224395E38F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.target_bearing_SET((short)13935) ;
        p62.nav_roll_SET(3.2180004E38F) ;
        p62.wp_dist_SET((char)42979) ;
        p62.xtrack_error_SET(-1.5224395E38F) ;
        p62.nav_bearing_SET((short) -22686) ;
        p62.alt_error_SET(-1.798237E38F) ;
        p62.nav_pitch_SET(-1.1111889E38F) ;
        p62.aspd_error_SET(-2.3557207E38F) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == 2.0782174E38F);
            assert(pack.vz_GET() == 2.0481834E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.823993E38F, 1.7869396E37F, 9.770588E37F, 3.2338951E38F, -3.122102E38F, 1.697912E38F, 5.406837E37F, 1.7619662E38F, -1.5005197E38F, -1.4576573E38F, -1.2057879E38F, -5.6843694E37F, 1.9082738E38F, 7.7221137E37F, -1.6808559E38F, -3.3769539E38F, 3.0655049E38F, -6.671476E37F, -1.9357106E38F, 2.581458E38F, 6.393521E37F, 2.6923966E38F, 1.8952896E38F, 1.3568627E38F, -2.8233642E37F, -1.9386937E38F, -2.4712526E38F, 5.063107E37F, -1.7428865E38F, -1.6683769E38F, 9.96293E37F, 3.0968321E37F, 2.265929E38F, 2.2739968E38F, -3.195585E38F, -2.0353388E37F}));
            assert(pack.alt_GET() == -225529876);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
            assert(pack.lon_GET() == 1460228160);
            assert(pack.vx_GET() == -3.0827638E38F);
            assert(pack.lat_GET() == -695189789);
            assert(pack.time_usec_GET() == 7830625300157011979L);
            assert(pack.relative_alt_GET() == -1311073637);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION) ;
        p63.lon_SET(1460228160) ;
        p63.vx_SET(-3.0827638E38F) ;
        p63.vy_SET(2.0782174E38F) ;
        p63.relative_alt_SET(-1311073637) ;
        p63.covariance_SET(new float[] {-2.823993E38F, 1.7869396E37F, 9.770588E37F, 3.2338951E38F, -3.122102E38F, 1.697912E38F, 5.406837E37F, 1.7619662E38F, -1.5005197E38F, -1.4576573E38F, -1.2057879E38F, -5.6843694E37F, 1.9082738E38F, 7.7221137E37F, -1.6808559E38F, -3.3769539E38F, 3.0655049E38F, -6.671476E37F, -1.9357106E38F, 2.581458E38F, 6.393521E37F, 2.6923966E38F, 1.8952896E38F, 1.3568627E38F, -2.8233642E37F, -1.9386937E38F, -2.4712526E38F, 5.063107E37F, -1.7428865E38F, -1.6683769E38F, 9.96293E37F, 3.0968321E37F, 2.265929E38F, 2.2739968E38F, -3.195585E38F, -2.0353388E37F}, 0) ;
        p63.time_usec_SET(7830625300157011979L) ;
        p63.vz_SET(2.0481834E38F) ;
        p63.alt_SET(-225529876) ;
        p63.lat_SET(-695189789) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.ax_GET() == -1.6137751E38F);
            assert(pack.vz_GET() == 1.4757768E38F);
            assert(pack.y_GET() == -1.8032764E38F);
            assert(pack.time_usec_GET() == 6427519037239868669L);
            assert(pack.vx_GET() == -1.3051713E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-9.400001E37F, 3.706585E37F, -1.2372319E38F, -2.005954E38F, -1.7775808E38F, -1.2173536E38F, -2.088122E38F, -2.9663316E38F, -1.8686663E38F, -7.88572E37F, -1.9164163E38F, 2.7385782E38F, -1.1914973E38F, -1.4937355E38F, -4.093669E36F, -3.2905324E37F, 3.3182357E37F, 2.7533992E38F, 6.2791485E37F, -2.4987042E38F, 1.8854564E38F, -2.9656976E38F, 4.7791153E37F, -2.0799625E38F, -1.0482709E38F, 3.9108666E37F, 3.1569023E38F, -1.2611878E38F, -1.8789973E38F, -3.0310124E38F, -2.8996112E38F, -2.1430244E38F, -1.4364332E38F, 1.0297632E38F, 6.448897E36F, 2.764011E38F, -3.1236243E38F, -1.5159932E38F, 1.4413764E38F, -2.4410164E38F, 3.2415913E38F, -2.3211952E38F, 2.6594746E38F, 2.9389522E38F, -1.0154117E38F}));
            assert(pack.x_GET() == 1.7455459E37F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
            assert(pack.az_GET() == -2.6079729E38F);
            assert(pack.z_GET() == -6.0776965E37F);
            assert(pack.vy_GET() == -3.1092615E38F);
            assert(pack.ay_GET() == -1.7902328E38F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS) ;
        p64.y_SET(-1.8032764E38F) ;
        p64.ay_SET(-1.7902328E38F) ;
        p64.vz_SET(1.4757768E38F) ;
        p64.time_usec_SET(6427519037239868669L) ;
        p64.vy_SET(-3.1092615E38F) ;
        p64.x_SET(1.7455459E37F) ;
        p64.covariance_SET(new float[] {-9.400001E37F, 3.706585E37F, -1.2372319E38F, -2.005954E38F, -1.7775808E38F, -1.2173536E38F, -2.088122E38F, -2.9663316E38F, -1.8686663E38F, -7.88572E37F, -1.9164163E38F, 2.7385782E38F, -1.1914973E38F, -1.4937355E38F, -4.093669E36F, -3.2905324E37F, 3.3182357E37F, 2.7533992E38F, 6.2791485E37F, -2.4987042E38F, 1.8854564E38F, -2.9656976E38F, 4.7791153E37F, -2.0799625E38F, -1.0482709E38F, 3.9108666E37F, 3.1569023E38F, -1.2611878E38F, -1.8789973E38F, -3.0310124E38F, -2.8996112E38F, -2.1430244E38F, -1.4364332E38F, 1.0297632E38F, 6.448897E36F, 2.764011E38F, -3.1236243E38F, -1.5159932E38F, 1.4413764E38F, -2.4410164E38F, 3.2415913E38F, -2.3211952E38F, 2.6594746E38F, 2.9389522E38F, -1.0154117E38F}, 0) ;
        p64.ax_SET(-1.6137751E38F) ;
        p64.az_SET(-2.6079729E38F) ;
        p64.z_SET(-6.0776965E37F) ;
        p64.vx_SET(-1.3051713E38F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.rssi_GET() == (char)234);
            assert(pack.chan11_raw_GET() == (char)48276);
            assert(pack.chan12_raw_GET() == (char)15575);
            assert(pack.chan14_raw_GET() == (char)49563);
            assert(pack.chan7_raw_GET() == (char)2874);
            assert(pack.chan8_raw_GET() == (char)31008);
            assert(pack.chan4_raw_GET() == (char)28555);
            assert(pack.chancount_GET() == (char)50);
            assert(pack.chan3_raw_GET() == (char)42497);
            assert(pack.chan6_raw_GET() == (char)39338);
            assert(pack.chan17_raw_GET() == (char)24286);
            assert(pack.chan16_raw_GET() == (char)4054);
            assert(pack.chan18_raw_GET() == (char)17509);
            assert(pack.time_boot_ms_GET() == 3195762348L);
            assert(pack.chan5_raw_GET() == (char)59638);
            assert(pack.chan1_raw_GET() == (char)1028);
            assert(pack.chan15_raw_GET() == (char)15286);
            assert(pack.chan13_raw_GET() == (char)47271);
            assert(pack.chan10_raw_GET() == (char)16900);
            assert(pack.chan9_raw_GET() == (char)32496);
            assert(pack.chan2_raw_GET() == (char)57971);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan11_raw_SET((char)48276) ;
        p65.chan13_raw_SET((char)47271) ;
        p65.chan18_raw_SET((char)17509) ;
        p65.chan2_raw_SET((char)57971) ;
        p65.chan4_raw_SET((char)28555) ;
        p65.chan12_raw_SET((char)15575) ;
        p65.time_boot_ms_SET(3195762348L) ;
        p65.chan16_raw_SET((char)4054) ;
        p65.chan17_raw_SET((char)24286) ;
        p65.chancount_SET((char)50) ;
        p65.chan7_raw_SET((char)2874) ;
        p65.chan3_raw_SET((char)42497) ;
        p65.chan10_raw_SET((char)16900) ;
        p65.chan1_raw_SET((char)1028) ;
        p65.chan8_raw_SET((char)31008) ;
        p65.chan14_raw_SET((char)49563) ;
        p65.chan6_raw_SET((char)39338) ;
        p65.chan5_raw_SET((char)59638) ;
        p65.chan9_raw_SET((char)32496) ;
        p65.chan15_raw_SET((char)15286) ;
        p65.rssi_SET((char)234) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)183);
            assert(pack.target_component_GET() == (char)64);
            assert(pack.req_message_rate_GET() == (char)17920);
            assert(pack.start_stop_GET() == (char)11);
            assert(pack.req_stream_id_GET() == (char)121);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_system_SET((char)183) ;
        p66.target_component_SET((char)64) ;
        p66.req_stream_id_SET((char)121) ;
        p66.start_stop_SET((char)11) ;
        p66.req_message_rate_SET((char)17920) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.message_rate_GET() == (char)53290);
            assert(pack.on_off_GET() == (char)249);
            assert(pack.stream_id_GET() == (char)141);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.message_rate_SET((char)53290) ;
        p67.stream_id_SET((char)141) ;
        p67.on_off_SET((char)249) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == (short)18886);
            assert(pack.buttons_GET() == (char)52833);
            assert(pack.target_GET() == (char)125);
            assert(pack.y_GET() == (short)18200);
            assert(pack.r_GET() == (short) -8753);
            assert(pack.x_GET() == (short)20637);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.r_SET((short) -8753) ;
        p69.y_SET((short)18200) ;
        p69.z_SET((short)18886) ;
        p69.x_SET((short)20637) ;
        p69.buttons_SET((char)52833) ;
        p69.target_SET((char)125) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan8_raw_GET() == (char)1660);
            assert(pack.chan2_raw_GET() == (char)31020);
            assert(pack.chan3_raw_GET() == (char)52811);
            assert(pack.chan6_raw_GET() == (char)26940);
            assert(pack.target_component_GET() == (char)255);
            assert(pack.chan5_raw_GET() == (char)45462);
            assert(pack.chan1_raw_GET() == (char)48443);
            assert(pack.chan4_raw_GET() == (char)47393);
            assert(pack.target_system_GET() == (char)87);
            assert(pack.chan7_raw_GET() == (char)7606);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan4_raw_SET((char)47393) ;
        p70.target_component_SET((char)255) ;
        p70.chan1_raw_SET((char)48443) ;
        p70.chan5_raw_SET((char)45462) ;
        p70.chan8_raw_SET((char)1660) ;
        p70.chan7_raw_SET((char)7606) ;
        p70.chan2_raw_SET((char)31020) ;
        p70.chan6_raw_SET((char)26940) ;
        p70.chan3_raw_SET((char)52811) ;
        p70.target_system_SET((char)87) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST);
            assert(pack.seq_GET() == (char)61546);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.param1_GET() == -4.4300793E37F);
            assert(pack.y_GET() == 1388772123);
            assert(pack.param4_GET() == 1.0691107E38F);
            assert(pack.target_component_GET() == (char)232);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.autocontinue_GET() == (char)125);
            assert(pack.param3_GET() == 1.010348E38F);
            assert(pack.current_GET() == (char)240);
            assert(pack.param2_GET() == 1.1649802E37F);
            assert(pack.x_GET() == 1197606971);
            assert(pack.target_system_GET() == (char)73);
            assert(pack.z_GET() == -2.3364703E38F);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.current_SET((char)240) ;
        p73.target_component_SET((char)232) ;
        p73.param3_SET(1.010348E38F) ;
        p73.z_SET(-2.3364703E38F) ;
        p73.autocontinue_SET((char)125) ;
        p73.param4_SET(1.0691107E38F) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p73.command_SET(MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST) ;
        p73.x_SET(1197606971) ;
        p73.y_SET(1388772123) ;
        p73.param1_SET(-4.4300793E37F) ;
        p73.seq_SET((char)61546) ;
        p73.param2_SET(1.1649802E37F) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p73.target_system_SET((char)73) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -2.1356818E38F);
            assert(pack.groundspeed_GET() == 2.9526684E38F);
            assert(pack.heading_GET() == (short) -10727);
            assert(pack.throttle_GET() == (char)45584);
            assert(pack.airspeed_GET() == 1.7772208E38F);
            assert(pack.climb_GET() == -2.623233E38F);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.throttle_SET((char)45584) ;
        p74.alt_SET(-2.1356818E38F) ;
        p74.heading_SET((short) -10727) ;
        p74.climb_SET(-2.623233E38F) ;
        p74.groundspeed_SET(2.9526684E38F) ;
        p74.airspeed_SET(1.7772208E38F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)89);
            assert(pack.current_GET() == (char)100);
            assert(pack.param2_GET() == -2.3253489E38F);
            assert(pack.y_GET() == 1397996538);
            assert(pack.param3_GET() == -4.7942186E37F);
            assert(pack.x_GET() == 1726919286);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.param1_GET() == 2.9136794E37F);
            assert(pack.param4_GET() == -2.584648E38F);
            assert(pack.z_GET() == -3.1887343E37F);
            assert(pack.target_component_GET() == (char)157);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_CONDITION_GATE);
            assert(pack.autocontinue_GET() == (char)101);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.command_SET(MAV_CMD.MAV_CMD_CONDITION_GATE) ;
        p75.x_SET(1726919286) ;
        p75.param2_SET(-2.3253489E38F) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p75.param1_SET(2.9136794E37F) ;
        p75.target_system_SET((char)89) ;
        p75.target_component_SET((char)157) ;
        p75.current_SET((char)100) ;
        p75.y_SET(1397996538) ;
        p75.param3_SET(-4.7942186E37F) ;
        p75.z_SET(-3.1887343E37F) ;
        p75.param4_SET(-2.584648E38F) ;
        p75.autocontinue_SET((char)101) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param7_GET() == -1.6668315E38F);
            assert(pack.param1_GET() == -6.3053783E37F);
            assert(pack.target_system_GET() == (char)233);
            assert(pack.param2_GET() == 2.586005E38F);
            assert(pack.param5_GET() == -1.7567763E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_MIDLEVEL_STORAGE);
            assert(pack.param3_GET() == -1.6985725E38F);
            assert(pack.target_component_GET() == (char)218);
            assert(pack.confirmation_GET() == (char)142);
            assert(pack.param6_GET() == -1.5233516E38F);
            assert(pack.param4_GET() == 1.3965983E38F);
        });
        COMMAND_LONG p76 = new COMMAND_LONG();
        PH.setPack(p76);
        p76.param5_SET(-1.7567763E38F) ;
        p76.confirmation_SET((char)142) ;
        p76.param4_SET(1.3965983E38F) ;
        p76.target_component_SET((char)218) ;
        p76.param3_SET(-1.6985725E38F) ;
        p76.param6_SET(-1.5233516E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_MIDLEVEL_STORAGE) ;
        p76.param7_SET(-1.6668315E38F) ;
        p76.target_system_SET((char)233) ;
        p76.param2_SET(2.586005E38F) ;
        p76.param1_SET(-6.3053783E37F) ;
        TestChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_TRY(ph) == (char)97);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_FAILED);
            assert(pack.target_component_TRY(ph) == (char)147);
            assert(pack.result_param2_TRY(ph) == 959243330);
            assert(pack.progress_TRY(ph) == (char)240);
        });
        COMMAND_ACK p77 = new COMMAND_ACK();
        PH.setPack(p77);
        p77.target_system_SET((char)97, PH) ;
        p77.target_component_SET((char)147, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_FAILED) ;
        p77.result_param2_SET(959243330, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS) ;
        p77.progress_SET((char)240, PH) ;
        TestChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3164457660L);
            assert(pack.thrust_GET() == 2.6130124E38F);
            assert(pack.roll_GET() == 2.0322885E38F);
            assert(pack.yaw_GET() == 2.987872E38F);
            assert(pack.mode_switch_GET() == (char)133);
            assert(pack.manual_override_switch_GET() == (char)99);
            assert(pack.pitch_GET() == -1.0712977E38F);
        });
        MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.mode_switch_SET((char)133) ;
        p81.roll_SET(2.0322885E38F) ;
        p81.thrust_SET(2.6130124E38F) ;
        p81.pitch_SET(-1.0712977E38F) ;
        p81.time_boot_ms_SET(3164457660L) ;
        p81.yaw_SET(2.987872E38F) ;
        p81.manual_override_switch_SET((char)99) ;
        TestChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)80);
            assert(pack.body_roll_rate_GET() == -2.9567436E37F);
            assert(pack.target_component_GET() == (char)96);
            assert(pack.body_pitch_rate_GET() == 1.2843851E38F);
            assert(pack.body_yaw_rate_GET() == -3.691611E37F);
            assert(pack.type_mask_GET() == (char)106);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.5759737E38F, 3.020788E38F, -1.8946452E38F, -3.8862598E37F}));
            assert(pack.time_boot_ms_GET() == 2747741372L);
            assert(pack.thrust_GET() == -7.3229594E37F);
        });
        SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.body_pitch_rate_SET(1.2843851E38F) ;
        p82.q_SET(new float[] {2.5759737E38F, 3.020788E38F, -1.8946452E38F, -3.8862598E37F}, 0) ;
        p82.target_system_SET((char)80) ;
        p82.time_boot_ms_SET(2747741372L) ;
        p82.target_component_SET((char)96) ;
        p82.type_mask_SET((char)106) ;
        p82.body_roll_rate_SET(-2.9567436E37F) ;
        p82.thrust_SET(-7.3229594E37F) ;
        p82.body_yaw_rate_SET(-3.691611E37F) ;
        TestChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 4127578407L);
            assert(pack.body_pitch_rate_GET() == -2.0459027E38F);
            assert(pack.thrust_GET() == 2.9059572E37F);
            assert(pack.type_mask_GET() == (char)183);
            assert(Arrays.equals(pack.q_GET(),  new float[] {9.739445E37F, -4.3910104E37F, -5.5759954E37F, 1.82007E38F}));
            assert(pack.body_roll_rate_GET() == 2.7628676E38F);
            assert(pack.body_yaw_rate_GET() == -2.4426554E38F);
        });
        GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.body_yaw_rate_SET(-2.4426554E38F) ;
        p83.thrust_SET(2.9059572E37F) ;
        p83.body_pitch_rate_SET(-2.0459027E38F) ;
        p83.body_roll_rate_SET(2.7628676E38F) ;
        p83.time_boot_ms_SET(4127578407L) ;
        p83.type_mask_SET((char)183) ;
        p83.q_SET(new float[] {9.739445E37F, -4.3910104E37F, -5.5759954E37F, 1.82007E38F}, 0) ;
        CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 7.461339E36F);
            assert(pack.x_GET() == 6.871916E37F);
            assert(pack.yaw_rate_GET() == -7.6881097E37F);
            assert(pack.target_system_GET() == (char)160);
            assert(pack.afy_GET() == -2.9053428E38F);
            assert(pack.vz_GET() == 2.876955E38F);
            assert(pack.type_mask_GET() == (char)64193);
            assert(pack.z_GET() == -1.8562327E38F);
            assert(pack.vy_GET() == -1.0362519E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.y_GET() == -9.246665E37F);
            assert(pack.afx_GET() == -3.0621786E37F);
            assert(pack.afz_GET() == 2.6947337E38F);
            assert(pack.time_boot_ms_GET() == 3317378508L);
            assert(pack.target_component_GET() == (char)209);
            assert(pack.vx_GET() == 7.4317984E37F);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.x_SET(6.871916E37F) ;
        p84.y_SET(-9.246665E37F) ;
        p84.target_system_SET((char)160) ;
        p84.target_component_SET((char)209) ;
        p84.afz_SET(2.6947337E38F) ;
        p84.time_boot_ms_SET(3317378508L) ;
        p84.vz_SET(2.876955E38F) ;
        p84.afy_SET(-2.9053428E38F) ;
        p84.type_mask_SET((char)64193) ;
        p84.vx_SET(7.4317984E37F) ;
        p84.z_SET(-1.8562327E38F) ;
        p84.yaw_SET(7.461339E36F) ;
        p84.yaw_rate_SET(-7.6881097E37F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p84.vy_SET(-1.0362519E38F) ;
        p84.afx_SET(-3.0621786E37F) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)230);
            assert(pack.afz_GET() == -1.9556936E38F);
            assert(pack.vz_GET() == -3.0145321E38F);
            assert(pack.vx_GET() == -1.7021562E38F);
            assert(pack.alt_GET() == 1.7380735E38F);
            assert(pack.vy_GET() == -2.3527084E38F);
            assert(pack.lon_int_GET() == 1876987301);
            assert(pack.afy_GET() == 1.2483965E38F);
            assert(pack.afx_GET() == 6.8906155E37F);
            assert(pack.yaw_GET() == -9.767584E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.target_component_GET() == (char)151);
            assert(pack.lat_int_GET() == 1603119421);
            assert(pack.yaw_rate_GET() == -3.046839E38F);
            assert(pack.type_mask_GET() == (char)54934);
            assert(pack.time_boot_ms_GET() == 3767576019L);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.vx_SET(-1.7021562E38F) ;
        p86.afx_SET(6.8906155E37F) ;
        p86.vz_SET(-3.0145321E38F) ;
        p86.lon_int_SET(1876987301) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p86.target_system_SET((char)230) ;
        p86.lat_int_SET(1603119421) ;
        p86.afz_SET(-1.9556936E38F) ;
        p86.alt_SET(1.7380735E38F) ;
        p86.time_boot_ms_SET(3767576019L) ;
        p86.target_component_SET((char)151) ;
        p86.yaw_rate_SET(-3.046839E38F) ;
        p86.yaw_SET(-9.767584E37F) ;
        p86.type_mask_SET((char)54934) ;
        p86.vy_SET(-2.3527084E38F) ;
        p86.afy_SET(1.2483965E38F) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.afx_GET() == -1.6682472E38F);
            assert(pack.afz_GET() == -2.5260163E38F);
            assert(pack.vx_GET() == -1.914698E38F);
            assert(pack.type_mask_GET() == (char)10996);
            assert(pack.lat_int_GET() == 140944803);
            assert(pack.yaw_rate_GET() == -2.8562253E38F);
            assert(pack.vz_GET() == -8.507143E37F);
            assert(pack.afy_GET() == 1.5715778E38F);
            assert(pack.alt_GET() == -1.5212398E38F);
            assert(pack.vy_GET() == 1.976227E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.lon_int_GET() == -1907692530);
            assert(pack.time_boot_ms_GET() == 1104396142L);
            assert(pack.yaw_GET() == -1.4531248E38F);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.time_boot_ms_SET(1104396142L) ;
        p87.afx_SET(-1.6682472E38F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p87.afz_SET(-2.5260163E38F) ;
        p87.afy_SET(1.5715778E38F) ;
        p87.yaw_SET(-1.4531248E38F) ;
        p87.lat_int_SET(140944803) ;
        p87.lon_int_SET(-1907692530) ;
        p87.vz_SET(-8.507143E37F) ;
        p87.alt_SET(-1.5212398E38F) ;
        p87.type_mask_SET((char)10996) ;
        p87.vx_SET(-1.914698E38F) ;
        p87.vy_SET(1.976227E38F) ;
        p87.yaw_rate_SET(-2.8562253E38F) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -2.0872964E38F);
            assert(pack.time_boot_ms_GET() == 1733384057L);
            assert(pack.roll_GET() == 2.104103E38F);
            assert(pack.z_GET() == -9.044592E37F);
            assert(pack.pitch_GET() == 3.3592739E38F);
            assert(pack.x_GET() == -3.1936887E38F);
            assert(pack.yaw_GET() == 3.3955378E38F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.time_boot_ms_SET(1733384057L) ;
        p89.y_SET(-2.0872964E38F) ;
        p89.roll_SET(2.104103E38F) ;
        p89.x_SET(-3.1936887E38F) ;
        p89.yaw_SET(3.3955378E38F) ;
        p89.pitch_SET(3.3592739E38F) ;
        p89.z_SET(-9.044592E37F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1138260678);
            assert(pack.vx_GET() == (short)32414);
            assert(pack.yawspeed_GET() == 1.8074633E38F);
            assert(pack.zacc_GET() == (short)3066);
            assert(pack.pitch_GET() == -3.0043252E38F);
            assert(pack.vz_GET() == (short) -13139);
            assert(pack.vy_GET() == (short) -6673);
            assert(pack.lon_GET() == 1352618272);
            assert(pack.yaw_GET() == 1.6769769E38F);
            assert(pack.rollspeed_GET() == 1.021106E38F);
            assert(pack.alt_GET() == -2126471206);
            assert(pack.xacc_GET() == (short) -22548);
            assert(pack.time_usec_GET() == 2749684255330608395L);
            assert(pack.roll_GET() == 1.6136897E38F);
            assert(pack.yacc_GET() == (short)28124);
            assert(pack.pitchspeed_GET() == 8.467339E37F);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.roll_SET(1.6136897E38F) ;
        p90.lat_SET(-1138260678) ;
        p90.pitch_SET(-3.0043252E38F) ;
        p90.time_usec_SET(2749684255330608395L) ;
        p90.rollspeed_SET(1.021106E38F) ;
        p90.vx_SET((short)32414) ;
        p90.zacc_SET((short)3066) ;
        p90.alt_SET(-2126471206) ;
        p90.xacc_SET((short) -22548) ;
        p90.yaw_SET(1.6769769E38F) ;
        p90.vz_SET((short) -13139) ;
        p90.vy_SET((short) -6673) ;
        p90.lon_SET(1352618272) ;
        p90.yacc_SET((short)28124) ;
        p90.yawspeed_SET(1.8074633E38F) ;
        p90.pitchspeed_SET(8.467339E37F) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.aux4_GET() == 2.7878878E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
            assert(pack.aux2_GET() == 9.797776E37F);
            assert(pack.pitch_elevator_GET() == 2.0286044E38F);
            assert(pack.time_usec_GET() == 2510615281155737881L);
            assert(pack.yaw_rudder_GET() == -3.320281E38F);
            assert(pack.roll_ailerons_GET() == -1.3550859E38F);
            assert(pack.aux3_GET() == 2.1820143E38F);
            assert(pack.aux1_GET() == -4.698025E37F);
            assert(pack.throttle_GET() == 2.937839E38F);
            assert(pack.nav_mode_GET() == (char)50);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.roll_ailerons_SET(-1.3550859E38F) ;
        p91.throttle_SET(2.937839E38F) ;
        p91.nav_mode_SET((char)50) ;
        p91.yaw_rudder_SET(-3.320281E38F) ;
        p91.time_usec_SET(2510615281155737881L) ;
        p91.aux2_SET(9.797776E37F) ;
        p91.aux3_SET(2.1820143E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_GUIDED_DISARMED) ;
        p91.pitch_elevator_SET(2.0286044E38F) ;
        p91.aux1_SET(-4.698025E37F) ;
        p91.aux4_SET(2.7878878E38F) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.rssi_GET() == (char)165);
            assert(pack.chan4_raw_GET() == (char)23245);
            assert(pack.chan8_raw_GET() == (char)42930);
            assert(pack.chan11_raw_GET() == (char)4849);
            assert(pack.chan5_raw_GET() == (char)23071);
            assert(pack.chan10_raw_GET() == (char)22739);
            assert(pack.chan9_raw_GET() == (char)3489);
            assert(pack.chan3_raw_GET() == (char)59515);
            assert(pack.chan7_raw_GET() == (char)18679);
            assert(pack.chan2_raw_GET() == (char)22924);
            assert(pack.chan6_raw_GET() == (char)60922);
            assert(pack.chan12_raw_GET() == (char)15020);
            assert(pack.chan1_raw_GET() == (char)6232);
            assert(pack.time_usec_GET() == 2354267604199508323L);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.time_usec_SET(2354267604199508323L) ;
        p92.chan10_raw_SET((char)22739) ;
        p92.chan4_raw_SET((char)23245) ;
        p92.chan8_raw_SET((char)42930) ;
        p92.chan11_raw_SET((char)4849) ;
        p92.chan6_raw_SET((char)60922) ;
        p92.chan9_raw_SET((char)3489) ;
        p92.chan7_raw_SET((char)18679) ;
        p92.chan5_raw_SET((char)23071) ;
        p92.chan2_raw_SET((char)22924) ;
        p92.chan1_raw_SET((char)6232) ;
        p92.chan3_raw_SET((char)59515) ;
        p92.rssi_SET((char)165) ;
        p92.chan12_raw_SET((char)15020) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == 5315767003408798398L);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_ARMED);
            assert(pack.time_usec_GET() == 1299908078938548074L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {3.0414956E38F, -1.0095173E38F, 1.6938974E38F, 2.4530781E38F, 4.5456575E36F, -7.7716305E36F, -3.7174549E37F, 6.343964E37F, -3.0909447E38F, 1.6583139E38F, -2.7332591E38F, -2.0459453E38F, 2.6212733E37F, -2.5983464E38F, 6.52155E37F, -3.721525E37F}));
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.mode_SET(MAV_MODE.MAV_MODE_GUIDED_ARMED) ;
        p93.controls_SET(new float[] {3.0414956E38F, -1.0095173E38F, 1.6938974E38F, 2.4530781E38F, 4.5456575E36F, -7.7716305E36F, -3.7174549E37F, 6.343964E37F, -3.0909447E38F, 1.6583139E38F, -2.7332591E38F, -2.0459453E38F, 2.6212733E37F, -2.5983464E38F, 6.52155E37F, -3.721525E37F}, 0) ;
        p93.time_usec_SET(1299908078938548074L) ;
        p93.flags_SET(5315767003408798398L) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_x_GET() == (short)13305);
            assert(pack.flow_y_GET() == (short) -31476);
            assert(pack.flow_comp_m_x_GET() == 2.118103E38F);
            assert(pack.ground_distance_GET() == -1.6260213E38F);
            assert(pack.quality_GET() == (char)171);
            assert(pack.flow_rate_x_TRY(ph) == -2.6900448E38F);
            assert(pack.flow_rate_y_TRY(ph) == -1.1213035E38F);
            assert(pack.sensor_id_GET() == (char)9);
            assert(pack.flow_comp_m_y_GET() == 2.1328615E38F);
            assert(pack.time_usec_GET() == 2432402666465331315L);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.time_usec_SET(2432402666465331315L) ;
        p100.flow_rate_x_SET(-2.6900448E38F, PH) ;
        p100.flow_comp_m_x_SET(2.118103E38F) ;
        p100.flow_comp_m_y_SET(2.1328615E38F) ;
        p100.flow_y_SET((short) -31476) ;
        p100.flow_x_SET((short)13305) ;
        p100.flow_rate_y_SET(-1.1213035E38F, PH) ;
        p100.ground_distance_SET(-1.6260213E38F) ;
        p100.sensor_id_SET((char)9) ;
        p100.quality_SET((char)171) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -2.808979E38F);
            assert(pack.z_GET() == -2.8234224E38F);
            assert(pack.x_GET() == 1.8477137E38F);
            assert(pack.roll_GET() == 1.3817423E38F);
            assert(pack.pitch_GET() == -2.0853925E38F);
            assert(pack.yaw_GET() == 2.4425203E38F);
            assert(pack.usec_GET() == 6575480858378286593L);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.z_SET(-2.8234224E38F) ;
        p101.usec_SET(6575480858378286593L) ;
        p101.y_SET(-2.808979E38F) ;
        p101.x_SET(1.8477137E38F) ;
        p101.yaw_SET(2.4425203E38F) ;
        p101.roll_SET(1.3817423E38F) ;
        p101.pitch_SET(-2.0853925E38F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -1.1444577E37F);
            assert(pack.pitch_GET() == -2.6009022E38F);
            assert(pack.yaw_GET() == -2.6332013E38F);
            assert(pack.roll_GET() == 3.0111172E38F);
            assert(pack.y_GET() == 1.3919849E38F);
            assert(pack.usec_GET() == 5708219096058505358L);
            assert(pack.x_GET() == -2.5910006E38F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.pitch_SET(-2.6009022E38F) ;
        p102.yaw_SET(-2.6332013E38F) ;
        p102.y_SET(1.3919849E38F) ;
        p102.roll_SET(3.0111172E38F) ;
        p102.x_SET(-2.5910006E38F) ;
        p102.z_SET(-1.1444577E37F) ;
        p102.usec_SET(5708219096058505358L) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -1.5896811E38F);
            assert(pack.y_GET() == -2.624113E37F);
            assert(pack.usec_GET() == 6789741171038176643L);
            assert(pack.x_GET() == 8.0042045E37F);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(6789741171038176643L) ;
        p103.z_SET(-1.5896811E38F) ;
        p103.x_SET(8.0042045E37F) ;
        p103.y_SET(-2.624113E37F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 2.6264933E38F);
            assert(pack.y_GET() == -1.5052245E38F);
            assert(pack.x_GET() == 2.1703284E38F);
            assert(pack.z_GET() == 2.8502477E38F);
            assert(pack.pitch_GET() == -3.1453348E38F);
            assert(pack.usec_GET() == 1717822356031017478L);
            assert(pack.roll_GET() == 9.480633E37F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.x_SET(2.1703284E38F) ;
        p104.y_SET(-1.5052245E38F) ;
        p104.roll_SET(9.480633E37F) ;
        p104.yaw_SET(2.6264933E38F) ;
        p104.usec_SET(1717822356031017478L) ;
        p104.z_SET(2.8502477E38F) ;
        p104.pitch_SET(-3.1453348E38F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == 2.4865259E38F);
            assert(pack.temperature_GET() == 1.3395002E38F);
            assert(pack.diff_pressure_GET() == 2.931573E38F);
            assert(pack.zmag_GET() == 2.8743482E38F);
            assert(pack.abs_pressure_GET() == -2.8205388E38F);
            assert(pack.xmag_GET() == 1.6518349E38F);
            assert(pack.fields_updated_GET() == (char)25405);
            assert(pack.xgyro_GET() == 5.524553E37F);
            assert(pack.yacc_GET() == 1.5378653E38F);
            assert(pack.pressure_alt_GET() == 1.0747074E38F);
            assert(pack.xacc_GET() == -2.6838763E38F);
            assert(pack.ymag_GET() == -2.5948414E38F);
            assert(pack.zacc_GET() == -4.5911567E37F);
            assert(pack.zgyro_GET() == -1.2318827E38F);
            assert(pack.time_usec_GET() == 2477266556937500831L);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.zgyro_SET(-1.2318827E38F) ;
        p105.time_usec_SET(2477266556937500831L) ;
        p105.zacc_SET(-4.5911567E37F) ;
        p105.zmag_SET(2.8743482E38F) ;
        p105.ygyro_SET(2.4865259E38F) ;
        p105.xacc_SET(-2.6838763E38F) ;
        p105.abs_pressure_SET(-2.8205388E38F) ;
        p105.ymag_SET(-2.5948414E38F) ;
        p105.yacc_SET(1.5378653E38F) ;
        p105.diff_pressure_SET(2.931573E38F) ;
        p105.temperature_SET(1.3395002E38F) ;
        p105.fields_updated_SET((char)25405) ;
        p105.xgyro_SET(5.524553E37F) ;
        p105.xmag_SET(1.6518349E38F) ;
        p105.pressure_alt_SET(1.0747074E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integrated_zgyro_GET() == 1.3526934E38F);
            assert(pack.integrated_xgyro_GET() == -3.3714967E38F);
            assert(pack.integrated_y_GET() == -1.0709983E38F);
            assert(pack.integration_time_us_GET() == 2930750258L);
            assert(pack.time_delta_distance_us_GET() == 1353754067L);
            assert(pack.integrated_ygyro_GET() == -3.0709426E38F);
            assert(pack.sensor_id_GET() == (char)26);
            assert(pack.time_usec_GET() == 101267414661668255L);
            assert(pack.quality_GET() == (char)185);
            assert(pack.temperature_GET() == (short)12690);
            assert(pack.distance_GET() == -3.2558679E38F);
            assert(pack.integrated_x_GET() == 1.3846811E37F);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integrated_ygyro_SET(-3.0709426E38F) ;
        p106.temperature_SET((short)12690) ;
        p106.quality_SET((char)185) ;
        p106.integrated_x_SET(1.3846811E37F) ;
        p106.time_usec_SET(101267414661668255L) ;
        p106.time_delta_distance_us_SET(1353754067L) ;
        p106.integrated_zgyro_SET(1.3526934E38F) ;
        p106.sensor_id_SET((char)26) ;
        p106.integration_time_us_SET(2930750258L) ;
        p106.distance_SET(-3.2558679E38F) ;
        p106.integrated_y_SET(-1.0709983E38F) ;
        p106.integrated_xgyro_SET(-3.3714967E38F) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == -2.3423273E38F);
            assert(pack.ygyro_GET() == -6.0208373E37F);
            assert(pack.temperature_GET() == -2.9555335E38F);
            assert(pack.xmag_GET() == -1.1747235E38F);
            assert(pack.zacc_GET() == 9.098745E37F);
            assert(pack.xgyro_GET() == -1.0176283E38F);
            assert(pack.fields_updated_GET() == 1846200935L);
            assert(pack.ymag_GET() == -1.916846E38F);
            assert(pack.time_usec_GET() == 1779565015531804292L);
            assert(pack.xacc_GET() == 2.2674837E38F);
            assert(pack.diff_pressure_GET() == -3.9142708E37F);
            assert(pack.zmag_GET() == -6.1277317E37F);
            assert(pack.pressure_alt_GET() == -1.9905013E38F);
            assert(pack.abs_pressure_GET() == 8.3046823E37F);
            assert(pack.yacc_GET() == 2.3852958E37F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.diff_pressure_SET(-3.9142708E37F) ;
        p107.ygyro_SET(-6.0208373E37F) ;
        p107.zmag_SET(-6.1277317E37F) ;
        p107.xmag_SET(-1.1747235E38F) ;
        p107.xgyro_SET(-1.0176283E38F) ;
        p107.zgyro_SET(-2.3423273E38F) ;
        p107.yacc_SET(2.3852958E37F) ;
        p107.ymag_SET(-1.916846E38F) ;
        p107.temperature_SET(-2.9555335E38F) ;
        p107.time_usec_SET(1779565015531804292L) ;
        p107.zacc_SET(9.098745E37F) ;
        p107.abs_pressure_SET(8.3046823E37F) ;
        p107.xacc_SET(2.2674837E38F) ;
        p107.fields_updated_SET(1846200935L) ;
        p107.pressure_alt_SET(-1.9905013E38F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.std_dev_horz_GET() == 1.6368786E36F);
            assert(pack.xgyro_GET() == -2.3810312E38F);
            assert(pack.pitch_GET() == 9.367871E37F);
            assert(pack.lat_GET() == 1.273605E38F);
            assert(pack.xacc_GET() == 3.203731E38F);
            assert(pack.zacc_GET() == -1.2336657E38F);
            assert(pack.q2_GET() == 2.475256E37F);
            assert(pack.ygyro_GET() == 2.4152549E38F);
            assert(pack.yaw_GET() == -5.9200627E37F);
            assert(pack.roll_GET() == 1.7132223E38F);
            assert(pack.lon_GET() == 2.0660705E38F);
            assert(pack.q3_GET() == -1.170929E38F);
            assert(pack.ve_GET() == -2.8324304E38F);
            assert(pack.vd_GET() == -1.9892253E38F);
            assert(pack.q4_GET() == -3.5746456E37F);
            assert(pack.q1_GET() == -5.934942E37F);
            assert(pack.alt_GET() == -2.6586245E38F);
            assert(pack.vn_GET() == -1.5012412E37F);
            assert(pack.zgyro_GET() == -2.0733281E38F);
            assert(pack.std_dev_vert_GET() == -3.164563E38F);
            assert(pack.yacc_GET() == -1.9168453E38F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.q4_SET(-3.5746456E37F) ;
        p108.alt_SET(-2.6586245E38F) ;
        p108.q1_SET(-5.934942E37F) ;
        p108.lon_SET(2.0660705E38F) ;
        p108.yacc_SET(-1.9168453E38F) ;
        p108.vd_SET(-1.9892253E38F) ;
        p108.ve_SET(-2.8324304E38F) ;
        p108.std_dev_horz_SET(1.6368786E36F) ;
        p108.q3_SET(-1.170929E38F) ;
        p108.q2_SET(2.475256E37F) ;
        p108.zacc_SET(-1.2336657E38F) ;
        p108.xacc_SET(3.203731E38F) ;
        p108.lat_SET(1.273605E38F) ;
        p108.xgyro_SET(-2.3810312E38F) ;
        p108.roll_SET(1.7132223E38F) ;
        p108.vn_SET(-1.5012412E37F) ;
        p108.yaw_SET(-5.9200627E37F) ;
        p108.pitch_SET(9.367871E37F) ;
        p108.ygyro_SET(2.4152549E38F) ;
        p108.std_dev_vert_SET(-3.164563E38F) ;
        p108.zgyro_SET(-2.0733281E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.rxerrors_GET() == (char)21184);
            assert(pack.noise_GET() == (char)0);
            assert(pack.fixed__GET() == (char)54850);
            assert(pack.txbuf_GET() == (char)56);
            assert(pack.remnoise_GET() == (char)28);
            assert(pack.remrssi_GET() == (char)54);
            assert(pack.rssi_GET() == (char)24);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.noise_SET((char)0) ;
        p109.rxerrors_SET((char)21184) ;
        p109.rssi_SET((char)24) ;
        p109.remnoise_SET((char)28) ;
        p109.txbuf_SET((char)56) ;
        p109.fixed__SET((char)54850) ;
        p109.remrssi_SET((char)54) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)184);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)12, (char)15, (char)151, (char)48, (char)191, (char)156, (char)191, (char)83, (char)210, (char)188, (char)199, (char)115, (char)28, (char)98, (char)223, (char)109, (char)45, (char)38, (char)30, (char)104, (char)69, (char)70, (char)217, (char)172, (char)203, (char)118, (char)205, (char)140, (char)210, (char)223, (char)62, (char)138, (char)161, (char)212, (char)8, (char)14, (char)243, (char)149, (char)165, (char)144, (char)83, (char)221, (char)143, (char)11, (char)76, (char)114, (char)130, (char)2, (char)148, (char)207, (char)20, (char)142, (char)23, (char)6, (char)255, (char)189, (char)30, (char)115, (char)127, (char)70, (char)176, (char)87, (char)151, (char)117, (char)16, (char)82, (char)3, (char)197, (char)246, (char)22, (char)102, (char)209, (char)231, (char)246, (char)76, (char)14, (char)122, (char)147, (char)171, (char)36, (char)139, (char)76, (char)12, (char)129, (char)249, (char)115, (char)56, (char)16, (char)182, (char)149, (char)61, (char)234, (char)224, (char)122, (char)230, (char)144, (char)208, (char)183, (char)216, (char)8, (char)6, (char)248, (char)59, (char)236, (char)145, (char)28, (char)175, (char)225, (char)178, (char)171, (char)129, (char)97, (char)147, (char)54, (char)203, (char)246, (char)177, (char)141, (char)13, (char)9, (char)12, (char)181, (char)252, (char)129, (char)125, (char)239, (char)123, (char)176, (char)230, (char)65, (char)72, (char)150, (char)152, (char)108, (char)42, (char)75, (char)189, (char)88, (char)156, (char)206, (char)87, (char)184, (char)75, (char)43, (char)69, (char)98, (char)95, (char)128, (char)111, (char)116, (char)223, (char)77, (char)16, (char)167, (char)203, (char)64, (char)249, (char)169, (char)2, (char)73, (char)133, (char)171, (char)177, (char)208, (char)181, (char)128, (char)144, (char)149, (char)106, (char)243, (char)218, (char)246, (char)170, (char)2, (char)51, (char)158, (char)208, (char)204, (char)89, (char)240, (char)196, (char)116, (char)211, (char)218, (char)199, (char)177, (char)6, (char)89, (char)120, (char)35, (char)24, (char)204, (char)40, (char)220, (char)149, (char)21, (char)216, (char)108, (char)247, (char)46, (char)4, (char)241, (char)18, (char)187, (char)30, (char)154, (char)210, (char)98, (char)15, (char)120, (char)185, (char)18, (char)219, (char)29, (char)206, (char)150, (char)184, (char)171, (char)107, (char)53, (char)54, (char)224, (char)55, (char)172, (char)202, (char)72, (char)86, (char)120, (char)18, (char)65, (char)199, (char)237, (char)58, (char)170, (char)114, (char)19, (char)234, (char)210, (char)36, (char)7, (char)103, (char)139, (char)167, (char)26, (char)119, (char)220, (char)224, (char)107, (char)76, (char)230, (char)44}));
            assert(pack.target_network_GET() == (char)41);
            assert(pack.target_component_GET() == (char)116);
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_system_SET((char)184) ;
        p110.target_component_SET((char)116) ;
        p110.target_network_SET((char)41) ;
        p110.payload_SET(new char[] {(char)12, (char)15, (char)151, (char)48, (char)191, (char)156, (char)191, (char)83, (char)210, (char)188, (char)199, (char)115, (char)28, (char)98, (char)223, (char)109, (char)45, (char)38, (char)30, (char)104, (char)69, (char)70, (char)217, (char)172, (char)203, (char)118, (char)205, (char)140, (char)210, (char)223, (char)62, (char)138, (char)161, (char)212, (char)8, (char)14, (char)243, (char)149, (char)165, (char)144, (char)83, (char)221, (char)143, (char)11, (char)76, (char)114, (char)130, (char)2, (char)148, (char)207, (char)20, (char)142, (char)23, (char)6, (char)255, (char)189, (char)30, (char)115, (char)127, (char)70, (char)176, (char)87, (char)151, (char)117, (char)16, (char)82, (char)3, (char)197, (char)246, (char)22, (char)102, (char)209, (char)231, (char)246, (char)76, (char)14, (char)122, (char)147, (char)171, (char)36, (char)139, (char)76, (char)12, (char)129, (char)249, (char)115, (char)56, (char)16, (char)182, (char)149, (char)61, (char)234, (char)224, (char)122, (char)230, (char)144, (char)208, (char)183, (char)216, (char)8, (char)6, (char)248, (char)59, (char)236, (char)145, (char)28, (char)175, (char)225, (char)178, (char)171, (char)129, (char)97, (char)147, (char)54, (char)203, (char)246, (char)177, (char)141, (char)13, (char)9, (char)12, (char)181, (char)252, (char)129, (char)125, (char)239, (char)123, (char)176, (char)230, (char)65, (char)72, (char)150, (char)152, (char)108, (char)42, (char)75, (char)189, (char)88, (char)156, (char)206, (char)87, (char)184, (char)75, (char)43, (char)69, (char)98, (char)95, (char)128, (char)111, (char)116, (char)223, (char)77, (char)16, (char)167, (char)203, (char)64, (char)249, (char)169, (char)2, (char)73, (char)133, (char)171, (char)177, (char)208, (char)181, (char)128, (char)144, (char)149, (char)106, (char)243, (char)218, (char)246, (char)170, (char)2, (char)51, (char)158, (char)208, (char)204, (char)89, (char)240, (char)196, (char)116, (char)211, (char)218, (char)199, (char)177, (char)6, (char)89, (char)120, (char)35, (char)24, (char)204, (char)40, (char)220, (char)149, (char)21, (char)216, (char)108, (char)247, (char)46, (char)4, (char)241, (char)18, (char)187, (char)30, (char)154, (char)210, (char)98, (char)15, (char)120, (char)185, (char)18, (char)219, (char)29, (char)206, (char)150, (char)184, (char)171, (char)107, (char)53, (char)54, (char)224, (char)55, (char)172, (char)202, (char)72, (char)86, (char)120, (char)18, (char)65, (char)199, (char)237, (char)58, (char)170, (char)114, (char)19, (char)234, (char)210, (char)36, (char)7, (char)103, (char)139, (char)167, (char)26, (char)119, (char)220, (char)224, (char)107, (char)76, (char)230, (char)44}, 0) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.ts1_GET() == -6238464702619656805L);
            assert(pack.tc1_GET() == 8340091365211165319L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(-6238464702619656805L) ;
        p111.tc1_SET(8340091365211165319L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7006143941605165420L);
            assert(pack.seq_GET() == 2915994400L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(2915994400L) ;
        p112.time_usec_SET(7006143941605165420L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.vn_GET() == (short)13649);
            assert(pack.ve_GET() == (short) -16281);
            assert(pack.fix_type_GET() == (char)231);
            assert(pack.epv_GET() == (char)33326);
            assert(pack.time_usec_GET() == 3989142596547467511L);
            assert(pack.cog_GET() == (char)12068);
            assert(pack.lat_GET() == -2084430457);
            assert(pack.eph_GET() == (char)39436);
            assert(pack.satellites_visible_GET() == (char)1);
            assert(pack.vel_GET() == (char)2995);
            assert(pack.vd_GET() == (short)27093);
            assert(pack.lon_GET() == 233203190);
            assert(pack.alt_GET() == -1125531675);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.lat_SET(-2084430457) ;
        p113.lon_SET(233203190) ;
        p113.vd_SET((short)27093) ;
        p113.time_usec_SET(3989142596547467511L) ;
        p113.eph_SET((char)39436) ;
        p113.satellites_visible_SET((char)1) ;
        p113.alt_SET(-1125531675) ;
        p113.epv_SET((char)33326) ;
        p113.vn_SET((short)13649) ;
        p113.fix_type_SET((char)231) ;
        p113.ve_SET((short) -16281) ;
        p113.vel_SET((char)2995) ;
        p113.cog_SET((char)12068) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integrated_xgyro_GET() == 2.5470675E37F);
            assert(pack.time_usec_GET() == 6645231106651029755L);
            assert(pack.time_delta_distance_us_GET() == 4253609806L);
            assert(pack.integrated_ygyro_GET() == -1.50251E38F);
            assert(pack.integrated_x_GET() == 2.1087094E38F);
            assert(pack.integration_time_us_GET() == 1546154220L);
            assert(pack.temperature_GET() == (short) -4687);
            assert(pack.distance_GET() == -2.2521878E38F);
            assert(pack.integrated_y_GET() == -8.05733E37F);
            assert(pack.integrated_zgyro_GET() == 3.0819626E38F);
            assert(pack.quality_GET() == (char)213);
            assert(pack.sensor_id_GET() == (char)105);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.temperature_SET((short) -4687) ;
        p114.time_delta_distance_us_SET(4253609806L) ;
        p114.integrated_y_SET(-8.05733E37F) ;
        p114.distance_SET(-2.2521878E38F) ;
        p114.integration_time_us_SET(1546154220L) ;
        p114.integrated_xgyro_SET(2.5470675E37F) ;
        p114.integrated_zgyro_SET(3.0819626E38F) ;
        p114.integrated_ygyro_SET(-1.50251E38F) ;
        p114.time_usec_SET(6645231106651029755L) ;
        p114.integrated_x_SET(2.1087094E38F) ;
        p114.sensor_id_SET((char)105) ;
        p114.quality_SET((char)213) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == (short) -11591);
            assert(pack.ind_airspeed_GET() == (char)37435);
            assert(pack.lat_GET() == -1894334506);
            assert(pack.vy_GET() == (short)9913);
            assert(pack.pitchspeed_GET() == -5.022628E37F);
            assert(pack.alt_GET() == 2109873621);
            assert(pack.yacc_GET() == (short)12028);
            assert(pack.yawspeed_GET() == 2.2253779E38F);
            assert(pack.time_usec_GET() == 7444921273343626310L);
            assert(pack.zacc_GET() == (short)1167);
            assert(pack.true_airspeed_GET() == (char)59513);
            assert(pack.lon_GET() == 1499828622);
            assert(pack.xacc_GET() == (short) -23591);
            assert(pack.vx_GET() == (short)19544);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {3.193003E38F, -7.915314E37F, 6.895656E37F, 6.9849343E37F}));
            assert(pack.rollspeed_GET() == 2.1333629E38F);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.yacc_SET((short)12028) ;
        p115.rollspeed_SET(2.1333629E38F) ;
        p115.zacc_SET((short)1167) ;
        p115.yawspeed_SET(2.2253779E38F) ;
        p115.vz_SET((short) -11591) ;
        p115.ind_airspeed_SET((char)37435) ;
        p115.time_usec_SET(7444921273343626310L) ;
        p115.pitchspeed_SET(-5.022628E37F) ;
        p115.attitude_quaternion_SET(new float[] {3.193003E38F, -7.915314E37F, 6.895656E37F, 6.9849343E37F}, 0) ;
        p115.alt_SET(2109873621) ;
        p115.xacc_SET((short) -23591) ;
        p115.true_airspeed_SET((char)59513) ;
        p115.vy_SET((short)9913) ;
        p115.lon_SET(1499828622) ;
        p115.lat_SET(-1894334506) ;
        p115.vx_SET((short)19544) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.ymag_GET() == (short) -14474);
            assert(pack.xgyro_GET() == (short)14345);
            assert(pack.zmag_GET() == (short)26036);
            assert(pack.zgyro_GET() == (short) -31176);
            assert(pack.ygyro_GET() == (short) -10511);
            assert(pack.time_boot_ms_GET() == 3711771813L);
            assert(pack.zacc_GET() == (short) -24080);
            assert(pack.yacc_GET() == (short) -22413);
            assert(pack.xacc_GET() == (short)24566);
            assert(pack.xmag_GET() == (short) -28785);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.zgyro_SET((short) -31176) ;
        p116.yacc_SET((short) -22413) ;
        p116.zacc_SET((short) -24080) ;
        p116.xgyro_SET((short)14345) ;
        p116.xmag_SET((short) -28785) ;
        p116.zmag_SET((short)26036) ;
        p116.time_boot_ms_SET(3711771813L) ;
        p116.xacc_SET((short)24566) ;
        p116.ymag_SET((short) -14474) ;
        p116.ygyro_SET((short) -10511) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_GET() == (char)29301);
            assert(pack.target_component_GET() == (char)130);
            assert(pack.end_GET() == (char)45238);
            assert(pack.target_system_GET() == (char)207);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.end_SET((char)45238) ;
        p117.target_system_SET((char)207) ;
        p117.target_component_SET((char)130) ;
        p117.start_SET((char)29301) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.num_logs_GET() == (char)17049);
            assert(pack.id_GET() == (char)2251);
            assert(pack.time_utc_GET() == 1168488073L);
            assert(pack.size_GET() == 1503256485L);
            assert(pack.last_log_num_GET() == (char)12593);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.time_utc_SET(1168488073L) ;
        p118.last_log_num_SET((char)12593) ;
        p118.id_SET((char)2251) ;
        p118.size_SET(1503256485L) ;
        p118.num_logs_SET((char)17049) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)23145);
            assert(pack.ofs_GET() == 2322859922L);
            assert(pack.count_GET() == 2349199067L);
            assert(pack.target_component_GET() == (char)34);
            assert(pack.target_system_GET() == (char)50);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)50) ;
        p119.ofs_SET(2322859922L) ;
        p119.target_component_SET((char)34) ;
        p119.count_SET(2349199067L) ;
        p119.id_SET((char)23145) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.ofs_GET() == 2378465251L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)227, (char)57, (char)52, (char)57, (char)28, (char)30, (char)213, (char)175, (char)78, (char)37, (char)224, (char)42, (char)125, (char)189, (char)110, (char)163, (char)227, (char)231, (char)41, (char)157, (char)210, (char)192, (char)191, (char)183, (char)90, (char)101, (char)150, (char)152, (char)214, (char)64, (char)191, (char)129, (char)243, (char)171, (char)127, (char)80, (char)216, (char)212, (char)205, (char)250, (char)77, (char)169, (char)128, (char)224, (char)125, (char)133, (char)43, (char)50, (char)16, (char)43, (char)214, (char)170, (char)180, (char)66, (char)209, (char)187, (char)80, (char)207, (char)103, (char)131, (char)32, (char)47, (char)200, (char)137, (char)86, (char)77, (char)73, (char)186, (char)101, (char)100, (char)234, (char)22, (char)18, (char)127, (char)136, (char)13, (char)139, (char)207, (char)126, (char)254, (char)254, (char)187, (char)213, (char)169, (char)183, (char)157, (char)249, (char)225, (char)165, (char)228}));
            assert(pack.id_GET() == (char)469);
            assert(pack.count_GET() == (char)52);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)469) ;
        p120.count_SET((char)52) ;
        p120.ofs_SET(2378465251L) ;
        p120.data__SET(new char[] {(char)227, (char)57, (char)52, (char)57, (char)28, (char)30, (char)213, (char)175, (char)78, (char)37, (char)224, (char)42, (char)125, (char)189, (char)110, (char)163, (char)227, (char)231, (char)41, (char)157, (char)210, (char)192, (char)191, (char)183, (char)90, (char)101, (char)150, (char)152, (char)214, (char)64, (char)191, (char)129, (char)243, (char)171, (char)127, (char)80, (char)216, (char)212, (char)205, (char)250, (char)77, (char)169, (char)128, (char)224, (char)125, (char)133, (char)43, (char)50, (char)16, (char)43, (char)214, (char)170, (char)180, (char)66, (char)209, (char)187, (char)80, (char)207, (char)103, (char)131, (char)32, (char)47, (char)200, (char)137, (char)86, (char)77, (char)73, (char)186, (char)101, (char)100, (char)234, (char)22, (char)18, (char)127, (char)136, (char)13, (char)139, (char)207, (char)126, (char)254, (char)254, (char)187, (char)213, (char)169, (char)183, (char)157, (char)249, (char)225, (char)165, (char)228}, 0) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)251);
            assert(pack.target_system_GET() == (char)156);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_component_SET((char)251) ;
        p121.target_system_SET((char)156) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)196);
            assert(pack.target_component_GET() == (char)38);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)196) ;
        p122.target_component_SET((char)38) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)30);
            assert(pack.target_system_GET() == (char)123);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)8, (char)237, (char)188, (char)194, (char)213, (char)0, (char)22, (char)243, (char)212, (char)69, (char)118, (char)40, (char)76, (char)8, (char)218, (char)5, (char)109, (char)7, (char)186, (char)10, (char)201, (char)252, (char)198, (char)185, (char)42, (char)37, (char)181, (char)14, (char)188, (char)251, (char)101, (char)151, (char)193, (char)47, (char)45, (char)180, (char)26, (char)78, (char)73, (char)116, (char)135, (char)45, (char)248, (char)66, (char)186, (char)107, (char)54, (char)176, (char)196, (char)114, (char)122, (char)210, (char)130, (char)186, (char)58, (char)248, (char)31, (char)224, (char)197, (char)159, (char)1, (char)109, (char)217, (char)75, (char)235, (char)52, (char)55, (char)116, (char)133, (char)67, (char)195, (char)231, (char)162, (char)11, (char)121, (char)85, (char)248, (char)84, (char)194, (char)60, (char)45, (char)145, (char)140, (char)191, (char)16, (char)12, (char)39, (char)23, (char)109, (char)57, (char)104, (char)54, (char)50, (char)234, (char)136, (char)190, (char)166, (char)237, (char)94, (char)104, (char)38, (char)134, (char)123, (char)132, (char)125, (char)185, (char)93, (char)233, (char)2, (char)205}));
            assert(pack.target_component_GET() == (char)108);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.data__SET(new char[] {(char)8, (char)237, (char)188, (char)194, (char)213, (char)0, (char)22, (char)243, (char)212, (char)69, (char)118, (char)40, (char)76, (char)8, (char)218, (char)5, (char)109, (char)7, (char)186, (char)10, (char)201, (char)252, (char)198, (char)185, (char)42, (char)37, (char)181, (char)14, (char)188, (char)251, (char)101, (char)151, (char)193, (char)47, (char)45, (char)180, (char)26, (char)78, (char)73, (char)116, (char)135, (char)45, (char)248, (char)66, (char)186, (char)107, (char)54, (char)176, (char)196, (char)114, (char)122, (char)210, (char)130, (char)186, (char)58, (char)248, (char)31, (char)224, (char)197, (char)159, (char)1, (char)109, (char)217, (char)75, (char)235, (char)52, (char)55, (char)116, (char)133, (char)67, (char)195, (char)231, (char)162, (char)11, (char)121, (char)85, (char)248, (char)84, (char)194, (char)60, (char)45, (char)145, (char)140, (char)191, (char)16, (char)12, (char)39, (char)23, (char)109, (char)57, (char)104, (char)54, (char)50, (char)234, (char)136, (char)190, (char)166, (char)237, (char)94, (char)104, (char)38, (char)134, (char)123, (char)132, (char)125, (char)185, (char)93, (char)233, (char)2, (char)205}, 0) ;
        p123.len_SET((char)30) ;
        p123.target_system_SET((char)123) ;
        p123.target_component_SET((char)108) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.satellites_visible_GET() == (char)168);
            assert(pack.epv_GET() == (char)9926);
            assert(pack.vel_GET() == (char)48886);
            assert(pack.dgps_numch_GET() == (char)160);
            assert(pack.eph_GET() == (char)53150);
            assert(pack.dgps_age_GET() == 1923720537L);
            assert(pack.cog_GET() == (char)49941);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.lat_GET() == 1169981068);
            assert(pack.lon_GET() == 883987869);
            assert(pack.time_usec_GET() == 3045298857804596949L);
            assert(pack.alt_GET() == -618165508);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.dgps_age_SET(1923720537L) ;
        p124.time_usec_SET(3045298857804596949L) ;
        p124.cog_SET((char)49941) ;
        p124.eph_SET((char)53150) ;
        p124.vel_SET((char)48886) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p124.lat_SET(1169981068) ;
        p124.dgps_numch_SET((char)160) ;
        p124.epv_SET((char)9926) ;
        p124.alt_SET(-618165508) ;
        p124.lon_SET(883987869) ;
        p124.satellites_visible_SET((char)168) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vcc_GET() == (char)49150);
            assert(pack.flags_GET() == MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED);
            assert(pack.Vservo_GET() == (char)29594);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vservo_SET((char)29594) ;
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED) ;
        p125.Vcc_SET((char)49150) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)146, (char)144, (char)47, (char)238, (char)63, (char)216, (char)25, (char)72, (char)125, (char)175, (char)87, (char)39, (char)229, (char)79, (char)225, (char)25, (char)252, (char)181, (char)151, (char)216, (char)227, (char)110, (char)162, (char)129, (char)104, (char)136, (char)84, (char)151, (char)132, (char)100, (char)118, (char)105, (char)194, (char)46, (char)132, (char)73, (char)70, (char)142, (char)228, (char)166, (char)57, (char)141, (char)182, (char)47, (char)168, (char)239, (char)131, (char)216, (char)157, (char)203, (char)214, (char)212, (char)93, (char)101, (char)220, (char)68, (char)47, (char)157, (char)2, (char)230, (char)122, (char)61, (char)58, (char)98, (char)29, (char)69, (char)66, (char)185, (char)92, (char)1}));
            assert(pack.flags_GET() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
            assert(pack.baudrate_GET() == 4140917959L);
            assert(pack.timeout_GET() == (char)47980);
            assert(pack.count_GET() == (char)97);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.timeout_SET((char)47980) ;
        p126.count_SET((char)97) ;
        p126.data__SET(new char[] {(char)146, (char)144, (char)47, (char)238, (char)63, (char)216, (char)25, (char)72, (char)125, (char)175, (char)87, (char)39, (char)229, (char)79, (char)225, (char)25, (char)252, (char)181, (char)151, (char)216, (char)227, (char)110, (char)162, (char)129, (char)104, (char)136, (char)84, (char)151, (char)132, (char)100, (char)118, (char)105, (char)194, (char)46, (char)132, (char)73, (char)70, (char)142, (char)228, (char)166, (char)57, (char)141, (char)182, (char)47, (char)168, (char)239, (char)131, (char)216, (char)157, (char)203, (char)214, (char)212, (char)93, (char)101, (char)220, (char)68, (char)47, (char)157, (char)2, (char)230, (char)122, (char)61, (char)58, (char)98, (char)29, (char)69, (char)66, (char)185, (char)92, (char)1}, 0) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL) ;
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE) ;
        p126.baudrate_SET(4140917959L) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.wn_GET() == (char)59664);
            assert(pack.baseline_coords_type_GET() == (char)23);
            assert(pack.rtk_receiver_id_GET() == (char)198);
            assert(pack.rtk_rate_GET() == (char)12);
            assert(pack.time_last_baseline_ms_GET() == 1546728224L);
            assert(pack.iar_num_hypotheses_GET() == 1407874178);
            assert(pack.accuracy_GET() == 4010176079L);
            assert(pack.nsats_GET() == (char)189);
            assert(pack.baseline_a_mm_GET() == 1858510998);
            assert(pack.rtk_health_GET() == (char)110);
            assert(pack.tow_GET() == 2675601722L);
            assert(pack.baseline_c_mm_GET() == -351750834);
            assert(pack.baseline_b_mm_GET() == -1975450694);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.nsats_SET((char)189) ;
        p127.rtk_rate_SET((char)12) ;
        p127.wn_SET((char)59664) ;
        p127.baseline_coords_type_SET((char)23) ;
        p127.baseline_b_mm_SET(-1975450694) ;
        p127.baseline_a_mm_SET(1858510998) ;
        p127.tow_SET(2675601722L) ;
        p127.rtk_health_SET((char)110) ;
        p127.baseline_c_mm_SET(-351750834) ;
        p127.time_last_baseline_ms_SET(1546728224L) ;
        p127.rtk_receiver_id_SET((char)198) ;
        p127.iar_num_hypotheses_SET(1407874178) ;
        p127.accuracy_SET(4010176079L) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.rtk_rate_GET() == (char)37);
            assert(pack.baseline_a_mm_GET() == -1152713245);
            assert(pack.baseline_b_mm_GET() == 1993253006);
            assert(pack.accuracy_GET() == 3053636248L);
            assert(pack.wn_GET() == (char)37043);
            assert(pack.baseline_c_mm_GET() == -700681329);
            assert(pack.tow_GET() == 3808470034L);
            assert(pack.iar_num_hypotheses_GET() == -1936813240);
            assert(pack.time_last_baseline_ms_GET() == 2681815429L);
            assert(pack.baseline_coords_type_GET() == (char)163);
            assert(pack.nsats_GET() == (char)148);
            assert(pack.rtk_receiver_id_GET() == (char)72);
            assert(pack.rtk_health_GET() == (char)178);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.accuracy_SET(3053636248L) ;
        p128.rtk_rate_SET((char)37) ;
        p128.rtk_receiver_id_SET((char)72) ;
        p128.wn_SET((char)37043) ;
        p128.baseline_c_mm_SET(-700681329) ;
        p128.baseline_coords_type_SET((char)163) ;
        p128.baseline_b_mm_SET(1993253006) ;
        p128.nsats_SET((char)148) ;
        p128.tow_SET(3808470034L) ;
        p128.rtk_health_SET((char)178) ;
        p128.iar_num_hypotheses_SET(-1936813240) ;
        p128.time_last_baseline_ms_SET(2681815429L) ;
        p128.baseline_a_mm_SET(-1152713245) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short)5067);
            assert(pack.xacc_GET() == (short) -22173);
            assert(pack.zmag_GET() == (short)23036);
            assert(pack.ymag_GET() == (short) -10175);
            assert(pack.xgyro_GET() == (short) -2957);
            assert(pack.xmag_GET() == (short)11480);
            assert(pack.time_boot_ms_GET() == 501860032L);
            assert(pack.zgyro_GET() == (short)32193);
            assert(pack.yacc_GET() == (short)24371);
            assert(pack.zacc_GET() == (short)24585);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.ygyro_SET((short)5067) ;
        p129.time_boot_ms_SET(501860032L) ;
        p129.xgyro_SET((short) -2957) ;
        p129.zacc_SET((short)24585) ;
        p129.yacc_SET((short)24371) ;
        p129.zgyro_SET((short)32193) ;
        p129.ymag_SET((short) -10175) ;
        p129.xacc_SET((short) -22173) ;
        p129.zmag_SET((short)23036) ;
        p129.xmag_SET((short)11480) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)77);
            assert(pack.jpg_quality_GET() == (char)70);
            assert(pack.packets_GET() == (char)34954);
            assert(pack.width_GET() == (char)961);
            assert(pack.size_GET() == 1636923308L);
            assert(pack.height_GET() == (char)542);
            assert(pack.payload_GET() == (char)11);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.width_SET((char)961) ;
        p130.height_SET((char)542) ;
        p130.packets_SET((char)34954) ;
        p130.type_SET((char)77) ;
        p130.payload_SET((char)11) ;
        p130.jpg_quality_SET((char)70) ;
        p130.size_SET(1636923308L) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)17, (char)117, (char)203, (char)232, (char)191, (char)125, (char)124, (char)154, (char)81, (char)198, (char)51, (char)37, (char)88, (char)234, (char)255, (char)78, (char)215, (char)193, (char)38, (char)8, (char)164, (char)110, (char)102, (char)230, (char)56, (char)38, (char)72, (char)2, (char)227, (char)178, (char)222, (char)151, (char)135, (char)96, (char)179, (char)85, (char)218, (char)125, (char)91, (char)213, (char)127, (char)92, (char)232, (char)215, (char)119, (char)154, (char)56, (char)128, (char)220, (char)22, (char)94, (char)74, (char)217, (char)81, (char)139, (char)57, (char)9, (char)63, (char)87, (char)140, (char)195, (char)129, (char)153, (char)29, (char)181, (char)174, (char)163, (char)149, (char)223, (char)214, (char)35, (char)128, (char)156, (char)151, (char)104, (char)231, (char)209, (char)144, (char)68, (char)220, (char)216, (char)153, (char)232, (char)182, (char)152, (char)105, (char)110, (char)2, (char)247, (char)1, (char)172, (char)40, (char)213, (char)77, (char)176, (char)24, (char)15, (char)111, (char)98, (char)134, (char)227, (char)187, (char)174, (char)162, (char)11, (char)82, (char)52, (char)34, (char)150, (char)68, (char)5, (char)64, (char)105, (char)19, (char)172, (char)218, (char)42, (char)176, (char)177, (char)125, (char)234, (char)220, (char)109, (char)220, (char)195, (char)100, (char)105, (char)79, (char)241, (char)39, (char)174, (char)60, (char)121, (char)99, (char)64, (char)19, (char)234, (char)216, (char)241, (char)233, (char)53, (char)206, (char)253, (char)186, (char)85, (char)197, (char)176, (char)43, (char)161, (char)15, (char)174, (char)59, (char)141, (char)102, (char)189, (char)60, (char)170, (char)118, (char)88, (char)244, (char)168, (char)3, (char)213, (char)173, (char)178, (char)52, (char)147, (char)179, (char)173, (char)141, (char)128, (char)31, (char)241, (char)99, (char)127, (char)150, (char)233, (char)206, (char)174, (char)167, (char)106, (char)143, (char)3, (char)119, (char)241, (char)158, (char)164, (char)237, (char)241, (char)159, (char)19, (char)104, (char)171, (char)167, (char)185, (char)131, (char)26, (char)126, (char)88, (char)231, (char)224, (char)251, (char)82, (char)235, (char)114, (char)149, (char)98, (char)144, (char)24, (char)229, (char)255, (char)233, (char)200, (char)106, (char)34, (char)180, (char)51, (char)102, (char)132, (char)186, (char)241, (char)118, (char)109, (char)71, (char)13, (char)70, (char)250, (char)108, (char)142, (char)165, (char)75, (char)188, (char)175, (char)134, (char)84, (char)15, (char)22, (char)7, (char)46, (char)80, (char)131, (char)240, (char)245, (char)124, (char)9, (char)236, (char)14, (char)244, (char)69, (char)105, (char)130, (char)54, (char)127}));
            assert(pack.seqnr_GET() == (char)18071);
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)18071) ;
        p131.data__SET(new char[] {(char)17, (char)117, (char)203, (char)232, (char)191, (char)125, (char)124, (char)154, (char)81, (char)198, (char)51, (char)37, (char)88, (char)234, (char)255, (char)78, (char)215, (char)193, (char)38, (char)8, (char)164, (char)110, (char)102, (char)230, (char)56, (char)38, (char)72, (char)2, (char)227, (char)178, (char)222, (char)151, (char)135, (char)96, (char)179, (char)85, (char)218, (char)125, (char)91, (char)213, (char)127, (char)92, (char)232, (char)215, (char)119, (char)154, (char)56, (char)128, (char)220, (char)22, (char)94, (char)74, (char)217, (char)81, (char)139, (char)57, (char)9, (char)63, (char)87, (char)140, (char)195, (char)129, (char)153, (char)29, (char)181, (char)174, (char)163, (char)149, (char)223, (char)214, (char)35, (char)128, (char)156, (char)151, (char)104, (char)231, (char)209, (char)144, (char)68, (char)220, (char)216, (char)153, (char)232, (char)182, (char)152, (char)105, (char)110, (char)2, (char)247, (char)1, (char)172, (char)40, (char)213, (char)77, (char)176, (char)24, (char)15, (char)111, (char)98, (char)134, (char)227, (char)187, (char)174, (char)162, (char)11, (char)82, (char)52, (char)34, (char)150, (char)68, (char)5, (char)64, (char)105, (char)19, (char)172, (char)218, (char)42, (char)176, (char)177, (char)125, (char)234, (char)220, (char)109, (char)220, (char)195, (char)100, (char)105, (char)79, (char)241, (char)39, (char)174, (char)60, (char)121, (char)99, (char)64, (char)19, (char)234, (char)216, (char)241, (char)233, (char)53, (char)206, (char)253, (char)186, (char)85, (char)197, (char)176, (char)43, (char)161, (char)15, (char)174, (char)59, (char)141, (char)102, (char)189, (char)60, (char)170, (char)118, (char)88, (char)244, (char)168, (char)3, (char)213, (char)173, (char)178, (char)52, (char)147, (char)179, (char)173, (char)141, (char)128, (char)31, (char)241, (char)99, (char)127, (char)150, (char)233, (char)206, (char)174, (char)167, (char)106, (char)143, (char)3, (char)119, (char)241, (char)158, (char)164, (char)237, (char)241, (char)159, (char)19, (char)104, (char)171, (char)167, (char)185, (char)131, (char)26, (char)126, (char)88, (char)231, (char)224, (char)251, (char)82, (char)235, (char)114, (char)149, (char)98, (char)144, (char)24, (char)229, (char)255, (char)233, (char)200, (char)106, (char)34, (char)180, (char)51, (char)102, (char)132, (char)186, (char)241, (char)118, (char)109, (char)71, (char)13, (char)70, (char)250, (char)108, (char)142, (char)165, (char)75, (char)188, (char)175, (char)134, (char)84, (char)15, (char)22, (char)7, (char)46, (char)80, (char)131, (char)240, (char)245, (char)124, (char)9, (char)236, (char)14, (char)244, (char)69, (char)105, (char)130, (char)54, (char)127}, 0) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2724720106L);
            assert(pack.min_distance_GET() == (char)17207);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315);
            assert(pack.covariance_GET() == (char)39);
            assert(pack.max_distance_GET() == (char)9877);
            assert(pack.id_GET() == (char)37);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            assert(pack.current_distance_GET() == (char)53551);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.time_boot_ms_SET(2724720106L) ;
        p132.id_SET((char)37) ;
        p132.current_distance_SET((char)53551) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR) ;
        p132.min_distance_SET((char)17207) ;
        p132.covariance_SET((char)39) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315) ;
        p132.max_distance_SET((char)9877) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 385223418);
            assert(pack.mask_GET() == 2249604590926766754L);
            assert(pack.lon_GET() == 1261962124);
            assert(pack.grid_spacing_GET() == (char)51314);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(385223418) ;
        p133.grid_spacing_SET((char)51314) ;
        p133.lon_SET(1261962124) ;
        p133.mask_SET(2249604590926766754L) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.gridbit_GET() == (char)23);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -23912, (short) -15101, (short)9493, (short)14760, (short)3996, (short)26313, (short)11495, (short) -27664, (short)10231, (short)28981, (short)21397, (short)29213, (short)14469, (short) -9652, (short) -2125, (short)18932}));
            assert(pack.lat_GET() == 1386331110);
            assert(pack.lon_GET() == -33419324);
            assert(pack.grid_spacing_GET() == (char)6200);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(1386331110) ;
        p134.gridbit_SET((char)23) ;
        p134.lon_SET(-33419324) ;
        p134.grid_spacing_SET((char)6200) ;
        p134.data__SET(new short[] {(short) -23912, (short) -15101, (short)9493, (short)14760, (short)3996, (short)26313, (short)11495, (short) -27664, (short)10231, (short)28981, (short)21397, (short)29213, (short)14469, (short) -9652, (short) -2125, (short)18932}, 0) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1087662555);
            assert(pack.lon_GET() == 155179298);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lon_SET(155179298) ;
        p135.lat_SET(1087662555) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.spacing_GET() == (char)52915);
            assert(pack.lat_GET() == -160567194);
            assert(pack.lon_GET() == 1017734577);
            assert(pack.current_height_GET() == -3.259414E38F);
            assert(pack.pending_GET() == (char)48560);
            assert(pack.terrain_height_GET() == 1.941936E38F);
            assert(pack.loaded_GET() == (char)36464);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.current_height_SET(-3.259414E38F) ;
        p136.lon_SET(1017734577) ;
        p136.loaded_SET((char)36464) ;
        p136.lat_SET(-160567194) ;
        p136.spacing_SET((char)52915) ;
        p136.terrain_height_SET(1.941936E38F) ;
        p136.pending_SET((char)48560) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)25659);
            assert(pack.time_boot_ms_GET() == 852893650L);
            assert(pack.press_abs_GET() == -2.0313281E37F);
            assert(pack.press_diff_GET() == 1.0473104E38F);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(852893650L) ;
        p137.temperature_SET((short)25659) ;
        p137.press_diff_SET(1.0473104E38F) ;
        p137.press_abs_SET(-2.0313281E37F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 2.6490385E38F);
            assert(pack.time_usec_GET() == 8496433728184508838L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.2577342E38F, -4.8629055E37F, 2.6882756E38F, 9.3955E37F}));
            assert(pack.z_GET() == 3.1086068E38F);
            assert(pack.x_GET() == -2.5006897E38F);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.y_SET(2.6490385E38F) ;
        p138.z_SET(3.1086068E38F) ;
        p138.q_SET(new float[] {-2.2577342E38F, -4.8629055E37F, 2.6882756E38F, 9.3955E37F}, 0) ;
        p138.x_SET(-2.5006897E38F) ;
        p138.time_usec_SET(8496433728184508838L) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-2.93452E38F, -4.766794E37F, 1.7501543E38F, -7.351998E37F, 3.2983877E38F, 2.1337823E38F, -2.422981E38F, 2.2755614E38F}));
            assert(pack.time_usec_GET() == 6064007519374613363L);
            assert(pack.target_component_GET() == (char)69);
            assert(pack.group_mlx_GET() == (char)217);
            assert(pack.target_system_GET() == (char)154);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.target_system_SET((char)154) ;
        p139.target_component_SET((char)69) ;
        p139.time_usec_SET(6064007519374613363L) ;
        p139.group_mlx_SET((char)217) ;
        p139.controls_SET(new float[] {-2.93452E38F, -4.766794E37F, 1.7501543E38F, -7.351998E37F, 3.2983877E38F, 2.1337823E38F, -2.422981E38F, 2.2755614E38F}, 0) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)116);
            assert(pack.time_usec_GET() == 8001765441841354518L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.1415732E37F, -3.3836162E38F, -7.830443E37F, -6.002406E37F, -3.1421472E38F, 3.5453094E37F, -3.290835E38F, 4.409858E37F}));
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.group_mlx_SET((char)116) ;
        p140.time_usec_SET(8001765441841354518L) ;
        p140.controls_SET(new float[] {2.1415732E37F, -3.3836162E38F, -7.830443E37F, -6.002406E37F, -3.1421472E38F, 3.5453094E37F, -3.290835E38F, 4.409858E37F}, 0) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_terrain_GET() == 6.963049E37F);
            assert(pack.time_usec_GET() == 9142362202514508513L);
            assert(pack.altitude_local_GET() == 5.957942E37F);
            assert(pack.altitude_relative_GET() == -3.2808562E38F);
            assert(pack.altitude_amsl_GET() == -7.552427E37F);
            assert(pack.bottom_clearance_GET() == -4.4317085E37F);
            assert(pack.altitude_monotonic_GET() == -1.2720548E37F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_terrain_SET(6.963049E37F) ;
        p141.altitude_local_SET(5.957942E37F) ;
        p141.altitude_relative_SET(-3.2808562E38F) ;
        p141.time_usec_SET(9142362202514508513L) ;
        p141.altitude_amsl_SET(-7.552427E37F) ;
        p141.bottom_clearance_SET(-4.4317085E37F) ;
        p141.altitude_monotonic_SET(-1.2720548E37F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)244, (char)174, (char)199, (char)162, (char)228, (char)60, (char)6, (char)165, (char)115, (char)155, (char)170, (char)82, (char)107, (char)149, (char)63, (char)221, (char)254, (char)80, (char)119, (char)100, (char)232, (char)36, (char)40, (char)122, (char)93, (char)216, (char)35, (char)239, (char)182, (char)144, (char)165, (char)47, (char)223, (char)21, (char)112, (char)196, (char)142, (char)156, (char)149, (char)193, (char)197, (char)34, (char)119, (char)47, (char)28, (char)239, (char)92, (char)90, (char)173, (char)74, (char)164, (char)10, (char)74, (char)146, (char)185, (char)229, (char)100, (char)128, (char)76, (char)8, (char)110, (char)118, (char)6, (char)2, (char)174, (char)203, (char)198, (char)77, (char)231, (char)30, (char)208, (char)183, (char)27, (char)29, (char)198, (char)96, (char)253, (char)161, (char)75, (char)3, (char)43, (char)219, (char)73, (char)131, (char)165, (char)8, (char)219, (char)140, (char)121, (char)111, (char)41, (char)14, (char)90, (char)99, (char)147, (char)158, (char)146, (char)111, (char)168, (char)109, (char)188, (char)29, (char)104, (char)174, (char)20, (char)128, (char)230, (char)120, (char)112, (char)197, (char)162, (char)219, (char)47, (char)29, (char)217, (char)228, (char)64, (char)184, (char)253, (char)85}));
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)52, (char)116, (char)114, (char)49, (char)134, (char)14, (char)131, (char)75, (char)157, (char)157, (char)70, (char)92, (char)192, (char)208, (char)21, (char)80, (char)223, (char)81, (char)218, (char)31, (char)206, (char)49, (char)234, (char)23, (char)60, (char)18, (char)0, (char)248, (char)7, (char)11, (char)241, (char)39, (char)132, (char)78, (char)40, (char)237, (char)184, (char)138, (char)164, (char)100, (char)120, (char)198, (char)157, (char)137, (char)244, (char)73, (char)238, (char)70, (char)104, (char)88, (char)218, (char)65, (char)35, (char)162, (char)140, (char)226, (char)213, (char)243, (char)202, (char)56, (char)34, (char)161, (char)159, (char)33, (char)222, (char)88, (char)82, (char)170, (char)73, (char)237, (char)159, (char)95, (char)35, (char)24, (char)61, (char)109, (char)22, (char)13, (char)1, (char)1, (char)215, (char)180, (char)33, (char)237, (char)101, (char)207, (char)81, (char)36, (char)221, (char)195, (char)166, (char)17, (char)188, (char)174, (char)165, (char)113, (char)185, (char)246, (char)171, (char)52, (char)116, (char)201, (char)66, (char)178, (char)31, (char)19, (char)192, (char)150, (char)99, (char)199, (char)10, (char)183, (char)188, (char)203, (char)230, (char)37, (char)20, (char)85, (char)215, (char)177}));
            assert(pack.transfer_type_GET() == (char)69);
            assert(pack.request_id_GET() == (char)233);
            assert(pack.uri_type_GET() == (char)239);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.request_id_SET((char)233) ;
        p142.storage_SET(new char[] {(char)244, (char)174, (char)199, (char)162, (char)228, (char)60, (char)6, (char)165, (char)115, (char)155, (char)170, (char)82, (char)107, (char)149, (char)63, (char)221, (char)254, (char)80, (char)119, (char)100, (char)232, (char)36, (char)40, (char)122, (char)93, (char)216, (char)35, (char)239, (char)182, (char)144, (char)165, (char)47, (char)223, (char)21, (char)112, (char)196, (char)142, (char)156, (char)149, (char)193, (char)197, (char)34, (char)119, (char)47, (char)28, (char)239, (char)92, (char)90, (char)173, (char)74, (char)164, (char)10, (char)74, (char)146, (char)185, (char)229, (char)100, (char)128, (char)76, (char)8, (char)110, (char)118, (char)6, (char)2, (char)174, (char)203, (char)198, (char)77, (char)231, (char)30, (char)208, (char)183, (char)27, (char)29, (char)198, (char)96, (char)253, (char)161, (char)75, (char)3, (char)43, (char)219, (char)73, (char)131, (char)165, (char)8, (char)219, (char)140, (char)121, (char)111, (char)41, (char)14, (char)90, (char)99, (char)147, (char)158, (char)146, (char)111, (char)168, (char)109, (char)188, (char)29, (char)104, (char)174, (char)20, (char)128, (char)230, (char)120, (char)112, (char)197, (char)162, (char)219, (char)47, (char)29, (char)217, (char)228, (char)64, (char)184, (char)253, (char)85}, 0) ;
        p142.uri_SET(new char[] {(char)52, (char)116, (char)114, (char)49, (char)134, (char)14, (char)131, (char)75, (char)157, (char)157, (char)70, (char)92, (char)192, (char)208, (char)21, (char)80, (char)223, (char)81, (char)218, (char)31, (char)206, (char)49, (char)234, (char)23, (char)60, (char)18, (char)0, (char)248, (char)7, (char)11, (char)241, (char)39, (char)132, (char)78, (char)40, (char)237, (char)184, (char)138, (char)164, (char)100, (char)120, (char)198, (char)157, (char)137, (char)244, (char)73, (char)238, (char)70, (char)104, (char)88, (char)218, (char)65, (char)35, (char)162, (char)140, (char)226, (char)213, (char)243, (char)202, (char)56, (char)34, (char)161, (char)159, (char)33, (char)222, (char)88, (char)82, (char)170, (char)73, (char)237, (char)159, (char)95, (char)35, (char)24, (char)61, (char)109, (char)22, (char)13, (char)1, (char)1, (char)215, (char)180, (char)33, (char)237, (char)101, (char)207, (char)81, (char)36, (char)221, (char)195, (char)166, (char)17, (char)188, (char)174, (char)165, (char)113, (char)185, (char)246, (char)171, (char)52, (char)116, (char)201, (char)66, (char)178, (char)31, (char)19, (char)192, (char)150, (char)99, (char)199, (char)10, (char)183, (char)188, (char)203, (char)230, (char)37, (char)20, (char)85, (char)215, (char)177}, 0) ;
        p142.uri_type_SET((char)239) ;
        p142.transfer_type_SET((char)69) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == 3.1880198E38F);
            assert(pack.press_abs_GET() == 7.447884E37F);
            assert(pack.temperature_GET() == (short)18441);
            assert(pack.time_boot_ms_GET() == 1351453972L);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.press_abs_SET(7.447884E37F) ;
        p143.temperature_SET((short)18441) ;
        p143.press_diff_SET(3.1880198E38F) ;
        p143.time_boot_ms_SET(1351453972L) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.est_capabilities_GET() == (char)29);
            assert(Arrays.equals(pack.rates_GET(),  new float[] {2.6238672E38F, -7.1226453E37F, -2.9335595E37F}));
            assert(Arrays.equals(pack.vel_GET(),  new float[] {2.425229E38F, 1.7614341E37F, -1.6101615E38F}));
            assert(pack.lat_GET() == 1376063424);
            assert(pack.lon_GET() == 1644780188);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {2.968609E37F, -2.5447784E38F, 1.7076136E38F}));
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {1.791305E38F, 2.7577234E38F, 1.652199E38F, -2.157411E37F}));
            assert(pack.custom_state_GET() == 8441547400329290291L);
            assert(pack.timestamp_GET() == 7040442328244382203L);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-1.081458E38F, -3.4805568E37F, -8.0987844E37F}));
            assert(pack.alt_GET() == 1.5991996E38F);
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.rates_SET(new float[] {2.6238672E38F, -7.1226453E37F, -2.9335595E37F}, 0) ;
        p144.custom_state_SET(8441547400329290291L) ;
        p144.position_cov_SET(new float[] {2.968609E37F, -2.5447784E38F, 1.7076136E38F}, 0) ;
        p144.lon_SET(1644780188) ;
        p144.alt_SET(1.5991996E38F) ;
        p144.acc_SET(new float[] {-1.081458E38F, -3.4805568E37F, -8.0987844E37F}, 0) ;
        p144.lat_SET(1376063424) ;
        p144.attitude_q_SET(new float[] {1.791305E38F, 2.7577234E38F, 1.652199E38F, -2.157411E37F}, 0) ;
        p144.vel_SET(new float[] {2.425229E38F, 1.7614341E37F, -1.6101615E38F}, 0) ;
        p144.timestamp_SET(7040442328244382203L) ;
        p144.est_capabilities_SET((char)29) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.x_acc_GET() == 1.2487986E38F);
            assert(pack.z_acc_GET() == -1.0442011E38F);
            assert(pack.y_pos_GET() == 1.7558095E38F);
            assert(pack.yaw_rate_GET() == -5.9601564E37F);
            assert(pack.x_vel_GET() == 1.2287005E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {-2.3549636E38F, -3.2424432E38F, -1.4508113E38F}));
            assert(pack.z_pos_GET() == -2.8842773E38F);
            assert(pack.y_vel_GET() == 1.4557044E38F);
            assert(pack.time_usec_GET() == 5807465087656677570L);
            assert(pack.y_acc_GET() == -1.4839061E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.208124E38F, 1.4271895E38F, -2.1982082E38F, -6.2623334E37F}));
            assert(pack.x_pos_GET() == 3.053732E36F);
            assert(pack.pitch_rate_GET() == 2.4181171E38F);
            assert(pack.airspeed_GET() == 1.9757825E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {1.8946205E35F, 2.4328633E38F, 3.2811318E38F}));
            assert(pack.roll_rate_GET() == -3.1672883E38F);
            assert(pack.z_vel_GET() == 1.803622E38F);
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.z_acc_SET(-1.0442011E38F) ;
        p146.q_SET(new float[] {-3.208124E38F, 1.4271895E38F, -2.1982082E38F, -6.2623334E37F}, 0) ;
        p146.z_vel_SET(1.803622E38F) ;
        p146.yaw_rate_SET(-5.9601564E37F) ;
        p146.z_pos_SET(-2.8842773E38F) ;
        p146.y_acc_SET(-1.4839061E38F) ;
        p146.time_usec_SET(5807465087656677570L) ;
        p146.pitch_rate_SET(2.4181171E38F) ;
        p146.x_acc_SET(1.2487986E38F) ;
        p146.y_vel_SET(1.4557044E38F) ;
        p146.vel_variance_SET(new float[] {1.8946205E35F, 2.4328633E38F, 3.2811318E38F}, 0) ;
        p146.pos_variance_SET(new float[] {-2.3549636E38F, -3.2424432E38F, -1.4508113E38F}, 0) ;
        p146.x_vel_SET(1.2287005E38F) ;
        p146.airspeed_SET(1.9757825E38F) ;
        p146.y_pos_SET(1.7558095E38F) ;
        p146.roll_rate_SET(-3.1672883E38F) ;
        p146.x_pos_SET(3.053732E36F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
            assert(pack.battery_remaining_GET() == (byte) - 93);
            assert(pack.energy_consumed_GET() == 1606219519);
            assert(pack.id_GET() == (char)137);
            assert(pack.current_battery_GET() == (short)12483);
            assert(pack.temperature_GET() == (short)14952);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
            assert(pack.current_consumed_GET() == -1874996705);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)49240, (char)26666, (char)10858, (char)45595, (char)43244, (char)63480, (char)30701, (char)40709, (char)23363, (char)60506}));
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.temperature_SET((short)14952) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL) ;
        p147.energy_consumed_SET(1606219519) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION) ;
        p147.current_consumed_SET(-1874996705) ;
        p147.id_SET((char)137) ;
        p147.voltages_SET(new char[] {(char)49240, (char)26666, (char)10858, (char)45595, (char)43244, (char)63480, (char)30701, (char)40709, (char)23363, (char)60506}, 0) ;
        p147.battery_remaining_SET((byte) - 93) ;
        p147.current_battery_SET((short)12483) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)190, (char)25, (char)159, (char)81, (char)101, (char)177, (char)97, (char)160}));
            assert(pack.flight_sw_version_GET() == 2571315011L);
            assert(pack.os_sw_version_GET() == 2006474192L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)156, (char)231, (char)179, (char)172, (char)173, (char)3, (char)113, (char)168, (char)31, (char)162, (char)228, (char)16, (char)214, (char)61, (char)40, (char)70, (char)133, (char)240}));
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)82, (char)147, (char)253, (char)10, (char)54, (char)58, (char)239, (char)247}));
            assert(pack.board_version_GET() == 2903550845L);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)170, (char)195, (char)169, (char)216, (char)101, (char)68, (char)103, (char)189}));
            assert(pack.vendor_id_GET() == (char)9958);
            assert(pack.capabilities_GET() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE);
            assert(pack.middleware_sw_version_GET() == 907235614L);
            assert(pack.product_id_GET() == (char)59447);
            assert(pack.uid_GET() == 6609886811400878217L);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.uid_SET(6609886811400878217L) ;
        p148.vendor_id_SET((char)9958) ;
        p148.os_custom_version_SET(new char[] {(char)170, (char)195, (char)169, (char)216, (char)101, (char)68, (char)103, (char)189}, 0) ;
        p148.uid2_SET(new char[] {(char)156, (char)231, (char)179, (char)172, (char)173, (char)3, (char)113, (char)168, (char)31, (char)162, (char)228, (char)16, (char)214, (char)61, (char)40, (char)70, (char)133, (char)240}, 0, PH) ;
        p148.middleware_custom_version_SET(new char[] {(char)82, (char)147, (char)253, (char)10, (char)54, (char)58, (char)239, (char)247}, 0) ;
        p148.middleware_sw_version_SET(907235614L) ;
        p148.product_id_SET((char)59447) ;
        p148.flight_custom_version_SET(new char[] {(char)190, (char)25, (char)159, (char)81, (char)101, (char)177, (char)97, (char)160}, 0) ;
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE) ;
        p148.flight_sw_version_SET(2571315011L) ;
        p148.os_sw_version_SET(2006474192L) ;
        p148.board_version_SET(2903550845L) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.position_valid_TRY(ph) == (char)244);
            assert(pack.z_TRY(ph) == -2.0577721E38F);
            assert(pack.target_num_GET() == (char)134);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {-1.0917627E38F, -3.358357E38F, 4.3044643E37F, -1.5087056E38F}));
            assert(pack.y_TRY(ph) == 3.3487009E38F);
            assert(pack.angle_x_GET() == 1.7777708E38F);
            assert(pack.time_usec_GET() == 7277508588511318063L);
            assert(pack.x_TRY(ph) == -3.244138E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.size_y_GET() == 2.6619004E38F);
            assert(pack.size_x_GET() == 1.6552923E38F);
            assert(pack.angle_y_GET() == -2.068216E38F);
            assert(pack.distance_GET() == -1.3150281E38F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER) ;
        p149.target_num_SET((char)134) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p149.distance_SET(-1.3150281E38F) ;
        p149.y_SET(3.3487009E38F, PH) ;
        p149.angle_y_SET(-2.068216E38F) ;
        p149.q_SET(new float[] {-1.0917627E38F, -3.358357E38F, 4.3044643E37F, -1.5087056E38F}, 0, PH) ;
        p149.time_usec_SET(7277508588511318063L) ;
        p149.size_y_SET(2.6619004E38F) ;
        p149.size_x_SET(1.6552923E38F) ;
        p149.x_SET(-3.244138E38F, PH) ;
        p149.angle_x_SET(1.7777708E38F) ;
        p149.z_SET(-2.0577721E38F, PH) ;
        p149.position_valid_SET((char)244, PH) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_CPU_LOAD.add((src, ph, pack) ->
        {
            assert(pack.ctrlLoad_GET() == (char)236);
            assert(pack.batVolt_GET() == (char)5674);
            assert(pack.sensLoad_GET() == (char)213);
        });
        GroundControl.CPU_LOAD p170 = CommunicationChannel.new_CPU_LOAD();
        PH.setPack(p170);
        p170.sensLoad_SET((char)213) ;
        p170.batVolt_SET((char)5674) ;
        p170.ctrlLoad_SET((char)236) ;
        CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SENSOR_BIAS.add((src, ph, pack) ->
        {
            assert(pack.gxBias_GET() == -5.537095E37F);
            assert(pack.axBias_GET() == -2.2264267E38F);
            assert(pack.azBias_GET() == -3.1566145E38F);
            assert(pack.gyBias_GET() == 1.8684403E38F);
            assert(pack.gzBias_GET() == -2.9998483E38F);
            assert(pack.ayBias_GET() == -2.5612657E38F);
        });
        GroundControl.SENSOR_BIAS p172 = CommunicationChannel.new_SENSOR_BIAS();
        PH.setPack(p172);
        p172.gxBias_SET(-5.537095E37F) ;
        p172.gyBias_SET(1.8684403E38F) ;
        p172.ayBias_SET(-2.5612657E38F) ;
        p172.azBias_SET(-3.1566145E38F) ;
        p172.axBias_SET(-2.2264267E38F) ;
        p172.gzBias_SET(-2.9998483E38F) ;
        CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DIAGNOSTIC.add((src, ph, pack) ->
        {
            assert(pack.diagSh1_GET() == (short)8750);
            assert(pack.diagFl2_GET() == -2.0356538E38F);
            assert(pack.diagFl3_GET() == 2.367716E38F);
            assert(pack.diagSh2_GET() == (short)13677);
            assert(pack.diagSh3_GET() == (short)11320);
            assert(pack.diagFl1_GET() == 1.3064597E38F);
        });
        GroundControl.DIAGNOSTIC p173 = CommunicationChannel.new_DIAGNOSTIC();
        PH.setPack(p173);
        p173.diagSh3_SET((short)11320) ;
        p173.diagFl3_SET(2.367716E38F) ;
        p173.diagSh1_SET((short)8750) ;
        p173.diagFl2_SET(-2.0356538E38F) ;
        p173.diagFl1_SET(1.3064597E38F) ;
        p173.diagSh2_SET((short)13677) ;
        CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SLUGS_NAVIGATION.add((src, ph, pack) ->
        {
            assert(pack.psiDot_c_GET() == 2.1377074E38F);
            assert(pack.toWP_GET() == (char)183);
            assert(pack.h_c_GET() == (char)36464);
            assert(pack.dist2Go_GET() == -9.323965E37F);
            assert(pack.totalDist_GET() == -3.1118855E38F);
            assert(pack.fromWP_GET() == (char)40);
            assert(pack.theta_c_GET() == -3.0160756E38F);
            assert(pack.u_m_GET() == 4.7374263E37F);
            assert(pack.phi_c_GET() == -2.947784E38F);
            assert(pack.ay_body_GET() == -8.2053274E37F);
        });
        GroundControl.SLUGS_NAVIGATION p176 = CommunicationChannel.new_SLUGS_NAVIGATION();
        PH.setPack(p176);
        p176.dist2Go_SET(-9.323965E37F) ;
        p176.h_c_SET((char)36464) ;
        p176.totalDist_SET(-3.1118855E38F) ;
        p176.fromWP_SET((char)40) ;
        p176.ay_body_SET(-8.2053274E37F) ;
        p176.phi_c_SET(-2.947784E38F) ;
        p176.u_m_SET(4.7374263E37F) ;
        p176.psiDot_c_SET(2.1377074E38F) ;
        p176.theta_c_SET(-3.0160756E38F) ;
        p176.toWP_SET((char)183) ;
        CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DATA_LOG.add((src, ph, pack) ->
        {
            assert(pack.fl_2_GET() == -2.3860042E38F);
            assert(pack.fl_3_GET() == 2.7179802E38F);
            assert(pack.fl_6_GET() == 2.3642333E38F);
            assert(pack.fl_4_GET() == 1.6219287E38F);
            assert(pack.fl_5_GET() == -2.0060058E38F);
            assert(pack.fl_1_GET() == -1.4649646E38F);
        });
        GroundControl.DATA_LOG p177 = CommunicationChannel.new_DATA_LOG();
        PH.setPack(p177);
        p177.fl_2_SET(-2.3860042E38F) ;
        p177.fl_1_SET(-1.4649646E38F) ;
        p177.fl_3_SET(2.7179802E38F) ;
        p177.fl_6_SET(2.3642333E38F) ;
        p177.fl_5_SET(-2.0060058E38F) ;
        p177.fl_4_SET(1.6219287E38F) ;
        CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_DATE_TIME.add((src, ph, pack) ->
        {
            assert(pack.month_GET() == (char)142);
            assert(pack.percentUsed_GET() == (char)45);
            assert(pack.visSat_GET() == (char)19);
            assert(pack.useSat_GET() == (char)149);
            assert(pack.hour_GET() == (char)143);
            assert(pack.sec_GET() == (char)6);
            assert(pack.year_GET() == (char)44);
            assert(pack.min_GET() == (char)167);
            assert(pack.day_GET() == (char)122);
            assert(pack.sigUsedMask_GET() == (char)118);
            assert(pack.GppGl_GET() == (char)76);
            assert(pack.clockStat_GET() == (char)113);
        });
        GroundControl.GPS_DATE_TIME p179 = CommunicationChannel.new_GPS_DATE_TIME();
        PH.setPack(p179);
        p179.useSat_SET((char)149) ;
        p179.hour_SET((char)143) ;
        p179.min_SET((char)167) ;
        p179.year_SET((char)44) ;
        p179.sec_SET((char)6) ;
        p179.month_SET((char)142) ;
        p179.sigUsedMask_SET((char)118) ;
        p179.day_SET((char)122) ;
        p179.clockStat_SET((char)113) ;
        p179.GppGl_SET((char)76) ;
        p179.percentUsed_SET((char)45) ;
        p179.visSat_SET((char)19) ;
        CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MID_LVL_CMDS.add((src, ph, pack) ->
        {
            assert(pack.target_GET() == (char)48);
            assert(pack.hCommand_GET() == 3.2707655E38F);
            assert(pack.uCommand_GET() == -9.62046E37F);
            assert(pack.rCommand_GET() == 2.6890317E38F);
        });
        GroundControl.MID_LVL_CMDS p180 = CommunicationChannel.new_MID_LVL_CMDS();
        PH.setPack(p180);
        p180.rCommand_SET(2.6890317E38F) ;
        p180.uCommand_SET(-9.62046E37F) ;
        p180.hCommand_SET(3.2707655E38F) ;
        p180.target_SET((char)48) ;
        CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CTRL_SRFC_PT.add((src, ph, pack) ->
        {
            assert(pack.target_GET() == (char)247);
            assert(pack.bitfieldPt_GET() == (char)35597);
        });
        GroundControl.CTRL_SRFC_PT p181 = CommunicationChannel.new_CTRL_SRFC_PT();
        PH.setPack(p181);
        p181.bitfieldPt_SET((char)35597) ;
        p181.target_SET((char)247) ;
        CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SLUGS_CAMERA_ORDER.add((src, ph, pack) ->
        {
            assert(pack.zoom_GET() == (byte)119);
            assert(pack.moveHome_GET() == (byte) - 9);
            assert(pack.pan_GET() == (byte) - 19);
            assert(pack.tilt_GET() == (byte) - 51);
            assert(pack.target_GET() == (char)54);
        });
        GroundControl.SLUGS_CAMERA_ORDER p184 = CommunicationChannel.new_SLUGS_CAMERA_ORDER();
        PH.setPack(p184);
        p184.tilt_SET((byte) - 51) ;
        p184.moveHome_SET((byte) - 9) ;
        p184.pan_SET((byte) - 19) ;
        p184.target_SET((char)54) ;
        p184.zoom_SET((byte)119) ;
        CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CONTROL_SURFACE.add((src, ph, pack) ->
        {
            assert(pack.mControl_GET() == -2.6692736E38F);
            assert(pack.target_GET() == (char)205);
            assert(pack.bControl_GET() == -1.5126061E38F);
            assert(pack.idSurface_GET() == (char)44);
        });
        GroundControl.CONTROL_SURFACE p185 = CommunicationChannel.new_CONTROL_SURFACE();
        PH.setPack(p185);
        p185.mControl_SET(-2.6692736E38F) ;
        p185.bControl_SET(-1.5126061E38F) ;
        p185.idSurface_SET((char)44) ;
        p185.target_SET((char)205) ;
        CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SLUGS_MOBILE_LOCATION.add((src, ph, pack) ->
        {
            assert(pack.target_GET() == (char)217);
            assert(pack.longitude_GET() == 7.720786E36F);
            assert(pack.latitude_GET() == -3.2319568E37F);
        });
        GroundControl.SLUGS_MOBILE_LOCATION p186 = CommunicationChannel.new_SLUGS_MOBILE_LOCATION();
        PH.setPack(p186);
        p186.longitude_SET(7.720786E36F) ;
        p186.latitude_SET(-3.2319568E37F) ;
        p186.target_SET((char)217) ;
        CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SLUGS_CONFIGURATION_CAMERA.add((src, ph, pack) ->
        {
            assert(pack.order_GET() == (char)63);
            assert(pack.target_GET() == (char)93);
            assert(pack.idOrder_GET() == (char)42);
        });
        GroundControl.SLUGS_CONFIGURATION_CAMERA p188 = CommunicationChannel.new_SLUGS_CONFIGURATION_CAMERA();
        PH.setPack(p188);
        p188.order_SET((char)63) ;
        p188.idOrder_SET((char)42) ;
        p188.target_SET((char)93) ;
        CommunicationChannel.instance.send(p188);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ISR_LOCATION.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == -2.0775428E38F);
            assert(pack.target_GET() == (char)204);
            assert(pack.option1_GET() == (char)60);
            assert(pack.option3_GET() == (char)178);
            assert(pack.option2_GET() == (char)223);
            assert(pack.height_GET() == -5.2348174E37F);
            assert(pack.longitude_GET() == -8.624639E37F);
        });
        GroundControl.ISR_LOCATION p189 = CommunicationChannel.new_ISR_LOCATION();
        PH.setPack(p189);
        p189.longitude_SET(-8.624639E37F) ;
        p189.target_SET((char)204) ;
        p189.option3_SET((char)178) ;
        p189.option2_SET((char)223) ;
        p189.latitude_SET(-2.0775428E38F) ;
        p189.option1_SET((char)60) ;
        p189.height_SET(-5.2348174E37F) ;
        CommunicationChannel.instance.send(p189);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VOLT_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.r2Type_GET() == (char)8);
            assert(pack.voltage_GET() == (char)7080);
            assert(pack.reading2_GET() == (char)43628);
        });
        GroundControl.VOLT_SENSOR p191 = CommunicationChannel.new_VOLT_SENSOR();
        PH.setPack(p191);
        p191.r2Type_SET((char)8) ;
        p191.voltage_SET((char)7080) ;
        p191.reading2_SET((char)43628) ;
        CommunicationChannel.instance.send(p191);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PTZ_STATUS.add((src, ph, pack) ->
        {
            assert(pack.zoom_GET() == (char)43);
            assert(pack.tilt_GET() == (short) -27286);
            assert(pack.pan_GET() == (short) -6508);
        });
        GroundControl.PTZ_STATUS p192 = CommunicationChannel.new_PTZ_STATUS();
        PH.setPack(p192);
        p192.zoom_SET((char)43) ;
        p192.tilt_SET((short) -27286) ;
        p192.pan_SET((short) -6508) ;
        CommunicationChannel.instance.send(p192);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAV_STATUS.add((src, ph, pack) ->
        {
            assert(pack.course_GET() == 6.790444E37F);
            assert(pack.target_GET() == (char)234);
            assert(pack.latitude_GET() == 1.607423E38F);
            assert(pack.longitude_GET() == 2.8608907E38F);
            assert(pack.altitude_GET() == -7.710153E34F);
            assert(pack.speed_GET() == -1.1974907E38F);
        });
        GroundControl.UAV_STATUS p193 = CommunicationChannel.new_UAV_STATUS();
        PH.setPack(p193);
        p193.longitude_SET(2.8608907E38F) ;
        p193.altitude_SET(-7.710153E34F) ;
        p193.course_SET(6.790444E37F) ;
        p193.speed_SET(-1.1974907E38F) ;
        p193.latitude_SET(1.607423E38F) ;
        p193.target_SET((char)234) ;
        CommunicationChannel.instance.send(p193);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STATUS_GPS.add((src, ph, pack) ->
        {
            assert(pack.magVar_GET() == 1.3042412E38F);
            assert(pack.modeInd_GET() == (char)39);
            assert(pack.posStatus_GET() == (char)28);
            assert(pack.magDir_GET() == (byte)100);
            assert(pack.csFails_GET() == (char)29045);
            assert(pack.gpsQuality_GET() == (char)133);
            assert(pack.msgsType_GET() == (char)146);
        });
        GroundControl.STATUS_GPS p194 = CommunicationChannel.new_STATUS_GPS();
        PH.setPack(p194);
        p194.magVar_SET(1.3042412E38F) ;
        p194.magDir_SET((byte)100) ;
        p194.posStatus_SET((char)28) ;
        p194.csFails_SET((char)29045) ;
        p194.msgsType_SET((char)146) ;
        p194.gpsQuality_SET((char)133) ;
        p194.modeInd_SET((char)39) ;
        CommunicationChannel.instance.send(p194);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NOVATEL_DIAG.add((src, ph, pack) ->
        {
            assert(pack.posSolAge_GET() == -3.0529085E38F);
            assert(pack.posType_GET() == (char)110);
            assert(pack.csFails_GET() == (char)60307);
            assert(pack.timeStatus_GET() == (char)142);
            assert(pack.solStatus_GET() == (char)242);
            assert(pack.receiverStatus_GET() == 3726139916L);
            assert(pack.velType_GET() == (char)195);
        });
        GroundControl.NOVATEL_DIAG p195 = CommunicationChannel.new_NOVATEL_DIAG();
        PH.setPack(p195);
        p195.receiverStatus_SET(3726139916L) ;
        p195.timeStatus_SET((char)142) ;
        p195.posSolAge_SET(-3.0529085E38F) ;
        p195.csFails_SET((char)60307) ;
        p195.velType_SET((char)195) ;
        p195.solStatus_SET((char)242) ;
        p195.posType_SET((char)110) ;
        CommunicationChannel.instance.send(p195);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SENSOR_DIAG.add((src, ph, pack) ->
        {
            assert(pack.char1_GET() == (byte)124);
            assert(pack.float2_GET() == -1.6850941E38F);
            assert(pack.int1_GET() == (short)18932);
            assert(pack.float1_GET() == 3.2818407E38F);
        });
        GroundControl.SENSOR_DIAG p196 = CommunicationChannel.new_SENSOR_DIAG();
        PH.setPack(p196);
        p196.float1_SET(3.2818407E38F) ;
        p196.float2_SET(-1.6850941E38F) ;
        p196.char1_SET((byte)124) ;
        p196.int1_SET((short)18932) ;
        CommunicationChannel.instance.send(p196);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BOOT.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == 3313568463L);
        });
        GroundControl.BOOT p197 = CommunicationChannel.new_BOOT();
        PH.setPack(p197);
        p197.version_SET(3313568463L) ;
        CommunicationChannel.instance.send(p197);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        CommunicationChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE);
            assert(pack.pos_horiz_ratio_GET() == -1.7085113E38F);
            assert(pack.vel_ratio_GET() == -6.752197E37F);
            assert(pack.hagl_ratio_GET() == 4.0598927E37F);
            assert(pack.pos_horiz_accuracy_GET() == 2.8645307E38F);
            assert(pack.pos_vert_accuracy_GET() == 2.4461779E38F);
            assert(pack.pos_vert_ratio_GET() == 5.6117356E37F);
            assert(pack.mag_ratio_GET() == -3.0568372E37F);
            assert(pack.tas_ratio_GET() == -2.0488994E38F);
            assert(pack.time_usec_GET() == 5302928954185130735L);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.pos_horiz_accuracy_SET(2.8645307E38F) ;
        p230.pos_vert_ratio_SET(5.6117356E37F) ;
        p230.hagl_ratio_SET(4.0598927E37F) ;
        p230.tas_ratio_SET(-2.0488994E38F) ;
        p230.time_usec_SET(5302928954185130735L) ;
        p230.mag_ratio_SET(-3.0568372E37F) ;
        p230.vel_ratio_SET(-6.752197E37F) ;
        p230.pos_horiz_ratio_SET(-1.7085113E38F) ;
        p230.pos_vert_accuracy_SET(2.4461779E38F) ;
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_y_GET() == -1.1741283E37F);
            assert(pack.vert_accuracy_GET() == 2.0942134E38F);
            assert(pack.var_horiz_GET() == 5.5330525E37F);
            assert(pack.wind_x_GET() == 3.0697482E38F);
            assert(pack.wind_z_GET() == -1.7475787E38F);
            assert(pack.var_vert_GET() == -4.1313207E37F);
            assert(pack.time_usec_GET() == 4330498924329936128L);
            assert(pack.horiz_accuracy_GET() == -2.5313405E38F);
            assert(pack.wind_alt_GET() == 2.152178E38F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_z_SET(-1.7475787E38F) ;
        p231.var_vert_SET(-4.1313207E37F) ;
        p231.wind_y_SET(-1.1741283E37F) ;
        p231.wind_x_SET(3.0697482E38F) ;
        p231.horiz_accuracy_SET(-2.5313405E38F) ;
        p231.vert_accuracy_SET(2.0942134E38F) ;
        p231.time_usec_SET(4330498924329936128L) ;
        p231.var_horiz_SET(5.5330525E37F) ;
        p231.wind_alt_SET(2.152178E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -3.3812363E38F);
            assert(pack.horiz_accuracy_GET() == -2.9964054E38F);
            assert(pack.speed_accuracy_GET() == -8.4966305E37F);
            assert(pack.gps_id_GET() == (char)101);
            assert(pack.vdop_GET() == 1.0680281E38F);
            assert(pack.hdop_GET() == 1.9132876E38F);
            assert(pack.time_week_ms_GET() == 1405916348L);
            assert(pack.ve_GET() == 2.5805515E38F);
            assert(pack.vert_accuracy_GET() == -3.2931873E38F);
            assert(pack.vd_GET() == -1.8025797E38F);
            assert(pack.ignore_flags_GET() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT);
            assert(pack.vn_GET() == -1.3296339E38F);
            assert(pack.lat_GET() == 1015551868);
            assert(pack.fix_type_GET() == (char)46);
            assert(pack.time_week_GET() == (char)2647);
            assert(pack.time_usec_GET() == 5988142370262579722L);
            assert(pack.lon_GET() == 1358918143);
            assert(pack.satellites_visible_GET() == (char)177);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.horiz_accuracy_SET(-2.9964054E38F) ;
        p232.ve_SET(2.5805515E38F) ;
        p232.time_week_ms_SET(1405916348L) ;
        p232.vd_SET(-1.8025797E38F) ;
        p232.satellites_visible_SET((char)177) ;
        p232.vdop_SET(1.0680281E38F) ;
        p232.time_usec_SET(5988142370262579722L) ;
        p232.alt_SET(-3.3812363E38F) ;
        p232.gps_id_SET((char)101) ;
        p232.lon_SET(1358918143) ;
        p232.hdop_SET(1.9132876E38F) ;
        p232.lat_SET(1015551868) ;
        p232.vn_SET(-1.3296339E38F) ;
        p232.fix_type_SET((char)46) ;
        p232.time_week_SET((char)2647) ;
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT) ;
        p232.speed_accuracy_SET(-8.4966305E37F) ;
        p232.vert_accuracy_SET(-3.2931873E38F) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (char)138);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)30, (char)227, (char)37, (char)139, (char)173, (char)95, (char)226, (char)53, (char)54, (char)110, (char)96, (char)240, (char)105, (char)52, (char)197, (char)36, (char)153, (char)211, (char)8, (char)18, (char)5, (char)64, (char)116, (char)23, (char)1, (char)66, (char)177, (char)100, (char)93, (char)196, (char)145, (char)81, (char)13, (char)121, (char)59, (char)171, (char)216, (char)156, (char)158, (char)93, (char)232, (char)91, (char)48, (char)206, (char)4, (char)42, (char)211, (char)217, (char)81, (char)225, (char)144, (char)26, (char)135, (char)100, (char)80, (char)129, (char)229, (char)71, (char)236, (char)255, (char)189, (char)152, (char)76, (char)148, (char)155, (char)244, (char)37, (char)120, (char)155, (char)200, (char)90, (char)221, (char)183, (char)247, (char)56, (char)166, (char)40, (char)101, (char)91, (char)67, (char)251, (char)145, (char)118, (char)243, (char)52, (char)255, (char)144, (char)109, (char)161, (char)197, (char)162, (char)115, (char)156, (char)237, (char)117, (char)134, (char)112, (char)182, (char)179, (char)210, (char)156, (char)42, (char)210, (char)10, (char)28, (char)90, (char)48, (char)152, (char)230, (char)169, (char)245, (char)171, (char)147, (char)37, (char)190, (char)157, (char)44, (char)176, (char)170, (char)121, (char)245, (char)124, (char)140, (char)250, (char)38, (char)215, (char)2, (char)252, (char)61, (char)199, (char)82, (char)110, (char)232, (char)112, (char)213, (char)175, (char)91, (char)75, (char)240, (char)52, (char)230, (char)47, (char)136, (char)16, (char)26, (char)238, (char)137, (char)166, (char)187, (char)154, (char)107, (char)85, (char)133, (char)115, (char)157, (char)199, (char)210, (char)210, (char)92, (char)121, (char)63, (char)0, (char)153, (char)76, (char)196, (char)131, (char)238, (char)112, (char)149, (char)248, (char)250, (char)151, (char)219, (char)95, (char)134, (char)32, (char)67, (char)116, (char)210, (char)168}));
            assert(pack.len_GET() == (char)11);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)138) ;
        p233.data__SET(new char[] {(char)30, (char)227, (char)37, (char)139, (char)173, (char)95, (char)226, (char)53, (char)54, (char)110, (char)96, (char)240, (char)105, (char)52, (char)197, (char)36, (char)153, (char)211, (char)8, (char)18, (char)5, (char)64, (char)116, (char)23, (char)1, (char)66, (char)177, (char)100, (char)93, (char)196, (char)145, (char)81, (char)13, (char)121, (char)59, (char)171, (char)216, (char)156, (char)158, (char)93, (char)232, (char)91, (char)48, (char)206, (char)4, (char)42, (char)211, (char)217, (char)81, (char)225, (char)144, (char)26, (char)135, (char)100, (char)80, (char)129, (char)229, (char)71, (char)236, (char)255, (char)189, (char)152, (char)76, (char)148, (char)155, (char)244, (char)37, (char)120, (char)155, (char)200, (char)90, (char)221, (char)183, (char)247, (char)56, (char)166, (char)40, (char)101, (char)91, (char)67, (char)251, (char)145, (char)118, (char)243, (char)52, (char)255, (char)144, (char)109, (char)161, (char)197, (char)162, (char)115, (char)156, (char)237, (char)117, (char)134, (char)112, (char)182, (char)179, (char)210, (char)156, (char)42, (char)210, (char)10, (char)28, (char)90, (char)48, (char)152, (char)230, (char)169, (char)245, (char)171, (char)147, (char)37, (char)190, (char)157, (char)44, (char)176, (char)170, (char)121, (char)245, (char)124, (char)140, (char)250, (char)38, (char)215, (char)2, (char)252, (char)61, (char)199, (char)82, (char)110, (char)232, (char)112, (char)213, (char)175, (char)91, (char)75, (char)240, (char)52, (char)230, (char)47, (char)136, (char)16, (char)26, (char)238, (char)137, (char)166, (char)187, (char)154, (char)107, (char)85, (char)133, (char)115, (char)157, (char)199, (char)210, (char)210, (char)92, (char)121, (char)63, (char)0, (char)153, (char)76, (char)196, (char)131, (char)238, (char)112, (char)149, (char)248, (char)250, (char)151, (char)219, (char)95, (char)134, (char)32, (char)67, (char)116, (char)210, (char)168}, 0) ;
        p233.len_SET((char)11) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.temperature_air_GET() == (byte) - 56);
            assert(pack.longitude_GET() == 55118944);
            assert(pack.throttle_GET() == (byte)116);
            assert(pack.heading_GET() == (char)54616);
            assert(pack.roll_GET() == (short)5499);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
            assert(pack.latitude_GET() == 558374742);
            assert(pack.wp_distance_GET() == (char)64160);
            assert(pack.climb_rate_GET() == (byte)124);
            assert(pack.failsafe_GET() == (char)86);
            assert(pack.altitude_amsl_GET() == (short) -1073);
            assert(pack.gps_nsat_GET() == (char)200);
            assert(pack.heading_sp_GET() == (short) -20497);
            assert(pack.pitch_GET() == (short) -223);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
            assert(pack.temperature_GET() == (byte) - 110);
            assert(pack.wp_num_GET() == (char)134);
            assert(pack.altitude_sp_GET() == (short)638);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
            assert(pack.battery_remaining_GET() == (char)118);
            assert(pack.airspeed_sp_GET() == (char)131);
            assert(pack.groundspeed_GET() == (char)112);
            assert(pack.airspeed_GET() == (char)79);
            assert(pack.custom_mode_GET() == 3807754051L);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.gps_nsat_SET((char)200) ;
        p234.pitch_SET((short) -223) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED) ;
        p234.altitude_amsl_SET((short) -1073) ;
        p234.latitude_SET(558374742) ;
        p234.roll_SET((short)5499) ;
        p234.throttle_SET((byte)116) ;
        p234.airspeed_sp_SET((char)131) ;
        p234.failsafe_SET((char)86) ;
        p234.custom_mode_SET(3807754051L) ;
        p234.climb_rate_SET((byte)124) ;
        p234.temperature_SET((byte) - 110) ;
        p234.heading_SET((char)54616) ;
        p234.longitude_SET(55118944) ;
        p234.battery_remaining_SET((char)118) ;
        p234.groundspeed_SET((char)112) ;
        p234.wp_distance_SET((char)64160) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT) ;
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) ;
        p234.temperature_air_SET((byte) - 56) ;
        p234.airspeed_SET((char)79) ;
        p234.heading_sp_SET((short) -20497) ;
        p234.altitude_sp_SET((short)638) ;
        p234.wp_num_SET((char)134) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.vibration_x_GET() == -1.4947836E38F);
            assert(pack.clipping_2_GET() == 3163128777L);
            assert(pack.time_usec_GET() == 8385616183149291829L);
            assert(pack.vibration_y_GET() == -1.7101166E37F);
            assert(pack.vibration_z_GET() == 1.5078008E38F);
            assert(pack.clipping_0_GET() == 1570586254L);
            assert(pack.clipping_1_GET() == 119915982L);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(8385616183149291829L) ;
        p241.vibration_z_SET(1.5078008E38F) ;
        p241.vibration_y_SET(-1.7101166E37F) ;
        p241.clipping_1_SET(119915982L) ;
        p241.clipping_2_SET(3163128777L) ;
        p241.vibration_x_SET(-1.4947836E38F) ;
        p241.clipping_0_SET(1570586254L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -6.480957E37F);
            assert(pack.altitude_GET() == 1122262758);
            assert(pack.approach_z_GET() == -2.8109406E38F);
            assert(pack.x_GET() == -2.4205393E38F);
            assert(pack.approach_y_GET() == -1.9079484E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.3242673E38F, -7.9205086E37F, -8.246216E37F, -1.1797915E38F}));
            assert(pack.y_GET() == -2.015238E38F);
            assert(pack.longitude_GET() == 1794784914);
            assert(pack.latitude_GET() == -540364624);
            assert(pack.approach_x_GET() == 2.9194586E38F);
            assert(pack.time_usec_TRY(ph) == 4923411297539288497L);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.q_SET(new float[] {1.3242673E38F, -7.9205086E37F, -8.246216E37F, -1.1797915E38F}, 0) ;
        p242.longitude_SET(1794784914) ;
        p242.time_usec_SET(4923411297539288497L, PH) ;
        p242.y_SET(-2.015238E38F) ;
        p242.approach_y_SET(-1.9079484E38F) ;
        p242.z_SET(-6.480957E37F) ;
        p242.latitude_SET(-540364624) ;
        p242.x_SET(-2.4205393E38F) ;
        p242.approach_x_SET(2.9194586E38F) ;
        p242.approach_z_SET(-2.8109406E38F) ;
        p242.altitude_SET(1122262758) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)230);
            assert(pack.x_GET() == 2.2999737E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.2213666E38F, -8.769805E37F, -1.7476097E38F, 1.2753206E37F}));
            assert(pack.longitude_GET() == 872282343);
            assert(pack.approach_z_GET() == 1.1982142E38F);
            assert(pack.time_usec_TRY(ph) == 5281711102195995300L);
            assert(pack.approach_x_GET() == -9.857677E37F);
            assert(pack.y_GET() == 9.776645E37F);
            assert(pack.approach_y_GET() == 1.6659545E38F);
            assert(pack.altitude_GET() == 1321328900);
            assert(pack.latitude_GET() == -68948885);
            assert(pack.z_GET() == -2.233633E37F);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.approach_y_SET(1.6659545E38F) ;
        p243.z_SET(-2.233633E37F) ;
        p243.approach_x_SET(-9.857677E37F) ;
        p243.approach_z_SET(1.1982142E38F) ;
        p243.longitude_SET(872282343) ;
        p243.q_SET(new float[] {1.2213666E38F, -8.769805E37F, -1.7476097E38F, 1.2753206E37F}, 0) ;
        p243.latitude_SET(-68948885) ;
        p243.y_SET(9.776645E37F) ;
        p243.x_SET(2.2999737E38F) ;
        p243.target_system_SET((char)230) ;
        p243.altitude_SET(1321328900) ;
        p243.time_usec_SET(5281711102195995300L, PH) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.interval_us_GET() == -362243275);
            assert(pack.message_id_GET() == (char)30191);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(-362243275) ;
        p244.message_id_SET((char)30191) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 122828695);
            assert(pack.flags_GET() == ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS);
            assert(pack.lat_GET() == -1661918728);
            assert(pack.ver_velocity_GET() == (short)16960);
            assert(pack.callsign_LEN(ph) == 2);
            assert(pack.callsign_TRY(ph).equals("wh"));
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED);
            assert(pack.hor_velocity_GET() == (char)59574);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            assert(pack.heading_GET() == (char)5491);
            assert(pack.tslc_GET() == (char)248);
            assert(pack.squawk_GET() == (char)31444);
            assert(pack.ICAO_address_GET() == 1659094839L);
            assert(pack.altitude_GET() == -560747774);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.ver_velocity_SET((short)16960) ;
        p246.tslc_SET((char)248) ;
        p246.heading_SET((char)5491) ;
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS) ;
        p246.squawk_SET((char)31444) ;
        p246.ICAO_address_SET(1659094839L) ;
        p246.lon_SET(122828695) ;
        p246.altitude_SET(-560747774) ;
        p246.lat_SET(-1661918728) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.hor_velocity_SET((char)59574) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED) ;
        p246.callsign_SET("wh", PH) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
            assert(pack.horizontal_minimum_delta_GET() == -9.781347E37F);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
            assert(pack.time_to_minimum_delta_GET() == -2.0356814E38F);
            assert(pack.altitude_minimum_delta_GET() == 1.9155048E38F);
            assert(pack.id_GET() == 3724883771L);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.altitude_minimum_delta_SET(1.9155048E38F) ;
        p247.time_to_minimum_delta_SET(-2.0356814E38F) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH) ;
        p247.id_SET(3724883771L) ;
        p247.horizontal_minimum_delta_SET(-9.781347E37F) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)122);
            assert(pack.target_system_GET() == (char)137);
            assert(pack.target_component_GET() == (char)38);
            assert(pack.message_type_GET() == (char)10910);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)245, (char)189, (char)246, (char)254, (char)166, (char)147, (char)54, (char)200, (char)154, (char)56, (char)203, (char)87, (char)157, (char)41, (char)191, (char)158, (char)181, (char)103, (char)215, (char)177, (char)206, (char)235, (char)17, (char)230, (char)86, (char)233, (char)184, (char)236, (char)217, (char)50, (char)4, (char)186, (char)96, (char)119, (char)60, (char)61, (char)86, (char)162, (char)175, (char)17, (char)164, (char)95, (char)91, (char)87, (char)211, (char)90, (char)194, (char)138, (char)192, (char)234, (char)7, (char)155, (char)220, (char)80, (char)194, (char)124, (char)239, (char)235, (char)125, (char)152, (char)242, (char)60, (char)227, (char)192, (char)86, (char)83, (char)81, (char)44, (char)189, (char)195, (char)197, (char)33, (char)6, (char)193, (char)234, (char)140, (char)156, (char)44, (char)226, (char)240, (char)113, (char)45, (char)223, (char)5, (char)168, (char)176, (char)102, (char)226, (char)28, (char)235, (char)28, (char)101, (char)184, (char)144, (char)61, (char)125, (char)95, (char)66, (char)144, (char)242, (char)25, (char)117, (char)47, (char)117, (char)82, (char)55, (char)225, (char)158, (char)102, (char)102, (char)162, (char)116, (char)213, (char)38, (char)168, (char)39, (char)20, (char)144, (char)240, (char)130, (char)199, (char)121, (char)224, (char)53, (char)210, (char)157, (char)68, (char)110, (char)50, (char)239, (char)150, (char)226, (char)72, (char)225, (char)110, (char)238, (char)57, (char)199, (char)26, (char)16, (char)24, (char)82, (char)191, (char)145, (char)247, (char)102, (char)95, (char)231, (char)4, (char)30, (char)252, (char)188, (char)81, (char)65, (char)203, (char)15, (char)30, (char)39, (char)255, (char)190, (char)163, (char)103, (char)110, (char)189, (char)54, (char)249, (char)25, (char)238, (char)112, (char)206, (char)16, (char)37, (char)62, (char)223, (char)100, (char)140, (char)211, (char)3, (char)155, (char)142, (char)215, (char)6, (char)31, (char)160, (char)198, (char)184, (char)225, (char)135, (char)226, (char)174, (char)236, (char)92, (char)3, (char)38, (char)151, (char)100, (char)122, (char)249, (char)21, (char)109, (char)147, (char)11, (char)28, (char)66, (char)41, (char)127, (char)5, (char)23, (char)232, (char)178, (char)80, (char)148, (char)36, (char)44, (char)213, (char)27, (char)117, (char)115, (char)172, (char)117, (char)177, (char)7, (char)134, (char)234, (char)28, (char)138, (char)62, (char)182, (char)116, (char)214, (char)89, (char)136, (char)160, (char)101, (char)80, (char)104, (char)65, (char)133, (char)205, (char)31, (char)46, (char)68, (char)32, (char)63, (char)255, (char)158, (char)166, (char)151, (char)69}));
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_system_SET((char)137) ;
        p248.message_type_SET((char)10910) ;
        p248.payload_SET(new char[] {(char)245, (char)189, (char)246, (char)254, (char)166, (char)147, (char)54, (char)200, (char)154, (char)56, (char)203, (char)87, (char)157, (char)41, (char)191, (char)158, (char)181, (char)103, (char)215, (char)177, (char)206, (char)235, (char)17, (char)230, (char)86, (char)233, (char)184, (char)236, (char)217, (char)50, (char)4, (char)186, (char)96, (char)119, (char)60, (char)61, (char)86, (char)162, (char)175, (char)17, (char)164, (char)95, (char)91, (char)87, (char)211, (char)90, (char)194, (char)138, (char)192, (char)234, (char)7, (char)155, (char)220, (char)80, (char)194, (char)124, (char)239, (char)235, (char)125, (char)152, (char)242, (char)60, (char)227, (char)192, (char)86, (char)83, (char)81, (char)44, (char)189, (char)195, (char)197, (char)33, (char)6, (char)193, (char)234, (char)140, (char)156, (char)44, (char)226, (char)240, (char)113, (char)45, (char)223, (char)5, (char)168, (char)176, (char)102, (char)226, (char)28, (char)235, (char)28, (char)101, (char)184, (char)144, (char)61, (char)125, (char)95, (char)66, (char)144, (char)242, (char)25, (char)117, (char)47, (char)117, (char)82, (char)55, (char)225, (char)158, (char)102, (char)102, (char)162, (char)116, (char)213, (char)38, (char)168, (char)39, (char)20, (char)144, (char)240, (char)130, (char)199, (char)121, (char)224, (char)53, (char)210, (char)157, (char)68, (char)110, (char)50, (char)239, (char)150, (char)226, (char)72, (char)225, (char)110, (char)238, (char)57, (char)199, (char)26, (char)16, (char)24, (char)82, (char)191, (char)145, (char)247, (char)102, (char)95, (char)231, (char)4, (char)30, (char)252, (char)188, (char)81, (char)65, (char)203, (char)15, (char)30, (char)39, (char)255, (char)190, (char)163, (char)103, (char)110, (char)189, (char)54, (char)249, (char)25, (char)238, (char)112, (char)206, (char)16, (char)37, (char)62, (char)223, (char)100, (char)140, (char)211, (char)3, (char)155, (char)142, (char)215, (char)6, (char)31, (char)160, (char)198, (char)184, (char)225, (char)135, (char)226, (char)174, (char)236, (char)92, (char)3, (char)38, (char)151, (char)100, (char)122, (char)249, (char)21, (char)109, (char)147, (char)11, (char)28, (char)66, (char)41, (char)127, (char)5, (char)23, (char)232, (char)178, (char)80, (char)148, (char)36, (char)44, (char)213, (char)27, (char)117, (char)115, (char)172, (char)117, (char)177, (char)7, (char)134, (char)234, (char)28, (char)138, (char)62, (char)182, (char)116, (char)214, (char)89, (char)136, (char)160, (char)101, (char)80, (char)104, (char)65, (char)133, (char)205, (char)31, (char)46, (char)68, (char)32, (char)63, (char)255, (char)158, (char)166, (char)151, (char)69}, 0) ;
        p248.target_component_SET((char)38) ;
        p248.target_network_SET((char)122) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)93, (byte) - 27, (byte)117, (byte) - 124, (byte)100, (byte)8, (byte) - 90, (byte) - 116, (byte) - 34, (byte)82, (byte) - 73, (byte)107, (byte)126, (byte) - 124, (byte) - 17, (byte) - 42, (byte)61, (byte) - 17, (byte) - 11, (byte) - 18, (byte) - 124, (byte)45, (byte)72, (byte)113, (byte)118, (byte)107, (byte)30, (byte)20, (byte)101, (byte)126, (byte) - 117, (byte)87}));
            assert(pack.address_GET() == (char)44132);
            assert(pack.type_GET() == (char)255);
            assert(pack.ver_GET() == (char)21);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.ver_SET((char)21) ;
        p249.type_SET((char)255) ;
        p249.value_SET(new byte[] {(byte)93, (byte) - 27, (byte)117, (byte) - 124, (byte)100, (byte)8, (byte) - 90, (byte) - 116, (byte) - 34, (byte)82, (byte) - 73, (byte)107, (byte)126, (byte) - 124, (byte) - 17, (byte) - 42, (byte)61, (byte) - 17, (byte) - 11, (byte) - 18, (byte) - 124, (byte)45, (byte)72, (byte)113, (byte)118, (byte)107, (byte)30, (byte)20, (byte)101, (byte)126, (byte) - 117, (byte)87}, 0) ;
        p249.address_SET((char)44132) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 1.2097998E38F);
            assert(pack.name_LEN(ph) == 4);
            assert(pack.name_TRY(ph).equals("erej"));
            assert(pack.y_GET() == -2.8272586E37F);
            assert(pack.x_GET() == -1.1517376E38F);
            assert(pack.time_usec_GET() == 9094876492507461717L);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.x_SET(-1.1517376E38F) ;
        p250.time_usec_SET(9094876492507461717L) ;
        p250.name_SET("erej", PH) ;
        p250.z_SET(1.2097998E38F) ;
        p250.y_SET(-2.8272586E37F) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 583319961L);
            assert(pack.value_GET() == -1.515353E38F);
            assert(pack.name_LEN(ph) == 1);
            assert(pack.name_TRY(ph).equals("t"));
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.name_SET("t", PH) ;
        p251.time_boot_ms_SET(583319961L) ;
        p251.value_SET(-1.515353E38F) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 8);
            assert(pack.name_TRY(ph).equals("qmlLmwow"));
            assert(pack.time_boot_ms_GET() == 3979713924L);
            assert(pack.value_GET() == 2017367225);
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.name_SET("qmlLmwow", PH) ;
        p252.value_SET(2017367225) ;
        p252.time_boot_ms_SET(3979713924L) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 6);
            assert(pack.text_TRY(ph).equals("oYiyrE"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_WARNING);
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_WARNING) ;
        p253.text_SET("oYiyrE", PH) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.ind_GET() == (char)204);
            assert(pack.value_GET() == 1.2838969E38F);
            assert(pack.time_boot_ms_GET() == 2160540065L);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(2160540065L) ;
        p254.ind_SET((char)204) ;
        p254.value_SET(1.2838969E38F) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)237);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)106, (char)117, (char)224, (char)42, (char)140, (char)213, (char)174, (char)100, (char)31, (char)16, (char)216, (char)156, (char)33, (char)220, (char)116, (char)116, (char)97, (char)157, (char)225, (char)137, (char)139, (char)221, (char)119, (char)154, (char)211, (char)234, (char)125, (char)232, (char)100, (char)219, (char)110, (char)249}));
            assert(pack.target_component_GET() == (char)194);
            assert(pack.initial_timestamp_GET() == 5830245928972756585L);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.secret_key_SET(new char[] {(char)106, (char)117, (char)224, (char)42, (char)140, (char)213, (char)174, (char)100, (char)31, (char)16, (char)216, (char)156, (char)33, (char)220, (char)116, (char)116, (char)97, (char)157, (char)225, (char)137, (char)139, (char)221, (char)119, (char)154, (char)211, (char)234, (char)125, (char)232, (char)100, (char)219, (char)110, (char)249}, 0) ;
        p256.target_system_SET((char)237) ;
        p256.target_component_SET((char)194) ;
        p256.initial_timestamp_SET(5830245928972756585L) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.last_change_ms_GET() == 3241093133L);
            assert(pack.time_boot_ms_GET() == 1897292312L);
            assert(pack.state_GET() == (char)112);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.state_SET((char)112) ;
        p257.time_boot_ms_SET(1897292312L) ;
        p257.last_change_ms_SET(3241093133L) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)57);
            assert(pack.target_component_GET() == (char)147);
            assert(pack.tune_LEN(ph) == 8);
            assert(pack.tune_TRY(ph).equals("DofUkjok"));
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)57) ;
        p258.tune_SET("DofUkjok", PH) ;
        p258.target_component_SET((char)147) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.sensor_size_h_GET() == 4.8500307E37F);
            assert(pack.firmware_version_GET() == 3637964881L);
            assert(pack.focal_length_GET() == 1.1026519E38F);
            assert(pack.resolution_h_GET() == (char)10485);
            assert(pack.sensor_size_v_GET() == -2.2288606E38F);
            assert(pack.lens_id_GET() == (char)228);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)30, (char)57, (char)152, (char)112, (char)159, (char)17, (char)208, (char)246, (char)117, (char)117, (char)101, (char)209, (char)100, (char)143, (char)60, (char)87, (char)119, (char)229, (char)129, (char)81, (char)48, (char)1, (char)141, (char)178, (char)140, (char)84, (char)29, (char)9, (char)219, (char)52, (char)77, (char)216}));
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)145, (char)199, (char)146, (char)3, (char)113, (char)90, (char)145, (char)144, (char)173, (char)97, (char)17, (char)87, (char)2, (char)121, (char)0, (char)55, (char)184, (char)46, (char)62, (char)163, (char)145, (char)5, (char)193, (char)87, (char)85, (char)75, (char)175, (char)52, (char)124, (char)127, (char)147, (char)255}));
            assert(pack.cam_definition_version_GET() == (char)21523);
            assert(pack.flags_GET() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE);
            assert(pack.resolution_v_GET() == (char)35198);
            assert(pack.cam_definition_uri_LEN(ph) == 106);
            assert(pack.cam_definition_uri_TRY(ph).equals("faarawbzfeeckbmmshcovnEbegOquzvcoqtixnifajjrmsiokbjbudlplisduszrgElzxjzkbubmeopTnqilpkvxdbsckrsbdibstwgPuv"));
            assert(pack.time_boot_ms_GET() == 2956169437L);
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.focal_length_SET(1.1026519E38F) ;
        p259.cam_definition_version_SET((char)21523) ;
        p259.vendor_name_SET(new char[] {(char)145, (char)199, (char)146, (char)3, (char)113, (char)90, (char)145, (char)144, (char)173, (char)97, (char)17, (char)87, (char)2, (char)121, (char)0, (char)55, (char)184, (char)46, (char)62, (char)163, (char)145, (char)5, (char)193, (char)87, (char)85, (char)75, (char)175, (char)52, (char)124, (char)127, (char)147, (char)255}, 0) ;
        p259.firmware_version_SET(3637964881L) ;
        p259.model_name_SET(new char[] {(char)30, (char)57, (char)152, (char)112, (char)159, (char)17, (char)208, (char)246, (char)117, (char)117, (char)101, (char)209, (char)100, (char)143, (char)60, (char)87, (char)119, (char)229, (char)129, (char)81, (char)48, (char)1, (char)141, (char)178, (char)140, (char)84, (char)29, (char)9, (char)219, (char)52, (char)77, (char)216}, 0) ;
        p259.lens_id_SET((char)228) ;
        p259.time_boot_ms_SET(2956169437L) ;
        p259.resolution_h_SET((char)10485) ;
        p259.resolution_v_SET((char)35198) ;
        p259.cam_definition_uri_SET("faarawbzfeeckbmmshcovnEbegOquzvcoqtixnifajjrmsiokbjbudlplisduszrgElzxjzkbubmeopTnqilpkvxdbsckrsbdibstwgPuv", PH) ;
        p259.sensor_size_h_SET(4.8500307E37F) ;
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE) ;
        p259.sensor_size_v_SET(-2.2288606E38F) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE);
            assert(pack.time_boot_ms_GET() == 2256257703L);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE) ;
        p260.time_boot_ms_SET(2256257703L) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.read_speed_GET() == -2.9059691E38F);
            assert(pack.storage_id_GET() == (char)143);
            assert(pack.total_capacity_GET() == 8.772295E37F);
            assert(pack.storage_count_GET() == (char)227);
            assert(pack.time_boot_ms_GET() == 2536338832L);
            assert(pack.available_capacity_GET() == 1.5356246E38F);
            assert(pack.status_GET() == (char)80);
            assert(pack.write_speed_GET() == 2.9810658E38F);
            assert(pack.used_capacity_GET() == -2.9600126E38F);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.storage_id_SET((char)143) ;
        p261.storage_count_SET((char)227) ;
        p261.write_speed_SET(2.9810658E38F) ;
        p261.time_boot_ms_SET(2536338832L) ;
        p261.read_speed_SET(-2.9059691E38F) ;
        p261.status_SET((char)80) ;
        p261.total_capacity_SET(8.772295E37F) ;
        p261.available_capacity_SET(1.5356246E38F) ;
        p261.used_capacity_SET(-2.9600126E38F) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.available_capacity_GET() == -3.0631684E38F);
            assert(pack.image_interval_GET() == 1.0918512E38F);
            assert(pack.recording_time_ms_GET() == 1140798098L);
            assert(pack.time_boot_ms_GET() == 1705445271L);
            assert(pack.image_status_GET() == (char)44);
            assert(pack.video_status_GET() == (char)111);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.video_status_SET((char)111) ;
        p262.time_boot_ms_SET(1705445271L) ;
        p262.recording_time_ms_SET(1140798098L) ;
        p262.image_interval_SET(1.0918512E38F) ;
        p262.available_capacity_SET(-3.0631684E38F) ;
        p262.image_status_SET((char)44) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 132544720);
            assert(pack.lat_GET() == 638492906);
            assert(pack.file_url_LEN(ph) == 23);
            assert(pack.file_url_TRY(ph).equals("qyobcdfghhsByedduTmqaYx"));
            assert(pack.relative_alt_GET() == -115846407);
            assert(pack.capture_result_GET() == (byte) - 120);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-4.0224387E37F, -7.257009E37F, 1.5381157E38F, 2.1212172E37F}));
            assert(pack.image_index_GET() == -815277473);
            assert(pack.time_boot_ms_GET() == 604332283L);
            assert(pack.time_utc_GET() == 8881584898991900407L);
            assert(pack.alt_GET() == -1072699403);
            assert(pack.camera_id_GET() == (char)108);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.image_index_SET(-815277473) ;
        p263.file_url_SET("qyobcdfghhsByedduTmqaYx", PH) ;
        p263.alt_SET(-1072699403) ;
        p263.q_SET(new float[] {-4.0224387E37F, -7.257009E37F, 1.5381157E38F, 2.1212172E37F}, 0) ;
        p263.relative_alt_SET(-115846407) ;
        p263.lat_SET(638492906) ;
        p263.capture_result_SET((byte) - 120) ;
        p263.time_boot_ms_SET(604332283L) ;
        p263.camera_id_SET((char)108) ;
        p263.time_utc_SET(8881584898991900407L) ;
        p263.lon_SET(132544720) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.flight_uuid_GET() == 8727347709771857519L);
            assert(pack.time_boot_ms_GET() == 2073943335L);
            assert(pack.arming_time_utc_GET() == 1303413093255647573L);
            assert(pack.takeoff_time_utc_GET() == 482990771859291647L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.takeoff_time_utc_SET(482990771859291647L) ;
        p264.flight_uuid_SET(8727347709771857519L) ;
        p264.time_boot_ms_SET(2073943335L) ;
        p264.arming_time_utc_SET(1303413093255647573L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -3.315448E38F);
            assert(pack.time_boot_ms_GET() == 1466154795L);
            assert(pack.pitch_GET() == 4.0914534E37F);
            assert(pack.yaw_GET() == -1.5186326E38F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.roll_SET(-3.315448E38F) ;
        p265.pitch_SET(4.0914534E37F) ;
        p265.time_boot_ms_SET(1466154795L) ;
        p265.yaw_SET(-1.5186326E38F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.first_message_offset_GET() == (char)46);
            assert(pack.sequence_GET() == (char)8074);
            assert(pack.target_system_GET() == (char)222);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)22, (char)131, (char)8, (char)64, (char)251, (char)113, (char)220, (char)232, (char)61, (char)43, (char)80, (char)154, (char)69, (char)99, (char)78, (char)22, (char)49, (char)223, (char)185, (char)158, (char)205, (char)191, (char)52, (char)240, (char)34, (char)232, (char)103, (char)161, (char)100, (char)60, (char)88, (char)5, (char)207, (char)215, (char)213, (char)238, (char)166, (char)19, (char)33, (char)78, (char)73, (char)236, (char)123, (char)113, (char)214, (char)157, (char)212, (char)41, (char)123, (char)35, (char)9, (char)141, (char)142, (char)233, (char)155, (char)8, (char)234, (char)181, (char)75, (char)9, (char)76, (char)231, (char)141, (char)72, (char)251, (char)26, (char)71, (char)148, (char)111, (char)70, (char)0, (char)231, (char)108, (char)200, (char)239, (char)102, (char)72, (char)186, (char)51, (char)76, (char)179, (char)37, (char)207, (char)190, (char)57, (char)111, (char)5, (char)49, (char)250, (char)21, (char)145, (char)10, (char)208, (char)110, (char)107, (char)186, (char)115, (char)155, (char)126, (char)67, (char)153, (char)239, (char)191, (char)168, (char)65, (char)181, (char)11, (char)86, (char)23, (char)0, (char)229, (char)134, (char)234, (char)157, (char)71, (char)255, (char)115, (char)147, (char)59, (char)103, (char)126, (char)87, (char)12, (char)180, (char)147, (char)185, (char)202, (char)51, (char)144, (char)71, (char)125, (char)61, (char)123, (char)45, (char)0, (char)75, (char)87, (char)26, (char)61, (char)184, (char)218, (char)3, (char)77, (char)222, (char)129, (char)217, (char)91, (char)244, (char)210, (char)31, (char)166, (char)200, (char)152, (char)245, (char)30, (char)205, (char)140, (char)155, (char)111, (char)16, (char)157, (char)140, (char)112, (char)25, (char)239, (char)92, (char)205, (char)115, (char)7, (char)165, (char)235, (char)34, (char)89, (char)171, (char)218, (char)177, (char)139, (char)245, (char)176, (char)25, (char)237, (char)130, (char)148, (char)56, (char)135, (char)136, (char)236, (char)160, (char)182, (char)22, (char)94, (char)8, (char)21, (char)84, (char)75, (char)169, (char)187, (char)108, (char)227, (char)130, (char)12, (char)225, (char)37, (char)4, (char)156, (char)126, (char)214, (char)72, (char)184, (char)17, (char)24, (char)26, (char)226, (char)166, (char)65, (char)85, (char)99, (char)124, (char)163, (char)84, (char)40, (char)188, (char)71, (char)83, (char)187, (char)119, (char)183, (char)234, (char)54, (char)62, (char)75, (char)185, (char)198, (char)196, (char)122, (char)124, (char)232, (char)216, (char)107, (char)20, (char)227, (char)135, (char)42, (char)87, (char)247, (char)240, (char)188, (char)38, (char)209}));
            assert(pack.target_component_GET() == (char)127);
            assert(pack.length_GET() == (char)24);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.sequence_SET((char)8074) ;
        p266.target_system_SET((char)222) ;
        p266.data__SET(new char[] {(char)22, (char)131, (char)8, (char)64, (char)251, (char)113, (char)220, (char)232, (char)61, (char)43, (char)80, (char)154, (char)69, (char)99, (char)78, (char)22, (char)49, (char)223, (char)185, (char)158, (char)205, (char)191, (char)52, (char)240, (char)34, (char)232, (char)103, (char)161, (char)100, (char)60, (char)88, (char)5, (char)207, (char)215, (char)213, (char)238, (char)166, (char)19, (char)33, (char)78, (char)73, (char)236, (char)123, (char)113, (char)214, (char)157, (char)212, (char)41, (char)123, (char)35, (char)9, (char)141, (char)142, (char)233, (char)155, (char)8, (char)234, (char)181, (char)75, (char)9, (char)76, (char)231, (char)141, (char)72, (char)251, (char)26, (char)71, (char)148, (char)111, (char)70, (char)0, (char)231, (char)108, (char)200, (char)239, (char)102, (char)72, (char)186, (char)51, (char)76, (char)179, (char)37, (char)207, (char)190, (char)57, (char)111, (char)5, (char)49, (char)250, (char)21, (char)145, (char)10, (char)208, (char)110, (char)107, (char)186, (char)115, (char)155, (char)126, (char)67, (char)153, (char)239, (char)191, (char)168, (char)65, (char)181, (char)11, (char)86, (char)23, (char)0, (char)229, (char)134, (char)234, (char)157, (char)71, (char)255, (char)115, (char)147, (char)59, (char)103, (char)126, (char)87, (char)12, (char)180, (char)147, (char)185, (char)202, (char)51, (char)144, (char)71, (char)125, (char)61, (char)123, (char)45, (char)0, (char)75, (char)87, (char)26, (char)61, (char)184, (char)218, (char)3, (char)77, (char)222, (char)129, (char)217, (char)91, (char)244, (char)210, (char)31, (char)166, (char)200, (char)152, (char)245, (char)30, (char)205, (char)140, (char)155, (char)111, (char)16, (char)157, (char)140, (char)112, (char)25, (char)239, (char)92, (char)205, (char)115, (char)7, (char)165, (char)235, (char)34, (char)89, (char)171, (char)218, (char)177, (char)139, (char)245, (char)176, (char)25, (char)237, (char)130, (char)148, (char)56, (char)135, (char)136, (char)236, (char)160, (char)182, (char)22, (char)94, (char)8, (char)21, (char)84, (char)75, (char)169, (char)187, (char)108, (char)227, (char)130, (char)12, (char)225, (char)37, (char)4, (char)156, (char)126, (char)214, (char)72, (char)184, (char)17, (char)24, (char)26, (char)226, (char)166, (char)65, (char)85, (char)99, (char)124, (char)163, (char)84, (char)40, (char)188, (char)71, (char)83, (char)187, (char)119, (char)183, (char)234, (char)54, (char)62, (char)75, (char)185, (char)198, (char)196, (char)122, (char)124, (char)232, (char)216, (char)107, (char)20, (char)227, (char)135, (char)42, (char)87, (char)247, (char)240, (char)188, (char)38, (char)209}, 0) ;
        p266.first_message_offset_SET((char)46) ;
        p266.length_SET((char)24) ;
        p266.target_component_SET((char)127) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)17, (char)230, (char)55, (char)150, (char)95, (char)162, (char)171, (char)34, (char)107, (char)16, (char)123, (char)30, (char)104, (char)2, (char)183, (char)216, (char)103, (char)235, (char)12, (char)72, (char)28, (char)128, (char)250, (char)55, (char)113, (char)207, (char)197, (char)192, (char)193, (char)1, (char)11, (char)30, (char)10, (char)217, (char)102, (char)252, (char)89, (char)168, (char)148, (char)55, (char)25, (char)79, (char)233, (char)252, (char)237, (char)196, (char)117, (char)178, (char)140, (char)129, (char)180, (char)149, (char)137, (char)78, (char)135, (char)88, (char)111, (char)109, (char)215, (char)122, (char)6, (char)98, (char)188, (char)158, (char)187, (char)211, (char)20, (char)205, (char)242, (char)37, (char)218, (char)60, (char)188, (char)34, (char)21, (char)106, (char)245, (char)236, (char)119, (char)81, (char)85, (char)149, (char)41, (char)173, (char)151, (char)232, (char)143, (char)142, (char)103, (char)44, (char)81, (char)238, (char)23, (char)87, (char)97, (char)35, (char)196, (char)26, (char)107, (char)206, (char)36, (char)197, (char)37, (char)84, (char)32, (char)136, (char)42, (char)188, (char)57, (char)204, (char)66, (char)179, (char)165, (char)106, (char)7, (char)5, (char)166, (char)164, (char)144, (char)100, (char)56, (char)198, (char)41, (char)85, (char)183, (char)38, (char)15, (char)148, (char)118, (char)4, (char)47, (char)95, (char)200, (char)56, (char)213, (char)70, (char)201, (char)14, (char)130, (char)93, (char)196, (char)177, (char)216, (char)165, (char)230, (char)199, (char)145, (char)19, (char)171, (char)119, (char)210, (char)241, (char)49, (char)128, (char)138, (char)195, (char)235, (char)194, (char)129, (char)15, (char)52, (char)173, (char)50, (char)52, (char)169, (char)203, (char)12, (char)28, (char)133, (char)236, (char)198, (char)70, (char)45, (char)186, (char)16, (char)108, (char)34, (char)44, (char)33, (char)135, (char)37, (char)46, (char)21, (char)118, (char)85, (char)12, (char)170, (char)216, (char)98, (char)78, (char)108, (char)211, (char)243, (char)144, (char)53, (char)243, (char)102, (char)99, (char)186, (char)148, (char)249, (char)68, (char)169, (char)189, (char)164, (char)159, (char)5, (char)28, (char)9, (char)78, (char)213, (char)17, (char)189, (char)92, (char)168, (char)3, (char)48, (char)141, (char)54, (char)253, (char)248, (char)99, (char)78, (char)131, (char)179, (char)16, (char)169, (char)65, (char)178, (char)87, (char)155, (char)67, (char)3, (char)69, (char)1, (char)10, (char)22, (char)55, (char)161, (char)80, (char)157, (char)141, (char)160, (char)148, (char)225, (char)27, (char)16, (char)47, (char)34}));
            assert(pack.length_GET() == (char)186);
            assert(pack.sequence_GET() == (char)12468);
            assert(pack.target_component_GET() == (char)106);
            assert(pack.target_system_GET() == (char)145);
            assert(pack.first_message_offset_GET() == (char)82);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_component_SET((char)106) ;
        p267.first_message_offset_SET((char)82) ;
        p267.target_system_SET((char)145) ;
        p267.sequence_SET((char)12468) ;
        p267.length_SET((char)186) ;
        p267.data__SET(new char[] {(char)17, (char)230, (char)55, (char)150, (char)95, (char)162, (char)171, (char)34, (char)107, (char)16, (char)123, (char)30, (char)104, (char)2, (char)183, (char)216, (char)103, (char)235, (char)12, (char)72, (char)28, (char)128, (char)250, (char)55, (char)113, (char)207, (char)197, (char)192, (char)193, (char)1, (char)11, (char)30, (char)10, (char)217, (char)102, (char)252, (char)89, (char)168, (char)148, (char)55, (char)25, (char)79, (char)233, (char)252, (char)237, (char)196, (char)117, (char)178, (char)140, (char)129, (char)180, (char)149, (char)137, (char)78, (char)135, (char)88, (char)111, (char)109, (char)215, (char)122, (char)6, (char)98, (char)188, (char)158, (char)187, (char)211, (char)20, (char)205, (char)242, (char)37, (char)218, (char)60, (char)188, (char)34, (char)21, (char)106, (char)245, (char)236, (char)119, (char)81, (char)85, (char)149, (char)41, (char)173, (char)151, (char)232, (char)143, (char)142, (char)103, (char)44, (char)81, (char)238, (char)23, (char)87, (char)97, (char)35, (char)196, (char)26, (char)107, (char)206, (char)36, (char)197, (char)37, (char)84, (char)32, (char)136, (char)42, (char)188, (char)57, (char)204, (char)66, (char)179, (char)165, (char)106, (char)7, (char)5, (char)166, (char)164, (char)144, (char)100, (char)56, (char)198, (char)41, (char)85, (char)183, (char)38, (char)15, (char)148, (char)118, (char)4, (char)47, (char)95, (char)200, (char)56, (char)213, (char)70, (char)201, (char)14, (char)130, (char)93, (char)196, (char)177, (char)216, (char)165, (char)230, (char)199, (char)145, (char)19, (char)171, (char)119, (char)210, (char)241, (char)49, (char)128, (char)138, (char)195, (char)235, (char)194, (char)129, (char)15, (char)52, (char)173, (char)50, (char)52, (char)169, (char)203, (char)12, (char)28, (char)133, (char)236, (char)198, (char)70, (char)45, (char)186, (char)16, (char)108, (char)34, (char)44, (char)33, (char)135, (char)37, (char)46, (char)21, (char)118, (char)85, (char)12, (char)170, (char)216, (char)98, (char)78, (char)108, (char)211, (char)243, (char)144, (char)53, (char)243, (char)102, (char)99, (char)186, (char)148, (char)249, (char)68, (char)169, (char)189, (char)164, (char)159, (char)5, (char)28, (char)9, (char)78, (char)213, (char)17, (char)189, (char)92, (char)168, (char)3, (char)48, (char)141, (char)54, (char)253, (char)248, (char)99, (char)78, (char)131, (char)179, (char)16, (char)169, (char)65, (char)178, (char)87, (char)155, (char)67, (char)3, (char)69, (char)1, (char)10, (char)22, (char)55, (char)161, (char)80, (char)157, (char)141, (char)160, (char)148, (char)225, (char)27, (char)16, (char)47, (char)34}, 0) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)193);
            assert(pack.sequence_GET() == (char)5327);
            assert(pack.target_system_GET() == (char)103);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)103) ;
        p268.target_component_SET((char)193) ;
        p268.sequence_SET((char)5327) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.resolution_v_GET() == (char)34101);
            assert(pack.framerate_GET() == 3.0002746E38F);
            assert(pack.bitrate_GET() == 1718530406L);
            assert(pack.camera_id_GET() == (char)164);
            assert(pack.resolution_h_GET() == (char)20402);
            assert(pack.rotation_GET() == (char)38166);
            assert(pack.uri_LEN(ph) == 200);
            assert(pack.uri_TRY(ph).equals("lqYsbvpfwnpxaipjyuPuziNfmpslfYkisMjqjMdZvxuVptloijPcnhysbLjarjdxmcegtjvtzlnwiEbyuXtdjcdYztmFusgeTujyctrWpxabezcadDtgViEdepdfkLsktpmifvkqzjebUifhkUiwijoilxntWxohKRifvcVvzwtxibzJbcrBQvjzwniytjbsiwkGlIDq"));
            assert(pack.status_GET() == (char)28);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.status_SET((char)28) ;
        p269.framerate_SET(3.0002746E38F) ;
        p269.bitrate_SET(1718530406L) ;
        p269.resolution_v_SET((char)34101) ;
        p269.uri_SET("lqYsbvpfwnpxaipjyuPuziNfmpslfYkisMjqjMdZvxuVptloijPcnhysbLjarjdxmcegtjvtzlnwiEbyuXtdjcdYztmFusgeTujyctrWpxabezcadDtgViEdepdfkLsktpmifvkqzjebUifhkUiwijoilxntWxohKRifvcVvzwtxibzJbcrBQvjzwniytjbsiwkGlIDq", PH) ;
        p269.resolution_h_SET((char)20402) ;
        p269.camera_id_SET((char)164) ;
        p269.rotation_SET((char)38166) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.resolution_v_GET() == (char)34827);
            assert(pack.rotation_GET() == (char)8772);
            assert(pack.target_system_GET() == (char)57);
            assert(pack.resolution_h_GET() == (char)555);
            assert(pack.camera_id_GET() == (char)13);
            assert(pack.uri_LEN(ph) == 219);
            assert(pack.uri_TRY(ph).equals("qdnhhbonnindkbkazkhkoKoenmmTgabalaojzmtxycwzvvpuorxcbotzuvhzlrqkaygdOsywmiycfBxLzovyysgbqkenqBfDzlsceleirqednyurhYgrgmxQtzovevhrftmnnpkfpzqdwqstXlltrkgnZlzpkgesnztmsxkUjrkSifluujbavcklvzdgrizfqgfviviqhwZpnqEcxsscglnuhmy"));
            assert(pack.bitrate_GET() == 2532352247L);
            assert(pack.target_component_GET() == (char)160);
            assert(pack.framerate_GET() == 1.7086125E38F);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.framerate_SET(1.7086125E38F) ;
        p270.camera_id_SET((char)13) ;
        p270.rotation_SET((char)8772) ;
        p270.target_component_SET((char)160) ;
        p270.resolution_v_SET((char)34827) ;
        p270.bitrate_SET(2532352247L) ;
        p270.resolution_h_SET((char)555) ;
        p270.uri_SET("qdnhhbonnindkbkazkhkoKoenmmTgabalaojzmtxycwzvvpuorxcbotzuvhzlrqkaygdOsywmiycfBxLzovyysgbqkenqBfDzlsceleirqednyurhYgrgmxQtzovevhrftmnnpkfpzqdwqstXlltrkgnZlzpkgesnztmsxkUjrkSifluujbavcklvzdgrizfqgfviviqhwZpnqEcxsscglnuhmy", PH) ;
        p270.target_system_SET((char)57) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 20);
            assert(pack.ssid_TRY(ph).equals("lgmcglppneghgxAahdad"));
            assert(pack.password_LEN(ph) == 12);
            assert(pack.password_TRY(ph).equals("ZajqxGvsmEhq"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("lgmcglppneghgxAahdad", PH) ;
        p299.password_SET("ZajqxGvsmEhq", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)3, (char)24, (char)176, (char)207, (char)39, (char)28, (char)244, (char)15}));
            assert(pack.min_version_GET() == (char)63234);
            assert(pack.version_GET() == (char)57386);
            assert(pack.max_version_GET() == (char)45594);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)132, (char)178, (char)11, (char)234, (char)27, (char)247, (char)33, (char)223}));
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.spec_version_hash_SET(new char[] {(char)132, (char)178, (char)11, (char)234, (char)27, (char)247, (char)33, (char)223}, 0) ;
        p300.version_SET((char)57386) ;
        p300.min_version_SET((char)63234) ;
        p300.max_version_SET((char)45594) ;
        p300.library_version_hash_SET(new char[] {(char)3, (char)24, (char)176, (char)207, (char)39, (char)28, (char)244, (char)15}, 0) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.sub_mode_GET() == (char)197);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
            assert(pack.uptime_sec_GET() == 2950340332L);
            assert(pack.time_usec_GET() == 3587384081191671427L);
            assert(pack.vendor_specific_status_code_GET() == (char)39274);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.vendor_specific_status_code_SET((char)39274) ;
        p310.uptime_sec_SET(2950340332L) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE) ;
        p310.sub_mode_SET((char)197) ;
        p310.time_usec_SET(3587384081191671427L) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 29);
            assert(pack.name_TRY(ph).equals("isVxIsavfhynmuqfsfKToxvrfiHiy"));
            assert(pack.uptime_sec_GET() == 4073586154L);
            assert(pack.hw_version_major_GET() == (char)126);
            assert(pack.time_usec_GET() == 2952867663499078286L);
            assert(pack.sw_version_minor_GET() == (char)250);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)79, (char)30, (char)10, (char)208, (char)175, (char)25, (char)183, (char)59, (char)153, (char)107, (char)129, (char)177, (char)26, (char)104, (char)54, (char)55}));
            assert(pack.sw_version_major_GET() == (char)91);
            assert(pack.hw_version_minor_GET() == (char)72);
            assert(pack.sw_vcs_commit_GET() == 1277223164L);
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.time_usec_SET(2952867663499078286L) ;
        p311.hw_version_major_SET((char)126) ;
        p311.sw_version_minor_SET((char)250) ;
        p311.uptime_sec_SET(4073586154L) ;
        p311.sw_version_major_SET((char)91) ;
        p311.hw_version_minor_SET((char)72) ;
        p311.hw_unique_id_SET(new char[] {(char)79, (char)30, (char)10, (char)208, (char)175, (char)25, (char)183, (char)59, (char)153, (char)107, (char)129, (char)177, (char)26, (char)104, (char)54, (char)55}, 0) ;
        p311.name_SET("isVxIsavfhynmuqfsfKToxvrfiHiy", PH) ;
        p311.sw_vcs_commit_SET(1277223164L) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)123);
            assert(pack.target_system_GET() == (char)28);
            assert(pack.param_index_GET() == (short) -20878);
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("dligbcyp"));
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.param_index_SET((short) -20878) ;
        p320.target_component_SET((char)123) ;
        p320.target_system_SET((char)28) ;
        p320.param_id_SET("dligbcyp", PH) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)170);
            assert(pack.target_system_GET() == (char)27);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)27) ;
        p321.target_component_SET((char)170) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 112);
            assert(pack.param_value_TRY(ph).equals("wnjvtkhisxrlfpkesrqmioxdxzfwoohukbKqhqUordfiOoidfbatNnqkguwjXcakignptykplghcfzanzndqyohjqfsdbyyqafZnpfHsMzcahQyq"));
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("Ervao"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
            assert(pack.param_count_GET() == (char)48502);
            assert(pack.param_index_GET() == (char)27061);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("Ervao", PH) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32) ;
        p322.param_index_SET((char)27061) ;
        p322.param_count_SET((char)48502) ;
        p322.param_value_SET("wnjvtkhisxrlfpkesrqmioxdxzfwoohukbKqhqUordfiOoidfbatNnqkguwjXcakignptykplghcfzanzndqyohjqfsdbyyqafZnpfHsMzcahQyq", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)210);
            assert(pack.param_value_LEN(ph) == 78);
            assert(pack.param_value_TRY(ph).equals("solwoluXmxzlzkuvtrQwmqabhdprlgyDbuoonqbgvsphqdjgIzkmkqdqnSzzcgojiyhbfpvybmplyc"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
            assert(pack.target_component_GET() == (char)154);
            assert(pack.param_id_LEN(ph) == 7);
            assert(pack.param_id_TRY(ph).equals("jpofjbn"));
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_id_SET("jpofjbn", PH) ;
        p323.target_system_SET((char)210) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16) ;
        p323.param_value_SET("solwoluXmxzlzkuvtrQwmqabhdprlgyDbuoonqbgvsphqdjgIzkmkqdqnSzzcgojiyhbfpvybmplyc", PH) ;
        p323.target_component_SET((char)154) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 8);
            assert(pack.param_value_TRY(ph).equals("vekerqxL"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("g"));
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED) ;
        p324.param_id_SET("g", PH) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16) ;
        p324.param_value_SET("vekerqxL", PH) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.max_distance_GET() == (char)50521);
            assert(pack.min_distance_GET() == (char)3527);
            assert(pack.time_usec_GET() == 8941755099806008335L);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
            assert(pack.increment_GET() == (char)224);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)36290, (char)24383, (char)46271, (char)49177, (char)10441, (char)33135, (char)37732, (char)64264, (char)9262, (char)30958, (char)55838, (char)53226, (char)17745, (char)14707, (char)625, (char)57573, (char)33665, (char)9808, (char)26776, (char)56286, (char)56407, (char)42762, (char)36326, (char)63507, (char)57117, (char)3542, (char)49541, (char)20996, (char)29379, (char)60994, (char)59535, (char)7784, (char)21349, (char)23888, (char)15774, (char)33792, (char)53798, (char)26087, (char)33987, (char)27158, (char)20657, (char)1242, (char)60528, (char)52634, (char)64507, (char)36228, (char)36375, (char)39825, (char)24988, (char)65162, (char)38224, (char)41311, (char)45553, (char)19048, (char)45695, (char)59654, (char)264, (char)44974, (char)5418, (char)30798, (char)29361, (char)44786, (char)13543, (char)29590, (char)6218, (char)40904, (char)2711, (char)29120, (char)8201, (char)4068, (char)13801, (char)55658}));
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND) ;
        p330.increment_SET((char)224) ;
        p330.distances_SET(new char[] {(char)36290, (char)24383, (char)46271, (char)49177, (char)10441, (char)33135, (char)37732, (char)64264, (char)9262, (char)30958, (char)55838, (char)53226, (char)17745, (char)14707, (char)625, (char)57573, (char)33665, (char)9808, (char)26776, (char)56286, (char)56407, (char)42762, (char)36326, (char)63507, (char)57117, (char)3542, (char)49541, (char)20996, (char)29379, (char)60994, (char)59535, (char)7784, (char)21349, (char)23888, (char)15774, (char)33792, (char)53798, (char)26087, (char)33987, (char)27158, (char)20657, (char)1242, (char)60528, (char)52634, (char)64507, (char)36228, (char)36375, (char)39825, (char)24988, (char)65162, (char)38224, (char)41311, (char)45553, (char)19048, (char)45695, (char)59654, (char)264, (char)44974, (char)5418, (char)30798, (char)29361, (char)44786, (char)13543, (char)29590, (char)6218, (char)40904, (char)2711, (char)29120, (char)8201, (char)4068, (char)13801, (char)55658}, 0) ;
        p330.time_usec_SET(8941755099806008335L) ;
        p330.max_distance_SET((char)50521) ;
        p330.min_distance_SET((char)3527) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
    }

}
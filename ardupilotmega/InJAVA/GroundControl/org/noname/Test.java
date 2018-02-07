
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
        {  set_bits(- 1 +   src, 8, data, 50); }
        public void system_status_SET(@MAV_STATE int  src) //System status flag, see MAV_STATE ENUM
        {  set_bits(- 0 +   src, 4, data, 58); }
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
        {  set_bits(- 1 +   src, 26, data, 152); }
        /**
        *Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of
        *	1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
        public void onboard_control_sensors_enabled_SET(@MAV_SYS_STATUS_SENSOR int  src)
        {  set_bits(- 1 +   src, 26, data, 178); }
        /**
        *Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not
        *	enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
        public void onboard_control_sensors_health_SET(@MAV_SYS_STATUS_SENSOR int  src)
        {  set_bits(- 1 +   src, 26, data, 204); }
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
    public static class VISION_POSITION_ESTIMATE extends GroundControl.VISION_POSITION_ESTIMATE
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
        {  return  0 + (int)get_bits(data, 56, 2); }
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
        {  return  1 + (int)get_bits(data, 147, 3); }
        public @LIMIT_MODULE int mods_required_GET()//AP_Limit_Module bitfield of required modules, (see enum moduleid or LIMIT_MODULE)
        {  return  1 + (int)get_bits(data, 150, 3); }
        public @LIMIT_MODULE int mods_triggered_GET()//AP_Limit_Module bitfield of triggered modules, (see enum moduleid or LIMIT_MODULE)
        {  return  1 + (int)get_bits(data, 153, 3); }
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
        {  return  1 + (int)get_bits(data, 144, 1); }
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
        {  return  2147483645 + (int)get_bits(data, 1616, 1); }
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
        {  return  0 + (int)get_bits(data, 48, 1); }
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
        {  return  1 + (int)get_bits(data, 160, 10); }
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
        {  return  0 + (int)get_bits(data, 0, 2); }
        public @GOPRO_CAPTURE_MODE int capture_mode_GET()//Current capture mode
        {
            switch((int)get_bits(data, 2, 4))
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
        {  return  1 + (int)get_bits(data, 6, 1); }
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
        {  return  0 + (int)get_bits(data, 37, 1); }
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
        {  return  0 + (int)get_bits(data, 5, 1); }
    }
    public static class RPM extends GroundControl.RPM
    {
        public float rpm1_GET()//RPM Sensor1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float rpm2_GET()//RPM Sensor2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
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
        {  return  0 + (int)get_bits(data, 120, 2); }
        public @UAVCAN_NODE_MODE int mode_GET()//Generalized operating mode.
        {
            switch((int)get_bits(data, 122, 3))
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
        {  return  0 + (int)get_bits(data, 4, 2); }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	ID is stored as strin*/
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  8 && !try_visit_field(ph, 8)  ||  !try_visit_item(ph, 0)) return null;
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
            return (ph.field_bit !=  8 && !try_visit_field(ph, 8)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public String param_value_TRY(Bounds.Inside ph)//Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
        {
            if(ph.field_bit !=  9 && !try_visit_field(ph, 9)  ||  !try_visit_item(ph, 0)) return null;
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
            return (ph.field_bit !=  9 && !try_visit_field(ph, 9)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
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
        {  return  0 + (int)get_bits(data, 53, 4); }
        public @UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT int gpsOffsetLat_GET()//GPS antenna lateral offset (table 2-36 of DO-282B)
        {  return  0 + (int)get_bits(data, 57, 3); }
        /**
        *GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and add
        *	one] (table 2-37 DO-282B*/
        public @UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON int gpsOffsetLon_GET()
        {  return  0 + (int)get_bits(data, 60, 1); }
        public @UAVIONIX_ADSB_OUT_RF_SELECT int rfSelect_GET()//ADS-B transponder reciever and transmit enable flags
        {  return  0 + (int)get_bits(data, 61, 2); }
        public String callsign_TRY(Bounds.Inside ph)//Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
        {
            if(ph.field_bit !=  63 && !try_visit_field(ph, 63)  ||  !try_visit_item(ph, 0)) return null;
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
            return (ph.field_bit !=  63 && !try_visit_field(ph, 63)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
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
        {  return  0 + (int)get_bits(data, 299, 3); }
        public @UAVIONIX_ADSB_OUT_DYNAMIC_STATE int state_GET()//ADS-B transponder dynamic input state flags
        {  return  1 + (int)get_bits(data, 302, 5); }
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
        {  return  0 + (int)get_bits(data, 80, 1); }
        public String busname_TRY(Bounds.Inside ph)//Name of device on bus (for SPI)
        {
            if(ph.field_bit !=  81 && !try_visit_field(ph, 81)  ||  !try_visit_item(ph, 0)) return null;
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
            return (ph.field_bit !=  81 && !try_visit_field(ph, 81)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
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
        {  return  0 + (int)get_bits(data, 1104, 1); }
        public String busname_TRY(Bounds.Inside ph)//Name of device on bus (for SPI)
        {
            if(ph.field_bit !=  1105 && !try_visit_field(ph, 1105)  ||  !try_visit_item(ph, 0)) return null;
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
            return (ph.field_bit !=  1105 && !try_visit_field(ph, 1105)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
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
                case 102:
                    if(pack == null) return new VISION_POSITION_ESTIMATE();
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
            assert(pack.custom_mode_GET() == 1059132147L);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_BOOT);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_GROUND_ROVER);
            assert(pack.mavlink_version_GET() == (char)108);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_AEROB);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED));
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.type_SET(MAV_TYPE.MAV_TYPE_GROUND_ROVER) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_AEROB) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_BOOT) ;
        p0.mavlink_version_SET((char)108) ;
        p0.custom_mode_SET(1059132147L) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.battery_remaining_GET() == (byte)39);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
            assert(pack.errors_count2_GET() == (char)26889);
            assert(pack.voltage_battery_GET() == (char)64105);
            assert(pack.errors_count4_GET() == (char)26840);
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL));
            assert(pack.errors_count1_GET() == (char)61103);
            assert(pack.drop_rate_comm_GET() == (char)22614);
            assert(pack.load_GET() == (char)22763);
            assert(pack.errors_count3_GET() == (char)38596);
            assert(pack.errors_comm_GET() == (char)36864);
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
            assert(pack.current_battery_GET() == (short) -19176);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.errors_count4_SET((char)26840) ;
        p1.errors_count3_SET((char)38596) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL)) ;
        p1.load_SET((char)22763) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL)) ;
        p1.errors_count2_SET((char)26889) ;
        p1.voltage_battery_SET((char)64105) ;
        p1.errors_count1_SET((char)61103) ;
        p1.errors_comm_SET((char)36864) ;
        p1.current_battery_SET((short) -19176) ;
        p1.battery_remaining_SET((byte)39) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL)) ;
        p1.drop_rate_comm_SET((char)22614) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2485782651L);
            assert(pack.time_unix_usec_GET() == 4809077243855948400L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(2485782651L) ;
        p2.time_unix_usec_SET(4809077243855948400L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == -2.1689157E38F);
            assert(pack.yaw_rate_GET() == 1.4131547E37F);
            assert(pack.x_GET() == -3.5374132E37F);
            assert(pack.afx_GET() == 1.2488905E38F);
            assert(pack.vy_GET() == -1.682E38F);
            assert(pack.yaw_GET() == 1.4773964E38F);
            assert(pack.vz_GET() == 1.5650628E38F);
            assert(pack.afy_GET() == -1.3835349E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.y_GET() == -1.6525934E38F);
            assert(pack.afz_GET() == -1.8142524E38F);
            assert(pack.z_GET() == 2.727646E38F);
            assert(pack.type_mask_GET() == (char)7566);
            assert(pack.time_boot_ms_GET() == 243505316L);
        });
        POSITION_TARGET_LOCAL_NED p3 = new POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.afx_SET(1.2488905E38F) ;
        p3.type_mask_SET((char)7566) ;
        p3.vy_SET(-1.682E38F) ;
        p3.y_SET(-1.6525934E38F) ;
        p3.x_SET(-3.5374132E37F) ;
        p3.afy_SET(-1.3835349E38F) ;
        p3.vx_SET(-2.1689157E38F) ;
        p3.z_SET(2.727646E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p3.afz_SET(-1.8142524E38F) ;
        p3.yaw_rate_SET(1.4131547E37F) ;
        p3.time_boot_ms_SET(243505316L) ;
        p3.yaw_SET(1.4773964E38F) ;
        p3.vz_SET(1.5650628E38F) ;
        TestChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5881775856049706287L);
            assert(pack.target_component_GET() == (char)44);
            assert(pack.seq_GET() == 3171167437L);
            assert(pack.target_system_GET() == (char)134);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.time_usec_SET(5881775856049706287L) ;
        p4.seq_SET(3171167437L) ;
        p4.target_component_SET((char)44) ;
        p4.target_system_SET((char)134) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.control_request_GET() == (char)255);
            assert(pack.target_system_GET() == (char)208);
            assert(pack.version_GET() == (char)29);
            assert(pack.passkey_LEN(ph) == 25);
            assert(pack.passkey_TRY(ph).equals("cwutZOfipjbhhyfrkMoyqqujg"));
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.passkey_SET("cwutZOfipjbhhyfrkMoyqqujg", PH) ;
        p5.control_request_SET((char)255) ;
        p5.version_SET((char)29) ;
        p5.target_system_SET((char)208) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.gcs_system_id_GET() == (char)47);
            assert(pack.ack_GET() == (char)38);
            assert(pack.control_request_GET() == (char)220);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.control_request_SET((char)220) ;
        p6.ack_SET((char)38) ;
        p6.gcs_system_id_SET((char)47) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 19);
            assert(pack.key_TRY(ph).equals("nvwxxkxorkiLwsGpxex"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("nvwxxkxorkiLwsGpxex", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)81);
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_TEST_DISARMED);
            assert(pack.custom_mode_GET() == 1606174532L);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.target_system_SET((char)81) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_TEST_DISARMED) ;
        p11.custom_mode_SET(1606174532L) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (short)20265);
            assert(pack.target_component_GET() == (char)13);
            assert(pack.target_system_GET() == (char)165);
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("NaqyXkaofxDd"));
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_component_SET((char)13) ;
        p20.param_id_SET("NaqyXkaofxDd", PH) ;
        p20.target_system_SET((char)165) ;
        p20.param_index_SET((short)20265) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)148);
            assert(pack.target_system_GET() == (char)89);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)89) ;
        p21.target_component_SET((char)148) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)1497);
            assert(pack.param_value_GET() == 7.9270644E37F);
            assert(pack.param_index_GET() == (char)41731);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("kaas"));
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_count_SET((char)1497) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16) ;
        p22.param_value_SET(7.9270644E37F) ;
        p22.param_index_SET((char)41731) ;
        p22.param_id_SET("kaas", PH) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 16);
            assert(pack.param_id_TRY(ph).equals("Xekhavxqrijukpzy"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64);
            assert(pack.target_component_GET() == (char)254);
            assert(pack.target_system_GET() == (char)17);
            assert(pack.param_value_GET() == -2.2362878E38F);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64) ;
        p23.target_system_SET((char)17) ;
        p23.param_id_SET("Xekhavxqrijukpzy", PH) ;
        p23.target_component_SET((char)254) ;
        p23.param_value_SET(-2.2362878E38F) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7776002303356316340L);
            assert(pack.v_acc_TRY(ph) == 3551042950L);
            assert(pack.satellites_visible_GET() == (char)157);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.alt_GET() == 983887243);
            assert(pack.vel_GET() == (char)15665);
            assert(pack.cog_GET() == (char)15399);
            assert(pack.lat_GET() == 1580523958);
            assert(pack.eph_GET() == (char)53729);
            assert(pack.epv_GET() == (char)40582);
            assert(pack.hdg_acc_TRY(ph) == 3791189394L);
            assert(pack.vel_acc_TRY(ph) == 2285668141L);
            assert(pack.alt_ellipsoid_TRY(ph) == -623256039);
            assert(pack.h_acc_TRY(ph) == 724376213L);
            assert(pack.lon_GET() == -134386381);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.hdg_acc_SET(3791189394L, PH) ;
        p24.epv_SET((char)40582) ;
        p24.alt_SET(983887243) ;
        p24.vel_acc_SET(2285668141L, PH) ;
        p24.alt_ellipsoid_SET(-623256039, PH) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p24.time_usec_SET(7776002303356316340L) ;
        p24.vel_SET((char)15665) ;
        p24.h_acc_SET(724376213L, PH) ;
        p24.cog_SET((char)15399) ;
        p24.lon_SET(-134386381) ;
        p24.v_acc_SET(3551042950L, PH) ;
        p24.eph_SET((char)53729) ;
        p24.lat_SET(1580523958) ;
        p24.satellites_visible_SET((char)157) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)164, (char)111, (char)197, (char)209, (char)101, (char)252, (char)143, (char)35, (char)211, (char)128, (char)239, (char)95, (char)161, (char)149, (char)193, (char)91, (char)126, (char)133, (char)156, (char)28}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)96, (char)74, (char)119, (char)205, (char)128, (char)19, (char)226, (char)100, (char)133, (char)234, (char)86, (char)31, (char)128, (char)52, (char)152, (char)239, (char)78, (char)32, (char)65, (char)57}));
            assert(pack.satellites_visible_GET() == (char)219);
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)96, (char)138, (char)48, (char)99, (char)168, (char)155, (char)93, (char)142, (char)35, (char)247, (char)3, (char)142, (char)236, (char)249, (char)231, (char)217, (char)172, (char)90, (char)61, (char)110}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)134, (char)50, (char)168, (char)182, (char)229, (char)192, (char)197, (char)47, (char)166, (char)39, (char)8, (char)96, (char)10, (char)159, (char)32, (char)202, (char)167, (char)170, (char)252, (char)136}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)185, (char)115, (char)117, (char)45, (char)3, (char)125, (char)186, (char)32, (char)129, (char)237, (char)133, (char)136, (char)68, (char)101, (char)231, (char)146, (char)244, (char)95, (char)232, (char)96}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_prn_SET(new char[] {(char)164, (char)111, (char)197, (char)209, (char)101, (char)252, (char)143, (char)35, (char)211, (char)128, (char)239, (char)95, (char)161, (char)149, (char)193, (char)91, (char)126, (char)133, (char)156, (char)28}, 0) ;
        p25.satellites_visible_SET((char)219) ;
        p25.satellite_elevation_SET(new char[] {(char)96, (char)74, (char)119, (char)205, (char)128, (char)19, (char)226, (char)100, (char)133, (char)234, (char)86, (char)31, (char)128, (char)52, (char)152, (char)239, (char)78, (char)32, (char)65, (char)57}, 0) ;
        p25.satellite_used_SET(new char[] {(char)96, (char)138, (char)48, (char)99, (char)168, (char)155, (char)93, (char)142, (char)35, (char)247, (char)3, (char)142, (char)236, (char)249, (char)231, (char)217, (char)172, (char)90, (char)61, (char)110}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)134, (char)50, (char)168, (char)182, (char)229, (char)192, (char)197, (char)47, (char)166, (char)39, (char)8, (char)96, (char)10, (char)159, (char)32, (char)202, (char)167, (char)170, (char)252, (char)136}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)185, (char)115, (char)117, (char)45, (char)3, (char)125, (char)186, (char)32, (char)129, (char)237, (char)133, (char)136, (char)68, (char)101, (char)231, (char)146, (char)244, (char)95, (char)232, (char)96}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == (short) -549);
            assert(pack.xacc_GET() == (short)20895);
            assert(pack.xmag_GET() == (short) -19172);
            assert(pack.yacc_GET() == (short) -7088);
            assert(pack.xgyro_GET() == (short) -22358);
            assert(pack.zgyro_GET() == (short)22197);
            assert(pack.ygyro_GET() == (short) -20768);
            assert(pack.ymag_GET() == (short) -17157);
            assert(pack.zmag_GET() == (short) -20767);
            assert(pack.time_boot_ms_GET() == 2694915013L);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.ymag_SET((short) -17157) ;
        p26.xgyro_SET((short) -22358) ;
        p26.zgyro_SET((short)22197) ;
        p26.zacc_SET((short) -549) ;
        p26.time_boot_ms_SET(2694915013L) ;
        p26.xacc_SET((short)20895) ;
        p26.ygyro_SET((short) -20768) ;
        p26.yacc_SET((short) -7088) ;
        p26.zmag_SET((short) -20767) ;
        p26.xmag_SET((short) -19172) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short) -18087);
            assert(pack.zacc_GET() == (short)30016);
            assert(pack.xmag_GET() == (short) -15188);
            assert(pack.ygyro_GET() == (short)18383);
            assert(pack.zgyro_GET() == (short) -7072);
            assert(pack.xgyro_GET() == (short)24252);
            assert(pack.yacc_GET() == (short) -9931);
            assert(pack.ymag_GET() == (short)3221);
            assert(pack.zmag_GET() == (short)6500);
            assert(pack.time_usec_GET() == 886798794701362873L);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.zacc_SET((short)30016) ;
        p27.yacc_SET((short) -9931) ;
        p27.time_usec_SET(886798794701362873L) ;
        p27.xacc_SET((short) -18087) ;
        p27.xmag_SET((short) -15188) ;
        p27.zgyro_SET((short) -7072) ;
        p27.ygyro_SET((short)18383) ;
        p27.zmag_SET((short)6500) ;
        p27.ymag_SET((short)3221) ;
        p27.xgyro_SET((short)24252) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7778302078041954797L);
            assert(pack.press_diff2_GET() == (short) -29100);
            assert(pack.press_abs_GET() == (short) -25643);
            assert(pack.temperature_GET() == (short) -14065);
            assert(pack.press_diff1_GET() == (short) -8486);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_diff2_SET((short) -29100) ;
        p28.temperature_SET((short) -14065) ;
        p28.time_usec_SET(7778302078041954797L) ;
        p28.press_abs_SET((short) -25643) ;
        p28.press_diff1_SET((short) -8486) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == 2.980248E37F);
            assert(pack.time_boot_ms_GET() == 300384567L);
            assert(pack.press_diff_GET() == 3.079154E38F);
            assert(pack.temperature_GET() == (short) -28127);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_abs_SET(2.980248E37F) ;
        p29.temperature_SET((short) -28127) ;
        p29.press_diff_SET(3.079154E38F) ;
        p29.time_boot_ms_SET(300384567L) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -3.2965684E38F);
            assert(pack.rollspeed_GET() == -4.7586102E35F);
            assert(pack.time_boot_ms_GET() == 1611100278L);
            assert(pack.pitchspeed_GET() == 3.3318884E38F);
            assert(pack.yawspeed_GET() == -1.317563E38F);
            assert(pack.pitch_GET() == -2.7695156E38F);
            assert(pack.roll_GET() == 2.4178928E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.pitch_SET(-2.7695156E38F) ;
        p30.yawspeed_SET(-1.317563E38F) ;
        p30.pitchspeed_SET(3.3318884E38F) ;
        p30.rollspeed_SET(-4.7586102E35F) ;
        p30.roll_SET(2.4178928E38F) ;
        p30.time_boot_ms_SET(1611100278L) ;
        p30.yaw_SET(-3.2965684E38F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q4_GET() == 1.0663559E38F);
            assert(pack.time_boot_ms_GET() == 654656984L);
            assert(pack.q2_GET() == 1.2367921E38F);
            assert(pack.rollspeed_GET() == -3.1979385E38F);
            assert(pack.pitchspeed_GET() == 3.2093487E38F);
            assert(pack.q1_GET() == -1.8146923E38F);
            assert(pack.q3_GET() == 1.3675576E38F);
            assert(pack.yawspeed_GET() == -2.917342E38F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.time_boot_ms_SET(654656984L) ;
        p31.q3_SET(1.3675576E38F) ;
        p31.pitchspeed_SET(3.2093487E38F) ;
        p31.q1_SET(-1.8146923E38F) ;
        p31.q4_SET(1.0663559E38F) ;
        p31.q2_SET(1.2367921E38F) ;
        p31.rollspeed_SET(-3.1979385E38F) ;
        p31.yawspeed_SET(-2.917342E38F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == -3.3486392E38F);
            assert(pack.vy_GET() == 2.4237911E38F);
            assert(pack.y_GET() == -3.868535E36F);
            assert(pack.vx_GET() == 2.7317367E38F);
            assert(pack.z_GET() == -1.6519448E38F);
            assert(pack.x_GET() == -2.244524E38F);
            assert(pack.time_boot_ms_GET() == 3163290685L);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.x_SET(-2.244524E38F) ;
        p32.time_boot_ms_SET(3163290685L) ;
        p32.z_SET(-1.6519448E38F) ;
        p32.y_SET(-3.868535E36F) ;
        p32.vz_SET(-3.3486392E38F) ;
        p32.vx_SET(2.7317367E38F) ;
        p32.vy_SET(2.4237911E38F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == (short)13307);
            assert(pack.vy_GET() == (short)17805);
            assert(pack.time_boot_ms_GET() == 31667481L);
            assert(pack.relative_alt_GET() == -1524589750);
            assert(pack.vx_GET() == (short)2229);
            assert(pack.hdg_GET() == (char)39244);
            assert(pack.alt_GET() == -1038593473);
            assert(pack.lon_GET() == 1814145204);
            assert(pack.lat_GET() == 126926409);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.hdg_SET((char)39244) ;
        p33.vy_SET((short)17805) ;
        p33.alt_SET(-1038593473) ;
        p33.lat_SET(126926409) ;
        p33.vz_SET((short)13307) ;
        p33.time_boot_ms_SET(31667481L) ;
        p33.lon_SET(1814145204) ;
        p33.relative_alt_SET(-1524589750) ;
        p33.vx_SET((short)2229) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan5_scaled_GET() == (short) -19581);
            assert(pack.chan6_scaled_GET() == (short) -32496);
            assert(pack.chan7_scaled_GET() == (short) -8170);
            assert(pack.chan3_scaled_GET() == (short) -16418);
            assert(pack.rssi_GET() == (char)177);
            assert(pack.chan2_scaled_GET() == (short) -19789);
            assert(pack.port_GET() == (char)246);
            assert(pack.chan8_scaled_GET() == (short)17267);
            assert(pack.time_boot_ms_GET() == 3818014488L);
            assert(pack.chan1_scaled_GET() == (short)6164);
            assert(pack.chan4_scaled_GET() == (short) -9960);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.time_boot_ms_SET(3818014488L) ;
        p34.rssi_SET((char)177) ;
        p34.chan1_scaled_SET((short)6164) ;
        p34.chan7_scaled_SET((short) -8170) ;
        p34.chan5_scaled_SET((short) -19581) ;
        p34.chan8_scaled_SET((short)17267) ;
        p34.chan6_scaled_SET((short) -32496) ;
        p34.chan2_scaled_SET((short) -19789) ;
        p34.chan3_scaled_SET((short) -16418) ;
        p34.chan4_scaled_SET((short) -9960) ;
        p34.port_SET((char)246) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan4_raw_GET() == (char)3388);
            assert(pack.chan5_raw_GET() == (char)32680);
            assert(pack.chan3_raw_GET() == (char)24574);
            assert(pack.chan8_raw_GET() == (char)43287);
            assert(pack.chan2_raw_GET() == (char)64644);
            assert(pack.chan6_raw_GET() == (char)41355);
            assert(pack.chan1_raw_GET() == (char)31532);
            assert(pack.chan7_raw_GET() == (char)27781);
            assert(pack.port_GET() == (char)222);
            assert(pack.rssi_GET() == (char)196);
            assert(pack.time_boot_ms_GET() == 4033096025L);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan6_raw_SET((char)41355) ;
        p35.chan7_raw_SET((char)27781) ;
        p35.time_boot_ms_SET(4033096025L) ;
        p35.rssi_SET((char)196) ;
        p35.chan8_raw_SET((char)43287) ;
        p35.chan5_raw_SET((char)32680) ;
        p35.chan4_raw_SET((char)3388) ;
        p35.chan1_raw_SET((char)31532) ;
        p35.chan3_raw_SET((char)24574) ;
        p35.port_SET((char)222) ;
        p35.chan2_raw_SET((char)64644) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo9_raw_TRY(ph) == (char)48566);
            assert(pack.port_GET() == (char)4);
            assert(pack.servo3_raw_GET() == (char)16300);
            assert(pack.servo15_raw_TRY(ph) == (char)53320);
            assert(pack.servo14_raw_TRY(ph) == (char)9468);
            assert(pack.time_usec_GET() == 2152714193L);
            assert(pack.servo5_raw_GET() == (char)21765);
            assert(pack.servo10_raw_TRY(ph) == (char)7465);
            assert(pack.servo12_raw_TRY(ph) == (char)18376);
            assert(pack.servo7_raw_GET() == (char)45660);
            assert(pack.servo6_raw_GET() == (char)42146);
            assert(pack.servo2_raw_GET() == (char)49453);
            assert(pack.servo13_raw_TRY(ph) == (char)40836);
            assert(pack.servo4_raw_GET() == (char)32255);
            assert(pack.servo8_raw_GET() == (char)14521);
            assert(pack.servo1_raw_GET() == (char)52072);
            assert(pack.servo16_raw_TRY(ph) == (char)38241);
            assert(pack.servo11_raw_TRY(ph) == (char)17960);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo7_raw_SET((char)45660) ;
        p36.servo14_raw_SET((char)9468, PH) ;
        p36.servo11_raw_SET((char)17960, PH) ;
        p36.servo10_raw_SET((char)7465, PH) ;
        p36.servo5_raw_SET((char)21765) ;
        p36.servo9_raw_SET((char)48566, PH) ;
        p36.servo13_raw_SET((char)40836, PH) ;
        p36.servo2_raw_SET((char)49453) ;
        p36.port_SET((char)4) ;
        p36.servo16_raw_SET((char)38241, PH) ;
        p36.servo8_raw_SET((char)14521) ;
        p36.servo1_raw_SET((char)52072) ;
        p36.time_usec_SET(2152714193L) ;
        p36.servo4_raw_SET((char)32255) ;
        p36.servo6_raw_SET((char)42146) ;
        p36.servo15_raw_SET((char)53320, PH) ;
        p36.servo3_raw_SET((char)16300) ;
        p36.servo12_raw_SET((char)18376, PH) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_index_GET() == (short)4624);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)41);
            assert(pack.end_index_GET() == (short) -13236);
            assert(pack.target_component_GET() == (char)0);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_system_SET((char)41) ;
        p37.start_index_SET((short)4624) ;
        p37.end_index_SET((short) -13236) ;
        p37.target_component_SET((char)0) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)11);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)1);
            assert(pack.start_index_GET() == (short)19747);
            assert(pack.end_index_GET() == (short)9130);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.end_index_SET((short)9130) ;
        p38.start_index_SET((short)19747) ;
        p38.target_component_SET((char)1) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p38.target_system_SET((char)11) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 9.802777E37F);
            assert(pack.y_GET() == -3.280546E38F);
            assert(pack.target_system_GET() == (char)104);
            assert(pack.target_component_GET() == (char)160);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_GRIPPER);
            assert(pack.param3_GET() == -7.270843E37F);
            assert(pack.param1_GET() == -2.993157E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.z_GET() == -2.3862754E37F);
            assert(pack.autocontinue_GET() == (char)33);
            assert(pack.seq_GET() == (char)5059);
            assert(pack.param4_GET() == 2.426541E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.param2_GET() == 1.5619491E38F);
            assert(pack.current_GET() == (char)177);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.y_SET(-3.280546E38F) ;
        p39.seq_SET((char)5059) ;
        p39.param4_SET(2.426541E38F) ;
        p39.autocontinue_SET((char)33) ;
        p39.x_SET(9.802777E37F) ;
        p39.command_SET(MAV_CMD.MAV_CMD_DO_GRIPPER) ;
        p39.param2_SET(1.5619491E38F) ;
        p39.param3_SET(-7.270843E37F) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p39.current_SET((char)177) ;
        p39.param1_SET(-2.993157E37F) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p39.target_system_SET((char)104) ;
        p39.target_component_SET((char)160) ;
        p39.z_SET(-2.3862754E37F) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)5139);
            assert(pack.target_component_GET() == (char)107);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)245);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_component_SET((char)107) ;
        p40.target_system_SET((char)245) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p40.seq_SET((char)5139) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)195);
            assert(pack.seq_GET() == (char)54342);
            assert(pack.target_system_GET() == (char)197);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)197) ;
        p41.target_component_SET((char)195) ;
        p41.seq_SET((char)54342) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)55503);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)55503) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)230);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)207);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)207) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p43.target_component_SET((char)230) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)185);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.count_GET() == (char)64467);
            assert(pack.target_system_GET() == (char)40);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.target_system_SET((char)40) ;
        p44.count_SET((char)64467) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p44.target_component_SET((char)185) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)17);
            assert(pack.target_system_GET() == (char)46);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)46) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p45.target_component_SET((char)17) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)46343);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)46343) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)186);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE);
            assert(pack.target_system_GET() == (char)112);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p47.target_system_SET((char)112) ;
        p47.target_component_SET((char)186) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == 2107362338);
            assert(pack.target_system_GET() == (char)211);
            assert(pack.time_usec_TRY(ph) == 6075636464074334212L);
            assert(pack.latitude_GET() == -211790145);
            assert(pack.altitude_GET() == 1273196543);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.altitude_SET(1273196543) ;
        p48.time_usec_SET(6075636464074334212L, PH) ;
        p48.latitude_SET(-211790145) ;
        p48.longitude_SET(2107362338) ;
        p48.target_system_SET((char)211) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 1692395509);
            assert(pack.altitude_GET() == -530697279);
            assert(pack.time_usec_TRY(ph) == 2514911382841141764L);
            assert(pack.longitude_GET() == 1360936996);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.latitude_SET(1692395509) ;
        p49.time_usec_SET(2514911382841141764L, PH) ;
        p49.altitude_SET(-530697279) ;
        p49.longitude_SET(1360936996) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)236);
            assert(pack.parameter_rc_channel_index_GET() == (char)54);
            assert(pack.param_value_min_GET() == 1.02237174E37F);
            assert(pack.param_id_LEN(ph) == 7);
            assert(pack.param_id_TRY(ph).equals("jhqgetl"));
            assert(pack.target_system_GET() == (char)29);
            assert(pack.scale_GET() == 7.5345105E37F);
            assert(pack.param_value0_GET() == -2.9915035E38F);
            assert(pack.param_index_GET() == (short)3550);
            assert(pack.param_value_max_GET() == 8.2686866E37F);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.param_value_max_SET(8.2686866E37F) ;
        p50.param_id_SET("jhqgetl", PH) ;
        p50.scale_SET(7.5345105E37F) ;
        p50.target_component_SET((char)236) ;
        p50.param_index_SET((short)3550) ;
        p50.param_value0_SET(-2.9915035E38F) ;
        p50.param_value_min_SET(1.02237174E37F) ;
        p50.parameter_rc_channel_index_SET((char)54) ;
        p50.target_system_SET((char)29) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)29862);
            assert(pack.target_system_GET() == (char)2);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)102);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_system_SET((char)2) ;
        p51.target_component_SET((char)102) ;
        p51.seq_SET((char)29862) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1z_GET() == 7.43899E37F);
            assert(pack.p2y_GET() == -1.6505613E38F);
            assert(pack.p2z_GET() == 8.289066E37F);
            assert(pack.target_system_GET() == (char)237);
            assert(pack.target_component_GET() == (char)150);
            assert(pack.p2x_GET() == -2.6719592E38F);
            assert(pack.p1x_GET() == -2.4275446E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.p1y_GET() == 2.3755949E38F);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p54.target_component_SET((char)150) ;
        p54.p2z_SET(8.289066E37F) ;
        p54.p2y_SET(-1.6505613E38F) ;
        p54.target_system_SET((char)237) ;
        p54.p1x_SET(-2.4275446E38F) ;
        p54.p2x_SET(-2.6719592E38F) ;
        p54.p1y_SET(2.3755949E38F) ;
        p54.p1z_SET(7.43899E37F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1z_GET() == 1.3946458E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.p2z_GET() == -9.361785E37F);
            assert(pack.p2y_GET() == -7.4174476E37F);
            assert(pack.p1y_GET() == 2.438402E37F);
            assert(pack.p2x_GET() == 2.5331285E38F);
            assert(pack.p1x_GET() == -3.2744127E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p1y_SET(2.438402E37F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p55.p1z_SET(1.3946458E38F) ;
        p55.p2z_SET(-9.361785E37F) ;
        p55.p2y_SET(-7.4174476E37F) ;
        p55.p1x_SET(-3.2744127E38F) ;
        p55.p2x_SET(2.5331285E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7079782852484346093L);
            assert(pack.pitchspeed_GET() == -2.644232E38F);
            assert(pack.rollspeed_GET() == -3.1445907E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.6391064E38F, 1.9826398E38F, -1.973013E38F, -2.8215164E38F}));
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {7.581609E37F, 2.4855784E37F, 2.0987599E38F, 2.5865674E38F, -2.1058978E38F, -9.030805E37F, -1.976203E38F, 1.7201761E38F, -3.0994422E38F}));
            assert(pack.yawspeed_GET() == 3.1991795E37F);
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.covariance_SET(new float[] {7.581609E37F, 2.4855784E37F, 2.0987599E38F, 2.5865674E38F, -2.1058978E38F, -9.030805E37F, -1.976203E38F, 1.7201761E38F, -3.0994422E38F}, 0) ;
        p61.time_usec_SET(7079782852484346093L) ;
        p61.pitchspeed_SET(-2.644232E38F) ;
        p61.rollspeed_SET(-3.1445907E38F) ;
        p61.yawspeed_SET(3.1991795E37F) ;
        p61.q_SET(new float[] {1.6391064E38F, 1.9826398E38F, -1.973013E38F, -2.8215164E38F}, 0) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.nav_pitch_GET() == -9.140074E37F);
            assert(pack.target_bearing_GET() == (short)10258);
            assert(pack.xtrack_error_GET() == 1.5832314E38F);
            assert(pack.alt_error_GET() == -3.0358344E38F);
            assert(pack.nav_bearing_GET() == (short) -17984);
            assert(pack.nav_roll_GET() == 2.4686433E38F);
            assert(pack.wp_dist_GET() == (char)45043);
            assert(pack.aspd_error_GET() == -2.7738657E38F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.target_bearing_SET((short)10258) ;
        p62.nav_pitch_SET(-9.140074E37F) ;
        p62.xtrack_error_SET(1.5832314E38F) ;
        p62.nav_bearing_SET((short) -17984) ;
        p62.alt_error_SET(-3.0358344E38F) ;
        p62.aspd_error_SET(-2.7738657E38F) ;
        p62.nav_roll_SET(2.4686433E38F) ;
        p62.wp_dist_SET((char)45043) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == 9.995862E37F);
            assert(pack.time_usec_GET() == 2614886422871322742L);
            assert(pack.lon_GET() == -1656662276);
            assert(pack.vy_GET() == -2.534609E37F);
            assert(pack.alt_GET() == -2100813808);
            assert(pack.relative_alt_GET() == -106648861);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
            assert(pack.vz_GET() == -2.1553932E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.817861E38F, 2.1828343E38F, 1.6225483E38F, 8.177176E37F, 1.1216173E38F, -1.6282196E37F, 2.649841E38F, -3.2402898E38F, -2.8649078E38F, 1.1495483E38F, 2.4763392E38F, -1.7192543E36F, 4.566501E37F, 2.4060242E38F, -9.602111E37F, 8.413905E37F, -3.8974785E37F, 2.6524055E38F, -2.773771E38F, -1.7627108E38F, -7.213037E37F, -8.5019825E37F, 7.8376306E37F, 3.1334915E38F, -1.710735E38F, -1.256466E38F, -1.4238055E38F, -7.915352E37F, -1.938845E38F, 2.380171E38F, 1.4862454E38F, 4.904319E37F, 8.680555E37F, 2.3181484E38F, 2.1346567E38F, 2.843314E38F}));
            assert(pack.lat_GET() == -1234214631);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.vy_SET(-2.534609E37F) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS) ;
        p63.vx_SET(9.995862E37F) ;
        p63.vz_SET(-2.1553932E38F) ;
        p63.relative_alt_SET(-106648861) ;
        p63.lat_SET(-1234214631) ;
        p63.alt_SET(-2100813808) ;
        p63.time_usec_SET(2614886422871322742L) ;
        p63.lon_SET(-1656662276) ;
        p63.covariance_SET(new float[] {-1.817861E38F, 2.1828343E38F, 1.6225483E38F, 8.177176E37F, 1.1216173E38F, -1.6282196E37F, 2.649841E38F, -3.2402898E38F, -2.8649078E38F, 1.1495483E38F, 2.4763392E38F, -1.7192543E36F, 4.566501E37F, 2.4060242E38F, -9.602111E37F, 8.413905E37F, -3.8974785E37F, 2.6524055E38F, -2.773771E38F, -1.7627108E38F, -7.213037E37F, -8.5019825E37F, 7.8376306E37F, 3.1334915E38F, -1.710735E38F, -1.256466E38F, -1.4238055E38F, -7.915352E37F, -1.938845E38F, 2.380171E38F, 1.4862454E38F, 4.904319E37F, 8.680555E37F, 2.3181484E38F, 2.1346567E38F, 2.843314E38F}, 0) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 2.3526265E38F);
            assert(pack.z_GET() == -8.744203E37F);
            assert(pack.ay_GET() == 3.0092478E38F);
            assert(pack.vy_GET() == -1.0255275E38F);
            assert(pack.y_GET() == 2.3882805E38F);
            assert(pack.vx_GET() == 1.0949933E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.4927649E38F, 3.1699003E38F, 1.0330938E38F, 7.3968396E37F, -7.5541606E37F, 1.3874683E38F, -3.8177762E37F, 2.8838544E38F, 4.432726E37F, -1.3621122E38F, 3.016234E38F, -1.1175292E38F, -2.147665E38F, -1.8494586E38F, 2.4696564E38F, -1.7156823E38F, -2.1253304E38F, -2.3781926E38F, -2.7224952E37F, 3.3698061E38F, -1.9474389E37F, 2.773752E37F, -3.7817795E37F, 2.9593717E38F, 9.637827E37F, 7.977659E37F, 2.5815427E38F, 1.2693031E38F, -2.101133E38F, 1.7316827E38F, 2.4312508E38F, -1.475755E38F, 8.222058E36F, -4.1233076E37F, 1.9880946E38F, -2.9789156E38F, -2.9105344E37F, 2.2290407E38F, -3.1375924E38F, 1.2562888E38F, -8.118025E37F, -3.2310694E38F, 3.2076268E37F, 2.461026E38F, -7.1244905E37F}));
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            assert(pack.ax_GET() == -2.126893E38F);
            assert(pack.vz_GET() == 2.2341208E38F);
            assert(pack.az_GET() == 2.8374343E38F);
            assert(pack.time_usec_GET() == 5636657591880980334L);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.x_SET(2.3526265E38F) ;
        p64.az_SET(2.8374343E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        p64.time_usec_SET(5636657591880980334L) ;
        p64.vx_SET(1.0949933E38F) ;
        p64.covariance_SET(new float[] {-1.4927649E38F, 3.1699003E38F, 1.0330938E38F, 7.3968396E37F, -7.5541606E37F, 1.3874683E38F, -3.8177762E37F, 2.8838544E38F, 4.432726E37F, -1.3621122E38F, 3.016234E38F, -1.1175292E38F, -2.147665E38F, -1.8494586E38F, 2.4696564E38F, -1.7156823E38F, -2.1253304E38F, -2.3781926E38F, -2.7224952E37F, 3.3698061E38F, -1.9474389E37F, 2.773752E37F, -3.7817795E37F, 2.9593717E38F, 9.637827E37F, 7.977659E37F, 2.5815427E38F, 1.2693031E38F, -2.101133E38F, 1.7316827E38F, 2.4312508E38F, -1.475755E38F, 8.222058E36F, -4.1233076E37F, 1.9880946E38F, -2.9789156E38F, -2.9105344E37F, 2.2290407E38F, -3.1375924E38F, 1.2562888E38F, -8.118025E37F, -3.2310694E38F, 3.2076268E37F, 2.461026E38F, -7.1244905E37F}, 0) ;
        p64.z_SET(-8.744203E37F) ;
        p64.y_SET(2.3882805E38F) ;
        p64.vz_SET(2.2341208E38F) ;
        p64.ay_SET(3.0092478E38F) ;
        p64.ax_SET(-2.126893E38F) ;
        p64.vy_SET(-1.0255275E38F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan9_raw_GET() == (char)56065);
            assert(pack.chan12_raw_GET() == (char)22877);
            assert(pack.chan7_raw_GET() == (char)33553);
            assert(pack.chan10_raw_GET() == (char)49473);
            assert(pack.chan14_raw_GET() == (char)3288);
            assert(pack.chan11_raw_GET() == (char)34800);
            assert(pack.chan16_raw_GET() == (char)56727);
            assert(pack.chan17_raw_GET() == (char)40207);
            assert(pack.chancount_GET() == (char)193);
            assert(pack.chan2_raw_GET() == (char)26789);
            assert(pack.chan4_raw_GET() == (char)51056);
            assert(pack.time_boot_ms_GET() == 2985457733L);
            assert(pack.chan15_raw_GET() == (char)33771);
            assert(pack.rssi_GET() == (char)17);
            assert(pack.chan5_raw_GET() == (char)64930);
            assert(pack.chan1_raw_GET() == (char)31142);
            assert(pack.chan13_raw_GET() == (char)20595);
            assert(pack.chan18_raw_GET() == (char)51537);
            assert(pack.chan6_raw_GET() == (char)46031);
            assert(pack.chan8_raw_GET() == (char)52544);
            assert(pack.chan3_raw_GET() == (char)3277);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan2_raw_SET((char)26789) ;
        p65.chan3_raw_SET((char)3277) ;
        p65.chan13_raw_SET((char)20595) ;
        p65.chan18_raw_SET((char)51537) ;
        p65.time_boot_ms_SET(2985457733L) ;
        p65.chan10_raw_SET((char)49473) ;
        p65.chan7_raw_SET((char)33553) ;
        p65.chan16_raw_SET((char)56727) ;
        p65.chan4_raw_SET((char)51056) ;
        p65.chan9_raw_SET((char)56065) ;
        p65.chan8_raw_SET((char)52544) ;
        p65.chan15_raw_SET((char)33771) ;
        p65.chan1_raw_SET((char)31142) ;
        p65.chan12_raw_SET((char)22877) ;
        p65.chancount_SET((char)193) ;
        p65.chan6_raw_SET((char)46031) ;
        p65.chan14_raw_SET((char)3288) ;
        p65.chan17_raw_SET((char)40207) ;
        p65.chan5_raw_SET((char)64930) ;
        p65.rssi_SET((char)17) ;
        p65.chan11_raw_SET((char)34800) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.req_stream_id_GET() == (char)243);
            assert(pack.target_component_GET() == (char)172);
            assert(pack.start_stop_GET() == (char)29);
            assert(pack.req_message_rate_GET() == (char)43543);
            assert(pack.target_system_GET() == (char)119);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_component_SET((char)172) ;
        p66.target_system_SET((char)119) ;
        p66.req_stream_id_SET((char)243) ;
        p66.req_message_rate_SET((char)43543) ;
        p66.start_stop_SET((char)29) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.message_rate_GET() == (char)1011);
            assert(pack.stream_id_GET() == (char)72);
            assert(pack.on_off_GET() == (char)158);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.message_rate_SET((char)1011) ;
        p67.on_off_SET((char)158) ;
        p67.stream_id_SET((char)72) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == (short) -27860);
            assert(pack.y_GET() == (short)4007);
            assert(pack.r_GET() == (short) -17498);
            assert(pack.target_GET() == (char)224);
            assert(pack.buttons_GET() == (char)58198);
            assert(pack.z_GET() == (short) -2950);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.y_SET((short)4007) ;
        p69.buttons_SET((char)58198) ;
        p69.z_SET((short) -2950) ;
        p69.r_SET((short) -17498) ;
        p69.x_SET((short) -27860) ;
        p69.target_SET((char)224) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)96);
            assert(pack.chan6_raw_GET() == (char)1736);
            assert(pack.chan2_raw_GET() == (char)65326);
            assert(pack.chan3_raw_GET() == (char)59042);
            assert(pack.chan5_raw_GET() == (char)58725);
            assert(pack.chan8_raw_GET() == (char)10790);
            assert(pack.chan4_raw_GET() == (char)21572);
            assert(pack.target_system_GET() == (char)182);
            assert(pack.chan7_raw_GET() == (char)16668);
            assert(pack.chan1_raw_GET() == (char)12383);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan5_raw_SET((char)58725) ;
        p70.target_component_SET((char)96) ;
        p70.chan2_raw_SET((char)65326) ;
        p70.target_system_SET((char)182) ;
        p70.chan8_raw_SET((char)10790) ;
        p70.chan1_raw_SET((char)12383) ;
        p70.chan7_raw_SET((char)16668) ;
        p70.chan6_raw_SET((char)1736) ;
        p70.chan3_raw_SET((char)59042) ;
        p70.chan4_raw_SET((char)21572) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.param4_GET() == 4.082965E37F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL);
            assert(pack.z_GET() == -2.9998767E38F);
            assert(pack.y_GET() == -1886691470);
            assert(pack.param2_GET() == -1.9737871E37F);
            assert(pack.param3_GET() == 1.2517913E38F);
            assert(pack.seq_GET() == (char)28170);
            assert(pack.target_component_GET() == (char)35);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.x_GET() == -1898503715);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)42);
            assert(pack.current_GET() == (char)64);
            assert(pack.autocontinue_GET() == (char)255);
            assert(pack.param1_GET() == 3.443263E37F);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.param3_SET(1.2517913E38F) ;
        p73.command_SET(MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL) ;
        p73.autocontinue_SET((char)255) ;
        p73.param1_SET(3.443263E37F) ;
        p73.z_SET(-2.9998767E38F) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p73.param2_SET(-1.9737871E37F) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p73.param4_SET(4.082965E37F) ;
        p73.y_SET(-1886691470) ;
        p73.x_SET(-1898503715) ;
        p73.target_system_SET((char)42) ;
        p73.target_component_SET((char)35) ;
        p73.seq_SET((char)28170) ;
        p73.current_SET((char)64) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.heading_GET() == (short)4831);
            assert(pack.groundspeed_GET() == -1.339534E38F);
            assert(pack.alt_GET() == -1.1928092E38F);
            assert(pack.airspeed_GET() == -2.3711802E38F);
            assert(pack.throttle_GET() == (char)6638);
            assert(pack.climb_GET() == -2.944048E38F);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.climb_SET(-2.944048E38F) ;
        p74.airspeed_SET(-2.3711802E38F) ;
        p74.alt_SET(-1.1928092E38F) ;
        p74.groundspeed_SET(-1.339534E38F) ;
        p74.heading_SET((short)4831) ;
        p74.throttle_SET((char)6638) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.autocontinue_GET() == (char)37);
            assert(pack.x_GET() == 1969516060);
            assert(pack.param4_GET() == 2.2747777E38F);
            assert(pack.target_component_GET() == (char)235);
            assert(pack.param1_GET() == 1.8837564E38F);
            assert(pack.current_GET() == (char)109);
            assert(pack.target_system_GET() == (char)175);
            assert(pack.y_GET() == 735956557);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_SPATIAL_USER_4);
            assert(pack.z_GET() == 2.314979E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.param2_GET() == 1.6118251E38F);
            assert(pack.param3_GET() == -1.5529891E38F);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.y_SET(735956557) ;
        p75.param4_SET(2.2747777E38F) ;
        p75.current_SET((char)109) ;
        p75.param1_SET(1.8837564E38F) ;
        p75.autocontinue_SET((char)37) ;
        p75.z_SET(2.314979E38F) ;
        p75.param3_SET(-1.5529891E38F) ;
        p75.command_SET(MAV_CMD.MAV_CMD_SPATIAL_USER_4) ;
        p75.x_SET(1969516060) ;
        p75.target_system_SET((char)175) ;
        p75.param2_SET(1.6118251E38F) ;
        p75.target_component_SET((char)235) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.confirmation_GET() == (char)196);
            assert(pack.param2_GET() == -3.5829581E37F);
            assert(pack.param6_GET() == 2.1147039E38F);
            assert(pack.param5_GET() == -2.4584357E38F);
            assert(pack.param1_GET() == 1.180421E38F);
            assert(pack.target_component_GET() == (char)88);
            assert(pack.param3_GET() == 1.7374198E38F);
            assert(pack.param7_GET() == 9.967533E37F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_FOLLOW);
            assert(pack.param4_GET() == -4.4061725E37F);
            assert(pack.target_system_GET() == (char)59);
        });
        COMMAND_LONG p76 = new COMMAND_LONG();
        PH.setPack(p76);
        p76.command_SET(MAV_CMD.MAV_CMD_NAV_FOLLOW) ;
        p76.param2_SET(-3.5829581E37F) ;
        p76.param1_SET(1.180421E38F) ;
        p76.confirmation_SET((char)196) ;
        p76.param7_SET(9.967533E37F) ;
        p76.target_system_SET((char)59) ;
        p76.param6_SET(2.1147039E38F) ;
        p76.param3_SET(1.7374198E38F) ;
        p76.param5_SET(-2.4584357E38F) ;
        p76.param4_SET(-4.4061725E37F) ;
        p76.target_component_SET((char)88) ;
        TestChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_TRY(ph) == (char)98);
            assert(pack.progress_TRY(ph) == (char)221);
            assert(pack.result_param2_TRY(ph) == -817202939);
            assert(pack.target_system_TRY(ph) == (char)140);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_JUMP);
        });
        COMMAND_ACK p77 = new COMMAND_ACK();
        PH.setPack(p77);
        p77.result_param2_SET(-817202939, PH) ;
        p77.target_component_SET((char)98, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED) ;
        p77.progress_SET((char)221, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_DO_JUMP) ;
        p77.target_system_SET((char)140, PH) ;
        TestChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.manual_override_switch_GET() == (char)2);
            assert(pack.thrust_GET() == 1.0994151E36F);
            assert(pack.mode_switch_GET() == (char)47);
            assert(pack.yaw_GET() == 8.162315E37F);
            assert(pack.roll_GET() == -3.6130537E37F);
            assert(pack.pitch_GET() == -1.1643731E38F);
            assert(pack.time_boot_ms_GET() == 4179068054L);
        });
        MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.yaw_SET(8.162315E37F) ;
        p81.manual_override_switch_SET((char)2) ;
        p81.roll_SET(-3.6130537E37F) ;
        p81.time_boot_ms_SET(4179068054L) ;
        p81.mode_switch_SET((char)47) ;
        p81.pitch_SET(-1.1643731E38F) ;
        p81.thrust_SET(1.0994151E36F) ;
        TestChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_yaw_rate_GET() == 1.8398482E38F);
            assert(pack.body_pitch_rate_GET() == 2.1784008E38F);
            assert(pack.body_roll_rate_GET() == 1.3253423E38F);
            assert(pack.target_component_GET() == (char)188);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.1845646E38F, -2.2120718E38F, -3.249639E38F, 2.7427349E38F}));
            assert(pack.type_mask_GET() == (char)146);
            assert(pack.time_boot_ms_GET() == 574480465L);
            assert(pack.thrust_GET() == -3.0423807E38F);
            assert(pack.target_system_GET() == (char)211);
        });
        SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.target_component_SET((char)188) ;
        p82.target_system_SET((char)211) ;
        p82.body_roll_rate_SET(1.3253423E38F) ;
        p82.type_mask_SET((char)146) ;
        p82.q_SET(new float[] {2.1845646E38F, -2.2120718E38F, -3.249639E38F, 2.7427349E38F}, 0) ;
        p82.thrust_SET(-3.0423807E38F) ;
        p82.body_pitch_rate_SET(2.1784008E38F) ;
        p82.body_yaw_rate_SET(1.8398482E38F) ;
        p82.time_boot_ms_SET(574480465L) ;
        TestChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.thrust_GET() == 1.0015283E38F);
            assert(pack.body_pitch_rate_GET() == -2.8603568E38F);
            assert(pack.type_mask_GET() == (char)180);
            assert(pack.time_boot_ms_GET() == 3261102268L);
            assert(pack.body_roll_rate_GET() == 3.2956782E38F);
            assert(pack.body_yaw_rate_GET() == 4.902222E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-4.8884476E37F, -6.07821E37F, 2.2547498E38F, 8.088542E37F}));
        });
        ATTITUDE_TARGET p83 = new ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.body_yaw_rate_SET(4.902222E37F) ;
        p83.q_SET(new float[] {-4.8884476E37F, -6.07821E37F, 2.2547498E38F, 8.088542E37F}, 0) ;
        p83.body_pitch_rate_SET(-2.8603568E38F) ;
        p83.type_mask_SET((char)180) ;
        p83.time_boot_ms_SET(3261102268L) ;
        p83.thrust_SET(1.0015283E38F) ;
        p83.body_roll_rate_SET(3.2956782E38F) ;
        TestChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)229);
            assert(pack.time_boot_ms_GET() == 3053273393L);
            assert(pack.yaw_rate_GET() == 3.0758572E38F);
            assert(pack.target_system_GET() == (char)10);
            assert(pack.vz_GET() == 2.6492342E38F);
            assert(pack.vy_GET() == -3.9897845E37F);
            assert(pack.yaw_GET() == -5.506495E37F);
            assert(pack.z_GET() == -7.1029526E37F);
            assert(pack.afx_GET() == -2.478352E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.y_GET() == 3.023498E38F);
            assert(pack.vx_GET() == -2.5819686E38F);
            assert(pack.afy_GET() == -3.2887177E38F);
            assert(pack.x_GET() == 7.4913486E37F);
            assert(pack.type_mask_GET() == (char)1116);
            assert(pack.afz_GET() == 2.4266615E38F);
        });
        SET_POSITION_TARGET_LOCAL_NED p84 = new SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.x_SET(7.4913486E37F) ;
        p84.type_mask_SET((char)1116) ;
        p84.z_SET(-7.1029526E37F) ;
        p84.afy_SET(-3.2887177E38F) ;
        p84.vz_SET(2.6492342E38F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p84.y_SET(3.023498E38F) ;
        p84.time_boot_ms_SET(3053273393L) ;
        p84.yaw_SET(-5.506495E37F) ;
        p84.afz_SET(2.4266615E38F) ;
        p84.afx_SET(-2.478352E38F) ;
        p84.vx_SET(-2.5819686E38F) ;
        p84.target_component_SET((char)229) ;
        p84.yaw_rate_SET(3.0758572E38F) ;
        p84.vy_SET(-3.9897845E37F) ;
        p84.target_system_SET((char)10) ;
        TestChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 499445004L);
            assert(pack.yaw_rate_GET() == -2.0978374E38F);
            assert(pack.vy_GET() == -2.950709E38F);
            assert(pack.afy_GET() == -1.4453933E38F);
            assert(pack.target_system_GET() == (char)217);
            assert(pack.vz_GET() == 1.8556379E38F);
            assert(pack.alt_GET() == 1.4976581E38F);
            assert(pack.afx_GET() == -1.3441013E37F);
            assert(pack.vx_GET() == 2.465779E38F);
            assert(pack.target_component_GET() == (char)77);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.afz_GET() == -7.351532E37F);
            assert(pack.lat_int_GET() == 1095854307);
            assert(pack.type_mask_GET() == (char)59564);
            assert(pack.lon_int_GET() == -82715209);
            assert(pack.yaw_GET() == -1.0220074E38F);
        });
        SET_POSITION_TARGET_GLOBAL_INT p86 = new SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p86.type_mask_SET((char)59564) ;
        p86.vx_SET(2.465779E38F) ;
        p86.time_boot_ms_SET(499445004L) ;
        p86.afz_SET(-7.351532E37F) ;
        p86.vy_SET(-2.950709E38F) ;
        p86.afy_SET(-1.4453933E38F) ;
        p86.target_component_SET((char)77) ;
        p86.alt_SET(1.4976581E38F) ;
        p86.target_system_SET((char)217) ;
        p86.yaw_rate_SET(-2.0978374E38F) ;
        p86.afx_SET(-1.3441013E37F) ;
        p86.vz_SET(1.8556379E38F) ;
        p86.yaw_SET(-1.0220074E38F) ;
        p86.lat_int_SET(1095854307) ;
        p86.lon_int_SET(-82715209) ;
        TestChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.afy_GET() == 2.1993785E38F);
            assert(pack.vx_GET() == -9.892353E37F);
            assert(pack.yaw_GET() == -3.2790073E38F);
            assert(pack.afx_GET() == -4.8923905E37F);
            assert(pack.lat_int_GET() == 1404714622);
            assert(pack.alt_GET() == 1.3353266E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.vy_GET() == -2.181065E38F);
            assert(pack.type_mask_GET() == (char)60775);
            assert(pack.afz_GET() == 3.3879542E38F);
            assert(pack.time_boot_ms_GET() == 3265151524L);
            assert(pack.yaw_rate_GET() == -1.7123086E38F);
            assert(pack.lon_int_GET() == -1974242393);
            assert(pack.vz_GET() == -2.3620673E38F);
        });
        POSITION_TARGET_GLOBAL_INT p87 = new POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.vx_SET(-9.892353E37F) ;
        p87.time_boot_ms_SET(3265151524L) ;
        p87.lon_int_SET(-1974242393) ;
        p87.yaw_SET(-3.2790073E38F) ;
        p87.alt_SET(1.3353266E38F) ;
        p87.type_mask_SET((char)60775) ;
        p87.afy_SET(2.1993785E38F) ;
        p87.afz_SET(3.3879542E38F) ;
        p87.vz_SET(-2.3620673E38F) ;
        p87.afx_SET(-4.8923905E37F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p87.lat_int_SET(1404714622) ;
        p87.vy_SET(-2.181065E38F) ;
        p87.yaw_rate_SET(-1.7123086E38F) ;
        TestChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 2.3581757E38F);
            assert(pack.x_GET() == -3.2377084E38F);
            assert(pack.y_GET() == 3.2849959E38F);
            assert(pack.time_boot_ms_GET() == 1531022204L);
            assert(pack.roll_GET() == 5.684042E37F);
            assert(pack.yaw_GET() == 3.2645625E38F);
            assert(pack.z_GET() == 1.654417E38F);
        });
        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = new LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.yaw_SET(3.2645625E38F) ;
        p89.z_SET(1.654417E38F) ;
        p89.time_boot_ms_SET(1531022204L) ;
        p89.x_SET(-3.2377084E38F) ;
        p89.pitch_SET(2.3581757E38F) ;
        p89.roll_SET(5.684042E37F) ;
        p89.y_SET(3.2849959E38F) ;
        TestChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short) -12505);
            assert(pack.yaw_GET() == 5.4975896E36F);
            assert(pack.xacc_GET() == (short) -8010);
            assert(pack.vy_GET() == (short) -9351);
            assert(pack.vx_GET() == (short) -11570);
            assert(pack.lon_GET() == -1691885156);
            assert(pack.yawspeed_GET() == 4.492994E35F);
            assert(pack.lat_GET() == 1299068051);
            assert(pack.roll_GET() == -1.8036964E38F);
            assert(pack.pitchspeed_GET() == -1.065376E38F);
            assert(pack.time_usec_GET() == 2613780827068405387L);
            assert(pack.zacc_GET() == (short)24629);
            assert(pack.rollspeed_GET() == -1.6701036E38F);
            assert(pack.vz_GET() == (short)7674);
            assert(pack.pitch_GET() == -1.6848498E38F);
            assert(pack.alt_GET() == -1866761200);
        });
        HIL_STATE p90 = new HIL_STATE();
        PH.setPack(p90);
        p90.lat_SET(1299068051) ;
        p90.vx_SET((short) -11570) ;
        p90.yaw_SET(5.4975896E36F) ;
        p90.pitchspeed_SET(-1.065376E38F) ;
        p90.time_usec_SET(2613780827068405387L) ;
        p90.lon_SET(-1691885156) ;
        p90.roll_SET(-1.8036964E38F) ;
        p90.vz_SET((short)7674) ;
        p90.yawspeed_SET(4.492994E35F) ;
        p90.rollspeed_SET(-1.6701036E38F) ;
        p90.yacc_SET((short) -12505) ;
        p90.vy_SET((short) -9351) ;
        p90.xacc_SET((short) -8010) ;
        p90.pitch_SET(-1.6848498E38F) ;
        p90.zacc_SET((short)24629) ;
        p90.alt_SET(-1866761200) ;
        TestChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.aux1_GET() == -2.8671944E38F);
            assert(pack.roll_ailerons_GET() == 1.0896261E38F);
            assert(pack.aux2_GET() == -1.5620167E38F);
            assert(pack.time_usec_GET() == 7236631087812986662L);
            assert(pack.aux4_GET() == 1.172716E38F);
            assert(pack.nav_mode_GET() == (char)216);
            assert(pack.aux3_GET() == 2.856354E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_AUTO_ARMED);
            assert(pack.pitch_elevator_GET() == 3.2570363E38F);
            assert(pack.yaw_rudder_GET() == -3.307679E38F);
            assert(pack.throttle_GET() == -1.6022439E38F);
        });
        HIL_CONTROLS p91 = new HIL_CONTROLS();
        PH.setPack(p91);
        p91.nav_mode_SET((char)216) ;
        p91.roll_ailerons_SET(1.0896261E38F) ;
        p91.aux4_SET(1.172716E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED) ;
        p91.pitch_elevator_SET(3.2570363E38F) ;
        p91.aux1_SET(-2.8671944E38F) ;
        p91.throttle_SET(-1.6022439E38F) ;
        p91.aux3_SET(2.856354E38F) ;
        p91.time_usec_SET(7236631087812986662L) ;
        p91.aux2_SET(-1.5620167E38F) ;
        p91.yaw_rudder_SET(-3.307679E38F) ;
        TestChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan1_raw_GET() == (char)33040);
            assert(pack.chan6_raw_GET() == (char)62033);
            assert(pack.chan2_raw_GET() == (char)49257);
            assert(pack.rssi_GET() == (char)132);
            assert(pack.chan12_raw_GET() == (char)3428);
            assert(pack.chan10_raw_GET() == (char)55393);
            assert(pack.chan8_raw_GET() == (char)27262);
            assert(pack.chan9_raw_GET() == (char)37313);
            assert(pack.time_usec_GET() == 8773505595551185087L);
            assert(pack.chan5_raw_GET() == (char)56680);
            assert(pack.chan3_raw_GET() == (char)11705);
            assert(pack.chan7_raw_GET() == (char)21986);
            assert(pack.chan11_raw_GET() == (char)17572);
            assert(pack.chan4_raw_GET() == (char)54087);
        });
        HIL_RC_INPUTS_RAW p92 = new HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan5_raw_SET((char)56680) ;
        p92.chan6_raw_SET((char)62033) ;
        p92.chan2_raw_SET((char)49257) ;
        p92.chan11_raw_SET((char)17572) ;
        p92.chan7_raw_SET((char)21986) ;
        p92.chan1_raw_SET((char)33040) ;
        p92.rssi_SET((char)132) ;
        p92.chan4_raw_SET((char)54087) ;
        p92.chan10_raw_SET((char)55393) ;
        p92.chan12_raw_SET((char)3428) ;
        p92.chan3_raw_SET((char)11705) ;
        p92.chan8_raw_SET((char)27262) ;
        p92.time_usec_SET(8773505595551185087L) ;
        p92.chan9_raw_SET((char)37313) ;
        TestChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.3433154E38F, -1.8420608E38F, 2.4892392E38F, -2.8215357E38F, 1.9941963E38F, -2.5109039E38F, 2.2173834E38F, -3.3427707E38F, 2.1999397E38F, -3.0400935E38F, 3.0761173E38F, -1.7380031E38F, -2.1834E38F, -1.0439471E38F, -2.190802E38F, 3.8620687E37F}));
            assert(pack.time_usec_GET() == 7927459274255178889L);
            assert(pack.flags_GET() == 5842119115973682250L);
        });
        HIL_ACTUATOR_CONTROLS p93 = new HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.flags_SET(5842119115973682250L) ;
        p93.controls_SET(new float[] {2.3433154E38F, -1.8420608E38F, 2.4892392E38F, -2.8215357E38F, 1.9941963E38F, -2.5109039E38F, 2.2173834E38F, -3.3427707E38F, 2.1999397E38F, -3.0400935E38F, 3.0761173E38F, -1.7380031E38F, -2.1834E38F, -1.0439471E38F, -2.190802E38F, 3.8620687E37F}, 0) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_GUIDED_DISARMED) ;
        p93.time_usec_SET(7927459274255178889L) ;
        TestChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.sensor_id_GET() == (char)219);
            assert(pack.flow_comp_m_x_GET() == -4.753361E37F);
            assert(pack.ground_distance_GET() == 1.6488542E38F);
            assert(pack.flow_x_GET() == (short)21814);
            assert(pack.quality_GET() == (char)60);
            assert(pack.flow_rate_x_TRY(ph) == -1.963097E38F);
            assert(pack.flow_rate_y_TRY(ph) == -1.8502152E38F);
            assert(pack.flow_y_GET() == (short)21777);
            assert(pack.time_usec_GET() == 4550308949704712395L);
            assert(pack.flow_comp_m_y_GET() == -2.3767646E38F);
        });
        OPTICAL_FLOW p100 = new OPTICAL_FLOW();
        PH.setPack(p100);
        p100.flow_rate_x_SET(-1.963097E38F, PH) ;
        p100.time_usec_SET(4550308949704712395L) ;
        p100.quality_SET((char)60) ;
        p100.flow_x_SET((short)21814) ;
        p100.flow_y_SET((short)21777) ;
        p100.ground_distance_SET(1.6488542E38F) ;
        p100.flow_rate_y_SET(-1.8502152E38F, PH) ;
        p100.flow_comp_m_y_SET(-2.3767646E38F) ;
        p100.sensor_id_SET((char)219) ;
        p100.flow_comp_m_x_SET(-4.753361E37F) ;
        TestChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 3.4098143E37F);
            assert(pack.roll_GET() == -2.5220258E38F);
            assert(pack.usec_GET() == 5687728095172627687L);
            assert(pack.x_GET() == 6.2411687E37F);
            assert(pack.yaw_GET() == 1.7190533E38F);
            assert(pack.y_GET() == -4.8878265E37F);
            assert(pack.z_GET() == -8.7658216E36F);
        });
        GLOBAL_VISION_POSITION_ESTIMATE p101 = new GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.usec_SET(5687728095172627687L) ;
        p101.x_SET(6.2411687E37F) ;
        p101.yaw_SET(1.7190533E38F) ;
        p101.roll_SET(-2.5220258E38F) ;
        p101.pitch_SET(3.4098143E37F) ;
        p101.z_SET(-8.7658216E36F) ;
        p101.y_SET(-4.8878265E37F) ;
        TestChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -1.1762456E38F);
            assert(pack.y_GET() == -1.4647574E38F);
            assert(pack.pitch_GET() == -3.264501E38F);
            assert(pack.usec_GET() == 5144593175060984993L);
            assert(pack.x_GET() == -9.171073E37F);
            assert(pack.roll_GET() == 1.0639917E38F);
            assert(pack.z_GET() == -1.247857E38F);
        });
        VISION_POSITION_ESTIMATE p102 = new VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.yaw_SET(-1.1762456E38F) ;
        p102.y_SET(-1.4647574E38F) ;
        p102.pitch_SET(-3.264501E38F) ;
        p102.roll_SET(1.0639917E38F) ;
        p102.z_SET(-1.247857E38F) ;
        p102.x_SET(-9.171073E37F) ;
        p102.usec_SET(5144593175060984993L) ;
        TestChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -1.4278028E38F);
            assert(pack.x_GET() == 3.1895462E38F);
            assert(pack.usec_GET() == 5647302648455346075L);
            assert(pack.z_GET() == -1.7241264E37F);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.z_SET(-1.7241264E37F) ;
        p103.x_SET(3.1895462E38F) ;
        p103.y_SET(-1.4278028E38F) ;
        p103.usec_SET(5647302648455346075L) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -3.0098514E38F);
            assert(pack.pitch_GET() == 2.247378E38F);
            assert(pack.roll_GET() == -2.0839746E38F);
            assert(pack.usec_GET() == 4736662468842449131L);
            assert(pack.z_GET() == -2.312919E38F);
            assert(pack.yaw_GET() == -2.41398E38F);
            assert(pack.y_GET() == 2.3199028E38F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.yaw_SET(-2.41398E38F) ;
        p104.pitch_SET(2.247378E38F) ;
        p104.x_SET(-3.0098514E38F) ;
        p104.roll_SET(-2.0839746E38F) ;
        p104.usec_SET(4736662468842449131L) ;
        p104.z_SET(-2.312919E38F) ;
        p104.y_SET(2.3199028E38F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == 2.23733E38F);
            assert(pack.temperature_GET() == -1.506621E38F);
            assert(pack.pressure_alt_GET() == -1.4021635E38F);
            assert(pack.ygyro_GET() == -1.1338157E38F);
            assert(pack.zmag_GET() == 2.3108368E38F);
            assert(pack.xacc_GET() == -2.4883206E38F);
            assert(pack.xmag_GET() == 1.8040317E38F);
            assert(pack.xgyro_GET() == -1.7179558E38F);
            assert(pack.zgyro_GET() == -3.0906587E38F);
            assert(pack.diff_pressure_GET() == 7.3525226E37F);
            assert(pack.time_usec_GET() == 5561413886255488405L);
            assert(pack.fields_updated_GET() == (char)49529);
            assert(pack.ymag_GET() == 1.2604574E38F);
            assert(pack.abs_pressure_GET() == -1.6009847E38F);
            assert(pack.yacc_GET() == 2.2846702E37F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.yacc_SET(2.2846702E37F) ;
        p105.xacc_SET(-2.4883206E38F) ;
        p105.zacc_SET(2.23733E38F) ;
        p105.time_usec_SET(5561413886255488405L) ;
        p105.zmag_SET(2.3108368E38F) ;
        p105.ygyro_SET(-1.1338157E38F) ;
        p105.xgyro_SET(-1.7179558E38F) ;
        p105.ymag_SET(1.2604574E38F) ;
        p105.zgyro_SET(-3.0906587E38F) ;
        p105.temperature_SET(-1.506621E38F) ;
        p105.abs_pressure_SET(-1.6009847E38F) ;
        p105.xmag_SET(1.8040317E38F) ;
        p105.pressure_alt_SET(-1.4021635E38F) ;
        p105.diff_pressure_SET(7.3525226E37F) ;
        p105.fields_updated_SET((char)49529) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.sensor_id_GET() == (char)231);
            assert(pack.temperature_GET() == (short) -21212);
            assert(pack.quality_GET() == (char)9);
            assert(pack.distance_GET() == -2.7039488E38F);
            assert(pack.integrated_ygyro_GET() == 1.2790731E38F);
            assert(pack.integrated_y_GET() == -2.9882025E38F);
            assert(pack.integrated_zgyro_GET() == 2.8226506E38F);
            assert(pack.integrated_x_GET() == 5.4719E37F);
            assert(pack.integrated_xgyro_GET() == 2.1762952E37F);
            assert(pack.time_usec_GET() == 3086083758887356324L);
            assert(pack.time_delta_distance_us_GET() == 277339533L);
            assert(pack.integration_time_us_GET() == 765936120L);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_usec_SET(3086083758887356324L) ;
        p106.temperature_SET((short) -21212) ;
        p106.integrated_y_SET(-2.9882025E38F) ;
        p106.integrated_zgyro_SET(2.8226506E38F) ;
        p106.integrated_x_SET(5.4719E37F) ;
        p106.distance_SET(-2.7039488E38F) ;
        p106.integrated_xgyro_SET(2.1762952E37F) ;
        p106.integrated_ygyro_SET(1.2790731E38F) ;
        p106.time_delta_distance_us_SET(277339533L) ;
        p106.quality_SET((char)9) ;
        p106.integration_time_us_SET(765936120L) ;
        p106.sensor_id_SET((char)231) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.diff_pressure_GET() == 5.3763643E37F);
            assert(pack.temperature_GET() == -8.494815E37F);
            assert(pack.fields_updated_GET() == 3097915040L);
            assert(pack.pressure_alt_GET() == -2.9887147E38F);
            assert(pack.abs_pressure_GET() == -1.0652529E38F);
            assert(pack.xacc_GET() == 3.0005107E38F);
            assert(pack.ymag_GET() == 1.133604E38F);
            assert(pack.time_usec_GET() == 6849895890152366255L);
            assert(pack.zmag_GET() == -2.8434754E38F);
            assert(pack.xmag_GET() == 1.6465383E38F);
            assert(pack.yacc_GET() == -7.4082825E37F);
            assert(pack.xgyro_GET() == -3.0911216E38F);
            assert(pack.ygyro_GET() == 3.1782319E38F);
            assert(pack.zgyro_GET() == -2.8288544E38F);
            assert(pack.zacc_GET() == -8.444326E37F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.fields_updated_SET(3097915040L) ;
        p107.xgyro_SET(-3.0911216E38F) ;
        p107.diff_pressure_SET(5.3763643E37F) ;
        p107.yacc_SET(-7.4082825E37F) ;
        p107.pressure_alt_SET(-2.9887147E38F) ;
        p107.zgyro_SET(-2.8288544E38F) ;
        p107.zacc_SET(-8.444326E37F) ;
        p107.ymag_SET(1.133604E38F) ;
        p107.abs_pressure_SET(-1.0652529E38F) ;
        p107.xmag_SET(1.6465383E38F) ;
        p107.time_usec_SET(6849895890152366255L) ;
        p107.zmag_SET(-2.8434754E38F) ;
        p107.xacc_SET(3.0005107E38F) ;
        p107.ygyro_SET(3.1782319E38F) ;
        p107.temperature_SET(-8.494815E37F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == 1.2920894E38F);
            assert(pack.alt_GET() == 1.1164607E38F);
            assert(pack.q4_GET() == -1.499783E38F);
            assert(pack.q3_GET() == -1.8005332E38F);
            assert(pack.ygyro_GET() == 1.7472718E38F);
            assert(pack.ve_GET() == -1.0482887E38F);
            assert(pack.std_dev_vert_GET() == 1.8425417E38F);
            assert(pack.q2_GET() == 8.4064407E37F);
            assert(pack.vd_GET() == -1.7781388E38F);
            assert(pack.lat_GET() == -1.7247437E38F);
            assert(pack.yacc_GET() == 1.7756994E38F);
            assert(pack.yaw_GET() == -2.6700685E38F);
            assert(pack.lon_GET() == -3.207852E38F);
            assert(pack.xgyro_GET() == -1.5478513E37F);
            assert(pack.roll_GET() == 2.4854904E38F);
            assert(pack.xacc_GET() == -2.3601853E38F);
            assert(pack.vn_GET() == 6.184658E37F);
            assert(pack.q1_GET() == -3.0634768E38F);
            assert(pack.zacc_GET() == -2.4799525E38F);
            assert(pack.pitch_GET() == -3.1308443E38F);
            assert(pack.std_dev_horz_GET() == -3.071961E37F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.std_dev_vert_SET(1.8425417E38F) ;
        p108.lat_SET(-1.7247437E38F) ;
        p108.std_dev_horz_SET(-3.071961E37F) ;
        p108.q4_SET(-1.499783E38F) ;
        p108.q3_SET(-1.8005332E38F) ;
        p108.lon_SET(-3.207852E38F) ;
        p108.ygyro_SET(1.7472718E38F) ;
        p108.vn_SET(6.184658E37F) ;
        p108.yacc_SET(1.7756994E38F) ;
        p108.zacc_SET(-2.4799525E38F) ;
        p108.yaw_SET(-2.6700685E38F) ;
        p108.q2_SET(8.4064407E37F) ;
        p108.q1_SET(-3.0634768E38F) ;
        p108.vd_SET(-1.7781388E38F) ;
        p108.xgyro_SET(-1.5478513E37F) ;
        p108.pitch_SET(-3.1308443E38F) ;
        p108.alt_SET(1.1164607E38F) ;
        p108.ve_SET(-1.0482887E38F) ;
        p108.xacc_SET(-2.3601853E38F) ;
        p108.roll_SET(2.4854904E38F) ;
        p108.zgyro_SET(1.2920894E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.rssi_GET() == (char)12);
            assert(pack.remnoise_GET() == (char)1);
            assert(pack.remrssi_GET() == (char)211);
            assert(pack.fixed__GET() == (char)50113);
            assert(pack.txbuf_GET() == (char)104);
            assert(pack.noise_GET() == (char)104);
            assert(pack.rxerrors_GET() == (char)30897);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)12) ;
        p109.remrssi_SET((char)211) ;
        p109.fixed__SET((char)50113) ;
        p109.txbuf_SET((char)104) ;
        p109.rxerrors_SET((char)30897) ;
        p109.noise_SET((char)104) ;
        p109.remnoise_SET((char)1) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)67, (char)77, (char)209, (char)105, (char)109, (char)101, (char)64, (char)66, (char)182, (char)112, (char)138, (char)192, (char)205, (char)143, (char)237, (char)12, (char)237, (char)215, (char)77, (char)194, (char)138, (char)70, (char)29, (char)90, (char)202, (char)4, (char)82, (char)211, (char)44, (char)42, (char)103, (char)52, (char)199, (char)131, (char)232, (char)165, (char)125, (char)11, (char)96, (char)88, (char)170, (char)15, (char)157, (char)219, (char)87, (char)227, (char)38, (char)246, (char)156, (char)77, (char)141, (char)135, (char)118, (char)33, (char)15, (char)192, (char)255, (char)203, (char)242, (char)105, (char)118, (char)49, (char)32, (char)61, (char)70, (char)196, (char)253, (char)2, (char)128, (char)159, (char)255, (char)191, (char)124, (char)113, (char)42, (char)140, (char)179, (char)22, (char)55, (char)239, (char)233, (char)179, (char)149, (char)223, (char)22, (char)102, (char)101, (char)160, (char)179, (char)152, (char)84, (char)60, (char)107, (char)118, (char)191, (char)61, (char)178, (char)114, (char)61, (char)25, (char)6, (char)129, (char)72, (char)77, (char)82, (char)120, (char)9, (char)230, (char)175, (char)191, (char)10, (char)232, (char)68, (char)172, (char)92, (char)239, (char)64, (char)86, (char)7, (char)198, (char)137, (char)84, (char)234, (char)5, (char)121, (char)58, (char)194, (char)146, (char)2, (char)71, (char)157, (char)219, (char)195, (char)222, (char)109, (char)208, (char)5, (char)96, (char)182, (char)137, (char)84, (char)63, (char)198, (char)221, (char)24, (char)80, (char)142, (char)104, (char)41, (char)187, (char)81, (char)127, (char)26, (char)210, (char)89, (char)216, (char)218, (char)112, (char)159, (char)91, (char)67, (char)161, (char)1, (char)48, (char)94, (char)150, (char)184, (char)61, (char)242, (char)207, (char)157, (char)3, (char)98, (char)114, (char)207, (char)4, (char)150, (char)183, (char)96, (char)3, (char)29, (char)119, (char)2, (char)202, (char)190, (char)184, (char)181, (char)59, (char)69, (char)86, (char)140, (char)34, (char)75, (char)183, (char)7, (char)210, (char)67, (char)254, (char)52, (char)128, (char)161, (char)56, (char)187, (char)19, (char)146, (char)35, (char)12, (char)178, (char)231, (char)242, (char)107, (char)126, (char)143, (char)80, (char)201, (char)170, (char)63, (char)158, (char)100, (char)111, (char)29, (char)65, (char)141, (char)186, (char)116, (char)62, (char)41, (char)129, (char)52, (char)47, (char)55, (char)147, (char)201, (char)72, (char)11, (char)235, (char)255, (char)60, (char)219, (char)247, (char)24, (char)41, (char)167, (char)182, (char)236, (char)60, (char)85, (char)79, (char)219, (char)173, (char)104}));
            assert(pack.target_system_GET() == (char)13);
            assert(pack.target_network_GET() == (char)138);
            assert(pack.target_component_GET() == (char)147);
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)138) ;
        p110.payload_SET(new char[] {(char)67, (char)77, (char)209, (char)105, (char)109, (char)101, (char)64, (char)66, (char)182, (char)112, (char)138, (char)192, (char)205, (char)143, (char)237, (char)12, (char)237, (char)215, (char)77, (char)194, (char)138, (char)70, (char)29, (char)90, (char)202, (char)4, (char)82, (char)211, (char)44, (char)42, (char)103, (char)52, (char)199, (char)131, (char)232, (char)165, (char)125, (char)11, (char)96, (char)88, (char)170, (char)15, (char)157, (char)219, (char)87, (char)227, (char)38, (char)246, (char)156, (char)77, (char)141, (char)135, (char)118, (char)33, (char)15, (char)192, (char)255, (char)203, (char)242, (char)105, (char)118, (char)49, (char)32, (char)61, (char)70, (char)196, (char)253, (char)2, (char)128, (char)159, (char)255, (char)191, (char)124, (char)113, (char)42, (char)140, (char)179, (char)22, (char)55, (char)239, (char)233, (char)179, (char)149, (char)223, (char)22, (char)102, (char)101, (char)160, (char)179, (char)152, (char)84, (char)60, (char)107, (char)118, (char)191, (char)61, (char)178, (char)114, (char)61, (char)25, (char)6, (char)129, (char)72, (char)77, (char)82, (char)120, (char)9, (char)230, (char)175, (char)191, (char)10, (char)232, (char)68, (char)172, (char)92, (char)239, (char)64, (char)86, (char)7, (char)198, (char)137, (char)84, (char)234, (char)5, (char)121, (char)58, (char)194, (char)146, (char)2, (char)71, (char)157, (char)219, (char)195, (char)222, (char)109, (char)208, (char)5, (char)96, (char)182, (char)137, (char)84, (char)63, (char)198, (char)221, (char)24, (char)80, (char)142, (char)104, (char)41, (char)187, (char)81, (char)127, (char)26, (char)210, (char)89, (char)216, (char)218, (char)112, (char)159, (char)91, (char)67, (char)161, (char)1, (char)48, (char)94, (char)150, (char)184, (char)61, (char)242, (char)207, (char)157, (char)3, (char)98, (char)114, (char)207, (char)4, (char)150, (char)183, (char)96, (char)3, (char)29, (char)119, (char)2, (char)202, (char)190, (char)184, (char)181, (char)59, (char)69, (char)86, (char)140, (char)34, (char)75, (char)183, (char)7, (char)210, (char)67, (char)254, (char)52, (char)128, (char)161, (char)56, (char)187, (char)19, (char)146, (char)35, (char)12, (char)178, (char)231, (char)242, (char)107, (char)126, (char)143, (char)80, (char)201, (char)170, (char)63, (char)158, (char)100, (char)111, (char)29, (char)65, (char)141, (char)186, (char)116, (char)62, (char)41, (char)129, (char)52, (char)47, (char)55, (char)147, (char)201, (char)72, (char)11, (char)235, (char)255, (char)60, (char)219, (char)247, (char)24, (char)41, (char)167, (char)182, (char)236, (char)60, (char)85, (char)79, (char)219, (char)173, (char)104}, 0) ;
        p110.target_component_SET((char)147) ;
        p110.target_system_SET((char)13) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == -3723057503137289252L);
            assert(pack.ts1_GET() == -8383157274772566681L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(-3723057503137289252L) ;
        p111.ts1_SET(-8383157274772566681L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 2645132099L);
            assert(pack.time_usec_GET() == 7419571713161140823L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(2645132099L) ;
        p112.time_usec_SET(7419571713161140823L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.fix_type_GET() == (char)241);
            assert(pack.ve_GET() == (short) -18121);
            assert(pack.lat_GET() == 1600338336);
            assert(pack.alt_GET() == -1532210581);
            assert(pack.epv_GET() == (char)24416);
            assert(pack.vel_GET() == (char)59686);
            assert(pack.satellites_visible_GET() == (char)21);
            assert(pack.lon_GET() == -2103556744);
            assert(pack.eph_GET() == (char)60936);
            assert(pack.vd_GET() == (short)7769);
            assert(pack.vn_GET() == (short)1088);
            assert(pack.cog_GET() == (char)45842);
            assert(pack.time_usec_GET() == 3216391635303014443L);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.cog_SET((char)45842) ;
        p113.fix_type_SET((char)241) ;
        p113.ve_SET((short) -18121) ;
        p113.alt_SET(-1532210581) ;
        p113.satellites_visible_SET((char)21) ;
        p113.lat_SET(1600338336) ;
        p113.time_usec_SET(3216391635303014443L) ;
        p113.vn_SET((short)1088) ;
        p113.epv_SET((char)24416) ;
        p113.vd_SET((short)7769) ;
        p113.eph_SET((char)60936) ;
        p113.vel_SET((char)59686) ;
        p113.lon_SET(-2103556744) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.distance_GET() == -2.2826268E38F);
            assert(pack.integrated_zgyro_GET() == -3.2264862E38F);
            assert(pack.integrated_y_GET() == -2.0085293E38F);
            assert(pack.temperature_GET() == (short) -10132);
            assert(pack.time_usec_GET() == 909405912520413479L);
            assert(pack.sensor_id_GET() == (char)246);
            assert(pack.integrated_xgyro_GET() == -6.9347323E37F);
            assert(pack.integrated_x_GET() == -2.1798956E38F);
            assert(pack.integrated_ygyro_GET() == 4.682207E37F);
            assert(pack.integration_time_us_GET() == 1989861592L);
            assert(pack.time_delta_distance_us_GET() == 1584113291L);
            assert(pack.quality_GET() == (char)15);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.sensor_id_SET((char)246) ;
        p114.integrated_y_SET(-2.0085293E38F) ;
        p114.integrated_ygyro_SET(4.682207E37F) ;
        p114.time_usec_SET(909405912520413479L) ;
        p114.integrated_x_SET(-2.1798956E38F) ;
        p114.integrated_xgyro_SET(-6.9347323E37F) ;
        p114.distance_SET(-2.2826268E38F) ;
        p114.time_delta_distance_us_SET(1584113291L) ;
        p114.temperature_SET((short) -10132) ;
        p114.integrated_zgyro_SET(-3.2264862E38F) ;
        p114.quality_SET((char)15) ;
        p114.integration_time_us_SET(1989861592L) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.ind_airspeed_GET() == (char)35650);
            assert(pack.lon_GET() == -1233709412);
            assert(pack.yawspeed_GET() == -1.7382589E38F);
            assert(pack.vz_GET() == (short)14034);
            assert(pack.xacc_GET() == (short)29218);
            assert(pack.vy_GET() == (short) -9393);
            assert(pack.time_usec_GET() == 994061894491525162L);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {2.364699E38F, 3.313558E38F, 1.8959664E38F, 2.3829661E38F}));
            assert(pack.yacc_GET() == (short) -14636);
            assert(pack.vx_GET() == (short)7791);
            assert(pack.pitchspeed_GET() == 1.165545E36F);
            assert(pack.lat_GET() == -79452000);
            assert(pack.rollspeed_GET() == 2.4785725E38F);
            assert(pack.zacc_GET() == (short) -31554);
            assert(pack.true_airspeed_GET() == (char)35261);
            assert(pack.alt_GET() == -109667471);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.lon_SET(-1233709412) ;
        p115.yawspeed_SET(-1.7382589E38F) ;
        p115.attitude_quaternion_SET(new float[] {2.364699E38F, 3.313558E38F, 1.8959664E38F, 2.3829661E38F}, 0) ;
        p115.rollspeed_SET(2.4785725E38F) ;
        p115.pitchspeed_SET(1.165545E36F) ;
        p115.true_airspeed_SET((char)35261) ;
        p115.vy_SET((short) -9393) ;
        p115.alt_SET(-109667471) ;
        p115.yacc_SET((short) -14636) ;
        p115.time_usec_SET(994061894491525162L) ;
        p115.vz_SET((short)14034) ;
        p115.lat_SET(-79452000) ;
        p115.ind_airspeed_SET((char)35650) ;
        p115.vx_SET((short)7791) ;
        p115.zacc_SET((short) -31554) ;
        p115.xacc_SET((short)29218) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short) -24794);
            assert(pack.xmag_GET() == (short)3296);
            assert(pack.zacc_GET() == (short)28643);
            assert(pack.zmag_GET() == (short)27672);
            assert(pack.time_boot_ms_GET() == 2851408799L);
            assert(pack.yacc_GET() == (short) -30468);
            assert(pack.xgyro_GET() == (short) -31410);
            assert(pack.ymag_GET() == (short) -18971);
            assert(pack.ygyro_GET() == (short)23109);
            assert(pack.zgyro_GET() == (short)17518);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.zgyro_SET((short)17518) ;
        p116.ygyro_SET((short)23109) ;
        p116.time_boot_ms_SET(2851408799L) ;
        p116.zacc_SET((short)28643) ;
        p116.ymag_SET((short) -18971) ;
        p116.xgyro_SET((short) -31410) ;
        p116.xmag_SET((short)3296) ;
        p116.xacc_SET((short) -24794) ;
        p116.yacc_SET((short) -30468) ;
        p116.zmag_SET((short)27672) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)159);
            assert(pack.end_GET() == (char)20294);
            assert(pack.target_system_GET() == (char)211);
            assert(pack.start_GET() == (char)37841);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.start_SET((char)37841) ;
        p117.end_SET((char)20294) ;
        p117.target_component_SET((char)159) ;
        p117.target_system_SET((char)211) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.num_logs_GET() == (char)20266);
            assert(pack.last_log_num_GET() == (char)10307);
            assert(pack.id_GET() == (char)17307);
            assert(pack.time_utc_GET() == 278583288L);
            assert(pack.size_GET() == 2245697311L);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.last_log_num_SET((char)10307) ;
        p118.size_SET(2245697311L) ;
        p118.id_SET((char)17307) ;
        p118.time_utc_SET(278583288L) ;
        p118.num_logs_SET((char)20266) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.ofs_GET() == 1581549284L);
            assert(pack.target_system_GET() == (char)201);
            assert(pack.target_component_GET() == (char)45);
            assert(pack.count_GET() == 584987483L);
            assert(pack.id_GET() == (char)45763);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.ofs_SET(1581549284L) ;
        p119.target_system_SET((char)201) ;
        p119.target_component_SET((char)45) ;
        p119.id_SET((char)45763) ;
        p119.count_SET(584987483L) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)72, (char)150, (char)147, (char)33, (char)252, (char)213, (char)80, (char)114, (char)2, (char)156, (char)149, (char)121, (char)80, (char)171, (char)199, (char)66, (char)29, (char)214, (char)23, (char)65, (char)185, (char)115, (char)99, (char)15, (char)102, (char)210, (char)80, (char)209, (char)109, (char)72, (char)5, (char)231, (char)1, (char)215, (char)196, (char)119, (char)111, (char)232, (char)224, (char)136, (char)16, (char)184, (char)115, (char)157, (char)43, (char)122, (char)41, (char)226, (char)65, (char)32, (char)175, (char)5, (char)168, (char)146, (char)27, (char)190, (char)30, (char)211, (char)181, (char)111, (char)72, (char)30, (char)57, (char)164, (char)69, (char)73, (char)237, (char)62, (char)147, (char)185, (char)7, (char)69, (char)164, (char)0, (char)174, (char)81, (char)115, (char)134, (char)221, (char)132, (char)104, (char)14, (char)35, (char)98, (char)87, (char)34, (char)106, (char)247, (char)255, (char)65}));
            assert(pack.id_GET() == (char)37955);
            assert(pack.ofs_GET() == 3785793041L);
            assert(pack.count_GET() == (char)185);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.ofs_SET(3785793041L) ;
        p120.count_SET((char)185) ;
        p120.id_SET((char)37955) ;
        p120.data__SET(new char[] {(char)72, (char)150, (char)147, (char)33, (char)252, (char)213, (char)80, (char)114, (char)2, (char)156, (char)149, (char)121, (char)80, (char)171, (char)199, (char)66, (char)29, (char)214, (char)23, (char)65, (char)185, (char)115, (char)99, (char)15, (char)102, (char)210, (char)80, (char)209, (char)109, (char)72, (char)5, (char)231, (char)1, (char)215, (char)196, (char)119, (char)111, (char)232, (char)224, (char)136, (char)16, (char)184, (char)115, (char)157, (char)43, (char)122, (char)41, (char)226, (char)65, (char)32, (char)175, (char)5, (char)168, (char)146, (char)27, (char)190, (char)30, (char)211, (char)181, (char)111, (char)72, (char)30, (char)57, (char)164, (char)69, (char)73, (char)237, (char)62, (char)147, (char)185, (char)7, (char)69, (char)164, (char)0, (char)174, (char)81, (char)115, (char)134, (char)221, (char)132, (char)104, (char)14, (char)35, (char)98, (char)87, (char)34, (char)106, (char)247, (char)255, (char)65}, 0) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)152);
            assert(pack.target_system_GET() == (char)30);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_component_SET((char)152) ;
        p121.target_system_SET((char)30) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)114);
            assert(pack.target_system_GET() == (char)172);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)172) ;
        p122.target_component_SET((char)114) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)61);
            assert(pack.len_GET() == (char)102);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)11, (char)7, (char)152, (char)245, (char)150, (char)240, (char)210, (char)111, (char)156, (char)37, (char)103, (char)171, (char)218, (char)144, (char)135, (char)175, (char)237, (char)252, (char)106, (char)32, (char)165, (char)137, (char)116, (char)124, (char)146, (char)178, (char)105, (char)18, (char)168, (char)52, (char)43, (char)107, (char)117, (char)245, (char)195, (char)89, (char)144, (char)193, (char)11, (char)26, (char)132, (char)128, (char)142, (char)21, (char)232, (char)131, (char)232, (char)174, (char)118, (char)117, (char)147, (char)102, (char)149, (char)216, (char)55, (char)18, (char)227, (char)218, (char)53, (char)129, (char)26, (char)150, (char)117, (char)20, (char)48, (char)255, (char)135, (char)56, (char)179, (char)55, (char)114, (char)25, (char)205, (char)128, (char)59, (char)188, (char)200, (char)63, (char)156, (char)120, (char)167, (char)51, (char)138, (char)195, (char)21, (char)96, (char)65, (char)169, (char)201, (char)66, (char)177, (char)173, (char)14, (char)232, (char)14, (char)131, (char)255, (char)23, (char)69, (char)230, (char)192, (char)188, (char)43, (char)32, (char)203, (char)210, (char)158, (char)27, (char)243, (char)37}));
            assert(pack.target_component_GET() == (char)200);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_component_SET((char)200) ;
        p123.data__SET(new char[] {(char)11, (char)7, (char)152, (char)245, (char)150, (char)240, (char)210, (char)111, (char)156, (char)37, (char)103, (char)171, (char)218, (char)144, (char)135, (char)175, (char)237, (char)252, (char)106, (char)32, (char)165, (char)137, (char)116, (char)124, (char)146, (char)178, (char)105, (char)18, (char)168, (char)52, (char)43, (char)107, (char)117, (char)245, (char)195, (char)89, (char)144, (char)193, (char)11, (char)26, (char)132, (char)128, (char)142, (char)21, (char)232, (char)131, (char)232, (char)174, (char)118, (char)117, (char)147, (char)102, (char)149, (char)216, (char)55, (char)18, (char)227, (char)218, (char)53, (char)129, (char)26, (char)150, (char)117, (char)20, (char)48, (char)255, (char)135, (char)56, (char)179, (char)55, (char)114, (char)25, (char)205, (char)128, (char)59, (char)188, (char)200, (char)63, (char)156, (char)120, (char)167, (char)51, (char)138, (char)195, (char)21, (char)96, (char)65, (char)169, (char)201, (char)66, (char)177, (char)173, (char)14, (char)232, (char)14, (char)131, (char)255, (char)23, (char)69, (char)230, (char)192, (char)188, (char)43, (char)32, (char)203, (char)210, (char)158, (char)27, (char)243, (char)37}, 0) ;
        p123.len_SET((char)102) ;
        p123.target_system_SET((char)61) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.epv_GET() == (char)50831);
            assert(pack.dgps_numch_GET() == (char)37);
            assert(pack.time_usec_GET() == 4066409803050426440L);
            assert(pack.satellites_visible_GET() == (char)214);
            assert(pack.vel_GET() == (char)64612);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
            assert(pack.lon_GET() == -1052460879);
            assert(pack.alt_GET() == 1186122548);
            assert(pack.eph_GET() == (char)28028);
            assert(pack.dgps_age_GET() == 1516308271L);
            assert(pack.cog_GET() == (char)16331);
            assert(pack.lat_GET() == 458724901);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.dgps_numch_SET((char)37) ;
        p124.lon_SET(-1052460879) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP) ;
        p124.vel_SET((char)64612) ;
        p124.time_usec_SET(4066409803050426440L) ;
        p124.eph_SET((char)28028) ;
        p124.cog_SET((char)16331) ;
        p124.epv_SET((char)50831) ;
        p124.satellites_visible_SET((char)214) ;
        p124.lat_SET(458724901) ;
        p124.alt_SET(1186122548) ;
        p124.dgps_age_SET(1516308271L) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT));
            assert(pack.Vservo_GET() == (char)56527);
            assert(pack.Vcc_GET() == (char)62881);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT)) ;
        p125.Vservo_SET((char)56527) ;
        p125.Vcc_SET((char)62881) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)121, (char)240, (char)48, (char)13, (char)28, (char)162, (char)109, (char)18, (char)25, (char)245, (char)81, (char)172, (char)51, (char)164, (char)136, (char)251, (char)92, (char)26, (char)212, (char)64, (char)128, (char)188, (char)157, (char)180, (char)96, (char)10, (char)149, (char)153, (char)236, (char)170, (char)206, (char)39, (char)49, (char)135, (char)31, (char)94, (char)12, (char)54, (char)80, (char)35, (char)4, (char)123, (char)242, (char)221, (char)36, (char)157, (char)37, (char)127, (char)155, (char)250, (char)92, (char)166, (char)232, (char)45, (char)166, (char)207, (char)252, (char)29, (char)199, (char)248, (char)211, (char)92, (char)73, (char)108, (char)203, (char)82, (char)185, (char)201, (char)221, (char)174}));
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
            assert(pack.count_GET() == (char)86);
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE));
            assert(pack.timeout_GET() == (char)14631);
            assert(pack.baudrate_GET() == 2335509956L);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE)) ;
        p126.baudrate_SET(2335509956L) ;
        p126.timeout_SET((char)14631) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1) ;
        p126.count_SET((char)86) ;
        p126.data__SET(new char[] {(char)121, (char)240, (char)48, (char)13, (char)28, (char)162, (char)109, (char)18, (char)25, (char)245, (char)81, (char)172, (char)51, (char)164, (char)136, (char)251, (char)92, (char)26, (char)212, (char)64, (char)128, (char)188, (char)157, (char)180, (char)96, (char)10, (char)149, (char)153, (char)236, (char)170, (char)206, (char)39, (char)49, (char)135, (char)31, (char)94, (char)12, (char)54, (char)80, (char)35, (char)4, (char)123, (char)242, (char)221, (char)36, (char)157, (char)37, (char)127, (char)155, (char)250, (char)92, (char)166, (char)232, (char)45, (char)166, (char)207, (char)252, (char)29, (char)199, (char)248, (char)211, (char)92, (char)73, (char)108, (char)203, (char)82, (char)185, (char)201, (char)221, (char)174}, 0) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_b_mm_GET() == -614590179);
            assert(pack.baseline_c_mm_GET() == 574984947);
            assert(pack.rtk_health_GET() == (char)42);
            assert(pack.baseline_coords_type_GET() == (char)213);
            assert(pack.nsats_GET() == (char)45);
            assert(pack.time_last_baseline_ms_GET() == 1426726602L);
            assert(pack.rtk_rate_GET() == (char)252);
            assert(pack.wn_GET() == (char)7788);
            assert(pack.rtk_receiver_id_GET() == (char)250);
            assert(pack.tow_GET() == 4258233711L);
            assert(pack.accuracy_GET() == 1335395958L);
            assert(pack.iar_num_hypotheses_GET() == 1240183799);
            assert(pack.baseline_a_mm_GET() == 1907969922);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.baseline_b_mm_SET(-614590179) ;
        p127.wn_SET((char)7788) ;
        p127.nsats_SET((char)45) ;
        p127.rtk_rate_SET((char)252) ;
        p127.baseline_coords_type_SET((char)213) ;
        p127.accuracy_SET(1335395958L) ;
        p127.iar_num_hypotheses_SET(1240183799) ;
        p127.baseline_c_mm_SET(574984947) ;
        p127.rtk_health_SET((char)42) ;
        p127.baseline_a_mm_SET(1907969922) ;
        p127.tow_SET(4258233711L) ;
        p127.time_last_baseline_ms_SET(1426726602L) ;
        p127.rtk_receiver_id_SET((char)250) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.rtk_receiver_id_GET() == (char)164);
            assert(pack.wn_GET() == (char)35681);
            assert(pack.rtk_rate_GET() == (char)204);
            assert(pack.rtk_health_GET() == (char)51);
            assert(pack.baseline_c_mm_GET() == -1519909068);
            assert(pack.baseline_coords_type_GET() == (char)52);
            assert(pack.time_last_baseline_ms_GET() == 1570804730L);
            assert(pack.baseline_b_mm_GET() == -1346455957);
            assert(pack.tow_GET() == 1390736223L);
            assert(pack.nsats_GET() == (char)118);
            assert(pack.iar_num_hypotheses_GET() == 2013062998);
            assert(pack.baseline_a_mm_GET() == 1509083616);
            assert(pack.accuracy_GET() == 1081567432L);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.tow_SET(1390736223L) ;
        p128.time_last_baseline_ms_SET(1570804730L) ;
        p128.rtk_receiver_id_SET((char)164) ;
        p128.rtk_health_SET((char)51) ;
        p128.wn_SET((char)35681) ;
        p128.nsats_SET((char)118) ;
        p128.baseline_b_mm_SET(-1346455957) ;
        p128.baseline_coords_type_SET((char)52) ;
        p128.iar_num_hypotheses_SET(2013062998) ;
        p128.baseline_a_mm_SET(1509083616) ;
        p128.accuracy_SET(1081567432L) ;
        p128.baseline_c_mm_SET(-1519909068) ;
        p128.rtk_rate_SET((char)204) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.xmag_GET() == (short)6667);
            assert(pack.yacc_GET() == (short) -11511);
            assert(pack.zacc_GET() == (short)29316);
            assert(pack.zgyro_GET() == (short) -7181);
            assert(pack.zmag_GET() == (short)9227);
            assert(pack.xgyro_GET() == (short)18309);
            assert(pack.ygyro_GET() == (short) -7907);
            assert(pack.time_boot_ms_GET() == 837288047L);
            assert(pack.xacc_GET() == (short) -29202);
            assert(pack.ymag_GET() == (short) -17532);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.time_boot_ms_SET(837288047L) ;
        p129.ymag_SET((short) -17532) ;
        p129.zmag_SET((short)9227) ;
        p129.yacc_SET((short) -11511) ;
        p129.xacc_SET((short) -29202) ;
        p129.zgyro_SET((short) -7181) ;
        p129.ygyro_SET((short) -7907) ;
        p129.xgyro_SET((short)18309) ;
        p129.xmag_SET((short)6667) ;
        p129.zacc_SET((short)29316) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.packets_GET() == (char)42002);
            assert(pack.jpg_quality_GET() == (char)38);
            assert(pack.payload_GET() == (char)107);
            assert(pack.height_GET() == (char)2262);
            assert(pack.type_GET() == (char)222);
            assert(pack.width_GET() == (char)21302);
            assert(pack.size_GET() == 3864829829L);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.jpg_quality_SET((char)38) ;
        p130.payload_SET((char)107) ;
        p130.width_SET((char)21302) ;
        p130.height_SET((char)2262) ;
        p130.size_SET(3864829829L) ;
        p130.packets_SET((char)42002) ;
        p130.type_SET((char)222) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)32517);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)242, (char)106, (char)235, (char)222, (char)140, (char)33, (char)40, (char)120, (char)115, (char)56, (char)58, (char)14, (char)121, (char)222, (char)239, (char)195, (char)90, (char)215, (char)63, (char)18, (char)250, (char)222, (char)252, (char)210, (char)201, (char)21, (char)215, (char)11, (char)96, (char)18, (char)88, (char)203, (char)17, (char)87, (char)109, (char)247, (char)36, (char)155, (char)118, (char)145, (char)80, (char)0, (char)149, (char)73, (char)12, (char)193, (char)155, (char)221, (char)60, (char)238, (char)71, (char)86, (char)156, (char)248, (char)151, (char)250, (char)91, (char)103, (char)85, (char)166, (char)77, (char)98, (char)220, (char)224, (char)30, (char)60, (char)23, (char)121, (char)156, (char)182, (char)246, (char)7, (char)73, (char)200, (char)185, (char)16, (char)194, (char)184, (char)27, (char)59, (char)252, (char)189, (char)186, (char)150, (char)64, (char)49, (char)250, (char)134, (char)10, (char)195, (char)13, (char)194, (char)12, (char)127, (char)215, (char)128, (char)235, (char)140, (char)184, (char)55, (char)245, (char)2, (char)4, (char)47, (char)177, (char)99, (char)204, (char)173, (char)46, (char)193, (char)107, (char)175, (char)151, (char)67, (char)30, (char)95, (char)186, (char)232, (char)0, (char)72, (char)91, (char)105, (char)161, (char)132, (char)102, (char)173, (char)177, (char)184, (char)66, (char)10, (char)74, (char)144, (char)244, (char)148, (char)92, (char)177, (char)58, (char)12, (char)74, (char)17, (char)6, (char)237, (char)121, (char)13, (char)108, (char)13, (char)164, (char)221, (char)195, (char)219, (char)73, (char)254, (char)144, (char)65, (char)169, (char)118, (char)162, (char)82, (char)128, (char)243, (char)119, (char)78, (char)1, (char)6, (char)199, (char)183, (char)160, (char)168, (char)145, (char)230, (char)58, (char)171, (char)7, (char)23, (char)231, (char)111, (char)167, (char)61, (char)151, (char)86, (char)75, (char)218, (char)107, (char)247, (char)50, (char)208, (char)184, (char)176, (char)230, (char)86, (char)142, (char)96, (char)147, (char)233, (char)229, (char)161, (char)91, (char)68, (char)150, (char)120, (char)174, (char)196, (char)58, (char)128, (char)225, (char)182, (char)51, (char)72, (char)144, (char)3, (char)220, (char)243, (char)31, (char)177, (char)34, (char)201, (char)37, (char)202, (char)10, (char)29, (char)43, (char)67, (char)156, (char)228, (char)172, (char)11, (char)32, (char)100, (char)22, (char)228, (char)63, (char)75, (char)118, (char)118, (char)203, (char)72, (char)82, (char)239, (char)240, (char)163, (char)178, (char)100, (char)179, (char)213, (char)215, (char)52, (char)239, (char)213, (char)196, (char)30, (char)199, (char)11, (char)36}));
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)32517) ;
        p131.data__SET(new char[] {(char)242, (char)106, (char)235, (char)222, (char)140, (char)33, (char)40, (char)120, (char)115, (char)56, (char)58, (char)14, (char)121, (char)222, (char)239, (char)195, (char)90, (char)215, (char)63, (char)18, (char)250, (char)222, (char)252, (char)210, (char)201, (char)21, (char)215, (char)11, (char)96, (char)18, (char)88, (char)203, (char)17, (char)87, (char)109, (char)247, (char)36, (char)155, (char)118, (char)145, (char)80, (char)0, (char)149, (char)73, (char)12, (char)193, (char)155, (char)221, (char)60, (char)238, (char)71, (char)86, (char)156, (char)248, (char)151, (char)250, (char)91, (char)103, (char)85, (char)166, (char)77, (char)98, (char)220, (char)224, (char)30, (char)60, (char)23, (char)121, (char)156, (char)182, (char)246, (char)7, (char)73, (char)200, (char)185, (char)16, (char)194, (char)184, (char)27, (char)59, (char)252, (char)189, (char)186, (char)150, (char)64, (char)49, (char)250, (char)134, (char)10, (char)195, (char)13, (char)194, (char)12, (char)127, (char)215, (char)128, (char)235, (char)140, (char)184, (char)55, (char)245, (char)2, (char)4, (char)47, (char)177, (char)99, (char)204, (char)173, (char)46, (char)193, (char)107, (char)175, (char)151, (char)67, (char)30, (char)95, (char)186, (char)232, (char)0, (char)72, (char)91, (char)105, (char)161, (char)132, (char)102, (char)173, (char)177, (char)184, (char)66, (char)10, (char)74, (char)144, (char)244, (char)148, (char)92, (char)177, (char)58, (char)12, (char)74, (char)17, (char)6, (char)237, (char)121, (char)13, (char)108, (char)13, (char)164, (char)221, (char)195, (char)219, (char)73, (char)254, (char)144, (char)65, (char)169, (char)118, (char)162, (char)82, (char)128, (char)243, (char)119, (char)78, (char)1, (char)6, (char)199, (char)183, (char)160, (char)168, (char)145, (char)230, (char)58, (char)171, (char)7, (char)23, (char)231, (char)111, (char)167, (char)61, (char)151, (char)86, (char)75, (char)218, (char)107, (char)247, (char)50, (char)208, (char)184, (char)176, (char)230, (char)86, (char)142, (char)96, (char)147, (char)233, (char)229, (char)161, (char)91, (char)68, (char)150, (char)120, (char)174, (char)196, (char)58, (char)128, (char)225, (char)182, (char)51, (char)72, (char)144, (char)3, (char)220, (char)243, (char)31, (char)177, (char)34, (char)201, (char)37, (char)202, (char)10, (char)29, (char)43, (char)67, (char)156, (char)228, (char)172, (char)11, (char)32, (char)100, (char)22, (char)228, (char)63, (char)75, (char)118, (char)118, (char)203, (char)72, (char)82, (char)239, (char)240, (char)163, (char)178, (char)100, (char)179, (char)213, (char)215, (char)52, (char)239, (char)213, (char)196, (char)30, (char)199, (char)11, (char)36}, 0) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1489509841L);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_90);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            assert(pack.max_distance_GET() == (char)6233);
            assert(pack.covariance_GET() == (char)137);
            assert(pack.current_distance_GET() == (char)8257);
            assert(pack.min_distance_GET() == (char)51015);
            assert(pack.id_GET() == (char)35);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.min_distance_SET((char)51015) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_90) ;
        p132.current_distance_SET((char)8257) ;
        p132.id_SET((char)35) ;
        p132.covariance_SET((char)137) ;
        p132.time_boot_ms_SET(1489509841L) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        p132.max_distance_SET((char)6233) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 1612748693);
            assert(pack.lat_GET() == -1533519392);
            assert(pack.mask_GET() == 5955001697797805299L);
            assert(pack.grid_spacing_GET() == (char)35887);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(-1533519392) ;
        p133.grid_spacing_SET((char)35887) ;
        p133.mask_SET(5955001697797805299L) ;
        p133.lon_SET(1612748693) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.grid_spacing_GET() == (char)14805);
            assert(pack.lon_GET() == 208324930);
            assert(pack.lat_GET() == -1835472252);
            assert(pack.gridbit_GET() == (char)190);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short)3788, (short)15750, (short)16022, (short)19576, (short)4720, (short)19909, (short)19702, (short)32218, (short)1355, (short) -8313, (short)4042, (short)15476, (short)9965, (short) -29269, (short)9106, (short) -10145}));
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(-1835472252) ;
        p134.grid_spacing_SET((char)14805) ;
        p134.gridbit_SET((char)190) ;
        p134.lon_SET(208324930) ;
        p134.data__SET(new short[] {(short)3788, (short)15750, (short)16022, (short)19576, (short)4720, (short)19909, (short)19702, (short)32218, (short)1355, (short) -8313, (short)4042, (short)15476, (short)9965, (short) -29269, (short)9106, (short) -10145}, 0) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 924329342);
            assert(pack.lon_GET() == -956748335);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lon_SET(-956748335) ;
        p135.lat_SET(924329342) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.spacing_GET() == (char)15630);
            assert(pack.terrain_height_GET() == -3.2523696E38F);
            assert(pack.loaded_GET() == (char)32715);
            assert(pack.pending_GET() == (char)30833);
            assert(pack.lon_GET() == 1527345410);
            assert(pack.current_height_GET() == 4.9861885E37F);
            assert(pack.lat_GET() == -264275995);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.loaded_SET((char)32715) ;
        p136.terrain_height_SET(-3.2523696E38F) ;
        p136.lon_SET(1527345410) ;
        p136.pending_SET((char)30833) ;
        p136.current_height_SET(4.9861885E37F) ;
        p136.spacing_SET((char)15630) ;
        p136.lat_SET(-264275995) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == 2.497635E38F);
            assert(pack.press_diff_GET() == -2.6773101E38F);
            assert(pack.time_boot_ms_GET() == 2170116681L);
            assert(pack.temperature_GET() == (short)418);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(2170116681L) ;
        p137.press_abs_SET(2.497635E38F) ;
        p137.temperature_SET((short)418) ;
        p137.press_diff_SET(-2.6773101E38F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -1.4658624E38F);
            assert(pack.z_GET() == -3.238074E38F);
            assert(pack.y_GET() == 2.8664405E38F);
            assert(pack.time_usec_GET() == 1508274296917154446L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.210047E38F, 1.196269E38F, -2.0407503E38F, -2.6389313E38F}));
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(1508274296917154446L) ;
        p138.z_SET(-3.238074E38F) ;
        p138.x_SET(-1.4658624E38F) ;
        p138.q_SET(new float[] {1.210047E38F, 1.196269E38F, -2.0407503E38F, -2.6389313E38F}, 0) ;
        p138.y_SET(2.8664405E38F) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-3.1254313E38F, 9.231657E37F, 1.8138423E38F, -2.1526877E37F, 2.4560951E38F, -2.5092322E38F, 2.6058797E38F, 1.4191441E38F}));
            assert(pack.group_mlx_GET() == (char)140);
            assert(pack.target_system_GET() == (char)34);
            assert(pack.target_component_GET() == (char)217);
            assert(pack.time_usec_GET() == 5562160568582511312L);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.target_system_SET((char)34) ;
        p139.group_mlx_SET((char)140) ;
        p139.target_component_SET((char)217) ;
        p139.controls_SET(new float[] {-3.1254313E38F, 9.231657E37F, 1.8138423E38F, -2.1526877E37F, 2.4560951E38F, -2.5092322E38F, 2.6058797E38F, 1.4191441E38F}, 0) ;
        p139.time_usec_SET(5562160568582511312L) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-9.275694E37F, -3.2398318E38F, -6.8770537E37F, -6.0225776E37F, 1.8067007E38F, 2.0134056E38F, 3.1076166E38F, 1.1100413E37F}));
            assert(pack.time_usec_GET() == 6600828431664948645L);
            assert(pack.group_mlx_GET() == (char)231);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.controls_SET(new float[] {-9.275694E37F, -3.2398318E38F, -6.8770537E37F, -6.0225776E37F, 1.8067007E38F, 2.0134056E38F, 3.1076166E38F, 1.1100413E37F}, 0) ;
        p140.group_mlx_SET((char)231) ;
        p140.time_usec_SET(6600828431664948645L) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_relative_GET() == -4.7562747E37F);
            assert(pack.time_usec_GET() == 4923966254732258694L);
            assert(pack.bottom_clearance_GET() == 2.4523693E38F);
            assert(pack.altitude_amsl_GET() == 7.15984E37F);
            assert(pack.altitude_local_GET() == 1.7676491E38F);
            assert(pack.altitude_monotonic_GET() == 2.0065035E38F);
            assert(pack.altitude_terrain_GET() == -2.3180646E38F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_amsl_SET(7.15984E37F) ;
        p141.altitude_relative_SET(-4.7562747E37F) ;
        p141.time_usec_SET(4923966254732258694L) ;
        p141.bottom_clearance_SET(2.4523693E38F) ;
        p141.altitude_monotonic_SET(2.0065035E38F) ;
        p141.altitude_local_SET(1.7676491E38F) ;
        p141.altitude_terrain_SET(-2.3180646E38F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)93, (char)113, (char)67, (char)199, (char)60, (char)222, (char)187, (char)68, (char)96, (char)17, (char)250, (char)125, (char)7, (char)28, (char)43, (char)70, (char)9, (char)192, (char)10, (char)101, (char)44, (char)130, (char)27, (char)201, (char)217, (char)166, (char)64, (char)87, (char)141, (char)94, (char)61, (char)136, (char)194, (char)234, (char)128, (char)201, (char)141, (char)114, (char)244, (char)58, (char)185, (char)14, (char)36, (char)255, (char)136, (char)24, (char)220, (char)165, (char)10, (char)51, (char)12, (char)84, (char)134, (char)218, (char)122, (char)32, (char)112, (char)78, (char)221, (char)153, (char)73, (char)108, (char)116, (char)6, (char)38, (char)45, (char)134, (char)224, (char)88, (char)76, (char)214, (char)154, (char)197, (char)16, (char)255, (char)138, (char)101, (char)13, (char)85, (char)81, (char)245, (char)16, (char)82, (char)78, (char)155, (char)154, (char)255, (char)140, (char)198, (char)52, (char)167, (char)101, (char)200, (char)152, (char)237, (char)138, (char)224, (char)86, (char)200, (char)46, (char)53, (char)171, (char)198, (char)212, (char)11, (char)56, (char)148, (char)189, (char)129, (char)2, (char)35, (char)249, (char)105, (char)41, (char)173, (char)164, (char)50, (char)251, (char)78, (char)253}));
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)235, (char)103, (char)228, (char)233, (char)238, (char)214, (char)106, (char)251, (char)144, (char)160, (char)22, (char)128, (char)247, (char)4, (char)183, (char)174, (char)28, (char)183, (char)24, (char)185, (char)35, (char)183, (char)18, (char)5, (char)208, (char)213, (char)24, (char)149, (char)53, (char)1, (char)52, (char)186, (char)237, (char)254, (char)211, (char)132, (char)182, (char)99, (char)11, (char)45, (char)79, (char)21, (char)175, (char)183, (char)22, (char)10, (char)185, (char)70, (char)225, (char)22, (char)240, (char)6, (char)95, (char)78, (char)17, (char)197, (char)139, (char)78, (char)126, (char)14, (char)67, (char)121, (char)3, (char)120, (char)112, (char)83, (char)16, (char)55, (char)199, (char)201, (char)147, (char)147, (char)145, (char)105, (char)52, (char)83, (char)96, (char)63, (char)13, (char)10, (char)32, (char)154, (char)75, (char)145, (char)7, (char)217, (char)194, (char)147, (char)35, (char)163, (char)7, (char)228, (char)211, (char)8, (char)162, (char)240, (char)91, (char)126, (char)185, (char)21, (char)16, (char)140, (char)111, (char)182, (char)235, (char)2, (char)8, (char)25, (char)125, (char)136, (char)240, (char)56, (char)186, (char)102, (char)248, (char)23, (char)83, (char)254, (char)120, (char)246}));
            assert(pack.request_id_GET() == (char)122);
            assert(pack.uri_type_GET() == (char)251);
            assert(pack.transfer_type_GET() == (char)201);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.request_id_SET((char)122) ;
        p142.uri_type_SET((char)251) ;
        p142.uri_SET(new char[] {(char)235, (char)103, (char)228, (char)233, (char)238, (char)214, (char)106, (char)251, (char)144, (char)160, (char)22, (char)128, (char)247, (char)4, (char)183, (char)174, (char)28, (char)183, (char)24, (char)185, (char)35, (char)183, (char)18, (char)5, (char)208, (char)213, (char)24, (char)149, (char)53, (char)1, (char)52, (char)186, (char)237, (char)254, (char)211, (char)132, (char)182, (char)99, (char)11, (char)45, (char)79, (char)21, (char)175, (char)183, (char)22, (char)10, (char)185, (char)70, (char)225, (char)22, (char)240, (char)6, (char)95, (char)78, (char)17, (char)197, (char)139, (char)78, (char)126, (char)14, (char)67, (char)121, (char)3, (char)120, (char)112, (char)83, (char)16, (char)55, (char)199, (char)201, (char)147, (char)147, (char)145, (char)105, (char)52, (char)83, (char)96, (char)63, (char)13, (char)10, (char)32, (char)154, (char)75, (char)145, (char)7, (char)217, (char)194, (char)147, (char)35, (char)163, (char)7, (char)228, (char)211, (char)8, (char)162, (char)240, (char)91, (char)126, (char)185, (char)21, (char)16, (char)140, (char)111, (char)182, (char)235, (char)2, (char)8, (char)25, (char)125, (char)136, (char)240, (char)56, (char)186, (char)102, (char)248, (char)23, (char)83, (char)254, (char)120, (char)246}, 0) ;
        p142.storage_SET(new char[] {(char)93, (char)113, (char)67, (char)199, (char)60, (char)222, (char)187, (char)68, (char)96, (char)17, (char)250, (char)125, (char)7, (char)28, (char)43, (char)70, (char)9, (char)192, (char)10, (char)101, (char)44, (char)130, (char)27, (char)201, (char)217, (char)166, (char)64, (char)87, (char)141, (char)94, (char)61, (char)136, (char)194, (char)234, (char)128, (char)201, (char)141, (char)114, (char)244, (char)58, (char)185, (char)14, (char)36, (char)255, (char)136, (char)24, (char)220, (char)165, (char)10, (char)51, (char)12, (char)84, (char)134, (char)218, (char)122, (char)32, (char)112, (char)78, (char)221, (char)153, (char)73, (char)108, (char)116, (char)6, (char)38, (char)45, (char)134, (char)224, (char)88, (char)76, (char)214, (char)154, (char)197, (char)16, (char)255, (char)138, (char)101, (char)13, (char)85, (char)81, (char)245, (char)16, (char)82, (char)78, (char)155, (char)154, (char)255, (char)140, (char)198, (char)52, (char)167, (char)101, (char)200, (char)152, (char)237, (char)138, (char)224, (char)86, (char)200, (char)46, (char)53, (char)171, (char)198, (char)212, (char)11, (char)56, (char)148, (char)189, (char)129, (char)2, (char)35, (char)249, (char)105, (char)41, (char)173, (char)164, (char)50, (char)251, (char)78, (char)253}, 0) ;
        p142.transfer_type_SET((char)201) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 4226007897L);
            assert(pack.press_diff_GET() == 1.6829895E38F);
            assert(pack.temperature_GET() == (short) -1839);
            assert(pack.press_abs_GET() == 1.5110974E38F);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(4226007897L) ;
        p143.press_diff_SET(1.6829895E38F) ;
        p143.press_abs_SET(1.5110974E38F) ;
        p143.temperature_SET((short) -1839) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1149004511);
            assert(pack.timestamp_GET() == 3638777598372352471L);
            assert(Arrays.equals(pack.vel_GET(),  new float[] {-1.1041884E38F, 1.4383123E38F, 1.1570575E38F}));
            assert(pack.alt_GET() == 3.5536731E37F);
            assert(pack.est_capabilities_GET() == (char)152);
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-2.4116473E38F, -3.098363E38F, 8.952106E37F}));
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {4.956628E36F, -1.789785E38F, 2.2150048E38F}));
            assert(pack.custom_state_GET() == 7438401319363026703L);
            assert(pack.lon_GET() == -1518453981);
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-2.7433044E38F, -2.647783E38F, -1.9372774E38F, -3.0547323E38F}));
            assert(Arrays.equals(pack.acc_GET(),  new float[] {2.8822117E38F, -9.632996E36F, 1.5532747E38F}));
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.alt_SET(3.5536731E37F) ;
        p144.custom_state_SET(7438401319363026703L) ;
        p144.rates_SET(new float[] {-2.4116473E38F, -3.098363E38F, 8.952106E37F}, 0) ;
        p144.lat_SET(1149004511) ;
        p144.attitude_q_SET(new float[] {-2.7433044E38F, -2.647783E38F, -1.9372774E38F, -3.0547323E38F}, 0) ;
        p144.position_cov_SET(new float[] {4.956628E36F, -1.789785E38F, 2.2150048E38F}, 0) ;
        p144.acc_SET(new float[] {2.8822117E38F, -9.632996E36F, 1.5532747E38F}, 0) ;
        p144.timestamp_SET(3638777598372352471L) ;
        p144.lon_SET(-1518453981) ;
        p144.vel_SET(new float[] {-1.1041884E38F, 1.4383123E38F, 1.1570575E38F}, 0) ;
        p144.est_capabilities_SET((char)152) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.x_acc_GET() == 3.5749664E37F);
            assert(pack.y_acc_GET() == -1.688001E38F);
            assert(pack.x_pos_GET() == 1.9742046E38F);
            assert(pack.y_vel_GET() == 1.1524428E38F);
            assert(pack.airspeed_GET() == 2.2429905E35F);
            assert(pack.x_vel_GET() == 3.0800001E38F);
            assert(pack.time_usec_GET() == 5176194339293624059L);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {3.227287E38F, -2.5355756E38F, 2.3052705E37F}));
            assert(Arrays.equals(pack.q_GET(),  new float[] {6.928633E37F, 2.1587743E38F, 1.4824398E37F, 1.6325472E38F}));
            assert(pack.z_acc_GET() == 1.5808564E38F);
            assert(pack.yaw_rate_GET() == 1.933969E38F);
            assert(pack.roll_rate_GET() == 1.7713679E38F);
            assert(pack.pitch_rate_GET() == -7.5497857E37F);
            assert(pack.z_vel_GET() == -8.829198E37F);
            assert(pack.y_pos_GET() == -6.7750337E37F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {2.2821747E38F, -2.5663566E38F, 2.9263838E38F}));
            assert(pack.z_pos_GET() == -2.5927974E38F);
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.z_vel_SET(-8.829198E37F) ;
        p146.roll_rate_SET(1.7713679E38F) ;
        p146.x_vel_SET(3.0800001E38F) ;
        p146.vel_variance_SET(new float[] {2.2821747E38F, -2.5663566E38F, 2.9263838E38F}, 0) ;
        p146.q_SET(new float[] {6.928633E37F, 2.1587743E38F, 1.4824398E37F, 1.6325472E38F}, 0) ;
        p146.y_pos_SET(-6.7750337E37F) ;
        p146.pos_variance_SET(new float[] {3.227287E38F, -2.5355756E38F, 2.3052705E37F}, 0) ;
        p146.pitch_rate_SET(-7.5497857E37F) ;
        p146.y_acc_SET(-1.688001E38F) ;
        p146.airspeed_SET(2.2429905E35F) ;
        p146.time_usec_SET(5176194339293624059L) ;
        p146.y_vel_SET(1.1524428E38F) ;
        p146.x_pos_SET(1.9742046E38F) ;
        p146.yaw_rate_SET(1.933969E38F) ;
        p146.z_pos_SET(-2.5927974E38F) ;
        p146.x_acc_SET(3.5749664E37F) ;
        p146.z_acc_SET(1.5808564E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)70);
            assert(pack.battery_remaining_GET() == (byte)13);
            assert(pack.temperature_GET() == (short)2290);
            assert(pack.current_battery_GET() == (short) -8669);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
            assert(pack.energy_consumed_GET() == -1419668153);
            assert(pack.current_consumed_GET() == -97292514);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)35508, (char)53823, (char)32593, (char)14039, (char)37188, (char)17459, (char)36065, (char)12487, (char)5544, (char)13889}));
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.current_consumed_SET(-97292514) ;
        p147.current_battery_SET((short) -8669) ;
        p147.battery_remaining_SET((byte)13) ;
        p147.voltages_SET(new char[] {(char)35508, (char)53823, (char)32593, (char)14039, (char)37188, (char)17459, (char)36065, (char)12487, (char)5544, (char)13889}, 0) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL) ;
        p147.id_SET((char)70) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN) ;
        p147.energy_consumed_SET(-1419668153) ;
        p147.temperature_SET((short)2290) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE));
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)69, (char)189, (char)233, (char)92, (char)204, (char)56, (char)94, (char)51}));
            assert(pack.board_version_GET() == 1448558448L);
            assert(pack.middleware_sw_version_GET() == 481846488L);
            assert(pack.product_id_GET() == (char)21772);
            assert(pack.os_sw_version_GET() == 2403216786L);
            assert(pack.flight_sw_version_GET() == 1352229971L);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)194, (char)62, (char)219, (char)56, (char)139, (char)85, (char)56, (char)216}));
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)217, (char)240, (char)18, (char)250, (char)118, (char)127, (char)179, (char)81}));
            assert(pack.uid_GET() == 5357629312467609931L);
            assert(pack.vendor_id_GET() == (char)41832);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)1, (char)249, (char)198, (char)254, (char)251, (char)251, (char)26, (char)244, (char)181, (char)91, (char)8, (char)253, (char)232, (char)226, (char)8, (char)40, (char)108, (char)105}));
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.os_custom_version_SET(new char[] {(char)194, (char)62, (char)219, (char)56, (char)139, (char)85, (char)56, (char)216}, 0) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE)) ;
        p148.middleware_sw_version_SET(481846488L) ;
        p148.flight_custom_version_SET(new char[] {(char)69, (char)189, (char)233, (char)92, (char)204, (char)56, (char)94, (char)51}, 0) ;
        p148.uid2_SET(new char[] {(char)1, (char)249, (char)198, (char)254, (char)251, (char)251, (char)26, (char)244, (char)181, (char)91, (char)8, (char)253, (char)232, (char)226, (char)8, (char)40, (char)108, (char)105}, 0, PH) ;
        p148.os_sw_version_SET(2403216786L) ;
        p148.flight_sw_version_SET(1352229971L) ;
        p148.vendor_id_SET((char)41832) ;
        p148.board_version_SET(1448558448L) ;
        p148.uid_SET(5357629312467609931L) ;
        p148.middleware_custom_version_SET(new char[] {(char)217, (char)240, (char)18, (char)250, (char)118, (char)127, (char)179, (char)81}, 0) ;
        p148.product_id_SET((char)21772) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_num_GET() == (char)198);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {7.0295744E37F, 1.617485E37F, 2.8734224E38F, -2.8364418E38F}));
            assert(pack.y_TRY(ph) == -2.0275455E38F);
            assert(pack.size_y_GET() == -1.4191754E38F);
            assert(pack.angle_y_GET() == 1.4672686E38F);
            assert(pack.angle_x_GET() == -5.139697E37F);
            assert(pack.position_valid_TRY(ph) == (char)60);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL);
            assert(pack.size_x_GET() == -1.2876353E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.x_TRY(ph) == 1.6362849E38F);
            assert(pack.time_usec_GET() == 2852293266848371189L);
            assert(pack.distance_GET() == 2.2415168E38F);
            assert(pack.z_TRY(ph) == 9.923403E37F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.target_num_SET((char)198) ;
        p149.x_SET(1.6362849E38F, PH) ;
        p149.angle_y_SET(1.4672686E38F) ;
        p149.z_SET(9.923403E37F, PH) ;
        p149.distance_SET(2.2415168E38F) ;
        p149.angle_x_SET(-5.139697E37F) ;
        p149.position_valid_SET((char)60, PH) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL) ;
        p149.size_x_SET(-1.2876353E38F) ;
        p149.time_usec_SET(2852293266848371189L) ;
        p149.q_SET(new float[] {7.0295744E37F, 1.617485E37F, 2.8734224E38F, -2.8364418E38F}, 0, PH) ;
        p149.size_y_SET(-1.4191754E38F) ;
        p149.y_SET(-2.0275455E38F, PH) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_SENSOR_OFFSETS.add((src, ph, pack) ->
        {
            assert(pack.raw_temp_GET() == -642677332);
            assert(pack.gyro_cal_y_GET() == 2.0427567E38F);
            assert(pack.mag_ofs_x_GET() == (short) -9892);
            assert(pack.mag_ofs_z_GET() == (short) -11787);
            assert(pack.gyro_cal_x_GET() == -1.5362493E38F);
            assert(pack.accel_cal_x_GET() == -1.1739408E38F);
            assert(pack.mag_declination_GET() == 3.2470945E38F);
            assert(pack.accel_cal_z_GET() == 6.9537263E37F);
            assert(pack.accel_cal_y_GET() == 3.3820626E38F);
            assert(pack.gyro_cal_z_GET() == 2.369119E38F);
            assert(pack.raw_press_GET() == -2053915646);
            assert(pack.mag_ofs_y_GET() == (short) -28112);
        });
        GroundControl.SENSOR_OFFSETS p150 = CommunicationChannel.new_SENSOR_OFFSETS();
        PH.setPack(p150);
        p150.mag_ofs_x_SET((short) -9892) ;
        p150.accel_cal_x_SET(-1.1739408E38F) ;
        p150.accel_cal_y_SET(3.3820626E38F) ;
        p150.raw_temp_SET(-642677332) ;
        p150.mag_ofs_z_SET((short) -11787) ;
        p150.accel_cal_z_SET(6.9537263E37F) ;
        p150.raw_press_SET(-2053915646) ;
        p150.mag_ofs_y_SET((short) -28112) ;
        p150.gyro_cal_x_SET(-1.5362493E38F) ;
        p150.mag_declination_SET(3.2470945E38F) ;
        p150.gyro_cal_y_SET(2.0427567E38F) ;
        p150.gyro_cal_z_SET(2.369119E38F) ;
        CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_MAG_OFFSETS.add((src, ph, pack) ->
        {
            assert(pack.mag_ofs_x_GET() == (short)3019);
            assert(pack.mag_ofs_y_GET() == (short)27632);
            assert(pack.mag_ofs_z_GET() == (short)6655);
            assert(pack.target_component_GET() == (char)24);
            assert(pack.target_system_GET() == (char)179);
        });
        GroundControl.SET_MAG_OFFSETS p151 = CommunicationChannel.new_SET_MAG_OFFSETS();
        PH.setPack(p151);
        p151.mag_ofs_y_SET((short)27632) ;
        p151.mag_ofs_x_SET((short)3019) ;
        p151.mag_ofs_z_SET((short)6655) ;
        p151.target_component_SET((char)24) ;
        p151.target_system_SET((char)179) ;
        CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MEMINFO.add((src, ph, pack) ->
        {
            assert(pack.freemem32_TRY(ph) == 1285977614L);
            assert(pack.brkval_GET() == (char)44727);
            assert(pack.freemem_GET() == (char)44565);
        });
        GroundControl.MEMINFO p152 = CommunicationChannel.new_MEMINFO();
        PH.setPack(p152);
        p152.freemem32_SET(1285977614L, PH) ;
        p152.brkval_SET((char)44727) ;
        p152.freemem_SET((char)44565) ;
        CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AP_ADC.add((src, ph, pack) ->
        {
            assert(pack.adc3_GET() == (char)54114);
            assert(pack.adc5_GET() == (char)35522);
            assert(pack.adc4_GET() == (char)19414);
            assert(pack.adc6_GET() == (char)19470);
            assert(pack.adc1_GET() == (char)35052);
            assert(pack.adc2_GET() == (char)59303);
        });
        GroundControl.AP_ADC p153 = CommunicationChannel.new_AP_ADC();
        PH.setPack(p153);
        p153.adc2_SET((char)59303) ;
        p153.adc5_SET((char)35522) ;
        p153.adc6_SET((char)19470) ;
        p153.adc1_SET((char)35052) ;
        p153.adc4_SET((char)19414) ;
        p153.adc3_SET((char)54114) ;
        CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DIGICAM_CONFIGURE.add((src, ph, pack) ->
        {
            assert(pack.extra_value_GET() == 2.2104163E38F);
            assert(pack.extra_param_GET() == (char)71);
            assert(pack.mode_GET() == (char)39);
            assert(pack.engine_cut_off_GET() == (char)89);
            assert(pack.exposure_type_GET() == (char)108);
            assert(pack.target_component_GET() == (char)98);
            assert(pack.target_system_GET() == (char)144);
            assert(pack.command_id_GET() == (char)205);
            assert(pack.shutter_speed_GET() == (char)24089);
            assert(pack.aperture_GET() == (char)246);
            assert(pack.iso_GET() == (char)153);
        });
        GroundControl.DIGICAM_CONFIGURE p154 = CommunicationChannel.new_DIGICAM_CONFIGURE();
        PH.setPack(p154);
        p154.target_system_SET((char)144) ;
        p154.aperture_SET((char)246) ;
        p154.engine_cut_off_SET((char)89) ;
        p154.extra_param_SET((char)71) ;
        p154.exposure_type_SET((char)108) ;
        p154.mode_SET((char)39) ;
        p154.shutter_speed_SET((char)24089) ;
        p154.extra_value_SET(2.2104163E38F) ;
        p154.target_component_SET((char)98) ;
        p154.iso_SET((char)153) ;
        p154.command_id_SET((char)205) ;
        CommunicationChannel.instance.send(p154);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DIGICAM_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.zoom_pos_GET() == (char)162);
            assert(pack.command_id_GET() == (char)32);
            assert(pack.target_component_GET() == (char)37);
            assert(pack.zoom_step_GET() == (byte)114);
            assert(pack.session_GET() == (char)6);
            assert(pack.focus_lock_GET() == (char)52);
            assert(pack.extra_value_GET() == 5.075982E37F);
            assert(pack.target_system_GET() == (char)135);
            assert(pack.shot_GET() == (char)208);
            assert(pack.extra_param_GET() == (char)74);
        });
        GroundControl.DIGICAM_CONTROL p155 = CommunicationChannel.new_DIGICAM_CONTROL();
        PH.setPack(p155);
        p155.zoom_step_SET((byte)114) ;
        p155.extra_param_SET((char)74) ;
        p155.target_component_SET((char)37) ;
        p155.target_system_SET((char)135) ;
        p155.shot_SET((char)208) ;
        p155.session_SET((char)6) ;
        p155.extra_value_SET(5.075982E37F) ;
        p155.command_id_SET((char)32) ;
        p155.zoom_pos_SET((char)162) ;
        p155.focus_lock_SET((char)52) ;
        CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_CONFIGURE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)38);
            assert(pack.stab_pitch_GET() == (char)240);
            assert(pack.stab_yaw_GET() == (char)252);
            assert(pack.mount_mode_GET() == MAV_MOUNT_MODE.MAV_MOUNT_MODE_RC_TARGETING);
            assert(pack.target_component_GET() == (char)185);
            assert(pack.stab_roll_GET() == (char)49);
        });
        GroundControl.MOUNT_CONFIGURE p156 = CommunicationChannel.new_MOUNT_CONFIGURE();
        PH.setPack(p156);
        p156.stab_roll_SET((char)49) ;
        p156.stab_yaw_SET((char)252) ;
        p156.target_component_SET((char)185) ;
        p156.target_system_SET((char)38) ;
        p156.mount_mode_SET(MAV_MOUNT_MODE.MAV_MOUNT_MODE_RC_TARGETING) ;
        p156.stab_pitch_SET((char)240) ;
        CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.input_c_GET() == -1179877632);
            assert(pack.save_position_GET() == (char)169);
            assert(pack.target_component_GET() == (char)148);
            assert(pack.target_system_GET() == (char)223);
            assert(pack.input_a_GET() == -1188326663);
            assert(pack.input_b_GET() == 202536583);
        });
        GroundControl.MOUNT_CONTROL p157 = CommunicationChannel.new_MOUNT_CONTROL();
        PH.setPack(p157);
        p157.target_component_SET((char)148) ;
        p157.input_c_SET(-1179877632) ;
        p157.input_b_SET(202536583) ;
        p157.input_a_SET(-1188326663) ;
        p157.target_system_SET((char)223) ;
        p157.save_position_SET((char)169) ;
        CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pointing_a_GET() == -173951034);
            assert(pack.pointing_c_GET() == 542889497);
            assert(pack.target_component_GET() == (char)153);
            assert(pack.pointing_b_GET() == 1828699401);
            assert(pack.target_system_GET() == (char)215);
        });
        GroundControl.MOUNT_STATUS p158 = CommunicationChannel.new_MOUNT_STATUS();
        PH.setPack(p158);
        p158.target_system_SET((char)215) ;
        p158.target_component_SET((char)153) ;
        p158.pointing_c_SET(542889497) ;
        p158.pointing_a_SET(-173951034) ;
        p158.pointing_b_SET(1828699401) ;
        CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FENCE_POINT.add((src, ph, pack) ->
        {
            assert(pack.lng_GET() == 1.2847138E38F);
            assert(pack.target_component_GET() == (char)194);
            assert(pack.target_system_GET() == (char)152);
            assert(pack.lat_GET() == 8.576065E37F);
            assert(pack.count_GET() == (char)186);
            assert(pack.idx_GET() == (char)230);
        });
        GroundControl.FENCE_POINT p160 = CommunicationChannel.new_FENCE_POINT();
        PH.setPack(p160);
        p160.lng_SET(1.2847138E38F) ;
        p160.target_system_SET((char)152) ;
        p160.count_SET((char)186) ;
        p160.target_component_SET((char)194) ;
        p160.idx_SET((char)230) ;
        p160.lat_SET(8.576065E37F) ;
        CommunicationChannel.instance.send(p160);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FENCE_FETCH_POINT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)194);
            assert(pack.target_system_GET() == (char)142);
            assert(pack.idx_GET() == (char)110);
        });
        GroundControl.FENCE_FETCH_POINT p161 = CommunicationChannel.new_FENCE_FETCH_POINT();
        PH.setPack(p161);
        p161.idx_SET((char)110) ;
        p161.target_component_SET((char)194) ;
        p161.target_system_SET((char)142) ;
        CommunicationChannel.instance.send(p161);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FENCE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.breach_time_GET() == 3794649799L);
            assert(pack.breach_status_GET() == (char)163);
            assert(pack.breach_count_GET() == (char)1852);
            assert(pack.breach_type_GET() == FENCE_BREACH.FENCE_BREACH_MAXALT);
        });
        GroundControl.FENCE_STATUS p162 = CommunicationChannel.new_FENCE_STATUS();
        PH.setPack(p162);
        p162.breach_time_SET(3794649799L) ;
        p162.breach_count_SET((char)1852) ;
        p162.breach_type_SET(FENCE_BREACH.FENCE_BREACH_MAXALT) ;
        p162.breach_status_SET((char)163) ;
        CommunicationChannel.instance.send(p162);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AHRS.add((src, ph, pack) ->
        {
            assert(pack.omegaIx_GET() == -2.1112893E38F);
            assert(pack.error_yaw_GET() == -2.4166179E38F);
            assert(pack.error_rp_GET() == 9.089531E37F);
            assert(pack.accel_weight_GET() == 2.1560452E37F);
            assert(pack.renorm_val_GET() == 1.6993255E38F);
            assert(pack.omegaIz_GET() == 2.6439906E38F);
            assert(pack.omegaIy_GET() == -1.2842266E38F);
        });
        GroundControl.AHRS p163 = CommunicationChannel.new_AHRS();
        PH.setPack(p163);
        p163.accel_weight_SET(2.1560452E37F) ;
        p163.omegaIx_SET(-2.1112893E38F) ;
        p163.error_yaw_SET(-2.4166179E38F) ;
        p163.renorm_val_SET(1.6993255E38F) ;
        p163.omegaIy_SET(-1.2842266E38F) ;
        p163.error_rp_SET(9.089531E37F) ;
        p163.omegaIz_SET(2.6439906E38F) ;
        CommunicationChannel.instance.send(p163);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SIMSTATE.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == 1.113288E38F);
            assert(pack.lat_GET() == -201741737);
            assert(pack.zacc_GET() == -1.3864755E38F);
            assert(pack.roll_GET() == -3.0531393E38F);
            assert(pack.yacc_GET() == 2.2465369E38F);
            assert(pack.yaw_GET() == 1.2212752E38F);
            assert(pack.xgyro_GET() == 2.5637014E38F);
            assert(pack.pitch_GET() == 2.7440632E38F);
            assert(pack.xacc_GET() == -2.930244E38F);
            assert(pack.lng_GET() == 744556981);
            assert(pack.ygyro_GET() == 1.2168252E38F);
        });
        GroundControl.SIMSTATE p164 = CommunicationChannel.new_SIMSTATE();
        PH.setPack(p164);
        p164.yaw_SET(1.2212752E38F) ;
        p164.xacc_SET(-2.930244E38F) ;
        p164.roll_SET(-3.0531393E38F) ;
        p164.lng_SET(744556981) ;
        p164.zacc_SET(-1.3864755E38F) ;
        p164.zgyro_SET(1.113288E38F) ;
        p164.yacc_SET(2.2465369E38F) ;
        p164.pitch_SET(2.7440632E38F) ;
        p164.lat_SET(-201741737) ;
        p164.xgyro_SET(2.5637014E38F) ;
        p164.ygyro_SET(1.2168252E38F) ;
        CommunicationChannel.instance.send(p164);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HWSTATUS.add((src, ph, pack) ->
        {
            assert(pack.Vcc_GET() == (char)20201);
            assert(pack.I2Cerr_GET() == (char)184);
        });
        GroundControl.HWSTATUS p165 = CommunicationChannel.new_HWSTATUS();
        PH.setPack(p165);
        p165.Vcc_SET((char)20201) ;
        p165.I2Cerr_SET((char)184) ;
        CommunicationChannel.instance.send(p165);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RADIO.add((src, ph, pack) ->
        {
            assert(pack.remnoise_GET() == (char)162);
            assert(pack.txbuf_GET() == (char)94);
            assert(pack.rxerrors_GET() == (char)11650);
            assert(pack.remrssi_GET() == (char)65);
            assert(pack.fixed__GET() == (char)60362);
            assert(pack.rssi_GET() == (char)250);
            assert(pack.noise_GET() == (char)247);
        });
        GroundControl.RADIO p166 = CommunicationChannel.new_RADIO();
        PH.setPack(p166);
        p166.remrssi_SET((char)65) ;
        p166.fixed__SET((char)60362) ;
        p166.rssi_SET((char)250) ;
        p166.rxerrors_SET((char)11650) ;
        p166.txbuf_SET((char)94) ;
        p166.noise_SET((char)247) ;
        p166.remnoise_SET((char)162) ;
        CommunicationChannel.instance.send(p166);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LIMITS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.last_trigger_GET() == 685025900L);
            assert(pack.breach_count_GET() == (char)1492);
            assert(pack.last_recovery_GET() == 2073779427L);
            assert(pack.mods_triggered_GET() == (LIMIT_MODULE.LIMIT_GEOFENCE |
                                                 LIMIT_MODULE.LIMIT_GPSLOCK));
            assert(pack.mods_required_GET() == (LIMIT_MODULE.LIMIT_GPSLOCK));
            assert(pack.limits_state_GET() == LIMITS_STATE.LIMITS_RECOVERED);
            assert(pack.mods_enabled_GET() == (LIMIT_MODULE.LIMIT_GPSLOCK));
            assert(pack.last_clear_GET() == 3156245160L);
            assert(pack.last_action_GET() == 924144129L);
        });
        GroundControl.LIMITS_STATUS p167 = CommunicationChannel.new_LIMITS_STATUS();
        PH.setPack(p167);
        p167.last_recovery_SET(2073779427L) ;
        p167.breach_count_SET((char)1492) ;
        p167.mods_enabled_SET((LIMIT_MODULE.LIMIT_GPSLOCK)) ;
        p167.mods_triggered_SET((LIMIT_MODULE.LIMIT_GEOFENCE |
                                 LIMIT_MODULE.LIMIT_GPSLOCK)) ;
        p167.last_action_SET(924144129L) ;
        p167.last_clear_SET(3156245160L) ;
        p167.last_trigger_SET(685025900L) ;
        p167.limits_state_SET(LIMITS_STATE.LIMITS_RECOVERED) ;
        p167.mods_required_SET((LIMIT_MODULE.LIMIT_GPSLOCK)) ;
        CommunicationChannel.instance.send(p167);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIND.add((src, ph, pack) ->
        {
            assert(pack.direction_GET() == 1.5439933E38F);
            assert(pack.speed_z_GET() == 2.1152158E38F);
            assert(pack.speed_GET() == -7.74608E37F);
        });
        GroundControl.WIND p168 = CommunicationChannel.new_WIND();
        PH.setPack(p168);
        p168.speed_z_SET(2.1152158E38F) ;
        p168.speed_SET(-7.74608E37F) ;
        p168.direction_SET(1.5439933E38F) ;
        CommunicationChannel.instance.send(p168);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DATA16.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)161);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)102, (char)5, (char)58, (char)122, (char)223, (char)159, (char)244, (char)183, (char)218, (char)157, (char)141, (char)90, (char)41, (char)184, (char)121, (char)82}));
            assert(pack.type_GET() == (char)42);
        });
        GroundControl.DATA16 p169 = CommunicationChannel.new_DATA16();
        PH.setPack(p169);
        p169.type_SET((char)42) ;
        p169.len_SET((char)161) ;
        p169.data__SET(new char[] {(char)102, (char)5, (char)58, (char)122, (char)223, (char)159, (char)244, (char)183, (char)218, (char)157, (char)141, (char)90, (char)41, (char)184, (char)121, (char)82}, 0) ;
        CommunicationChannel.instance.send(p169);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DATA32.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)50);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)162, (char)195, (char)16, (char)111, (char)104, (char)130, (char)66, (char)60, (char)216, (char)39, (char)26, (char)96, (char)2, (char)118, (char)218, (char)57, (char)136, (char)63, (char)37, (char)164, (char)64, (char)83, (char)4, (char)223, (char)250, (char)198, (char)131, (char)27, (char)39, (char)148, (char)59, (char)130}));
            assert(pack.len_GET() == (char)238);
        });
        GroundControl.DATA32 p170 = CommunicationChannel.new_DATA32();
        PH.setPack(p170);
        p170.type_SET((char)50) ;
        p170.data__SET(new char[] {(char)162, (char)195, (char)16, (char)111, (char)104, (char)130, (char)66, (char)60, (char)216, (char)39, (char)26, (char)96, (char)2, (char)118, (char)218, (char)57, (char)136, (char)63, (char)37, (char)164, (char)64, (char)83, (char)4, (char)223, (char)250, (char)198, (char)131, (char)27, (char)39, (char)148, (char)59, (char)130}, 0) ;
        p170.len_SET((char)238) ;
        CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DATA64.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)200);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)116, (char)4, (char)233, (char)59, (char)37, (char)104, (char)30, (char)228, (char)186, (char)39, (char)235, (char)24, (char)24, (char)127, (char)73, (char)43, (char)83, (char)72, (char)159, (char)153, (char)32, (char)123, (char)215, (char)86, (char)154, (char)33, (char)86, (char)48, (char)111, (char)251, (char)60, (char)250, (char)49, (char)93, (char)174, (char)97, (char)119, (char)121, (char)83, (char)156, (char)80, (char)235, (char)77, (char)205, (char)208, (char)97, (char)120, (char)138, (char)77, (char)236, (char)75, (char)24, (char)115, (char)251, (char)170, (char)58, (char)157, (char)133, (char)58, (char)249, (char)209, (char)210, (char)6, (char)116}));
            assert(pack.type_GET() == (char)241);
        });
        GroundControl.DATA64 p171 = CommunicationChannel.new_DATA64();
        PH.setPack(p171);
        p171.data__SET(new char[] {(char)116, (char)4, (char)233, (char)59, (char)37, (char)104, (char)30, (char)228, (char)186, (char)39, (char)235, (char)24, (char)24, (char)127, (char)73, (char)43, (char)83, (char)72, (char)159, (char)153, (char)32, (char)123, (char)215, (char)86, (char)154, (char)33, (char)86, (char)48, (char)111, (char)251, (char)60, (char)250, (char)49, (char)93, (char)174, (char)97, (char)119, (char)121, (char)83, (char)156, (char)80, (char)235, (char)77, (char)205, (char)208, (char)97, (char)120, (char)138, (char)77, (char)236, (char)75, (char)24, (char)115, (char)251, (char)170, (char)58, (char)157, (char)133, (char)58, (char)249, (char)209, (char)210, (char)6, (char)116}, 0) ;
        p171.type_SET((char)241) ;
        p171.len_SET((char)200) ;
        CommunicationChannel.instance.send(p171);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DATA96.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)20, (char)158, (char)204, (char)236, (char)234, (char)73, (char)19, (char)34, (char)129, (char)45, (char)68, (char)207, (char)123, (char)83, (char)216, (char)143, (char)128, (char)108, (char)99, (char)170, (char)117, (char)140, (char)217, (char)76, (char)16, (char)26, (char)26, (char)24, (char)192, (char)86, (char)140, (char)117, (char)1, (char)235, (char)46, (char)139, (char)38, (char)207, (char)43, (char)174, (char)187, (char)158, (char)198, (char)214, (char)65, (char)107, (char)185, (char)18, (char)62, (char)164, (char)169, (char)215, (char)138, (char)245, (char)171, (char)155, (char)51, (char)187, (char)3, (char)43, (char)77, (char)109, (char)136, (char)248, (char)156, (char)203, (char)157, (char)114, (char)68, (char)127, (char)136, (char)5, (char)230, (char)126, (char)22, (char)0, (char)181, (char)176, (char)31, (char)57, (char)202, (char)133, (char)213, (char)242, (char)91, (char)98, (char)24, (char)215, (char)234, (char)62, (char)81, (char)121, (char)213, (char)107, (char)249, (char)212}));
            assert(pack.len_GET() == (char)81);
            assert(pack.type_GET() == (char)49);
        });
        GroundControl.DATA96 p172 = CommunicationChannel.new_DATA96();
        PH.setPack(p172);
        p172.data__SET(new char[] {(char)20, (char)158, (char)204, (char)236, (char)234, (char)73, (char)19, (char)34, (char)129, (char)45, (char)68, (char)207, (char)123, (char)83, (char)216, (char)143, (char)128, (char)108, (char)99, (char)170, (char)117, (char)140, (char)217, (char)76, (char)16, (char)26, (char)26, (char)24, (char)192, (char)86, (char)140, (char)117, (char)1, (char)235, (char)46, (char)139, (char)38, (char)207, (char)43, (char)174, (char)187, (char)158, (char)198, (char)214, (char)65, (char)107, (char)185, (char)18, (char)62, (char)164, (char)169, (char)215, (char)138, (char)245, (char)171, (char)155, (char)51, (char)187, (char)3, (char)43, (char)77, (char)109, (char)136, (char)248, (char)156, (char)203, (char)157, (char)114, (char)68, (char)127, (char)136, (char)5, (char)230, (char)126, (char)22, (char)0, (char)181, (char)176, (char)31, (char)57, (char)202, (char)133, (char)213, (char)242, (char)91, (char)98, (char)24, (char)215, (char)234, (char)62, (char)81, (char)121, (char)213, (char)107, (char)249, (char)212}, 0) ;
        p172.len_SET((char)81) ;
        p172.type_SET((char)49) ;
        CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RANGEFINDER.add((src, ph, pack) ->
        {
            assert(pack.voltage_GET() == -1.7895243E38F);
            assert(pack.distance_GET() == -1.8245973E38F);
        });
        GroundControl.RANGEFINDER p173 = CommunicationChannel.new_RANGEFINDER();
        PH.setPack(p173);
        p173.voltage_SET(-1.7895243E38F) ;
        p173.distance_SET(-1.8245973E38F) ;
        CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AIRSPEED_AUTOCAL.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == 1.4114723E38F);
            assert(pack.vx_GET() == -1.424236E38F);
            assert(pack.diff_pressure_GET() == -1.9348092E38F);
            assert(pack.Pcz_GET() == -2.5353083E38F);
            assert(pack.state_x_GET() == 6.491593E37F);
            assert(pack.EAS2TAS_GET() == 1.571678E38F);
            assert(pack.ratio_GET() == 1.4817162E38F);
            assert(pack.state_z_GET() == 1.2667724E38F);
            assert(pack.Pby_GET() == 5.0502054E37F);
            assert(pack.vy_GET() == -2.7640752E38F);
            assert(pack.Pax_GET() == -7.7620264E37F);
            assert(pack.state_y_GET() == 2.8649196E38F);
        });
        GroundControl.AIRSPEED_AUTOCAL p174 = CommunicationChannel.new_AIRSPEED_AUTOCAL();
        PH.setPack(p174);
        p174.ratio_SET(1.4817162E38F) ;
        p174.vx_SET(-1.424236E38F) ;
        p174.Pcz_SET(-2.5353083E38F) ;
        p174.diff_pressure_SET(-1.9348092E38F) ;
        p174.Pby_SET(5.0502054E37F) ;
        p174.EAS2TAS_SET(1.571678E38F) ;
        p174.Pax_SET(-7.7620264E37F) ;
        p174.state_z_SET(1.2667724E38F) ;
        p174.state_y_SET(2.8649196E38F) ;
        p174.state_x_SET(6.491593E37F) ;
        p174.vz_SET(1.4114723E38F) ;
        p174.vy_SET(-2.7640752E38F) ;
        CommunicationChannel.instance.send(p174);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RALLY_POINT.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1172000146);
            assert(pack.break_alt_GET() == (short)27300);
            assert(pack.target_system_GET() == (char)44);
            assert(pack.lng_GET() == -722046414);
            assert(pack.idx_GET() == (char)50);
            assert(pack.target_component_GET() == (char)17);
            assert(pack.alt_GET() == (short) -25467);
            assert(pack.land_dir_GET() == (char)53958);
            assert(pack.count_GET() == (char)170);
            assert(pack.flags_GET() == RALLY_FLAGS.LAND_IMMEDIATELY);
        });
        GroundControl.RALLY_POINT p175 = CommunicationChannel.new_RALLY_POINT();
        PH.setPack(p175);
        p175.alt_SET((short) -25467) ;
        p175.target_component_SET((char)17) ;
        p175.break_alt_SET((short)27300) ;
        p175.idx_SET((char)50) ;
        p175.lng_SET(-722046414) ;
        p175.flags_SET(RALLY_FLAGS.LAND_IMMEDIATELY) ;
        p175.lat_SET(1172000146) ;
        p175.count_SET((char)170) ;
        p175.land_dir_SET((char)53958) ;
        p175.target_system_SET((char)44) ;
        CommunicationChannel.instance.send(p175);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RALLY_FETCH_POINT.add((src, ph, pack) ->
        {
            assert(pack.idx_GET() == (char)107);
            assert(pack.target_component_GET() == (char)97);
            assert(pack.target_system_GET() == (char)82);
        });
        GroundControl.RALLY_FETCH_POINT p176 = CommunicationChannel.new_RALLY_FETCH_POINT();
        PH.setPack(p176);
        p176.target_component_SET((char)97) ;
        p176.idx_SET((char)107) ;
        p176.target_system_SET((char)82) ;
        CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_COMPASSMOT_STATUS.add((src, ph, pack) ->
        {
            assert(pack.CompensationY_GET() == -1.7128203E38F);
            assert(pack.interference_GET() == (char)195);
            assert(pack.throttle_GET() == (char)59575);
            assert(pack.CompensationX_GET() == -7.5327875E37F);
            assert(pack.current_GET() == 2.7873118E38F);
            assert(pack.CompensationZ_GET() == -2.0489463E38F);
        });
        GroundControl.COMPASSMOT_STATUS p177 = CommunicationChannel.new_COMPASSMOT_STATUS();
        PH.setPack(p177);
        p177.CompensationY_SET(-1.7128203E38F) ;
        p177.interference_SET((char)195) ;
        p177.current_SET(2.7873118E38F) ;
        p177.CompensationZ_SET(-2.0489463E38F) ;
        p177.CompensationX_SET(-7.5327875E37F) ;
        p177.throttle_SET((char)59575) ;
        CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AHRS2.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 3.3461818E38F);
            assert(pack.lat_GET() == 568268771);
            assert(pack.roll_GET() == -2.707652E38F);
            assert(pack.altitude_GET() == 1.6964939E38F);
            assert(pack.pitch_GET() == -2.6576832E38F);
            assert(pack.lng_GET() == 493016876);
        });
        GroundControl.AHRS2 p178 = CommunicationChannel.new_AHRS2();
        PH.setPack(p178);
        p178.yaw_SET(3.3461818E38F) ;
        p178.pitch_SET(-2.6576832E38F) ;
        p178.lat_SET(568268771) ;
        p178.roll_SET(-2.707652E38F) ;
        p178.lng_SET(493016876) ;
        p178.altitude_SET(1.6964939E38F) ;
        CommunicationChannel.instance.send(p178);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_STATUS.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)70);
            assert(pack.p2_GET() == -1.2823642E38F);
            assert(pack.p4_GET() == -1.8474131E38F);
            assert(pack.p1_GET() == 2.1989278E38F);
            assert(pack.time_usec_GET() == 3212523843019188651L);
            assert(pack.event_id_GET() == CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_HEARTBEAT);
            assert(pack.p3_GET() == 1.2539714E38F);
            assert(pack.cam_idx_GET() == (char)195);
            assert(pack.img_idx_GET() == (char)29177);
        });
        GroundControl.CAMERA_STATUS p179 = CommunicationChannel.new_CAMERA_STATUS();
        PH.setPack(p179);
        p179.event_id_SET(CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_HEARTBEAT) ;
        p179.target_system_SET((char)70) ;
        p179.p1_SET(2.1989278E38F) ;
        p179.p3_SET(1.2539714E38F) ;
        p179.p2_SET(-1.2823642E38F) ;
        p179.p4_SET(-1.8474131E38F) ;
        p179.cam_idx_SET((char)195) ;
        p179.time_usec_SET(3212523843019188651L) ;
        p179.img_idx_SET((char)29177) ;
        CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_FEEDBACK.add((src, ph, pack) ->
        {
            assert(pack.cam_idx_GET() == (char)223);
            assert(pack.yaw_GET() == -2.369506E38F);
            assert(pack.flags_GET() == CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_CLOSEDLOOP);
            assert(pack.img_idx_GET() == (char)17133);
            assert(pack.pitch_GET() == 7.210955E37F);
            assert(pack.roll_GET() == 2.2924682E38F);
            assert(pack.time_usec_GET() == 2183433041905769127L);
            assert(pack.alt_rel_GET() == 1.2417925E38F);
            assert(pack.target_system_GET() == (char)168);
            assert(pack.lng_GET() == 639476161);
            assert(pack.lat_GET() == 1158299903);
            assert(pack.alt_msl_GET() == 3.151931E38F);
            assert(pack.foc_len_GET() == 2.543975E38F);
        });
        GroundControl.CAMERA_FEEDBACK p180 = CommunicationChannel.new_CAMERA_FEEDBACK();
        PH.setPack(p180);
        p180.img_idx_SET((char)17133) ;
        p180.roll_SET(2.2924682E38F) ;
        p180.alt_msl_SET(3.151931E38F) ;
        p180.foc_len_SET(2.543975E38F) ;
        p180.alt_rel_SET(1.2417925E38F) ;
        p180.time_usec_SET(2183433041905769127L) ;
        p180.flags_SET(CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_CLOSEDLOOP) ;
        p180.lat_SET(1158299903) ;
        p180.lng_SET(639476161) ;
        p180.yaw_SET(-2.369506E38F) ;
        p180.pitch_SET(7.210955E37F) ;
        p180.cam_idx_SET((char)223) ;
        p180.target_system_SET((char)168) ;
        CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BATTERY2.add((src, ph, pack) ->
        {
            assert(pack.voltage_GET() == (char)6653);
            assert(pack.current_battery_GET() == (short)77);
        });
        GroundControl.BATTERY2 p181 = CommunicationChannel.new_BATTERY2();
        PH.setPack(p181);
        p181.current_battery_SET((short)77) ;
        p181.voltage_SET((char)6653) ;
        CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AHRS3.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == 1.425372E38F);
            assert(pack.lat_GET() == 1416273721);
            assert(pack.pitch_GET() == -2.7386433E38F);
            assert(pack.roll_GET() == 1.2424901E38F);
            assert(pack.v1_GET() == 4.501989E37F);
            assert(pack.yaw_GET() == -3.391902E38F);
            assert(pack.lng_GET() == 490172406);
            assert(pack.v2_GET() == -5.8793367E34F);
            assert(pack.v3_GET() == 9.664338E37F);
            assert(pack.v4_GET() == -2.3297808E38F);
        });
        GroundControl.AHRS3 p182 = CommunicationChannel.new_AHRS3();
        PH.setPack(p182);
        p182.v4_SET(-2.3297808E38F) ;
        p182.roll_SET(1.2424901E38F) ;
        p182.lng_SET(490172406) ;
        p182.v1_SET(4.501989E37F) ;
        p182.altitude_SET(1.425372E38F) ;
        p182.pitch_SET(-2.7386433E38F) ;
        p182.v2_SET(-5.8793367E34F) ;
        p182.yaw_SET(-3.391902E38F) ;
        p182.v3_SET(9.664338E37F) ;
        p182.lat_SET(1416273721) ;
        CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)33);
            assert(pack.target_system_GET() == (char)181);
        });
        GroundControl.AUTOPILOT_VERSION_REQUEST p183 = CommunicationChannel.new_AUTOPILOT_VERSION_REQUEST();
        PH.setPack(p183);
        p183.target_system_SET((char)181) ;
        p183.target_component_SET((char)33) ;
        CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_REMOTE_LOG_DATA_BLOCK.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)19, (char)9, (char)197, (char)160, (char)205, (char)185, (char)210, (char)202, (char)230, (char)253, (char)97, (char)238, (char)52, (char)73, (char)121, (char)56, (char)124, (char)225, (char)232, (char)195, (char)196, (char)158, (char)189, (char)71, (char)39, (char)160, (char)176, (char)60, (char)122, (char)214, (char)239, (char)72, (char)149, (char)112, (char)118, (char)159, (char)116, (char)190, (char)205, (char)233, (char)254, (char)88, (char)77, (char)193, (char)39, (char)197, (char)203, (char)250, (char)163, (char)124, (char)65, (char)25, (char)243, (char)130, (char)63, (char)232, (char)47, (char)132, (char)78, (char)225, (char)24, (char)127, (char)19, (char)55, (char)233, (char)226, (char)146, (char)52, (char)109, (char)194, (char)98, (char)116, (char)157, (char)135, (char)209, (char)150, (char)181, (char)201, (char)7, (char)105, (char)66, (char)167, (char)69, (char)34, (char)41, (char)73, (char)233, (char)106, (char)92, (char)8, (char)123, (char)1, (char)55, (char)68, (char)104, (char)117, (char)171, (char)137, (char)212, (char)37, (char)19, (char)87, (char)159, (char)152, (char)158, (char)209, (char)24, (char)243, (char)60, (char)92, (char)99, (char)181, (char)166, (char)87, (char)25, (char)129, (char)244, (char)70, (char)113, (char)242, (char)125, (char)24, (char)66, (char)122, (char)109, (char)179, (char)161, (char)193, (char)72, (char)102, (char)118, (char)77, (char)1, (char)155, (char)171, (char)72, (char)95, (char)11, (char)212, (char)171, (char)254, (char)39, (char)183, (char)228, (char)247, (char)164, (char)61, (char)25, (char)65, (char)54, (char)218, (char)159, (char)40, (char)6, (char)81, (char)169, (char)114, (char)70, (char)145, (char)65, (char)93, (char)108, (char)236, (char)246, (char)46, (char)101, (char)175, (char)225, (char)136, (char)143, (char)23, (char)162, (char)161, (char)85, (char)169, (char)225, (char)118, (char)7, (char)165, (char)63, (char)230, (char)5, (char)59, (char)81, (char)44, (char)47, (char)116, (char)37, (char)238, (char)223, (char)252, (char)12, (char)68, (char)165, (char)134, (char)194, (char)60, (char)223, (char)83, (char)122}));
            assert(pack.target_system_GET() == (char)48);
            assert(pack.seqno_GET() == MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP);
            assert(pack.target_component_GET() == (char)81);
        });
        GroundControl.REMOTE_LOG_DATA_BLOCK p184 = CommunicationChannel.new_REMOTE_LOG_DATA_BLOCK();
        PH.setPack(p184);
        p184.target_component_SET((char)81) ;
        p184.data__SET(new char[] {(char)19, (char)9, (char)197, (char)160, (char)205, (char)185, (char)210, (char)202, (char)230, (char)253, (char)97, (char)238, (char)52, (char)73, (char)121, (char)56, (char)124, (char)225, (char)232, (char)195, (char)196, (char)158, (char)189, (char)71, (char)39, (char)160, (char)176, (char)60, (char)122, (char)214, (char)239, (char)72, (char)149, (char)112, (char)118, (char)159, (char)116, (char)190, (char)205, (char)233, (char)254, (char)88, (char)77, (char)193, (char)39, (char)197, (char)203, (char)250, (char)163, (char)124, (char)65, (char)25, (char)243, (char)130, (char)63, (char)232, (char)47, (char)132, (char)78, (char)225, (char)24, (char)127, (char)19, (char)55, (char)233, (char)226, (char)146, (char)52, (char)109, (char)194, (char)98, (char)116, (char)157, (char)135, (char)209, (char)150, (char)181, (char)201, (char)7, (char)105, (char)66, (char)167, (char)69, (char)34, (char)41, (char)73, (char)233, (char)106, (char)92, (char)8, (char)123, (char)1, (char)55, (char)68, (char)104, (char)117, (char)171, (char)137, (char)212, (char)37, (char)19, (char)87, (char)159, (char)152, (char)158, (char)209, (char)24, (char)243, (char)60, (char)92, (char)99, (char)181, (char)166, (char)87, (char)25, (char)129, (char)244, (char)70, (char)113, (char)242, (char)125, (char)24, (char)66, (char)122, (char)109, (char)179, (char)161, (char)193, (char)72, (char)102, (char)118, (char)77, (char)1, (char)155, (char)171, (char)72, (char)95, (char)11, (char)212, (char)171, (char)254, (char)39, (char)183, (char)228, (char)247, (char)164, (char)61, (char)25, (char)65, (char)54, (char)218, (char)159, (char)40, (char)6, (char)81, (char)169, (char)114, (char)70, (char)145, (char)65, (char)93, (char)108, (char)236, (char)246, (char)46, (char)101, (char)175, (char)225, (char)136, (char)143, (char)23, (char)162, (char)161, (char)85, (char)169, (char)225, (char)118, (char)7, (char)165, (char)63, (char)230, (char)5, (char)59, (char)81, (char)44, (char)47, (char)116, (char)37, (char)238, (char)223, (char)252, (char)12, (char)68, (char)165, (char)134, (char)194, (char)60, (char)223, (char)83, (char)122}, 0) ;
        p184.seqno_SET(MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP) ;
        p184.target_system_SET((char)48) ;
        CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_REMOTE_LOG_BLOCK_STATUS.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)154);
            assert(pack.target_system_GET() == (char)127);
            assert(pack.status_GET() == MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK);
            assert(pack.seqno_GET() == 10785721L);
        });
        GroundControl.REMOTE_LOG_BLOCK_STATUS p185 = CommunicationChannel.new_REMOTE_LOG_BLOCK_STATUS();
        PH.setPack(p185);
        p185.status_SET(MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK) ;
        p185.seqno_SET(10785721L) ;
        p185.target_component_SET((char)154) ;
        p185.target_system_SET((char)127) ;
        CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LED_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.custom_len_GET() == (char)175);
            assert(pack.pattern_GET() == (char)117);
            assert(pack.instance_GET() == (char)185);
            assert(pack.target_system_GET() == (char)237);
            assert(Arrays.equals(pack.custom_bytes_GET(),  new char[] {(char)220, (char)133, (char)73, (char)37, (char)232, (char)225, (char)237, (char)119, (char)188, (char)162, (char)240, (char)234, (char)219, (char)35, (char)156, (char)106, (char)155, (char)51, (char)69, (char)49, (char)88, (char)11, (char)230, (char)105}));
            assert(pack.target_component_GET() == (char)87);
        });
        GroundControl.LED_CONTROL p186 = CommunicationChannel.new_LED_CONTROL();
        PH.setPack(p186);
        p186.instance_SET((char)185) ;
        p186.target_component_SET((char)87) ;
        p186.custom_len_SET((char)175) ;
        p186.custom_bytes_SET(new char[] {(char)220, (char)133, (char)73, (char)37, (char)232, (char)225, (char)237, (char)119, (char)188, (char)162, (char)240, (char)234, (char)219, (char)35, (char)156, (char)106, (char)155, (char)51, (char)69, (char)49, (char)88, (char)11, (char)230, (char)105}, 0) ;
        p186.pattern_SET((char)117) ;
        p186.target_system_SET((char)237) ;
        CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MAG_CAL_PROGRESS.add((src, ph, pack) ->
        {
            assert(pack.direction_x_GET() == -2.3841346E38F);
            assert(pack.cal_mask_GET() == (char)212);
            assert(pack.cal_status_GET() == MAG_CAL_STATUS.MAG_CAL_RUNNING_STEP_TWO);
            assert(Arrays.equals(pack.completion_mask_GET(),  new char[] {(char)192, (char)171, (char)206, (char)28, (char)70, (char)84, (char)156, (char)38, (char)186, (char)172}));
            assert(pack.attempt_GET() == (char)202);
            assert(pack.direction_z_GET() == -1.8739496E38F);
            assert(pack.completion_pct_GET() == (char)138);
            assert(pack.direction_y_GET() == -9.63706E37F);
            assert(pack.compass_id_GET() == (char)49);
        });
        GroundControl.MAG_CAL_PROGRESS p191 = CommunicationChannel.new_MAG_CAL_PROGRESS();
        PH.setPack(p191);
        p191.direction_z_SET(-1.8739496E38F) ;
        p191.direction_x_SET(-2.3841346E38F) ;
        p191.direction_y_SET(-9.63706E37F) ;
        p191.cal_status_SET(MAG_CAL_STATUS.MAG_CAL_RUNNING_STEP_TWO) ;
        p191.compass_id_SET((char)49) ;
        p191.attempt_SET((char)202) ;
        p191.cal_mask_SET((char)212) ;
        p191.completion_pct_SET((char)138) ;
        p191.completion_mask_SET(new char[] {(char)192, (char)171, (char)206, (char)28, (char)70, (char)84, (char)156, (char)38, (char)186, (char)172}, 0) ;
        CommunicationChannel.instance.send(p191);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MAG_CAL_REPORT.add((src, ph, pack) ->
        {
            assert(pack.ofs_z_GET() == 2.953284E37F);
            assert(pack.offdiag_z_GET() == 8.953356E37F);
            assert(pack.cal_mask_GET() == (char)21);
            assert(pack.diag_y_GET() == 2.271106E38F);
            assert(pack.ofs_x_GET() == -9.935474E37F);
            assert(pack.offdiag_y_GET() == -1.6959216E38F);
            assert(pack.compass_id_GET() == (char)43);
            assert(pack.diag_x_GET() == -2.545963E38F);
            assert(pack.autosaved_GET() == (char)90);
            assert(pack.diag_z_GET() == -9.457495E37F);
            assert(pack.ofs_y_GET() == -2.3110702E38F);
            assert(pack.fitness_GET() == 5.96335E37F);
            assert(pack.offdiag_x_GET() == 2.6489839E37F);
            assert(pack.cal_status_GET() == MAG_CAL_STATUS.MAG_CAL_SUCCESS);
        });
        GroundControl.MAG_CAL_REPORT p192 = CommunicationChannel.new_MAG_CAL_REPORT();
        PH.setPack(p192);
        p192.cal_status_SET(MAG_CAL_STATUS.MAG_CAL_SUCCESS) ;
        p192.cal_mask_SET((char)21) ;
        p192.diag_y_SET(2.271106E38F) ;
        p192.ofs_y_SET(-2.3110702E38F) ;
        p192.diag_x_SET(-2.545963E38F) ;
        p192.autosaved_SET((char)90) ;
        p192.ofs_x_SET(-9.935474E37F) ;
        p192.diag_z_SET(-9.457495E37F) ;
        p192.offdiag_z_SET(8.953356E37F) ;
        p192.ofs_z_SET(2.953284E37F) ;
        p192.compass_id_SET((char)43) ;
        p192.offdiag_y_SET(-1.6959216E38F) ;
        p192.offdiag_x_SET(2.6489839E37F) ;
        p192.fitness_SET(5.96335E37F) ;
        CommunicationChannel.instance.send(p192);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_EKF_STATUS_REPORT.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (EKF_STATUS_FLAGS.EKF_VELOCITY_HORIZ |
                                        EKF_STATUS_FLAGS.EKF_POS_HORIZ_ABS |
                                        EKF_STATUS_FLAGS.EKF_ATTITUDE |
                                        EKF_STATUS_FLAGS.EKF_POS_VERT_ABS));
            assert(pack.pos_vert_variance_GET() == -3.1393227E38F);
            assert(pack.terrain_alt_variance_GET() == 1.6083967E38F);
            assert(pack.compass_variance_GET() == -3.2547117E37F);
            assert(pack.pos_horiz_variance_GET() == -2.1810328E38F);
            assert(pack.velocity_variance_GET() == 2.767362E38F);
        });
        GroundControl.EKF_STATUS_REPORT p193 = CommunicationChannel.new_EKF_STATUS_REPORT();
        PH.setPack(p193);
        p193.terrain_alt_variance_SET(1.6083967E38F) ;
        p193.compass_variance_SET(-3.2547117E37F) ;
        p193.flags_SET((EKF_STATUS_FLAGS.EKF_VELOCITY_HORIZ |
                        EKF_STATUS_FLAGS.EKF_POS_HORIZ_ABS |
                        EKF_STATUS_FLAGS.EKF_ATTITUDE |
                        EKF_STATUS_FLAGS.EKF_POS_VERT_ABS)) ;
        p193.pos_vert_variance_SET(-3.1393227E38F) ;
        p193.velocity_variance_SET(2.767362E38F) ;
        p193.pos_horiz_variance_SET(-2.1810328E38F) ;
        CommunicationChannel.instance.send(p193);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PID_TUNING.add((src, ph, pack) ->
        {
            assert(pack.axis_GET() == PID_TUNING_AXIS.PID_TUNING_ACCZ);
            assert(pack.desired_GET() == -3.7153724E37F);
            assert(pack.P_GET() == 1.2054062E37F);
            assert(pack.D_GET() == 2.1478265E38F);
            assert(pack.I_GET() == 1.0581641E38F);
            assert(pack.achieved_GET() == 2.0072667E38F);
            assert(pack.FF_GET() == -9.575374E37F);
        });
        GroundControl.PID_TUNING p194 = CommunicationChannel.new_PID_TUNING();
        PH.setPack(p194);
        p194.D_SET(2.1478265E38F) ;
        p194.P_SET(1.2054062E37F) ;
        p194.axis_SET(PID_TUNING_AXIS.PID_TUNING_ACCZ) ;
        p194.desired_SET(-3.7153724E37F) ;
        p194.achieved_SET(2.0072667E38F) ;
        p194.FF_SET(-9.575374E37F) ;
        p194.I_SET(1.0581641E38F) ;
        CommunicationChannel.instance.send(p194);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GIMBAL_REPORT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)169);
            assert(pack.joint_az_GET() == -3.0934344E36F);
            assert(pack.delta_velocity_z_GET() == -1.1561335E38F);
            assert(pack.delta_time_GET() == -2.4117097E38F);
            assert(pack.target_system_GET() == (char)235);
            assert(pack.joint_el_GET() == -1.2233124E38F);
            assert(pack.delta_velocity_x_GET() == 1.6841838E38F);
            assert(pack.delta_angle_z_GET() == -1.7149065E38F);
            assert(pack.delta_angle_x_GET() == -1.9119671E37F);
            assert(pack.delta_velocity_y_GET() == -1.7188636E38F);
            assert(pack.delta_angle_y_GET() == 1.940021E38F);
            assert(pack.joint_roll_GET() == -2.485777E38F);
        });
        GroundControl.GIMBAL_REPORT p200 = CommunicationChannel.new_GIMBAL_REPORT();
        PH.setPack(p200);
        p200.delta_time_SET(-2.4117097E38F) ;
        p200.joint_el_SET(-1.2233124E38F) ;
        p200.target_system_SET((char)235) ;
        p200.delta_velocity_x_SET(1.6841838E38F) ;
        p200.target_component_SET((char)169) ;
        p200.joint_roll_SET(-2.485777E38F) ;
        p200.joint_az_SET(-3.0934344E36F) ;
        p200.delta_angle_y_SET(1.940021E38F) ;
        p200.delta_angle_x_SET(-1.9119671E37F) ;
        p200.delta_velocity_z_SET(-1.1561335E38F) ;
        p200.delta_velocity_y_SET(-1.7188636E38F) ;
        p200.delta_angle_z_SET(-1.7149065E38F) ;
        CommunicationChannel.instance.send(p200);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GIMBAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.demanded_rate_z_GET() == 1.0185585E38F);
            assert(pack.target_system_GET() == (char)216);
            assert(pack.demanded_rate_x_GET() == 2.268474E38F);
            assert(pack.target_component_GET() == (char)87);
            assert(pack.demanded_rate_y_GET() == 1.1101768E38F);
        });
        GroundControl.GIMBAL_CONTROL p201 = CommunicationChannel.new_GIMBAL_CONTROL();
        PH.setPack(p201);
        p201.target_system_SET((char)216) ;
        p201.demanded_rate_y_SET(1.1101768E38F) ;
        p201.demanded_rate_x_SET(2.268474E38F) ;
        p201.demanded_rate_z_SET(1.0185585E38F) ;
        p201.target_component_SET((char)87) ;
        CommunicationChannel.instance.send(p201);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GIMBAL_TORQUE_CMD_REPORT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)131);
            assert(pack.az_torque_cmd_GET() == (short)19212);
            assert(pack.target_system_GET() == (char)180);
            assert(pack.el_torque_cmd_GET() == (short) -26135);
            assert(pack.rl_torque_cmd_GET() == (short)10638);
        });
        GroundControl.GIMBAL_TORQUE_CMD_REPORT p214 = CommunicationChannel.new_GIMBAL_TORQUE_CMD_REPORT();
        PH.setPack(p214);
        p214.target_system_SET((char)180) ;
        p214.target_component_SET((char)131) ;
        p214.rl_torque_cmd_SET((short)10638) ;
        p214.el_torque_cmd_SET((short) -26135) ;
        p214.az_torque_cmd_SET((short)19212) ;
        CommunicationChannel.instance.send(p214);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GOPRO_HEARTBEAT.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING);
            assert(pack.capture_mode_GET() == GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_PHOTO);
            assert(pack.status_GET() == GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_CONNECTED);
        });
        GroundControl.GOPRO_HEARTBEAT p215 = CommunicationChannel.new_GOPRO_HEARTBEAT();
        PH.setPack(p215);
        p215.capture_mode_SET(GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_PHOTO) ;
        p215.flags_SET(GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING) ;
        p215.status_SET(GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_CONNECTED) ;
        CommunicationChannel.instance.send(p215);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GOPRO_GET_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.cmd_id_GET() == GOPRO_COMMAND.GOPRO_COMMAND_TIME);
            assert(pack.target_component_GET() == (char)172);
            assert(pack.target_system_GET() == (char)198);
        });
        GroundControl.GOPRO_GET_REQUEST p216 = CommunicationChannel.new_GOPRO_GET_REQUEST();
        PH.setPack(p216);
        p216.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_TIME) ;
        p216.target_component_SET((char)172) ;
        p216.target_system_SET((char)198) ;
        CommunicationChannel.instance.send(p216);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GOPRO_GET_RESPONSE.add((src, ph, pack) ->
        {
            assert(pack.status_GET() == GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED);
            assert(pack.cmd_id_GET() == GOPRO_COMMAND.GOPRO_COMMAND_BATTERY);
            assert(Arrays.equals(pack.value_GET(),  new char[] {(char)147, (char)220, (char)117, (char)55}));
        });
        GroundControl.GOPRO_GET_RESPONSE p217 = CommunicationChannel.new_GOPRO_GET_RESPONSE();
        PH.setPack(p217);
        p217.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_BATTERY) ;
        p217.value_SET(new char[] {(char)147, (char)220, (char)117, (char)55}, 0) ;
        p217.status_SET(GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED) ;
        CommunicationChannel.instance.send(p217);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GOPRO_SET_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.value_GET(),  new char[] {(char)2, (char)119, (char)23, (char)179}));
            assert(pack.target_system_GET() == (char)101);
            assert(pack.cmd_id_GET() == GOPRO_COMMAND.GOPRO_COMMAND_MODEL);
            assert(pack.target_component_GET() == (char)241);
        });
        GroundControl.GOPRO_SET_REQUEST p218 = CommunicationChannel.new_GOPRO_SET_REQUEST();
        PH.setPack(p218);
        p218.value_SET(new char[] {(char)2, (char)119, (char)23, (char)179}, 0) ;
        p218.target_component_SET((char)241) ;
        p218.target_system_SET((char)101) ;
        p218.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_MODEL) ;
        CommunicationChannel.instance.send(p218);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GOPRO_SET_RESPONSE.add((src, ph, pack) ->
        {
            assert(pack.status_GET() == GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS);
            assert(pack.cmd_id_GET() == GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_COLOUR);
        });
        GroundControl.GOPRO_SET_RESPONSE p219 = CommunicationChannel.new_GOPRO_SET_RESPONSE();
        PH.setPack(p219);
        p219.status_SET(GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS) ;
        p219.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_COLOUR) ;
        CommunicationChannel.instance.send(p219);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RPM.add((src, ph, pack) ->
        {
            assert(pack.rpm1_GET() == 1.3047795E38F);
            assert(pack.rpm2_GET() == 1.2555018E38F);
        });
        GroundControl.RPM p226 = CommunicationChannel.new_RPM();
        PH.setPack(p226);
        p226.rpm2_SET(1.2555018E38F) ;
        p226.rpm1_SET(1.3047795E38F) ;
        CommunicationChannel.instance.send(p226);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        CommunicationChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.tas_ratio_GET() == -2.8287903E38F);
            assert(pack.hagl_ratio_GET() == 8.939467E37F);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS));
            assert(pack.pos_horiz_ratio_GET() == 1.795169E38F);
            assert(pack.mag_ratio_GET() == -3.084025E37F);
            assert(pack.pos_horiz_accuracy_GET() == -1.5210037E38F);
            assert(pack.vel_ratio_GET() == 4.5594806E37F);
            assert(pack.pos_vert_accuracy_GET() == 1.8216427E38F);
            assert(pack.pos_vert_ratio_GET() == -2.7650386E38F);
            assert(pack.time_usec_GET() == 841666765025753201L);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.tas_ratio_SET(-2.8287903E38F) ;
        p230.vel_ratio_SET(4.5594806E37F) ;
        p230.hagl_ratio_SET(8.939467E37F) ;
        p230.time_usec_SET(841666765025753201L) ;
        p230.pos_vert_accuracy_SET(1.8216427E38F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS)) ;
        p230.mag_ratio_SET(-3.084025E37F) ;
        p230.pos_horiz_accuracy_SET(-1.5210037E38F) ;
        p230.pos_vert_ratio_SET(-2.7650386E38F) ;
        p230.pos_horiz_ratio_SET(1.795169E38F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_y_GET() == 4.1713164E37F);
            assert(pack.var_vert_GET() == 1.536019E38F);
            assert(pack.wind_alt_GET() == -3.2569712E38F);
            assert(pack.wind_x_GET() == 1.823433E38F);
            assert(pack.var_horiz_GET() == 2.520003E38F);
            assert(pack.horiz_accuracy_GET() == -8.558673E37F);
            assert(pack.wind_z_GET() == 2.1832262E38F);
            assert(pack.vert_accuracy_GET() == -3.0189261E37F);
            assert(pack.time_usec_GET() == 888378587543450952L);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_alt_SET(-3.2569712E38F) ;
        p231.horiz_accuracy_SET(-8.558673E37F) ;
        p231.time_usec_SET(888378587543450952L) ;
        p231.var_vert_SET(1.536019E38F) ;
        p231.var_horiz_SET(2.520003E38F) ;
        p231.vert_accuracy_SET(-3.0189261E37F) ;
        p231.wind_y_SET(4.1713164E37F) ;
        p231.wind_x_SET(1.823433E38F) ;
        p231.wind_z_SET(2.1832262E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.satellites_visible_GET() == (char)46);
            assert(pack.fix_type_GET() == (char)211);
            assert(pack.ve_GET() == 2.3580056E38F);
            assert(pack.hdop_GET() == -5.555707E37F);
            assert(pack.time_usec_GET() == 893345376373471939L);
            assert(pack.vert_accuracy_GET() == 6.88504E37F);
            assert(pack.alt_GET() == 1.3434211E36F);
            assert(pack.gps_id_GET() == (char)102);
            assert(pack.time_week_ms_GET() == 995267868L);
            assert(pack.vdop_GET() == -1.6967752E38F);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY));
            assert(pack.horiz_accuracy_GET() == -2.2506153E38F);
            assert(pack.vd_GET() == 1.4902078E38F);
            assert(pack.lon_GET() == -1077287901);
            assert(pack.speed_accuracy_GET() == -2.4954402E38F);
            assert(pack.vn_GET() == -1.9627663E38F);
            assert(pack.lat_GET() == 1641528654);
            assert(pack.time_week_GET() == (char)18788);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.speed_accuracy_SET(-2.4954402E38F) ;
        p232.hdop_SET(-5.555707E37F) ;
        p232.lon_SET(-1077287901) ;
        p232.vd_SET(1.4902078E38F) ;
        p232.time_week_ms_SET(995267868L) ;
        p232.vert_accuracy_SET(6.88504E37F) ;
        p232.vn_SET(-1.9627663E38F) ;
        p232.time_week_SET((char)18788) ;
        p232.fix_type_SET((char)211) ;
        p232.vdop_SET(-1.6967752E38F) ;
        p232.horiz_accuracy_SET(-2.2506153E38F) ;
        p232.time_usec_SET(893345376373471939L) ;
        p232.satellites_visible_SET((char)46) ;
        p232.lat_SET(1641528654) ;
        p232.gps_id_SET((char)102) ;
        p232.ve_SET(2.3580056E38F) ;
        p232.alt_SET(1.3434211E36F) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY)) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)208, (char)100, (char)19, (char)24, (char)46, (char)74, (char)83, (char)117, (char)198, (char)242, (char)198, (char)183, (char)91, (char)43, (char)146, (char)229, (char)123, (char)239, (char)133, (char)134, (char)113, (char)255, (char)178, (char)37, (char)125, (char)22, (char)209, (char)162, (char)128, (char)1, (char)30, (char)251, (char)204, (char)179, (char)75, (char)131, (char)184, (char)108, (char)58, (char)247, (char)16, (char)48, (char)170, (char)111, (char)140, (char)219, (char)212, (char)114, (char)124, (char)194, (char)7, (char)251, (char)109, (char)250, (char)147, (char)63, (char)240, (char)83, (char)130, (char)200, (char)229, (char)156, (char)231, (char)121, (char)182, (char)201, (char)216, (char)225, (char)174, (char)192, (char)11, (char)171, (char)224, (char)138, (char)182, (char)9, (char)205, (char)91, (char)25, (char)144, (char)238, (char)207, (char)48, (char)17, (char)251, (char)235, (char)57, (char)26, (char)58, (char)148, (char)46, (char)238, (char)170, (char)128, (char)123, (char)179, (char)151, (char)80, (char)118, (char)20, (char)202, (char)125, (char)40, (char)115, (char)241, (char)196, (char)6, (char)159, (char)139, (char)94, (char)178, (char)139, (char)111, (char)235, (char)203, (char)192, (char)138, (char)243, (char)7, (char)250, (char)196, (char)166, (char)174, (char)229, (char)130, (char)156, (char)31, (char)181, (char)217, (char)20, (char)165, (char)57, (char)40, (char)241, (char)236, (char)62, (char)223, (char)54, (char)31, (char)232, (char)103, (char)224, (char)188, (char)113, (char)53, (char)105, (char)173, (char)103, (char)191, (char)246, (char)5, (char)73, (char)34, (char)171, (char)139, (char)39, (char)248, (char)202, (char)244, (char)200, (char)182, (char)40, (char)26, (char)144, (char)6, (char)144, (char)201, (char)248, (char)60, (char)87, (char)74, (char)3, (char)218, (char)128, (char)193, (char)66, (char)79, (char)223, (char)13, (char)105}));
            assert(pack.flags_GET() == (char)134);
            assert(pack.len_GET() == (char)227);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)134) ;
        p233.len_SET((char)227) ;
        p233.data__SET(new char[] {(char)208, (char)100, (char)19, (char)24, (char)46, (char)74, (char)83, (char)117, (char)198, (char)242, (char)198, (char)183, (char)91, (char)43, (char)146, (char)229, (char)123, (char)239, (char)133, (char)134, (char)113, (char)255, (char)178, (char)37, (char)125, (char)22, (char)209, (char)162, (char)128, (char)1, (char)30, (char)251, (char)204, (char)179, (char)75, (char)131, (char)184, (char)108, (char)58, (char)247, (char)16, (char)48, (char)170, (char)111, (char)140, (char)219, (char)212, (char)114, (char)124, (char)194, (char)7, (char)251, (char)109, (char)250, (char)147, (char)63, (char)240, (char)83, (char)130, (char)200, (char)229, (char)156, (char)231, (char)121, (char)182, (char)201, (char)216, (char)225, (char)174, (char)192, (char)11, (char)171, (char)224, (char)138, (char)182, (char)9, (char)205, (char)91, (char)25, (char)144, (char)238, (char)207, (char)48, (char)17, (char)251, (char)235, (char)57, (char)26, (char)58, (char)148, (char)46, (char)238, (char)170, (char)128, (char)123, (char)179, (char)151, (char)80, (char)118, (char)20, (char)202, (char)125, (char)40, (char)115, (char)241, (char)196, (char)6, (char)159, (char)139, (char)94, (char)178, (char)139, (char)111, (char)235, (char)203, (char)192, (char)138, (char)243, (char)7, (char)250, (char)196, (char)166, (char)174, (char)229, (char)130, (char)156, (char)31, (char)181, (char)217, (char)20, (char)165, (char)57, (char)40, (char)241, (char)236, (char)62, (char)223, (char)54, (char)31, (char)232, (char)103, (char)224, (char)188, (char)113, (char)53, (char)105, (char)173, (char)103, (char)191, (char)246, (char)5, (char)73, (char)34, (char)171, (char)139, (char)39, (char)248, (char)202, (char)244, (char)200, (char)182, (char)40, (char)26, (char)144, (char)6, (char)144, (char)201, (char)248, (char)60, (char)87, (char)74, (char)3, (char)218, (char)128, (char)193, (char)66, (char)79, (char)223, (char)13, (char)105}, 0) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.heading_GET() == (char)32208);
            assert(pack.climb_rate_GET() == (byte)21);
            assert(pack.temperature_GET() == (byte) - 44);
            assert(pack.wp_distance_GET() == (char)8783);
            assert(pack.pitch_GET() == (short) -20838);
            assert(pack.altitude_amsl_GET() == (short) -8258);
            assert(pack.latitude_GET() == 1042978165);
            assert(pack.battery_remaining_GET() == (char)197);
            assert(pack.failsafe_GET() == (char)242);
            assert(pack.roll_GET() == (short) -24172);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
            assert(pack.groundspeed_GET() == (char)164);
            assert(pack.custom_mode_GET() == 2867522807L);
            assert(pack.temperature_air_GET() == (byte) - 86);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED));
            assert(pack.airspeed_sp_GET() == (char)203);
            assert(pack.throttle_GET() == (byte)104);
            assert(pack.longitude_GET() == -1061067168);
            assert(pack.airspeed_GET() == (char)129);
            assert(pack.heading_sp_GET() == (short)21974);
            assert(pack.wp_num_GET() == (char)121);
            assert(pack.gps_nsat_GET() == (char)24);
            assert(pack.altitude_sp_GET() == (short) -18948);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.wp_distance_SET((char)8783) ;
        p234.temperature_SET((byte) - 44) ;
        p234.latitude_SET(1042978165) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT) ;
        p234.roll_SET((short) -24172) ;
        p234.throttle_SET((byte)104) ;
        p234.heading_SET((char)32208) ;
        p234.altitude_sp_SET((short) -18948) ;
        p234.airspeed_sp_SET((char)203) ;
        p234.wp_num_SET((char)121) ;
        p234.heading_sp_SET((short)21974) ;
        p234.altitude_amsl_SET((short) -8258) ;
        p234.longitude_SET(-1061067168) ;
        p234.custom_mode_SET(2867522807L) ;
        p234.temperature_air_SET((byte) - 86) ;
        p234.failsafe_SET((char)242) ;
        p234.gps_nsat_SET((char)24) ;
        p234.airspeed_SET((char)129) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED)) ;
        p234.climb_rate_SET((byte)21) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED) ;
        p234.pitch_SET((short) -20838) ;
        p234.battery_remaining_SET((char)197) ;
        p234.groundspeed_SET((char)164) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.vibration_z_GET() == -2.785158E38F);
            assert(pack.time_usec_GET() == 2998389673358662463L);
            assert(pack.clipping_2_GET() == 1855897643L);
            assert(pack.clipping_1_GET() == 3433522058L);
            assert(pack.vibration_x_GET() == 3.2167707E38F);
            assert(pack.clipping_0_GET() == 769120636L);
            assert(pack.vibration_y_GET() == -5.584865E37F);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.clipping_2_SET(1855897643L) ;
        p241.vibration_y_SET(-5.584865E37F) ;
        p241.vibration_z_SET(-2.785158E38F) ;
        p241.vibration_x_SET(3.2167707E38F) ;
        p241.clipping_1_SET(3433522058L) ;
        p241.time_usec_SET(2998389673358662463L) ;
        p241.clipping_0_SET(769120636L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.6916885E38F, -3.1534143E38F, -1.3461866E38F, -1.9901924E38F}));
            assert(pack.x_GET() == 3.1018536E38F);
            assert(pack.z_GET() == -2.727637E38F);
            assert(pack.y_GET() == -1.2878598E38F);
            assert(pack.longitude_GET() == 877526845);
            assert(pack.altitude_GET() == 1998510530);
            assert(pack.approach_x_GET() == -3.2676712E38F);
            assert(pack.approach_z_GET() == 1.711485E38F);
            assert(pack.latitude_GET() == -223595151);
            assert(pack.time_usec_TRY(ph) == 1146509776084404698L);
            assert(pack.approach_y_GET() == 8.110108E37F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.z_SET(-2.727637E38F) ;
        p242.altitude_SET(1998510530) ;
        p242.q_SET(new float[] {1.6916885E38F, -3.1534143E38F, -1.3461866E38F, -1.9901924E38F}, 0) ;
        p242.approach_y_SET(8.110108E37F) ;
        p242.longitude_SET(877526845) ;
        p242.approach_z_SET(1.711485E38F) ;
        p242.approach_x_SET(-3.2676712E38F) ;
        p242.time_usec_SET(1146509776084404698L, PH) ;
        p242.x_SET(3.1018536E38F) ;
        p242.y_SET(-1.2878598E38F) ;
        p242.latitude_SET(-223595151) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.approach_y_GET() == -1.7400742E38F);
            assert(pack.x_GET() == -1.7848005E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.4036737E38F, 1.783278E38F, -1.1970942E38F, -3.0219137E37F}));
            assert(pack.altitude_GET() == -400445701);
            assert(pack.latitude_GET() == -1707570950);
            assert(pack.target_system_GET() == (char)157);
            assert(pack.longitude_GET() == 2066539085);
            assert(pack.approach_x_GET() == -6.8590774E37F);
            assert(pack.z_GET() == 7.842415E37F);
            assert(pack.time_usec_TRY(ph) == 577095112697335940L);
            assert(pack.y_GET() == 2.164819E38F);
            assert(pack.approach_z_GET() == 4.9784954E37F);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.q_SET(new float[] {-1.4036737E38F, 1.783278E38F, -1.1970942E38F, -3.0219137E37F}, 0) ;
        p243.approach_z_SET(4.9784954E37F) ;
        p243.latitude_SET(-1707570950) ;
        p243.z_SET(7.842415E37F) ;
        p243.altitude_SET(-400445701) ;
        p243.approach_y_SET(-1.7400742E38F) ;
        p243.time_usec_SET(577095112697335940L, PH) ;
        p243.longitude_SET(2066539085) ;
        p243.target_system_SET((char)157) ;
        p243.x_SET(-1.7848005E38F) ;
        p243.approach_x_SET(-6.8590774E37F) ;
        p243.y_SET(2.164819E38F) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.interval_us_GET() == -1299362547);
            assert(pack.message_id_GET() == (char)58159);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(-1299362547) ;
        p244.message_id_SET((char)58159) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.callsign_LEN(ph) == 3);
            assert(pack.callsign_TRY(ph).equals("veb"));
            assert(pack.squawk_GET() == (char)61391);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            assert(pack.ver_velocity_GET() == (short)5903);
            assert(pack.altitude_GET() == 24581571);
            assert(pack.lon_GET() == 878861350);
            assert(pack.tslc_GET() == (char)13);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_PARACHUTE);
            assert(pack.lat_GET() == 1529828206);
            assert(pack.ICAO_address_GET() == 783944164L);
            assert(pack.hor_velocity_GET() == (char)57937);
            assert(pack.heading_GET() == (char)20912);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED));
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.heading_SET((char)20912) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_PARACHUTE) ;
        p246.hor_velocity_SET((char)57937) ;
        p246.callsign_SET("veb", PH) ;
        p246.altitude_SET(24581571) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.lat_SET(1529828206) ;
        p246.lon_SET(878861350) ;
        p246.ver_velocity_SET((short)5903) ;
        p246.ICAO_address_SET(783944164L) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED)) ;
        p246.squawk_SET((char)61391) ;
        p246.tslc_SET((char)13) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.horizontal_minimum_delta_GET() == 1.8669842E38F);
            assert(pack.id_GET() == 495649327L);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
            assert(pack.altitude_minimum_delta_GET() == -1.2648125E38F);
            assert(pack.time_to_minimum_delta_GET() == -1.7653779E38F);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE) ;
        p247.altitude_minimum_delta_SET(-1.2648125E38F) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND) ;
        p247.time_to_minimum_delta_SET(-1.7653779E38F) ;
        p247.id_SET(495649327L) ;
        p247.horizontal_minimum_delta_SET(1.8669842E38F) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)200, (char)220, (char)249, (char)237, (char)103, (char)174, (char)69, (char)111, (char)55, (char)173, (char)18, (char)121, (char)253, (char)129, (char)250, (char)34, (char)17, (char)179, (char)181, (char)56, (char)62, (char)204, (char)201, (char)174, (char)0, (char)82, (char)240, (char)11, (char)41, (char)152, (char)81, (char)94, (char)42, (char)100, (char)19, (char)109, (char)6, (char)238, (char)239, (char)35, (char)169, (char)49, (char)165, (char)181, (char)181, (char)43, (char)28, (char)74, (char)150, (char)112, (char)170, (char)99, (char)55, (char)194, (char)83, (char)167, (char)16, (char)152, (char)118, (char)21, (char)124, (char)3, (char)213, (char)0, (char)198, (char)0, (char)15, (char)108, (char)230, (char)61, (char)61, (char)123, (char)42, (char)24, (char)201, (char)93, (char)189, (char)99, (char)37, (char)222, (char)50, (char)182, (char)90, (char)227, (char)242, (char)135, (char)252, (char)192, (char)162, (char)126, (char)238, (char)90, (char)222, (char)226, (char)224, (char)151, (char)242, (char)250, (char)255, (char)166, (char)125, (char)114, (char)231, (char)136, (char)35, (char)231, (char)194, (char)174, (char)16, (char)206, (char)142, (char)165, (char)18, (char)93, (char)9, (char)226, (char)75, (char)85, (char)35, (char)97, (char)202, (char)87, (char)67, (char)181, (char)107, (char)147, (char)161, (char)209, (char)87, (char)172, (char)65, (char)245, (char)149, (char)202, (char)86, (char)196, (char)240, (char)9, (char)72, (char)176, (char)19, (char)176, (char)185, (char)248, (char)15, (char)53, (char)83, (char)160, (char)86, (char)163, (char)126, (char)19, (char)254, (char)19, (char)57, (char)32, (char)13, (char)110, (char)144, (char)78, (char)141, (char)149, (char)185, (char)126, (char)218, (char)50, (char)156, (char)9, (char)57, (char)153, (char)181, (char)193, (char)142, (char)57, (char)17, (char)76, (char)42, (char)143, (char)60, (char)23, (char)53, (char)59, (char)105, (char)35, (char)28, (char)248, (char)96, (char)96, (char)4, (char)56, (char)101, (char)169, (char)176, (char)209, (char)215, (char)209, (char)85, (char)93, (char)101, (char)193, (char)73, (char)72, (char)199, (char)148, (char)107, (char)205, (char)7, (char)20, (char)209, (char)192, (char)172, (char)44, (char)96, (char)1, (char)17, (char)35, (char)14, (char)46, (char)163, (char)67, (char)252, (char)10, (char)179, (char)254, (char)60, (char)169, (char)121, (char)188, (char)54, (char)186, (char)151, (char)227, (char)156, (char)252, (char)245, (char)244, (char)233, (char)240, (char)163, (char)102, (char)253, (char)118, (char)28, (char)105, (char)39, (char)232, (char)203, (char)254, (char)248}));
            assert(pack.message_type_GET() == (char)13516);
            assert(pack.target_system_GET() == (char)65);
            assert(pack.target_network_GET() == (char)255);
            assert(pack.target_component_GET() == (char)131);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.payload_SET(new char[] {(char)200, (char)220, (char)249, (char)237, (char)103, (char)174, (char)69, (char)111, (char)55, (char)173, (char)18, (char)121, (char)253, (char)129, (char)250, (char)34, (char)17, (char)179, (char)181, (char)56, (char)62, (char)204, (char)201, (char)174, (char)0, (char)82, (char)240, (char)11, (char)41, (char)152, (char)81, (char)94, (char)42, (char)100, (char)19, (char)109, (char)6, (char)238, (char)239, (char)35, (char)169, (char)49, (char)165, (char)181, (char)181, (char)43, (char)28, (char)74, (char)150, (char)112, (char)170, (char)99, (char)55, (char)194, (char)83, (char)167, (char)16, (char)152, (char)118, (char)21, (char)124, (char)3, (char)213, (char)0, (char)198, (char)0, (char)15, (char)108, (char)230, (char)61, (char)61, (char)123, (char)42, (char)24, (char)201, (char)93, (char)189, (char)99, (char)37, (char)222, (char)50, (char)182, (char)90, (char)227, (char)242, (char)135, (char)252, (char)192, (char)162, (char)126, (char)238, (char)90, (char)222, (char)226, (char)224, (char)151, (char)242, (char)250, (char)255, (char)166, (char)125, (char)114, (char)231, (char)136, (char)35, (char)231, (char)194, (char)174, (char)16, (char)206, (char)142, (char)165, (char)18, (char)93, (char)9, (char)226, (char)75, (char)85, (char)35, (char)97, (char)202, (char)87, (char)67, (char)181, (char)107, (char)147, (char)161, (char)209, (char)87, (char)172, (char)65, (char)245, (char)149, (char)202, (char)86, (char)196, (char)240, (char)9, (char)72, (char)176, (char)19, (char)176, (char)185, (char)248, (char)15, (char)53, (char)83, (char)160, (char)86, (char)163, (char)126, (char)19, (char)254, (char)19, (char)57, (char)32, (char)13, (char)110, (char)144, (char)78, (char)141, (char)149, (char)185, (char)126, (char)218, (char)50, (char)156, (char)9, (char)57, (char)153, (char)181, (char)193, (char)142, (char)57, (char)17, (char)76, (char)42, (char)143, (char)60, (char)23, (char)53, (char)59, (char)105, (char)35, (char)28, (char)248, (char)96, (char)96, (char)4, (char)56, (char)101, (char)169, (char)176, (char)209, (char)215, (char)209, (char)85, (char)93, (char)101, (char)193, (char)73, (char)72, (char)199, (char)148, (char)107, (char)205, (char)7, (char)20, (char)209, (char)192, (char)172, (char)44, (char)96, (char)1, (char)17, (char)35, (char)14, (char)46, (char)163, (char)67, (char)252, (char)10, (char)179, (char)254, (char)60, (char)169, (char)121, (char)188, (char)54, (char)186, (char)151, (char)227, (char)156, (char)252, (char)245, (char)244, (char)233, (char)240, (char)163, (char)102, (char)253, (char)118, (char)28, (char)105, (char)39, (char)232, (char)203, (char)254, (char)248}, 0) ;
        p248.target_system_SET((char)65) ;
        p248.target_network_SET((char)255) ;
        p248.target_component_SET((char)131) ;
        p248.message_type_SET((char)13516) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)114, (byte) - 97, (byte) - 8, (byte) - 121, (byte)44, (byte)42, (byte)36, (byte)112, (byte)127, (byte) - 19, (byte) - 65, (byte) - 36, (byte) - 106, (byte) - 56, (byte)65, (byte)82, (byte) - 28, (byte) - 97, (byte)116, (byte)117, (byte)24, (byte)122, (byte) - 93, (byte) - 106, (byte) - 38, (byte)16, (byte)7, (byte)56, (byte)34, (byte) - 49, (byte)94, (byte)8}));
            assert(pack.ver_GET() == (char)204);
            assert(pack.address_GET() == (char)42258);
            assert(pack.type_GET() == (char)83);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.ver_SET((char)204) ;
        p249.type_SET((char)83) ;
        p249.value_SET(new byte[] {(byte)114, (byte) - 97, (byte) - 8, (byte) - 121, (byte)44, (byte)42, (byte)36, (byte)112, (byte)127, (byte) - 19, (byte) - 65, (byte) - 36, (byte) - 106, (byte) - 56, (byte)65, (byte)82, (byte) - 28, (byte) - 97, (byte)116, (byte)117, (byte)24, (byte)122, (byte) - 93, (byte) - 106, (byte) - 38, (byte)16, (byte)7, (byte)56, (byte)34, (byte) - 49, (byte)94, (byte)8}, 0) ;
        p249.address_SET((char)42258) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 9);
            assert(pack.name_TRY(ph).equals("ygFWqkjnv"));
            assert(pack.z_GET() == -1.3875085E38F);
            assert(pack.time_usec_GET() == 3091891808398664785L);
            assert(pack.y_GET() == -1.476369E38F);
            assert(pack.x_GET() == -3.2473854E38F);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.y_SET(-1.476369E38F) ;
        p250.x_SET(-3.2473854E38F) ;
        p250.z_SET(-1.3875085E38F) ;
        p250.time_usec_SET(3091891808398664785L) ;
        p250.name_SET("ygFWqkjnv", PH) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 9);
            assert(pack.name_TRY(ph).equals("ztmonusXo"));
            assert(pack.value_GET() == -9.079907E37F);
            assert(pack.time_boot_ms_GET() == 1563979727L);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.value_SET(-9.079907E37F) ;
        p251.name_SET("ztmonusXo", PH) ;
        p251.time_boot_ms_SET(1563979727L) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 991919109L);
            assert(pack.name_LEN(ph) == 5);
            assert(pack.name_TRY(ph).equals("eobas"));
            assert(pack.value_GET() == -66862068);
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.value_SET(-66862068) ;
        p252.name_SET("eobas", PH) ;
        p252.time_boot_ms_SET(991919109L) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_INFO);
            assert(pack.text_LEN(ph) == 31);
            assert(pack.text_TRY(ph).equals("IbgaqUdpyjmznorkAGqvcxhsHqitaEb"));
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("IbgaqUdpyjmznorkAGqvcxhsHqitaEb", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_INFO) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.ind_GET() == (char)15);
            assert(pack.time_boot_ms_GET() == 1604329176L);
            assert(pack.value_GET() == -1.1293296E38F);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.value_SET(-1.1293296E38F) ;
        p254.time_boot_ms_SET(1604329176L) ;
        p254.ind_SET((char)15) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)249);
            assert(pack.target_component_GET() == (char)165);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)4, (char)128, (char)72, (char)3, (char)196, (char)206, (char)109, (char)123, (char)77, (char)188, (char)152, (char)143, (char)34, (char)170, (char)149, (char)246, (char)92, (char)62, (char)131, (char)229, (char)180, (char)247, (char)125, (char)186, (char)15, (char)230, (char)105, (char)174, (char)21, (char)186, (char)209, (char)104}));
            assert(pack.initial_timestamp_GET() == 1405383449973171161L);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)249) ;
        p256.target_component_SET((char)165) ;
        p256.secret_key_SET(new char[] {(char)4, (char)128, (char)72, (char)3, (char)196, (char)206, (char)109, (char)123, (char)77, (char)188, (char)152, (char)143, (char)34, (char)170, (char)149, (char)246, (char)92, (char)62, (char)131, (char)229, (char)180, (char)247, (char)125, (char)186, (char)15, (char)230, (char)105, (char)174, (char)21, (char)186, (char)209, (char)104}, 0) ;
        p256.initial_timestamp_SET(1405383449973171161L) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1971654321L);
            assert(pack.state_GET() == (char)53);
            assert(pack.last_change_ms_GET() == 1015479380L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(1971654321L) ;
        p257.state_SET((char)53) ;
        p257.last_change_ms_SET(1015479380L) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.tune_LEN(ph) == 2);
            assert(pack.tune_TRY(ph).equals("hS"));
            assert(pack.target_component_GET() == (char)103);
            assert(pack.target_system_GET() == (char)185);
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.tune_SET("hS", PH) ;
        p258.target_system_SET((char)185) ;
        p258.target_component_SET((char)103) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.firmware_version_GET() == 2037320832L);
            assert(pack.time_boot_ms_GET() == 1819719184L);
            assert(pack.sensor_size_h_GET() == -1.3400554E38F);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)117, (char)94, (char)122, (char)157, (char)134, (char)35, (char)8, (char)179, (char)29, (char)129, (char)76, (char)222, (char)97, (char)48, (char)53, (char)216, (char)0, (char)202, (char)158, (char)161, (char)105, (char)171, (char)248, (char)48, (char)77, (char)33, (char)82, (char)193, (char)93, (char)207, (char)114, (char)212}));
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)205, (char)55, (char)91, (char)199, (char)113, (char)4, (char)69, (char)152, (char)155, (char)233, (char)252, (char)38, (char)250, (char)89, (char)217, (char)53, (char)127, (char)109, (char)145, (char)39, (char)41, (char)105, (char)249, (char)104, (char)174, (char)24, (char)151, (char)212, (char)73, (char)187, (char)189, (char)10}));
            assert(pack.resolution_h_GET() == (char)28701);
            assert(pack.focal_length_GET() == -8.068563E37F);
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE));
            assert(pack.cam_definition_version_GET() == (char)28018);
            assert(pack.resolution_v_GET() == (char)56720);
            assert(pack.cam_definition_uri_LEN(ph) == 70);
            assert(pack.cam_definition_uri_TRY(ph).equals("gfmgorryHiycrecAnuidnuvUOyrkswuchsfcgxyrGbBpqwkhntfaglysqLqiakdZvcwugc"));
            assert(pack.sensor_size_v_GET() == 1.3803918E38F);
            assert(pack.lens_id_GET() == (char)173);
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.sensor_size_v_SET(1.3803918E38F) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE)) ;
        p259.resolution_v_SET((char)56720) ;
        p259.resolution_h_SET((char)28701) ;
        p259.time_boot_ms_SET(1819719184L) ;
        p259.model_name_SET(new char[] {(char)205, (char)55, (char)91, (char)199, (char)113, (char)4, (char)69, (char)152, (char)155, (char)233, (char)252, (char)38, (char)250, (char)89, (char)217, (char)53, (char)127, (char)109, (char)145, (char)39, (char)41, (char)105, (char)249, (char)104, (char)174, (char)24, (char)151, (char)212, (char)73, (char)187, (char)189, (char)10}, 0) ;
        p259.firmware_version_SET(2037320832L) ;
        p259.sensor_size_h_SET(-1.3400554E38F) ;
        p259.lens_id_SET((char)173) ;
        p259.cam_definition_version_SET((char)28018) ;
        p259.cam_definition_uri_SET("gfmgorryHiycrecAnuidnuvUOyrkswuchsfcgxyrGbBpqwkhntfaglysqLqiakdZvcwugc", PH) ;
        p259.focal_length_SET(-8.068563E37F) ;
        p259.vendor_name_SET(new char[] {(char)117, (char)94, (char)122, (char)157, (char)134, (char)35, (char)8, (char)179, (char)29, (char)129, (char)76, (char)222, (char)97, (char)48, (char)53, (char)216, (char)0, (char)202, (char)158, (char)161, (char)105, (char)171, (char)248, (char)48, (char)77, (char)33, (char)82, (char)193, (char)93, (char)207, (char)114, (char)212}, 0) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE);
            assert(pack.time_boot_ms_GET() == 3647826865L);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE) ;
        p260.time_boot_ms_SET(3647826865L) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.read_speed_GET() == 3.0761684E38F);
            assert(pack.used_capacity_GET() == 9.76174E37F);
            assert(pack.storage_count_GET() == (char)206);
            assert(pack.storage_id_GET() == (char)85);
            assert(pack.status_GET() == (char)255);
            assert(pack.time_boot_ms_GET() == 4013380077L);
            assert(pack.total_capacity_GET() == 3.08944E38F);
            assert(pack.write_speed_GET() == 3.4027507E37F);
            assert(pack.available_capacity_GET() == 3.445306E37F);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.time_boot_ms_SET(4013380077L) ;
        p261.available_capacity_SET(3.445306E37F) ;
        p261.storage_count_SET((char)206) ;
        p261.total_capacity_SET(3.08944E38F) ;
        p261.used_capacity_SET(9.76174E37F) ;
        p261.storage_id_SET((char)85) ;
        p261.status_SET((char)255) ;
        p261.read_speed_SET(3.0761684E38F) ;
        p261.write_speed_SET(3.4027507E37F) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3873457739L);
            assert(pack.available_capacity_GET() == -4.5186516E37F);
            assert(pack.recording_time_ms_GET() == 4140994515L);
            assert(pack.image_status_GET() == (char)145);
            assert(pack.image_interval_GET() == -2.9229926E38F);
            assert(pack.video_status_GET() == (char)224);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.video_status_SET((char)224) ;
        p262.available_capacity_SET(-4.5186516E37F) ;
        p262.image_status_SET((char)145) ;
        p262.image_interval_SET(-2.9229926E38F) ;
        p262.time_boot_ms_SET(3873457739L) ;
        p262.recording_time_ms_SET(4140994515L) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.2090218E38F, 2.0065538E38F, -2.0182431E37F, 2.9071185E38F}));
            assert(pack.lat_GET() == -446168650);
            assert(pack.capture_result_GET() == (byte)79);
            assert(pack.alt_GET() == -1293692276);
            assert(pack.lon_GET() == 820501020);
            assert(pack.time_utc_GET() == 987063874796745032L);
            assert(pack.file_url_LEN(ph) == 99);
            assert(pack.file_url_TRY(ph).equals("xnivtpxhBihpktaRieswgawqoddscsrkjcLaSoDWzhktaqkxhrlhgccwinngdIclizsnlKcrrYCvnqwwaiwceoXwmgjVjNqHqph"));
            assert(pack.time_boot_ms_GET() == 2394461266L);
            assert(pack.image_index_GET() == 81265778);
            assert(pack.relative_alt_GET() == -1486234764);
            assert(pack.camera_id_GET() == (char)234);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.lat_SET(-446168650) ;
        p263.lon_SET(820501020) ;
        p263.q_SET(new float[] {3.2090218E38F, 2.0065538E38F, -2.0182431E37F, 2.9071185E38F}, 0) ;
        p263.capture_result_SET((byte)79) ;
        p263.image_index_SET(81265778) ;
        p263.camera_id_SET((char)234) ;
        p263.time_boot_ms_SET(2394461266L) ;
        p263.relative_alt_SET(-1486234764) ;
        p263.time_utc_SET(987063874796745032L) ;
        p263.file_url_SET("xnivtpxhBihpktaRieswgawqoddscsrkjcLaSoDWzhktaqkxhrlhgccwinngdIclizsnlKcrrYCvnqwwaiwceoXwmgjVjNqHqph", PH) ;
        p263.alt_SET(-1293692276) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.arming_time_utc_GET() == 9110455947668487456L);
            assert(pack.takeoff_time_utc_GET() == 8734049119091199614L);
            assert(pack.time_boot_ms_GET() == 1952426275L);
            assert(pack.flight_uuid_GET() == 4586522802065498723L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.arming_time_utc_SET(9110455947668487456L) ;
        p264.time_boot_ms_SET(1952426275L) ;
        p264.flight_uuid_SET(4586522802065498723L) ;
        p264.takeoff_time_utc_SET(8734049119091199614L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -1.526647E38F);
            assert(pack.roll_GET() == -1.477284E38F);
            assert(pack.time_boot_ms_GET() == 524027374L);
            assert(pack.yaw_GET() == -9.340645E37F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.roll_SET(-1.477284E38F) ;
        p265.pitch_SET(-1.526647E38F) ;
        p265.time_boot_ms_SET(524027374L) ;
        p265.yaw_SET(-9.340645E37F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.first_message_offset_GET() == (char)147);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)141, (char)233, (char)82, (char)46, (char)169, (char)129, (char)110, (char)230, (char)223, (char)119, (char)91, (char)230, (char)5, (char)139, (char)224, (char)231, (char)64, (char)3, (char)245, (char)212, (char)157, (char)249, (char)99, (char)159, (char)247, (char)132, (char)172, (char)160, (char)110, (char)97, (char)8, (char)15, (char)106, (char)217, (char)167, (char)226, (char)116, (char)38, (char)254, (char)152, (char)90, (char)220, (char)197, (char)204, (char)177, (char)161, (char)15, (char)104, (char)102, (char)16, (char)94, (char)67, (char)174, (char)116, (char)172, (char)33, (char)4, (char)225, (char)1, (char)91, (char)131, (char)177, (char)193, (char)197, (char)5, (char)76, (char)175, (char)87, (char)32, (char)225, (char)247, (char)90, (char)235, (char)76, (char)245, (char)27, (char)207, (char)131, (char)233, (char)182, (char)220, (char)78, (char)13, (char)231, (char)223, (char)99, (char)104, (char)65, (char)105, (char)72, (char)128, (char)143, (char)241, (char)221, (char)159, (char)0, (char)75, (char)156, (char)27, (char)199, (char)225, (char)77, (char)66, (char)44, (char)128, (char)141, (char)29, (char)149, (char)19, (char)248, (char)118, (char)243, (char)193, (char)171, (char)242, (char)147, (char)237, (char)4, (char)47, (char)40, (char)126, (char)149, (char)195, (char)119, (char)182, (char)140, (char)78, (char)144, (char)1, (char)62, (char)24, (char)95, (char)135, (char)190, (char)19, (char)171, (char)41, (char)242, (char)213, (char)210, (char)125, (char)9, (char)228, (char)82, (char)169, (char)132, (char)30, (char)143, (char)179, (char)169, (char)223, (char)10, (char)23, (char)100, (char)245, (char)73, (char)65, (char)163, (char)146, (char)220, (char)246, (char)120, (char)65, (char)23, (char)93, (char)0, (char)18, (char)143, (char)59, (char)245, (char)219, (char)150, (char)232, (char)89, (char)6, (char)55, (char)78, (char)153, (char)25, (char)117, (char)175, (char)233, (char)23, (char)224, (char)112, (char)29, (char)199, (char)169, (char)98, (char)84, (char)53, (char)45, (char)178, (char)134, (char)140, (char)156, (char)78, (char)29, (char)220, (char)56, (char)49, (char)100, (char)99, (char)136, (char)41, (char)201, (char)127, (char)253, (char)31, (char)188, (char)149, (char)100, (char)59, (char)5, (char)76, (char)153, (char)102, (char)103, (char)192, (char)48, (char)200, (char)15, (char)118, (char)19, (char)69, (char)52, (char)86, (char)127, (char)61, (char)252, (char)174, (char)222, (char)23, (char)106, (char)65, (char)121, (char)53, (char)241, (char)245, (char)113, (char)220, (char)211, (char)104, (char)153, (char)121, (char)150, (char)48, (char)218, (char)149}));
            assert(pack.target_system_GET() == (char)217);
            assert(pack.sequence_GET() == (char)22108);
            assert(pack.length_GET() == (char)26);
            assert(pack.target_component_GET() == (char)61);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_system_SET((char)217) ;
        p266.first_message_offset_SET((char)147) ;
        p266.sequence_SET((char)22108) ;
        p266.data__SET(new char[] {(char)141, (char)233, (char)82, (char)46, (char)169, (char)129, (char)110, (char)230, (char)223, (char)119, (char)91, (char)230, (char)5, (char)139, (char)224, (char)231, (char)64, (char)3, (char)245, (char)212, (char)157, (char)249, (char)99, (char)159, (char)247, (char)132, (char)172, (char)160, (char)110, (char)97, (char)8, (char)15, (char)106, (char)217, (char)167, (char)226, (char)116, (char)38, (char)254, (char)152, (char)90, (char)220, (char)197, (char)204, (char)177, (char)161, (char)15, (char)104, (char)102, (char)16, (char)94, (char)67, (char)174, (char)116, (char)172, (char)33, (char)4, (char)225, (char)1, (char)91, (char)131, (char)177, (char)193, (char)197, (char)5, (char)76, (char)175, (char)87, (char)32, (char)225, (char)247, (char)90, (char)235, (char)76, (char)245, (char)27, (char)207, (char)131, (char)233, (char)182, (char)220, (char)78, (char)13, (char)231, (char)223, (char)99, (char)104, (char)65, (char)105, (char)72, (char)128, (char)143, (char)241, (char)221, (char)159, (char)0, (char)75, (char)156, (char)27, (char)199, (char)225, (char)77, (char)66, (char)44, (char)128, (char)141, (char)29, (char)149, (char)19, (char)248, (char)118, (char)243, (char)193, (char)171, (char)242, (char)147, (char)237, (char)4, (char)47, (char)40, (char)126, (char)149, (char)195, (char)119, (char)182, (char)140, (char)78, (char)144, (char)1, (char)62, (char)24, (char)95, (char)135, (char)190, (char)19, (char)171, (char)41, (char)242, (char)213, (char)210, (char)125, (char)9, (char)228, (char)82, (char)169, (char)132, (char)30, (char)143, (char)179, (char)169, (char)223, (char)10, (char)23, (char)100, (char)245, (char)73, (char)65, (char)163, (char)146, (char)220, (char)246, (char)120, (char)65, (char)23, (char)93, (char)0, (char)18, (char)143, (char)59, (char)245, (char)219, (char)150, (char)232, (char)89, (char)6, (char)55, (char)78, (char)153, (char)25, (char)117, (char)175, (char)233, (char)23, (char)224, (char)112, (char)29, (char)199, (char)169, (char)98, (char)84, (char)53, (char)45, (char)178, (char)134, (char)140, (char)156, (char)78, (char)29, (char)220, (char)56, (char)49, (char)100, (char)99, (char)136, (char)41, (char)201, (char)127, (char)253, (char)31, (char)188, (char)149, (char)100, (char)59, (char)5, (char)76, (char)153, (char)102, (char)103, (char)192, (char)48, (char)200, (char)15, (char)118, (char)19, (char)69, (char)52, (char)86, (char)127, (char)61, (char)252, (char)174, (char)222, (char)23, (char)106, (char)65, (char)121, (char)53, (char)241, (char)245, (char)113, (char)220, (char)211, (char)104, (char)153, (char)121, (char)150, (char)48, (char)218, (char)149}, 0) ;
        p266.target_component_SET((char)61) ;
        p266.length_SET((char)26) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)123, (char)237, (char)128, (char)58, (char)241, (char)234, (char)72, (char)7, (char)232, (char)75, (char)62, (char)110, (char)49, (char)158, (char)186, (char)151, (char)247, (char)142, (char)133, (char)100, (char)195, (char)170, (char)244, (char)190, (char)46, (char)134, (char)91, (char)101, (char)251, (char)97, (char)152, (char)3, (char)75, (char)249, (char)166, (char)220, (char)3, (char)140, (char)118, (char)179, (char)205, (char)203, (char)191, (char)14, (char)70, (char)40, (char)62, (char)229, (char)69, (char)51, (char)91, (char)55, (char)194, (char)251, (char)31, (char)178, (char)195, (char)233, (char)249, (char)25, (char)53, (char)177, (char)74, (char)125, (char)183, (char)58, (char)95, (char)45, (char)181, (char)169, (char)125, (char)135, (char)194, (char)57, (char)32, (char)244, (char)99, (char)226, (char)33, (char)229, (char)156, (char)77, (char)66, (char)84, (char)45, (char)175, (char)157, (char)5, (char)138, (char)237, (char)52, (char)134, (char)13, (char)151, (char)215, (char)186, (char)255, (char)164, (char)25, (char)1, (char)193, (char)249, (char)141, (char)172, (char)117, (char)146, (char)20, (char)38, (char)104, (char)119, (char)190, (char)63, (char)62, (char)0, (char)29, (char)236, (char)134, (char)152, (char)131, (char)170, (char)177, (char)144, (char)163, (char)96, (char)98, (char)121, (char)71, (char)240, (char)193, (char)215, (char)171, (char)177, (char)207, (char)89, (char)219, (char)224, (char)234, (char)249, (char)217, (char)219, (char)180, (char)144, (char)224, (char)26, (char)83, (char)106, (char)165, (char)173, (char)141, (char)7, (char)191, (char)161, (char)219, (char)60, (char)1, (char)172, (char)49, (char)148, (char)12, (char)190, (char)99, (char)123, (char)107, (char)151, (char)42, (char)192, (char)29, (char)79, (char)242, (char)31, (char)184, (char)32, (char)186, (char)252, (char)110, (char)187, (char)69, (char)163, (char)53, (char)210, (char)142, (char)82, (char)66, (char)15, (char)28, (char)54, (char)40, (char)4, (char)106, (char)8, (char)167, (char)143, (char)245, (char)188, (char)131, (char)50, (char)219, (char)76, (char)93, (char)133, (char)5, (char)4, (char)223, (char)79, (char)90, (char)219, (char)218, (char)148, (char)9, (char)81, (char)105, (char)22, (char)164, (char)204, (char)241, (char)20, (char)182, (char)123, (char)235, (char)78, (char)73, (char)175, (char)138, (char)182, (char)100, (char)87, (char)39, (char)126, (char)129, (char)215, (char)63, (char)192, (char)17, (char)219, (char)149, (char)199, (char)206, (char)29, (char)167, (char)64, (char)123, (char)192, (char)22, (char)229, (char)99, (char)89, (char)90, (char)189, (char)129}));
            assert(pack.sequence_GET() == (char)8487);
            assert(pack.first_message_offset_GET() == (char)147);
            assert(pack.target_component_GET() == (char)32);
            assert(pack.target_system_GET() == (char)113);
            assert(pack.length_GET() == (char)210);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_system_SET((char)113) ;
        p267.target_component_SET((char)32) ;
        p267.data__SET(new char[] {(char)123, (char)237, (char)128, (char)58, (char)241, (char)234, (char)72, (char)7, (char)232, (char)75, (char)62, (char)110, (char)49, (char)158, (char)186, (char)151, (char)247, (char)142, (char)133, (char)100, (char)195, (char)170, (char)244, (char)190, (char)46, (char)134, (char)91, (char)101, (char)251, (char)97, (char)152, (char)3, (char)75, (char)249, (char)166, (char)220, (char)3, (char)140, (char)118, (char)179, (char)205, (char)203, (char)191, (char)14, (char)70, (char)40, (char)62, (char)229, (char)69, (char)51, (char)91, (char)55, (char)194, (char)251, (char)31, (char)178, (char)195, (char)233, (char)249, (char)25, (char)53, (char)177, (char)74, (char)125, (char)183, (char)58, (char)95, (char)45, (char)181, (char)169, (char)125, (char)135, (char)194, (char)57, (char)32, (char)244, (char)99, (char)226, (char)33, (char)229, (char)156, (char)77, (char)66, (char)84, (char)45, (char)175, (char)157, (char)5, (char)138, (char)237, (char)52, (char)134, (char)13, (char)151, (char)215, (char)186, (char)255, (char)164, (char)25, (char)1, (char)193, (char)249, (char)141, (char)172, (char)117, (char)146, (char)20, (char)38, (char)104, (char)119, (char)190, (char)63, (char)62, (char)0, (char)29, (char)236, (char)134, (char)152, (char)131, (char)170, (char)177, (char)144, (char)163, (char)96, (char)98, (char)121, (char)71, (char)240, (char)193, (char)215, (char)171, (char)177, (char)207, (char)89, (char)219, (char)224, (char)234, (char)249, (char)217, (char)219, (char)180, (char)144, (char)224, (char)26, (char)83, (char)106, (char)165, (char)173, (char)141, (char)7, (char)191, (char)161, (char)219, (char)60, (char)1, (char)172, (char)49, (char)148, (char)12, (char)190, (char)99, (char)123, (char)107, (char)151, (char)42, (char)192, (char)29, (char)79, (char)242, (char)31, (char)184, (char)32, (char)186, (char)252, (char)110, (char)187, (char)69, (char)163, (char)53, (char)210, (char)142, (char)82, (char)66, (char)15, (char)28, (char)54, (char)40, (char)4, (char)106, (char)8, (char)167, (char)143, (char)245, (char)188, (char)131, (char)50, (char)219, (char)76, (char)93, (char)133, (char)5, (char)4, (char)223, (char)79, (char)90, (char)219, (char)218, (char)148, (char)9, (char)81, (char)105, (char)22, (char)164, (char)204, (char)241, (char)20, (char)182, (char)123, (char)235, (char)78, (char)73, (char)175, (char)138, (char)182, (char)100, (char)87, (char)39, (char)126, (char)129, (char)215, (char)63, (char)192, (char)17, (char)219, (char)149, (char)199, (char)206, (char)29, (char)167, (char)64, (char)123, (char)192, (char)22, (char)229, (char)99, (char)89, (char)90, (char)189, (char)129}, 0) ;
        p267.length_SET((char)210) ;
        p267.first_message_offset_SET((char)147) ;
        p267.sequence_SET((char)8487) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)96);
            assert(pack.target_system_GET() == (char)58);
            assert(pack.sequence_GET() == (char)18627);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_component_SET((char)96) ;
        p268.target_system_SET((char)58) ;
        p268.sequence_SET((char)18627) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.uri_LEN(ph) == 181);
            assert(pack.uri_TRY(ph).equals("wUxdhiktnipebxinkgjcajXcruUynmJwlPavgmhLfbjjpxxfmspytOlsmkeevnpogkmbmmvvSzmjuektwjciflwmurabqNwlMzMppaDrrgZxlikiSzjtknvjwwsluwevjdwuwpnxvrsjxftHpjigvuIUlhXfxbToMzpncniwBphdnjsoguopz"));
            assert(pack.framerate_GET() == -4.1506912E37F);
            assert(pack.rotation_GET() == (char)16518);
            assert(pack.bitrate_GET() == 691648139L);
            assert(pack.resolution_h_GET() == (char)48053);
            assert(pack.camera_id_GET() == (char)86);
            assert(pack.resolution_v_GET() == (char)21096);
            assert(pack.status_GET() == (char)93);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.rotation_SET((char)16518) ;
        p269.framerate_SET(-4.1506912E37F) ;
        p269.uri_SET("wUxdhiktnipebxinkgjcajXcruUynmJwlPavgmhLfbjjpxxfmspytOlsmkeevnpogkmbmmvvSzmjuektwjciflwmurabqNwlMzMppaDrrgZxlikiSzjtknvjwwsluwevjdwuwpnxvrsjxftHpjigvuIUlhXfxbToMzpncniwBphdnjsoguopz", PH) ;
        p269.resolution_v_SET((char)21096) ;
        p269.resolution_h_SET((char)48053) ;
        p269.status_SET((char)93) ;
        p269.bitrate_SET(691648139L) ;
        p269.camera_id_SET((char)86) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.uri_LEN(ph) == 163);
            assert(pack.uri_TRY(ph).equals("cbglyeybjHdktprtLpalrffycypvziGzibguefVambmcPJhtmywkkahdeVpiHiXkoqqcqeDemeajlcxkirlzjdpkohupswohhkbpEsrYnrgrqekikdgsuibHihgwQhFGnhgwtvamvgirbQthedQiqcgbprbovyzonli"));
            assert(pack.bitrate_GET() == 990854457L);
            assert(pack.framerate_GET() == -7.7182996E37F);
            assert(pack.camera_id_GET() == (char)240);
            assert(pack.rotation_GET() == (char)42716);
            assert(pack.resolution_h_GET() == (char)49565);
            assert(pack.target_component_GET() == (char)216);
            assert(pack.target_system_GET() == (char)151);
            assert(pack.resolution_v_GET() == (char)57184);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.camera_id_SET((char)240) ;
        p270.bitrate_SET(990854457L) ;
        p270.uri_SET("cbglyeybjHdktprtLpalrffycypvziGzibguefVambmcPJhtmywkkahdeVpiHiXkoqqcqeDemeajlcxkirlzjdpkohupswohhkbpEsrYnrgrqekikdgsuibHihgwQhFGnhgwtvamvgirbQthedQiqcgbprbovyzonli", PH) ;
        p270.rotation_SET((char)42716) ;
        p270.framerate_SET(-7.7182996E37F) ;
        p270.target_component_SET((char)216) ;
        p270.resolution_v_SET((char)57184) ;
        p270.target_system_SET((char)151) ;
        p270.resolution_h_SET((char)49565) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.password_LEN(ph) == 52);
            assert(pack.password_TRY(ph).equals("kjtebqaytxeqhdfLrehSbbqhhjspdzsBzobeydhBfjteborjhjvh"));
            assert(pack.ssid_LEN(ph) == 4);
            assert(pack.ssid_TRY(ph).equals("teqq"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("kjtebqaytxeqhdfLrehSbbqhhjspdzsBzobeydhBfjteborjhjvh", PH) ;
        p299.ssid_SET("teqq", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)79, (char)74, (char)250, (char)160, (char)22, (char)142, (char)81, (char)163}));
            assert(pack.max_version_GET() == (char)61968);
            assert(pack.version_GET() == (char)63264);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)67, (char)169, (char)72, (char)127, (char)230, (char)149, (char)224, (char)145}));
            assert(pack.min_version_GET() == (char)24767);
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.library_version_hash_SET(new char[] {(char)67, (char)169, (char)72, (char)127, (char)230, (char)149, (char)224, (char)145}, 0) ;
        p300.version_SET((char)63264) ;
        p300.max_version_SET((char)61968) ;
        p300.spec_version_hash_SET(new char[] {(char)79, (char)74, (char)250, (char)160, (char)22, (char)142, (char)81, (char)163}, 0) ;
        p300.min_version_SET((char)24767) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.uptime_sec_GET() == 456519722L);
            assert(pack.time_usec_GET() == 555377966150246044L);
            assert(pack.vendor_specific_status_code_GET() == (char)57208);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
            assert(pack.sub_mode_GET() == (char)101);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION) ;
        p310.time_usec_SET(555377966150246044L) ;
        p310.uptime_sec_SET(456519722L) ;
        p310.sub_mode_SET((char)101) ;
        p310.vendor_specific_status_code_SET((char)57208) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 9);
            assert(pack.name_TRY(ph).equals("xszcIaitn"));
            assert(pack.sw_version_major_GET() == (char)220);
            assert(pack.time_usec_GET() == 8307228299286083030L);
            assert(pack.sw_vcs_commit_GET() == 2023230786L);
            assert(pack.uptime_sec_GET() == 2810969068L);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)61, (char)119, (char)39, (char)18, (char)22, (char)144, (char)196, (char)187, (char)121, (char)6, (char)85, (char)221, (char)61, (char)200, (char)187, (char)177}));
            assert(pack.hw_version_major_GET() == (char)17);
            assert(pack.sw_version_minor_GET() == (char)209);
            assert(pack.hw_version_minor_GET() == (char)31);
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.hw_version_major_SET((char)17) ;
        p311.hw_version_minor_SET((char)31) ;
        p311.sw_version_minor_SET((char)209) ;
        p311.time_usec_SET(8307228299286083030L) ;
        p311.name_SET("xszcIaitn", PH) ;
        p311.sw_vcs_commit_SET(2023230786L) ;
        p311.hw_unique_id_SET(new char[] {(char)61, (char)119, (char)39, (char)18, (char)22, (char)144, (char)196, (char)187, (char)121, (char)6, (char)85, (char)221, (char)61, (char)200, (char)187, (char)177}, 0) ;
        p311.sw_version_major_SET((char)220) ;
        p311.uptime_sec_SET(2810969068L) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 7);
            assert(pack.param_id_TRY(ph).equals("yfbdejJ"));
            assert(pack.target_component_GET() == (char)78);
            assert(pack.param_index_GET() == (short) -7199);
            assert(pack.target_system_GET() == (char)146);
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.param_id_SET("yfbdejJ", PH) ;
        p320.target_component_SET((char)78) ;
        p320.target_system_SET((char)146) ;
        p320.param_index_SET((short) -7199) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)243);
            assert(pack.target_system_GET() == (char)16);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)243) ;
        p321.target_system_SET((char)16) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 10);
            assert(pack.param_value_TRY(ph).equals("myuyjmntqu"));
            assert(pack.param_count_GET() == (char)29822);
            assert(pack.param_id_LEN(ph) == 11);
            assert(pack.param_id_TRY(ph).equals("hdauhRwuavg"));
            assert(pack.param_index_GET() == (char)57301);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_value_SET("myuyjmntqu", PH) ;
        p322.param_id_SET("hdauhRwuavg", PH) ;
        p322.param_index_SET((char)57301) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM) ;
        p322.param_count_SET((char)29822) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("cepdq"));
            assert(pack.target_component_GET() == (char)20);
            assert(pack.target_system_GET() == (char)62);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
            assert(pack.param_value_LEN(ph) == 113);
            assert(pack.param_value_TRY(ph).equals("gnjyvgrpbteepvwuRcdlrkOsnodzqlyPnadpwrbavpsekdnxggzBzjqaqbplhjfsdsiKBahuqiijlypgpfwztJSbxmMneuwvQzaxlXmhutuvrabpk"));
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)62) ;
        p323.param_value_SET("gnjyvgrpbteepvwuRcdlrkOsnodzqlyPnadpwrbavpsekdnxggzBzjqaqbplhjfsdsiKBahuqiijlypgpfwztJSbxmMneuwvQzaxlXmhutuvrabpk", PH) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16) ;
        p323.target_component_SET((char)20) ;
        p323.param_id_SET("cepdq", PH) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 118);
            assert(pack.param_value_TRY(ph).equals("bmglnqiaCfuEopekgmybgzoxjxeqLengbfhmvfnzujtxksflgfgwbqLZcscwbfkgoqhudlcruAGvbgLgmmmzudugaghxfxpIhlGeXrsuexxrbFqsfsqvzx"));
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("i"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_IN_PROGRESS);
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64) ;
        p324.param_value_SET("bmglnqiaCfuEopekgmybgzoxjxeqLengbfhmvfnzujtxksflgfgwbqLZcscwbfkgoqhudlcruAGvbgLgmmmzudugaghxfxpIhlGeXrsuexxrbFqsfsqvzx", PH) ;
        p324.param_id_SET("i", PH) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_IN_PROGRESS) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
            assert(pack.increment_GET() == (char)225);
            assert(pack.min_distance_GET() == (char)12453);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)45716, (char)29728, (char)51922, (char)45668, (char)4762, (char)13663, (char)34469, (char)64659, (char)29582, (char)15936, (char)48666, (char)24683, (char)14466, (char)11659, (char)26021, (char)27136, (char)44487, (char)56147, (char)2595, (char)46594, (char)59975, (char)47453, (char)26910, (char)57406, (char)8960, (char)43954, (char)32955, (char)38520, (char)18907, (char)56212, (char)36556, (char)9215, (char)35892, (char)23589, (char)22730, (char)42227, (char)24497, (char)9034, (char)46487, (char)22703, (char)60106, (char)63358, (char)54602, (char)48456, (char)46730, (char)13483, (char)20877, (char)33986, (char)55758, (char)33085, (char)22045, (char)21084, (char)62, (char)62173, (char)58139, (char)28887, (char)49836, (char)45616, (char)26324, (char)27338, (char)55252, (char)20894, (char)61336, (char)36535, (char)42535, (char)28217, (char)18167, (char)44883, (char)46097, (char)31027, (char)26661, (char)10882}));
            assert(pack.time_usec_GET() == 2075414449930877090L);
            assert(pack.max_distance_GET() == (char)61892);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED) ;
        p330.min_distance_SET((char)12453) ;
        p330.increment_SET((char)225) ;
        p330.distances_SET(new char[] {(char)45716, (char)29728, (char)51922, (char)45668, (char)4762, (char)13663, (char)34469, (char)64659, (char)29582, (char)15936, (char)48666, (char)24683, (char)14466, (char)11659, (char)26021, (char)27136, (char)44487, (char)56147, (char)2595, (char)46594, (char)59975, (char)47453, (char)26910, (char)57406, (char)8960, (char)43954, (char)32955, (char)38520, (char)18907, (char)56212, (char)36556, (char)9215, (char)35892, (char)23589, (char)22730, (char)42227, (char)24497, (char)9034, (char)46487, (char)22703, (char)60106, (char)63358, (char)54602, (char)48456, (char)46730, (char)13483, (char)20877, (char)33986, (char)55758, (char)33085, (char)22045, (char)21084, (char)62, (char)62173, (char)58139, (char)28887, (char)49836, (char)45616, (char)26324, (char)27338, (char)55252, (char)20894, (char)61336, (char)36535, (char)42535, (char)28217, (char)18167, (char)44883, (char)46097, (char)31027, (char)26661, (char)10882}, 0) ;
        p330.time_usec_SET(2075414449930877090L) ;
        p330.max_distance_SET((char)61892) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVIONIX_ADSB_OUT_CFG.add((src, ph, pack) ->
        {
            assert(pack.gpsOffsetLat_GET() == UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_4M);
            assert(pack.stallSpeed_GET() == (char)22791);
            assert(pack.ICAO_GET() == 508754360L);
            assert(pack.callsign_LEN(ph) == 7);
            assert(pack.callsign_TRY(ph).equals("iPlmfpz"));
            assert(pack.emitterType_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SERVICE_SURFACE);
            assert(pack.gpsOffsetLon_GET() == UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR);
            assert(pack.rfSelect_GET() == UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED);
            assert(pack.aircraftSize_GET() == UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25_34M);
        });
        GroundControl.UAVIONIX_ADSB_OUT_CFG p10001 = CommunicationChannel.new_UAVIONIX_ADSB_OUT_CFG();
        PH.setPack(p10001);
        p10001.emitterType_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SERVICE_SURFACE) ;
        p10001.stallSpeed_SET((char)22791) ;
        p10001.rfSelect_SET(UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED) ;
        p10001.ICAO_SET(508754360L) ;
        p10001.aircraftSize_SET(UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25_34M) ;
        p10001.gpsOffsetLat_SET(UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_4M) ;
        p10001.callsign_SET("iPlmfpz", PH) ;
        p10001.gpsOffsetLon_SET(UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR) ;
        CommunicationChannel.instance.send(p10001);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVIONIX_ADSB_OUT_DYNAMIC.add((src, ph, pack) ->
        {
            assert(pack.gpsLat_GET() == -1942761640);
            assert(pack.state_GET() == (UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT |
                                        UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED));
            assert(pack.VelEW_GET() == (short) -17334);
            assert(pack.emergencyStatus_GET() == UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_NO_EMERGENCY);
            assert(pack.gpsAlt_GET() == 276372559);
            assert(pack.baroAltMSL_GET() == -1020464388);
            assert(pack.numSats_GET() == (char)214);
            assert(pack.velVert_GET() == (short) -20633);
            assert(pack.velNS_GET() == (short) -2163);
            assert(pack.gpsFix_GET() == UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_1);
            assert(pack.accuracyVel_GET() == (char)19547);
            assert(pack.accuracyHor_GET() == 603433932L);
            assert(pack.utcTime_GET() == 2837336422L);
            assert(pack.accuracyVert_GET() == (char)26075);
            assert(pack.gpsLon_GET() == 2145741435);
            assert(pack.squawk_GET() == (char)26656);
        });
        GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC p10002 = CommunicationChannel.new_UAVIONIX_ADSB_OUT_DYNAMIC();
        PH.setPack(p10002);
        p10002.squawk_SET((char)26656) ;
        p10002.VelEW_SET((short) -17334) ;
        p10002.baroAltMSL_SET(-1020464388) ;
        p10002.velVert_SET((short) -20633) ;
        p10002.emergencyStatus_SET(UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_NO_EMERGENCY) ;
        p10002.gpsLat_SET(-1942761640) ;
        p10002.gpsAlt_SET(276372559) ;
        p10002.accuracyVert_SET((char)26075) ;
        p10002.accuracyVel_SET((char)19547) ;
        p10002.gpsFix_SET(UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_1) ;
        p10002.gpsLon_SET(2145741435) ;
        p10002.utcTime_SET(2837336422L) ;
        p10002.numSats_SET((char)214) ;
        p10002.accuracyHor_SET(603433932L) ;
        p10002.state_SET((UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT |
                          UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED)) ;
        p10002.velNS_SET((short) -2163) ;
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
            assert(pack.request_id_GET() == 844630422L);
            assert(pack.bustype_GET() == DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_SPI);
            assert(pack.count_GET() == (char)117);
            assert(pack.target_component_GET() == (char)140);
            assert(pack.busname_LEN(ph) == 2);
            assert(pack.busname_TRY(ph).equals("xu"));
            assert(pack.address_GET() == (char)196);
            assert(pack.regstart_GET() == (char)153);
            assert(pack.target_system_GET() == (char)51);
            assert(pack.bus_GET() == (char)62);
        });
        GroundControl.DEVICE_OP_READ p11000 = CommunicationChannel.new_DEVICE_OP_READ();
        PH.setPack(p11000);
        p11000.request_id_SET(844630422L) ;
        p11000.address_SET((char)196) ;
        p11000.target_system_SET((char)51) ;
        p11000.target_component_SET((char)140) ;
        p11000.bus_SET((char)62) ;
        p11000.regstart_SET((char)153) ;
        p11000.busname_SET("xu", PH) ;
        p11000.bustype_SET(DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_SPI) ;
        p11000.count_SET((char)117) ;
        CommunicationChannel.instance.send(p11000);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEVICE_OP_READ_REPLY.add((src, ph, pack) ->
        {
            assert(pack.request_id_GET() == 4203110816L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)241, (char)232, (char)146, (char)41, (char)134, (char)248, (char)41, (char)4, (char)194, (char)192, (char)26, (char)242, (char)144, (char)213, (char)90, (char)26, (char)209, (char)247, (char)234, (char)249, (char)16, (char)231, (char)198, (char)172, (char)64, (char)35, (char)74, (char)205, (char)224, (char)188, (char)246, (char)116, (char)244, (char)203, (char)87, (char)195, (char)180, (char)148, (char)124, (char)238, (char)208, (char)169, (char)239, (char)230, (char)233, (char)217, (char)142, (char)252, (char)221, (char)194, (char)48, (char)186, (char)161, (char)234, (char)180, (char)121, (char)149, (char)122, (char)205, (char)179, (char)135, (char)52, (char)25, (char)255, (char)135, (char)102, (char)13, (char)214, (char)123, (char)83, (char)40, (char)235, (char)26, (char)164, (char)133, (char)22, (char)164, (char)231, (char)243, (char)0, (char)7, (char)40, (char)141, (char)237, (char)11, (char)118, (char)155, (char)67, (char)131, (char)124, (char)239, (char)181, (char)224, (char)61, (char)242, (char)220, (char)230, (char)209, (char)195, (char)7, (char)159, (char)8, (char)39, (char)232, (char)150, (char)106, (char)215, (char)14, (char)146, (char)138, (char)193, (char)192, (char)115, (char)29, (char)156, (char)70, (char)117, (char)215, (char)178, (char)57, (char)93, (char)49, (char)51, (char)4, (char)110, (char)15, (char)114, (char)124}));
            assert(pack.count_GET() == (char)67);
            assert(pack.result_GET() == (char)128);
            assert(pack.regstart_GET() == (char)236);
        });
        GroundControl.DEVICE_OP_READ_REPLY p11001 = CommunicationChannel.new_DEVICE_OP_READ_REPLY();
        PH.setPack(p11001);
        p11001.count_SET((char)67) ;
        p11001.request_id_SET(4203110816L) ;
        p11001.data__SET(new char[] {(char)241, (char)232, (char)146, (char)41, (char)134, (char)248, (char)41, (char)4, (char)194, (char)192, (char)26, (char)242, (char)144, (char)213, (char)90, (char)26, (char)209, (char)247, (char)234, (char)249, (char)16, (char)231, (char)198, (char)172, (char)64, (char)35, (char)74, (char)205, (char)224, (char)188, (char)246, (char)116, (char)244, (char)203, (char)87, (char)195, (char)180, (char)148, (char)124, (char)238, (char)208, (char)169, (char)239, (char)230, (char)233, (char)217, (char)142, (char)252, (char)221, (char)194, (char)48, (char)186, (char)161, (char)234, (char)180, (char)121, (char)149, (char)122, (char)205, (char)179, (char)135, (char)52, (char)25, (char)255, (char)135, (char)102, (char)13, (char)214, (char)123, (char)83, (char)40, (char)235, (char)26, (char)164, (char)133, (char)22, (char)164, (char)231, (char)243, (char)0, (char)7, (char)40, (char)141, (char)237, (char)11, (char)118, (char)155, (char)67, (char)131, (char)124, (char)239, (char)181, (char)224, (char)61, (char)242, (char)220, (char)230, (char)209, (char)195, (char)7, (char)159, (char)8, (char)39, (char)232, (char)150, (char)106, (char)215, (char)14, (char)146, (char)138, (char)193, (char)192, (char)115, (char)29, (char)156, (char)70, (char)117, (char)215, (char)178, (char)57, (char)93, (char)49, (char)51, (char)4, (char)110, (char)15, (char)114, (char)124}, 0) ;
        p11001.regstart_SET((char)236) ;
        p11001.result_SET((char)128) ;
        CommunicationChannel.instance.send(p11001);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEVICE_OP_WRITE.add((src, ph, pack) ->
        {
            assert(pack.regstart_GET() == (char)62);
            assert(pack.bustype_GET() == DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C);
            assert(pack.request_id_GET() == 1789192184L);
            assert(pack.target_system_GET() == (char)108);
            assert(pack.address_GET() == (char)72);
            assert(pack.count_GET() == (char)151);
            assert(pack.target_component_GET() == (char)31);
            assert(pack.busname_LEN(ph) == 20);
            assert(pack.busname_TRY(ph).equals("zdgcolmuVagrnbqljmzj"));
            assert(pack.bus_GET() == (char)6);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)68, (char)146, (char)172, (char)156, (char)247, (char)252, (char)54, (char)85, (char)20, (char)99, (char)40, (char)184, (char)156, (char)203, (char)50, (char)100, (char)220, (char)183, (char)139, (char)162, (char)188, (char)186, (char)238, (char)169, (char)82, (char)241, (char)32, (char)100, (char)71, (char)107, (char)203, (char)244, (char)64, (char)92, (char)103, (char)189, (char)58, (char)127, (char)66, (char)3, (char)202, (char)238, (char)120, (char)12, (char)92, (char)141, (char)49, (char)12, (char)51, (char)189, (char)38, (char)145, (char)165, (char)34, (char)64, (char)133, (char)170, (char)77, (char)149, (char)19, (char)175, (char)55, (char)49, (char)88, (char)100, (char)212, (char)107, (char)64, (char)152, (char)41, (char)118, (char)5, (char)199, (char)47, (char)201, (char)105, (char)188, (char)101, (char)92, (char)201, (char)131, (char)215, (char)133, (char)57, (char)176, (char)151, (char)34, (char)28, (char)146, (char)172, (char)6, (char)75, (char)174, (char)43, (char)165, (char)168, (char)101, (char)143, (char)20, (char)140, (char)225, (char)29, (char)8, (char)108, (char)108, (char)252, (char)150, (char)191, (char)37, (char)80, (char)24, (char)190, (char)177, (char)20, (char)110, (char)252, (char)195, (char)15, (char)213, (char)89, (char)157, (char)168, (char)137, (char)194, (char)20, (char)232, (char)249, (char)183}));
        });
        GroundControl.DEVICE_OP_WRITE p11002 = CommunicationChannel.new_DEVICE_OP_WRITE();
        PH.setPack(p11002);
        p11002.bustype_SET(DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C) ;
        p11002.request_id_SET(1789192184L) ;
        p11002.target_system_SET((char)108) ;
        p11002.address_SET((char)72) ;
        p11002.regstart_SET((char)62) ;
        p11002.data__SET(new char[] {(char)68, (char)146, (char)172, (char)156, (char)247, (char)252, (char)54, (char)85, (char)20, (char)99, (char)40, (char)184, (char)156, (char)203, (char)50, (char)100, (char)220, (char)183, (char)139, (char)162, (char)188, (char)186, (char)238, (char)169, (char)82, (char)241, (char)32, (char)100, (char)71, (char)107, (char)203, (char)244, (char)64, (char)92, (char)103, (char)189, (char)58, (char)127, (char)66, (char)3, (char)202, (char)238, (char)120, (char)12, (char)92, (char)141, (char)49, (char)12, (char)51, (char)189, (char)38, (char)145, (char)165, (char)34, (char)64, (char)133, (char)170, (char)77, (char)149, (char)19, (char)175, (char)55, (char)49, (char)88, (char)100, (char)212, (char)107, (char)64, (char)152, (char)41, (char)118, (char)5, (char)199, (char)47, (char)201, (char)105, (char)188, (char)101, (char)92, (char)201, (char)131, (char)215, (char)133, (char)57, (char)176, (char)151, (char)34, (char)28, (char)146, (char)172, (char)6, (char)75, (char)174, (char)43, (char)165, (char)168, (char)101, (char)143, (char)20, (char)140, (char)225, (char)29, (char)8, (char)108, (char)108, (char)252, (char)150, (char)191, (char)37, (char)80, (char)24, (char)190, (char)177, (char)20, (char)110, (char)252, (char)195, (char)15, (char)213, (char)89, (char)157, (char)168, (char)137, (char)194, (char)20, (char)232, (char)249, (char)183}, 0) ;
        p11002.bus_SET((char)6) ;
        p11002.busname_SET("zdgcolmuVagrnbqljmzj", PH) ;
        p11002.target_component_SET((char)31) ;
        p11002.count_SET((char)151) ;
        CommunicationChannel.instance.send(p11002);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEVICE_OP_WRITE_REPLY.add((src, ph, pack) ->
        {
            assert(pack.request_id_GET() == 2308930788L);
            assert(pack.result_GET() == (char)202);
        });
        GroundControl.DEVICE_OP_WRITE_REPLY p11003 = CommunicationChannel.new_DEVICE_OP_WRITE_REPLY();
        PH.setPack(p11003);
        p11003.result_SET((char)202) ;
        p11003.request_id_SET(2308930788L) ;
        CommunicationChannel.instance.send(p11003);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ADAP_TUNING.add((src, ph, pack) ->
        {
            assert(pack.omega_dot_GET() == 1.7202714E38F);
            assert(pack.error_GET() == 1.1962453E38F);
            assert(pack.achieved_GET() == 1.0843391E38F);
            assert(pack.sigma_GET() == -4.0451697E37F);
            assert(pack.theta_GET() == -8.698323E37F);
            assert(pack.f_dot_GET() == -7.651803E37F);
            assert(pack.desired_GET() == 2.8504552E38F);
            assert(pack.omega_GET() == 6.1533073E37F);
            assert(pack.axis_GET() == PID_TUNING_AXIS.PID_TUNING_YAW);
            assert(pack.u_GET() == 1.9464882E38F);
            assert(pack.sigma_dot_GET() == 2.505696E38F);
            assert(pack.theta_dot_GET() == 1.4113654E38F);
            assert(pack.f_GET() == -2.2205498E38F);
        });
        GroundControl.ADAP_TUNING p11010 = CommunicationChannel.new_ADAP_TUNING();
        PH.setPack(p11010);
        p11010.f_SET(-2.2205498E38F) ;
        p11010.omega_SET(6.1533073E37F) ;
        p11010.axis_SET(PID_TUNING_AXIS.PID_TUNING_YAW) ;
        p11010.theta_dot_SET(1.4113654E38F) ;
        p11010.f_dot_SET(-7.651803E37F) ;
        p11010.omega_dot_SET(1.7202714E38F) ;
        p11010.desired_SET(2.8504552E38F) ;
        p11010.theta_SET(-8.698323E37F) ;
        p11010.achieved_SET(1.0843391E38F) ;
        p11010.sigma_SET(-4.0451697E37F) ;
        p11010.u_SET(1.9464882E38F) ;
        p11010.sigma_dot_SET(2.505696E38F) ;
        p11010.error_SET(1.1962453E38F) ;
        CommunicationChannel.instance.send(p11010);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VISION_POSITION_DELTA.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 8868101885072526009L);
            assert(pack.time_delta_usec_GET() == 6879709271276562878L);
            assert(Arrays.equals(pack.angle_delta_GET(),  new float[] {-6.654339E37F, -3.1871773E37F, 1.7576383E38F}));
            assert(pack.confidence_GET() == -2.7309768E38F);
            assert(Arrays.equals(pack.position_delta_GET(),  new float[] {1.617491E38F, 2.0771331E37F, 3.1439228E38F}));
        });
        GroundControl.VISION_POSITION_DELTA p11011 = CommunicationChannel.new_VISION_POSITION_DELTA();
        PH.setPack(p11011);
        p11011.angle_delta_SET(new float[] {-6.654339E37F, -3.1871773E37F, 1.7576383E38F}, 0) ;
        p11011.confidence_SET(-2.7309768E38F) ;
        p11011.time_usec_SET(8868101885072526009L) ;
        p11011.position_delta_SET(new float[] {1.617491E38F, 2.0771331E37F, 3.1439228E38F}, 0) ;
        p11011.time_delta_usec_SET(6879709271276562878L) ;
        CommunicationChannel.instance.send(p11011);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
    }

}
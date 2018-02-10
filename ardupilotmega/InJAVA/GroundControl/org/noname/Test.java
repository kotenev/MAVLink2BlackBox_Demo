
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
            assert(pack.mavlink_version_GET() == (char)113);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED));
            assert(pack.custom_mode_GET() == 544332504L);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_UDB);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_COAXIAL);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_CRITICAL);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.system_status_SET(MAV_STATE.MAV_STATE_CRITICAL) ;
        p0.mavlink_version_SET((char)113) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED)) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_UDB) ;
        p0.custom_mode_SET(544332504L) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_COAXIAL) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.battery_remaining_GET() == (byte) - 10);
            assert(pack.load_GET() == (char)65078);
            assert(pack.current_battery_GET() == (short)13864);
            assert(pack.voltage_battery_GET() == (char)62708);
            assert(pack.errors_count4_GET() == (char)58022);
            assert(pack.errors_count2_GET() == (char)25937);
            assert(pack.errors_count3_GET() == (char)49384);
            assert(pack.drop_rate_comm_GET() == (char)37505);
            assert(pack.errors_count1_GET() == (char)64172);
            assert(pack.errors_comm_GET() == (char)56034);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW));
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION));
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.errors_count4_SET((char)58022) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        p1.current_battery_SET((short)13864) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW)) ;
        p1.errors_count2_SET((char)25937) ;
        p1.voltage_battery_SET((char)62708) ;
        p1.battery_remaining_SET((byte) - 10) ;
        p1.errors_comm_SET((char)56034) ;
        p1.drop_rate_comm_SET((char)37505) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION)) ;
        p1.load_SET((char)65078) ;
        p1.errors_count1_SET((char)64172) ;
        p1.errors_count3_SET((char)49384) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 2169999227556182645L);
            assert(pack.time_boot_ms_GET() == 2509307503L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(2509307503L) ;
        p2.time_unix_usec_SET(2169999227556182645L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == -4.6820315E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.x_GET() == -3.0852502E38F);
            assert(pack.z_GET() == -8.538196E37F);
            assert(pack.vz_GET() == 7.8971514E37F);
            assert(pack.vy_GET() == -1.4066968E38F);
            assert(pack.yaw_GET() == 2.906673E38F);
            assert(pack.yaw_rate_GET() == 1.1336661E38F);
            assert(pack.time_boot_ms_GET() == 3783465409L);
            assert(pack.type_mask_GET() == (char)38864);
            assert(pack.afy_GET() == 2.9105663E38F);
            assert(pack.afx_GET() == -2.9728964E38F);
            assert(pack.y_GET() == 1.7301654E38F);
            assert(pack.afz_GET() == -3.8964895E37F);
        });
        POSITION_TARGET_LOCAL_NED p3 = new POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.x_SET(-3.0852502E38F) ;
        p3.yaw_rate_SET(1.1336661E38F) ;
        p3.vx_SET(-4.6820315E37F) ;
        p3.afx_SET(-2.9728964E38F) ;
        p3.vz_SET(7.8971514E37F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p3.time_boot_ms_SET(3783465409L) ;
        p3.yaw_SET(2.906673E38F) ;
        p3.afz_SET(-3.8964895E37F) ;
        p3.vy_SET(-1.4066968E38F) ;
        p3.afy_SET(2.9105663E38F) ;
        p3.type_mask_SET((char)38864) ;
        p3.z_SET(-8.538196E37F) ;
        p3.y_SET(1.7301654E38F) ;
        TestChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)12);
            assert(pack.seq_GET() == 3166329616L);
            assert(pack.time_usec_GET() == 5459757149468808893L);
            assert(pack.target_system_GET() == (char)63);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.time_usec_SET(5459757149468808893L) ;
        p4.target_system_SET((char)63) ;
        p4.seq_SET(3166329616L) ;
        p4.target_component_SET((char)12) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.passkey_LEN(ph) == 15);
            assert(pack.passkey_TRY(ph).equals("ToxjuvIzekrsZko"));
            assert(pack.target_system_GET() == (char)120);
            assert(pack.control_request_GET() == (char)104);
            assert(pack.version_GET() == (char)125);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.passkey_SET("ToxjuvIzekrsZko", PH) ;
        p5.target_system_SET((char)120) ;
        p5.version_SET((char)125) ;
        p5.control_request_SET((char)104) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.gcs_system_id_GET() == (char)146);
            assert(pack.ack_GET() == (char)141);
            assert(pack.control_request_GET() == (char)57);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.control_request_SET((char)57) ;
        p6.ack_SET((char)141) ;
        p6.gcs_system_id_SET((char)146) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 14);
            assert(pack.key_TRY(ph).equals("cqjxuqqgwPscjb"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("cqjxuqqgwPscjb", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
            assert(pack.custom_mode_GET() == 2475890472L);
            assert(pack.target_system_GET() == (char)57);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.custom_mode_SET(2475890472L) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_STABILIZE_DISARMED) ;
        p11.target_system_SET((char)57) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)11);
            assert(pack.param_index_GET() == (short)5589);
            assert(pack.target_component_GET() == (char)215);
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("kurf"));
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_component_SET((char)215) ;
        p20.target_system_SET((char)11) ;
        p20.param_id_SET("kurf", PH) ;
        p20.param_index_SET((short)5589) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)32);
            assert(pack.target_system_GET() == (char)249);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)249) ;
        p21.target_component_SET((char)32) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_value_GET() == -1.2990767E38F);
            assert(pack.param_count_GET() == (char)55211);
            assert(pack.param_index_GET() == (char)49837);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("cjgs"));
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_value_SET(-1.2990767E38F) ;
        p22.param_index_SET((char)49837) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16) ;
        p22.param_count_SET((char)55211) ;
        p22.param_id_SET("cjgs", PH) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("kmgcjsgoqers"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
            assert(pack.param_value_GET() == 2.0638486E38F);
            assert(pack.target_component_GET() == (char)10);
            assert(pack.target_system_GET() == (char)130);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.target_component_SET((char)10) ;
        p23.target_system_SET((char)130) ;
        p23.param_id_SET("kmgcjsgoqers", PH) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8) ;
        p23.param_value_SET(2.0638486E38F) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.epv_GET() == (char)20347);
            assert(pack.vel_acc_TRY(ph) == 1324414826L);
            assert(pack.alt_GET() == 1561544943);
            assert(pack.alt_ellipsoid_TRY(ph) == -899429753);
            assert(pack.h_acc_TRY(ph) == 3543941262L);
            assert(pack.time_usec_GET() == 535339226081651055L);
            assert(pack.vel_GET() == (char)31769);
            assert(pack.lon_GET() == 1667391649);
            assert(pack.eph_GET() == (char)57802);
            assert(pack.hdg_acc_TRY(ph) == 3119796206L);
            assert(pack.v_acc_TRY(ph) == 1600594246L);
            assert(pack.cog_GET() == (char)10243);
            assert(pack.lat_GET() == -871208114);
            assert(pack.satellites_visible_GET() == (char)14);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.vel_SET((char)31769) ;
        p24.time_usec_SET(535339226081651055L) ;
        p24.hdg_acc_SET(3119796206L, PH) ;
        p24.alt_SET(1561544943) ;
        p24.lat_SET(-871208114) ;
        p24.satellites_visible_SET((char)14) ;
        p24.cog_SET((char)10243) ;
        p24.h_acc_SET(3543941262L, PH) ;
        p24.alt_ellipsoid_SET(-899429753, PH) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT) ;
        p24.lon_SET(1667391649) ;
        p24.vel_acc_SET(1324414826L, PH) ;
        p24.v_acc_SET(1600594246L, PH) ;
        p24.epv_SET((char)20347) ;
        p24.eph_SET((char)57802) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)44, (char)85, (char)239, (char)1, (char)207, (char)179, (char)119, (char)82, (char)27, (char)66, (char)0, (char)9, (char)81, (char)46, (char)205, (char)203, (char)72, (char)155, (char)249, (char)42}));
            assert(pack.satellites_visible_GET() == (char)193);
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)186, (char)72, (char)131, (char)26, (char)163, (char)102, (char)223, (char)136, (char)211, (char)213, (char)6, (char)38, (char)193, (char)73, (char)252, (char)167, (char)191, (char)228, (char)195, (char)33}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)139, (char)8, (char)119, (char)205, (char)53, (char)84, (char)98, (char)239, (char)231, (char)130, (char)200, (char)150, (char)57, (char)207, (char)128, (char)30, (char)206, (char)72, (char)199, (char)87}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)62, (char)21, (char)221, (char)122, (char)71, (char)135, (char)68, (char)201, (char)105, (char)135, (char)178, (char)118, (char)179, (char)11, (char)155, (char)180, (char)240, (char)123, (char)188, (char)165}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)65, (char)207, (char)51, (char)213, (char)117, (char)205, (char)41, (char)242, (char)144, (char)118, (char)95, (char)58, (char)62, (char)105, (char)8, (char)215, (char)81, (char)128, (char)53, (char)228}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_elevation_SET(new char[] {(char)186, (char)72, (char)131, (char)26, (char)163, (char)102, (char)223, (char)136, (char)211, (char)213, (char)6, (char)38, (char)193, (char)73, (char)252, (char)167, (char)191, (char)228, (char)195, (char)33}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)62, (char)21, (char)221, (char)122, (char)71, (char)135, (char)68, (char)201, (char)105, (char)135, (char)178, (char)118, (char)179, (char)11, (char)155, (char)180, (char)240, (char)123, (char)188, (char)165}, 0) ;
        p25.satellites_visible_SET((char)193) ;
        p25.satellite_azimuth_SET(new char[] {(char)139, (char)8, (char)119, (char)205, (char)53, (char)84, (char)98, (char)239, (char)231, (char)130, (char)200, (char)150, (char)57, (char)207, (char)128, (char)30, (char)206, (char)72, (char)199, (char)87}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)65, (char)207, (char)51, (char)213, (char)117, (char)205, (char)41, (char)242, (char)144, (char)118, (char)95, (char)58, (char)62, (char)105, (char)8, (char)215, (char)81, (char)128, (char)53, (char)228}, 0) ;
        p25.satellite_used_SET(new char[] {(char)44, (char)85, (char)239, (char)1, (char)207, (char)179, (char)119, (char)82, (char)27, (char)66, (char)0, (char)9, (char)81, (char)46, (char)205, (char)203, (char)72, (char)155, (char)249, (char)42}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == (short)27436);
            assert(pack.xmag_GET() == (short)11571);
            assert(pack.ymag_GET() == (short) -23463);
            assert(pack.yacc_GET() == (short) -6395);
            assert(pack.xacc_GET() == (short) -6326);
            assert(pack.zmag_GET() == (short)23119);
            assert(pack.zacc_GET() == (short)31828);
            assert(pack.time_boot_ms_GET() == 286354936L);
            assert(pack.ygyro_GET() == (short) -30850);
            assert(pack.xgyro_GET() == (short)1170);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.yacc_SET((short) -6395) ;
        p26.ymag_SET((short) -23463) ;
        p26.xgyro_SET((short)1170) ;
        p26.zgyro_SET((short)27436) ;
        p26.zacc_SET((short)31828) ;
        p26.ygyro_SET((short) -30850) ;
        p26.xmag_SET((short)11571) ;
        p26.xacc_SET((short) -6326) ;
        p26.time_boot_ms_SET(286354936L) ;
        p26.zmag_SET((short)23119) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == (short) -22681);
            assert(pack.ygyro_GET() == (short)31466);
            assert(pack.xgyro_GET() == (short) -2789);
            assert(pack.xacc_GET() == (short)13129);
            assert(pack.zmag_GET() == (short)23642);
            assert(pack.yacc_GET() == (short)17459);
            assert(pack.zacc_GET() == (short)4385);
            assert(pack.xmag_GET() == (short)25364);
            assert(pack.time_usec_GET() == 8502555059581473599L);
            assert(pack.ymag_GET() == (short)9315);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.zmag_SET((short)23642) ;
        p27.zgyro_SET((short) -22681) ;
        p27.zacc_SET((short)4385) ;
        p27.xmag_SET((short)25364) ;
        p27.xgyro_SET((short) -2789) ;
        p27.yacc_SET((short)17459) ;
        p27.time_usec_SET(8502555059581473599L) ;
        p27.ygyro_SET((short)31466) ;
        p27.xacc_SET((short)13129) ;
        p27.ymag_SET((short)9315) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6900069446363656765L);
            assert(pack.temperature_GET() == (short)12631);
            assert(pack.press_abs_GET() == (short)22678);
            assert(pack.press_diff1_GET() == (short)29880);
            assert(pack.press_diff2_GET() == (short) -3745);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.time_usec_SET(6900069446363656765L) ;
        p28.press_diff2_SET((short) -3745) ;
        p28.temperature_SET((short)12631) ;
        p28.press_diff1_SET((short)29880) ;
        p28.press_abs_SET((short)22678) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 878962498L);
            assert(pack.press_abs_GET() == -1.6840894E38F);
            assert(pack.press_diff_GET() == -4.847729E36F);
            assert(pack.temperature_GET() == (short)15600);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_diff_SET(-4.847729E36F) ;
        p29.temperature_SET((short)15600) ;
        p29.press_abs_SET(-1.6840894E38F) ;
        p29.time_boot_ms_SET(878962498L) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3839130907L);
            assert(pack.rollspeed_GET() == 2.1300335E38F);
            assert(pack.pitchspeed_GET() == 2.8236832E38F);
            assert(pack.pitch_GET() == 6.0271117E37F);
            assert(pack.yawspeed_GET() == 1.2214562E38F);
            assert(pack.roll_GET() == 2.7561884E38F);
            assert(pack.yaw_GET() == -1.9689378E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.pitch_SET(6.0271117E37F) ;
        p30.roll_SET(2.7561884E38F) ;
        p30.time_boot_ms_SET(3839130907L) ;
        p30.pitchspeed_SET(2.8236832E38F) ;
        p30.yawspeed_SET(1.2214562E38F) ;
        p30.rollspeed_SET(2.1300335E38F) ;
        p30.yaw_SET(-1.9689378E38F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3435807251L);
            assert(pack.q2_GET() == 3.3251968E38F);
            assert(pack.rollspeed_GET() == -5.932082E37F);
            assert(pack.q3_GET() == -1.4488905E38F);
            assert(pack.q1_GET() == -1.0584799E38F);
            assert(pack.q4_GET() == -3.1419473E37F);
            assert(pack.yawspeed_GET() == 8.607238E37F);
            assert(pack.pitchspeed_GET() == -1.0207298E38F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q1_SET(-1.0584799E38F) ;
        p31.q4_SET(-3.1419473E37F) ;
        p31.pitchspeed_SET(-1.0207298E38F) ;
        p31.q3_SET(-1.4488905E38F) ;
        p31.yawspeed_SET(8.607238E37F) ;
        p31.q2_SET(3.3251968E38F) ;
        p31.time_boot_ms_SET(3435807251L) ;
        p31.rollspeed_SET(-5.932082E37F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 1.3207733E38F);
            assert(pack.time_boot_ms_GET() == 1937358312L);
            assert(pack.vy_GET() == -3.3702646E37F);
            assert(pack.y_GET() == 2.8021315E38F);
            assert(pack.x_GET() == 1.5840385E38F);
            assert(pack.vx_GET() == 2.2763703E38F);
            assert(pack.vz_GET() == -1.2768975E38F);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.z_SET(1.3207733E38F) ;
        p32.x_SET(1.5840385E38F) ;
        p32.vz_SET(-1.2768975E38F) ;
        p32.vx_SET(2.2763703E38F) ;
        p32.y_SET(2.8021315E38F) ;
        p32.vy_SET(-3.3702646E37F) ;
        p32.time_boot_ms_SET(1937358312L) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == (short)22814);
            assert(pack.vz_GET() == (short) -18903);
            assert(pack.relative_alt_GET() == 999814129);
            assert(pack.lon_GET() == -738275461);
            assert(pack.vy_GET() == (short) -19433);
            assert(pack.time_boot_ms_GET() == 811489832L);
            assert(pack.hdg_GET() == (char)17603);
            assert(pack.lat_GET() == -1804754978);
            assert(pack.alt_GET() == 1395957149);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.lat_SET(-1804754978) ;
        p33.alt_SET(1395957149) ;
        p33.time_boot_ms_SET(811489832L) ;
        p33.relative_alt_SET(999814129) ;
        p33.vz_SET((short) -18903) ;
        p33.hdg_SET((char)17603) ;
        p33.vy_SET((short) -19433) ;
        p33.lon_SET(-738275461) ;
        p33.vx_SET((short)22814) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan5_scaled_GET() == (short)6528);
            assert(pack.port_GET() == (char)191);
            assert(pack.chan6_scaled_GET() == (short)3572);
            assert(pack.chan8_scaled_GET() == (short)351);
            assert(pack.rssi_GET() == (char)132);
            assert(pack.chan1_scaled_GET() == (short) -10210);
            assert(pack.chan3_scaled_GET() == (short)19009);
            assert(pack.chan4_scaled_GET() == (short) -20726);
            assert(pack.time_boot_ms_GET() == 784361519L);
            assert(pack.chan7_scaled_GET() == (short)18648);
            assert(pack.chan2_scaled_GET() == (short) -3808);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan2_scaled_SET((short) -3808) ;
        p34.time_boot_ms_SET(784361519L) ;
        p34.chan6_scaled_SET((short)3572) ;
        p34.chan3_scaled_SET((short)19009) ;
        p34.chan1_scaled_SET((short) -10210) ;
        p34.port_SET((char)191) ;
        p34.chan7_scaled_SET((short)18648) ;
        p34.chan4_scaled_SET((short) -20726) ;
        p34.rssi_SET((char)132) ;
        p34.chan5_scaled_SET((short)6528) ;
        p34.chan8_scaled_SET((short)351) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan5_raw_GET() == (char)25148);
            assert(pack.rssi_GET() == (char)215);
            assert(pack.chan1_raw_GET() == (char)55596);
            assert(pack.chan3_raw_GET() == (char)54592);
            assert(pack.time_boot_ms_GET() == 1478263041L);
            assert(pack.chan7_raw_GET() == (char)9454);
            assert(pack.port_GET() == (char)196);
            assert(pack.chan6_raw_GET() == (char)5576);
            assert(pack.chan4_raw_GET() == (char)4085);
            assert(pack.chan2_raw_GET() == (char)6840);
            assert(pack.chan8_raw_GET() == (char)55055);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan6_raw_SET((char)5576) ;
        p35.rssi_SET((char)215) ;
        p35.chan4_raw_SET((char)4085) ;
        p35.chan2_raw_SET((char)6840) ;
        p35.time_boot_ms_SET(1478263041L) ;
        p35.chan1_raw_SET((char)55596) ;
        p35.chan3_raw_SET((char)54592) ;
        p35.chan5_raw_SET((char)25148) ;
        p35.chan7_raw_SET((char)9454) ;
        p35.port_SET((char)196) ;
        p35.chan8_raw_SET((char)55055) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo15_raw_TRY(ph) == (char)27147);
            assert(pack.time_usec_GET() == 1651312679L);
            assert(pack.servo9_raw_TRY(ph) == (char)3472);
            assert(pack.servo16_raw_TRY(ph) == (char)11151);
            assert(pack.servo1_raw_GET() == (char)32318);
            assert(pack.servo7_raw_GET() == (char)38341);
            assert(pack.port_GET() == (char)233);
            assert(pack.servo6_raw_GET() == (char)62640);
            assert(pack.servo5_raw_GET() == (char)32110);
            assert(pack.servo2_raw_GET() == (char)5649);
            assert(pack.servo4_raw_GET() == (char)8442);
            assert(pack.servo13_raw_TRY(ph) == (char)37606);
            assert(pack.servo8_raw_GET() == (char)23270);
            assert(pack.servo14_raw_TRY(ph) == (char)37447);
            assert(pack.servo12_raw_TRY(ph) == (char)44350);
            assert(pack.servo3_raw_GET() == (char)29291);
            assert(pack.servo11_raw_TRY(ph) == (char)4861);
            assert(pack.servo10_raw_TRY(ph) == (char)52387);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo15_raw_SET((char)27147, PH) ;
        p36.servo9_raw_SET((char)3472, PH) ;
        p36.servo11_raw_SET((char)4861, PH) ;
        p36.servo8_raw_SET((char)23270) ;
        p36.servo2_raw_SET((char)5649) ;
        p36.servo1_raw_SET((char)32318) ;
        p36.servo16_raw_SET((char)11151, PH) ;
        p36.servo7_raw_SET((char)38341) ;
        p36.port_SET((char)233) ;
        p36.servo10_raw_SET((char)52387, PH) ;
        p36.servo5_raw_SET((char)32110) ;
        p36.servo3_raw_SET((char)29291) ;
        p36.servo14_raw_SET((char)37447, PH) ;
        p36.servo4_raw_SET((char)8442) ;
        p36.servo6_raw_SET((char)62640) ;
        p36.servo12_raw_SET((char)44350, PH) ;
        p36.time_usec_SET(1651312679L) ;
        p36.servo13_raw_SET((char)37606, PH) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)9);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)102);
            assert(pack.end_index_GET() == (short) -5170);
            assert(pack.start_index_GET() == (short)29038);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.end_index_SET((short) -5170) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p37.target_component_SET((char)9) ;
        p37.start_index_SET((short)29038) ;
        p37.target_system_SET((char)102) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_index_GET() == (short)22932);
            assert(pack.target_system_GET() == (char)0);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.start_index_GET() == (short) -12102);
            assert(pack.target_component_GET() == (char)152);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.end_index_SET((short)22932) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p38.start_index_SET((short) -12102) ;
        p38.target_component_SET((char)152) ;
        p38.target_system_SET((char)0) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.param1_GET() == 4.1932591E37F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_SOLO_BTN_FLY_HOLD);
            assert(pack.z_GET() == 1.9293474E38F);
            assert(pack.target_component_GET() == (char)59);
            assert(pack.param2_GET() == 1.8152136E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.y_GET() == -2.4291096E38F);
            assert(pack.current_GET() == (char)20);
            assert(pack.x_GET() == 1.0114764E38F);
            assert(pack.target_system_GET() == (char)3);
            assert(pack.param4_GET() == -9.231156E37F);
            assert(pack.param3_GET() == 1.5827344E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.autocontinue_GET() == (char)76);
            assert(pack.seq_GET() == (char)10623);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p39.seq_SET((char)10623) ;
        p39.param3_SET(1.5827344E38F) ;
        p39.command_SET(MAV_CMD.MAV_CMD_SOLO_BTN_FLY_HOLD) ;
        p39.x_SET(1.0114764E38F) ;
        p39.current_SET((char)20) ;
        p39.y_SET(-2.4291096E38F) ;
        p39.target_system_SET((char)3) ;
        p39.z_SET(1.9293474E38F) ;
        p39.target_component_SET((char)59) ;
        p39.param2_SET(1.8152136E38F) ;
        p39.autocontinue_SET((char)76) ;
        p39.param4_SET(-9.231156E37F) ;
        p39.param1_SET(4.1932591E37F) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)139);
            assert(pack.target_component_GET() == (char)225);
            assert(pack.seq_GET() == (char)62663);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_system_SET((char)139) ;
        p40.seq_SET((char)62663) ;
        p40.target_component_SET((char)225) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)50191);
            assert(pack.target_component_GET() == (char)173);
            assert(pack.target_system_GET() == (char)53);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)53) ;
        p41.target_component_SET((char)173) ;
        p41.seq_SET((char)50191) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)55941);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)55941) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)39);
            assert(pack.target_system_GET() == (char)222);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_component_SET((char)39) ;
        p43.target_system_SET((char)222) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)121);
            assert(pack.target_system_GET() == (char)41);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.count_GET() == (char)51570);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p44.target_component_SET((char)121) ;
        p44.target_system_SET((char)41) ;
        p44.count_SET((char)51570) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)156);
            assert(pack.target_system_GET() == (char)151);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)151) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p45.target_component_SET((char)156) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)6794);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)6794) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)228);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED);
            assert(pack.target_component_GET() == (char)56);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p47.target_component_SET((char)56) ;
        p47.target_system_SET((char)228) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 109197590);
            assert(pack.target_system_GET() == (char)187);
            assert(pack.altitude_GET() == 96939727);
            assert(pack.time_usec_TRY(ph) == 977366672858751512L);
            assert(pack.longitude_GET() == 543768281);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.altitude_SET(96939727) ;
        p48.time_usec_SET(977366672858751512L, PH) ;
        p48.target_system_SET((char)187) ;
        p48.latitude_SET(109197590) ;
        p48.longitude_SET(543768281) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 4063591726155845639L);
            assert(pack.latitude_GET() == 839989442);
            assert(pack.longitude_GET() == -1890722826);
            assert(pack.altitude_GET() == -172990511);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.altitude_SET(-172990511) ;
        p49.latitude_SET(839989442) ;
        p49.longitude_SET(-1890722826) ;
        p49.time_usec_SET(4063591726155845639L, PH) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.parameter_rc_channel_index_GET() == (char)196);
            assert(pack.param_value_max_GET() == -2.927972E38F);
            assert(pack.param_value0_GET() == -8.454219E37F);
            assert(pack.param_id_LEN(ph) == 11);
            assert(pack.param_id_TRY(ph).equals("tkysfnekbit"));
            assert(pack.param_index_GET() == (short)12100);
            assert(pack.target_component_GET() == (char)156);
            assert(pack.param_value_min_GET() == -1.3062144E38F);
            assert(pack.scale_GET() == -6.9486323E37F);
            assert(pack.target_system_GET() == (char)48);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.target_component_SET((char)156) ;
        p50.scale_SET(-6.9486323E37F) ;
        p50.param_id_SET("tkysfnekbit", PH) ;
        p50.param_value_max_SET(-2.927972E38F) ;
        p50.param_index_SET((short)12100) ;
        p50.target_system_SET((char)48) ;
        p50.param_value_min_SET(-1.3062144E38F) ;
        p50.param_value0_SET(-8.454219E37F) ;
        p50.parameter_rc_channel_index_SET((char)196) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)23466);
            assert(pack.target_component_GET() == (char)202);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)0);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_component_SET((char)202) ;
        p51.seq_SET((char)23466) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p51.target_system_SET((char)0) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.target_system_GET() == (char)19);
            assert(pack.p1z_GET() == 1.3006676E38F);
            assert(pack.p2x_GET() == -3.3822559E38F);
            assert(pack.target_component_GET() == (char)238);
            assert(pack.p1x_GET() == 3.3380043E38F);
            assert(pack.p2z_GET() == 2.1306207E37F);
            assert(pack.p2y_GET() == -3.1817274E38F);
            assert(pack.p1y_GET() == 8.1772124E37F);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p1y_SET(8.1772124E37F) ;
        p54.p1x_SET(3.3380043E38F) ;
        p54.p2x_SET(-3.3822559E38F) ;
        p54.p1z_SET(1.3006676E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p54.target_component_SET((char)238) ;
        p54.p2y_SET(-3.1817274E38F) ;
        p54.target_system_SET((char)19) ;
        p54.p2z_SET(2.1306207E37F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1x_GET() == -2.144563E37F);
            assert(pack.p2x_GET() == 2.8351478E38F);
            assert(pack.p2y_GET() == 3.3464842E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.p2z_GET() == 3.1906922E38F);
            assert(pack.p1z_GET() == 7.0787795E37F);
            assert(pack.p1y_GET() == -2.1437915E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p55.p1z_SET(7.0787795E37F) ;
        p55.p2y_SET(3.3464842E38F) ;
        p55.p2x_SET(2.8351478E38F) ;
        p55.p1y_SET(-2.1437915E38F) ;
        p55.p1x_SET(-2.144563E37F) ;
        p55.p2z_SET(3.1906922E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 135002151175170340L);
            assert(pack.rollspeed_GET() == 2.0326274E38F);
            assert(pack.yawspeed_GET() == -7.4448156E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.4658407E38F, -3.3255992E38F, 9.484599E37F, 3.8616985E37F}));
            assert(pack.pitchspeed_GET() == -3.3869598E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {1.7072158E38F, -1.6865531E38F, -3.9402884E36F, -1.137799E38F, 9.630106E37F, -3.0027556E38F, -6.4842813E37F, 1.4200897E38F, -1.9340288E38F}));
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.covariance_SET(new float[] {1.7072158E38F, -1.6865531E38F, -3.9402884E36F, -1.137799E38F, 9.630106E37F, -3.0027556E38F, -6.4842813E37F, 1.4200897E38F, -1.9340288E38F}, 0) ;
        p61.rollspeed_SET(2.0326274E38F) ;
        p61.yawspeed_SET(-7.4448156E37F) ;
        p61.pitchspeed_SET(-3.3869598E38F) ;
        p61.q_SET(new float[] {-1.4658407E38F, -3.3255992E38F, 9.484599E37F, 3.8616985E37F}, 0) ;
        p61.time_usec_SET(135002151175170340L) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.xtrack_error_GET() == -2.4876095E38F);
            assert(pack.nav_bearing_GET() == (short)17880);
            assert(pack.aspd_error_GET() == -2.1479114E38F);
            assert(pack.nav_pitch_GET() == 2.890637E38F);
            assert(pack.target_bearing_GET() == (short)27824);
            assert(pack.alt_error_GET() == 2.420924E38F);
            assert(pack.wp_dist_GET() == (char)30183);
            assert(pack.nav_roll_GET() == 1.9211172E38F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.alt_error_SET(2.420924E38F) ;
        p62.aspd_error_SET(-2.1479114E38F) ;
        p62.xtrack_error_SET(-2.4876095E38F) ;
        p62.nav_bearing_SET((short)17880) ;
        p62.nav_roll_SET(1.9211172E38F) ;
        p62.target_bearing_SET((short)27824) ;
        p62.wp_dist_SET((char)30183) ;
        p62.nav_pitch_SET(2.890637E38F) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == -3.3546742E38F);
            assert(pack.relative_alt_GET() == 727682649);
            assert(pack.time_usec_GET() == 7739275235954993584L);
            assert(pack.alt_GET() == -1459764030);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.2817624E38F, 1.6092969E38F, -2.2739297E38F, 6.5422667E37F, -2.5487845E38F, 1.0940721E38F, 2.9029615E38F, 1.9380047E38F, 3.0995692E38F, 2.5970528E38F, -3.3840779E38F, 7.491155E36F, -1.3028356E38F, -2.4816337E38F, 2.0620992E38F, 2.9234443E38F, -9.573674E37F, -2.5843568E38F, -6.085712E37F, 2.4818236E38F, -1.4622645E38F, 1.762339E38F, -9.813918E35F, -1.0962842E38F, 9.5638844E36F, 1.023982E38F, -1.7665656E38F, 2.7630439E38F, 9.3390647E36F, 8.133432E37F, -3.9768266E37F, 1.5553609E38F, -1.5513392E38F, -2.0467981E38F, -1.5001123E38F, 1.3107618E38F}));
            assert(pack.lat_GET() == -1047934623);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
            assert(pack.vz_GET() == 8.853233E37F);
            assert(pack.vy_GET() == -5.7155475E37F);
            assert(pack.lon_GET() == 698257905);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.vz_SET(8.853233E37F) ;
        p63.vy_SET(-5.7155475E37F) ;
        p63.lon_SET(698257905) ;
        p63.alt_SET(-1459764030) ;
        p63.relative_alt_SET(727682649) ;
        p63.time_usec_SET(7739275235954993584L) ;
        p63.covariance_SET(new float[] {-2.2817624E38F, 1.6092969E38F, -2.2739297E38F, 6.5422667E37F, -2.5487845E38F, 1.0940721E38F, 2.9029615E38F, 1.9380047E38F, 3.0995692E38F, 2.5970528E38F, -3.3840779E38F, 7.491155E36F, -1.3028356E38F, -2.4816337E38F, 2.0620992E38F, 2.9234443E38F, -9.573674E37F, -2.5843568E38F, -6.085712E37F, 2.4818236E38F, -1.4622645E38F, 1.762339E38F, -9.813918E35F, -1.0962842E38F, 9.5638844E36F, 1.023982E38F, -1.7665656E38F, 2.7630439E38F, 9.3390647E36F, 8.133432E37F, -3.9768266E37F, 1.5553609E38F, -1.5513392E38F, -2.0467981E38F, -1.5001123E38F, 1.3107618E38F}, 0) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION) ;
        p63.lat_SET(-1047934623) ;
        p63.vx_SET(-3.3546742E38F) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 2.1179651E37F);
            assert(pack.y_GET() == -7.617942E36F);
            assert(pack.time_usec_GET() == 3572078806838282726L);
            assert(pack.x_GET() == -9.337353E37F);
            assert(pack.ay_GET() == -3.9282188E37F);
            assert(pack.az_GET() == -1.8101629E38F);
            assert(pack.vx_GET() == 2.092998E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {7.763302E37F, 1.9102111E38F, 3.2661123E38F, -3.3643845E37F, -2.65797E38F, 1.5102264E38F, 8.0381735E37F, -1.8363515E38F, -2.6638619E38F, 2.4555289E38F, 6.488891E37F, -4.044772E37F, -1.405947E38F, 2.7741128E38F, 2.057534E38F, -1.7199524E37F, -2.4005048E37F, -9.949682E37F, -2.470977E38F, -1.7209237E38F, 2.8790027E38F, 4.8652993E37F, 1.0763216E38F, 2.3320043E38F, -3.4000367E38F, -1.7168151E38F, 1.1404637E38F, -1.268015E38F, 2.1865125E38F, 8.0163643E37F, 3.2997312E38F, 1.4700198E38F, -1.9346451E38F, 2.2383026E38F, -2.8849369E38F, -6.9833173E37F, 9.702862E37F, 2.0686206E38F, -1.01914576E37F, 1.098585E37F, -1.4619374E38F, 1.4206478E37F, -5.4330516E37F, 1.2965693E38F, 1.1474675E38F}));
            assert(pack.ax_GET() == -2.4244416E38F);
            assert(pack.vz_GET() == 3.2994404E38F);
            assert(pack.vy_GET() == 6.172482E37F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.x_SET(-9.337353E37F) ;
        p64.time_usec_SET(3572078806838282726L) ;
        p64.ay_SET(-3.9282188E37F) ;
        p64.az_SET(-1.8101629E38F) ;
        p64.vx_SET(2.092998E38F) ;
        p64.y_SET(-7.617942E36F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        p64.vz_SET(3.2994404E38F) ;
        p64.vy_SET(6.172482E37F) ;
        p64.covariance_SET(new float[] {7.763302E37F, 1.9102111E38F, 3.2661123E38F, -3.3643845E37F, -2.65797E38F, 1.5102264E38F, 8.0381735E37F, -1.8363515E38F, -2.6638619E38F, 2.4555289E38F, 6.488891E37F, -4.044772E37F, -1.405947E38F, 2.7741128E38F, 2.057534E38F, -1.7199524E37F, -2.4005048E37F, -9.949682E37F, -2.470977E38F, -1.7209237E38F, 2.8790027E38F, 4.8652993E37F, 1.0763216E38F, 2.3320043E38F, -3.4000367E38F, -1.7168151E38F, 1.1404637E38F, -1.268015E38F, 2.1865125E38F, 8.0163643E37F, 3.2997312E38F, 1.4700198E38F, -1.9346451E38F, 2.2383026E38F, -2.8849369E38F, -6.9833173E37F, 9.702862E37F, 2.0686206E38F, -1.01914576E37F, 1.098585E37F, -1.4619374E38F, 1.4206478E37F, -5.4330516E37F, 1.2965693E38F, 1.1474675E38F}, 0) ;
        p64.ax_SET(-2.4244416E38F) ;
        p64.z_SET(2.1179651E37F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan14_raw_GET() == (char)25637);
            assert(pack.chan12_raw_GET() == (char)486);
            assert(pack.time_boot_ms_GET() == 3974229610L);
            assert(pack.chan6_raw_GET() == (char)16079);
            assert(pack.chan3_raw_GET() == (char)47042);
            assert(pack.chan18_raw_GET() == (char)24850);
            assert(pack.chan10_raw_GET() == (char)1160);
            assert(pack.chan17_raw_GET() == (char)55740);
            assert(pack.chan13_raw_GET() == (char)52715);
            assert(pack.chan2_raw_GET() == (char)51444);
            assert(pack.chan4_raw_GET() == (char)54931);
            assert(pack.chan1_raw_GET() == (char)10975);
            assert(pack.chan5_raw_GET() == (char)60266);
            assert(pack.chan9_raw_GET() == (char)34156);
            assert(pack.chan8_raw_GET() == (char)9208);
            assert(pack.chancount_GET() == (char)117);
            assert(pack.chan15_raw_GET() == (char)7358);
            assert(pack.chan7_raw_GET() == (char)28229);
            assert(pack.chan11_raw_GET() == (char)57768);
            assert(pack.rssi_GET() == (char)126);
            assert(pack.chan16_raw_GET() == (char)15055);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan2_raw_SET((char)51444) ;
        p65.chan8_raw_SET((char)9208) ;
        p65.chan12_raw_SET((char)486) ;
        p65.chancount_SET((char)117) ;
        p65.rssi_SET((char)126) ;
        p65.chan6_raw_SET((char)16079) ;
        p65.chan7_raw_SET((char)28229) ;
        p65.time_boot_ms_SET(3974229610L) ;
        p65.chan3_raw_SET((char)47042) ;
        p65.chan11_raw_SET((char)57768) ;
        p65.chan10_raw_SET((char)1160) ;
        p65.chan16_raw_SET((char)15055) ;
        p65.chan1_raw_SET((char)10975) ;
        p65.chan4_raw_SET((char)54931) ;
        p65.chan5_raw_SET((char)60266) ;
        p65.chan13_raw_SET((char)52715) ;
        p65.chan14_raw_SET((char)25637) ;
        p65.chan9_raw_SET((char)34156) ;
        p65.chan18_raw_SET((char)24850) ;
        p65.chan17_raw_SET((char)55740) ;
        p65.chan15_raw_SET((char)7358) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.req_stream_id_GET() == (char)252);
            assert(pack.target_component_GET() == (char)231);
            assert(pack.target_system_GET() == (char)138);
            assert(pack.req_message_rate_GET() == (char)1245);
            assert(pack.start_stop_GET() == (char)208);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.start_stop_SET((char)208) ;
        p66.target_system_SET((char)138) ;
        p66.target_component_SET((char)231) ;
        p66.req_stream_id_SET((char)252) ;
        p66.req_message_rate_SET((char)1245) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.on_off_GET() == (char)115);
            assert(pack.stream_id_GET() == (char)12);
            assert(pack.message_rate_GET() == (char)12668);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.on_off_SET((char)115) ;
        p67.message_rate_SET((char)12668) ;
        p67.stream_id_SET((char)12) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.buttons_GET() == (char)51375);
            assert(pack.target_GET() == (char)210);
            assert(pack.y_GET() == (short)23487);
            assert(pack.x_GET() == (short) -19185);
            assert(pack.z_GET() == (short)19145);
            assert(pack.r_GET() == (short) -19470);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.y_SET((short)23487) ;
        p69.z_SET((short)19145) ;
        p69.x_SET((short) -19185) ;
        p69.buttons_SET((char)51375) ;
        p69.target_SET((char)210) ;
        p69.r_SET((short) -19470) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan3_raw_GET() == (char)38842);
            assert(pack.chan5_raw_GET() == (char)41999);
            assert(pack.chan8_raw_GET() == (char)35452);
            assert(pack.chan1_raw_GET() == (char)51951);
            assert(pack.chan2_raw_GET() == (char)11824);
            assert(pack.target_system_GET() == (char)124);
            assert(pack.chan7_raw_GET() == (char)19208);
            assert(pack.chan4_raw_GET() == (char)17488);
            assert(pack.chan6_raw_GET() == (char)46763);
            assert(pack.target_component_GET() == (char)250);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan2_raw_SET((char)11824) ;
        p70.chan1_raw_SET((char)51951) ;
        p70.chan5_raw_SET((char)41999) ;
        p70.target_system_SET((char)124) ;
        p70.chan4_raw_SET((char)17488) ;
        p70.target_component_SET((char)250) ;
        p70.chan7_raw_SET((char)19208) ;
        p70.chan8_raw_SET((char)35452) ;
        p70.chan6_raw_SET((char)46763) ;
        p70.chan3_raw_SET((char)38842) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)153);
            assert(pack.x_GET() == -1724958772);
            assert(pack.param1_GET() == -1.2968852E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.z_GET() == 1.4758081E38F);
            assert(pack.target_system_GET() == (char)14);
            assert(pack.param4_GET() == 2.6332982E37F);
            assert(pack.y_GET() == 2057199149);
            assert(pack.param2_GET() == 1.0473201E38F);
            assert(pack.autocontinue_GET() == (char)106);
            assert(pack.seq_GET() == (char)1540);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.current_GET() == (char)100);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION);
            assert(pack.param3_GET() == 1.7302776E38F);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.param2_SET(1.0473201E38F) ;
        p73.z_SET(1.4758081E38F) ;
        p73.x_SET(-1724958772) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p73.param4_SET(2.6332982E37F) ;
        p73.current_SET((char)100) ;
        p73.target_system_SET((char)14) ;
        p73.seq_SET((char)1540) ;
        p73.param3_SET(1.7302776E38F) ;
        p73.target_component_SET((char)153) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p73.command_SET(MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION) ;
        p73.y_SET(2057199149) ;
        p73.param1_SET(-1.2968852E38F) ;
        p73.autocontinue_SET((char)106) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 9.084086E37F);
            assert(pack.groundspeed_GET() == -1.407403E38F);
            assert(pack.airspeed_GET() == 1.874714E38F);
            assert(pack.climb_GET() == -2.3918267E37F);
            assert(pack.heading_GET() == (short)10372);
            assert(pack.throttle_GET() == (char)39903);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.groundspeed_SET(-1.407403E38F) ;
        p74.airspeed_SET(1.874714E38F) ;
        p74.climb_SET(-2.3918267E37F) ;
        p74.alt_SET(9.084086E37F) ;
        p74.heading_SET((short)10372) ;
        p74.throttle_SET((char)39903) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.param2_GET() == -1.5373069E38F);
            assert(pack.z_GET() == 1.9875104E38F);
            assert(pack.param4_GET() == -2.9472187E37F);
            assert(pack.target_system_GET() == (char)140);
            assert(pack.current_GET() == (char)254);
            assert(pack.target_component_GET() == (char)87);
            assert(pack.x_GET() == -971821529);
            assert(pack.param1_GET() == 2.9765318E38F);
            assert(pack.autocontinue_GET() == (char)88);
            assert(pack.y_GET() == 2037495472);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_PANORAMA_CREATE);
            assert(pack.param3_GET() == -3.1884873E38F);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.y_SET(2037495472) ;
        p75.command_SET(MAV_CMD.MAV_CMD_PANORAMA_CREATE) ;
        p75.param2_SET(-1.5373069E38F) ;
        p75.autocontinue_SET((char)88) ;
        p75.current_SET((char)254) ;
        p75.param1_SET(2.9765318E38F) ;
        p75.target_component_SET((char)87) ;
        p75.z_SET(1.9875104E38F) ;
        p75.x_SET(-971821529) ;
        p75.param4_SET(-2.9472187E37F) ;
        p75.param3_SET(-3.1884873E38F) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p75.target_system_SET((char)140) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param1_GET() == 3.1449038E38F);
            assert(pack.param3_GET() == -3.1358836E38F);
            assert(pack.param4_GET() == 3.3611731E38F);
            assert(pack.confirmation_GET() == (char)169);
            assert(pack.target_system_GET() == (char)92);
            assert(pack.target_component_GET() == (char)146);
            assert(pack.param6_GET() == -2.7315948E38F);
            assert(pack.param5_GET() == 9.694782E37F);
            assert(pack.param2_GET() == 8.971675E37F);
            assert(pack.param7_GET() == 2.5124066E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE);
        });
        COMMAND_LONG p76 = new COMMAND_LONG();
        PH.setPack(p76);
        p76.target_system_SET((char)92) ;
        p76.param4_SET(3.3611731E38F) ;
        p76.param2_SET(8.971675E37F) ;
        p76.param7_SET(2.5124066E38F) ;
        p76.param6_SET(-2.7315948E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE) ;
        p76.param5_SET(9.694782E37F) ;
        p76.param3_SET(-3.1358836E38F) ;
        p76.confirmation_SET((char)169) ;
        p76.target_component_SET((char)146) ;
        p76.param1_SET(3.1449038E38F) ;
        TestChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_TRY(ph) == (char)108);
            assert(pack.target_component_TRY(ph) == (char)150);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_UNSUPPORTED);
            assert(pack.progress_TRY(ph) == (char)243);
            assert(pack.result_param2_TRY(ph) == -2012130596);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_GET_HOME_POSITION);
        });
        COMMAND_ACK p77 = new COMMAND_ACK();
        PH.setPack(p77);
        p77.target_component_SET((char)150, PH) ;
        p77.result_param2_SET(-2012130596, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_GET_HOME_POSITION) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_UNSUPPORTED) ;
        p77.target_system_SET((char)108, PH) ;
        p77.progress_SET((char)243, PH) ;
        TestChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.manual_override_switch_GET() == (char)142);
            assert(pack.pitch_GET() == -1.325875E37F);
            assert(pack.yaw_GET() == 1.1901121E38F);
            assert(pack.thrust_GET() == 1.3704565E38F);
            assert(pack.mode_switch_GET() == (char)19);
            assert(pack.time_boot_ms_GET() == 3921646135L);
            assert(pack.roll_GET() == -2.9930602E38F);
        });
        MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.manual_override_switch_SET((char)142) ;
        p81.roll_SET(-2.9930602E38F) ;
        p81.thrust_SET(1.3704565E38F) ;
        p81.yaw_SET(1.1901121E38F) ;
        p81.pitch_SET(-1.325875E37F) ;
        p81.time_boot_ms_SET(3921646135L) ;
        p81.mode_switch_SET((char)19) ;
        TestChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_yaw_rate_GET() == -1.169844E37F);
            assert(pack.target_component_GET() == (char)228);
            assert(pack.target_system_GET() == (char)170);
            assert(pack.time_boot_ms_GET() == 2840201232L);
            assert(pack.body_pitch_rate_GET() == -3.3649032E37F);
            assert(pack.body_roll_rate_GET() == -1.780747E38F);
            assert(pack.type_mask_GET() == (char)34);
            assert(pack.thrust_GET() == -1.2217949E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.180391E38F, -2.594912E38F, -2.4346617E38F, -2.9302705E38F}));
        });
        SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.thrust_SET(-1.2217949E38F) ;
        p82.time_boot_ms_SET(2840201232L) ;
        p82.target_system_SET((char)170) ;
        p82.q_SET(new float[] {-2.180391E38F, -2.594912E38F, -2.4346617E38F, -2.9302705E38F}, 0) ;
        p82.body_pitch_rate_SET(-3.3649032E37F) ;
        p82.type_mask_SET((char)34) ;
        p82.target_component_SET((char)228) ;
        p82.body_roll_rate_SET(-1.780747E38F) ;
        p82.body_yaw_rate_SET(-1.169844E37F) ;
        TestChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3624002321L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {8.458765E37F, -2.0981481E38F, 2.3927072E37F, 1.5997064E38F}));
            assert(pack.body_yaw_rate_GET() == 2.3462604E38F);
            assert(pack.thrust_GET() == -1.6123247E38F);
            assert(pack.type_mask_GET() == (char)115);
            assert(pack.body_roll_rate_GET() == 9.716038E37F);
            assert(pack.body_pitch_rate_GET() == -2.134798E38F);
        });
        ATTITUDE_TARGET p83 = new ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.body_yaw_rate_SET(2.3462604E38F) ;
        p83.time_boot_ms_SET(3624002321L) ;
        p83.q_SET(new float[] {8.458765E37F, -2.0981481E38F, 2.3927072E37F, 1.5997064E38F}, 0) ;
        p83.thrust_SET(-1.6123247E38F) ;
        p83.type_mask_SET((char)115) ;
        p83.body_pitch_rate_SET(-2.134798E38F) ;
        p83.body_roll_rate_SET(9.716038E37F) ;
        TestChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)133);
            assert(pack.vy_GET() == 2.33655E38F);
            assert(pack.yaw_GET() == 1.2682474E38F);
            assert(pack.afz_GET() == 1.3966656E38F);
            assert(pack.yaw_rate_GET() == 2.5105423E38F);
            assert(pack.afx_GET() == -1.9373565E38F);
            assert(pack.vx_GET() == -1.2960085E38F);
            assert(pack.type_mask_GET() == (char)27574);
            assert(pack.x_GET() == -1.4785617E38F);
            assert(pack.z_GET() == -3.2616114E38F);
            assert(pack.vz_GET() == -3.0269726E38F);
            assert(pack.afy_GET() == 2.727959E38F);
            assert(pack.y_GET() == -3.2349332E38F);
            assert(pack.time_boot_ms_GET() == 4126054530L);
            assert(pack.target_component_GET() == (char)53);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
        });
        SET_POSITION_TARGET_LOCAL_NED p84 = new SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.afy_SET(2.727959E38F) ;
        p84.type_mask_SET((char)27574) ;
        p84.target_component_SET((char)53) ;
        p84.yaw_rate_SET(2.5105423E38F) ;
        p84.vx_SET(-1.2960085E38F) ;
        p84.time_boot_ms_SET(4126054530L) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p84.yaw_SET(1.2682474E38F) ;
        p84.target_system_SET((char)133) ;
        p84.vy_SET(2.33655E38F) ;
        p84.vz_SET(-3.0269726E38F) ;
        p84.y_SET(-3.2349332E38F) ;
        p84.z_SET(-3.2616114E38F) ;
        p84.x_SET(-1.4785617E38F) ;
        p84.afx_SET(-1.9373565E38F) ;
        p84.afz_SET(1.3966656E38F) ;
        TestChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1358273000L);
            assert(pack.target_system_GET() == (char)75);
            assert(pack.alt_GET() == 6.401032E37F);
            assert(pack.vy_GET() == 2.8364818E38F);
            assert(pack.lon_int_GET() == 669978922);
            assert(pack.afz_GET() == -2.8997195E38F);
            assert(pack.target_component_GET() == (char)237);
            assert(pack.vx_GET() == 2.5303024E38F);
            assert(pack.yaw_rate_GET() == 2.2995551E38F);
            assert(pack.afx_GET() == -2.2954718E38F);
            assert(pack.yaw_GET() == 5.122387E36F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.afy_GET() == -5.8974975E37F);
            assert(pack.lat_int_GET() == -1478416337);
            assert(pack.type_mask_GET() == (char)56865);
            assert(pack.vz_GET() == 1.3100856E38F);
        });
        SET_POSITION_TARGET_GLOBAL_INT p86 = new SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.afy_SET(-5.8974975E37F) ;
        p86.yaw_rate_SET(2.2995551E38F) ;
        p86.target_component_SET((char)237) ;
        p86.target_system_SET((char)75) ;
        p86.afz_SET(-2.8997195E38F) ;
        p86.type_mask_SET((char)56865) ;
        p86.vz_SET(1.3100856E38F) ;
        p86.alt_SET(6.401032E37F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p86.time_boot_ms_SET(1358273000L) ;
        p86.yaw_SET(5.122387E36F) ;
        p86.vx_SET(2.5303024E38F) ;
        p86.lat_int_SET(-1478416337) ;
        p86.afx_SET(-2.2954718E38F) ;
        p86.vy_SET(2.8364818E38F) ;
        p86.lon_int_SET(669978922) ;
        TestChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 3.7803678E37F);
            assert(pack.yaw_rate_GET() == -7.806783E37F);
            assert(pack.vy_GET() == 1.2656391E38F);
            assert(pack.alt_GET() == 7.299547E36F);
            assert(pack.lat_int_GET() == 397410486);
            assert(pack.vz_GET() == -2.656685E37F);
            assert(pack.afx_GET() == -1.3426196E38F);
            assert(pack.afz_GET() == 3.356465E37F);
            assert(pack.lon_int_GET() == 1503691381);
            assert(pack.type_mask_GET() == (char)4238);
            assert(pack.afy_GET() == -2.1820936E38F);
            assert(pack.vx_GET() == -1.6996803E38F);
            assert(pack.time_boot_ms_GET() == 1327962318L);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
        });
        POSITION_TARGET_GLOBAL_INT p87 = new POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.vy_SET(1.2656391E38F) ;
        p87.time_boot_ms_SET(1327962318L) ;
        p87.type_mask_SET((char)4238) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p87.lat_int_SET(397410486) ;
        p87.yaw_SET(3.7803678E37F) ;
        p87.afz_SET(3.356465E37F) ;
        p87.lon_int_SET(1503691381) ;
        p87.afx_SET(-1.3426196E38F) ;
        p87.vx_SET(-1.6996803E38F) ;
        p87.vz_SET(-2.656685E37F) ;
        p87.yaw_rate_SET(-7.806783E37F) ;
        p87.alt_SET(7.299547E36F) ;
        p87.afy_SET(-2.1820936E38F) ;
        TestChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -9.511989E37F);
            assert(pack.y_GET() == 1.2984247E38F);
            assert(pack.z_GET() == 1.2222958E38F);
            assert(pack.time_boot_ms_GET() == 230361987L);
            assert(pack.pitch_GET() == 2.3777925E38F);
            assert(pack.roll_GET() == -2.1175633E38F);
            assert(pack.yaw_GET() == -2.424296E38F);
        });
        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = new LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.time_boot_ms_SET(230361987L) ;
        p89.pitch_SET(2.3777925E38F) ;
        p89.y_SET(1.2984247E38F) ;
        p89.yaw_SET(-2.424296E38F) ;
        p89.x_SET(-9.511989E37F) ;
        p89.roll_SET(-2.1175633E38F) ;
        p89.z_SET(1.2222958E38F) ;
        TestChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == (short)21469);
            assert(pack.pitch_GET() == 9.974874E37F);
            assert(pack.alt_GET() == 1288264069);
            assert(pack.yacc_GET() == (short) -13382);
            assert(pack.pitchspeed_GET() == 1.9324145E38F);
            assert(pack.lat_GET() == 1370750575);
            assert(pack.vx_GET() == (short)30986);
            assert(pack.vz_GET() == (short)12326);
            assert(pack.yawspeed_GET() == 3.0911727E38F);
            assert(pack.yaw_GET() == 2.046647E38F);
            assert(pack.roll_GET() == 4.4713333E37F);
            assert(pack.rollspeed_GET() == -2.8970013E38F);
            assert(pack.lon_GET() == 382643124);
            assert(pack.time_usec_GET() == 2943862135196613308L);
            assert(pack.zacc_GET() == (short)8212);
            assert(pack.xacc_GET() == (short) -7087);
        });
        HIL_STATE p90 = new HIL_STATE();
        PH.setPack(p90);
        p90.vy_SET((short)21469) ;
        p90.lon_SET(382643124) ;
        p90.pitchspeed_SET(1.9324145E38F) ;
        p90.rollspeed_SET(-2.8970013E38F) ;
        p90.roll_SET(4.4713333E37F) ;
        p90.alt_SET(1288264069) ;
        p90.zacc_SET((short)8212) ;
        p90.xacc_SET((short) -7087) ;
        p90.yaw_SET(2.046647E38F) ;
        p90.lat_SET(1370750575) ;
        p90.vx_SET((short)30986) ;
        p90.vz_SET((short)12326) ;
        p90.pitch_SET(9.974874E37F) ;
        p90.yacc_SET((short) -13382) ;
        p90.time_usec_SET(2943862135196613308L) ;
        p90.yawspeed_SET(3.0911727E38F) ;
        TestChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.aux2_GET() == -2.27194E38F);
            assert(pack.roll_ailerons_GET() == 1.4887359E38F);
            assert(pack.nav_mode_GET() == (char)121);
            assert(pack.aux4_GET() == 2.1138964E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
            assert(pack.pitch_elevator_GET() == -8.510559E37F);
            assert(pack.throttle_GET() == 1.6865393E38F);
            assert(pack.yaw_rudder_GET() == 6.5420243E37F);
            assert(pack.aux3_GET() == -1.0344356E38F);
            assert(pack.aux1_GET() == 6.912642E36F);
            assert(pack.time_usec_GET() == 8835461450306809070L);
        });
        HIL_CONTROLS p91 = new HIL_CONTROLS();
        PH.setPack(p91);
        p91.time_usec_SET(8835461450306809070L) ;
        p91.aux2_SET(-2.27194E38F) ;
        p91.aux1_SET(6.912642E36F) ;
        p91.throttle_SET(1.6865393E38F) ;
        p91.pitch_elevator_SET(-8.510559E37F) ;
        p91.nav_mode_SET((char)121) ;
        p91.roll_ailerons_SET(1.4887359E38F) ;
        p91.yaw_rudder_SET(6.5420243E37F) ;
        p91.aux4_SET(2.1138964E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_GUIDED_DISARMED) ;
        p91.aux3_SET(-1.0344356E38F) ;
        TestChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan11_raw_GET() == (char)27074);
            assert(pack.chan8_raw_GET() == (char)12656);
            assert(pack.chan10_raw_GET() == (char)46717);
            assert(pack.rssi_GET() == (char)216);
            assert(pack.time_usec_GET() == 4802730722333975318L);
            assert(pack.chan3_raw_GET() == (char)46682);
            assert(pack.chan2_raw_GET() == (char)21265);
            assert(pack.chan4_raw_GET() == (char)48221);
            assert(pack.chan5_raw_GET() == (char)6500);
            assert(pack.chan7_raw_GET() == (char)60944);
            assert(pack.chan6_raw_GET() == (char)4389);
            assert(pack.chan12_raw_GET() == (char)35402);
            assert(pack.chan1_raw_GET() == (char)63349);
            assert(pack.chan9_raw_GET() == (char)64092);
        });
        HIL_RC_INPUTS_RAW p92 = new HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan7_raw_SET((char)60944) ;
        p92.chan5_raw_SET((char)6500) ;
        p92.time_usec_SET(4802730722333975318L) ;
        p92.chan2_raw_SET((char)21265) ;
        p92.chan8_raw_SET((char)12656) ;
        p92.chan1_raw_SET((char)63349) ;
        p92.rssi_SET((char)216) ;
        p92.chan9_raw_SET((char)64092) ;
        p92.chan3_raw_SET((char)46682) ;
        p92.chan11_raw_SET((char)27074) ;
        p92.chan10_raw_SET((char)46717) ;
        p92.chan4_raw_SET((char)48221) ;
        p92.chan6_raw_SET((char)4389) ;
        p92.chan12_raw_SET((char)35402) ;
        TestChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == 6758070133999636650L);
            assert(pack.time_usec_GET() == 9125989114797958036L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.0618136E38F, -2.0439935E38F, 1.8175483E38F, 1.4729149E38F, 8.30798E36F, 1.3866635E38F, -3.3186354E38F, 1.1233315E38F, -2.8378853E37F, -3.8013495E37F, 4.155236E37F, -2.4407487E38F, -2.766917E38F, 1.1737531E38F, -3.337758E38F, -2.9793756E38F}));
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_DISARMED);
        });
        HIL_ACTUATOR_CONTROLS p93 = new HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.controls_SET(new float[] {1.0618136E38F, -2.0439935E38F, 1.8175483E38F, 1.4729149E38F, 8.30798E36F, 1.3866635E38F, -3.3186354E38F, 1.1233315E38F, -2.8378853E37F, -3.8013495E37F, 4.155236E37F, -2.4407487E38F, -2.766917E38F, 1.1737531E38F, -3.337758E38F, -2.9793756E38F}, 0) ;
        p93.flags_SET(6758070133999636650L) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_TEST_DISARMED) ;
        p93.time_usec_SET(9125989114797958036L) ;
        TestChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 2232712929020734381L);
            assert(pack.flow_comp_m_y_GET() == -7.0097955E37F);
            assert(pack.flow_rate_y_TRY(ph) == 2.100882E38F);
            assert(pack.ground_distance_GET() == -2.4785198E37F);
            assert(pack.flow_y_GET() == (short)17581);
            assert(pack.flow_x_GET() == (short)3844);
            assert(pack.quality_GET() == (char)223);
            assert(pack.sensor_id_GET() == (char)40);
            assert(pack.flow_comp_m_x_GET() == -1.1180663E38F);
            assert(pack.flow_rate_x_TRY(ph) == -3.0411171E38F);
        });
        OPTICAL_FLOW p100 = new OPTICAL_FLOW();
        PH.setPack(p100);
        p100.time_usec_SET(2232712929020734381L) ;
        p100.flow_rate_x_SET(-3.0411171E38F, PH) ;
        p100.flow_comp_m_y_SET(-7.0097955E37F) ;
        p100.flow_rate_y_SET(2.100882E38F, PH) ;
        p100.flow_comp_m_x_SET(-1.1180663E38F) ;
        p100.quality_SET((char)223) ;
        p100.flow_y_SET((short)17581) ;
        p100.flow_x_SET((short)3844) ;
        p100.ground_distance_SET(-2.4785198E37F) ;
        p100.sensor_id_SET((char)40) ;
        TestChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 1.0883874E37F);
            assert(pack.y_GET() == -2.3925527E36F);
            assert(pack.x_GET() == 1.9215906E38F);
            assert(pack.roll_GET() == 1.6978834E38F);
            assert(pack.usec_GET() == 2848079450511219535L);
            assert(pack.yaw_GET() == 1.7445878E38F);
            assert(pack.pitch_GET() == 1.5590771E38F);
        });
        GLOBAL_VISION_POSITION_ESTIMATE p101 = new GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.usec_SET(2848079450511219535L) ;
        p101.yaw_SET(1.7445878E38F) ;
        p101.y_SET(-2.3925527E36F) ;
        p101.x_SET(1.9215906E38F) ;
        p101.z_SET(1.0883874E37F) ;
        p101.pitch_SET(1.5590771E38F) ;
        p101.roll_SET(1.6978834E38F) ;
        TestChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 2.3419804E38F);
            assert(pack.x_GET() == 5.7996987E37F);
            assert(pack.z_GET() == 1.3758137E38F);
            assert(pack.roll_GET() == -1.8056281E38F);
            assert(pack.usec_GET() == 1896124418099328909L);
            assert(pack.yaw_GET() == -2.5714039E38F);
            assert(pack.y_GET() == 5.352083E37F);
        });
        VISION_POSITION_ESTIMATE p102 = new VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.roll_SET(-1.8056281E38F) ;
        p102.z_SET(1.3758137E38F) ;
        p102.yaw_SET(-2.5714039E38F) ;
        p102.x_SET(5.7996987E37F) ;
        p102.y_SET(5.352083E37F) ;
        p102.pitch_SET(2.3419804E38F) ;
        p102.usec_SET(1896124418099328909L) ;
        TestChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 1.1755459E38F);
            assert(pack.z_GET() == 3.1528647E38F);
            assert(pack.x_GET() == -1.297165E38F);
            assert(pack.usec_GET() == 5200781537176371630L);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.y_SET(1.1755459E38F) ;
        p103.x_SET(-1.297165E38F) ;
        p103.usec_SET(5200781537176371630L) ;
        p103.z_SET(3.1528647E38F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 2.593113E38F);
            assert(pack.roll_GET() == 9.715158E37F);
            assert(pack.x_GET() == 6.387512E36F);
            assert(pack.usec_GET() == 4339162547551282068L);
            assert(pack.yaw_GET() == 3.0630531E38F);
            assert(pack.y_GET() == 2.983506E38F);
            assert(pack.z_GET() == -2.0728059E38F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(4339162547551282068L) ;
        p104.y_SET(2.983506E38F) ;
        p104.z_SET(-2.0728059E38F) ;
        p104.roll_SET(9.715158E37F) ;
        p104.pitch_SET(2.593113E38F) ;
        p104.yaw_SET(3.0630531E38F) ;
        p104.x_SET(6.387512E36F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.abs_pressure_GET() == -2.2686394E38F);
            assert(pack.yacc_GET() == -4.0002208E37F);
            assert(pack.fields_updated_GET() == (char)59656);
            assert(pack.zgyro_GET() == -2.4724995E37F);
            assert(pack.temperature_GET() == -1.3738367E38F);
            assert(pack.pressure_alt_GET() == -1.89154E38F);
            assert(pack.zmag_GET() == -1.4538842E38F);
            assert(pack.ymag_GET() == 1.7011132E38F);
            assert(pack.diff_pressure_GET() == -1.4444344E38F);
            assert(pack.zacc_GET() == -2.4700638E38F);
            assert(pack.time_usec_GET() == 6952026791479097206L);
            assert(pack.xacc_GET() == -1.7457298E37F);
            assert(pack.xmag_GET() == 3.2026316E38F);
            assert(pack.ygyro_GET() == 2.6674985E38F);
            assert(pack.xgyro_GET() == -1.6250464E38F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.zmag_SET(-1.4538842E38F) ;
        p105.yacc_SET(-4.0002208E37F) ;
        p105.abs_pressure_SET(-2.2686394E38F) ;
        p105.zgyro_SET(-2.4724995E37F) ;
        p105.xmag_SET(3.2026316E38F) ;
        p105.ymag_SET(1.7011132E38F) ;
        p105.pressure_alt_SET(-1.89154E38F) ;
        p105.xgyro_SET(-1.6250464E38F) ;
        p105.ygyro_SET(2.6674985E38F) ;
        p105.diff_pressure_SET(-1.4444344E38F) ;
        p105.temperature_SET(-1.3738367E38F) ;
        p105.xacc_SET(-1.7457298E37F) ;
        p105.fields_updated_SET((char)59656) ;
        p105.zacc_SET(-2.4700638E38F) ;
        p105.time_usec_SET(6952026791479097206L) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -15821);
            assert(pack.time_usec_GET() == 4972987337267868293L);
            assert(pack.integrated_xgyro_GET() == -5.126078E37F);
            assert(pack.quality_GET() == (char)212);
            assert(pack.integration_time_us_GET() == 3824292872L);
            assert(pack.integrated_ygyro_GET() == -2.6067722E38F);
            assert(pack.distance_GET() == 1.1644342E38F);
            assert(pack.integrated_zgyro_GET() == 2.2907718E38F);
            assert(pack.integrated_x_GET() == -2.8527203E38F);
            assert(pack.time_delta_distance_us_GET() == 1601238149L);
            assert(pack.sensor_id_GET() == (char)94);
            assert(pack.integrated_y_GET() == 2.5538079E38F);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.distance_SET(1.1644342E38F) ;
        p106.integrated_y_SET(2.5538079E38F) ;
        p106.integrated_x_SET(-2.8527203E38F) ;
        p106.integrated_ygyro_SET(-2.6067722E38F) ;
        p106.quality_SET((char)212) ;
        p106.sensor_id_SET((char)94) ;
        p106.integration_time_us_SET(3824292872L) ;
        p106.time_usec_SET(4972987337267868293L) ;
        p106.integrated_zgyro_SET(2.2907718E38F) ;
        p106.time_delta_distance_us_SET(1601238149L) ;
        p106.integrated_xgyro_SET(-5.126078E37F) ;
        p106.temperature_SET((short) -15821) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.ymag_GET() == -5.900919E37F);
            assert(pack.pressure_alt_GET() == 1.7009387E38F);
            assert(pack.zacc_GET() == -9.508957E37F);
            assert(pack.time_usec_GET() == 396000684659612165L);
            assert(pack.xmag_GET() == 6.1420795E37F);
            assert(pack.ygyro_GET() == 3.1249802E38F);
            assert(pack.abs_pressure_GET() == -1.5509288E38F);
            assert(pack.zgyro_GET() == 1.118981E38F);
            assert(pack.xacc_GET() == -8.3409143E37F);
            assert(pack.xgyro_GET() == 1.956474E38F);
            assert(pack.zmag_GET() == -2.5584588E38F);
            assert(pack.fields_updated_GET() == 2722317559L);
            assert(pack.diff_pressure_GET() == 2.171528E38F);
            assert(pack.yacc_GET() == 2.9567575E38F);
            assert(pack.temperature_GET() == 2.0996208E38F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.fields_updated_SET(2722317559L) ;
        p107.time_usec_SET(396000684659612165L) ;
        p107.zacc_SET(-9.508957E37F) ;
        p107.xmag_SET(6.1420795E37F) ;
        p107.ymag_SET(-5.900919E37F) ;
        p107.zmag_SET(-2.5584588E38F) ;
        p107.pressure_alt_SET(1.7009387E38F) ;
        p107.temperature_SET(2.0996208E38F) ;
        p107.abs_pressure_SET(-1.5509288E38F) ;
        p107.xacc_SET(-8.3409143E37F) ;
        p107.xgyro_SET(1.956474E38F) ;
        p107.zgyro_SET(1.118981E38F) ;
        p107.ygyro_SET(3.1249802E38F) ;
        p107.diff_pressure_SET(2.171528E38F) ;
        p107.yacc_SET(2.9567575E38F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == -2.8423536E38F);
            assert(pack.xgyro_GET() == -1.8995385E38F);
            assert(pack.yaw_GET() == 7.17125E37F);
            assert(pack.std_dev_vert_GET() == 1.903692E38F);
            assert(pack.q1_GET() == 1.4539083E37F);
            assert(pack.vn_GET() == 1.1741703E38F);
            assert(pack.q3_GET() == 1.2947758E38F);
            assert(pack.yacc_GET() == 2.906528E38F);
            assert(pack.q2_GET() == 2.2990525E38F);
            assert(pack.ve_GET() == -2.9449775E38F);
            assert(pack.zgyro_GET() == 3.307665E38F);
            assert(pack.vd_GET() == -1.1676893E38F);
            assert(pack.pitch_GET() == 3.0496325E38F);
            assert(pack.q4_GET() == 8.1219464E37F);
            assert(pack.ygyro_GET() == 1.1223835E38F);
            assert(pack.lat_GET() == 7.379283E37F);
            assert(pack.roll_GET() == 2.4783987E38F);
            assert(pack.zacc_GET() == -6.789738E37F);
            assert(pack.lon_GET() == -7.7413865E37F);
            assert(pack.alt_GET() == 1.6660859E38F);
            assert(pack.std_dev_horz_GET() == -1.0581862E38F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.pitch_SET(3.0496325E38F) ;
        p108.std_dev_horz_SET(-1.0581862E38F) ;
        p108.xgyro_SET(-1.8995385E38F) ;
        p108.q4_SET(8.1219464E37F) ;
        p108.q3_SET(1.2947758E38F) ;
        p108.yacc_SET(2.906528E38F) ;
        p108.vd_SET(-1.1676893E38F) ;
        p108.alt_SET(1.6660859E38F) ;
        p108.lat_SET(7.379283E37F) ;
        p108.std_dev_vert_SET(1.903692E38F) ;
        p108.vn_SET(1.1741703E38F) ;
        p108.yaw_SET(7.17125E37F) ;
        p108.q1_SET(1.4539083E37F) ;
        p108.roll_SET(2.4783987E38F) ;
        p108.ve_SET(-2.9449775E38F) ;
        p108.zacc_SET(-6.789738E37F) ;
        p108.ygyro_SET(1.1223835E38F) ;
        p108.xacc_SET(-2.8423536E38F) ;
        p108.q2_SET(2.2990525E38F) ;
        p108.lon_SET(-7.7413865E37F) ;
        p108.zgyro_SET(3.307665E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.noise_GET() == (char)46);
            assert(pack.rxerrors_GET() == (char)5993);
            assert(pack.remrssi_GET() == (char)58);
            assert(pack.remnoise_GET() == (char)52);
            assert(pack.txbuf_GET() == (char)195);
            assert(pack.rssi_GET() == (char)247);
            assert(pack.fixed__GET() == (char)50501);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.txbuf_SET((char)195) ;
        p109.remrssi_SET((char)58) ;
        p109.noise_SET((char)46) ;
        p109.rxerrors_SET((char)5993) ;
        p109.remnoise_SET((char)52) ;
        p109.rssi_SET((char)247) ;
        p109.fixed__SET((char)50501) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)108);
            assert(pack.target_system_GET() == (char)40);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)198, (char)187, (char)10, (char)38, (char)20, (char)129, (char)13, (char)251, (char)130, (char)221, (char)169, (char)20, (char)39, (char)146, (char)139, (char)93, (char)235, (char)246, (char)31, (char)2, (char)10, (char)179, (char)49, (char)1, (char)134, (char)111, (char)140, (char)107, (char)145, (char)126, (char)193, (char)244, (char)92, (char)103, (char)198, (char)4, (char)146, (char)149, (char)217, (char)128, (char)37, (char)158, (char)165, (char)191, (char)37, (char)39, (char)20, (char)106, (char)177, (char)215, (char)188, (char)51, (char)13, (char)205, (char)224, (char)229, (char)193, (char)39, (char)166, (char)246, (char)36, (char)116, (char)84, (char)151, (char)223, (char)251, (char)88, (char)164, (char)168, (char)63, (char)187, (char)225, (char)208, (char)219, (char)36, (char)103, (char)14, (char)248, (char)49, (char)214, (char)210, (char)174, (char)176, (char)166, (char)107, (char)112, (char)163, (char)12, (char)214, (char)233, (char)171, (char)236, (char)125, (char)54, (char)143, (char)15, (char)229, (char)78, (char)11, (char)109, (char)153, (char)239, (char)50, (char)206, (char)102, (char)43, (char)135, (char)159, (char)52, (char)93, (char)128, (char)1, (char)254, (char)255, (char)182, (char)78, (char)125, (char)81, (char)9, (char)23, (char)93, (char)29, (char)91, (char)168, (char)191, (char)160, (char)75, (char)233, (char)52, (char)102, (char)210, (char)237, (char)140, (char)125, (char)242, (char)245, (char)24, (char)197, (char)166, (char)244, (char)168, (char)162, (char)35, (char)208, (char)69, (char)127, (char)60, (char)173, (char)151, (char)107, (char)225, (char)72, (char)201, (char)236, (char)55, (char)40, (char)26, (char)240, (char)16, (char)255, (char)218, (char)170, (char)147, (char)222, (char)8, (char)202, (char)10, (char)89, (char)19, (char)95, (char)250, (char)93, (char)110, (char)140, (char)75, (char)15, (char)130, (char)58, (char)244, (char)199, (char)187, (char)204, (char)15, (char)59, (char)239, (char)0, (char)27, (char)27, (char)235, (char)160, (char)187, (char)232, (char)21, (char)152, (char)245, (char)196, (char)221, (char)238, (char)33, (char)55, (char)218, (char)5, (char)180, (char)101, (char)62, (char)240, (char)133, (char)173, (char)132, (char)242, (char)132, (char)110, (char)229, (char)106, (char)82, (char)161, (char)210, (char)47, (char)206, (char)119, (char)127, (char)136, (char)91, (char)219, (char)171, (char)68, (char)213, (char)3, (char)79, (char)195, (char)5, (char)100, (char)199, (char)90, (char)18, (char)223, (char)210, (char)178, (char)203, (char)47, (char)112, (char)52, (char)253, (char)164, (char)89, (char)203, (char)73, (char)136, (char)151, (char)70, (char)24}));
            assert(pack.target_component_GET() == (char)158);
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_system_SET((char)40) ;
        p110.target_network_SET((char)108) ;
        p110.target_component_SET((char)158) ;
        p110.payload_SET(new char[] {(char)198, (char)187, (char)10, (char)38, (char)20, (char)129, (char)13, (char)251, (char)130, (char)221, (char)169, (char)20, (char)39, (char)146, (char)139, (char)93, (char)235, (char)246, (char)31, (char)2, (char)10, (char)179, (char)49, (char)1, (char)134, (char)111, (char)140, (char)107, (char)145, (char)126, (char)193, (char)244, (char)92, (char)103, (char)198, (char)4, (char)146, (char)149, (char)217, (char)128, (char)37, (char)158, (char)165, (char)191, (char)37, (char)39, (char)20, (char)106, (char)177, (char)215, (char)188, (char)51, (char)13, (char)205, (char)224, (char)229, (char)193, (char)39, (char)166, (char)246, (char)36, (char)116, (char)84, (char)151, (char)223, (char)251, (char)88, (char)164, (char)168, (char)63, (char)187, (char)225, (char)208, (char)219, (char)36, (char)103, (char)14, (char)248, (char)49, (char)214, (char)210, (char)174, (char)176, (char)166, (char)107, (char)112, (char)163, (char)12, (char)214, (char)233, (char)171, (char)236, (char)125, (char)54, (char)143, (char)15, (char)229, (char)78, (char)11, (char)109, (char)153, (char)239, (char)50, (char)206, (char)102, (char)43, (char)135, (char)159, (char)52, (char)93, (char)128, (char)1, (char)254, (char)255, (char)182, (char)78, (char)125, (char)81, (char)9, (char)23, (char)93, (char)29, (char)91, (char)168, (char)191, (char)160, (char)75, (char)233, (char)52, (char)102, (char)210, (char)237, (char)140, (char)125, (char)242, (char)245, (char)24, (char)197, (char)166, (char)244, (char)168, (char)162, (char)35, (char)208, (char)69, (char)127, (char)60, (char)173, (char)151, (char)107, (char)225, (char)72, (char)201, (char)236, (char)55, (char)40, (char)26, (char)240, (char)16, (char)255, (char)218, (char)170, (char)147, (char)222, (char)8, (char)202, (char)10, (char)89, (char)19, (char)95, (char)250, (char)93, (char)110, (char)140, (char)75, (char)15, (char)130, (char)58, (char)244, (char)199, (char)187, (char)204, (char)15, (char)59, (char)239, (char)0, (char)27, (char)27, (char)235, (char)160, (char)187, (char)232, (char)21, (char)152, (char)245, (char)196, (char)221, (char)238, (char)33, (char)55, (char)218, (char)5, (char)180, (char)101, (char)62, (char)240, (char)133, (char)173, (char)132, (char)242, (char)132, (char)110, (char)229, (char)106, (char)82, (char)161, (char)210, (char)47, (char)206, (char)119, (char)127, (char)136, (char)91, (char)219, (char)171, (char)68, (char)213, (char)3, (char)79, (char)195, (char)5, (char)100, (char)199, (char)90, (char)18, (char)223, (char)210, (char)178, (char)203, (char)47, (char)112, (char)52, (char)253, (char)164, (char)89, (char)203, (char)73, (char)136, (char)151, (char)70, (char)24}, 0) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == 2211741560008531731L);
            assert(pack.ts1_GET() == -7500082329033465882L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(2211741560008531731L) ;
        p111.ts1_SET(-7500082329033465882L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4607025671007780708L);
            assert(pack.seq_GET() == 4194029679L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(4194029679L) ;
        p112.time_usec_SET(4607025671007780708L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.epv_GET() == (char)24510);
            assert(pack.vd_GET() == (short) -995);
            assert(pack.vel_GET() == (char)3755);
            assert(pack.time_usec_GET() == 4484122373462615881L);
            assert(pack.eph_GET() == (char)31478);
            assert(pack.ve_GET() == (short) -29179);
            assert(pack.lon_GET() == 711027064);
            assert(pack.vn_GET() == (short) -25438);
            assert(pack.cog_GET() == (char)13373);
            assert(pack.fix_type_GET() == (char)29);
            assert(pack.alt_GET() == 599309588);
            assert(pack.lat_GET() == -2128256130);
            assert(pack.satellites_visible_GET() == (char)17);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.vn_SET((short) -25438) ;
        p113.ve_SET((short) -29179) ;
        p113.vel_SET((char)3755) ;
        p113.lat_SET(-2128256130) ;
        p113.alt_SET(599309588) ;
        p113.satellites_visible_SET((char)17) ;
        p113.eph_SET((char)31478) ;
        p113.epv_SET((char)24510) ;
        p113.vd_SET((short) -995) ;
        p113.lon_SET(711027064) ;
        p113.fix_type_SET((char)29) ;
        p113.time_usec_SET(4484122373462615881L) ;
        p113.cog_SET((char)13373) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 8372706601360185415L);
            assert(pack.integrated_x_GET() == 3.3927989E38F);
            assert(pack.sensor_id_GET() == (char)242);
            assert(pack.integrated_zgyro_GET() == -1.3721649E38F);
            assert(pack.integration_time_us_GET() == 1785296776L);
            assert(pack.time_delta_distance_us_GET() == 2621642819L);
            assert(pack.distance_GET() == -1.4666121E38F);
            assert(pack.integrated_xgyro_GET() == 7.272693E37F);
            assert(pack.quality_GET() == (char)170);
            assert(pack.temperature_GET() == (short)32758);
            assert(pack.integrated_y_GET() == -1.00189076E37F);
            assert(pack.integrated_ygyro_GET() == -1.9698745E38F);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.integrated_ygyro_SET(-1.9698745E38F) ;
        p114.integrated_x_SET(3.3927989E38F) ;
        p114.integrated_zgyro_SET(-1.3721649E38F) ;
        p114.sensor_id_SET((char)242) ;
        p114.temperature_SET((short)32758) ;
        p114.time_usec_SET(8372706601360185415L) ;
        p114.quality_SET((char)170) ;
        p114.integration_time_us_SET(1785296776L) ;
        p114.time_delta_distance_us_SET(2621642819L) ;
        p114.distance_SET(-1.4666121E38F) ;
        p114.integrated_xgyro_SET(7.272693E37F) ;
        p114.integrated_y_SET(-1.00189076E37F) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.pitchspeed_GET() == 2.881973E38F);
            assert(pack.rollspeed_GET() == -3.2486579E38F);
            assert(pack.vx_GET() == (short) -22921);
            assert(pack.yawspeed_GET() == -3.1737785E38F);
            assert(pack.vz_GET() == (short) -4290);
            assert(pack.vy_GET() == (short)18504);
            assert(pack.lat_GET() == -1611474414);
            assert(pack.xacc_GET() == (short)5098);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {1.1324708E38F, 1.1270831E38F, -1.6404607E38F, -2.4838964E38F}));
            assert(pack.ind_airspeed_GET() == (char)47332);
            assert(pack.yacc_GET() == (short)6320);
            assert(pack.zacc_GET() == (short) -19921);
            assert(pack.alt_GET() == 590545531);
            assert(pack.lon_GET() == 429354013);
            assert(pack.time_usec_GET() == 1586198761237696279L);
            assert(pack.true_airspeed_GET() == (char)3583);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.attitude_quaternion_SET(new float[] {1.1324708E38F, 1.1270831E38F, -1.6404607E38F, -2.4838964E38F}, 0) ;
        p115.xacc_SET((short)5098) ;
        p115.true_airspeed_SET((char)3583) ;
        p115.zacc_SET((short) -19921) ;
        p115.pitchspeed_SET(2.881973E38F) ;
        p115.vx_SET((short) -22921) ;
        p115.yawspeed_SET(-3.1737785E38F) ;
        p115.time_usec_SET(1586198761237696279L) ;
        p115.vz_SET((short) -4290) ;
        p115.ind_airspeed_SET((char)47332) ;
        p115.lon_SET(429354013) ;
        p115.vy_SET((short)18504) ;
        p115.alt_SET(590545531) ;
        p115.rollspeed_SET(-3.2486579E38F) ;
        p115.lat_SET(-1611474414) ;
        p115.yacc_SET((short)6320) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short)16916);
            assert(pack.zgyro_GET() == (short)1691);
            assert(pack.xmag_GET() == (short) -23377);
            assert(pack.zacc_GET() == (short)16915);
            assert(pack.zmag_GET() == (short) -25664);
            assert(pack.xgyro_GET() == (short) -32621);
            assert(pack.time_boot_ms_GET() == 4183339632L);
            assert(pack.yacc_GET() == (short) -10557);
            assert(pack.ygyro_GET() == (short)29300);
            assert(pack.ymag_GET() == (short) -8540);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.ymag_SET((short) -8540) ;
        p116.zgyro_SET((short)1691) ;
        p116.zmag_SET((short) -25664) ;
        p116.time_boot_ms_SET(4183339632L) ;
        p116.xacc_SET((short)16916) ;
        p116.ygyro_SET((short)29300) ;
        p116.yacc_SET((short) -10557) ;
        p116.xgyro_SET((short) -32621) ;
        p116.zacc_SET((short)16915) ;
        p116.xmag_SET((short) -23377) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)245);
            assert(pack.end_GET() == (char)18294);
            assert(pack.start_GET() == (char)50541);
            assert(pack.target_system_GET() == (char)211);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_component_SET((char)245) ;
        p117.start_SET((char)50541) ;
        p117.end_SET((char)18294) ;
        p117.target_system_SET((char)211) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.last_log_num_GET() == (char)39105);
            assert(pack.num_logs_GET() == (char)61057);
            assert(pack.time_utc_GET() == 3328615140L);
            assert(pack.size_GET() == 2087861481L);
            assert(pack.id_GET() == (char)35174);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)35174) ;
        p118.last_log_num_SET((char)39105) ;
        p118.size_SET(2087861481L) ;
        p118.num_logs_SET((char)61057) ;
        p118.time_utc_SET(3328615140L) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == 1398617995L);
            assert(pack.ofs_GET() == 3986781972L);
            assert(pack.target_component_GET() == (char)120);
            assert(pack.target_system_GET() == (char)228);
            assert(pack.id_GET() == (char)56408);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.count_SET(1398617995L) ;
        p119.target_system_SET((char)228) ;
        p119.ofs_SET(3986781972L) ;
        p119.id_SET((char)56408) ;
        p119.target_component_SET((char)120) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)213);
            assert(pack.ofs_GET() == 3824162969L);
            assert(pack.id_GET() == (char)13114);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)227, (char)222, (char)60, (char)16, (char)73, (char)186, (char)190, (char)249, (char)141, (char)147, (char)133, (char)222, (char)90, (char)11, (char)49, (char)197, (char)203, (char)96, (char)104, (char)62, (char)34, (char)231, (char)106, (char)160, (char)239, (char)55, (char)241, (char)254, (char)210, (char)92, (char)17, (char)84, (char)231, (char)138, (char)104, (char)117, (char)246, (char)144, (char)162, (char)86, (char)245, (char)128, (char)203, (char)234, (char)73, (char)5, (char)49, (char)180, (char)222, (char)144, (char)67, (char)81, (char)45, (char)207, (char)93, (char)33, (char)210, (char)242, (char)236, (char)117, (char)71, (char)120, (char)18, (char)24, (char)24, (char)14, (char)36, (char)255, (char)68, (char)103, (char)78, (char)26, (char)63, (char)173, (char)144, (char)76, (char)235, (char)45, (char)96, (char)25, (char)69, (char)91, (char)185, (char)16, (char)230, (char)101, (char)40, (char)145, (char)76, (char)232}));
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.data__SET(new char[] {(char)227, (char)222, (char)60, (char)16, (char)73, (char)186, (char)190, (char)249, (char)141, (char)147, (char)133, (char)222, (char)90, (char)11, (char)49, (char)197, (char)203, (char)96, (char)104, (char)62, (char)34, (char)231, (char)106, (char)160, (char)239, (char)55, (char)241, (char)254, (char)210, (char)92, (char)17, (char)84, (char)231, (char)138, (char)104, (char)117, (char)246, (char)144, (char)162, (char)86, (char)245, (char)128, (char)203, (char)234, (char)73, (char)5, (char)49, (char)180, (char)222, (char)144, (char)67, (char)81, (char)45, (char)207, (char)93, (char)33, (char)210, (char)242, (char)236, (char)117, (char)71, (char)120, (char)18, (char)24, (char)24, (char)14, (char)36, (char)255, (char)68, (char)103, (char)78, (char)26, (char)63, (char)173, (char)144, (char)76, (char)235, (char)45, (char)96, (char)25, (char)69, (char)91, (char)185, (char)16, (char)230, (char)101, (char)40, (char)145, (char)76, (char)232}, 0) ;
        p120.ofs_SET(3824162969L) ;
        p120.id_SET((char)13114) ;
        p120.count_SET((char)213) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)39);
            assert(pack.target_system_GET() == (char)221);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_component_SET((char)39) ;
        p121.target_system_SET((char)221) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)174);
            assert(pack.target_system_GET() == (char)101);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)101) ;
        p122.target_component_SET((char)174) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)229);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)123, (char)73, (char)83, (char)230, (char)79, (char)219, (char)104, (char)160, (char)175, (char)66, (char)51, (char)125, (char)73, (char)39, (char)6, (char)215, (char)251, (char)161, (char)144, (char)49, (char)56, (char)250, (char)195, (char)246, (char)133, (char)114, (char)124, (char)237, (char)123, (char)198, (char)193, (char)200, (char)53, (char)151, (char)238, (char)253, (char)229, (char)77, (char)247, (char)202, (char)57, (char)211, (char)66, (char)133, (char)212, (char)33, (char)60, (char)53, (char)124, (char)142, (char)190, (char)76, (char)28, (char)96, (char)252, (char)135, (char)83, (char)182, (char)239, (char)119, (char)237, (char)236, (char)147, (char)187, (char)190, (char)114, (char)176, (char)21, (char)137, (char)90, (char)112, (char)26, (char)155, (char)188, (char)5, (char)20, (char)113, (char)72, (char)18, (char)61, (char)112, (char)78, (char)83, (char)172, (char)209, (char)84, (char)254, (char)179, (char)65, (char)207, (char)39, (char)103, (char)11, (char)119, (char)92, (char)249, (char)156, (char)22, (char)63, (char)59, (char)97, (char)136, (char)2, (char)238, (char)248, (char)140, (char)221, (char)34, (char)197, (char)240}));
            assert(pack.len_GET() == (char)92);
            assert(pack.target_component_GET() == (char)146);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_component_SET((char)146) ;
        p123.data__SET(new char[] {(char)123, (char)73, (char)83, (char)230, (char)79, (char)219, (char)104, (char)160, (char)175, (char)66, (char)51, (char)125, (char)73, (char)39, (char)6, (char)215, (char)251, (char)161, (char)144, (char)49, (char)56, (char)250, (char)195, (char)246, (char)133, (char)114, (char)124, (char)237, (char)123, (char)198, (char)193, (char)200, (char)53, (char)151, (char)238, (char)253, (char)229, (char)77, (char)247, (char)202, (char)57, (char)211, (char)66, (char)133, (char)212, (char)33, (char)60, (char)53, (char)124, (char)142, (char)190, (char)76, (char)28, (char)96, (char)252, (char)135, (char)83, (char)182, (char)239, (char)119, (char)237, (char)236, (char)147, (char)187, (char)190, (char)114, (char)176, (char)21, (char)137, (char)90, (char)112, (char)26, (char)155, (char)188, (char)5, (char)20, (char)113, (char)72, (char)18, (char)61, (char)112, (char)78, (char)83, (char)172, (char)209, (char)84, (char)254, (char)179, (char)65, (char)207, (char)39, (char)103, (char)11, (char)119, (char)92, (char)249, (char)156, (char)22, (char)63, (char)59, (char)97, (char)136, (char)2, (char)238, (char)248, (char)140, (char)221, (char)34, (char)197, (char)240}, 0) ;
        p123.target_system_SET((char)229) ;
        p123.len_SET((char)92) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.eph_GET() == (char)3198);
            assert(pack.vel_GET() == (char)39336);
            assert(pack.lat_GET() == -673190073);
            assert(pack.time_usec_GET() == 3717101887354080685L);
            assert(pack.lon_GET() == -1530109888);
            assert(pack.alt_GET() == -1073025123);
            assert(pack.dgps_numch_GET() == (char)164);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
            assert(pack.epv_GET() == (char)56540);
            assert(pack.cog_GET() == (char)27936);
            assert(pack.satellites_visible_GET() == (char)19);
            assert(pack.dgps_age_GET() == 2167880708L);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.vel_SET((char)39336) ;
        p124.lon_SET(-1530109888) ;
        p124.dgps_age_SET(2167880708L) ;
        p124.eph_SET((char)3198) ;
        p124.satellites_visible_SET((char)19) ;
        p124.time_usec_SET(3717101887354080685L) ;
        p124.dgps_numch_SET((char)164) ;
        p124.lat_SET(-673190073) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX) ;
        p124.cog_SET((char)27936) ;
        p124.epv_SET((char)56540) ;
        p124.alt_SET(-1073025123) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vservo_GET() == (char)14360);
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED));
            assert(pack.Vcc_GET() == (char)49876);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED)) ;
        p125.Vservo_SET((char)14360) ;
        p125.Vcc_SET((char)49876) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.timeout_GET() == (char)46317);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
            assert(pack.baudrate_GET() == 1132739822L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)136, (char)152, (char)83, (char)0, (char)129, (char)37, (char)162, (char)55, (char)32, (char)146, (char)228, (char)134, (char)107, (char)107, (char)208, (char)28, (char)252, (char)180, (char)191, (char)111, (char)222, (char)70, (char)80, (char)72, (char)224, (char)160, (char)197, (char)130, (char)235, (char)85, (char)131, (char)252, (char)21, (char)226, (char)243, (char)37, (char)133, (char)43, (char)29, (char)142, (char)170, (char)78, (char)240, (char)21, (char)185, (char)16, (char)40, (char)164, (char)7, (char)100, (char)222, (char)10, (char)52, (char)255, (char)218, (char)173, (char)31, (char)137, (char)34, (char)217, (char)14, (char)162, (char)61, (char)230, (char)155, (char)124, (char)185, (char)52, (char)240, (char)221}));
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE));
            assert(pack.count_GET() == (char)150);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.timeout_SET((char)46317) ;
        p126.count_SET((char)150) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE)) ;
        p126.baudrate_SET(1132739822L) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1) ;
        p126.data__SET(new char[] {(char)136, (char)152, (char)83, (char)0, (char)129, (char)37, (char)162, (char)55, (char)32, (char)146, (char)228, (char)134, (char)107, (char)107, (char)208, (char)28, (char)252, (char)180, (char)191, (char)111, (char)222, (char)70, (char)80, (char)72, (char)224, (char)160, (char)197, (char)130, (char)235, (char)85, (char)131, (char)252, (char)21, (char)226, (char)243, (char)37, (char)133, (char)43, (char)29, (char)142, (char)170, (char)78, (char)240, (char)21, (char)185, (char)16, (char)40, (char)164, (char)7, (char)100, (char)222, (char)10, (char)52, (char)255, (char)218, (char)173, (char)31, (char)137, (char)34, (char)217, (char)14, (char)162, (char)61, (char)230, (char)155, (char)124, (char)185, (char)52, (char)240, (char)221}, 0) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_c_mm_GET() == -1167347117);
            assert(pack.tow_GET() == 1810749604L);
            assert(pack.rtk_rate_GET() == (char)32);
            assert(pack.time_last_baseline_ms_GET() == 2361648478L);
            assert(pack.nsats_GET() == (char)13);
            assert(pack.baseline_coords_type_GET() == (char)176);
            assert(pack.accuracy_GET() == 412459569L);
            assert(pack.baseline_b_mm_GET() == 903869128);
            assert(pack.baseline_a_mm_GET() == 18893819);
            assert(pack.rtk_receiver_id_GET() == (char)230);
            assert(pack.rtk_health_GET() == (char)21);
            assert(pack.wn_GET() == (char)27453);
            assert(pack.iar_num_hypotheses_GET() == -1975673404);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.wn_SET((char)27453) ;
        p127.rtk_rate_SET((char)32) ;
        p127.nsats_SET((char)13) ;
        p127.baseline_coords_type_SET((char)176) ;
        p127.tow_SET(1810749604L) ;
        p127.time_last_baseline_ms_SET(2361648478L) ;
        p127.baseline_c_mm_SET(-1167347117) ;
        p127.rtk_health_SET((char)21) ;
        p127.rtk_receiver_id_SET((char)230) ;
        p127.baseline_b_mm_SET(903869128) ;
        p127.baseline_a_mm_SET(18893819) ;
        p127.accuracy_SET(412459569L) ;
        p127.iar_num_hypotheses_SET(-1975673404) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.accuracy_GET() == 1577483197L);
            assert(pack.rtk_receiver_id_GET() == (char)143);
            assert(pack.tow_GET() == 3922163191L);
            assert(pack.baseline_a_mm_GET() == -898339599);
            assert(pack.baseline_c_mm_GET() == -723000565);
            assert(pack.baseline_b_mm_GET() == 1769205645);
            assert(pack.baseline_coords_type_GET() == (char)6);
            assert(pack.wn_GET() == (char)65200);
            assert(pack.nsats_GET() == (char)152);
            assert(pack.rtk_rate_GET() == (char)115);
            assert(pack.time_last_baseline_ms_GET() == 2652884222L);
            assert(pack.iar_num_hypotheses_GET() == -2102026665);
            assert(pack.rtk_health_GET() == (char)144);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.iar_num_hypotheses_SET(-2102026665) ;
        p128.baseline_a_mm_SET(-898339599) ;
        p128.baseline_coords_type_SET((char)6) ;
        p128.accuracy_SET(1577483197L) ;
        p128.time_last_baseline_ms_SET(2652884222L) ;
        p128.baseline_c_mm_SET(-723000565) ;
        p128.baseline_b_mm_SET(1769205645) ;
        p128.tow_SET(3922163191L) ;
        p128.wn_SET((char)65200) ;
        p128.rtk_health_SET((char)144) ;
        p128.rtk_receiver_id_SET((char)143) ;
        p128.rtk_rate_SET((char)115) ;
        p128.nsats_SET((char)152) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == (short) -11103);
            assert(pack.xacc_GET() == (short)4804);
            assert(pack.time_boot_ms_GET() == 1229817039L);
            assert(pack.xgyro_GET() == (short) -12);
            assert(pack.xmag_GET() == (short) -21435);
            assert(pack.ymag_GET() == (short)23182);
            assert(pack.zacc_GET() == (short) -916);
            assert(pack.zmag_GET() == (short)9087);
            assert(pack.ygyro_GET() == (short)23344);
            assert(pack.yacc_GET() == (short) -623);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.zacc_SET((short) -916) ;
        p129.ygyro_SET((short)23344) ;
        p129.xacc_SET((short)4804) ;
        p129.yacc_SET((short) -623) ;
        p129.zmag_SET((short)9087) ;
        p129.ymag_SET((short)23182) ;
        p129.time_boot_ms_SET(1229817039L) ;
        p129.zgyro_SET((short) -11103) ;
        p129.xmag_SET((short) -21435) ;
        p129.xgyro_SET((short) -12) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.packets_GET() == (char)17698);
            assert(pack.size_GET() == 906112876L);
            assert(pack.type_GET() == (char)229);
            assert(pack.height_GET() == (char)54034);
            assert(pack.jpg_quality_GET() == (char)221);
            assert(pack.payload_GET() == (char)204);
            assert(pack.width_GET() == (char)46417);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.width_SET((char)46417) ;
        p130.payload_SET((char)204) ;
        p130.type_SET((char)229) ;
        p130.packets_SET((char)17698) ;
        p130.jpg_quality_SET((char)221) ;
        p130.size_SET(906112876L) ;
        p130.height_SET((char)54034) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)144, (char)77, (char)229, (char)248, (char)199, (char)197, (char)134, (char)66, (char)172, (char)10, (char)192, (char)199, (char)241, (char)201, (char)211, (char)153, (char)250, (char)219, (char)223, (char)216, (char)110, (char)77, (char)141, (char)196, (char)160, (char)85, (char)115, (char)210, (char)194, (char)129, (char)185, (char)80, (char)229, (char)78, (char)173, (char)35, (char)62, (char)105, (char)82, (char)217, (char)10, (char)109, (char)62, (char)249, (char)228, (char)228, (char)246, (char)241, (char)152, (char)167, (char)227, (char)112, (char)49, (char)225, (char)118, (char)183, (char)33, (char)232, (char)17, (char)171, (char)131, (char)202, (char)255, (char)166, (char)64, (char)10, (char)179, (char)145, (char)254, (char)252, (char)226, (char)169, (char)217, (char)0, (char)54, (char)111, (char)2, (char)7, (char)95, (char)31, (char)179, (char)226, (char)172, (char)41, (char)238, (char)223, (char)26, (char)211, (char)117, (char)255, (char)19, (char)237, (char)28, (char)246, (char)11, (char)130, (char)227, (char)110, (char)57, (char)209, (char)145, (char)133, (char)95, (char)47, (char)47, (char)254, (char)181, (char)174, (char)222, (char)137, (char)109, (char)20, (char)86, (char)137, (char)65, (char)204, (char)39, (char)233, (char)101, (char)167, (char)17, (char)0, (char)169, (char)120, (char)146, (char)130, (char)193, (char)4, (char)107, (char)160, (char)222, (char)191, (char)172, (char)169, (char)161, (char)138, (char)201, (char)164, (char)150, (char)143, (char)70, (char)187, (char)153, (char)185, (char)238, (char)240, (char)251, (char)126, (char)196, (char)152, (char)56, (char)146, (char)44, (char)176, (char)102, (char)25, (char)250, (char)207, (char)226, (char)192, (char)46, (char)184, (char)18, (char)228, (char)1, (char)138, (char)54, (char)202, (char)209, (char)72, (char)237, (char)54, (char)102, (char)240, (char)192, (char)58, (char)186, (char)93, (char)213, (char)32, (char)66, (char)100, (char)1, (char)233, (char)194, (char)201, (char)20, (char)32, (char)193, (char)94, (char)184, (char)30, (char)225, (char)153, (char)55, (char)109, (char)244, (char)210, (char)99, (char)90, (char)243, (char)225, (char)95, (char)123, (char)11, (char)168, (char)153, (char)200, (char)55, (char)12, (char)215, (char)97, (char)212, (char)154, (char)139, (char)111, (char)124, (char)31, (char)222, (char)83, (char)169, (char)14, (char)221, (char)47, (char)110, (char)134, (char)24, (char)89, (char)20, (char)11, (char)147, (char)118, (char)245, (char)80, (char)93, (char)229, (char)86, (char)191, (char)217, (char)23, (char)7, (char)148, (char)208, (char)1, (char)241, (char)33, (char)209, (char)230, (char)166, (char)172, (char)188, (char)179, (char)221}));
            assert(pack.seqnr_GET() == (char)9865);
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)9865) ;
        p131.data__SET(new char[] {(char)144, (char)77, (char)229, (char)248, (char)199, (char)197, (char)134, (char)66, (char)172, (char)10, (char)192, (char)199, (char)241, (char)201, (char)211, (char)153, (char)250, (char)219, (char)223, (char)216, (char)110, (char)77, (char)141, (char)196, (char)160, (char)85, (char)115, (char)210, (char)194, (char)129, (char)185, (char)80, (char)229, (char)78, (char)173, (char)35, (char)62, (char)105, (char)82, (char)217, (char)10, (char)109, (char)62, (char)249, (char)228, (char)228, (char)246, (char)241, (char)152, (char)167, (char)227, (char)112, (char)49, (char)225, (char)118, (char)183, (char)33, (char)232, (char)17, (char)171, (char)131, (char)202, (char)255, (char)166, (char)64, (char)10, (char)179, (char)145, (char)254, (char)252, (char)226, (char)169, (char)217, (char)0, (char)54, (char)111, (char)2, (char)7, (char)95, (char)31, (char)179, (char)226, (char)172, (char)41, (char)238, (char)223, (char)26, (char)211, (char)117, (char)255, (char)19, (char)237, (char)28, (char)246, (char)11, (char)130, (char)227, (char)110, (char)57, (char)209, (char)145, (char)133, (char)95, (char)47, (char)47, (char)254, (char)181, (char)174, (char)222, (char)137, (char)109, (char)20, (char)86, (char)137, (char)65, (char)204, (char)39, (char)233, (char)101, (char)167, (char)17, (char)0, (char)169, (char)120, (char)146, (char)130, (char)193, (char)4, (char)107, (char)160, (char)222, (char)191, (char)172, (char)169, (char)161, (char)138, (char)201, (char)164, (char)150, (char)143, (char)70, (char)187, (char)153, (char)185, (char)238, (char)240, (char)251, (char)126, (char)196, (char)152, (char)56, (char)146, (char)44, (char)176, (char)102, (char)25, (char)250, (char)207, (char)226, (char)192, (char)46, (char)184, (char)18, (char)228, (char)1, (char)138, (char)54, (char)202, (char)209, (char)72, (char)237, (char)54, (char)102, (char)240, (char)192, (char)58, (char)186, (char)93, (char)213, (char)32, (char)66, (char)100, (char)1, (char)233, (char)194, (char)201, (char)20, (char)32, (char)193, (char)94, (char)184, (char)30, (char)225, (char)153, (char)55, (char)109, (char)244, (char)210, (char)99, (char)90, (char)243, (char)225, (char)95, (char)123, (char)11, (char)168, (char)153, (char)200, (char)55, (char)12, (char)215, (char)97, (char)212, (char)154, (char)139, (char)111, (char)124, (char)31, (char)222, (char)83, (char)169, (char)14, (char)221, (char)47, (char)110, (char)134, (char)24, (char)89, (char)20, (char)11, (char)147, (char)118, (char)245, (char)80, (char)93, (char)229, (char)86, (char)191, (char)217, (char)23, (char)7, (char)148, (char)208, (char)1, (char)241, (char)33, (char)209, (char)230, (char)166, (char)172, (char)188, (char)179, (char)221}, 0) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.min_distance_GET() == (char)13940);
            assert(pack.max_distance_GET() == (char)53495);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
            assert(pack.current_distance_GET() == (char)52440);
            assert(pack.id_GET() == (char)124);
            assert(pack.covariance_GET() == (char)35);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_270);
            assert(pack.time_boot_ms_GET() == 458388481L);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.max_distance_SET((char)53495) ;
        p132.covariance_SET((char)35) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_270) ;
        p132.time_boot_ms_SET(458388481L) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED) ;
        p132.current_distance_SET((char)52440) ;
        p132.min_distance_SET((char)13940) ;
        p132.id_SET((char)124) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1258879442);
            assert(pack.lon_GET() == 245037367);
            assert(pack.mask_GET() == 4101397724907971077L);
            assert(pack.grid_spacing_GET() == (char)51024);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.grid_spacing_SET((char)51024) ;
        p133.mask_SET(4101397724907971077L) ;
        p133.lon_SET(245037367) ;
        p133.lat_SET(1258879442) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -1027214440);
            assert(pack.gridbit_GET() == (char)17);
            assert(pack.lat_GET() == -1258778554);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short)12128, (short) -7496, (short)4471, (short)11114, (short) -5410, (short)15594, (short)18636, (short)19861, (short)14334, (short)3031, (short)24460, (short)32512, (short)28819, (short) -23664, (short)12698, (short) -8878}));
            assert(pack.grid_spacing_GET() == (char)30481);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lon_SET(-1027214440) ;
        p134.gridbit_SET((char)17) ;
        p134.data__SET(new short[] {(short)12128, (short) -7496, (short)4471, (short)11114, (short) -5410, (short)15594, (short)18636, (short)19861, (short)14334, (short)3031, (short)24460, (short)32512, (short)28819, (short) -23664, (short)12698, (short) -8878}, 0) ;
        p134.grid_spacing_SET((char)30481) ;
        p134.lat_SET(-1258778554) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1305790819);
            assert(pack.lon_GET() == 825086792);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lon_SET(825086792) ;
        p135.lat_SET(-1305790819) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 1282937189);
            assert(pack.loaded_GET() == (char)1801);
            assert(pack.pending_GET() == (char)8512);
            assert(pack.lat_GET() == 13640450);
            assert(pack.terrain_height_GET() == 4.9692324E37F);
            assert(pack.spacing_GET() == (char)38482);
            assert(pack.current_height_GET() == 5.3302284E37F);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.spacing_SET((char)38482) ;
        p136.lat_SET(13640450) ;
        p136.loaded_SET((char)1801) ;
        p136.pending_SET((char)8512) ;
        p136.current_height_SET(5.3302284E37F) ;
        p136.terrain_height_SET(4.9692324E37F) ;
        p136.lon_SET(1282937189) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -18964);
            assert(pack.press_abs_GET() == 1.3857064E38F);
            assert(pack.time_boot_ms_GET() == 886982267L);
            assert(pack.press_diff_GET() == 2.4848335E38F);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.temperature_SET((short) -18964) ;
        p137.time_boot_ms_SET(886982267L) ;
        p137.press_diff_SET(2.4848335E38F) ;
        p137.press_abs_SET(1.3857064E38F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3954308316492549162L);
            assert(pack.z_GET() == -6.0797324E37F);
            assert(pack.y_GET() == -1.9650406E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.4599787E38F, 2.790351E38F, -4.9738923E37F, 7.3990747E37F}));
            assert(pack.x_GET() == 1.2820853E38F);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.z_SET(-6.0797324E37F) ;
        p138.time_usec_SET(3954308316492549162L) ;
        p138.q_SET(new float[] {-1.4599787E38F, 2.790351E38F, -4.9738923E37F, 7.3990747E37F}, 0) ;
        p138.y_SET(-1.9650406E38F) ;
        p138.x_SET(1.2820853E38F) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.3070245E38F, 2.7944658E38F, 1.7389396E38F, 2.0915898E38F, 3.079017E38F, -1.0582775E38F, 2.3124523E38F, 2.878285E38F}));
            assert(pack.group_mlx_GET() == (char)201);
            assert(pack.time_usec_GET() == 957185572906986011L);
            assert(pack.target_system_GET() == (char)147);
            assert(pack.target_component_GET() == (char)235);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.controls_SET(new float[] {2.3070245E38F, 2.7944658E38F, 1.7389396E38F, 2.0915898E38F, 3.079017E38F, -1.0582775E38F, 2.3124523E38F, 2.878285E38F}, 0) ;
        p139.target_system_SET((char)147) ;
        p139.target_component_SET((char)235) ;
        p139.time_usec_SET(957185572906986011L) ;
        p139.group_mlx_SET((char)201) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7779293317375561261L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.9234378E38F, -2.134746E38F, -2.119455E38F, 2.1086027E38F, -1.120411E38F, 2.0796037E38F, -2.0489957E37F, 4.5811853E37F}));
            assert(pack.group_mlx_GET() == (char)207);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(7779293317375561261L) ;
        p140.controls_SET(new float[] {2.9234378E38F, -2.134746E38F, -2.119455E38F, 2.1086027E38F, -1.120411E38F, 2.0796037E38F, -2.0489957E37F, 4.5811853E37F}, 0) ;
        p140.group_mlx_SET((char)207) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_amsl_GET() == -2.839912E38F);
            assert(pack.altitude_monotonic_GET() == -4.8988532E36F);
            assert(pack.bottom_clearance_GET() == -1.7096116E38F);
            assert(pack.time_usec_GET() == 4712719969938222770L);
            assert(pack.altitude_terrain_GET() == 1.3410468E38F);
            assert(pack.altitude_local_GET() == 1.7786463E37F);
            assert(pack.altitude_relative_GET() == 2.5712603E38F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.bottom_clearance_SET(-1.7096116E38F) ;
        p141.altitude_amsl_SET(-2.839912E38F) ;
        p141.altitude_local_SET(1.7786463E37F) ;
        p141.altitude_relative_SET(2.5712603E38F) ;
        p141.altitude_terrain_SET(1.3410468E38F) ;
        p141.time_usec_SET(4712719969938222770L) ;
        p141.altitude_monotonic_SET(-4.8988532E36F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)20, (char)77, (char)93, (char)93, (char)26, (char)33, (char)10, (char)180, (char)133, (char)15, (char)232, (char)80, (char)174, (char)37, (char)109, (char)245, (char)206, (char)183, (char)88, (char)186, (char)130, (char)27, (char)3, (char)120, (char)107, (char)38, (char)209, (char)97, (char)240, (char)251, (char)45, (char)142, (char)171, (char)247, (char)198, (char)75, (char)12, (char)44, (char)115, (char)27, (char)80, (char)80, (char)177, (char)252, (char)156, (char)106, (char)100, (char)250, (char)243, (char)180, (char)3, (char)89, (char)247, (char)59, (char)244, (char)244, (char)94, (char)52, (char)253, (char)58, (char)60, (char)57, (char)158, (char)122, (char)95, (char)170, (char)170, (char)176, (char)201, (char)133, (char)248, (char)182, (char)16, (char)96, (char)53, (char)173, (char)228, (char)7, (char)87, (char)36, (char)59, (char)247, (char)57, (char)71, (char)41, (char)129, (char)50, (char)180, (char)102, (char)90, (char)249, (char)180, (char)14, (char)24, (char)109, (char)80, (char)107, (char)242, (char)12, (char)233, (char)170, (char)3, (char)201, (char)132, (char)62, (char)236, (char)134, (char)17, (char)4, (char)0, (char)252, (char)28, (char)29, (char)64, (char)237, (char)84, (char)129, (char)103, (char)238, (char)221}));
            assert(pack.uri_type_GET() == (char)124);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)125, (char)96, (char)190, (char)156, (char)156, (char)60, (char)160, (char)48, (char)143, (char)15, (char)238, (char)153, (char)199, (char)90, (char)156, (char)138, (char)58, (char)245, (char)6, (char)159, (char)37, (char)143, (char)40, (char)88, (char)93, (char)79, (char)228, (char)62, (char)143, (char)132, (char)48, (char)60, (char)40, (char)197, (char)86, (char)104, (char)101, (char)56, (char)146, (char)195, (char)88, (char)54, (char)183, (char)193, (char)88, (char)35, (char)43, (char)38, (char)223, (char)187, (char)192, (char)154, (char)51, (char)23, (char)90, (char)131, (char)25, (char)58, (char)205, (char)160, (char)227, (char)237, (char)125, (char)234, (char)172, (char)94, (char)176, (char)4, (char)167, (char)219, (char)127, (char)200, (char)146, (char)102, (char)102, (char)50, (char)17, (char)98, (char)94, (char)4, (char)15, (char)58, (char)156, (char)65, (char)103, (char)242, (char)53, (char)89, (char)195, (char)79, (char)205, (char)213, (char)143, (char)32, (char)17, (char)161, (char)183, (char)124, (char)252, (char)213, (char)72, (char)243, (char)93, (char)108, (char)99, (char)66, (char)68, (char)22, (char)195, (char)20, (char)236, (char)194, (char)93, (char)170, (char)177, (char)164, (char)169, (char)213, (char)213, (char)200}));
            assert(pack.request_id_GET() == (char)54);
            assert(pack.transfer_type_GET() == (char)75);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.storage_SET(new char[] {(char)125, (char)96, (char)190, (char)156, (char)156, (char)60, (char)160, (char)48, (char)143, (char)15, (char)238, (char)153, (char)199, (char)90, (char)156, (char)138, (char)58, (char)245, (char)6, (char)159, (char)37, (char)143, (char)40, (char)88, (char)93, (char)79, (char)228, (char)62, (char)143, (char)132, (char)48, (char)60, (char)40, (char)197, (char)86, (char)104, (char)101, (char)56, (char)146, (char)195, (char)88, (char)54, (char)183, (char)193, (char)88, (char)35, (char)43, (char)38, (char)223, (char)187, (char)192, (char)154, (char)51, (char)23, (char)90, (char)131, (char)25, (char)58, (char)205, (char)160, (char)227, (char)237, (char)125, (char)234, (char)172, (char)94, (char)176, (char)4, (char)167, (char)219, (char)127, (char)200, (char)146, (char)102, (char)102, (char)50, (char)17, (char)98, (char)94, (char)4, (char)15, (char)58, (char)156, (char)65, (char)103, (char)242, (char)53, (char)89, (char)195, (char)79, (char)205, (char)213, (char)143, (char)32, (char)17, (char)161, (char)183, (char)124, (char)252, (char)213, (char)72, (char)243, (char)93, (char)108, (char)99, (char)66, (char)68, (char)22, (char)195, (char)20, (char)236, (char)194, (char)93, (char)170, (char)177, (char)164, (char)169, (char)213, (char)213, (char)200}, 0) ;
        p142.uri_type_SET((char)124) ;
        p142.transfer_type_SET((char)75) ;
        p142.request_id_SET((char)54) ;
        p142.uri_SET(new char[] {(char)20, (char)77, (char)93, (char)93, (char)26, (char)33, (char)10, (char)180, (char)133, (char)15, (char)232, (char)80, (char)174, (char)37, (char)109, (char)245, (char)206, (char)183, (char)88, (char)186, (char)130, (char)27, (char)3, (char)120, (char)107, (char)38, (char)209, (char)97, (char)240, (char)251, (char)45, (char)142, (char)171, (char)247, (char)198, (char)75, (char)12, (char)44, (char)115, (char)27, (char)80, (char)80, (char)177, (char)252, (char)156, (char)106, (char)100, (char)250, (char)243, (char)180, (char)3, (char)89, (char)247, (char)59, (char)244, (char)244, (char)94, (char)52, (char)253, (char)58, (char)60, (char)57, (char)158, (char)122, (char)95, (char)170, (char)170, (char)176, (char)201, (char)133, (char)248, (char)182, (char)16, (char)96, (char)53, (char)173, (char)228, (char)7, (char)87, (char)36, (char)59, (char)247, (char)57, (char)71, (char)41, (char)129, (char)50, (char)180, (char)102, (char)90, (char)249, (char)180, (char)14, (char)24, (char)109, (char)80, (char)107, (char)242, (char)12, (char)233, (char)170, (char)3, (char)201, (char)132, (char)62, (char)236, (char)134, (char)17, (char)4, (char)0, (char)252, (char)28, (char)29, (char)64, (char)237, (char)84, (char)129, (char)103, (char)238, (char)221}, 0) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == -1.4612037E38F);
            assert(pack.time_boot_ms_GET() == 882266561L);
            assert(pack.press_abs_GET() == 9.746248E37F);
            assert(pack.temperature_GET() == (short)16822);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.press_abs_SET(9.746248E37F) ;
        p143.temperature_SET((short)16822) ;
        p143.time_boot_ms_SET(882266561L) ;
        p143.press_diff_SET(-1.4612037E38F) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.custom_state_GET() == 4806779083342651431L);
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {2.595285E38F, -3.0391966E38F, 2.7644506E37F, 3.2545522E38F}));
            assert(Arrays.equals(pack.rates_GET(),  new float[] {4.5567045E37F, -2.9470094E38F, -2.124914E38F}));
            assert(Arrays.equals(pack.vel_GET(),  new float[] {-2.9288564E38F, 7.637736E37F, -5.2826043E37F}));
            assert(pack.alt_GET() == 1.3511073E38F);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-7.517672E37F, -2.3610524E38F, -3.2324823E38F}));
            assert(pack.lat_GET() == -1953450353);
            assert(pack.lon_GET() == 78823798);
            assert(pack.est_capabilities_GET() == (char)131);
            assert(pack.timestamp_GET() == 4108543605632613961L);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-1.6117064E38F, 3.441399E37F, 1.9580636E38F}));
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.lat_SET(-1953450353) ;
        p144.position_cov_SET(new float[] {-7.517672E37F, -2.3610524E38F, -3.2324823E38F}, 0) ;
        p144.acc_SET(new float[] {-1.6117064E38F, 3.441399E37F, 1.9580636E38F}, 0) ;
        p144.alt_SET(1.3511073E38F) ;
        p144.attitude_q_SET(new float[] {2.595285E38F, -3.0391966E38F, 2.7644506E37F, 3.2545522E38F}, 0) ;
        p144.est_capabilities_SET((char)131) ;
        p144.vel_SET(new float[] {-2.9288564E38F, 7.637736E37F, -5.2826043E37F}, 0) ;
        p144.timestamp_SET(4108543605632613961L) ;
        p144.custom_state_SET(4806779083342651431L) ;
        p144.rates_SET(new float[] {4.5567045E37F, -2.9470094E38F, -2.124914E38F}, 0) ;
        p144.lon_SET(78823798) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.roll_rate_GET() == -2.6143789E38F);
            assert(pack.z_acc_GET() == 2.3896595E38F);
            assert(pack.z_pos_GET() == 3.5944266E37F);
            assert(pack.y_pos_GET() == -1.614836E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {-3.382344E38F, -2.5996378E38F, -2.3769035E38F}));
            assert(pack.yaw_rate_GET() == 1.1333856E37F);
            assert(pack.x_acc_GET() == -1.9172931E38F);
            assert(pack.x_vel_GET() == 6.1855376E37F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {2.9438167E38F, 2.979182E35F, -2.6489634E38F}));
            assert(pack.x_pos_GET() == 1.3778808E38F);
            assert(pack.time_usec_GET() == 7819353917162080778L);
            assert(pack.z_vel_GET() == -2.8482892E38F);
            assert(pack.y_acc_GET() == -3.027292E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.6182582E38F, -3.3736492E38F, -2.441519E38F, -1.0932959E38F}));
            assert(pack.y_vel_GET() == 1.6488607E38F);
            assert(pack.pitch_rate_GET() == 1.3566659E38F);
            assert(pack.airspeed_GET() == -7.928821E37F);
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.pos_variance_SET(new float[] {2.9438167E38F, 2.979182E35F, -2.6489634E38F}, 0) ;
        p146.y_pos_SET(-1.614836E38F) ;
        p146.q_SET(new float[] {1.6182582E38F, -3.3736492E38F, -2.441519E38F, -1.0932959E38F}, 0) ;
        p146.x_vel_SET(6.1855376E37F) ;
        p146.y_acc_SET(-3.027292E38F) ;
        p146.x_acc_SET(-1.9172931E38F) ;
        p146.roll_rate_SET(-2.6143789E38F) ;
        p146.pitch_rate_SET(1.3566659E38F) ;
        p146.z_pos_SET(3.5944266E37F) ;
        p146.y_vel_SET(1.6488607E38F) ;
        p146.yaw_rate_SET(1.1333856E37F) ;
        p146.x_pos_SET(1.3778808E38F) ;
        p146.z_vel_SET(-2.8482892E38F) ;
        p146.vel_variance_SET(new float[] {-3.382344E38F, -2.5996378E38F, -2.3769035E38F}, 0) ;
        p146.time_usec_SET(7819353917162080778L) ;
        p146.airspeed_SET(-7.928821E37F) ;
        p146.z_acc_SET(2.3896595E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.current_consumed_GET() == 1815412150);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
            assert(pack.id_GET() == (char)99);
            assert(pack.temperature_GET() == (short)22572);
            assert(pack.energy_consumed_GET() == 944935878);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
            assert(pack.battery_remaining_GET() == (byte)48);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)39932, (char)17410, (char)31810, (char)25876, (char)60105, (char)48420, (char)1549, (char)17295, (char)30120, (char)62390}));
            assert(pack.current_battery_GET() == (short) -32682);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.current_battery_SET((short) -32682) ;
        p147.id_SET((char)99) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE) ;
        p147.temperature_SET((short)22572) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD) ;
        p147.current_consumed_SET(1815412150) ;
        p147.energy_consumed_SET(944935878) ;
        p147.voltages_SET(new char[] {(char)39932, (char)17410, (char)31810, (char)25876, (char)60105, (char)48420, (char)1549, (char)17295, (char)30120, (char)62390}, 0) ;
        p147.battery_remaining_SET((byte)48) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.flight_sw_version_GET() == 2515023399L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)123, (char)57, (char)16, (char)171, (char)227, (char)2, (char)85, (char)131, (char)82, (char)150, (char)85, (char)198, (char)183, (char)142, (char)248, (char)236, (char)209, (char)101}));
            assert(pack.board_version_GET() == 3999515404L);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)25, (char)32, (char)158, (char)93, (char)32, (char)162, (char)45, (char)236}));
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)147, (char)211, (char)194, (char)238, (char)227, (char)238, (char)192, (char)82}));
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT));
            assert(pack.uid_GET() == 1785143342346091690L);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)224, (char)152, (char)92, (char)250, (char)140, (char)126, (char)129, (char)221}));
            assert(pack.product_id_GET() == (char)40418);
            assert(pack.os_sw_version_GET() == 3000154548L);
            assert(pack.middleware_sw_version_GET() == 51749347L);
            assert(pack.vendor_id_GET() == (char)59978);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.product_id_SET((char)40418) ;
        p148.uid_SET(1785143342346091690L) ;
        p148.board_version_SET(3999515404L) ;
        p148.uid2_SET(new char[] {(char)123, (char)57, (char)16, (char)171, (char)227, (char)2, (char)85, (char)131, (char)82, (char)150, (char)85, (char)198, (char)183, (char)142, (char)248, (char)236, (char)209, (char)101}, 0, PH) ;
        p148.os_custom_version_SET(new char[] {(char)147, (char)211, (char)194, (char)238, (char)227, (char)238, (char)192, (char)82}, 0) ;
        p148.os_sw_version_SET(3000154548L) ;
        p148.flight_custom_version_SET(new char[] {(char)25, (char)32, (char)158, (char)93, (char)32, (char)162, (char)45, (char)236}, 0) ;
        p148.middleware_sw_version_SET(51749347L) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT)) ;
        p148.flight_sw_version_SET(2515023399L) ;
        p148.middleware_custom_version_SET(new char[] {(char)224, (char)152, (char)92, (char)250, (char)140, (char)126, (char)129, (char)221}, 0) ;
        p148.vendor_id_SET((char)59978) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {-2.0990591E37F, -1.5092053E38F, 9.225097E37F, -1.5529215E38F}));
            assert(pack.target_num_GET() == (char)184);
            assert(pack.position_valid_TRY(ph) == (char)207);
            assert(pack.size_x_GET() == 1.8324557E37F);
            assert(pack.y_TRY(ph) == 2.717918E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
            assert(pack.angle_x_GET() == 1.5538897E38F);
            assert(pack.z_TRY(ph) == 1.8462159E38F);
            assert(pack.x_TRY(ph) == 3.2109616E38F);
            assert(pack.size_y_GET() == 3.0463603E38F);
            assert(pack.angle_y_GET() == 2.6798004E38F);
            assert(pack.distance_GET() == 1.7064536E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.time_usec_GET() == 1797537280151120972L);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.q_SET(new float[] {-2.0990591E37F, -1.5092053E38F, 9.225097E37F, -1.5529215E38F}, 0, PH) ;
        p149.angle_y_SET(2.6798004E38F) ;
        p149.angle_x_SET(1.5538897E38F) ;
        p149.y_SET(2.717918E38F, PH) ;
        p149.position_valid_SET((char)207, PH) ;
        p149.time_usec_SET(1797537280151120972L) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p149.distance_SET(1.7064536E38F) ;
        p149.size_x_SET(1.8324557E37F) ;
        p149.z_SET(1.8462159E38F, PH) ;
        p149.target_num_SET((char)184) ;
        p149.x_SET(3.2109616E38F, PH) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON) ;
        p149.size_y_SET(3.0463603E38F) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_SENSOR_OFFSETS.add((src, ph, pack) ->
        {
            assert(pack.mag_ofs_x_GET() == (short)16472);
            assert(pack.raw_press_GET() == 687852525);
            assert(pack.accel_cal_y_GET() == -9.410135E37F);
            assert(pack.gyro_cal_x_GET() == 3.2102414E38F);
            assert(pack.mag_ofs_z_GET() == (short) -18731);
            assert(pack.accel_cal_z_GET() == 3.0480124E37F);
            assert(pack.mag_ofs_y_GET() == (short)19718);
            assert(pack.gyro_cal_z_GET() == -4.662424E37F);
            assert(pack.raw_temp_GET() == 2100315286);
            assert(pack.accel_cal_x_GET() == 9.106122E37F);
            assert(pack.gyro_cal_y_GET() == 8.992161E37F);
            assert(pack.mag_declination_GET() == -1.2360209E38F);
        });
        GroundControl.SENSOR_OFFSETS p150 = CommunicationChannel.new_SENSOR_OFFSETS();
        PH.setPack(p150);
        p150.gyro_cal_z_SET(-4.662424E37F) ;
        p150.mag_ofs_z_SET((short) -18731) ;
        p150.accel_cal_x_SET(9.106122E37F) ;
        p150.accel_cal_y_SET(-9.410135E37F) ;
        p150.mag_ofs_y_SET((short)19718) ;
        p150.gyro_cal_x_SET(3.2102414E38F) ;
        p150.mag_ofs_x_SET((short)16472) ;
        p150.raw_press_SET(687852525) ;
        p150.mag_declination_SET(-1.2360209E38F) ;
        p150.raw_temp_SET(2100315286) ;
        p150.gyro_cal_y_SET(8.992161E37F) ;
        p150.accel_cal_z_SET(3.0480124E37F) ;
        CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_MAG_OFFSETS.add((src, ph, pack) ->
        {
            assert(pack.mag_ofs_x_GET() == (short) -32762);
            assert(pack.target_component_GET() == (char)44);
            assert(pack.target_system_GET() == (char)141);
            assert(pack.mag_ofs_z_GET() == (short) -18622);
            assert(pack.mag_ofs_y_GET() == (short) -17706);
        });
        GroundControl.SET_MAG_OFFSETS p151 = CommunicationChannel.new_SET_MAG_OFFSETS();
        PH.setPack(p151);
        p151.target_component_SET((char)44) ;
        p151.mag_ofs_x_SET((short) -32762) ;
        p151.mag_ofs_y_SET((short) -17706) ;
        p151.mag_ofs_z_SET((short) -18622) ;
        p151.target_system_SET((char)141) ;
        CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MEMINFO.add((src, ph, pack) ->
        {
            assert(pack.freemem32_TRY(ph) == 4279124649L);
            assert(pack.brkval_GET() == (char)23770);
            assert(pack.freemem_GET() == (char)1581);
        });
        GroundControl.MEMINFO p152 = CommunicationChannel.new_MEMINFO();
        PH.setPack(p152);
        p152.freemem_SET((char)1581) ;
        p152.brkval_SET((char)23770) ;
        p152.freemem32_SET(4279124649L, PH) ;
        CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AP_ADC.add((src, ph, pack) ->
        {
            assert(pack.adc3_GET() == (char)54821);
            assert(pack.adc5_GET() == (char)17308);
            assert(pack.adc2_GET() == (char)59034);
            assert(pack.adc1_GET() == (char)27513);
            assert(pack.adc4_GET() == (char)63368);
            assert(pack.adc6_GET() == (char)64635);
        });
        GroundControl.AP_ADC p153 = CommunicationChannel.new_AP_ADC();
        PH.setPack(p153);
        p153.adc6_SET((char)64635) ;
        p153.adc5_SET((char)17308) ;
        p153.adc3_SET((char)54821) ;
        p153.adc4_SET((char)63368) ;
        p153.adc1_SET((char)27513) ;
        p153.adc2_SET((char)59034) ;
        CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DIGICAM_CONFIGURE.add((src, ph, pack) ->
        {
            assert(pack.shutter_speed_GET() == (char)2108);
            assert(pack.extra_value_GET() == 1.1834419E38F);
            assert(pack.target_system_GET() == (char)196);
            assert(pack.engine_cut_off_GET() == (char)133);
            assert(pack.extra_param_GET() == (char)14);
            assert(pack.command_id_GET() == (char)180);
            assert(pack.mode_GET() == (char)76);
            assert(pack.exposure_type_GET() == (char)139);
            assert(pack.aperture_GET() == (char)147);
            assert(pack.iso_GET() == (char)240);
            assert(pack.target_component_GET() == (char)6);
        });
        GroundControl.DIGICAM_CONFIGURE p154 = CommunicationChannel.new_DIGICAM_CONFIGURE();
        PH.setPack(p154);
        p154.target_system_SET((char)196) ;
        p154.command_id_SET((char)180) ;
        p154.target_component_SET((char)6) ;
        p154.extra_param_SET((char)14) ;
        p154.exposure_type_SET((char)139) ;
        p154.mode_SET((char)76) ;
        p154.iso_SET((char)240) ;
        p154.shutter_speed_SET((char)2108) ;
        p154.aperture_SET((char)147) ;
        p154.extra_value_SET(1.1834419E38F) ;
        p154.engine_cut_off_SET((char)133) ;
        CommunicationChannel.instance.send(p154);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DIGICAM_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.session_GET() == (char)243);
            assert(pack.shot_GET() == (char)250);
            assert(pack.target_component_GET() == (char)113);
            assert(pack.extra_value_GET() == -2.533705E38F);
            assert(pack.zoom_pos_GET() == (char)48);
            assert(pack.zoom_step_GET() == (byte) - 20);
            assert(pack.extra_param_GET() == (char)180);
            assert(pack.target_system_GET() == (char)4);
            assert(pack.command_id_GET() == (char)12);
            assert(pack.focus_lock_GET() == (char)33);
        });
        GroundControl.DIGICAM_CONTROL p155 = CommunicationChannel.new_DIGICAM_CONTROL();
        PH.setPack(p155);
        p155.zoom_step_SET((byte) - 20) ;
        p155.zoom_pos_SET((char)48) ;
        p155.focus_lock_SET((char)33) ;
        p155.target_component_SET((char)113) ;
        p155.target_system_SET((char)4) ;
        p155.session_SET((char)243) ;
        p155.extra_value_SET(-2.533705E38F) ;
        p155.command_id_SET((char)12) ;
        p155.shot_SET((char)250) ;
        p155.extra_param_SET((char)180) ;
        CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_CONFIGURE.add((src, ph, pack) ->
        {
            assert(pack.stab_roll_GET() == (char)66);
            assert(pack.stab_pitch_GET() == (char)85);
            assert(pack.stab_yaw_GET() == (char)44);
            assert(pack.mount_mode_GET() == MAV_MOUNT_MODE.MAV_MOUNT_MODE_RETRACT);
            assert(pack.target_component_GET() == (char)132);
            assert(pack.target_system_GET() == (char)234);
        });
        GroundControl.MOUNT_CONFIGURE p156 = CommunicationChannel.new_MOUNT_CONFIGURE();
        PH.setPack(p156);
        p156.target_system_SET((char)234) ;
        p156.mount_mode_SET(MAV_MOUNT_MODE.MAV_MOUNT_MODE_RETRACT) ;
        p156.stab_yaw_SET((char)44) ;
        p156.stab_pitch_SET((char)85) ;
        p156.stab_roll_SET((char)66) ;
        p156.target_component_SET((char)132) ;
        CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.input_b_GET() == -1596890449);
            assert(pack.save_position_GET() == (char)211);
            assert(pack.input_c_GET() == -2146019171);
            assert(pack.target_component_GET() == (char)87);
            assert(pack.target_system_GET() == (char)192);
            assert(pack.input_a_GET() == -1372082091);
        });
        GroundControl.MOUNT_CONTROL p157 = CommunicationChannel.new_MOUNT_CONTROL();
        PH.setPack(p157);
        p157.input_b_SET(-1596890449) ;
        p157.input_c_SET(-2146019171) ;
        p157.target_system_SET((char)192) ;
        p157.target_component_SET((char)87) ;
        p157.save_position_SET((char)211) ;
        p157.input_a_SET(-1372082091) ;
        CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_STATUS.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)141);
            assert(pack.target_component_GET() == (char)117);
            assert(pack.pointing_a_GET() == -1845408956);
            assert(pack.pointing_b_GET() == -1306162268);
            assert(pack.pointing_c_GET() == -1335292505);
        });
        GroundControl.MOUNT_STATUS p158 = CommunicationChannel.new_MOUNT_STATUS();
        PH.setPack(p158);
        p158.target_system_SET((char)141) ;
        p158.pointing_b_SET(-1306162268) ;
        p158.target_component_SET((char)117) ;
        p158.pointing_a_SET(-1845408956) ;
        p158.pointing_c_SET(-1335292505) ;
        CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FENCE_POINT.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1.7413653E38F);
            assert(pack.count_GET() == (char)168);
            assert(pack.target_component_GET() == (char)223);
            assert(pack.lng_GET() == 2.8221713E38F);
            assert(pack.idx_GET() == (char)79);
            assert(pack.target_system_GET() == (char)84);
        });
        GroundControl.FENCE_POINT p160 = CommunicationChannel.new_FENCE_POINT();
        PH.setPack(p160);
        p160.lat_SET(1.7413653E38F) ;
        p160.target_component_SET((char)223) ;
        p160.count_SET((char)168) ;
        p160.target_system_SET((char)84) ;
        p160.lng_SET(2.8221713E38F) ;
        p160.idx_SET((char)79) ;
        CommunicationChannel.instance.send(p160);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FENCE_FETCH_POINT.add((src, ph, pack) ->
        {
            assert(pack.idx_GET() == (char)244);
            assert(pack.target_component_GET() == (char)93);
            assert(pack.target_system_GET() == (char)14);
        });
        GroundControl.FENCE_FETCH_POINT p161 = CommunicationChannel.new_FENCE_FETCH_POINT();
        PH.setPack(p161);
        p161.target_component_SET((char)93) ;
        p161.idx_SET((char)244) ;
        p161.target_system_SET((char)14) ;
        CommunicationChannel.instance.send(p161);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FENCE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.breach_count_GET() == (char)22365);
            assert(pack.breach_time_GET() == 554772966L);
            assert(pack.breach_type_GET() == FENCE_BREACH.FENCE_BREACH_MINALT);
            assert(pack.breach_status_GET() == (char)73);
        });
        GroundControl.FENCE_STATUS p162 = CommunicationChannel.new_FENCE_STATUS();
        PH.setPack(p162);
        p162.breach_time_SET(554772966L) ;
        p162.breach_type_SET(FENCE_BREACH.FENCE_BREACH_MINALT) ;
        p162.breach_status_SET((char)73) ;
        p162.breach_count_SET((char)22365) ;
        CommunicationChannel.instance.send(p162);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AHRS.add((src, ph, pack) ->
        {
            assert(pack.error_rp_GET() == -6.690878E37F);
            assert(pack.omegaIz_GET() == 1.1233614E38F);
            assert(pack.omegaIx_GET() == -2.343385E38F);
            assert(pack.renorm_val_GET() == 3.0312895E38F);
            assert(pack.omegaIy_GET() == -3.0038163E38F);
            assert(pack.accel_weight_GET() == -2.9056746E38F);
            assert(pack.error_yaw_GET() == 1.4915253E38F);
        });
        GroundControl.AHRS p163 = CommunicationChannel.new_AHRS();
        PH.setPack(p163);
        p163.omegaIx_SET(-2.343385E38F) ;
        p163.renorm_val_SET(3.0312895E38F) ;
        p163.accel_weight_SET(-2.9056746E38F) ;
        p163.omegaIy_SET(-3.0038163E38F) ;
        p163.error_yaw_SET(1.4915253E38F) ;
        p163.omegaIz_SET(1.1233614E38F) ;
        p163.error_rp_SET(-6.690878E37F) ;
        CommunicationChannel.instance.send(p163);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SIMSTATE.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == -2.6662749E38F);
            assert(pack.pitch_GET() == 2.405486E38F);
            assert(pack.xacc_GET() == -5.8734765E37F);
            assert(pack.yacc_GET() == -2.5756529E38F);
            assert(pack.yaw_GET() == 2.2211453E38F);
            assert(pack.xgyro_GET() == -2.840327E38F);
            assert(pack.ygyro_GET() == -1.5380356E38F);
            assert(pack.lat_GET() == 275878766);
            assert(pack.lng_GET() == 724577674);
            assert(pack.roll_GET() == -8.92111E37F);
            assert(pack.zgyro_GET() == -1.891521E37F);
        });
        GroundControl.SIMSTATE p164 = CommunicationChannel.new_SIMSTATE();
        PH.setPack(p164);
        p164.ygyro_SET(-1.5380356E38F) ;
        p164.roll_SET(-8.92111E37F) ;
        p164.lat_SET(275878766) ;
        p164.pitch_SET(2.405486E38F) ;
        p164.zacc_SET(-2.6662749E38F) ;
        p164.zgyro_SET(-1.891521E37F) ;
        p164.xacc_SET(-5.8734765E37F) ;
        p164.yacc_SET(-2.5756529E38F) ;
        p164.yaw_SET(2.2211453E38F) ;
        p164.xgyro_SET(-2.840327E38F) ;
        p164.lng_SET(724577674) ;
        CommunicationChannel.instance.send(p164);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HWSTATUS.add((src, ph, pack) ->
        {
            assert(pack.Vcc_GET() == (char)19424);
            assert(pack.I2Cerr_GET() == (char)58);
        });
        GroundControl.HWSTATUS p165 = CommunicationChannel.new_HWSTATUS();
        PH.setPack(p165);
        p165.I2Cerr_SET((char)58) ;
        p165.Vcc_SET((char)19424) ;
        CommunicationChannel.instance.send(p165);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RADIO.add((src, ph, pack) ->
        {
            assert(pack.fixed__GET() == (char)41080);
            assert(pack.rxerrors_GET() == (char)29902);
            assert(pack.remnoise_GET() == (char)52);
            assert(pack.rssi_GET() == (char)95);
            assert(pack.noise_GET() == (char)48);
            assert(pack.remrssi_GET() == (char)241);
            assert(pack.txbuf_GET() == (char)121);
        });
        GroundControl.RADIO p166 = CommunicationChannel.new_RADIO();
        PH.setPack(p166);
        p166.remnoise_SET((char)52) ;
        p166.fixed__SET((char)41080) ;
        p166.txbuf_SET((char)121) ;
        p166.rssi_SET((char)95) ;
        p166.remrssi_SET((char)241) ;
        p166.noise_SET((char)48) ;
        p166.rxerrors_SET((char)29902) ;
        CommunicationChannel.instance.send(p166);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LIMITS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.mods_triggered_GET() == (LIMIT_MODULE.LIMIT_GEOFENCE));
            assert(pack.mods_required_GET() == (LIMIT_MODULE.LIMIT_GPSLOCK |
                                                LIMIT_MODULE.LIMIT_ALTITUDE));
            assert(pack.last_trigger_GET() == 3296787959L);
            assert(pack.mods_enabled_GET() == (LIMIT_MODULE.LIMIT_ALTITUDE |
                                               LIMIT_MODULE.LIMIT_GPSLOCK));
            assert(pack.last_clear_GET() == 1277574252L);
            assert(pack.last_action_GET() == 2209534933L);
            assert(pack.breach_count_GET() == (char)22957);
            assert(pack.last_recovery_GET() == 4195514904L);
            assert(pack.limits_state_GET() == LIMITS_STATE.LIMITS_TRIGGERED);
        });
        GroundControl.LIMITS_STATUS p167 = CommunicationChannel.new_LIMITS_STATUS();
        PH.setPack(p167);
        p167.mods_enabled_SET((LIMIT_MODULE.LIMIT_ALTITUDE |
                               LIMIT_MODULE.LIMIT_GPSLOCK)) ;
        p167.mods_triggered_SET((LIMIT_MODULE.LIMIT_GEOFENCE)) ;
        p167.limits_state_SET(LIMITS_STATE.LIMITS_TRIGGERED) ;
        p167.breach_count_SET((char)22957) ;
        p167.last_action_SET(2209534933L) ;
        p167.last_trigger_SET(3296787959L) ;
        p167.last_clear_SET(1277574252L) ;
        p167.mods_required_SET((LIMIT_MODULE.LIMIT_GPSLOCK |
                                LIMIT_MODULE.LIMIT_ALTITUDE)) ;
        p167.last_recovery_SET(4195514904L) ;
        CommunicationChannel.instance.send(p167);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIND.add((src, ph, pack) ->
        {
            assert(pack.direction_GET() == 3.384098E38F);
            assert(pack.speed_GET() == -1.6965057E38F);
            assert(pack.speed_z_GET() == -1.2269567E38F);
        });
        GroundControl.WIND p168 = CommunicationChannel.new_WIND();
        PH.setPack(p168);
        p168.speed_SET(-1.6965057E38F) ;
        p168.direction_SET(3.384098E38F) ;
        p168.speed_z_SET(-1.2269567E38F) ;
        CommunicationChannel.instance.send(p168);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DATA16.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)86);
            assert(pack.len_GET() == (char)249);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)207, (char)116, (char)47, (char)193, (char)130, (char)66, (char)62, (char)196, (char)21, (char)39, (char)253, (char)121, (char)236, (char)166, (char)249, (char)232}));
        });
        GroundControl.DATA16 p169 = CommunicationChannel.new_DATA16();
        PH.setPack(p169);
        p169.type_SET((char)86) ;
        p169.len_SET((char)249) ;
        p169.data__SET(new char[] {(char)207, (char)116, (char)47, (char)193, (char)130, (char)66, (char)62, (char)196, (char)21, (char)39, (char)253, (char)121, (char)236, (char)166, (char)249, (char)232}, 0) ;
        CommunicationChannel.instance.send(p169);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DATA32.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)105);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)73, (char)114, (char)13, (char)90, (char)52, (char)99, (char)98, (char)251, (char)124, (char)171, (char)216, (char)21, (char)58, (char)49, (char)18, (char)41, (char)200, (char)109, (char)29, (char)90, (char)126, (char)190, (char)91, (char)48, (char)186, (char)20, (char)199, (char)2, (char)153, (char)213, (char)78, (char)110}));
            assert(pack.type_GET() == (char)90);
        });
        GroundControl.DATA32 p170 = CommunicationChannel.new_DATA32();
        PH.setPack(p170);
        p170.data__SET(new char[] {(char)73, (char)114, (char)13, (char)90, (char)52, (char)99, (char)98, (char)251, (char)124, (char)171, (char)216, (char)21, (char)58, (char)49, (char)18, (char)41, (char)200, (char)109, (char)29, (char)90, (char)126, (char)190, (char)91, (char)48, (char)186, (char)20, (char)199, (char)2, (char)153, (char)213, (char)78, (char)110}, 0) ;
        p170.type_SET((char)90) ;
        p170.len_SET((char)105) ;
        CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DATA64.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)81);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)200, (char)208, (char)18, (char)135, (char)166, (char)99, (char)53, (char)178, (char)233, (char)52, (char)53, (char)15, (char)185, (char)24, (char)49, (char)222, (char)247, (char)205, (char)11, (char)231, (char)215, (char)203, (char)141, (char)42, (char)148, (char)40, (char)44, (char)122, (char)195, (char)190, (char)80, (char)115, (char)235, (char)121, (char)89, (char)20, (char)97, (char)152, (char)121, (char)251, (char)43, (char)36, (char)20, (char)233, (char)158, (char)125, (char)87, (char)185, (char)175, (char)8, (char)184, (char)246, (char)65, (char)94, (char)131, (char)35, (char)224, (char)103, (char)78, (char)26, (char)11, (char)39, (char)75, (char)153}));
            assert(pack.len_GET() == (char)144);
        });
        GroundControl.DATA64 p171 = CommunicationChannel.new_DATA64();
        PH.setPack(p171);
        p171.data__SET(new char[] {(char)200, (char)208, (char)18, (char)135, (char)166, (char)99, (char)53, (char)178, (char)233, (char)52, (char)53, (char)15, (char)185, (char)24, (char)49, (char)222, (char)247, (char)205, (char)11, (char)231, (char)215, (char)203, (char)141, (char)42, (char)148, (char)40, (char)44, (char)122, (char)195, (char)190, (char)80, (char)115, (char)235, (char)121, (char)89, (char)20, (char)97, (char)152, (char)121, (char)251, (char)43, (char)36, (char)20, (char)233, (char)158, (char)125, (char)87, (char)185, (char)175, (char)8, (char)184, (char)246, (char)65, (char)94, (char)131, (char)35, (char)224, (char)103, (char)78, (char)26, (char)11, (char)39, (char)75, (char)153}, 0) ;
        p171.type_SET((char)81) ;
        p171.len_SET((char)144) ;
        CommunicationChannel.instance.send(p171);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DATA96.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)147);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)43, (char)86, (char)218, (char)56, (char)55, (char)163, (char)144, (char)101, (char)42, (char)234, (char)107, (char)1, (char)228, (char)195, (char)227, (char)240, (char)156, (char)67, (char)212, (char)59, (char)191, (char)29, (char)141, (char)95, (char)79, (char)180, (char)80, (char)10, (char)25, (char)146, (char)152, (char)218, (char)91, (char)128, (char)230, (char)229, (char)18, (char)221, (char)188, (char)192, (char)214, (char)118, (char)205, (char)254, (char)167, (char)252, (char)254, (char)237, (char)33, (char)152, (char)13, (char)194, (char)12, (char)91, (char)62, (char)187, (char)49, (char)10, (char)136, (char)214, (char)76, (char)99, (char)70, (char)154, (char)10, (char)190, (char)116, (char)0, (char)171, (char)202, (char)41, (char)58, (char)70, (char)199, (char)149, (char)239, (char)183, (char)102, (char)46, (char)180, (char)43, (char)251, (char)44, (char)169, (char)89, (char)218, (char)147, (char)201, (char)222, (char)210, (char)192, (char)161, (char)4, (char)58, (char)123, (char)247}));
            assert(pack.type_GET() == (char)11);
        });
        GroundControl.DATA96 p172 = CommunicationChannel.new_DATA96();
        PH.setPack(p172);
        p172.len_SET((char)147) ;
        p172.type_SET((char)11) ;
        p172.data__SET(new char[] {(char)43, (char)86, (char)218, (char)56, (char)55, (char)163, (char)144, (char)101, (char)42, (char)234, (char)107, (char)1, (char)228, (char)195, (char)227, (char)240, (char)156, (char)67, (char)212, (char)59, (char)191, (char)29, (char)141, (char)95, (char)79, (char)180, (char)80, (char)10, (char)25, (char)146, (char)152, (char)218, (char)91, (char)128, (char)230, (char)229, (char)18, (char)221, (char)188, (char)192, (char)214, (char)118, (char)205, (char)254, (char)167, (char)252, (char)254, (char)237, (char)33, (char)152, (char)13, (char)194, (char)12, (char)91, (char)62, (char)187, (char)49, (char)10, (char)136, (char)214, (char)76, (char)99, (char)70, (char)154, (char)10, (char)190, (char)116, (char)0, (char)171, (char)202, (char)41, (char)58, (char)70, (char)199, (char)149, (char)239, (char)183, (char)102, (char)46, (char)180, (char)43, (char)251, (char)44, (char)169, (char)89, (char)218, (char)147, (char)201, (char)222, (char)210, (char)192, (char)161, (char)4, (char)58, (char)123, (char)247}, 0) ;
        CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RANGEFINDER.add((src, ph, pack) ->
        {
            assert(pack.distance_GET() == -8.1028414E37F);
            assert(pack.voltage_GET() == -1.9331688E38F);
        });
        GroundControl.RANGEFINDER p173 = CommunicationChannel.new_RANGEFINDER();
        PH.setPack(p173);
        p173.voltage_SET(-1.9331688E38F) ;
        p173.distance_SET(-8.1028414E37F) ;
        CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AIRSPEED_AUTOCAL.add((src, ph, pack) ->
        {
            assert(pack.state_x_GET() == 1.3852586E38F);
            assert(pack.vy_GET() == -2.3378528E38F);
            assert(pack.vx_GET() == 1.467863E38F);
            assert(pack.vz_GET() == 5.213391E37F);
            assert(pack.state_z_GET() == 2.9402503E36F);
            assert(pack.Pcz_GET() == 1.7999347E38F);
            assert(pack.ratio_GET() == 1.4496975E38F);
            assert(pack.state_y_GET() == -2.881788E38F);
            assert(pack.diff_pressure_GET() == -1.803884E38F);
            assert(pack.EAS2TAS_GET() == -1.4690277E38F);
            assert(pack.Pby_GET() == 2.3831077E38F);
            assert(pack.Pax_GET() == 1.282414E38F);
        });
        GroundControl.AIRSPEED_AUTOCAL p174 = CommunicationChannel.new_AIRSPEED_AUTOCAL();
        PH.setPack(p174);
        p174.ratio_SET(1.4496975E38F) ;
        p174.EAS2TAS_SET(-1.4690277E38F) ;
        p174.Pby_SET(2.3831077E38F) ;
        p174.vy_SET(-2.3378528E38F) ;
        p174.diff_pressure_SET(-1.803884E38F) ;
        p174.Pax_SET(1.282414E38F) ;
        p174.vz_SET(5.213391E37F) ;
        p174.vx_SET(1.467863E38F) ;
        p174.state_z_SET(2.9402503E36F) ;
        p174.state_y_SET(-2.881788E38F) ;
        p174.Pcz_SET(1.7999347E38F) ;
        p174.state_x_SET(1.3852586E38F) ;
        CommunicationChannel.instance.send(p174);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RALLY_POINT.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == (short)31239);
            assert(pack.count_GET() == (char)120);
            assert(pack.target_system_GET() == (char)149);
            assert(pack.flags_GET() == RALLY_FLAGS.LAND_IMMEDIATELY);
            assert(pack.lng_GET() == 165261833);
            assert(pack.land_dir_GET() == (char)37798);
            assert(pack.target_component_GET() == (char)155);
            assert(pack.lat_GET() == -276689431);
            assert(pack.break_alt_GET() == (short)19199);
            assert(pack.idx_GET() == (char)192);
        });
        GroundControl.RALLY_POINT p175 = CommunicationChannel.new_RALLY_POINT();
        PH.setPack(p175);
        p175.lng_SET(165261833) ;
        p175.break_alt_SET((short)19199) ;
        p175.target_system_SET((char)149) ;
        p175.alt_SET((short)31239) ;
        p175.lat_SET(-276689431) ;
        p175.target_component_SET((char)155) ;
        p175.flags_SET(RALLY_FLAGS.LAND_IMMEDIATELY) ;
        p175.land_dir_SET((char)37798) ;
        p175.count_SET((char)120) ;
        p175.idx_SET((char)192) ;
        CommunicationChannel.instance.send(p175);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RALLY_FETCH_POINT.add((src, ph, pack) ->
        {
            assert(pack.idx_GET() == (char)170);
            assert(pack.target_system_GET() == (char)206);
            assert(pack.target_component_GET() == (char)205);
        });
        GroundControl.RALLY_FETCH_POINT p176 = CommunicationChannel.new_RALLY_FETCH_POINT();
        PH.setPack(p176);
        p176.target_system_SET((char)206) ;
        p176.idx_SET((char)170) ;
        p176.target_component_SET((char)205) ;
        CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_COMPASSMOT_STATUS.add((src, ph, pack) ->
        {
            assert(pack.CompensationY_GET() == 1.0586899E38F);
            assert(pack.CompensationX_GET() == 1.0976974E38F);
            assert(pack.interference_GET() == (char)58163);
            assert(pack.current_GET() == 3.1918083E38F);
            assert(pack.throttle_GET() == (char)41002);
            assert(pack.CompensationZ_GET() == 2.7030702E38F);
        });
        GroundControl.COMPASSMOT_STATUS p177 = CommunicationChannel.new_COMPASSMOT_STATUS();
        PH.setPack(p177);
        p177.CompensationY_SET(1.0586899E38F) ;
        p177.CompensationX_SET(1.0976974E38F) ;
        p177.throttle_SET((char)41002) ;
        p177.interference_SET((char)58163) ;
        p177.CompensationZ_SET(2.7030702E38F) ;
        p177.current_SET(3.1918083E38F) ;
        CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AHRS2.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 1.3454069E37F);
            assert(pack.altitude_GET() == 1.5817753E38F);
            assert(pack.pitch_GET() == -1.9108837E38F);
            assert(pack.roll_GET() == -1.0642655E38F);
            assert(pack.lat_GET() == 1259308271);
            assert(pack.lng_GET() == 249976180);
        });
        GroundControl.AHRS2 p178 = CommunicationChannel.new_AHRS2();
        PH.setPack(p178);
        p178.altitude_SET(1.5817753E38F) ;
        p178.lat_SET(1259308271) ;
        p178.lng_SET(249976180) ;
        p178.roll_SET(-1.0642655E38F) ;
        p178.yaw_SET(1.3454069E37F) ;
        p178.pitch_SET(-1.9108837E38F) ;
        CommunicationChannel.instance.send(p178);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_STATUS.add((src, ph, pack) ->
        {
            assert(pack.p4_GET() == 8.4885535E37F);
            assert(pack.p1_GET() == -1.1600016E38F);
            assert(pack.target_system_GET() == (char)95);
            assert(pack.img_idx_GET() == (char)361);
            assert(pack.event_id_GET() == CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_LOWBATT);
            assert(pack.p2_GET() == 1.2817163E38F);
            assert(pack.p3_GET() == -1.4889177E37F);
            assert(pack.time_usec_GET() == 4375288937822413545L);
            assert(pack.cam_idx_GET() == (char)90);
        });
        GroundControl.CAMERA_STATUS p179 = CommunicationChannel.new_CAMERA_STATUS();
        PH.setPack(p179);
        p179.img_idx_SET((char)361) ;
        p179.cam_idx_SET((char)90) ;
        p179.target_system_SET((char)95) ;
        p179.time_usec_SET(4375288937822413545L) ;
        p179.p4_SET(8.4885535E37F) ;
        p179.p2_SET(1.2817163E38F) ;
        p179.p3_SET(-1.4889177E37F) ;
        p179.p1_SET(-1.1600016E38F) ;
        p179.event_id_SET(CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_LOWBATT) ;
        CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_FEEDBACK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1587698914);
            assert(pack.cam_idx_GET() == (char)189);
            assert(pack.alt_rel_GET() == -3.006645E38F);
            assert(pack.alt_msl_GET() == -1.8036723E38F);
            assert(pack.yaw_GET() == 1.6240474E38F);
            assert(pack.img_idx_GET() == (char)24069);
            assert(pack.pitch_GET() == -1.4149638E38F);
            assert(pack.roll_GET() == 1.9399814E38F);
            assert(pack.target_system_GET() == (char)67);
            assert(pack.time_usec_GET() == 8992336965740057970L);
            assert(pack.foc_len_GET() == -2.5834295E38F);
            assert(pack.flags_GET() == CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_BADEXPOSURE);
            assert(pack.lng_GET() == -675407924);
        });
        GroundControl.CAMERA_FEEDBACK p180 = CommunicationChannel.new_CAMERA_FEEDBACK();
        PH.setPack(p180);
        p180.cam_idx_SET((char)189) ;
        p180.alt_rel_SET(-3.006645E38F) ;
        p180.alt_msl_SET(-1.8036723E38F) ;
        p180.target_system_SET((char)67) ;
        p180.flags_SET(CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_BADEXPOSURE) ;
        p180.yaw_SET(1.6240474E38F) ;
        p180.foc_len_SET(-2.5834295E38F) ;
        p180.img_idx_SET((char)24069) ;
        p180.lng_SET(-675407924) ;
        p180.lat_SET(1587698914) ;
        p180.time_usec_SET(8992336965740057970L) ;
        p180.roll_SET(1.9399814E38F) ;
        p180.pitch_SET(-1.4149638E38F) ;
        CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BATTERY2.add((src, ph, pack) ->
        {
            assert(pack.voltage_GET() == (char)53484);
            assert(pack.current_battery_GET() == (short) -15903);
        });
        GroundControl.BATTERY2 p181 = CommunicationChannel.new_BATTERY2();
        PH.setPack(p181);
        p181.voltage_SET((char)53484) ;
        p181.current_battery_SET((short) -15903) ;
        CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AHRS3.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == 3.2238628E38F);
            assert(pack.v1_GET() == -2.36573E38F);
            assert(pack.pitch_GET() == -2.0461517E38F);
            assert(pack.roll_GET() == 3.0105213E38F);
            assert(pack.lng_GET() == 1486646120);
            assert(pack.v3_GET() == 2.1485463E38F);
            assert(pack.yaw_GET() == -8.331617E37F);
            assert(pack.lat_GET() == -1111196461);
            assert(pack.v2_GET() == 2.2737297E38F);
            assert(pack.v4_GET() == -2.4910436E38F);
        });
        GroundControl.AHRS3 p182 = CommunicationChannel.new_AHRS3();
        PH.setPack(p182);
        p182.lat_SET(-1111196461) ;
        p182.lng_SET(1486646120) ;
        p182.yaw_SET(-8.331617E37F) ;
        p182.altitude_SET(3.2238628E38F) ;
        p182.pitch_SET(-2.0461517E38F) ;
        p182.v1_SET(-2.36573E38F) ;
        p182.v4_SET(-2.4910436E38F) ;
        p182.v2_SET(2.2737297E38F) ;
        p182.roll_SET(3.0105213E38F) ;
        p182.v3_SET(2.1485463E38F) ;
        CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)188);
            assert(pack.target_component_GET() == (char)166);
        });
        GroundControl.AUTOPILOT_VERSION_REQUEST p183 = CommunicationChannel.new_AUTOPILOT_VERSION_REQUEST();
        PH.setPack(p183);
        p183.target_system_SET((char)188) ;
        p183.target_component_SET((char)166) ;
        CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_REMOTE_LOG_DATA_BLOCK.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)144, (char)133, (char)201, (char)113, (char)217, (char)172, (char)227, (char)32, (char)75, (char)73, (char)22, (char)34, (char)43, (char)252, (char)54, (char)157, (char)27, (char)186, (char)152, (char)118, (char)156, (char)137, (char)237, (char)175, (char)26, (char)175, (char)134, (char)231, (char)224, (char)35, (char)187, (char)189, (char)32, (char)7, (char)167, (char)183, (char)226, (char)112, (char)153, (char)5, (char)17, (char)146, (char)188, (char)95, (char)71, (char)239, (char)81, (char)78, (char)134, (char)22, (char)210, (char)55, (char)201, (char)56, (char)3, (char)126, (char)208, (char)46, (char)111, (char)5, (char)116, (char)172, (char)68, (char)243, (char)1, (char)141, (char)217, (char)23, (char)6, (char)92, (char)48, (char)92, (char)102, (char)219, (char)106, (char)254, (char)64, (char)203, (char)228, (char)250, (char)180, (char)1, (char)89, (char)56, (char)175, (char)80, (char)64, (char)85, (char)109, (char)138, (char)47, (char)36, (char)125, (char)141, (char)222, (char)67, (char)127, (char)229, (char)233, (char)162, (char)18, (char)175, (char)3, (char)143, (char)185, (char)106, (char)72, (char)27, (char)185, (char)184, (char)96, (char)245, (char)157, (char)115, (char)63, (char)157, (char)188, (char)31, (char)28, (char)229, (char)64, (char)35, (char)128, (char)213, (char)109, (char)212, (char)123, (char)54, (char)253, (char)51, (char)209, (char)195, (char)108, (char)166, (char)43, (char)46, (char)186, (char)160, (char)34, (char)48, (char)54, (char)65, (char)105, (char)124, (char)165, (char)131, (char)18, (char)12, (char)193, (char)38, (char)105, (char)4, (char)118, (char)149, (char)77, (char)162, (char)102, (char)134, (char)114, (char)46, (char)59, (char)3, (char)129, (char)239, (char)95, (char)39, (char)93, (char)206, (char)133, (char)196, (char)32, (char)152, (char)241, (char)27, (char)113, (char)190, (char)235, (char)23, (char)123, (char)129, (char)184, (char)179, (char)70, (char)201, (char)70, (char)201, (char)245, (char)176, (char)190, (char)2, (char)164, (char)71, (char)193, (char)209, (char)191, (char)164, (char)52, (char)32, (char)4, (char)15}));
            assert(pack.target_system_GET() == (char)43);
            assert(pack.seqno_GET() == MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_START);
            assert(pack.target_component_GET() == (char)186);
        });
        GroundControl.REMOTE_LOG_DATA_BLOCK p184 = CommunicationChannel.new_REMOTE_LOG_DATA_BLOCK();
        PH.setPack(p184);
        p184.data__SET(new char[] {(char)144, (char)133, (char)201, (char)113, (char)217, (char)172, (char)227, (char)32, (char)75, (char)73, (char)22, (char)34, (char)43, (char)252, (char)54, (char)157, (char)27, (char)186, (char)152, (char)118, (char)156, (char)137, (char)237, (char)175, (char)26, (char)175, (char)134, (char)231, (char)224, (char)35, (char)187, (char)189, (char)32, (char)7, (char)167, (char)183, (char)226, (char)112, (char)153, (char)5, (char)17, (char)146, (char)188, (char)95, (char)71, (char)239, (char)81, (char)78, (char)134, (char)22, (char)210, (char)55, (char)201, (char)56, (char)3, (char)126, (char)208, (char)46, (char)111, (char)5, (char)116, (char)172, (char)68, (char)243, (char)1, (char)141, (char)217, (char)23, (char)6, (char)92, (char)48, (char)92, (char)102, (char)219, (char)106, (char)254, (char)64, (char)203, (char)228, (char)250, (char)180, (char)1, (char)89, (char)56, (char)175, (char)80, (char)64, (char)85, (char)109, (char)138, (char)47, (char)36, (char)125, (char)141, (char)222, (char)67, (char)127, (char)229, (char)233, (char)162, (char)18, (char)175, (char)3, (char)143, (char)185, (char)106, (char)72, (char)27, (char)185, (char)184, (char)96, (char)245, (char)157, (char)115, (char)63, (char)157, (char)188, (char)31, (char)28, (char)229, (char)64, (char)35, (char)128, (char)213, (char)109, (char)212, (char)123, (char)54, (char)253, (char)51, (char)209, (char)195, (char)108, (char)166, (char)43, (char)46, (char)186, (char)160, (char)34, (char)48, (char)54, (char)65, (char)105, (char)124, (char)165, (char)131, (char)18, (char)12, (char)193, (char)38, (char)105, (char)4, (char)118, (char)149, (char)77, (char)162, (char)102, (char)134, (char)114, (char)46, (char)59, (char)3, (char)129, (char)239, (char)95, (char)39, (char)93, (char)206, (char)133, (char)196, (char)32, (char)152, (char)241, (char)27, (char)113, (char)190, (char)235, (char)23, (char)123, (char)129, (char)184, (char)179, (char)70, (char)201, (char)70, (char)201, (char)245, (char)176, (char)190, (char)2, (char)164, (char)71, (char)193, (char)209, (char)191, (char)164, (char)52, (char)32, (char)4, (char)15}, 0) ;
        p184.target_system_SET((char)43) ;
        p184.seqno_SET(MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_START) ;
        p184.target_component_SET((char)186) ;
        CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_REMOTE_LOG_BLOCK_STATUS.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)120);
            assert(pack.seqno_GET() == 1486053319L);
            assert(pack.status_GET() == MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK);
            assert(pack.target_component_GET() == (char)141);
        });
        GroundControl.REMOTE_LOG_BLOCK_STATUS p185 = CommunicationChannel.new_REMOTE_LOG_BLOCK_STATUS();
        PH.setPack(p185);
        p185.target_component_SET((char)141) ;
        p185.target_system_SET((char)120) ;
        p185.seqno_SET(1486053319L) ;
        p185.status_SET(MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK) ;
        CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LED_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)185);
            assert(pack.pattern_GET() == (char)84);
            assert(pack.instance_GET() == (char)252);
            assert(pack.target_component_GET() == (char)82);
            assert(Arrays.equals(pack.custom_bytes_GET(),  new char[] {(char)7, (char)72, (char)121, (char)217, (char)228, (char)51, (char)241, (char)121, (char)159, (char)76, (char)208, (char)216, (char)59, (char)129, (char)112, (char)196, (char)241, (char)225, (char)21, (char)78, (char)173, (char)185, (char)127, (char)160}));
            assert(pack.custom_len_GET() == (char)210);
        });
        GroundControl.LED_CONTROL p186 = CommunicationChannel.new_LED_CONTROL();
        PH.setPack(p186);
        p186.pattern_SET((char)84) ;
        p186.target_system_SET((char)185) ;
        p186.instance_SET((char)252) ;
        p186.custom_len_SET((char)210) ;
        p186.custom_bytes_SET(new char[] {(char)7, (char)72, (char)121, (char)217, (char)228, (char)51, (char)241, (char)121, (char)159, (char)76, (char)208, (char)216, (char)59, (char)129, (char)112, (char)196, (char)241, (char)225, (char)21, (char)78, (char)173, (char)185, (char)127, (char)160}, 0) ;
        p186.target_component_SET((char)82) ;
        CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MAG_CAL_PROGRESS.add((src, ph, pack) ->
        {
            assert(pack.attempt_GET() == (char)91);
            assert(pack.cal_status_GET() == MAG_CAL_STATUS.MAG_CAL_WAITING_TO_START);
            assert(pack.compass_id_GET() == (char)173);
            assert(pack.cal_mask_GET() == (char)40);
            assert(Arrays.equals(pack.completion_mask_GET(),  new char[] {(char)118, (char)175, (char)228, (char)112, (char)109, (char)124, (char)232, (char)227, (char)116, (char)174}));
            assert(pack.direction_x_GET() == 2.1842987E38F);
            assert(pack.direction_z_GET() == -2.8818286E38F);
            assert(pack.direction_y_GET() == -1.2023001E38F);
            assert(pack.completion_pct_GET() == (char)10);
        });
        GroundControl.MAG_CAL_PROGRESS p191 = CommunicationChannel.new_MAG_CAL_PROGRESS();
        PH.setPack(p191);
        p191.compass_id_SET((char)173) ;
        p191.direction_y_SET(-1.2023001E38F) ;
        p191.cal_status_SET(MAG_CAL_STATUS.MAG_CAL_WAITING_TO_START) ;
        p191.cal_mask_SET((char)40) ;
        p191.attempt_SET((char)91) ;
        p191.completion_pct_SET((char)10) ;
        p191.completion_mask_SET(new char[] {(char)118, (char)175, (char)228, (char)112, (char)109, (char)124, (char)232, (char)227, (char)116, (char)174}, 0) ;
        p191.direction_x_SET(2.1842987E38F) ;
        p191.direction_z_SET(-2.8818286E38F) ;
        CommunicationChannel.instance.send(p191);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MAG_CAL_REPORT.add((src, ph, pack) ->
        {
            assert(pack.diag_x_GET() == -2.5788404E38F);
            assert(pack.cal_mask_GET() == (char)189);
            assert(pack.offdiag_y_GET() == -3.391992E38F);
            assert(pack.fitness_GET() == -1.1991615E38F);
            assert(pack.ofs_z_GET() == -4.0665004E37F);
            assert(pack.offdiag_x_GET() == -3.199465E38F);
            assert(pack.ofs_y_GET() == -3.668494E37F);
            assert(pack.diag_y_GET() == -2.6423834E38F);
            assert(pack.offdiag_z_GET() == -1.1554763E38F);
            assert(pack.autosaved_GET() == (char)101);
            assert(pack.ofs_x_GET() == 3.0117595E38F);
            assert(pack.cal_status_GET() == MAG_CAL_STATUS.MAG_CAL_RUNNING_STEP_ONE);
            assert(pack.diag_z_GET() == -1.1234098E37F);
            assert(pack.compass_id_GET() == (char)3);
        });
        GroundControl.MAG_CAL_REPORT p192 = CommunicationChannel.new_MAG_CAL_REPORT();
        PH.setPack(p192);
        p192.diag_z_SET(-1.1234098E37F) ;
        p192.diag_x_SET(-2.5788404E38F) ;
        p192.diag_y_SET(-2.6423834E38F) ;
        p192.ofs_y_SET(-3.668494E37F) ;
        p192.offdiag_x_SET(-3.199465E38F) ;
        p192.ofs_z_SET(-4.0665004E37F) ;
        p192.ofs_x_SET(3.0117595E38F) ;
        p192.cal_status_SET(MAG_CAL_STATUS.MAG_CAL_RUNNING_STEP_ONE) ;
        p192.autosaved_SET((char)101) ;
        p192.offdiag_z_SET(-1.1554763E38F) ;
        p192.fitness_SET(-1.1991615E38F) ;
        p192.cal_mask_SET((char)189) ;
        p192.offdiag_y_SET(-3.391992E38F) ;
        p192.compass_id_SET((char)3) ;
        CommunicationChannel.instance.send(p192);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_EKF_STATUS_REPORT.add((src, ph, pack) ->
        {
            assert(pack.velocity_variance_GET() == 9.760824E37F);
            assert(pack.compass_variance_GET() == 2.329095E38F);
            assert(pack.terrain_alt_variance_GET() == 4.7901083E37F);
            assert(pack.pos_horiz_variance_GET() == 2.3795623E38F);
            assert(pack.flags_GET() == (EKF_STATUS_FLAGS.EKF_POS_HORIZ_REL |
                                        EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_ABS |
                                        EKF_STATUS_FLAGS.EKF_VELOCITY_HORIZ |
                                        EKF_STATUS_FLAGS.EKF_ATTITUDE));
            assert(pack.pos_vert_variance_GET() == -2.506086E38F);
        });
        GroundControl.EKF_STATUS_REPORT p193 = CommunicationChannel.new_EKF_STATUS_REPORT();
        PH.setPack(p193);
        p193.terrain_alt_variance_SET(4.7901083E37F) ;
        p193.flags_SET((EKF_STATUS_FLAGS.EKF_POS_HORIZ_REL |
                        EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_ABS |
                        EKF_STATUS_FLAGS.EKF_VELOCITY_HORIZ |
                        EKF_STATUS_FLAGS.EKF_ATTITUDE)) ;
        p193.pos_horiz_variance_SET(2.3795623E38F) ;
        p193.compass_variance_SET(2.329095E38F) ;
        p193.velocity_variance_SET(9.760824E37F) ;
        p193.pos_vert_variance_SET(-2.506086E38F) ;
        CommunicationChannel.instance.send(p193);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PID_TUNING.add((src, ph, pack) ->
        {
            assert(pack.axis_GET() == PID_TUNING_AXIS.PID_TUNING_STEER);
            assert(pack.I_GET() == 1.7165324E37F);
            assert(pack.desired_GET() == -2.5543616E38F);
            assert(pack.FF_GET() == -2.7820617E38F);
            assert(pack.P_GET() == 9.057169E37F);
            assert(pack.achieved_GET() == -1.1233541E38F);
            assert(pack.D_GET() == -1.5580808E38F);
        });
        GroundControl.PID_TUNING p194 = CommunicationChannel.new_PID_TUNING();
        PH.setPack(p194);
        p194.FF_SET(-2.7820617E38F) ;
        p194.P_SET(9.057169E37F) ;
        p194.D_SET(-1.5580808E38F) ;
        p194.axis_SET(PID_TUNING_AXIS.PID_TUNING_STEER) ;
        p194.I_SET(1.7165324E37F) ;
        p194.achieved_SET(-1.1233541E38F) ;
        p194.desired_SET(-2.5543616E38F) ;
        CommunicationChannel.instance.send(p194);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GIMBAL_REPORT.add((src, ph, pack) ->
        {
            assert(pack.delta_velocity_y_GET() == 2.138062E38F);
            assert(pack.delta_angle_x_GET() == -1.828318E38F);
            assert(pack.joint_az_GET() == -4.328706E37F);
            assert(pack.joint_el_GET() == 1.7543497E38F);
            assert(pack.delta_velocity_x_GET() == 1.6090513E37F);
            assert(pack.target_system_GET() == (char)72);
            assert(pack.joint_roll_GET() == -3.2458273E38F);
            assert(pack.target_component_GET() == (char)103);
            assert(pack.delta_time_GET() == 7.329622E37F);
            assert(pack.delta_angle_z_GET() == 1.223722E38F);
            assert(pack.delta_velocity_z_GET() == 1.7105872E38F);
            assert(pack.delta_angle_y_GET() == 7.936196E37F);
        });
        GroundControl.GIMBAL_REPORT p200 = CommunicationChannel.new_GIMBAL_REPORT();
        PH.setPack(p200);
        p200.joint_el_SET(1.7543497E38F) ;
        p200.joint_az_SET(-4.328706E37F) ;
        p200.delta_angle_z_SET(1.223722E38F) ;
        p200.target_system_SET((char)72) ;
        p200.delta_time_SET(7.329622E37F) ;
        p200.delta_velocity_x_SET(1.6090513E37F) ;
        p200.joint_roll_SET(-3.2458273E38F) ;
        p200.delta_angle_x_SET(-1.828318E38F) ;
        p200.delta_velocity_z_SET(1.7105872E38F) ;
        p200.delta_velocity_y_SET(2.138062E38F) ;
        p200.target_component_SET((char)103) ;
        p200.delta_angle_y_SET(7.936196E37F) ;
        CommunicationChannel.instance.send(p200);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GIMBAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)216);
            assert(pack.demanded_rate_y_GET() == -1.9131064E38F);
            assert(pack.demanded_rate_x_GET() == 2.7013223E38F);
            assert(pack.demanded_rate_z_GET() == -3.978528E36F);
            assert(pack.target_component_GET() == (char)207);
        });
        GroundControl.GIMBAL_CONTROL p201 = CommunicationChannel.new_GIMBAL_CONTROL();
        PH.setPack(p201);
        p201.demanded_rate_z_SET(-3.978528E36F) ;
        p201.demanded_rate_x_SET(2.7013223E38F) ;
        p201.target_component_SET((char)207) ;
        p201.target_system_SET((char)216) ;
        p201.demanded_rate_y_SET(-1.9131064E38F) ;
        CommunicationChannel.instance.send(p201);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GIMBAL_TORQUE_CMD_REPORT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)188);
            assert(pack.target_system_GET() == (char)24);
            assert(pack.el_torque_cmd_GET() == (short)5871);
            assert(pack.rl_torque_cmd_GET() == (short) -8422);
            assert(pack.az_torque_cmd_GET() == (short) -180);
        });
        GroundControl.GIMBAL_TORQUE_CMD_REPORT p214 = CommunicationChannel.new_GIMBAL_TORQUE_CMD_REPORT();
        PH.setPack(p214);
        p214.rl_torque_cmd_SET((short) -8422) ;
        p214.target_component_SET((char)188) ;
        p214.target_system_SET((char)24) ;
        p214.az_torque_cmd_SET((short) -180) ;
        p214.el_torque_cmd_SET((short)5871) ;
        CommunicationChannel.instance.send(p214);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GOPRO_HEARTBEAT.add((src, ph, pack) ->
        {
            assert(pack.capture_mode_GET() == GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_VIDEO);
            assert(pack.flags_GET() == GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING);
            assert(pack.status_GET() == GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_ERROR);
        });
        GroundControl.GOPRO_HEARTBEAT p215 = CommunicationChannel.new_GOPRO_HEARTBEAT();
        PH.setPack(p215);
        p215.status_SET(GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_ERROR) ;
        p215.capture_mode_SET(GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_VIDEO) ;
        p215.flags_SET(GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING) ;
        CommunicationChannel.instance.send(p215);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GOPRO_GET_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.cmd_id_GET() == GOPRO_COMMAND.GOPRO_COMMAND_VIDEO_SETTINGS);
            assert(pack.target_component_GET() == (char)176);
            assert(pack.target_system_GET() == (char)180);
        });
        GroundControl.GOPRO_GET_REQUEST p216 = CommunicationChannel.new_GOPRO_GET_REQUEST();
        PH.setPack(p216);
        p216.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_VIDEO_SETTINGS) ;
        p216.target_component_SET((char)176) ;
        p216.target_system_SET((char)180) ;
        CommunicationChannel.instance.send(p216);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GOPRO_GET_RESPONSE.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.value_GET(),  new char[] {(char)204, (char)22, (char)142, (char)20}));
            assert(pack.cmd_id_GET() == GOPRO_COMMAND.GOPRO_COMMAND_CAPTURE_MODE);
            assert(pack.status_GET() == GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS);
        });
        GroundControl.GOPRO_GET_RESPONSE p217 = CommunicationChannel.new_GOPRO_GET_RESPONSE();
        PH.setPack(p217);
        p217.status_SET(GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS) ;
        p217.value_SET(new char[] {(char)204, (char)22, (char)142, (char)20}, 0) ;
        p217.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_CAPTURE_MODE) ;
        CommunicationChannel.instance.send(p217);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GOPRO_SET_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)116);
            assert(pack.target_component_GET() == (char)248);
            assert(Arrays.equals(pack.value_GET(),  new char[] {(char)204, (char)56, (char)49, (char)9}));
            assert(pack.cmd_id_GET() == GOPRO_COMMAND.GOPRO_COMMAND_VIDEO_SETTINGS);
        });
        GroundControl.GOPRO_SET_REQUEST p218 = CommunicationChannel.new_GOPRO_SET_REQUEST();
        PH.setPack(p218);
        p218.value_SET(new char[] {(char)204, (char)56, (char)49, (char)9}, 0) ;
        p218.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_VIDEO_SETTINGS) ;
        p218.target_component_SET((char)248) ;
        p218.target_system_SET((char)116) ;
        CommunicationChannel.instance.send(p218);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GOPRO_SET_RESPONSE.add((src, ph, pack) ->
        {
            assert(pack.status_GET() == GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED);
            assert(pack.cmd_id_GET() == GOPRO_COMMAND.GOPRO_COMMAND_PHOTO_RESOLUTION);
        });
        GroundControl.GOPRO_SET_RESPONSE p219 = CommunicationChannel.new_GOPRO_SET_RESPONSE();
        PH.setPack(p219);
        p219.status_SET(GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED) ;
        p219.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_PHOTO_RESOLUTION) ;
        CommunicationChannel.instance.send(p219);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RPM.add((src, ph, pack) ->
        {
            assert(pack.rpm2_GET() == -1.0685893E38F);
            assert(pack.rpm1_GET() == -2.2070594E38F);
        });
        GroundControl.RPM p226 = CommunicationChannel.new_RPM();
        PH.setPack(p226);
        p226.rpm2_SET(-1.0685893E38F) ;
        p226.rpm1_SET(-2.2070594E38F) ;
        CommunicationChannel.instance.send(p226);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        CommunicationChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.vel_ratio_GET() == -1.7839233E38F);
            assert(pack.pos_horiz_accuracy_GET() == 1.0468725E38F);
            assert(pack.pos_vert_ratio_GET() == 4.0435486E37F);
            assert(pack.time_usec_GET() == 8902874433169875766L);
            assert(pack.hagl_ratio_GET() == -1.3859644E38F);
            assert(pack.tas_ratio_GET() == -2.2100874E38F);
            assert(pack.pos_vert_accuracy_GET() == -2.2764506E38F);
            assert(pack.mag_ratio_GET() == -3.217783E38F);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL));
            assert(pack.pos_horiz_ratio_GET() == -5.8793934E37F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.hagl_ratio_SET(-1.3859644E38F) ;
        p230.pos_horiz_accuracy_SET(1.0468725E38F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL)) ;
        p230.tas_ratio_SET(-2.2100874E38F) ;
        p230.time_usec_SET(8902874433169875766L) ;
        p230.vel_ratio_SET(-1.7839233E38F) ;
        p230.pos_vert_ratio_SET(4.0435486E37F) ;
        p230.pos_vert_accuracy_SET(-2.2764506E38F) ;
        p230.pos_horiz_ratio_SET(-5.8793934E37F) ;
        p230.mag_ratio_SET(-3.217783E38F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_alt_GET() == -1.9212398E37F);
            assert(pack.vert_accuracy_GET() == -9.951042E37F);
            assert(pack.time_usec_GET() == 6000995847683767474L);
            assert(pack.var_vert_GET() == 2.5519934E38F);
            assert(pack.var_horiz_GET() == 7.598335E37F);
            assert(pack.wind_x_GET() == -1.1106268E38F);
            assert(pack.wind_y_GET() == 9.104462E37F);
            assert(pack.horiz_accuracy_GET() == 2.1573825E38F);
            assert(pack.wind_z_GET() == 1.4379547E38F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.vert_accuracy_SET(-9.951042E37F) ;
        p231.time_usec_SET(6000995847683767474L) ;
        p231.var_horiz_SET(7.598335E37F) ;
        p231.wind_z_SET(1.4379547E38F) ;
        p231.wind_x_SET(-1.1106268E38F) ;
        p231.wind_y_SET(9.104462E37F) ;
        p231.wind_alt_SET(-1.9212398E37F) ;
        p231.var_vert_SET(2.5519934E38F) ;
        p231.horiz_accuracy_SET(2.1573825E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.vn_GET() == -5.3644266E37F);
            assert(pack.fix_type_GET() == (char)148);
            assert(pack.hdop_GET() == -2.5587826E38F);
            assert(pack.satellites_visible_GET() == (char)255);
            assert(pack.alt_GET() == -1.1547956E37F);
            assert(pack.lon_GET() == 916272393);
            assert(pack.horiz_accuracy_GET() == 5.106629E36F);
            assert(pack.lat_GET() == 198483521);
            assert(pack.time_usec_GET() == 3480522341291907301L);
            assert(pack.ve_GET() == 2.9983817E38F);
            assert(pack.time_week_ms_GET() == 2205162804L);
            assert(pack.vdop_GET() == -1.7737142E38F);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ));
            assert(pack.vd_GET() == -1.0244331E38F);
            assert(pack.vert_accuracy_GET() == -1.7339092E37F);
            assert(pack.time_week_GET() == (char)42132);
            assert(pack.gps_id_GET() == (char)58);
            assert(pack.speed_accuracy_GET() == -2.7617322E38F);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.lon_SET(916272393) ;
        p232.ve_SET(2.9983817E38F) ;
        p232.vn_SET(-5.3644266E37F) ;
        p232.satellites_visible_SET((char)255) ;
        p232.hdop_SET(-2.5587826E38F) ;
        p232.time_week_ms_SET(2205162804L) ;
        p232.gps_id_SET((char)58) ;
        p232.lat_SET(198483521) ;
        p232.horiz_accuracy_SET(5.106629E36F) ;
        p232.time_week_SET((char)42132) ;
        p232.alt_SET(-1.1547956E37F) ;
        p232.fix_type_SET((char)148) ;
        p232.time_usec_SET(3480522341291907301L) ;
        p232.speed_accuracy_SET(-2.7617322E38F) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ)) ;
        p232.vert_accuracy_SET(-1.7339092E37F) ;
        p232.vd_SET(-1.0244331E38F) ;
        p232.vdop_SET(-1.7737142E38F) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)136);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)3, (char)131, (char)218, (char)121, (char)160, (char)122, (char)231, (char)224, (char)174, (char)140, (char)147, (char)177, (char)224, (char)69, (char)189, (char)29, (char)234, (char)246, (char)200, (char)224, (char)220, (char)79, (char)171, (char)250, (char)81, (char)237, (char)99, (char)161, (char)60, (char)111, (char)200, (char)108, (char)144, (char)57, (char)200, (char)129, (char)141, (char)42, (char)246, (char)187, (char)76, (char)117, (char)198, (char)150, (char)244, (char)0, (char)115, (char)64, (char)89, (char)77, (char)75, (char)212, (char)235, (char)127, (char)185, (char)166, (char)48, (char)12, (char)152, (char)178, (char)161, (char)86, (char)128, (char)78, (char)126, (char)66, (char)76, (char)145, (char)249, (char)18, (char)27, (char)127, (char)139, (char)255, (char)79, (char)176, (char)144, (char)56, (char)26, (char)105, (char)24, (char)247, (char)250, (char)197, (char)154, (char)211, (char)162, (char)84, (char)206, (char)139, (char)185, (char)217, (char)223, (char)227, (char)97, (char)191, (char)200, (char)11, (char)252, (char)253, (char)230, (char)25, (char)86, (char)53, (char)137, (char)121, (char)245, (char)90, (char)149, (char)93, (char)165, (char)24, (char)154, (char)57, (char)253, (char)4, (char)49, (char)243, (char)32, (char)251, (char)136, (char)86, (char)145, (char)254, (char)105, (char)217, (char)192, (char)225, (char)65, (char)148, (char)174, (char)229, (char)77, (char)173, (char)159, (char)162, (char)24, (char)147, (char)172, (char)252, (char)109, (char)138, (char)216, (char)137, (char)14, (char)36, (char)218, (char)215, (char)126, (char)233, (char)53, (char)126, (char)180, (char)159, (char)88, (char)30, (char)134, (char)176, (char)112, (char)142, (char)102, (char)245, (char)6, (char)87, (char)150, (char)192, (char)205, (char)109, (char)132, (char)255, (char)130, (char)55, (char)88, (char)217, (char)28, (char)79, (char)169, (char)37, (char)29, (char)62}));
            assert(pack.flags_GET() == (char)197);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)197) ;
        p233.data__SET(new char[] {(char)3, (char)131, (char)218, (char)121, (char)160, (char)122, (char)231, (char)224, (char)174, (char)140, (char)147, (char)177, (char)224, (char)69, (char)189, (char)29, (char)234, (char)246, (char)200, (char)224, (char)220, (char)79, (char)171, (char)250, (char)81, (char)237, (char)99, (char)161, (char)60, (char)111, (char)200, (char)108, (char)144, (char)57, (char)200, (char)129, (char)141, (char)42, (char)246, (char)187, (char)76, (char)117, (char)198, (char)150, (char)244, (char)0, (char)115, (char)64, (char)89, (char)77, (char)75, (char)212, (char)235, (char)127, (char)185, (char)166, (char)48, (char)12, (char)152, (char)178, (char)161, (char)86, (char)128, (char)78, (char)126, (char)66, (char)76, (char)145, (char)249, (char)18, (char)27, (char)127, (char)139, (char)255, (char)79, (char)176, (char)144, (char)56, (char)26, (char)105, (char)24, (char)247, (char)250, (char)197, (char)154, (char)211, (char)162, (char)84, (char)206, (char)139, (char)185, (char)217, (char)223, (char)227, (char)97, (char)191, (char)200, (char)11, (char)252, (char)253, (char)230, (char)25, (char)86, (char)53, (char)137, (char)121, (char)245, (char)90, (char)149, (char)93, (char)165, (char)24, (char)154, (char)57, (char)253, (char)4, (char)49, (char)243, (char)32, (char)251, (char)136, (char)86, (char)145, (char)254, (char)105, (char)217, (char)192, (char)225, (char)65, (char)148, (char)174, (char)229, (char)77, (char)173, (char)159, (char)162, (char)24, (char)147, (char)172, (char)252, (char)109, (char)138, (char)216, (char)137, (char)14, (char)36, (char)218, (char)215, (char)126, (char)233, (char)53, (char)126, (char)180, (char)159, (char)88, (char)30, (char)134, (char)176, (char)112, (char)142, (char)102, (char)245, (char)6, (char)87, (char)150, (char)192, (char)205, (char)109, (char)132, (char)255, (char)130, (char)55, (char)88, (char)217, (char)28, (char)79, (char)169, (char)37, (char)29, (char)62}, 0) ;
        p233.len_SET((char)136) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.wp_num_GET() == (char)182);
            assert(pack.custom_mode_GET() == 34892435L);
            assert(pack.throttle_GET() == (byte)13);
            assert(pack.roll_GET() == (short) -21745);
            assert(pack.climb_rate_GET() == (byte) - 92);
            assert(pack.latitude_GET() == 1456686825);
            assert(pack.airspeed_GET() == (char)198);
            assert(pack.heading_GET() == (char)58547);
            assert(pack.airspeed_sp_GET() == (char)184);
            assert(pack.groundspeed_GET() == (char)98);
            assert(pack.longitude_GET() == -497032438);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED));
            assert(pack.temperature_GET() == (byte) - 54);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
            assert(pack.temperature_air_GET() == (byte)77);
            assert(pack.altitude_sp_GET() == (short) -32629);
            assert(pack.pitch_GET() == (short) -9014);
            assert(pack.altitude_amsl_GET() == (short) -12103);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
            assert(pack.wp_distance_GET() == (char)59177);
            assert(pack.heading_sp_GET() == (short)6287);
            assert(pack.failsafe_GET() == (char)71);
            assert(pack.battery_remaining_GET() == (char)67);
            assert(pack.gps_nsat_GET() == (char)72);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.temperature_air_SET((byte)77) ;
        p234.temperature_SET((byte) - 54) ;
        p234.heading_SET((char)58547) ;
        p234.battery_remaining_SET((char)67) ;
        p234.gps_nsat_SET((char)72) ;
        p234.failsafe_SET((char)71) ;
        p234.wp_distance_SET((char)59177) ;
        p234.airspeed_sp_SET((char)184) ;
        p234.custom_mode_SET(34892435L) ;
        p234.longitude_SET(-497032438) ;
        p234.airspeed_SET((char)198) ;
        p234.pitch_SET((short) -9014) ;
        p234.heading_sp_SET((short)6287) ;
        p234.throttle_SET((byte)13) ;
        p234.groundspeed_SET((char)98) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)) ;
        p234.climb_rate_SET((byte) - 92) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX) ;
        p234.latitude_SET(1456686825) ;
        p234.altitude_amsl_SET((short) -12103) ;
        p234.roll_SET((short) -21745) ;
        p234.altitude_sp_SET((short) -32629) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND) ;
        p234.wp_num_SET((char)182) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3555515640896258891L);
            assert(pack.vibration_y_GET() == -3.3817934E38F);
            assert(pack.vibration_z_GET() == 3.2826139E38F);
            assert(pack.clipping_0_GET() == 2207137261L);
            assert(pack.clipping_1_GET() == 1902540010L);
            assert(pack.clipping_2_GET() == 1866479847L);
            assert(pack.vibration_x_GET() == -3.1002048E38F);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.vibration_x_SET(-3.1002048E38F) ;
        p241.clipping_2_SET(1866479847L) ;
        p241.vibration_z_SET(3.2826139E38F) ;
        p241.clipping_0_SET(2207137261L) ;
        p241.time_usec_SET(3555515640896258891L) ;
        p241.clipping_1_SET(1902540010L) ;
        p241.vibration_y_SET(-3.3817934E38F) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == 675545578);
            assert(pack.approach_z_GET() == -2.6842327E38F);
            assert(pack.y_GET() == -2.8056846E38F);
            assert(pack.longitude_GET() == -960095622);
            assert(pack.approach_y_GET() == -2.709051E37F);
            assert(pack.time_usec_TRY(ph) == 5038016016119374150L);
            assert(pack.latitude_GET() == 1455537490);
            assert(pack.approach_x_GET() == 1.9324692E38F);
            assert(pack.z_GET() == -2.8526923E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.8774886E38F, -8.87712E37F, 1.3589402E38F, 1.4939171E38F}));
            assert(pack.x_GET() == -1.2318579E38F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.approach_y_SET(-2.709051E37F) ;
        p242.latitude_SET(1455537490) ;
        p242.longitude_SET(-960095622) ;
        p242.z_SET(-2.8526923E38F) ;
        p242.altitude_SET(675545578) ;
        p242.y_SET(-2.8056846E38F) ;
        p242.x_SET(-1.2318579E38F) ;
        p242.approach_z_SET(-2.6842327E38F) ;
        p242.approach_x_SET(1.9324692E38F) ;
        p242.time_usec_SET(5038016016119374150L, PH) ;
        p242.q_SET(new float[] {2.8774886E38F, -8.87712E37F, 1.3589402E38F, 1.4939171E38F}, 0) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 1237366875);
            assert(pack.target_system_GET() == (char)237);
            assert(pack.longitude_GET() == 1846789879);
            assert(pack.approach_z_GET() == -1.0580554E38F);
            assert(pack.altitude_GET() == 1677651450);
            assert(pack.approach_x_GET() == -9.843448E36F);
            assert(pack.approach_y_GET() == -1.0304397E38F);
            assert(pack.z_GET() == 2.1996068E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.094063E38F, 2.6777598E38F, 1.7796425E38F, -2.7366414E38F}));
            assert(pack.x_GET() == 1.028852E38F);
            assert(pack.time_usec_TRY(ph) == 9223208832881449652L);
            assert(pack.y_GET() == 2.6786621E38F);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.latitude_SET(1237366875) ;
        p243.time_usec_SET(9223208832881449652L, PH) ;
        p243.z_SET(2.1996068E38F) ;
        p243.longitude_SET(1846789879) ;
        p243.altitude_SET(1677651450) ;
        p243.q_SET(new float[] {2.094063E38F, 2.6777598E38F, 1.7796425E38F, -2.7366414E38F}, 0) ;
        p243.approach_y_SET(-1.0304397E38F) ;
        p243.approach_x_SET(-9.843448E36F) ;
        p243.target_system_SET((char)237) ;
        p243.approach_z_SET(-1.0580554E38F) ;
        p243.y_SET(2.6786621E38F) ;
        p243.x_SET(1.028852E38F) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)38530);
            assert(pack.interval_us_GET() == 2113496687);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(2113496687) ;
        p244.message_id_SET((char)38530) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_MC) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_NO_INFO);
            assert(pack.hor_velocity_GET() == (char)18018);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK));
            assert(pack.altitude_GET() == -1680234657);
            assert(pack.callsign_LEN(ph) == 6);
            assert(pack.callsign_TRY(ph).equals("qgxhdq"));
            assert(pack.squawk_GET() == (char)43876);
            assert(pack.ICAO_address_GET() == 285336871L);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            assert(pack.ver_velocity_GET() == (short) -25871);
            assert(pack.tslc_GET() == (char)100);
            assert(pack.heading_GET() == (char)14076);
            assert(pack.lat_GET() == -479423015);
            assert(pack.lon_GET() == -699809901);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.altitude_SET(-1680234657) ;
        p246.ICAO_address_SET(285336871L) ;
        p246.squawk_SET((char)43876) ;
        p246.lat_SET(-479423015) ;
        p246.tslc_SET((char)100) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK)) ;
        p246.callsign_SET("qgxhdq", PH) ;
        p246.hor_velocity_SET((char)18018) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.ver_velocity_SET((short) -25871) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_NO_INFO) ;
        p246.lon_SET(-699809901) ;
        p246.heading_SET((char)14076) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
            assert(pack.altitude_minimum_delta_GET() == -9.426789E37F);
            assert(pack.id_GET() == 153870466L);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
            assert(pack.time_to_minimum_delta_GET() == -2.2770556E38F);
            assert(pack.horizontal_minimum_delta_GET() == 7.3314877E37F);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR) ;
        p247.id_SET(153870466L) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        p247.altitude_minimum_delta_SET(-9.426789E37F) ;
        p247.time_to_minimum_delta_SET(-2.2770556E38F) ;
        p247.horizontal_minimum_delta_SET(7.3314877E37F) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)90);
            assert(pack.target_component_GET() == (char)132);
            assert(pack.target_network_GET() == (char)131);
            assert(pack.message_type_GET() == (char)17177);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)41, (char)214, (char)254, (char)33, (char)1, (char)33, (char)111, (char)209, (char)245, (char)2, (char)235, (char)245, (char)206, (char)125, (char)26, (char)112, (char)246, (char)237, (char)84, (char)117, (char)215, (char)108, (char)253, (char)12, (char)144, (char)23, (char)18, (char)121, (char)183, (char)201, (char)196, (char)62, (char)80, (char)61, (char)156, (char)200, (char)156, (char)223, (char)191, (char)23, (char)54, (char)20, (char)238, (char)60, (char)35, (char)48, (char)142, (char)45, (char)35, (char)214, (char)182, (char)74, (char)139, (char)134, (char)69, (char)171, (char)131, (char)43, (char)10, (char)173, (char)12, (char)8, (char)253, (char)24, (char)79, (char)138, (char)101, (char)87, (char)94, (char)45, (char)90, (char)54, (char)199, (char)22, (char)242, (char)43, (char)91, (char)121, (char)128, (char)102, (char)175, (char)152, (char)227, (char)107, (char)52, (char)68, (char)109, (char)22, (char)4, (char)175, (char)33, (char)252, (char)176, (char)179, (char)181, (char)225, (char)2, (char)89, (char)131, (char)101, (char)222, (char)243, (char)248, (char)195, (char)122, (char)186, (char)220, (char)31, (char)136, (char)222, (char)80, (char)10, (char)31, (char)151, (char)121, (char)60, (char)78, (char)246, (char)33, (char)55, (char)207, (char)26, (char)127, (char)38, (char)113, (char)23, (char)235, (char)144, (char)235, (char)25, (char)218, (char)136, (char)146, (char)160, (char)32, (char)210, (char)67, (char)204, (char)22, (char)68, (char)23, (char)173, (char)174, (char)85, (char)84, (char)163, (char)168, (char)101, (char)19, (char)249, (char)103, (char)1, (char)202, (char)183, (char)64, (char)216, (char)222, (char)119, (char)189, (char)32, (char)214, (char)252, (char)229, (char)229, (char)112, (char)112, (char)112, (char)195, (char)43, (char)148, (char)2, (char)77, (char)234, (char)69, (char)255, (char)78, (char)112, (char)235, (char)101, (char)121, (char)255, (char)252, (char)255, (char)59, (char)119, (char)40, (char)6, (char)95, (char)109, (char)29, (char)147, (char)75, (char)216, (char)99, (char)152, (char)152, (char)102, (char)195, (char)87, (char)105, (char)124, (char)163, (char)83, (char)96, (char)109, (char)15, (char)41, (char)132, (char)226, (char)232, (char)160, (char)230, (char)122, (char)165, (char)86, (char)96, (char)6, (char)121, (char)243, (char)99, (char)203, (char)139, (char)160, (char)110, (char)69, (char)88, (char)15, (char)36, (char)17, (char)180, (char)227, (char)38, (char)166, (char)49, (char)45, (char)170, (char)50, (char)12, (char)214, (char)121, (char)112, (char)2, (char)210, (char)75, (char)64, (char)22, (char)84, (char)103, (char)65}));
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_component_SET((char)132) ;
        p248.message_type_SET((char)17177) ;
        p248.target_network_SET((char)131) ;
        p248.payload_SET(new char[] {(char)41, (char)214, (char)254, (char)33, (char)1, (char)33, (char)111, (char)209, (char)245, (char)2, (char)235, (char)245, (char)206, (char)125, (char)26, (char)112, (char)246, (char)237, (char)84, (char)117, (char)215, (char)108, (char)253, (char)12, (char)144, (char)23, (char)18, (char)121, (char)183, (char)201, (char)196, (char)62, (char)80, (char)61, (char)156, (char)200, (char)156, (char)223, (char)191, (char)23, (char)54, (char)20, (char)238, (char)60, (char)35, (char)48, (char)142, (char)45, (char)35, (char)214, (char)182, (char)74, (char)139, (char)134, (char)69, (char)171, (char)131, (char)43, (char)10, (char)173, (char)12, (char)8, (char)253, (char)24, (char)79, (char)138, (char)101, (char)87, (char)94, (char)45, (char)90, (char)54, (char)199, (char)22, (char)242, (char)43, (char)91, (char)121, (char)128, (char)102, (char)175, (char)152, (char)227, (char)107, (char)52, (char)68, (char)109, (char)22, (char)4, (char)175, (char)33, (char)252, (char)176, (char)179, (char)181, (char)225, (char)2, (char)89, (char)131, (char)101, (char)222, (char)243, (char)248, (char)195, (char)122, (char)186, (char)220, (char)31, (char)136, (char)222, (char)80, (char)10, (char)31, (char)151, (char)121, (char)60, (char)78, (char)246, (char)33, (char)55, (char)207, (char)26, (char)127, (char)38, (char)113, (char)23, (char)235, (char)144, (char)235, (char)25, (char)218, (char)136, (char)146, (char)160, (char)32, (char)210, (char)67, (char)204, (char)22, (char)68, (char)23, (char)173, (char)174, (char)85, (char)84, (char)163, (char)168, (char)101, (char)19, (char)249, (char)103, (char)1, (char)202, (char)183, (char)64, (char)216, (char)222, (char)119, (char)189, (char)32, (char)214, (char)252, (char)229, (char)229, (char)112, (char)112, (char)112, (char)195, (char)43, (char)148, (char)2, (char)77, (char)234, (char)69, (char)255, (char)78, (char)112, (char)235, (char)101, (char)121, (char)255, (char)252, (char)255, (char)59, (char)119, (char)40, (char)6, (char)95, (char)109, (char)29, (char)147, (char)75, (char)216, (char)99, (char)152, (char)152, (char)102, (char)195, (char)87, (char)105, (char)124, (char)163, (char)83, (char)96, (char)109, (char)15, (char)41, (char)132, (char)226, (char)232, (char)160, (char)230, (char)122, (char)165, (char)86, (char)96, (char)6, (char)121, (char)243, (char)99, (char)203, (char)139, (char)160, (char)110, (char)69, (char)88, (char)15, (char)36, (char)17, (char)180, (char)227, (char)38, (char)166, (char)49, (char)45, (char)170, (char)50, (char)12, (char)214, (char)121, (char)112, (char)2, (char)210, (char)75, (char)64, (char)22, (char)84, (char)103, (char)65}, 0) ;
        p248.target_system_SET((char)90) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.address_GET() == (char)44795);
            assert(pack.type_GET() == (char)214);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte) - 41, (byte) - 24, (byte) - 61, (byte)117, (byte)122, (byte)123, (byte) - 67, (byte) - 122, (byte) - 123, (byte)54, (byte)33, (byte) - 115, (byte) - 24, (byte)55, (byte)99, (byte)11, (byte) - 39, (byte) - 82, (byte)51, (byte) - 118, (byte)125, (byte) - 120, (byte)105, (byte)83, (byte) - 78, (byte)1, (byte)10, (byte)114, (byte)106, (byte) - 105, (byte) - 94, (byte) - 89}));
            assert(pack.ver_GET() == (char)38);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.ver_SET((char)38) ;
        p249.value_SET(new byte[] {(byte) - 41, (byte) - 24, (byte) - 61, (byte)117, (byte)122, (byte)123, (byte) - 67, (byte) - 122, (byte) - 123, (byte)54, (byte)33, (byte) - 115, (byte) - 24, (byte)55, (byte)99, (byte)11, (byte) - 39, (byte) - 82, (byte)51, (byte) - 118, (byte)125, (byte) - 120, (byte)105, (byte)83, (byte) - 78, (byte)1, (byte)10, (byte)114, (byte)106, (byte) - 105, (byte) - 94, (byte) - 89}, 0) ;
        p249.type_SET((char)214) ;
        p249.address_SET((char)44795) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -3.3254607E38F);
            assert(pack.y_GET() == -1.1761769E38F);
            assert(pack.time_usec_GET() == 3833206282139088697L);
            assert(pack.name_LEN(ph) == 8);
            assert(pack.name_TRY(ph).equals("leyxmxpt"));
            assert(pack.z_GET() == -3.247104E38F);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.time_usec_SET(3833206282139088697L) ;
        p250.z_SET(-3.247104E38F) ;
        p250.y_SET(-1.1761769E38F) ;
        p250.name_SET("leyxmxpt", PH) ;
        p250.x_SET(-3.3254607E38F) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 661470757L);
            assert(pack.value_GET() == -9.089554E37F);
            assert(pack.name_LEN(ph) == 4);
            assert(pack.name_TRY(ph).equals("aySx"));
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.value_SET(-9.089554E37F) ;
        p251.name_SET("aySx", PH) ;
        p251.time_boot_ms_SET(661470757L) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == -870910958);
            assert(pack.time_boot_ms_GET() == 1636143645L);
            assert(pack.name_LEN(ph) == 6);
            assert(pack.name_TRY(ph).equals("mlqzmf"));
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.name_SET("mlqzmf", PH) ;
        p252.time_boot_ms_SET(1636143645L) ;
        p252.value_SET(-870910958) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 22);
            assert(pack.text_TRY(ph).equals("mmxigZellddbgqmeIszYdb"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_ERROR);
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("mmxigZellddbgqmeIszYdb", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_ERROR) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3668933182L);
            assert(pack.ind_GET() == (char)0);
            assert(pack.value_GET() == -1.4975197E38F);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.ind_SET((char)0) ;
        p254.value_SET(-1.4975197E38F) ;
        p254.time_boot_ms_SET(3668933182L) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)202, (char)199, (char)99, (char)149, (char)186, (char)241, (char)22, (char)144, (char)177, (char)96, (char)26, (char)251, (char)157, (char)104, (char)13, (char)223, (char)37, (char)133, (char)39, (char)84, (char)28, (char)146, (char)230, (char)156, (char)143, (char)253, (char)227, (char)88, (char)197, (char)50, (char)221, (char)52}));
            assert(pack.target_component_GET() == (char)89);
            assert(pack.target_system_GET() == (char)34);
            assert(pack.initial_timestamp_GET() == 7841532333559796983L);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_component_SET((char)89) ;
        p256.secret_key_SET(new char[] {(char)202, (char)199, (char)99, (char)149, (char)186, (char)241, (char)22, (char)144, (char)177, (char)96, (char)26, (char)251, (char)157, (char)104, (char)13, (char)223, (char)37, (char)133, (char)39, (char)84, (char)28, (char)146, (char)230, (char)156, (char)143, (char)253, (char)227, (char)88, (char)197, (char)50, (char)221, (char)52}, 0) ;
        p256.target_system_SET((char)34) ;
        p256.initial_timestamp_SET(7841532333559796983L) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1955165957L);
            assert(pack.state_GET() == (char)215);
            assert(pack.last_change_ms_GET() == 3614341744L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.state_SET((char)215) ;
        p257.last_change_ms_SET(3614341744L) ;
        p257.time_boot_ms_SET(1955165957L) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)54);
            assert(pack.tune_LEN(ph) == 3);
            assert(pack.tune_TRY(ph).equals("pRp"));
            assert(pack.target_component_GET() == (char)223);
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)54) ;
        p258.target_component_SET((char)223) ;
        p258.tune_SET("pRp", PH) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.resolution_v_GET() == (char)9163);
            assert(pack.sensor_size_h_GET() == 2.7191002E38F);
            assert(pack.lens_id_GET() == (char)115);
            assert(pack.time_boot_ms_GET() == 1316770648L);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)67, (char)139, (char)80, (char)233, (char)130, (char)124, (char)77, (char)192, (char)209, (char)89, (char)93, (char)251, (char)67, (char)83, (char)191, (char)64, (char)99, (char)62, (char)25, (char)144, (char)247, (char)164, (char)106, (char)23, (char)124, (char)221, (char)82, (char)225, (char)238, (char)130, (char)235, (char)4}));
            assert(pack.firmware_version_GET() == 2909966478L);
            assert(pack.cam_definition_version_GET() == (char)60203);
            assert(pack.sensor_size_v_GET() == -1.5194043E38F);
            assert(pack.cam_definition_uri_LEN(ph) == 89);
            assert(pack.cam_definition_uri_TRY(ph).equals("qekiNhgypcuarrorohqbndbZPrsyomhsfiCiiauacbsnbgkqvbmnpipwZaSngiwhkbskweqfnUnpukemwjjyvLfeo"));
            assert(pack.focal_length_GET() == 3.0351462E38F);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)161, (char)10, (char)46, (char)254, (char)17, (char)187, (char)13, (char)79, (char)94, (char)132, (char)117, (char)168, (char)49, (char)92, (char)202, (char)239, (char)178, (char)140, (char)187, (char)30, (char)141, (char)141, (char)169, (char)13, (char)227, (char)109, (char)245, (char)162, (char)166, (char)140, (char)175, (char)106}));
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE));
            assert(pack.resolution_h_GET() == (char)51942);
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.cam_definition_version_SET((char)60203) ;
        p259.vendor_name_SET(new char[] {(char)161, (char)10, (char)46, (char)254, (char)17, (char)187, (char)13, (char)79, (char)94, (char)132, (char)117, (char)168, (char)49, (char)92, (char)202, (char)239, (char)178, (char)140, (char)187, (char)30, (char)141, (char)141, (char)169, (char)13, (char)227, (char)109, (char)245, (char)162, (char)166, (char)140, (char)175, (char)106}, 0) ;
        p259.model_name_SET(new char[] {(char)67, (char)139, (char)80, (char)233, (char)130, (char)124, (char)77, (char)192, (char)209, (char)89, (char)93, (char)251, (char)67, (char)83, (char)191, (char)64, (char)99, (char)62, (char)25, (char)144, (char)247, (char)164, (char)106, (char)23, (char)124, (char)221, (char)82, (char)225, (char)238, (char)130, (char)235, (char)4}, 0) ;
        p259.sensor_size_v_SET(-1.5194043E38F) ;
        p259.lens_id_SET((char)115) ;
        p259.cam_definition_uri_SET("qekiNhgypcuarrorohqbndbZPrsyomhsfiCiiauacbsnbgkqvbmnpipwZaSngiwhkbskweqfnUnpukemwjjyvLfeo", PH) ;
        p259.time_boot_ms_SET(1316770648L) ;
        p259.sensor_size_h_SET(2.7191002E38F) ;
        p259.resolution_h_SET((char)51942) ;
        p259.focal_length_SET(3.0351462E38F) ;
        p259.resolution_v_SET((char)9163) ;
        p259.firmware_version_SET(2909966478L) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE)) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 810228492L);
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY) ;
        p260.time_boot_ms_SET(810228492L) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.write_speed_GET() == 1.6018612E37F);
            assert(pack.used_capacity_GET() == 2.316408E38F);
            assert(pack.storage_count_GET() == (char)4);
            assert(pack.storage_id_GET() == (char)119);
            assert(pack.status_GET() == (char)112);
            assert(pack.available_capacity_GET() == 1.2117674E38F);
            assert(pack.total_capacity_GET() == 7.992763E37F);
            assert(pack.time_boot_ms_GET() == 1447665786L);
            assert(pack.read_speed_GET() == 1.4816459E38F);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.used_capacity_SET(2.316408E38F) ;
        p261.status_SET((char)112) ;
        p261.storage_count_SET((char)4) ;
        p261.write_speed_SET(1.6018612E37F) ;
        p261.read_speed_SET(1.4816459E38F) ;
        p261.available_capacity_SET(1.2117674E38F) ;
        p261.total_capacity_SET(7.992763E37F) ;
        p261.time_boot_ms_SET(1447665786L) ;
        p261.storage_id_SET((char)119) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.video_status_GET() == (char)145);
            assert(pack.time_boot_ms_GET() == 4041831397L);
            assert(pack.available_capacity_GET() == -1.0439339E38F);
            assert(pack.image_status_GET() == (char)46);
            assert(pack.recording_time_ms_GET() == 1229959250L);
            assert(pack.image_interval_GET() == -2.6447587E38F);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.image_interval_SET(-2.6447587E38F) ;
        p262.time_boot_ms_SET(4041831397L) ;
        p262.video_status_SET((char)145) ;
        p262.available_capacity_SET(-1.0439339E38F) ;
        p262.image_status_SET((char)46) ;
        p262.recording_time_ms_SET(1229959250L) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.capture_result_GET() == (byte) - 30);
            assert(pack.time_boot_ms_GET() == 1251076267L);
            assert(pack.relative_alt_GET() == 58127269);
            assert(pack.lon_GET() == -1248264647);
            assert(pack.camera_id_GET() == (char)175);
            assert(pack.file_url_LEN(ph) == 41);
            assert(pack.file_url_TRY(ph).equals("xfapRaokozdqlfgucogaalxjqvpmjXirXhbnbbpLe"));
            assert(pack.alt_GET() == -823540909);
            assert(pack.image_index_GET() == 843660240);
            assert(pack.time_utc_GET() == 4537222063161813702L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.9769238E38F, -1.6201176E38F, -1.2879816E38F, 1.8224187E38F}));
            assert(pack.lat_GET() == -1714607587);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.file_url_SET("xfapRaokozdqlfgucogaalxjqvpmjXirXhbnbbpLe", PH) ;
        p263.time_utc_SET(4537222063161813702L) ;
        p263.lat_SET(-1714607587) ;
        p263.relative_alt_SET(58127269) ;
        p263.image_index_SET(843660240) ;
        p263.camera_id_SET((char)175) ;
        p263.q_SET(new float[] {1.9769238E38F, -1.6201176E38F, -1.2879816E38F, 1.8224187E38F}, 0) ;
        p263.capture_result_SET((byte) - 30) ;
        p263.lon_SET(-1248264647) ;
        p263.alt_SET(-823540909) ;
        p263.time_boot_ms_SET(1251076267L) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.flight_uuid_GET() == 3170374297677086700L);
            assert(pack.time_boot_ms_GET() == 361393369L);
            assert(pack.arming_time_utc_GET() == 5147122304207404806L);
            assert(pack.takeoff_time_utc_GET() == 6477717165944638468L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.flight_uuid_SET(3170374297677086700L) ;
        p264.takeoff_time_utc_SET(6477717165944638468L) ;
        p264.arming_time_utc_SET(5147122304207404806L) ;
        p264.time_boot_ms_SET(361393369L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2976800421L);
            assert(pack.yaw_GET() == -1.3112922E38F);
            assert(pack.roll_GET() == 2.2969158E38F);
            assert(pack.pitch_GET() == -1.9109843E38F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.yaw_SET(-1.3112922E38F) ;
        p265.pitch_SET(-1.9109843E38F) ;
        p265.roll_SET(2.2969158E38F) ;
        p265.time_boot_ms_SET(2976800421L) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)12923);
            assert(pack.length_GET() == (char)34);
            assert(pack.target_system_GET() == (char)80);
            assert(pack.target_component_GET() == (char)77);
            assert(pack.first_message_offset_GET() == (char)21);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)46, (char)126, (char)7, (char)175, (char)244, (char)57, (char)194, (char)13, (char)0, (char)249, (char)249, (char)10, (char)191, (char)215, (char)155, (char)137, (char)87, (char)141, (char)205, (char)4, (char)176, (char)18, (char)18, (char)12, (char)3, (char)101, (char)211, (char)136, (char)197, (char)124, (char)135, (char)190, (char)100, (char)183, (char)103, (char)204, (char)205, (char)182, (char)6, (char)123, (char)149, (char)203, (char)69, (char)3, (char)84, (char)145, (char)157, (char)163, (char)169, (char)147, (char)242, (char)65, (char)15, (char)52, (char)102, (char)165, (char)19, (char)200, (char)192, (char)229, (char)68, (char)3, (char)29, (char)64, (char)22, (char)125, (char)238, (char)203, (char)168, (char)211, (char)144, (char)163, (char)16, (char)243, (char)16, (char)248, (char)94, (char)248, (char)90, (char)223, (char)245, (char)240, (char)170, (char)89, (char)250, (char)248, (char)111, (char)14, (char)29, (char)251, (char)113, (char)128, (char)206, (char)237, (char)64, (char)158, (char)202, (char)163, (char)6, (char)42, (char)34, (char)43, (char)208, (char)77, (char)6, (char)78, (char)0, (char)167, (char)177, (char)246, (char)249, (char)238, (char)154, (char)211, (char)8, (char)226, (char)195, (char)93, (char)64, (char)80, (char)10, (char)40, (char)116, (char)182, (char)248, (char)30, (char)5, (char)236, (char)71, (char)102, (char)243, (char)52, (char)187, (char)65, (char)130, (char)107, (char)144, (char)107, (char)19, (char)76, (char)246, (char)114, (char)38, (char)7, (char)96, (char)200, (char)133, (char)46, (char)94, (char)12, (char)246, (char)126, (char)54, (char)82, (char)86, (char)175, (char)109, (char)211, (char)229, (char)44, (char)4, (char)167, (char)107, (char)201, (char)243, (char)123, (char)172, (char)76, (char)78, (char)179, (char)199, (char)131, (char)217, (char)28, (char)11, (char)110, (char)172, (char)209, (char)104, (char)97, (char)75, (char)174, (char)218, (char)35, (char)134, (char)8, (char)68, (char)186, (char)184, (char)168, (char)150, (char)140, (char)165, (char)131, (char)132, (char)129, (char)246, (char)206, (char)151, (char)250, (char)129, (char)243, (char)161, (char)231, (char)1, (char)222, (char)182, (char)68, (char)167, (char)53, (char)44, (char)235, (char)107, (char)94, (char)136, (char)233, (char)108, (char)2, (char)121, (char)73, (char)104, (char)136, (char)59, (char)120, (char)175, (char)220, (char)159, (char)215, (char)170, (char)187, (char)239, (char)9, (char)165, (char)192, (char)102, (char)202, (char)187, (char)114, (char)104, (char)145, (char)110, (char)39, (char)118, (char)117, (char)194, (char)63, (char)45, (char)147, (char)201}));
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.sequence_SET((char)12923) ;
        p266.target_system_SET((char)80) ;
        p266.target_component_SET((char)77) ;
        p266.data__SET(new char[] {(char)46, (char)126, (char)7, (char)175, (char)244, (char)57, (char)194, (char)13, (char)0, (char)249, (char)249, (char)10, (char)191, (char)215, (char)155, (char)137, (char)87, (char)141, (char)205, (char)4, (char)176, (char)18, (char)18, (char)12, (char)3, (char)101, (char)211, (char)136, (char)197, (char)124, (char)135, (char)190, (char)100, (char)183, (char)103, (char)204, (char)205, (char)182, (char)6, (char)123, (char)149, (char)203, (char)69, (char)3, (char)84, (char)145, (char)157, (char)163, (char)169, (char)147, (char)242, (char)65, (char)15, (char)52, (char)102, (char)165, (char)19, (char)200, (char)192, (char)229, (char)68, (char)3, (char)29, (char)64, (char)22, (char)125, (char)238, (char)203, (char)168, (char)211, (char)144, (char)163, (char)16, (char)243, (char)16, (char)248, (char)94, (char)248, (char)90, (char)223, (char)245, (char)240, (char)170, (char)89, (char)250, (char)248, (char)111, (char)14, (char)29, (char)251, (char)113, (char)128, (char)206, (char)237, (char)64, (char)158, (char)202, (char)163, (char)6, (char)42, (char)34, (char)43, (char)208, (char)77, (char)6, (char)78, (char)0, (char)167, (char)177, (char)246, (char)249, (char)238, (char)154, (char)211, (char)8, (char)226, (char)195, (char)93, (char)64, (char)80, (char)10, (char)40, (char)116, (char)182, (char)248, (char)30, (char)5, (char)236, (char)71, (char)102, (char)243, (char)52, (char)187, (char)65, (char)130, (char)107, (char)144, (char)107, (char)19, (char)76, (char)246, (char)114, (char)38, (char)7, (char)96, (char)200, (char)133, (char)46, (char)94, (char)12, (char)246, (char)126, (char)54, (char)82, (char)86, (char)175, (char)109, (char)211, (char)229, (char)44, (char)4, (char)167, (char)107, (char)201, (char)243, (char)123, (char)172, (char)76, (char)78, (char)179, (char)199, (char)131, (char)217, (char)28, (char)11, (char)110, (char)172, (char)209, (char)104, (char)97, (char)75, (char)174, (char)218, (char)35, (char)134, (char)8, (char)68, (char)186, (char)184, (char)168, (char)150, (char)140, (char)165, (char)131, (char)132, (char)129, (char)246, (char)206, (char)151, (char)250, (char)129, (char)243, (char)161, (char)231, (char)1, (char)222, (char)182, (char)68, (char)167, (char)53, (char)44, (char)235, (char)107, (char)94, (char)136, (char)233, (char)108, (char)2, (char)121, (char)73, (char)104, (char)136, (char)59, (char)120, (char)175, (char)220, (char)159, (char)215, (char)170, (char)187, (char)239, (char)9, (char)165, (char)192, (char)102, (char)202, (char)187, (char)114, (char)104, (char)145, (char)110, (char)39, (char)118, (char)117, (char)194, (char)63, (char)45, (char)147, (char)201}, 0) ;
        p266.length_SET((char)34) ;
        p266.first_message_offset_SET((char)21) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)156);
            assert(pack.sequence_GET() == (char)16531);
            assert(pack.length_GET() == (char)253);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)242, (char)55, (char)73, (char)97, (char)79, (char)107, (char)218, (char)40, (char)155, (char)49, (char)114, (char)74, (char)75, (char)142, (char)60, (char)61, (char)176, (char)96, (char)48, (char)88, (char)169, (char)244, (char)96, (char)206, (char)13, (char)29, (char)198, (char)129, (char)61, (char)78, (char)171, (char)125, (char)232, (char)89, (char)240, (char)110, (char)60, (char)159, (char)203, (char)17, (char)246, (char)131, (char)247, (char)17, (char)87, (char)55, (char)203, (char)23, (char)131, (char)140, (char)77, (char)60, (char)1, (char)19, (char)176, (char)30, (char)166, (char)105, (char)223, (char)71, (char)15, (char)17, (char)97, (char)155, (char)84, (char)119, (char)87, (char)86, (char)173, (char)107, (char)188, (char)58, (char)177, (char)237, (char)240, (char)128, (char)104, (char)144, (char)125, (char)65, (char)158, (char)252, (char)255, (char)223, (char)133, (char)50, (char)184, (char)129, (char)231, (char)242, (char)241, (char)132, (char)104, (char)76, (char)121, (char)70, (char)56, (char)179, (char)203, (char)65, (char)158, (char)229, (char)61, (char)214, (char)5, (char)159, (char)108, (char)7, (char)179, (char)97, (char)20, (char)207, (char)163, (char)82, (char)240, (char)56, (char)228, (char)238, (char)24, (char)54, (char)245, (char)56, (char)77, (char)23, (char)41, (char)197, (char)222, (char)237, (char)52, (char)74, (char)33, (char)183, (char)158, (char)253, (char)64, (char)84, (char)83, (char)166, (char)43, (char)127, (char)239, (char)109, (char)251, (char)40, (char)12, (char)100, (char)154, (char)28, (char)89, (char)77, (char)12, (char)53, (char)214, (char)15, (char)161, (char)118, (char)191, (char)1, (char)50, (char)111, (char)68, (char)3, (char)204, (char)239, (char)173, (char)76, (char)162, (char)108, (char)248, (char)42, (char)205, (char)77, (char)191, (char)133, (char)155, (char)172, (char)110, (char)20, (char)209, (char)252, (char)215, (char)78, (char)33, (char)125, (char)19, (char)71, (char)72, (char)53, (char)9, (char)95, (char)187, (char)44, (char)33, (char)228, (char)77, (char)34, (char)140, (char)232, (char)78, (char)179, (char)11, (char)208, (char)32, (char)38, (char)76, (char)135, (char)250, (char)112, (char)57, (char)114, (char)107, (char)48, (char)218, (char)186, (char)162, (char)96, (char)187, (char)154, (char)95, (char)88, (char)188, (char)66, (char)71, (char)243, (char)211, (char)184, (char)77, (char)92, (char)145, (char)5, (char)144, (char)103, (char)224, (char)81, (char)31, (char)113, (char)61, (char)98, (char)150, (char)17, (char)26, (char)125, (char)245, (char)134, (char)221, (char)218, (char)220, (char)238, (char)14}));
            assert(pack.target_component_GET() == (char)102);
            assert(pack.first_message_offset_GET() == (char)195);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.data__SET(new char[] {(char)242, (char)55, (char)73, (char)97, (char)79, (char)107, (char)218, (char)40, (char)155, (char)49, (char)114, (char)74, (char)75, (char)142, (char)60, (char)61, (char)176, (char)96, (char)48, (char)88, (char)169, (char)244, (char)96, (char)206, (char)13, (char)29, (char)198, (char)129, (char)61, (char)78, (char)171, (char)125, (char)232, (char)89, (char)240, (char)110, (char)60, (char)159, (char)203, (char)17, (char)246, (char)131, (char)247, (char)17, (char)87, (char)55, (char)203, (char)23, (char)131, (char)140, (char)77, (char)60, (char)1, (char)19, (char)176, (char)30, (char)166, (char)105, (char)223, (char)71, (char)15, (char)17, (char)97, (char)155, (char)84, (char)119, (char)87, (char)86, (char)173, (char)107, (char)188, (char)58, (char)177, (char)237, (char)240, (char)128, (char)104, (char)144, (char)125, (char)65, (char)158, (char)252, (char)255, (char)223, (char)133, (char)50, (char)184, (char)129, (char)231, (char)242, (char)241, (char)132, (char)104, (char)76, (char)121, (char)70, (char)56, (char)179, (char)203, (char)65, (char)158, (char)229, (char)61, (char)214, (char)5, (char)159, (char)108, (char)7, (char)179, (char)97, (char)20, (char)207, (char)163, (char)82, (char)240, (char)56, (char)228, (char)238, (char)24, (char)54, (char)245, (char)56, (char)77, (char)23, (char)41, (char)197, (char)222, (char)237, (char)52, (char)74, (char)33, (char)183, (char)158, (char)253, (char)64, (char)84, (char)83, (char)166, (char)43, (char)127, (char)239, (char)109, (char)251, (char)40, (char)12, (char)100, (char)154, (char)28, (char)89, (char)77, (char)12, (char)53, (char)214, (char)15, (char)161, (char)118, (char)191, (char)1, (char)50, (char)111, (char)68, (char)3, (char)204, (char)239, (char)173, (char)76, (char)162, (char)108, (char)248, (char)42, (char)205, (char)77, (char)191, (char)133, (char)155, (char)172, (char)110, (char)20, (char)209, (char)252, (char)215, (char)78, (char)33, (char)125, (char)19, (char)71, (char)72, (char)53, (char)9, (char)95, (char)187, (char)44, (char)33, (char)228, (char)77, (char)34, (char)140, (char)232, (char)78, (char)179, (char)11, (char)208, (char)32, (char)38, (char)76, (char)135, (char)250, (char)112, (char)57, (char)114, (char)107, (char)48, (char)218, (char)186, (char)162, (char)96, (char)187, (char)154, (char)95, (char)88, (char)188, (char)66, (char)71, (char)243, (char)211, (char)184, (char)77, (char)92, (char)145, (char)5, (char)144, (char)103, (char)224, (char)81, (char)31, (char)113, (char)61, (char)98, (char)150, (char)17, (char)26, (char)125, (char)245, (char)134, (char)221, (char)218, (char)220, (char)238, (char)14}, 0) ;
        p267.target_component_SET((char)102) ;
        p267.length_SET((char)253) ;
        p267.sequence_SET((char)16531) ;
        p267.target_system_SET((char)156) ;
        p267.first_message_offset_SET((char)195) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)44);
            assert(pack.sequence_GET() == (char)52365);
            assert(pack.target_system_GET() == (char)85);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_component_SET((char)44) ;
        p268.sequence_SET((char)52365) ;
        p268.target_system_SET((char)85) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.uri_LEN(ph) == 118);
            assert(pack.uri_TRY(ph).equals("HmZctfxliavudgeqoceQpnpwtohzvoourunhduxshdfkymymaGzsjjvlhdkbsqdvhbxhyqzffzoiBfpNldaSqwehuabmmeNcgubmnnrlbtyliZlicllsda"));
            assert(pack.resolution_v_GET() == (char)58607);
            assert(pack.bitrate_GET() == 3480989591L);
            assert(pack.resolution_h_GET() == (char)51721);
            assert(pack.status_GET() == (char)237);
            assert(pack.framerate_GET() == -3.139187E38F);
            assert(pack.camera_id_GET() == (char)41);
            assert(pack.rotation_GET() == (char)56831);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.resolution_h_SET((char)51721) ;
        p269.bitrate_SET(3480989591L) ;
        p269.framerate_SET(-3.139187E38F) ;
        p269.resolution_v_SET((char)58607) ;
        p269.uri_SET("HmZctfxliavudgeqoceQpnpwtohzvoourunhduxshdfkymymaGzsjjvlhdkbsqdvhbxhyqzffzoiBfpNldaSqwehuabmmeNcgubmnnrlbtyliZlicllsda", PH) ;
        p269.status_SET((char)237) ;
        p269.rotation_SET((char)56831) ;
        p269.camera_id_SET((char)41) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.framerate_GET() == -8.764291E37F);
            assert(pack.resolution_v_GET() == (char)65443);
            assert(pack.camera_id_GET() == (char)136);
            assert(pack.target_component_GET() == (char)141);
            assert(pack.resolution_h_GET() == (char)38578);
            assert(pack.bitrate_GET() == 65014516L);
            assert(pack.target_system_GET() == (char)222);
            assert(pack.uri_LEN(ph) == 205);
            assert(pack.uri_TRY(ph).equals("voegekaufmqphfblAitecrtemzrmzbfrxXkcAkvmwMdyertfusdyIjqvaexljlnvqwFotWRztqznqKvrqmgrdqnmansplqpmuSzfasvulcWbxdahkuqxsslMiotxzmcTaxRkQdybksamxtmscjpvjPunaOkjpbjtwzypdFnuxwmmarmjfnucvzejijylvcgoUefbVjhqkeqse"));
            assert(pack.rotation_GET() == (char)43425);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)222) ;
        p270.resolution_v_SET((char)65443) ;
        p270.framerate_SET(-8.764291E37F) ;
        p270.uri_SET("voegekaufmqphfblAitecrtemzrmzbfrxXkcAkvmwMdyertfusdyIjqvaexljlnvqwFotWRztqznqKvrqmgrdqnmansplqpmuSzfasvulcWbxdahkuqxsslMiotxzmcTaxRkQdybksamxtmscjpvjPunaOkjpbjtwzypdFnuxwmmarmjfnucvzejijylvcgoUefbVjhqkeqse", PH) ;
        p270.rotation_SET((char)43425) ;
        p270.resolution_h_SET((char)38578) ;
        p270.target_component_SET((char)141) ;
        p270.camera_id_SET((char)136) ;
        p270.bitrate_SET(65014516L) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.password_LEN(ph) == 5);
            assert(pack.password_TRY(ph).equals("zmkob"));
            assert(pack.ssid_LEN(ph) == 10);
            assert(pack.ssid_TRY(ph).equals("mzwvjavHdq"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("mzwvjavHdq", PH) ;
        p299.password_SET("zmkob", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.min_version_GET() == (char)41495);
            assert(pack.version_GET() == (char)18738);
            assert(pack.max_version_GET() == (char)8589);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)91, (char)85, (char)37, (char)70, (char)56, (char)246, (char)120, (char)187}));
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)10, (char)13, (char)134, (char)148, (char)34, (char)62, (char)185, (char)71}));
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.spec_version_hash_SET(new char[] {(char)91, (char)85, (char)37, (char)70, (char)56, (char)246, (char)120, (char)187}, 0) ;
        p300.library_version_hash_SET(new char[] {(char)10, (char)13, (char)134, (char)148, (char)34, (char)62, (char)185, (char)71}, 0) ;
        p300.version_SET((char)18738) ;
        p300.max_version_SET((char)8589) ;
        p300.min_version_SET((char)41495) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.vendor_specific_status_code_GET() == (char)50278);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
            assert(pack.sub_mode_GET() == (char)130);
            assert(pack.time_usec_GET() == 5980315791714396499L);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
            assert(pack.uptime_sec_GET() == 2319362793L);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE) ;
        p310.vendor_specific_status_code_SET((char)50278) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR) ;
        p310.uptime_sec_SET(2319362793L) ;
        p310.time_usec_SET(5980315791714396499L) ;
        p310.sub_mode_SET((char)130) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.sw_version_minor_GET() == (char)246);
            assert(pack.time_usec_GET() == 2698039687675401483L);
            assert(pack.sw_vcs_commit_GET() == 2224446917L);
            assert(pack.hw_version_minor_GET() == (char)105);
            assert(pack.name_LEN(ph) == 22);
            assert(pack.name_TRY(ph).equals("vfxlFsxmlZlilpwshchjid"));
            assert(pack.hw_version_major_GET() == (char)114);
            assert(pack.uptime_sec_GET() == 3602628301L);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)255, (char)211, (char)68, (char)75, (char)78, (char)180, (char)57, (char)127, (char)121, (char)113, (char)238, (char)49, (char)19, (char)108, (char)151, (char)180}));
            assert(pack.sw_version_major_GET() == (char)254);
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.uptime_sec_SET(3602628301L) ;
        p311.name_SET("vfxlFsxmlZlilpwshchjid", PH) ;
        p311.sw_version_minor_SET((char)246) ;
        p311.time_usec_SET(2698039687675401483L) ;
        p311.sw_vcs_commit_SET(2224446917L) ;
        p311.sw_version_major_SET((char)254) ;
        p311.hw_unique_id_SET(new char[] {(char)255, (char)211, (char)68, (char)75, (char)78, (char)180, (char)57, (char)127, (char)121, (char)113, (char)238, (char)49, (char)19, (char)108, (char)151, (char)180}, 0) ;
        p311.hw_version_major_SET((char)114) ;
        p311.hw_version_minor_SET((char)105) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)102);
            assert(pack.param_index_GET() == (short) -21725);
            assert(pack.target_system_GET() == (char)128);
            assert(pack.param_id_LEN(ph) == 16);
            assert(pack.param_id_TRY(ph).equals("yicsefkqamwxzbNc"));
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.param_id_SET("yicsefkqamwxzbNc", PH) ;
        p320.target_system_SET((char)128) ;
        p320.param_index_SET((short) -21725) ;
        p320.target_component_SET((char)102) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)79);
            assert(pack.target_component_GET() == (char)110);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)110) ;
        p321.target_system_SET((char)79) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("obfk"));
            assert(pack.param_index_GET() == (char)53332);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
            assert(pack.param_count_GET() == (char)10239);
            assert(pack.param_value_LEN(ph) == 92);
            assert(pack.param_value_TRY(ph).equals("uckPxsssnZXoluknAkagnourpisbuxiPyvqrlwkAYpmkkjllfmamoOhkpVmdtMkjCjetytbhucsycfvMzawtkhtvfxPl"));
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_index_SET((char)53332) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16) ;
        p322.param_count_SET((char)10239) ;
        p322.param_id_SET("obfk", PH) ;
        p322.param_value_SET("uckPxsssnZXoluknAkagnourpisbuxiPyvqrlwkAYpmkkjllfmamoOhkpVmdtMkjCjetytbhucsycfvMzawtkhtvfxPl", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 22);
            assert(pack.param_value_TRY(ph).equals("zlyOjkaxhGnWoyeaqrkbyE"));
            assert(pack.param_id_LEN(ph) == 3);
            assert(pack.param_id_TRY(ph).equals("wic"));
            assert(pack.target_system_GET() == (char)176);
            assert(pack.target_component_GET() == (char)205);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_id_SET("wic", PH) ;
        p323.target_system_SET((char)176) ;
        p323.param_value_SET("zlyOjkaxhGnWoyeaqrkbyE", PH) ;
        p323.target_component_SET((char)205) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
            assert(pack.param_value_LEN(ph) == 17);
            assert(pack.param_value_TRY(ph).equals("bmusOpebrjjnpufzb"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
            assert(pack.param_id_LEN(ph) == 10);
            assert(pack.param_id_TRY(ph).equals("kgwmbmimao"));
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_value_SET("bmusOpebrjjnpufzb", PH) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED) ;
        p324.param_id_SET("kgwmbmimao", PH) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)42534, (char)57446, (char)29933, (char)62505, (char)7683, (char)1236, (char)40827, (char)65415, (char)50774, (char)61222, (char)39321, (char)12167, (char)13944, (char)61916, (char)3275, (char)13676, (char)5744, (char)52763, (char)7154, (char)26593, (char)20797, (char)40620, (char)14102, (char)36738, (char)4397, (char)19684, (char)47123, (char)47295, (char)15368, (char)13803, (char)15484, (char)45757, (char)40248, (char)39176, (char)54071, (char)2047, (char)39706, (char)58491, (char)29688, (char)31381, (char)19367, (char)49911, (char)41581, (char)61759, (char)4454, (char)24484, (char)31302, (char)56783, (char)39105, (char)39457, (char)62371, (char)27797, (char)1527, (char)23203, (char)53669, (char)56994, (char)51523, (char)7263, (char)20248, (char)29047, (char)30750, (char)32312, (char)5045, (char)30529, (char)12221, (char)47320, (char)20581, (char)27484, (char)52211, (char)13014, (char)50587, (char)2621}));
            assert(pack.min_distance_GET() == (char)771);
            assert(pack.time_usec_GET() == 8390907409280688697L);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            assert(pack.increment_GET() == (char)63);
            assert(pack.max_distance_GET() == (char)34010);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.increment_SET((char)63) ;
        p330.time_usec_SET(8390907409280688697L) ;
        p330.max_distance_SET((char)34010) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR) ;
        p330.distances_SET(new char[] {(char)42534, (char)57446, (char)29933, (char)62505, (char)7683, (char)1236, (char)40827, (char)65415, (char)50774, (char)61222, (char)39321, (char)12167, (char)13944, (char)61916, (char)3275, (char)13676, (char)5744, (char)52763, (char)7154, (char)26593, (char)20797, (char)40620, (char)14102, (char)36738, (char)4397, (char)19684, (char)47123, (char)47295, (char)15368, (char)13803, (char)15484, (char)45757, (char)40248, (char)39176, (char)54071, (char)2047, (char)39706, (char)58491, (char)29688, (char)31381, (char)19367, (char)49911, (char)41581, (char)61759, (char)4454, (char)24484, (char)31302, (char)56783, (char)39105, (char)39457, (char)62371, (char)27797, (char)1527, (char)23203, (char)53669, (char)56994, (char)51523, (char)7263, (char)20248, (char)29047, (char)30750, (char)32312, (char)5045, (char)30529, (char)12221, (char)47320, (char)20581, (char)27484, (char)52211, (char)13014, (char)50587, (char)2621}, 0) ;
        p330.min_distance_SET((char)771) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVIONIX_ADSB_OUT_CFG.add((src, ph, pack) ->
        {
            assert(pack.callsign_LEN(ph) == 2);
            assert(pack.callsign_TRY(ph).equals("of"));
            assert(pack.stallSpeed_GET() == (char)20264);
            assert(pack.ICAO_GET() == 28662145L);
            assert(pack.rfSelect_GET() == UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED);
            assert(pack.gpsOffsetLat_GET() == UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_0M);
            assert(pack.gpsOffsetLon_GET() == UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR);
            assert(pack.emitterType_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT);
            assert(pack.aircraftSize_GET() == UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_45M);
        });
        GroundControl.UAVIONIX_ADSB_OUT_CFG p10001 = CommunicationChannel.new_UAVIONIX_ADSB_OUT_CFG();
        PH.setPack(p10001);
        p10001.gpsOffsetLat_SET(UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_0M) ;
        p10001.rfSelect_SET(UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED) ;
        p10001.aircraftSize_SET(UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_45M) ;
        p10001.emitterType_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT) ;
        p10001.stallSpeed_SET((char)20264) ;
        p10001.ICAO_SET(28662145L) ;
        p10001.gpsOffsetLon_SET(UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR) ;
        p10001.callsign_SET("of", PH) ;
        CommunicationChannel.instance.send(p10001);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVIONIX_ADSB_OUT_DYNAMIC.add((src, ph, pack) ->
        {
            assert(pack.gpsLon_GET() == 1193224166);
            assert(pack.gpsFix_GET() == UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_RTK);
            assert(pack.accuracyVel_GET() == (char)47881);
            assert(pack.baroAltMSL_GET() == 642873259);
            assert(pack.gpsAlt_GET() == 1743331528);
            assert(pack.accuracyHor_GET() == 4040091908L);
            assert(pack.emergencyStatus_GET() == UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_DOWNED_AIRCRAFT_EMERGENCY);
            assert(pack.velVert_GET() == (short)3437);
            assert(pack.utcTime_GET() == 1899287684L);
            assert(pack.state_GET() == (UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND |
                                        UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE));
            assert(pack.gpsLat_GET() == 1452135445);
            assert(pack.numSats_GET() == (char)190);
            assert(pack.squawk_GET() == (char)53957);
            assert(pack.VelEW_GET() == (short)19458);
            assert(pack.accuracyVert_GET() == (char)53651);
            assert(pack.velNS_GET() == (short) -4844);
        });
        GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC p10002 = CommunicationChannel.new_UAVIONIX_ADSB_OUT_DYNAMIC();
        PH.setPack(p10002);
        p10002.gpsFix_SET(UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_RTK) ;
        p10002.utcTime_SET(1899287684L) ;
        p10002.gpsLon_SET(1193224166) ;
        p10002.accuracyHor_SET(4040091908L) ;
        p10002.gpsLat_SET(1452135445) ;
        p10002.state_SET((UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND |
                          UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE)) ;
        p10002.baroAltMSL_SET(642873259) ;
        p10002.accuracyVert_SET((char)53651) ;
        p10002.velVert_SET((short)3437) ;
        p10002.accuracyVel_SET((char)47881) ;
        p10002.VelEW_SET((short)19458) ;
        p10002.squawk_SET((char)53957) ;
        p10002.velNS_SET((short) -4844) ;
        p10002.gpsAlt_SET(1743331528) ;
        p10002.numSats_SET((char)190) ;
        p10002.emergencyStatus_SET(UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_DOWNED_AIRCRAFT_EMERGENCY) ;
        CommunicationChannel.instance.send(p10002);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT.add((src, ph, pack) ->
        {
            assert(pack.rfHealth_GET() == UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_OK);
        });
        GroundControl.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT p10003 = CommunicationChannel.new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();
        PH.setPack(p10003);
        p10003.rfHealth_SET(UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_OK) ;
        CommunicationChannel.instance.send(p10003);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEVICE_OP_READ.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)181);
            assert(pack.busname_LEN(ph) == 22);
            assert(pack.busname_TRY(ph).equals("mFqrkqfutvlncuerpxpsqE"));
            assert(pack.regstart_GET() == (char)122);
            assert(pack.target_component_GET() == (char)46);
            assert(pack.bustype_GET() == DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C);
            assert(pack.target_system_GET() == (char)68);
            assert(pack.request_id_GET() == 3957648371L);
            assert(pack.address_GET() == (char)237);
            assert(pack.bus_GET() == (char)192);
        });
        GroundControl.DEVICE_OP_READ p11000 = CommunicationChannel.new_DEVICE_OP_READ();
        PH.setPack(p11000);
        p11000.target_system_SET((char)68) ;
        p11000.address_SET((char)237) ;
        p11000.bustype_SET(DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C) ;
        p11000.target_component_SET((char)46) ;
        p11000.regstart_SET((char)122) ;
        p11000.busname_SET("mFqrkqfutvlncuerpxpsqE", PH) ;
        p11000.request_id_SET(3957648371L) ;
        p11000.count_SET((char)181) ;
        p11000.bus_SET((char)192) ;
        CommunicationChannel.instance.send(p11000);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEVICE_OP_READ_REPLY.add((src, ph, pack) ->
        {
            assert(pack.result_GET() == (char)148);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)140, (char)208, (char)92, (char)207, (char)226, (char)179, (char)104, (char)111, (char)139, (char)128, (char)187, (char)226, (char)211, (char)41, (char)245, (char)98, (char)146, (char)161, (char)246, (char)104, (char)206, (char)62, (char)110, (char)77, (char)186, (char)15, (char)255, (char)51, (char)131, (char)194, (char)236, (char)253, (char)156, (char)136, (char)5, (char)220, (char)147, (char)90, (char)53, (char)49, (char)86, (char)90, (char)228, (char)246, (char)97, (char)56, (char)176, (char)149, (char)38, (char)155, (char)246, (char)46, (char)136, (char)133, (char)143, (char)26, (char)118, (char)194, (char)227, (char)91, (char)118, (char)5, (char)121, (char)94, (char)11, (char)162, (char)219, (char)35, (char)15, (char)112, (char)248, (char)146, (char)65, (char)151, (char)254, (char)118, (char)150, (char)209, (char)145, (char)76, (char)82, (char)218, (char)187, (char)80, (char)3, (char)111, (char)167, (char)165, (char)62, (char)236, (char)216, (char)100, (char)166, (char)27, (char)201, (char)221, (char)237, (char)187, (char)87, (char)35, (char)226, (char)19, (char)170, (char)115, (char)44, (char)174, (char)127, (char)58, (char)196, (char)252, (char)111, (char)16, (char)169, (char)95, (char)252, (char)104, (char)57, (char)73, (char)150, (char)97, (char)97, (char)223, (char)19, (char)254, (char)34, (char)116, (char)239, (char)252}));
            assert(pack.regstart_GET() == (char)196);
            assert(pack.count_GET() == (char)8);
            assert(pack.request_id_GET() == 2736095860L);
        });
        GroundControl.DEVICE_OP_READ_REPLY p11001 = CommunicationChannel.new_DEVICE_OP_READ_REPLY();
        PH.setPack(p11001);
        p11001.data__SET(new char[] {(char)140, (char)208, (char)92, (char)207, (char)226, (char)179, (char)104, (char)111, (char)139, (char)128, (char)187, (char)226, (char)211, (char)41, (char)245, (char)98, (char)146, (char)161, (char)246, (char)104, (char)206, (char)62, (char)110, (char)77, (char)186, (char)15, (char)255, (char)51, (char)131, (char)194, (char)236, (char)253, (char)156, (char)136, (char)5, (char)220, (char)147, (char)90, (char)53, (char)49, (char)86, (char)90, (char)228, (char)246, (char)97, (char)56, (char)176, (char)149, (char)38, (char)155, (char)246, (char)46, (char)136, (char)133, (char)143, (char)26, (char)118, (char)194, (char)227, (char)91, (char)118, (char)5, (char)121, (char)94, (char)11, (char)162, (char)219, (char)35, (char)15, (char)112, (char)248, (char)146, (char)65, (char)151, (char)254, (char)118, (char)150, (char)209, (char)145, (char)76, (char)82, (char)218, (char)187, (char)80, (char)3, (char)111, (char)167, (char)165, (char)62, (char)236, (char)216, (char)100, (char)166, (char)27, (char)201, (char)221, (char)237, (char)187, (char)87, (char)35, (char)226, (char)19, (char)170, (char)115, (char)44, (char)174, (char)127, (char)58, (char)196, (char)252, (char)111, (char)16, (char)169, (char)95, (char)252, (char)104, (char)57, (char)73, (char)150, (char)97, (char)97, (char)223, (char)19, (char)254, (char)34, (char)116, (char)239, (char)252}, 0) ;
        p11001.count_SET((char)8) ;
        p11001.request_id_SET(2736095860L) ;
        p11001.result_SET((char)148) ;
        p11001.regstart_SET((char)196) ;
        CommunicationChannel.instance.send(p11001);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEVICE_OP_WRITE.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)19);
            assert(pack.address_GET() == (char)217);
            assert(pack.target_component_GET() == (char)105);
            assert(pack.busname_LEN(ph) == 38);
            assert(pack.busname_TRY(ph).equals("spqcjorzmpyaopmnqRdnbnzAoYtfnsoCgbSvxa"));
            assert(pack.bustype_GET() == DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_SPI);
            assert(pack.target_system_GET() == (char)58);
            assert(pack.bus_GET() == (char)78);
            assert(pack.request_id_GET() == 3917639416L);
            assert(pack.regstart_GET() == (char)241);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)224, (char)213, (char)78, (char)180, (char)55, (char)25, (char)227, (char)116, (char)184, (char)182, (char)213, (char)11, (char)43, (char)28, (char)66, (char)196, (char)181, (char)53, (char)95, (char)225, (char)14, (char)252, (char)44, (char)195, (char)57, (char)220, (char)133, (char)2, (char)160, (char)67, (char)117, (char)150, (char)213, (char)71, (char)7, (char)221, (char)206, (char)76, (char)52, (char)64, (char)176, (char)91, (char)233, (char)222, (char)196, (char)177, (char)206, (char)105, (char)89, (char)52, (char)183, (char)205, (char)109, (char)234, (char)19, (char)116, (char)72, (char)17, (char)32, (char)196, (char)227, (char)130, (char)175, (char)75, (char)197, (char)208, (char)200, (char)44, (char)166, (char)203, (char)63, (char)26, (char)33, (char)6, (char)204, (char)73, (char)150, (char)2, (char)173, (char)124, (char)0, (char)179, (char)213, (char)143, (char)16, (char)255, (char)227, (char)45, (char)216, (char)26, (char)209, (char)42, (char)151, (char)0, (char)211, (char)222, (char)97, (char)78, (char)188, (char)126, (char)164, (char)43, (char)25, (char)75, (char)1, (char)243, (char)175, (char)138, (char)183, (char)28, (char)220, (char)123, (char)59, (char)149, (char)96, (char)89, (char)1, (char)189, (char)106, (char)90, (char)230, (char)229, (char)14, (char)42, (char)120, (char)65, (char)35, (char)112}));
        });
        GroundControl.DEVICE_OP_WRITE p11002 = CommunicationChannel.new_DEVICE_OP_WRITE();
        PH.setPack(p11002);
        p11002.target_component_SET((char)105) ;
        p11002.address_SET((char)217) ;
        p11002.count_SET((char)19) ;
        p11002.regstart_SET((char)241) ;
        p11002.data__SET(new char[] {(char)224, (char)213, (char)78, (char)180, (char)55, (char)25, (char)227, (char)116, (char)184, (char)182, (char)213, (char)11, (char)43, (char)28, (char)66, (char)196, (char)181, (char)53, (char)95, (char)225, (char)14, (char)252, (char)44, (char)195, (char)57, (char)220, (char)133, (char)2, (char)160, (char)67, (char)117, (char)150, (char)213, (char)71, (char)7, (char)221, (char)206, (char)76, (char)52, (char)64, (char)176, (char)91, (char)233, (char)222, (char)196, (char)177, (char)206, (char)105, (char)89, (char)52, (char)183, (char)205, (char)109, (char)234, (char)19, (char)116, (char)72, (char)17, (char)32, (char)196, (char)227, (char)130, (char)175, (char)75, (char)197, (char)208, (char)200, (char)44, (char)166, (char)203, (char)63, (char)26, (char)33, (char)6, (char)204, (char)73, (char)150, (char)2, (char)173, (char)124, (char)0, (char)179, (char)213, (char)143, (char)16, (char)255, (char)227, (char)45, (char)216, (char)26, (char)209, (char)42, (char)151, (char)0, (char)211, (char)222, (char)97, (char)78, (char)188, (char)126, (char)164, (char)43, (char)25, (char)75, (char)1, (char)243, (char)175, (char)138, (char)183, (char)28, (char)220, (char)123, (char)59, (char)149, (char)96, (char)89, (char)1, (char)189, (char)106, (char)90, (char)230, (char)229, (char)14, (char)42, (char)120, (char)65, (char)35, (char)112}, 0) ;
        p11002.bus_SET((char)78) ;
        p11002.bustype_SET(DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_SPI) ;
        p11002.target_system_SET((char)58) ;
        p11002.busname_SET("spqcjorzmpyaopmnqRdnbnzAoYtfnsoCgbSvxa", PH) ;
        p11002.request_id_SET(3917639416L) ;
        CommunicationChannel.instance.send(p11002);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEVICE_OP_WRITE_REPLY.add((src, ph, pack) ->
        {
            assert(pack.request_id_GET() == 2851929699L);
            assert(pack.result_GET() == (char)8);
        });
        GroundControl.DEVICE_OP_WRITE_REPLY p11003 = CommunicationChannel.new_DEVICE_OP_WRITE_REPLY();
        PH.setPack(p11003);
        p11003.request_id_SET(2851929699L) ;
        p11003.result_SET((char)8) ;
        CommunicationChannel.instance.send(p11003);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ADAP_TUNING.add((src, ph, pack) ->
        {
            assert(pack.omega_dot_GET() == -3.9128257E37F);
            assert(pack.theta_dot_GET() == -9.203217E37F);
            assert(pack.sigma_GET() == -5.699726E37F);
            assert(pack.achieved_GET() == -2.5913877E38F);
            assert(pack.f_GET() == 2.1012426E38F);
            assert(pack.sigma_dot_GET() == 5.979414E37F);
            assert(pack.axis_GET() == PID_TUNING_AXIS.PID_TUNING_ACCZ);
            assert(pack.theta_GET() == 2.9733572E38F);
            assert(pack.error_GET() == 9.213129E37F);
            assert(pack.desired_GET() == -2.8759818E38F);
            assert(pack.u_GET() == -1.3667662E37F);
            assert(pack.omega_GET() == -9.691915E37F);
            assert(pack.f_dot_GET() == 3.1058256E37F);
        });
        GroundControl.ADAP_TUNING p11010 = CommunicationChannel.new_ADAP_TUNING();
        PH.setPack(p11010);
        p11010.sigma_dot_SET(5.979414E37F) ;
        p11010.theta_SET(2.9733572E38F) ;
        p11010.omega_dot_SET(-3.9128257E37F) ;
        p11010.theta_dot_SET(-9.203217E37F) ;
        p11010.error_SET(9.213129E37F) ;
        p11010.f_SET(2.1012426E38F) ;
        p11010.u_SET(-1.3667662E37F) ;
        p11010.achieved_SET(-2.5913877E38F) ;
        p11010.desired_SET(-2.8759818E38F) ;
        p11010.sigma_SET(-5.699726E37F) ;
        p11010.omega_SET(-9.691915E37F) ;
        p11010.axis_SET(PID_TUNING_AXIS.PID_TUNING_ACCZ) ;
        p11010.f_dot_SET(3.1058256E37F) ;
        CommunicationChannel.instance.send(p11010);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VISION_POSITION_DELTA.add((src, ph, pack) ->
        {
            assert(pack.time_delta_usec_GET() == 2800985710286651704L);
            assert(pack.confidence_GET() == 1.9567223E38F);
            assert(pack.time_usec_GET() == 1690573179198728977L);
            assert(Arrays.equals(pack.angle_delta_GET(),  new float[] {3.205397E38F, -7.603547E37F, -2.1363712E38F}));
            assert(Arrays.equals(pack.position_delta_GET(),  new float[] {-3.2735645E38F, -2.8614499E38F, 9.535006E37F}));
        });
        GroundControl.VISION_POSITION_DELTA p11011 = CommunicationChannel.new_VISION_POSITION_DELTA();
        PH.setPack(p11011);
        p11011.position_delta_SET(new float[] {-3.2735645E38F, -2.8614499E38F, 9.535006E37F}, 0) ;
        p11011.time_usec_SET(1690573179198728977L) ;
        p11011.time_delta_usec_SET(2800985710286651704L) ;
        p11011.angle_delta_SET(new float[] {3.205397E38F, -7.603547E37F, -2.1363712E38F}, 0) ;
        p11011.confidence_SET(1.9567223E38F) ;
        CommunicationChannel.instance.send(p11011);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
    }

}
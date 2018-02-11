
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
        *	 (packets that were corrupted on reception on the MAV*/
        public void drop_rate_comm_SET(char  src)
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        /**
        *Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted
        *	 on reception on the MAV*/
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
        *	 present. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
        public void onboard_control_sensors_present_SET(@MAV_SYS_STATUS_SENSOR int  src)
        {  set_bits(- 1 +   src, 26, data, 152); }
        /**
        *Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of
        *	 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
        public void onboard_control_sensors_enabled_SET(@MAV_SYS_STATUS_SENSOR int  src)
        {  set_bits(- 1 +   src, 26, data, 178); }
        /**
        *Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not
        *	 enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
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
        *	 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
        *	 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
        *	 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
        *	 bit 11: yaw, bit 12: yaw rat*/
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
        *	 =*/
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
        *	 the system id of the requesting syste*/
        public void target_system_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  12); }
        /**
        *0: request ping from all receiving components, if greater than 0: message is a ping response and number
        *	 is the system id of the requesting syste*/
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
        *	 the safest mode possible initially and then gradually move down the encryption level if it gets a NACK
        *	 message indicating an encryption mismatch*/
        public void version_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        /**
        *Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
        *	 characters may involve A-Z, a-z, 0-9, and "!?,.-*/
        public void passkey_SET(String src, Bounds.Inside ph)
        {passkey_SET(src.toCharArray(), 0, src.length(), ph);}/**
*Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
*	 characters may involve A-Z, a-z, 0-9, and "!?,.-*/
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
        *	 contro*/
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
        *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
        *	 storage if the ID is stored as strin*/
        public void param_id_SET(String src, Bounds.Inside ph)
        {param_id_SET(src.toCharArray(), 0, src.length(), ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	 storage if the ID is stored as strin*/
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
        *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
        *	 storage if the ID is stored as strin*/
        public void param_id_SET(String src, Bounds.Inside ph)
        {param_id_SET(src.toCharArray(), 0, src.length(), ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	 storage if the ID is stored as strin*/
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
        *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
        *	 storage if the ID is stored as strin*/
        public void param_id_SET(String src, Bounds.Inside ph)
        {param_id_SET(src.toCharArray(), 0, src.length(), ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	 storage if the ID is stored as strin*/
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
        *	 unknown, set to: UINT16_MA*/
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
        *	 the AMSL altitude in addition to the WGS84 altitude*/
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
        *	 provide the AMSL as well*/
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
        *	 8 servos*/
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
        *	 8 servos*/
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
        *	 more than 8 servos*/
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
        *	 send -2 to disable any existing map for this rc_channel_index*/
        public void param_index_SET(short  src)
        {  set_bytes((short)(src) & -1L, 2, data,  2); }
        /**
        *Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob
        *	 on the RC*/
        public void parameter_rc_channel_index_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public void param_value0_SET(float  src) //Initial parameter value
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 5); }
        public void scale_SET(float  src) //Scale, maps the RC range [-1, 1] to a parameter value
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 9); }
        /**
        *Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends
        *	 on implementation*/
        public void param_value_min_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 13); }
        /**
        *Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends
        *	 on implementation*/
        public void param_value_max_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 17); }
        /**
        *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
        *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
        *	 storage if the ID is stored as strin*/
        public void param_id_SET(String src, Bounds.Inside ph)
        {param_id_SET(src.toCharArray(), 0, src.length(), ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	 storage if the ID is stored as strin*/
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
        *	 with Z axis up or local, right handed, Z axis down*/
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
        *	 with Z axis up or local, right handed, Z axis down*/
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
        *	 the second row, etc.*/
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
        *	 are available but not given in this message. This value should be 0 when no RC channels are available*/
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
        *	 bit corresponds to Button 1*/
        public void buttons_SET(char  src)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void target_SET(char  src) //The system to be controlled.
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        /**
        *X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
        *	 Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle*/
        public void x_SET(short  src)
        {  set_bytes((short)(src) & -1L, 2, data,  3); }
        /**
        *Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
        *	 Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle*/
        public void y_SET(short  src)
        {  set_bytes((short)(src) & -1L, 2, data,  5); }
        /**
        *Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
        *	 Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on
        *	 a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative
        *	 thrust*/
        public void z_SET(short  src)
        {  set_bytes((short)(src) & -1L, 2, data,  7); }
        /**
        *R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
        *	 Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise
        *	 being -1000, and the yaw of a vehicle*/
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
        *	 sequence (0,1,2,3,4)*/
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
        *	 was denied or the progress percentage or 255 if unknown the progress when result is MAV_RESULT_IN_PROGRESS*/
        public void progress_SET(char  src, Bounds.Inside ph)
        {
            if(ph.field_bit != 11)insert_field(ph, 11, 0);
            set_bytes((char)(src) & -1L, 1, data,  ph.BYTE);
        }/**
*WIP: Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to
*	 be denied*/
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
        *	 bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitud*/
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
        *	 bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitud*/
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
        *	 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
        *	 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
        *	 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
        *	 bit 11: yaw, bit 12: yaw rat*/
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
        *	 =*/
        public void coordinate_frame_SET(@MAV_FRAME int  src)
        {  set_bits(- 0 +   src, 4, data, 416); }
    }
    public static class SET_POSITION_TARGET_GLOBAL_INT extends GroundControl.SET_POSITION_TARGET_GLOBAL_INT
    {
        /**
        *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
        *	 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
        *	 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
        *	 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
        *	 bit 11: yaw, bit 12: yaw rat*/
        public void type_mask_SET(char  src)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        /**
        *Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
        *	 the system to compensate for the transport delay of the setpoint. This allows the system to compensate
        *	 processing latency*/
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
        *	 = 1*/
        public void coordinate_frame_SET(@MAV_FRAME int  src)
        {  set_bits(- 0 +   src, 4, data, 416); }
    }
    public static class POSITION_TARGET_GLOBAL_INT extends GroundControl.POSITION_TARGET_GLOBAL_INT
    {
        /**
        *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
        *	 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
        *	 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
        *	 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
        *	 bit 11: yaw, bit 12: yaw rat*/
        public void type_mask_SET(char  src)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        /**
        *Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
        *	 the system to compensate for the transport delay of the setpoint. This allows the system to compensate
        *	 processing latency*/
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
        *	 = 1*/
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
        *	 just onc*/
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
        *	 just onc*/
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
        *	 no CCB*/
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
        *	 lost (set to 255 if no start exists)*/
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
        *	 lost (set to 255 if no start exists)*/
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
        *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	 ID is stored as strin*/
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	 ID is stored as strin*/
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
        *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	 ID is stored as strin*/
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  38 && !try_visit_field(ph, 38)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	 ID is stored as strin*/
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
        *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	 ID is stored as strin*/
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  22 && !try_visit_field(ph, 22)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	 ID is stored as strin*/
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
        *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	 ID is stored as strin*/
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  8 && !try_visit_field(ph, 8)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	 ID is stored as strin*/
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
        *	 is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
        *	 for unknown/not used. In a array element, each unit corresponds to 1cm*/
        public char[] distances_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 0, dst_max = pos + 72; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        /**
        *Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
        *	 is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
        *	 for unknown/not used. In a array element, each unit corresponds to 1cm*/
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
        *	 one] (table 2-37 DO-282B*/
        public @UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON int gpsOffsetLon_GET()
        {  return  0 + (int)get_bits(data, 60, 1); }
        public @UAVIONIX_ADSB_OUT_RF_SELECT int rfSelect_GET()//ADS-B transponder reciever and transmit enable flags
        {  return  0 + (int)get_bits(data, 61, 3); }
        public String callsign_TRY(Bounds.Inside ph)//Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
        {
            if(ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) return null;
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
            return (ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
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
        *	 (m * 1E-3). (up +ve). If unknown set to INT32_MA*/
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
        {  return  0 + (int)get_bits(data, 0, 6); }
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
*	 2=down*/
        public float[] position_delta_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 28, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        /**
        *Change in position in meters from previous to current frame rotated into body frame (0=forward, 1=right,
        *	 2=down*/
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
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_CALIBRATING);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED));
            assert(pack.custom_mode_GET() == 2192501334L);
            assert(pack.mavlink_version_GET() == (char)191);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_VTOL_QUADROTOR);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_AUTOQUAD);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.custom_mode_SET(2192501334L) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_AUTOQUAD) ;
        p0.mavlink_version_SET((char)191) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_CALIBRATING) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED)) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_VTOL_QUADROTOR) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.load_GET() == (char)20722);
            assert(pack.drop_rate_comm_GET() == (char)47896);
            assert(pack.errors_count1_GET() == (char)3036);
            assert(pack.errors_count4_GET() == (char)27485);
            assert(pack.battery_remaining_GET() == (byte)44);
            assert(pack.errors_count2_GET() == (char)20715);
            assert(pack.errors_count3_GET() == (char)62419);
            assert(pack.voltage_battery_GET() == (char)28177);
            assert(pack.current_battery_GET() == (short) -17690);
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
            assert(pack.errors_comm_GET() == (char)19248);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.errors_comm_SET((char)19248) ;
        p1.voltage_battery_SET((char)28177) ;
        p1.load_SET((char)20722) ;
        p1.drop_rate_comm_SET((char)47896) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL)) ;
        p1.errors_count2_SET((char)20715) ;
        p1.errors_count4_SET((char)27485) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        p1.battery_remaining_SET((byte)44) ;
        p1.errors_count3_SET((char)62419) ;
        p1.current_battery_SET((short) -17690) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL)) ;
        p1.errors_count1_SET((char)3036) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1694546254L);
            assert(pack.time_unix_usec_GET() == 2206456394017015297L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(1694546254L) ;
        p2.time_unix_usec_SET(2206456394017015297L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 3.2918136E38F);
            assert(pack.vy_GET() == -1.8096958E38F);
            assert(pack.afz_GET() == 9.609844E37F);
            assert(pack.yaw_rate_GET() == 2.1247065E38F);
            assert(pack.afy_GET() == -3.0960224E37F);
            assert(pack.yaw_GET() == -1.2759153E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.time_boot_ms_GET() == 226414769L);
            assert(pack.vz_GET() == -1.574561E38F);
            assert(pack.afx_GET() == 3.2474046E38F);
            assert(pack.x_GET() == 1.6822917E38F);
            assert(pack.type_mask_GET() == (char)41816);
            assert(pack.z_GET() == 2.4695251E38F);
            assert(pack.vx_GET() == 1.5061007E38F);
        });
        POSITION_TARGET_LOCAL_NED p3 = new POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.yaw_SET(-1.2759153E37F) ;
        p3.afx_SET(3.2474046E38F) ;
        p3.x_SET(1.6822917E38F) ;
        p3.time_boot_ms_SET(226414769L) ;
        p3.yaw_rate_SET(2.1247065E38F) ;
        p3.afz_SET(9.609844E37F) ;
        p3.y_SET(3.2918136E38F) ;
        p3.vz_SET(-1.574561E38F) ;
        p3.afy_SET(-3.0960224E37F) ;
        p3.vx_SET(1.5061007E38F) ;
        p3.type_mask_SET((char)41816) ;
        p3.vy_SET(-1.8096958E38F) ;
        p3.z_SET(2.4695251E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        TestChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)160);
            assert(pack.target_system_GET() == (char)227);
            assert(pack.time_usec_GET() == 5157808612380393604L);
            assert(pack.seq_GET() == 217863983L);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.time_usec_SET(5157808612380393604L) ;
        p4.target_system_SET((char)227) ;
        p4.target_component_SET((char)160) ;
        p4.seq_SET(217863983L) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)159);
            assert(pack.control_request_GET() == (char)216);
            assert(pack.passkey_LEN(ph) == 22);
            assert(pack.passkey_TRY(ph).equals("jtvhpKscxWeoulnvnznedy"));
            assert(pack.version_GET() == (char)20);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.version_SET((char)20) ;
        p5.control_request_SET((char)216) ;
        p5.passkey_SET("jtvhpKscxWeoulnvnznedy", PH) ;
        p5.target_system_SET((char)159) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.control_request_GET() == (char)169);
            assert(pack.ack_GET() == (char)207);
            assert(pack.gcs_system_id_GET() == (char)17);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.gcs_system_id_SET((char)17) ;
        p6.control_request_SET((char)169) ;
        p6.ack_SET((char)207) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 14);
            assert(pack.key_TRY(ph).equals("SilznyyzbrWZfi"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("SilznyyzbrWZfi", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
            assert(pack.custom_mode_GET() == 2664810407L);
            assert(pack.target_system_GET() == (char)73);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.target_system_SET((char)73) ;
        p11.custom_mode_SET(2664810407L) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_STABILIZE_DISARMED) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)145);
            assert(pack.target_system_GET() == (char)202);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("a"));
            assert(pack.param_index_GET() == (short) -10906);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_system_SET((char)202) ;
        p20.param_index_SET((short) -10906) ;
        p20.param_id_SET("a", PH) ;
        p20.target_component_SET((char)145) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)26);
            assert(pack.target_component_GET() == (char)247);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)247) ;
        p21.target_system_SET((char)26) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)37135);
            assert(pack.param_index_GET() == (char)41141);
            assert(pack.param_value_GET() == -2.0870466E38F);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("sz"));
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8) ;
        p22.param_index_SET((char)41141) ;
        p22.param_id_SET("sz", PH) ;
        p22.param_value_SET(-2.0870466E38F) ;
        p22.param_count_SET((char)37135) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
            assert(pack.target_system_GET() == (char)124);
            assert(pack.target_component_GET() == (char)242);
            assert(pack.param_value_GET() == -8.11199E36F);
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("pkkvqdxwOjjq"));
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.target_system_SET((char)124) ;
        p23.param_value_SET(-8.11199E36F) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32) ;
        p23.param_id_SET("pkkvqdxwOjjq", PH) ;
        p23.target_component_SET((char)242) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.satellites_visible_GET() == (char)55);
            assert(pack.vel_GET() == (char)30798);
            assert(pack.eph_GET() == (char)39321);
            assert(pack.time_usec_GET() == 1482494313086304236L);
            assert(pack.cog_GET() == (char)22293);
            assert(pack.v_acc_TRY(ph) == 1806298939L);
            assert(pack.alt_GET() == 730001730);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
            assert(pack.h_acc_TRY(ph) == 1196873482L);
            assert(pack.hdg_acc_TRY(ph) == 1746397846L);
            assert(pack.lat_GET() == 1146565024);
            assert(pack.alt_ellipsoid_TRY(ph) == -1005591227);
            assert(pack.lon_GET() == -542113974);
            assert(pack.vel_acc_TRY(ph) == 3597634068L);
            assert(pack.epv_GET() == (char)54352);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.alt_ellipsoid_SET(-1005591227, PH) ;
        p24.epv_SET((char)54352) ;
        p24.v_acc_SET(1806298939L, PH) ;
        p24.alt_SET(730001730) ;
        p24.vel_acc_SET(3597634068L, PH) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP) ;
        p24.lon_SET(-542113974) ;
        p24.hdg_acc_SET(1746397846L, PH) ;
        p24.vel_SET((char)30798) ;
        p24.lat_SET(1146565024) ;
        p24.eph_SET((char)39321) ;
        p24.time_usec_SET(1482494313086304236L) ;
        p24.cog_SET((char)22293) ;
        p24.satellites_visible_SET((char)55) ;
        p24.h_acc_SET(1196873482L, PH) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)118, (char)153, (char)98, (char)140, (char)108, (char)19, (char)227, (char)73, (char)89, (char)211, (char)7, (char)163, (char)248, (char)165, (char)249, (char)170, (char)128, (char)206, (char)188, (char)5}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)63, (char)186, (char)253, (char)237, (char)242, (char)80, (char)9, (char)48, (char)208, (char)56, (char)191, (char)174, (char)71, (char)62, (char)136, (char)178, (char)176, (char)63, (char)177, (char)202}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)139, (char)126, (char)148, (char)236, (char)225, (char)111, (char)81, (char)247, (char)201, (char)61, (char)31, (char)216, (char)36, (char)86, (char)113, (char)165, (char)119, (char)234, (char)120, (char)33}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)8, (char)12, (char)149, (char)235, (char)68, (char)118, (char)120, (char)68, (char)189, (char)215, (char)112, (char)76, (char)131, (char)154, (char)136, (char)253, (char)92, (char)239, (char)117, (char)34}));
            assert(pack.satellites_visible_GET() == (char)153);
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)78, (char)102, (char)123, (char)110, (char)95, (char)215, (char)172, (char)73, (char)58, (char)154, (char)248, (char)59, (char)76, (char)6, (char)15, (char)86, (char)129, (char)39, (char)34, (char)202}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellites_visible_SET((char)153) ;
        p25.satellite_elevation_SET(new char[] {(char)139, (char)126, (char)148, (char)236, (char)225, (char)111, (char)81, (char)247, (char)201, (char)61, (char)31, (char)216, (char)36, (char)86, (char)113, (char)165, (char)119, (char)234, (char)120, (char)33}, 0) ;
        p25.satellite_used_SET(new char[] {(char)63, (char)186, (char)253, (char)237, (char)242, (char)80, (char)9, (char)48, (char)208, (char)56, (char)191, (char)174, (char)71, (char)62, (char)136, (char)178, (char)176, (char)63, (char)177, (char)202}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)78, (char)102, (char)123, (char)110, (char)95, (char)215, (char)172, (char)73, (char)58, (char)154, (char)248, (char)59, (char)76, (char)6, (char)15, (char)86, (char)129, (char)39, (char)34, (char)202}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)118, (char)153, (char)98, (char)140, (char)108, (char)19, (char)227, (char)73, (char)89, (char)211, (char)7, (char)163, (char)248, (char)165, (char)249, (char)170, (char)128, (char)206, (char)188, (char)5}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)8, (char)12, (char)149, (char)235, (char)68, (char)118, (char)120, (char)68, (char)189, (char)215, (char)112, (char)76, (char)131, (char)154, (char)136, (char)253, (char)92, (char)239, (char)117, (char)34}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.zmag_GET() == (short) -21181);
            assert(pack.zgyro_GET() == (short) -13202);
            assert(pack.yacc_GET() == (short)3831);
            assert(pack.time_boot_ms_GET() == 3396729570L);
            assert(pack.zacc_GET() == (short)32140);
            assert(pack.ymag_GET() == (short)29372);
            assert(pack.xgyro_GET() == (short) -17425);
            assert(pack.ygyro_GET() == (short)16968);
            assert(pack.xmag_GET() == (short) -9542);
            assert(pack.xacc_GET() == (short)12800);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.zmag_SET((short) -21181) ;
        p26.zgyro_SET((short) -13202) ;
        p26.yacc_SET((short)3831) ;
        p26.xgyro_SET((short) -17425) ;
        p26.xmag_SET((short) -9542) ;
        p26.ymag_SET((short)29372) ;
        p26.time_boot_ms_SET(3396729570L) ;
        p26.ygyro_SET((short)16968) ;
        p26.zacc_SET((short)32140) ;
        p26.xacc_SET((short)12800) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short)5737);
            assert(pack.xmag_GET() == (short) -26096);
            assert(pack.ymag_GET() == (short)32320);
            assert(pack.xacc_GET() == (short) -15752);
            assert(pack.xgyro_GET() == (short)25453);
            assert(pack.zacc_GET() == (short) -28058);
            assert(pack.yacc_GET() == (short) -20897);
            assert(pack.zmag_GET() == (short)29831);
            assert(pack.zgyro_GET() == (short) -19392);
            assert(pack.time_usec_GET() == 951442690706624923L);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.zgyro_SET((short) -19392) ;
        p27.xgyro_SET((short)25453) ;
        p27.ygyro_SET((short)5737) ;
        p27.zmag_SET((short)29831) ;
        p27.ymag_SET((short)32320) ;
        p27.zacc_SET((short) -28058) ;
        p27.yacc_SET((short) -20897) ;
        p27.xacc_SET((short) -15752) ;
        p27.time_usec_SET(951442690706624923L) ;
        p27.xmag_SET((short) -26096) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)922);
            assert(pack.press_diff2_GET() == (short) -5840);
            assert(pack.press_abs_GET() == (short)23686);
            assert(pack.press_diff1_GET() == (short)10848);
            assert(pack.time_usec_GET() == 1375531271669172385L);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.time_usec_SET(1375531271669172385L) ;
        p28.press_diff1_SET((short)10848) ;
        p28.press_diff2_SET((short) -5840) ;
        p28.temperature_SET((short)922) ;
        p28.press_abs_SET((short)23686) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == 1.1958634E38F);
            assert(pack.press_abs_GET() == 2.5738697E37F);
            assert(pack.time_boot_ms_GET() == 440951594L);
            assert(pack.temperature_GET() == (short)18169);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_diff_SET(1.1958634E38F) ;
        p29.temperature_SET((short)18169) ;
        p29.time_boot_ms_SET(440951594L) ;
        p29.press_abs_SET(2.5738697E37F) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 2.7371962E38F);
            assert(pack.roll_GET() == 2.9933117E38F);
            assert(pack.rollspeed_GET() == -9.07641E37F);
            assert(pack.yawspeed_GET() == -3.8747716E37F);
            assert(pack.time_boot_ms_GET() == 4279761889L);
            assert(pack.yaw_GET() == -2.2924705E38F);
            assert(pack.pitchspeed_GET() == -2.2910194E37F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.yaw_SET(-2.2924705E38F) ;
        p30.rollspeed_SET(-9.07641E37F) ;
        p30.yawspeed_SET(-3.8747716E37F) ;
        p30.time_boot_ms_SET(4279761889L) ;
        p30.pitch_SET(2.7371962E38F) ;
        p30.roll_SET(2.9933117E38F) ;
        p30.pitchspeed_SET(-2.2910194E37F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q2_GET() == -3.0969655E38F);
            assert(pack.time_boot_ms_GET() == 2306022839L);
            assert(pack.q1_GET() == -1.4536321E38F);
            assert(pack.q4_GET() == 3.8768939E37F);
            assert(pack.pitchspeed_GET() == 2.1951616E37F);
            assert(pack.q3_GET() == 3.0811513E38F);
            assert(pack.yawspeed_GET() == 2.4985728E38F);
            assert(pack.rollspeed_GET() == -5.2638197E37F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.pitchspeed_SET(2.1951616E37F) ;
        p31.q3_SET(3.0811513E38F) ;
        p31.rollspeed_SET(-5.2638197E37F) ;
        p31.yawspeed_SET(2.4985728E38F) ;
        p31.time_boot_ms_SET(2306022839L) ;
        p31.q4_SET(3.8768939E37F) ;
        p31.q1_SET(-1.4536321E38F) ;
        p31.q2_SET(-3.0969655E38F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -1.2222639E38F);
            assert(pack.vz_GET() == -9.748789E37F);
            assert(pack.vy_GET() == 2.5524557E38F);
            assert(pack.y_GET() == 2.5788556E38F);
            assert(pack.z_GET() == -1.1891218E38F);
            assert(pack.vx_GET() == 2.3198661E38F);
            assert(pack.time_boot_ms_GET() == 641402556L);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.y_SET(2.5788556E38F) ;
        p32.z_SET(-1.1891218E38F) ;
        p32.vx_SET(2.3198661E38F) ;
        p32.x_SET(-1.2222639E38F) ;
        p32.vy_SET(2.5524557E38F) ;
        p32.vz_SET(-9.748789E37F) ;
        p32.time_boot_ms_SET(641402556L) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.hdg_GET() == (char)30167);
            assert(pack.lat_GET() == -2064076753);
            assert(pack.vy_GET() == (short) -1416);
            assert(pack.vz_GET() == (short)14912);
            assert(pack.alt_GET() == -1311720655);
            assert(pack.vx_GET() == (short) -32279);
            assert(pack.relative_alt_GET() == -309099764);
            assert(pack.lon_GET() == 1336971708);
            assert(pack.time_boot_ms_GET() == 1351618912L);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.lat_SET(-2064076753) ;
        p33.vz_SET((short)14912) ;
        p33.hdg_SET((char)30167) ;
        p33.time_boot_ms_SET(1351618912L) ;
        p33.lon_SET(1336971708) ;
        p33.vx_SET((short) -32279) ;
        p33.vy_SET((short) -1416) ;
        p33.alt_SET(-1311720655) ;
        p33.relative_alt_SET(-309099764) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan5_scaled_GET() == (short)22942);
            assert(pack.port_GET() == (char)85);
            assert(pack.rssi_GET() == (char)51);
            assert(pack.chan4_scaled_GET() == (short) -12750);
            assert(pack.chan3_scaled_GET() == (short) -5236);
            assert(pack.time_boot_ms_GET() == 1910206714L);
            assert(pack.chan7_scaled_GET() == (short) -30954);
            assert(pack.chan2_scaled_GET() == (short) -21827);
            assert(pack.chan1_scaled_GET() == (short)6982);
            assert(pack.chan8_scaled_GET() == (short) -12679);
            assert(pack.chan6_scaled_GET() == (short) -14844);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan2_scaled_SET((short) -21827) ;
        p34.chan1_scaled_SET((short)6982) ;
        p34.rssi_SET((char)51) ;
        p34.chan3_scaled_SET((short) -5236) ;
        p34.chan7_scaled_SET((short) -30954) ;
        p34.chan4_scaled_SET((short) -12750) ;
        p34.chan6_scaled_SET((short) -14844) ;
        p34.time_boot_ms_SET(1910206714L) ;
        p34.chan5_scaled_SET((short)22942) ;
        p34.port_SET((char)85) ;
        p34.chan8_scaled_SET((short) -12679) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan2_raw_GET() == (char)15276);
            assert(pack.rssi_GET() == (char)193);
            assert(pack.chan6_raw_GET() == (char)23083);
            assert(pack.port_GET() == (char)89);
            assert(pack.time_boot_ms_GET() == 2204938452L);
            assert(pack.chan7_raw_GET() == (char)5826);
            assert(pack.chan1_raw_GET() == (char)50842);
            assert(pack.chan4_raw_GET() == (char)31381);
            assert(pack.chan5_raw_GET() == (char)3090);
            assert(pack.chan8_raw_GET() == (char)2351);
            assert(pack.chan3_raw_GET() == (char)28205);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan3_raw_SET((char)28205) ;
        p35.chan6_raw_SET((char)23083) ;
        p35.chan1_raw_SET((char)50842) ;
        p35.chan4_raw_SET((char)31381) ;
        p35.port_SET((char)89) ;
        p35.chan5_raw_SET((char)3090) ;
        p35.time_boot_ms_SET(2204938452L) ;
        p35.chan7_raw_SET((char)5826) ;
        p35.rssi_SET((char)193) ;
        p35.chan8_raw_SET((char)2351) ;
        p35.chan2_raw_SET((char)15276) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo16_raw_TRY(ph) == (char)25426);
            assert(pack.servo15_raw_TRY(ph) == (char)41920);
            assert(pack.servo13_raw_TRY(ph) == (char)7077);
            assert(pack.servo9_raw_TRY(ph) == (char)18242);
            assert(pack.servo12_raw_TRY(ph) == (char)2707);
            assert(pack.servo6_raw_GET() == (char)48356);
            assert(pack.port_GET() == (char)133);
            assert(pack.servo1_raw_GET() == (char)11991);
            assert(pack.servo11_raw_TRY(ph) == (char)47668);
            assert(pack.servo7_raw_GET() == (char)63911);
            assert(pack.servo2_raw_GET() == (char)2349);
            assert(pack.servo4_raw_GET() == (char)18695);
            assert(pack.time_usec_GET() == 2380840126L);
            assert(pack.servo8_raw_GET() == (char)22768);
            assert(pack.servo3_raw_GET() == (char)42121);
            assert(pack.servo5_raw_GET() == (char)5939);
            assert(pack.servo10_raw_TRY(ph) == (char)62933);
            assert(pack.servo14_raw_TRY(ph) == (char)42591);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo1_raw_SET((char)11991) ;
        p36.servo2_raw_SET((char)2349) ;
        p36.servo16_raw_SET((char)25426, PH) ;
        p36.time_usec_SET(2380840126L) ;
        p36.servo8_raw_SET((char)22768) ;
        p36.servo4_raw_SET((char)18695) ;
        p36.servo14_raw_SET((char)42591, PH) ;
        p36.servo9_raw_SET((char)18242, PH) ;
        p36.servo6_raw_SET((char)48356) ;
        p36.servo7_raw_SET((char)63911) ;
        p36.servo3_raw_SET((char)42121) ;
        p36.port_SET((char)133) ;
        p36.servo10_raw_SET((char)62933, PH) ;
        p36.servo15_raw_SET((char)41920, PH) ;
        p36.servo12_raw_SET((char)2707, PH) ;
        p36.servo5_raw_SET((char)5939) ;
        p36.servo11_raw_SET((char)47668, PH) ;
        p36.servo13_raw_SET((char)7077, PH) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_index_GET() == (short)290);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)247);
            assert(pack.target_component_GET() == (char)188);
            assert(pack.start_index_GET() == (short)31125);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.end_index_SET((short)290) ;
        p37.start_index_SET((short)31125) ;
        p37.target_system_SET((char)247) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p37.target_component_SET((char)188) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_index_GET() == (short) -26250);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.end_index_GET() == (short)2387);
            assert(pack.target_component_GET() == (char)246);
            assert(pack.target_system_GET() == (char)252);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.start_index_SET((short) -26250) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p38.target_system_SET((char)252) ;
        p38.target_component_SET((char)246) ;
        p38.end_index_SET((short)2387) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION);
            assert(pack.param3_GET() == -3.444121E37F);
            assert(pack.target_component_GET() == (char)178);
            assert(pack.seq_GET() == (char)62738);
            assert(pack.param4_GET() == 5.680924E37F);
            assert(pack.current_GET() == (char)236);
            assert(pack.x_GET() == 2.7340203E38F);
            assert(pack.autocontinue_GET() == (char)156);
            assert(pack.target_system_GET() == (char)122);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.z_GET() == -1.6222479E38F);
            assert(pack.param1_GET() == 3.202738E37F);
            assert(pack.param2_GET() == 2.0294928E38F);
            assert(pack.y_GET() == 1.3903939E38F);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.param4_SET(5.680924E37F) ;
        p39.target_component_SET((char)178) ;
        p39.z_SET(-1.6222479E38F) ;
        p39.autocontinue_SET((char)156) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p39.command_SET(MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION) ;
        p39.param3_SET(-3.444121E37F) ;
        p39.x_SET(2.7340203E38F) ;
        p39.param1_SET(3.202738E37F) ;
        p39.y_SET(1.3903939E38F) ;
        p39.current_SET((char)236) ;
        p39.param2_SET(2.0294928E38F) ;
        p39.target_system_SET((char)122) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p39.seq_SET((char)62738) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)94);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.seq_GET() == (char)7027);
            assert(pack.target_component_GET() == (char)156);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p40.target_component_SET((char)156) ;
        p40.seq_SET((char)7027) ;
        p40.target_system_SET((char)94) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)42);
            assert(pack.target_component_GET() == (char)61);
            assert(pack.seq_GET() == (char)62060);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)42) ;
        p41.seq_SET((char)62060) ;
        p41.target_component_SET((char)61) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)57827);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)57827) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)158);
            assert(pack.target_system_GET() == (char)24);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)24) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p43.target_component_SET((char)158) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)169);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_component_GET() == (char)154);
            assert(pack.count_GET() == (char)6871);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p44.target_component_SET((char)154) ;
        p44.target_system_SET((char)169) ;
        p44.count_SET((char)6871) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)173);
            assert(pack.target_component_GET() == (char)29);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_component_SET((char)29) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p45.target_system_SET((char)173) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)1897);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)1897) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)226);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_DENIED);
            assert(pack.target_component_GET() == (char)90);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.target_component_SET((char)90) ;
        p47.target_system_SET((char)226) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_DENIED) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 631510880);
            assert(pack.time_usec_TRY(ph) == 6537653435365256165L);
            assert(pack.longitude_GET() == -1432624069);
            assert(pack.altitude_GET() == -472854067);
            assert(pack.target_system_GET() == (char)244);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.altitude_SET(-472854067) ;
        p48.latitude_SET(631510880) ;
        p48.longitude_SET(-1432624069) ;
        p48.time_usec_SET(6537653435365256165L, PH) ;
        p48.target_system_SET((char)244) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 560765246);
            assert(pack.altitude_GET() == 1063860946);
            assert(pack.longitude_GET() == -1042751413);
            assert(pack.time_usec_TRY(ph) == 1199600806344498928L);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.longitude_SET(-1042751413) ;
        p49.time_usec_SET(1199600806344498928L, PH) ;
        p49.altitude_SET(1063860946) ;
        p49.latitude_SET(560765246) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.param_value_max_GET() == 2.3896842E38F);
            assert(pack.target_component_GET() == (char)237);
            assert(pack.param_id_LEN(ph) == 16);
            assert(pack.param_id_TRY(ph).equals("folxnddtfQrhskyD"));
            assert(pack.parameter_rc_channel_index_GET() == (char)78);
            assert(pack.param_value0_GET() == -1.8579592E38F);
            assert(pack.target_system_GET() == (char)2);
            assert(pack.param_index_GET() == (short)20010);
            assert(pack.param_value_min_GET() == -1.6272831E37F);
            assert(pack.scale_GET() == 2.9045293E38F);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.param_value_min_SET(-1.6272831E37F) ;
        p50.target_system_SET((char)2) ;
        p50.parameter_rc_channel_index_SET((char)78) ;
        p50.param_value_max_SET(2.3896842E38F) ;
        p50.param_id_SET("folxnddtfQrhskyD", PH) ;
        p50.target_component_SET((char)237) ;
        p50.scale_SET(2.9045293E38F) ;
        p50.param_value0_SET(-1.8579592E38F) ;
        p50.param_index_SET((short)20010) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)199);
            assert(pack.seq_GET() == (char)47417);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_component_GET() == (char)181);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_system_SET((char)199) ;
        p51.seq_SET((char)47417) ;
        p51.target_component_SET((char)181) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2y_GET() == 1.1000671E38F);
            assert(pack.p2x_GET() == 2.390821E38F);
            assert(pack.target_component_GET() == (char)111);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.p2z_GET() == 1.4171025E38F);
            assert(pack.p1z_GET() == 1.0734457E38F);
            assert(pack.p1x_GET() == -2.1064556E38F);
            assert(pack.target_system_GET() == (char)17);
            assert(pack.p1y_GET() == 3.3744246E38F);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.target_system_SET((char)17) ;
        p54.p1y_SET(3.3744246E38F) ;
        p54.p2x_SET(2.390821E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p54.p1z_SET(1.0734457E38F) ;
        p54.p2y_SET(1.1000671E38F) ;
        p54.target_component_SET((char)111) ;
        p54.p1x_SET(-2.1064556E38F) ;
        p54.p2z_SET(1.4171025E38F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1y_GET() == 2.3853284E38F);
            assert(pack.p2z_GET() == -1.902422E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.p1z_GET() == -1.5154532E38F);
            assert(pack.p1x_GET() == 2.8097686E38F);
            assert(pack.p2x_GET() == -2.7342256E38F);
            assert(pack.p2y_GET() == -3.1983679E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p1x_SET(2.8097686E38F) ;
        p55.p1y_SET(2.3853284E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p55.p1z_SET(-1.5154532E38F) ;
        p55.p2y_SET(-3.1983679E38F) ;
        p55.p2x_SET(-2.7342256E38F) ;
        p55.p2z_SET(-1.902422E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == 8.3541146E37F);
            assert(pack.time_usec_GET() == 4449787782291963066L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.4936489E38F, -3.3554535E38F, 1.2330704E38F, -2.7523526E38F}));
            assert(pack.pitchspeed_GET() == 1.7969497E38F);
            assert(pack.yawspeed_GET() == -2.8814325E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-5.2385803E37F, -1.1666072E38F, -1.8210195E37F, -1.7251804E38F, 2.7926047E38F, 2.2457223E38F, -1.4929671E38F, 2.5818887E38F, -2.4402791E38F}));
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.rollspeed_SET(8.3541146E37F) ;
        p61.q_SET(new float[] {1.4936489E38F, -3.3554535E38F, 1.2330704E38F, -2.7523526E38F}, 0) ;
        p61.pitchspeed_SET(1.7969497E38F) ;
        p61.covariance_SET(new float[] {-5.2385803E37F, -1.1666072E38F, -1.8210195E37F, -1.7251804E38F, 2.7926047E38F, 2.2457223E38F, -1.4929671E38F, 2.5818887E38F, -2.4402791E38F}, 0) ;
        p61.time_usec_SET(4449787782291963066L) ;
        p61.yawspeed_SET(-2.8814325E38F) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.nav_roll_GET() == -7.539574E37F);
            assert(pack.aspd_error_GET() == 2.4148458E38F);
            assert(pack.target_bearing_GET() == (short) -1313);
            assert(pack.alt_error_GET() == -1.8203998E38F);
            assert(pack.wp_dist_GET() == (char)46918);
            assert(pack.nav_pitch_GET() == 1.9171568E38F);
            assert(pack.xtrack_error_GET() == -2.0168658E38F);
            assert(pack.nav_bearing_GET() == (short)2373);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.alt_error_SET(-1.8203998E38F) ;
        p62.nav_bearing_SET((short)2373) ;
        p62.target_bearing_SET((short) -1313) ;
        p62.nav_roll_SET(-7.539574E37F) ;
        p62.nav_pitch_SET(1.9171568E38F) ;
        p62.aspd_error_SET(2.4148458E38F) ;
        p62.xtrack_error_SET(-2.0168658E38F) ;
        p62.wp_dist_SET((char)46918) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == -6.166494E37F);
            assert(pack.relative_alt_GET() == -1204787053);
            assert(pack.lon_GET() == 1984864984);
            assert(pack.vx_GET() == 1.6262361E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {1.1812605E38F, 2.840328E36F, -3.2889886E38F, 2.3892107E38F, -2.1213916E38F, 1.5824117E38F, -2.7908218E38F, 2.69158E38F, -1.927336E38F, 3.2843239E38F, 4.029296E37F, 4.2014466E37F, -2.0134733E37F, 8.4325097E37F, -5.4814774E37F, -3.0608904E38F, 2.4577709E38F, 9.719828E37F, 1.3206022E38F, 2.8153804E38F, 3.3583527E38F, 2.6742757E38F, 1.4226523E38F, -8.908462E37F, 3.530397E37F, -3.316926E38F, -8.1923725E37F, 4.1724532E37F, -1.052456E38F, 3.0990283E38F, -6.473621E37F, 2.974916E38F, -2.3235025E38F, 5.6636585E37F, 2.478769E38F, 2.9093115E38F}));
            assert(pack.lat_GET() == -1788386407);
            assert(pack.vy_GET() == 2.0397188E38F);
            assert(pack.alt_GET() == 1821703563);
            assert(pack.time_usec_GET() == 324157713858183774L);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.vx_SET(1.6262361E38F) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        p63.vy_SET(2.0397188E38F) ;
        p63.relative_alt_SET(-1204787053) ;
        p63.alt_SET(1821703563) ;
        p63.time_usec_SET(324157713858183774L) ;
        p63.lat_SET(-1788386407) ;
        p63.vz_SET(-6.166494E37F) ;
        p63.lon_SET(1984864984) ;
        p63.covariance_SET(new float[] {1.1812605E38F, 2.840328E36F, -3.2889886E38F, 2.3892107E38F, -2.1213916E38F, 1.5824117E38F, -2.7908218E38F, 2.69158E38F, -1.927336E38F, 3.2843239E38F, 4.029296E37F, 4.2014466E37F, -2.0134733E37F, 8.4325097E37F, -5.4814774E37F, -3.0608904E38F, 2.4577709E38F, 9.719828E37F, 1.3206022E38F, 2.8153804E38F, 3.3583527E38F, 2.6742757E38F, 1.4226523E38F, -8.908462E37F, 3.530397E37F, -3.316926E38F, -8.1923725E37F, 4.1724532E37F, -1.052456E38F, 3.0990283E38F, -6.473621E37F, 2.974916E38F, -2.3235025E38F, 5.6636585E37F, 2.478769E38F, 2.9093115E38F}, 0) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == 2.2902136E38F);
            assert(pack.time_usec_GET() == 3684611110192250301L);
            assert(pack.y_GET() == 2.1760162E38F);
            assert(pack.az_GET() == -2.238085E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.2515916E37F, 6.480704E37F, -2.4211934E38F, 3.2886672E37F, -9.216429E37F, 3.2856652E38F, 1.2257683E38F, 2.56787E38F, 2.2007082E38F, 2.3717818E38F, -2.7695604E38F, 1.3699E38F, 4.8652486E37F, -5.79867E37F, 1.7918156E38F, 2.852876E38F, 2.10763E38F, 1.3265648E38F, -2.9449316E38F, 3.1379433E38F, -2.9727427E38F, -2.4415738E38F, -1.0364356E37F, 2.2112532E38F, -1.9468975E38F, -2.2304207E38F, -5.548641E37F, 2.8920941E37F, -2.9777104E38F, -1.6468463E38F, 1.8575898E38F, -3.04694E38F, 1.5409972E38F, -2.49208E38F, 1.5723923E37F, 6.879431E37F, 4.2043924E37F, -3.431032E37F, -3.364717E38F, -1.5286742E38F, 5.2687337E37F, -1.565484E38F, -3.6948635E37F, 6.8927746E37F, -5.308979E37F}));
            assert(pack.ay_GET() == -1.7048241E38F);
            assert(pack.vx_GET() == -3.347961E37F);
            assert(pack.x_GET() == 2.485153E38F);
            assert(pack.z_GET() == 2.7885011E38F);
            assert(pack.vz_GET() == -1.87199E38F);
            assert(pack.ax_GET() == 2.5023887E38F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.time_usec_SET(3684611110192250301L) ;
        p64.az_SET(-2.238085E38F) ;
        p64.z_SET(2.7885011E38F) ;
        p64.ay_SET(-1.7048241E38F) ;
        p64.vx_SET(-3.347961E37F) ;
        p64.ax_SET(2.5023887E38F) ;
        p64.vy_SET(2.2902136E38F) ;
        p64.x_SET(2.485153E38F) ;
        p64.y_SET(2.1760162E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS) ;
        p64.vz_SET(-1.87199E38F) ;
        p64.covariance_SET(new float[] {-2.2515916E37F, 6.480704E37F, -2.4211934E38F, 3.2886672E37F, -9.216429E37F, 3.2856652E38F, 1.2257683E38F, 2.56787E38F, 2.2007082E38F, 2.3717818E38F, -2.7695604E38F, 1.3699E38F, 4.8652486E37F, -5.79867E37F, 1.7918156E38F, 2.852876E38F, 2.10763E38F, 1.3265648E38F, -2.9449316E38F, 3.1379433E38F, -2.9727427E38F, -2.4415738E38F, -1.0364356E37F, 2.2112532E38F, -1.9468975E38F, -2.2304207E38F, -5.548641E37F, 2.8920941E37F, -2.9777104E38F, -1.6468463E38F, 1.8575898E38F, -3.04694E38F, 1.5409972E38F, -2.49208E38F, 1.5723923E37F, 6.879431E37F, 4.2043924E37F, -3.431032E37F, -3.364717E38F, -1.5286742E38F, 5.2687337E37F, -1.565484E38F, -3.6948635E37F, 6.8927746E37F, -5.308979E37F}, 0) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan2_raw_GET() == (char)56768);
            assert(pack.chan1_raw_GET() == (char)65345);
            assert(pack.chan17_raw_GET() == (char)35402);
            assert(pack.chan9_raw_GET() == (char)39288);
            assert(pack.chan15_raw_GET() == (char)3033);
            assert(pack.chan12_raw_GET() == (char)61105);
            assert(pack.chancount_GET() == (char)138);
            assert(pack.chan18_raw_GET() == (char)36900);
            assert(pack.rssi_GET() == (char)116);
            assert(pack.chan3_raw_GET() == (char)15710);
            assert(pack.chan5_raw_GET() == (char)51088);
            assert(pack.chan4_raw_GET() == (char)9570);
            assert(pack.chan6_raw_GET() == (char)44556);
            assert(pack.chan7_raw_GET() == (char)50692);
            assert(pack.chan8_raw_GET() == (char)34505);
            assert(pack.chan10_raw_GET() == (char)25555);
            assert(pack.chan14_raw_GET() == (char)11605);
            assert(pack.time_boot_ms_GET() == 2750852182L);
            assert(pack.chan13_raw_GET() == (char)50614);
            assert(pack.chan16_raw_GET() == (char)22559);
            assert(pack.chan11_raw_GET() == (char)2723);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan9_raw_SET((char)39288) ;
        p65.chan13_raw_SET((char)50614) ;
        p65.chan11_raw_SET((char)2723) ;
        p65.chan14_raw_SET((char)11605) ;
        p65.chan16_raw_SET((char)22559) ;
        p65.chancount_SET((char)138) ;
        p65.chan8_raw_SET((char)34505) ;
        p65.chan7_raw_SET((char)50692) ;
        p65.chan10_raw_SET((char)25555) ;
        p65.time_boot_ms_SET(2750852182L) ;
        p65.chan6_raw_SET((char)44556) ;
        p65.chan5_raw_SET((char)51088) ;
        p65.chan2_raw_SET((char)56768) ;
        p65.chan1_raw_SET((char)65345) ;
        p65.chan18_raw_SET((char)36900) ;
        p65.chan3_raw_SET((char)15710) ;
        p65.chan17_raw_SET((char)35402) ;
        p65.rssi_SET((char)116) ;
        p65.chan12_raw_SET((char)61105) ;
        p65.chan4_raw_SET((char)9570) ;
        p65.chan15_raw_SET((char)3033) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.start_stop_GET() == (char)49);
            assert(pack.req_message_rate_GET() == (char)43708);
            assert(pack.target_system_GET() == (char)148);
            assert(pack.target_component_GET() == (char)177);
            assert(pack.req_stream_id_GET() == (char)196);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.req_stream_id_SET((char)196) ;
        p66.req_message_rate_SET((char)43708) ;
        p66.start_stop_SET((char)49) ;
        p66.target_system_SET((char)148) ;
        p66.target_component_SET((char)177) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.on_off_GET() == (char)122);
            assert(pack.stream_id_GET() == (char)191);
            assert(pack.message_rate_GET() == (char)59246);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.stream_id_SET((char)191) ;
        p67.message_rate_SET((char)59246) ;
        p67.on_off_SET((char)122) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == (short)18823);
            assert(pack.target_GET() == (char)244);
            assert(pack.y_GET() == (short)1775);
            assert(pack.r_GET() == (short) -8263);
            assert(pack.z_GET() == (short)26786);
            assert(pack.buttons_GET() == (char)11918);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.y_SET((short)1775) ;
        p69.z_SET((short)26786) ;
        p69.target_SET((char)244) ;
        p69.r_SET((short) -8263) ;
        p69.buttons_SET((char)11918) ;
        p69.x_SET((short)18823) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan1_raw_GET() == (char)17967);
            assert(pack.target_system_GET() == (char)135);
            assert(pack.chan2_raw_GET() == (char)38241);
            assert(pack.chan4_raw_GET() == (char)33437);
            assert(pack.target_component_GET() == (char)186);
            assert(pack.chan8_raw_GET() == (char)14717);
            assert(pack.chan6_raw_GET() == (char)27101);
            assert(pack.chan5_raw_GET() == (char)5637);
            assert(pack.chan7_raw_GET() == (char)38126);
            assert(pack.chan3_raw_GET() == (char)44244);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan5_raw_SET((char)5637) ;
        p70.chan8_raw_SET((char)14717) ;
        p70.chan2_raw_SET((char)38241) ;
        p70.chan3_raw_SET((char)44244) ;
        p70.chan4_raw_SET((char)33437) ;
        p70.chan1_raw_SET((char)17967) ;
        p70.chan6_raw_SET((char)27101) ;
        p70.target_system_SET((char)135) ;
        p70.chan7_raw_SET((char)38126) ;
        p70.target_component_SET((char)186) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 1524243164);
            assert(pack.target_system_GET() == (char)113);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.param4_GET() == 1.6132511E38F);
            assert(pack.current_GET() == (char)150);
            assert(pack.z_GET() == 1.2545207E38F);
            assert(pack.param1_GET() == -3.2513088E38F);
            assert(pack.y_GET() == -1987569628);
            assert(pack.param2_GET() == 1.6377078E38F);
            assert(pack.target_component_GET() == (char)212);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_START_RX_PAIR);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.param3_GET() == -2.0886053E38F);
            assert(pack.autocontinue_GET() == (char)32);
            assert(pack.seq_GET() == (char)53766);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.target_component_SET((char)212) ;
        p73.y_SET(-1987569628) ;
        p73.param2_SET(1.6377078E38F) ;
        p73.param3_SET(-2.0886053E38F) ;
        p73.x_SET(1524243164) ;
        p73.param4_SET(1.6132511E38F) ;
        p73.param1_SET(-3.2513088E38F) ;
        p73.autocontinue_SET((char)32) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p73.target_system_SET((char)113) ;
        p73.command_SET(MAV_CMD.MAV_CMD_START_RX_PAIR) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p73.seq_SET((char)53766) ;
        p73.z_SET(1.2545207E38F) ;
        p73.current_SET((char)150) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.groundspeed_GET() == -1.667788E38F);
            assert(pack.heading_GET() == (short)20413);
            assert(pack.airspeed_GET() == -4.1733583E37F);
            assert(pack.climb_GET() == 3.0616881E38F);
            assert(pack.alt_GET() == 1.2370651E38F);
            assert(pack.throttle_GET() == (char)36231);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.throttle_SET((char)36231) ;
        p74.heading_SET((short)20413) ;
        p74.alt_SET(1.2370651E38F) ;
        p74.groundspeed_SET(-1.667788E38F) ;
        p74.climb_SET(3.0616881E38F) ;
        p74.airspeed_SET(-4.1733583E37F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.autocontinue_GET() == (char)4);
            assert(pack.target_system_GET() == (char)23);
            assert(pack.y_GET() == 810781448);
            assert(pack.target_component_GET() == (char)108);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.current_GET() == (char)112);
            assert(pack.x_GET() == -919391553);
            assert(pack.param1_GET() == 2.0892133E37F);
            assert(pack.param3_GET() == -1.3765843E38F);
            assert(pack.z_GET() == 3.1493353E38F);
            assert(pack.param2_GET() == -2.71584E38F);
            assert(pack.param4_GET() == -1.729474E38F);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.y_SET(810781448) ;
        p75.current_SET((char)112) ;
        p75.autocontinue_SET((char)4) ;
        p75.param2_SET(-2.71584E38F) ;
        p75.param4_SET(-1.729474E38F) ;
        p75.z_SET(3.1493353E38F) ;
        p75.target_system_SET((char)23) ;
        p75.command_SET(MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE) ;
        p75.param3_SET(-1.3765843E38F) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p75.param1_SET(2.0892133E37F) ;
        p75.x_SET(-919391553) ;
        p75.target_component_SET((char)108) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param2_GET() == 2.7256605E36F);
            assert(pack.param4_GET() == 1.1016276E38F);
            assert(pack.param5_GET() == 1.6988689E38F);
            assert(pack.confirmation_GET() == (char)16);
            assert(pack.param7_GET() == -2.5146064E38F);
            assert(pack.target_component_GET() == (char)76);
            assert(pack.param6_GET() == 2.917639E38F);
            assert(pack.param1_GET() == -1.1348382E38F);
            assert(pack.param3_GET() == 8.357459E34F);
            assert(pack.target_system_GET() == (char)68);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE);
        });
        COMMAND_LONG p76 = new COMMAND_LONG();
        PH.setPack(p76);
        p76.param2_SET(2.7256605E36F) ;
        p76.param5_SET(1.6988689E38F) ;
        p76.param7_SET(-2.5146064E38F) ;
        p76.target_component_SET((char)76) ;
        p76.param1_SET(-1.1348382E38F) ;
        p76.confirmation_SET((char)16) ;
        p76.param3_SET(8.357459E34F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE) ;
        p76.param4_SET(1.1016276E38F) ;
        p76.target_system_SET((char)68) ;
        p76.param6_SET(2.917639E38F) ;
        TestChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_IN_PROGRESS);
            assert(pack.target_component_TRY(ph) == (char)210);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST);
            assert(pack.progress_TRY(ph) == (char)206);
            assert(pack.target_system_TRY(ph) == (char)83);
            assert(pack.result_param2_TRY(ph) == 1852461023);
        });
        COMMAND_ACK p77 = new COMMAND_ACK();
        PH.setPack(p77);
        p77.result_SET(MAV_RESULT.MAV_RESULT_IN_PROGRESS) ;
        p77.command_SET(MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST) ;
        p77.progress_SET((char)206, PH) ;
        p77.target_component_SET((char)210, PH) ;
        p77.result_param2_SET(1852461023, PH) ;
        p77.target_system_SET((char)83, PH) ;
        TestChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -2.2701405E38F);
            assert(pack.time_boot_ms_GET() == 2372657289L);
            assert(pack.manual_override_switch_GET() == (char)44);
            assert(pack.thrust_GET() == -1.6813749E38F);
            assert(pack.yaw_GET() == -2.1957008E38F);
            assert(pack.roll_GET() == -1.2939508E37F);
            assert(pack.mode_switch_GET() == (char)233);
        });
        MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.mode_switch_SET((char)233) ;
        p81.yaw_SET(-2.1957008E38F) ;
        p81.manual_override_switch_SET((char)44) ;
        p81.pitch_SET(-2.2701405E38F) ;
        p81.thrust_SET(-1.6813749E38F) ;
        p81.roll_SET(-1.2939508E37F) ;
        p81.time_boot_ms_SET(2372657289L) ;
        TestChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.8338076E38F, -2.4860342E38F, -2.5226373E38F, -2.321742E37F}));
            assert(pack.target_component_GET() == (char)173);
            assert(pack.body_yaw_rate_GET() == 2.6847878E38F);
            assert(pack.body_pitch_rate_GET() == 2.0419507E38F);
            assert(pack.body_roll_rate_GET() == 1.7790302E38F);
            assert(pack.type_mask_GET() == (char)215);
            assert(pack.thrust_GET() == 2.3876483E38F);
            assert(pack.target_system_GET() == (char)26);
            assert(pack.time_boot_ms_GET() == 4104679332L);
        });
        SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.body_roll_rate_SET(1.7790302E38F) ;
        p82.thrust_SET(2.3876483E38F) ;
        p82.target_component_SET((char)173) ;
        p82.target_system_SET((char)26) ;
        p82.type_mask_SET((char)215) ;
        p82.time_boot_ms_SET(4104679332L) ;
        p82.q_SET(new float[] {2.8338076E38F, -2.4860342E38F, -2.5226373E38F, -2.321742E37F}, 0) ;
        p82.body_yaw_rate_SET(2.6847878E38F) ;
        p82.body_pitch_rate_SET(2.0419507E38F) ;
        TestChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_yaw_rate_GET() == -2.52265E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.5837532E38F, -2.2340237E38F, -2.9702031E38F, -1.6829687E38F}));
            assert(pack.thrust_GET() == -2.8089642E38F);
            assert(pack.time_boot_ms_GET() == 182432351L);
            assert(pack.type_mask_GET() == (char)167);
            assert(pack.body_roll_rate_GET() == 1.6730169E38F);
            assert(pack.body_pitch_rate_GET() == 1.6569659E38F);
        });
        ATTITUDE_TARGET p83 = new ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.time_boot_ms_SET(182432351L) ;
        p83.body_roll_rate_SET(1.6730169E38F) ;
        p83.body_pitch_rate_SET(1.6569659E38F) ;
        p83.thrust_SET(-2.8089642E38F) ;
        p83.body_yaw_rate_SET(-2.52265E38F) ;
        p83.type_mask_SET((char)167) ;
        p83.q_SET(new float[] {1.5837532E38F, -2.2340237E38F, -2.9702031E38F, -1.6829687E38F}, 0) ;
        TestChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 2.0928461E38F);
            assert(pack.vz_GET() == -3.3526089E38F);
            assert(pack.yaw_rate_GET() == -2.2739269E38F);
            assert(pack.target_system_GET() == (char)18);
            assert(pack.z_GET() == -3.2354486E38F);
            assert(pack.vy_GET() == -1.776316E37F);
            assert(pack.type_mask_GET() == (char)24281);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.afy_GET() == -1.8589877E38F);
            assert(pack.x_GET() == -1.825352E38F);
            assert(pack.target_component_GET() == (char)124);
            assert(pack.time_boot_ms_GET() == 2091435085L);
            assert(pack.afx_GET() == 6.194062E36F);
            assert(pack.yaw_GET() == 1.3721415E38F);
            assert(pack.vx_GET() == -9.613637E37F);
            assert(pack.afz_GET() == 9.666057E37F);
        });
        SET_POSITION_TARGET_LOCAL_NED p84 = new SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.target_system_SET((char)18) ;
        p84.afx_SET(6.194062E36F) ;
        p84.vz_SET(-3.3526089E38F) ;
        p84.target_component_SET((char)124) ;
        p84.vy_SET(-1.776316E37F) ;
        p84.y_SET(2.0928461E38F) ;
        p84.x_SET(-1.825352E38F) ;
        p84.vx_SET(-9.613637E37F) ;
        p84.yaw_SET(1.3721415E38F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p84.time_boot_ms_SET(2091435085L) ;
        p84.afz_SET(9.666057E37F) ;
        p84.yaw_rate_SET(-2.2739269E38F) ;
        p84.afy_SET(-1.8589877E38F) ;
        p84.z_SET(-3.2354486E38F) ;
        p84.type_mask_SET((char)24281) ;
        TestChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.afx_GET() == 2.3359411E38F);
            assert(pack.afz_GET() == -2.3779315E37F);
            assert(pack.alt_GET() == 1.6360637E38F);
            assert(pack.time_boot_ms_GET() == 443309063L);
            assert(pack.vz_GET() == -3.3180823E38F);
            assert(pack.target_component_GET() == (char)171);
            assert(pack.target_system_GET() == (char)137);
            assert(pack.type_mask_GET() == (char)41851);
            assert(pack.lon_int_GET() == -965408595);
            assert(pack.yaw_rate_GET() == 2.4493239E38F);
            assert(pack.afy_GET() == -1.2436629E38F);
            assert(pack.lat_int_GET() == -2130771374);
            assert(pack.yaw_GET() == -7.256365E37F);
            assert(pack.vx_GET() == 3.1989893E38F);
            assert(pack.vy_GET() == 8.1879606E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
        });
        SET_POSITION_TARGET_GLOBAL_INT p86 = new SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.alt_SET(1.6360637E38F) ;
        p86.time_boot_ms_SET(443309063L) ;
        p86.vy_SET(8.1879606E37F) ;
        p86.lat_int_SET(-2130771374) ;
        p86.afx_SET(2.3359411E38F) ;
        p86.target_system_SET((char)137) ;
        p86.vx_SET(3.1989893E38F) ;
        p86.type_mask_SET((char)41851) ;
        p86.target_component_SET((char)171) ;
        p86.afz_SET(-2.3779315E37F) ;
        p86.afy_SET(-1.2436629E38F) ;
        p86.yaw_rate_SET(2.4493239E38F) ;
        p86.lon_int_SET(-965408595) ;
        p86.vz_SET(-3.3180823E38F) ;
        p86.yaw_SET(-7.256365E37F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        TestChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == 3.8582853E37F);
            assert(pack.afx_GET() == -1.3276887E38F);
            assert(pack.lon_int_GET() == 1913731144);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.yaw_GET() == 9.908905E37F);
            assert(pack.afz_GET() == -3.7354322E37F);
            assert(pack.afy_GET() == -4.933915E37F);
            assert(pack.vx_GET() == 3.430268E37F);
            assert(pack.lat_int_GET() == -409914741);
            assert(pack.time_boot_ms_GET() == 375558791L);
            assert(pack.yaw_rate_GET() == -2.7529004E38F);
            assert(pack.vy_GET() == 1.1543418E38F);
            assert(pack.type_mask_GET() == (char)23611);
            assert(pack.alt_GET() == -1.4218954E38F);
        });
        POSITION_TARGET_GLOBAL_INT p87 = new POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.lon_int_SET(1913731144) ;
        p87.afy_SET(-4.933915E37F) ;
        p87.vy_SET(1.1543418E38F) ;
        p87.lat_int_SET(-409914741) ;
        p87.vz_SET(3.8582853E37F) ;
        p87.yaw_rate_SET(-2.7529004E38F) ;
        p87.alt_SET(-1.4218954E38F) ;
        p87.vx_SET(3.430268E37F) ;
        p87.time_boot_ms_SET(375558791L) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p87.yaw_SET(9.908905E37F) ;
        p87.type_mask_SET((char)23611) ;
        p87.afx_SET(-1.3276887E38F) ;
        p87.afz_SET(-3.7354322E37F) ;
        TestChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 417356342L);
            assert(pack.y_GET() == -1.0269319E38F);
            assert(pack.yaw_GET() == 2.4262796E38F);
            assert(pack.x_GET() == -1.127963E37F);
            assert(pack.z_GET() == -6.83898E37F);
            assert(pack.roll_GET() == 1.740648E38F);
            assert(pack.pitch_GET() == -1.0709291E38F);
        });
        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = new LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.pitch_SET(-1.0709291E38F) ;
        p89.y_SET(-1.0269319E38F) ;
        p89.z_SET(-6.83898E37F) ;
        p89.time_boot_ms_SET(417356342L) ;
        p89.yaw_SET(2.4262796E38F) ;
        p89.x_SET(-1.127963E37F) ;
        p89.roll_SET(1.740648E38F) ;
        TestChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == 2.5341707E38F);
            assert(pack.yacc_GET() == (short) -4236);
            assert(pack.pitchspeed_GET() == 2.4610327E38F);
            assert(pack.yaw_GET() == -3.2084415E38F);
            assert(pack.roll_GET() == 8.8525936E36F);
            assert(pack.rollspeed_GET() == -1.2740802E38F);
            assert(pack.vz_GET() == (short)16475);
            assert(pack.lon_GET() == -131561949);
            assert(pack.lat_GET() == -344499241);
            assert(pack.zacc_GET() == (short) -23951);
            assert(pack.vy_GET() == (short) -20841);
            assert(pack.xacc_GET() == (short) -16315);
            assert(pack.pitch_GET() == 1.2718062E38F);
            assert(pack.time_usec_GET() == 4318915794652608936L);
            assert(pack.alt_GET() == 1550390298);
            assert(pack.vx_GET() == (short) -21134);
        });
        HIL_STATE p90 = new HIL_STATE();
        PH.setPack(p90);
        p90.yaw_SET(-3.2084415E38F) ;
        p90.vx_SET((short) -21134) ;
        p90.lat_SET(-344499241) ;
        p90.pitch_SET(1.2718062E38F) ;
        p90.pitchspeed_SET(2.4610327E38F) ;
        p90.lon_SET(-131561949) ;
        p90.zacc_SET((short) -23951) ;
        p90.vy_SET((short) -20841) ;
        p90.vz_SET((short)16475) ;
        p90.yawspeed_SET(2.5341707E38F) ;
        p90.rollspeed_SET(-1.2740802E38F) ;
        p90.time_usec_SET(4318915794652608936L) ;
        p90.roll_SET(8.8525936E36F) ;
        p90.yacc_SET((short) -4236) ;
        p90.xacc_SET((short) -16315) ;
        p90.alt_SET(1550390298) ;
        TestChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.nav_mode_GET() == (char)213);
            assert(pack.aux1_GET() == 2.3289177E38F);
            assert(pack.throttle_GET() == 2.8515733E38F);
            assert(pack.aux4_GET() == -3.164914E38F);
            assert(pack.aux3_GET() == 2.9859623E38F);
            assert(pack.time_usec_GET() == 4698676082672657724L);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_PREFLIGHT);
            assert(pack.aux2_GET() == -1.1622639E38F);
            assert(pack.pitch_elevator_GET() == 1.958004E38F);
            assert(pack.roll_ailerons_GET() == -1.8781144E38F);
            assert(pack.yaw_rudder_GET() == -1.2204372E38F);
        });
        HIL_CONTROLS p91 = new HIL_CONTROLS();
        PH.setPack(p91);
        p91.roll_ailerons_SET(-1.8781144E38F) ;
        p91.aux4_SET(-3.164914E38F) ;
        p91.nav_mode_SET((char)213) ;
        p91.pitch_elevator_SET(1.958004E38F) ;
        p91.yaw_rudder_SET(-1.2204372E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_PREFLIGHT) ;
        p91.aux1_SET(2.3289177E38F) ;
        p91.aux3_SET(2.9859623E38F) ;
        p91.time_usec_SET(4698676082672657724L) ;
        p91.aux2_SET(-1.1622639E38F) ;
        p91.throttle_SET(2.8515733E38F) ;
        TestChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.rssi_GET() == (char)221);
            assert(pack.chan10_raw_GET() == (char)18439);
            assert(pack.chan2_raw_GET() == (char)29415);
            assert(pack.chan12_raw_GET() == (char)55597);
            assert(pack.chan11_raw_GET() == (char)1217);
            assert(pack.chan9_raw_GET() == (char)26900);
            assert(pack.chan6_raw_GET() == (char)7760);
            assert(pack.time_usec_GET() == 4650243833169855643L);
            assert(pack.chan5_raw_GET() == (char)45138);
            assert(pack.chan8_raw_GET() == (char)21939);
            assert(pack.chan7_raw_GET() == (char)60099);
            assert(pack.chan1_raw_GET() == (char)6007);
            assert(pack.chan3_raw_GET() == (char)52870);
            assert(pack.chan4_raw_GET() == (char)8922);
        });
        HIL_RC_INPUTS_RAW p92 = new HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan2_raw_SET((char)29415) ;
        p92.chan7_raw_SET((char)60099) ;
        p92.chan6_raw_SET((char)7760) ;
        p92.time_usec_SET(4650243833169855643L) ;
        p92.rssi_SET((char)221) ;
        p92.chan11_raw_SET((char)1217) ;
        p92.chan9_raw_SET((char)26900) ;
        p92.chan8_raw_SET((char)21939) ;
        p92.chan1_raw_SET((char)6007) ;
        p92.chan3_raw_SET((char)52870) ;
        p92.chan4_raw_SET((char)8922) ;
        p92.chan12_raw_SET((char)55597) ;
        p92.chan5_raw_SET((char)45138) ;
        p92.chan10_raw_SET((char)18439) ;
        TestChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_ARMED);
            assert(pack.time_usec_GET() == 5990704933327924573L);
            assert(pack.flags_GET() == 4962651077019560917L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-1.4263511E38F, -2.3185522E38F, 2.9164618E38F, -1.76382E38F, 2.6802553E37F, 3.3944055E38F, 1.839135E38F, -3.2251648E37F, 1.5423469E38F, -2.4355904E38F, 3.2125231E38F, -7.233538E37F, 8.5025605E37F, 9.655882E37F, 1.8891531E38F, -1.941075E38F}));
        });
        HIL_ACTUATOR_CONTROLS p93 = new HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.mode_SET(MAV_MODE.MAV_MODE_GUIDED_ARMED) ;
        p93.time_usec_SET(5990704933327924573L) ;
        p93.flags_SET(4962651077019560917L) ;
        p93.controls_SET(new float[] {-1.4263511E38F, -2.3185522E38F, 2.9164618E38F, -1.76382E38F, 2.6802553E37F, 3.3944055E38F, 1.839135E38F, -3.2251648E37F, 1.5423469E38F, -2.4355904E38F, 3.2125231E38F, -7.233538E37F, 8.5025605E37F, 9.655882E37F, 1.8891531E38F, -1.941075E38F}, 0) ;
        TestChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_rate_x_TRY(ph) == -2.3354858E38F);
            assert(pack.flow_comp_m_y_GET() == -2.937465E37F);
            assert(pack.flow_rate_y_TRY(ph) == 3.935194E37F);
            assert(pack.ground_distance_GET() == -2.0263273E38F);
            assert(pack.sensor_id_GET() == (char)208);
            assert(pack.flow_y_GET() == (short)8770);
            assert(pack.time_usec_GET() == 6008191226748134753L);
            assert(pack.flow_comp_m_x_GET() == 6.638213E37F);
            assert(pack.flow_x_GET() == (short)26326);
            assert(pack.quality_GET() == (char)41);
        });
        OPTICAL_FLOW p100 = new OPTICAL_FLOW();
        PH.setPack(p100);
        p100.flow_rate_y_SET(3.935194E37F, PH) ;
        p100.time_usec_SET(6008191226748134753L) ;
        p100.flow_y_SET((short)8770) ;
        p100.flow_rate_x_SET(-2.3354858E38F, PH) ;
        p100.flow_x_SET((short)26326) ;
        p100.ground_distance_SET(-2.0263273E38F) ;
        p100.sensor_id_SET((char)208) ;
        p100.quality_SET((char)41) ;
        p100.flow_comp_m_y_SET(-2.937465E37F) ;
        p100.flow_comp_m_x_SET(6.638213E37F) ;
        TestChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -2.822195E37F);
            assert(pack.roll_GET() == -2.6122563E38F);
            assert(pack.yaw_GET() == 4.469678E37F);
            assert(pack.pitch_GET() == 3.1324819E38F);
            assert(pack.y_GET() == 2.9326898E38F);
            assert(pack.usec_GET() == 7663672036484758151L);
            assert(pack.z_GET() == -7.3565496E37F);
        });
        GLOBAL_VISION_POSITION_ESTIMATE p101 = new GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.roll_SET(-2.6122563E38F) ;
        p101.usec_SET(7663672036484758151L) ;
        p101.z_SET(-7.3565496E37F) ;
        p101.x_SET(-2.822195E37F) ;
        p101.y_SET(2.9326898E38F) ;
        p101.pitch_SET(3.1324819E38F) ;
        p101.yaw_SET(4.469678E37F) ;
        TestChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -1.2492019E38F);
            assert(pack.yaw_GET() == -2.1356775E38F);
            assert(pack.x_GET() == 3.2877137E38F);
            assert(pack.usec_GET() == 6596162040665206807L);
            assert(pack.roll_GET() == 1.473921E38F);
            assert(pack.z_GET() == 3.1205858E38F);
            assert(pack.y_GET() == -1.2338286E38F);
        });
        VISION_POSITION_ESTIMATE p102 = new VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.yaw_SET(-2.1356775E38F) ;
        p102.z_SET(3.1205858E38F) ;
        p102.usec_SET(6596162040665206807L) ;
        p102.roll_SET(1.473921E38F) ;
        p102.x_SET(3.2877137E38F) ;
        p102.y_SET(-1.2338286E38F) ;
        p102.pitch_SET(-1.2492019E38F) ;
        TestChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 1.8501166E38F);
            assert(pack.usec_GET() == 974856974202851864L);
            assert(pack.y_GET() == -1.1813734E38F);
            assert(pack.x_GET() == 2.0837835E38F);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(974856974202851864L) ;
        p103.x_SET(2.0837835E38F) ;
        p103.z_SET(1.8501166E38F) ;
        p103.y_SET(-1.1813734E38F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -7.4486515E37F);
            assert(pack.z_GET() == 1.8829838E38F);
            assert(pack.roll_GET() == -1.4084209E38F);
            assert(pack.pitch_GET() == -1.6614926E38F);
            assert(pack.usec_GET() == 2914908128630584551L);
            assert(pack.x_GET() == 3.3635936E38F);
            assert(pack.y_GET() == -2.8170523E38F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.yaw_SET(-7.4486515E37F) ;
        p104.x_SET(3.3635936E38F) ;
        p104.roll_SET(-1.4084209E38F) ;
        p104.y_SET(-2.8170523E38F) ;
        p104.z_SET(1.8829838E38F) ;
        p104.pitch_SET(-1.6614926E38F) ;
        p104.usec_SET(2914908128630584551L) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.abs_pressure_GET() == -2.4232829E38F);
            assert(pack.zacc_GET() == 3.259326E38F);
            assert(pack.ymag_GET() == 2.2145596E38F);
            assert(pack.time_usec_GET() == 7643835716227979186L);
            assert(pack.fields_updated_GET() == (char)36677);
            assert(pack.zgyro_GET() == 9.417082E37F);
            assert(pack.yacc_GET() == 1.6641425E38F);
            assert(pack.pressure_alt_GET() == -1.5037261E38F);
            assert(pack.ygyro_GET() == 2.6907464E38F);
            assert(pack.zmag_GET() == 4.2312658E37F);
            assert(pack.xgyro_GET() == 2.9985356E38F);
            assert(pack.xacc_GET() == 8.807635E37F);
            assert(pack.temperature_GET() == 4.5574686E37F);
            assert(pack.xmag_GET() == -1.712492E38F);
            assert(pack.diff_pressure_GET() == -9.451201E37F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.ymag_SET(2.2145596E38F) ;
        p105.abs_pressure_SET(-2.4232829E38F) ;
        p105.time_usec_SET(7643835716227979186L) ;
        p105.xgyro_SET(2.9985356E38F) ;
        p105.fields_updated_SET((char)36677) ;
        p105.zacc_SET(3.259326E38F) ;
        p105.zmag_SET(4.2312658E37F) ;
        p105.xmag_SET(-1.712492E38F) ;
        p105.ygyro_SET(2.6907464E38F) ;
        p105.temperature_SET(4.5574686E37F) ;
        p105.pressure_alt_SET(-1.5037261E38F) ;
        p105.yacc_SET(1.6641425E38F) ;
        p105.diff_pressure_SET(-9.451201E37F) ;
        p105.xacc_SET(8.807635E37F) ;
        p105.zgyro_SET(9.417082E37F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integration_time_us_GET() == 2990915111L);
            assert(pack.time_delta_distance_us_GET() == 1243128866L);
            assert(pack.integrated_zgyro_GET() == -7.415064E37F);
            assert(pack.distance_GET() == 1.6845687E38F);
            assert(pack.temperature_GET() == (short)11215);
            assert(pack.integrated_x_GET() == 5.824875E37F);
            assert(pack.sensor_id_GET() == (char)142);
            assert(pack.integrated_ygyro_GET() == -4.0046203E37F);
            assert(pack.time_usec_GET() == 8426851019019454288L);
            assert(pack.quality_GET() == (char)218);
            assert(pack.integrated_xgyro_GET() == -2.7693614E38F);
            assert(pack.integrated_y_GET() == -2.5763743E38F);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.sensor_id_SET((char)142) ;
        p106.quality_SET((char)218) ;
        p106.time_delta_distance_us_SET(1243128866L) ;
        p106.integrated_x_SET(5.824875E37F) ;
        p106.time_usec_SET(8426851019019454288L) ;
        p106.integrated_ygyro_SET(-4.0046203E37F) ;
        p106.temperature_SET((short)11215) ;
        p106.integration_time_us_SET(2990915111L) ;
        p106.distance_SET(1.6845687E38F) ;
        p106.integrated_zgyro_SET(-7.415064E37F) ;
        p106.integrated_xgyro_SET(-2.7693614E38F) ;
        p106.integrated_y_SET(-2.5763743E38F) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == -2.812464E38F);
            assert(pack.zgyro_GET() == -1.4603296E37F);
            assert(pack.pressure_alt_GET() == 2.5907162E38F);
            assert(pack.fields_updated_GET() == 2351702756L);
            assert(pack.xgyro_GET() == 1.1703221E38F);
            assert(pack.diff_pressure_GET() == 7.2797513E37F);
            assert(pack.zacc_GET() == -1.2464407E38F);
            assert(pack.temperature_GET() == -2.4619042E38F);
            assert(pack.ymag_GET() == -3.869103E37F);
            assert(pack.time_usec_GET() == 6403821419128724046L);
            assert(pack.xmag_GET() == -8.663687E37F);
            assert(pack.abs_pressure_GET() == 2.7725752E38F);
            assert(pack.xacc_GET() == -1.1485219E38F);
            assert(pack.ygyro_GET() == 2.8053148E38F);
            assert(pack.zmag_GET() == -6.361644E37F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.ygyro_SET(2.8053148E38F) ;
        p107.diff_pressure_SET(7.2797513E37F) ;
        p107.zmag_SET(-6.361644E37F) ;
        p107.ymag_SET(-3.869103E37F) ;
        p107.temperature_SET(-2.4619042E38F) ;
        p107.fields_updated_SET(2351702756L) ;
        p107.yacc_SET(-2.812464E38F) ;
        p107.xmag_SET(-8.663687E37F) ;
        p107.xgyro_SET(1.1703221E38F) ;
        p107.time_usec_SET(6403821419128724046L) ;
        p107.zgyro_SET(-1.4603296E37F) ;
        p107.abs_pressure_SET(2.7725752E38F) ;
        p107.zacc_SET(-1.2464407E38F) ;
        p107.pressure_alt_SET(2.5907162E38F) ;
        p107.xacc_SET(-1.1485219E38F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.vd_GET() == 1.6274594E38F);
            assert(pack.q3_GET() == -6.2423015E37F);
            assert(pack.yacc_GET() == 2.2159068E38F);
            assert(pack.zgyro_GET() == 7.279679E37F);
            assert(pack.zacc_GET() == 2.153247E38F);
            assert(pack.std_dev_horz_GET() == -3.308626E38F);
            assert(pack.alt_GET() == 9.922833E37F);
            assert(pack.xgyro_GET() == -2.7228806E38F);
            assert(pack.xacc_GET() == -3.0867116E38F);
            assert(pack.ve_GET() == 3.2604947E38F);
            assert(pack.std_dev_vert_GET() == -1.7911915E38F);
            assert(pack.roll_GET() == 7.813935E37F);
            assert(pack.q1_GET() == -2.1844234E38F);
            assert(pack.lon_GET() == -2.806685E38F);
            assert(pack.yaw_GET() == 1.8853481E38F);
            assert(pack.q4_GET() == 5.69998E36F);
            assert(pack.vn_GET() == 1.1031779E37F);
            assert(pack.pitch_GET() == -1.3366331E38F);
            assert(pack.ygyro_GET() == 7.076585E37F);
            assert(pack.q2_GET() == 2.6709082E38F);
            assert(pack.lat_GET() == -2.3195538E38F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.pitch_SET(-1.3366331E38F) ;
        p108.lat_SET(-2.3195538E38F) ;
        p108.roll_SET(7.813935E37F) ;
        p108.zacc_SET(2.153247E38F) ;
        p108.vn_SET(1.1031779E37F) ;
        p108.ve_SET(3.2604947E38F) ;
        p108.xacc_SET(-3.0867116E38F) ;
        p108.lon_SET(-2.806685E38F) ;
        p108.xgyro_SET(-2.7228806E38F) ;
        p108.yaw_SET(1.8853481E38F) ;
        p108.q4_SET(5.69998E36F) ;
        p108.alt_SET(9.922833E37F) ;
        p108.zgyro_SET(7.279679E37F) ;
        p108.q3_SET(-6.2423015E37F) ;
        p108.ygyro_SET(7.076585E37F) ;
        p108.yacc_SET(2.2159068E38F) ;
        p108.vd_SET(1.6274594E38F) ;
        p108.std_dev_vert_SET(-1.7911915E38F) ;
        p108.std_dev_horz_SET(-3.308626E38F) ;
        p108.q2_SET(2.6709082E38F) ;
        p108.q1_SET(-2.1844234E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.fixed__GET() == (char)56750);
            assert(pack.rxerrors_GET() == (char)4752);
            assert(pack.noise_GET() == (char)162);
            assert(pack.txbuf_GET() == (char)117);
            assert(pack.rssi_GET() == (char)6);
            assert(pack.remnoise_GET() == (char)235);
            assert(pack.remrssi_GET() == (char)188);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.remrssi_SET((char)188) ;
        p109.noise_SET((char)162) ;
        p109.txbuf_SET((char)117) ;
        p109.fixed__SET((char)56750) ;
        p109.remnoise_SET((char)235) ;
        p109.rssi_SET((char)6) ;
        p109.rxerrors_SET((char)4752) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)98);
            assert(pack.target_network_GET() == (char)1);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)160, (char)187, (char)57, (char)99, (char)77, (char)236, (char)152, (char)186, (char)45, (char)116, (char)72, (char)221, (char)72, (char)111, (char)140, (char)222, (char)91, (char)197, (char)4, (char)248, (char)156, (char)46, (char)3, (char)96, (char)170, (char)0, (char)141, (char)144, (char)181, (char)74, (char)25, (char)51, (char)71, (char)161, (char)71, (char)224, (char)52, (char)4, (char)204, (char)108, (char)247, (char)251, (char)208, (char)247, (char)145, (char)75, (char)196, (char)98, (char)222, (char)123, (char)188, (char)229, (char)98, (char)56, (char)190, (char)142, (char)50, (char)105, (char)125, (char)76, (char)207, (char)73, (char)21, (char)211, (char)198, (char)87, (char)205, (char)58, (char)233, (char)238, (char)105, (char)180, (char)187, (char)53, (char)143, (char)8, (char)192, (char)93, (char)237, (char)108, (char)6, (char)90, (char)69, (char)81, (char)32, (char)76, (char)245, (char)174, (char)125, (char)73, (char)228, (char)11, (char)179, (char)166, (char)194, (char)215, (char)59, (char)224, (char)131, (char)105, (char)51, (char)142, (char)133, (char)223, (char)159, (char)168, (char)174, (char)112, (char)99, (char)196, (char)130, (char)114, (char)140, (char)150, (char)236, (char)76, (char)26, (char)71, (char)193, (char)80, (char)113, (char)44, (char)120, (char)72, (char)227, (char)176, (char)150, (char)158, (char)210, (char)134, (char)60, (char)16, (char)69, (char)128, (char)172, (char)161, (char)150, (char)51, (char)98, (char)176, (char)72, (char)21, (char)255, (char)155, (char)103, (char)171, (char)32, (char)45, (char)118, (char)52, (char)0, (char)223, (char)157, (char)45, (char)197, (char)93, (char)83, (char)219, (char)169, (char)70, (char)38, (char)43, (char)48, (char)20, (char)154, (char)229, (char)113, (char)18, (char)247, (char)165, (char)74, (char)6, (char)66, (char)235, (char)142, (char)25, (char)25, (char)222, (char)246, (char)46, (char)135, (char)97, (char)18, (char)176, (char)69, (char)138, (char)178, (char)77, (char)105, (char)176, (char)168, (char)236, (char)135, (char)87, (char)109, (char)170, (char)54, (char)247, (char)220, (char)251, (char)81, (char)16, (char)51, (char)24, (char)253, (char)40, (char)115, (char)209, (char)198, (char)34, (char)176, (char)17, (char)126, (char)93, (char)6, (char)26, (char)16, (char)40, (char)229, (char)131, (char)51, (char)179, (char)60, (char)121, (char)237, (char)179, (char)168, (char)80, (char)78, (char)127, (char)60, (char)239, (char)140, (char)203, (char)88, (char)236, (char)59, (char)135, (char)181, (char)164, (char)34, (char)8, (char)118, (char)114, (char)37, (char)244, (char)138, (char)157, (char)253, (char)151, (char)103}));
            assert(pack.target_system_GET() == (char)141);
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_component_SET((char)98) ;
        p110.payload_SET(new char[] {(char)160, (char)187, (char)57, (char)99, (char)77, (char)236, (char)152, (char)186, (char)45, (char)116, (char)72, (char)221, (char)72, (char)111, (char)140, (char)222, (char)91, (char)197, (char)4, (char)248, (char)156, (char)46, (char)3, (char)96, (char)170, (char)0, (char)141, (char)144, (char)181, (char)74, (char)25, (char)51, (char)71, (char)161, (char)71, (char)224, (char)52, (char)4, (char)204, (char)108, (char)247, (char)251, (char)208, (char)247, (char)145, (char)75, (char)196, (char)98, (char)222, (char)123, (char)188, (char)229, (char)98, (char)56, (char)190, (char)142, (char)50, (char)105, (char)125, (char)76, (char)207, (char)73, (char)21, (char)211, (char)198, (char)87, (char)205, (char)58, (char)233, (char)238, (char)105, (char)180, (char)187, (char)53, (char)143, (char)8, (char)192, (char)93, (char)237, (char)108, (char)6, (char)90, (char)69, (char)81, (char)32, (char)76, (char)245, (char)174, (char)125, (char)73, (char)228, (char)11, (char)179, (char)166, (char)194, (char)215, (char)59, (char)224, (char)131, (char)105, (char)51, (char)142, (char)133, (char)223, (char)159, (char)168, (char)174, (char)112, (char)99, (char)196, (char)130, (char)114, (char)140, (char)150, (char)236, (char)76, (char)26, (char)71, (char)193, (char)80, (char)113, (char)44, (char)120, (char)72, (char)227, (char)176, (char)150, (char)158, (char)210, (char)134, (char)60, (char)16, (char)69, (char)128, (char)172, (char)161, (char)150, (char)51, (char)98, (char)176, (char)72, (char)21, (char)255, (char)155, (char)103, (char)171, (char)32, (char)45, (char)118, (char)52, (char)0, (char)223, (char)157, (char)45, (char)197, (char)93, (char)83, (char)219, (char)169, (char)70, (char)38, (char)43, (char)48, (char)20, (char)154, (char)229, (char)113, (char)18, (char)247, (char)165, (char)74, (char)6, (char)66, (char)235, (char)142, (char)25, (char)25, (char)222, (char)246, (char)46, (char)135, (char)97, (char)18, (char)176, (char)69, (char)138, (char)178, (char)77, (char)105, (char)176, (char)168, (char)236, (char)135, (char)87, (char)109, (char)170, (char)54, (char)247, (char)220, (char)251, (char)81, (char)16, (char)51, (char)24, (char)253, (char)40, (char)115, (char)209, (char)198, (char)34, (char)176, (char)17, (char)126, (char)93, (char)6, (char)26, (char)16, (char)40, (char)229, (char)131, (char)51, (char)179, (char)60, (char)121, (char)237, (char)179, (char)168, (char)80, (char)78, (char)127, (char)60, (char)239, (char)140, (char)203, (char)88, (char)236, (char)59, (char)135, (char)181, (char)164, (char)34, (char)8, (char)118, (char)114, (char)37, (char)244, (char)138, (char)157, (char)253, (char)151, (char)103}, 0) ;
        p110.target_network_SET((char)1) ;
        p110.target_system_SET((char)141) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.ts1_GET() == -4662826546030772771L);
            assert(pack.tc1_GET() == -3161937056179924847L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(-4662826546030772771L) ;
        p111.tc1_SET(-3161937056179924847L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 382243377L);
            assert(pack.time_usec_GET() == 3440817694534675681L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(382243377L) ;
        p112.time_usec_SET(3440817694534675681L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.eph_GET() == (char)7939);
            assert(pack.time_usec_GET() == 7803409591813934470L);
            assert(pack.ve_GET() == (short) -31284);
            assert(pack.satellites_visible_GET() == (char)44);
            assert(pack.vd_GET() == (short)18706);
            assert(pack.lon_GET() == 1660506236);
            assert(pack.vn_GET() == (short) -8578);
            assert(pack.cog_GET() == (char)34154);
            assert(pack.lat_GET() == -364173364);
            assert(pack.vel_GET() == (char)59054);
            assert(pack.fix_type_GET() == (char)104);
            assert(pack.epv_GET() == (char)36007);
            assert(pack.alt_GET() == 723033426);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.ve_SET((short) -31284) ;
        p113.vn_SET((short) -8578) ;
        p113.lon_SET(1660506236) ;
        p113.vd_SET((short)18706) ;
        p113.eph_SET((char)7939) ;
        p113.cog_SET((char)34154) ;
        p113.alt_SET(723033426) ;
        p113.time_usec_SET(7803409591813934470L) ;
        p113.vel_SET((char)59054) ;
        p113.lat_SET(-364173364) ;
        p113.fix_type_SET((char)104) ;
        p113.epv_SET((char)36007) ;
        p113.satellites_visible_SET((char)44) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integrated_x_GET() == 2.9768645E38F);
            assert(pack.integrated_xgyro_GET() == 2.0001328E38F);
            assert(pack.distance_GET() == -1.2354745E38F);
            assert(pack.integrated_ygyro_GET() == 2.5156684E38F);
            assert(pack.temperature_GET() == (short)28855);
            assert(pack.time_delta_distance_us_GET() == 2440127614L);
            assert(pack.integrated_y_GET() == 2.3752E38F);
            assert(pack.time_usec_GET() == 837177316504395781L);
            assert(pack.integration_time_us_GET() == 900691633L);
            assert(pack.sensor_id_GET() == (char)255);
            assert(pack.integrated_zgyro_GET() == 7.4004996E37F);
            assert(pack.quality_GET() == (char)93);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_usec_SET(837177316504395781L) ;
        p114.distance_SET(-1.2354745E38F) ;
        p114.quality_SET((char)93) ;
        p114.integrated_zgyro_SET(7.4004996E37F) ;
        p114.integration_time_us_SET(900691633L) ;
        p114.time_delta_distance_us_SET(2440127614L) ;
        p114.integrated_xgyro_SET(2.0001328E38F) ;
        p114.integrated_ygyro_SET(2.5156684E38F) ;
        p114.sensor_id_SET((char)255) ;
        p114.integrated_y_SET(2.3752E38F) ;
        p114.temperature_SET((short)28855) ;
        p114.integrated_x_SET(2.9768645E38F) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.ind_airspeed_GET() == (char)390);
            assert(pack.zacc_GET() == (short)9078);
            assert(pack.pitchspeed_GET() == -2.8295401E38F);
            assert(pack.yawspeed_GET() == -1.4351889E38F);
            assert(pack.lon_GET() == 1997883523);
            assert(pack.lat_GET() == -1643383281);
            assert(pack.vy_GET() == (short)20149);
            assert(pack.vx_GET() == (short)25452);
            assert(pack.xacc_GET() == (short) -31772);
            assert(pack.alt_GET() == 1391019134);
            assert(pack.vz_GET() == (short)14504);
            assert(pack.time_usec_GET() == 6845395852375777530L);
            assert(pack.rollspeed_GET() == -6.533835E37F);
            assert(pack.true_airspeed_GET() == (char)39032);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {1.29921E38F, -1.5066505E38F, -3.6682817E37F, 2.1074717E37F}));
            assert(pack.yacc_GET() == (short) -7144);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.true_airspeed_SET((char)39032) ;
        p115.ind_airspeed_SET((char)390) ;
        p115.yacc_SET((short) -7144) ;
        p115.time_usec_SET(6845395852375777530L) ;
        p115.lon_SET(1997883523) ;
        p115.vz_SET((short)14504) ;
        p115.yawspeed_SET(-1.4351889E38F) ;
        p115.xacc_SET((short) -31772) ;
        p115.rollspeed_SET(-6.533835E37F) ;
        p115.attitude_quaternion_SET(new float[] {1.29921E38F, -1.5066505E38F, -3.6682817E37F, 2.1074717E37F}, 0) ;
        p115.vx_SET((short)25452) ;
        p115.pitchspeed_SET(-2.8295401E38F) ;
        p115.alt_SET(1391019134) ;
        p115.zacc_SET((short)9078) ;
        p115.lat_SET(-1643383281) ;
        p115.vy_SET((short)20149) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short)22203);
            assert(pack.xacc_GET() == (short)14240);
            assert(pack.zacc_GET() == (short) -5788);
            assert(pack.xmag_GET() == (short)17833);
            assert(pack.zmag_GET() == (short) -26817);
            assert(pack.yacc_GET() == (short) -5103);
            assert(pack.xgyro_GET() == (short) -27925);
            assert(pack.time_boot_ms_GET() == 4090495561L);
            assert(pack.zgyro_GET() == (short) -17792);
            assert(pack.ymag_GET() == (short) -28925);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.xmag_SET((short)17833) ;
        p116.time_boot_ms_SET(4090495561L) ;
        p116.zmag_SET((short) -26817) ;
        p116.ymag_SET((short) -28925) ;
        p116.xacc_SET((short)14240) ;
        p116.yacc_SET((short) -5103) ;
        p116.zacc_SET((short) -5788) ;
        p116.xgyro_SET((short) -27925) ;
        p116.zgyro_SET((short) -17792) ;
        p116.ygyro_SET((short)22203) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_GET() == (char)19355);
            assert(pack.end_GET() == (char)42060);
            assert(pack.target_component_GET() == (char)18);
            assert(pack.target_system_GET() == (char)2);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)2) ;
        p117.target_component_SET((char)18) ;
        p117.end_SET((char)42060) ;
        p117.start_SET((char)19355) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.time_utc_GET() == 2462958510L);
            assert(pack.size_GET() == 414678745L);
            assert(pack.num_logs_GET() == (char)51401);
            assert(pack.id_GET() == (char)32641);
            assert(pack.last_log_num_GET() == (char)51324);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)32641) ;
        p118.time_utc_SET(2462958510L) ;
        p118.num_logs_SET((char)51401) ;
        p118.last_log_num_SET((char)51324) ;
        p118.size_SET(414678745L) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)18);
            assert(pack.id_GET() == (char)13214);
            assert(pack.count_GET() == 1316008063L);
            assert(pack.ofs_GET() == 1864535618L);
            assert(pack.target_system_GET() == (char)55);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.ofs_SET(1864535618L) ;
        p119.target_component_SET((char)18) ;
        p119.count_SET(1316008063L) ;
        p119.id_SET((char)13214) ;
        p119.target_system_SET((char)55) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.ofs_GET() == 786160215L);
            assert(pack.count_GET() == (char)86);
            assert(pack.id_GET() == (char)6037);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)159, (char)79, (char)87, (char)155, (char)238, (char)184, (char)227, (char)194, (char)144, (char)81, (char)56, (char)112, (char)222, (char)71, (char)219, (char)78, (char)32, (char)74, (char)56, (char)79, (char)164, (char)179, (char)117, (char)150, (char)85, (char)247, (char)186, (char)244, (char)43, (char)224, (char)219, (char)197, (char)14, (char)115, (char)47, (char)204, (char)74, (char)81, (char)87, (char)141, (char)13, (char)115, (char)96, (char)212, (char)245, (char)252, (char)182, (char)73, (char)255, (char)201, (char)212, (char)92, (char)215, (char)196, (char)8, (char)197, (char)165, (char)186, (char)6, (char)167, (char)157, (char)23, (char)3, (char)186, (char)246, (char)54, (char)202, (char)165, (char)113, (char)251, (char)18, (char)124, (char)200, (char)161, (char)93, (char)204, (char)7, (char)49, (char)207, (char)85, (char)197, (char)153, (char)44, (char)43, (char)30, (char)100, (char)80, (char)238, (char)243, (char)208}));
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.data__SET(new char[] {(char)159, (char)79, (char)87, (char)155, (char)238, (char)184, (char)227, (char)194, (char)144, (char)81, (char)56, (char)112, (char)222, (char)71, (char)219, (char)78, (char)32, (char)74, (char)56, (char)79, (char)164, (char)179, (char)117, (char)150, (char)85, (char)247, (char)186, (char)244, (char)43, (char)224, (char)219, (char)197, (char)14, (char)115, (char)47, (char)204, (char)74, (char)81, (char)87, (char)141, (char)13, (char)115, (char)96, (char)212, (char)245, (char)252, (char)182, (char)73, (char)255, (char)201, (char)212, (char)92, (char)215, (char)196, (char)8, (char)197, (char)165, (char)186, (char)6, (char)167, (char)157, (char)23, (char)3, (char)186, (char)246, (char)54, (char)202, (char)165, (char)113, (char)251, (char)18, (char)124, (char)200, (char)161, (char)93, (char)204, (char)7, (char)49, (char)207, (char)85, (char)197, (char)153, (char)44, (char)43, (char)30, (char)100, (char)80, (char)238, (char)243, (char)208}, 0) ;
        p120.count_SET((char)86) ;
        p120.id_SET((char)6037) ;
        p120.ofs_SET(786160215L) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)144);
            assert(pack.target_system_GET() == (char)157);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)157) ;
        p121.target_component_SET((char)144) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)224);
            assert(pack.target_system_GET() == (char)126);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_component_SET((char)224) ;
        p122.target_system_SET((char)126) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)124, (char)8, (char)12, (char)122, (char)19, (char)146, (char)203, (char)63, (char)121, (char)79, (char)160, (char)47, (char)35, (char)99, (char)146, (char)229, (char)206, (char)7, (char)116, (char)159, (char)223, (char)109, (char)254, (char)73, (char)65, (char)139, (char)8, (char)246, (char)170, (char)236, (char)165, (char)134, (char)46, (char)213, (char)239, (char)47, (char)99, (char)215, (char)24, (char)184, (char)23, (char)197, (char)198, (char)59, (char)183, (char)232, (char)84, (char)227, (char)148, (char)67, (char)89, (char)246, (char)145, (char)249, (char)5, (char)102, (char)237, (char)174, (char)57, (char)241, (char)38, (char)168, (char)142, (char)145, (char)19, (char)106, (char)184, (char)184, (char)78, (char)176, (char)255, (char)53, (char)196, (char)23, (char)73, (char)9, (char)226, (char)9, (char)174, (char)28, (char)71, (char)169, (char)117, (char)4, (char)192, (char)35, (char)202, (char)230, (char)127, (char)190, (char)195, (char)180, (char)17, (char)96, (char)168, (char)177, (char)201, (char)200, (char)163, (char)154, (char)151, (char)2, (char)81, (char)50, (char)85, (char)67, (char)187, (char)39, (char)86, (char)17}));
            assert(pack.target_component_GET() == (char)206);
            assert(pack.target_system_GET() == (char)202);
            assert(pack.len_GET() == (char)134);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_component_SET((char)206) ;
        p123.data__SET(new char[] {(char)124, (char)8, (char)12, (char)122, (char)19, (char)146, (char)203, (char)63, (char)121, (char)79, (char)160, (char)47, (char)35, (char)99, (char)146, (char)229, (char)206, (char)7, (char)116, (char)159, (char)223, (char)109, (char)254, (char)73, (char)65, (char)139, (char)8, (char)246, (char)170, (char)236, (char)165, (char)134, (char)46, (char)213, (char)239, (char)47, (char)99, (char)215, (char)24, (char)184, (char)23, (char)197, (char)198, (char)59, (char)183, (char)232, (char)84, (char)227, (char)148, (char)67, (char)89, (char)246, (char)145, (char)249, (char)5, (char)102, (char)237, (char)174, (char)57, (char)241, (char)38, (char)168, (char)142, (char)145, (char)19, (char)106, (char)184, (char)184, (char)78, (char)176, (char)255, (char)53, (char)196, (char)23, (char)73, (char)9, (char)226, (char)9, (char)174, (char)28, (char)71, (char)169, (char)117, (char)4, (char)192, (char)35, (char)202, (char)230, (char)127, (char)190, (char)195, (char)180, (char)17, (char)96, (char)168, (char)177, (char)201, (char)200, (char)163, (char)154, (char)151, (char)2, (char)81, (char)50, (char)85, (char)67, (char)187, (char)39, (char)86, (char)17}, 0) ;
        p123.target_system_SET((char)202) ;
        p123.len_SET((char)134) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.cog_GET() == (char)48000);
            assert(pack.dgps_numch_GET() == (char)77);
            assert(pack.epv_GET() == (char)34930);
            assert(pack.eph_GET() == (char)16557);
            assert(pack.lon_GET() == -648281562);
            assert(pack.dgps_age_GET() == 3280380055L);
            assert(pack.vel_GET() == (char)46879);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
            assert(pack.satellites_visible_GET() == (char)46);
            assert(pack.lat_GET() == -684730123);
            assert(pack.alt_GET() == -60813674);
            assert(pack.time_usec_GET() == 5240715374485740854L);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.alt_SET(-60813674) ;
        p124.lat_SET(-684730123) ;
        p124.dgps_age_SET(3280380055L) ;
        p124.lon_SET(-648281562) ;
        p124.satellites_visible_SET((char)46) ;
        p124.vel_SET((char)46879) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX) ;
        p124.time_usec_SET(5240715374485740854L) ;
        p124.dgps_numch_SET((char)77) ;
        p124.epv_SET((char)34930) ;
        p124.cog_SET((char)48000) ;
        p124.eph_SET((char)16557) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID));
            assert(pack.Vservo_GET() == (char)43597);
            assert(pack.Vcc_GET() == (char)14034);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)14034) ;
        p125.Vservo_SET((char)43597) ;
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID)) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)88, (char)197, (char)24, (char)24, (char)132, (char)217, (char)175, (char)176, (char)245, (char)5, (char)46, (char)101, (char)115, (char)59, (char)42, (char)94, (char)88, (char)135, (char)62, (char)53, (char)134, (char)95, (char)168, (char)176, (char)175, (char)135, (char)49, (char)56, (char)85, (char)196, (char)92, (char)151, (char)134, (char)219, (char)188, (char)30, (char)44, (char)220, (char)86, (char)64, (char)87, (char)195, (char)206, (char)116, (char)224, (char)57, (char)193, (char)15, (char)13, (char)0, (char)248, (char)160, (char)185, (char)239, (char)97, (char)56, (char)185, (char)170, (char)34, (char)174, (char)122, (char)135, (char)102, (char)62, (char)190, (char)64, (char)74, (char)222, (char)206, (char)77}));
            assert(pack.count_GET() == (char)117);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
            assert(pack.timeout_GET() == (char)63620);
            assert(pack.baudrate_GET() == 2053486300L);
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING));
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL) ;
        p126.baudrate_SET(2053486300L) ;
        p126.count_SET((char)117) ;
        p126.data__SET(new char[] {(char)88, (char)197, (char)24, (char)24, (char)132, (char)217, (char)175, (char)176, (char)245, (char)5, (char)46, (char)101, (char)115, (char)59, (char)42, (char)94, (char)88, (char)135, (char)62, (char)53, (char)134, (char)95, (char)168, (char)176, (char)175, (char)135, (char)49, (char)56, (char)85, (char)196, (char)92, (char)151, (char)134, (char)219, (char)188, (char)30, (char)44, (char)220, (char)86, (char)64, (char)87, (char)195, (char)206, (char)116, (char)224, (char)57, (char)193, (char)15, (char)13, (char)0, (char)248, (char)160, (char)185, (char)239, (char)97, (char)56, (char)185, (char)170, (char)34, (char)174, (char)122, (char)135, (char)102, (char)62, (char)190, (char)64, (char)74, (char)222, (char)206, (char)77}, 0) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING)) ;
        p126.timeout_SET((char)63620) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.iar_num_hypotheses_GET() == 739634283);
            assert(pack.rtk_receiver_id_GET() == (char)75);
            assert(pack.baseline_b_mm_GET() == 964387954);
            assert(pack.tow_GET() == 1880021381L);
            assert(pack.baseline_c_mm_GET() == 1131468632);
            assert(pack.baseline_a_mm_GET() == -35590822);
            assert(pack.baseline_coords_type_GET() == (char)195);
            assert(pack.wn_GET() == (char)57354);
            assert(pack.accuracy_GET() == 4089975119L);
            assert(pack.nsats_GET() == (char)253);
            assert(pack.rtk_rate_GET() == (char)165);
            assert(pack.time_last_baseline_ms_GET() == 1142475964L);
            assert(pack.rtk_health_GET() == (char)228);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.iar_num_hypotheses_SET(739634283) ;
        p127.rtk_receiver_id_SET((char)75) ;
        p127.tow_SET(1880021381L) ;
        p127.nsats_SET((char)253) ;
        p127.accuracy_SET(4089975119L) ;
        p127.baseline_c_mm_SET(1131468632) ;
        p127.rtk_rate_SET((char)165) ;
        p127.wn_SET((char)57354) ;
        p127.baseline_a_mm_SET(-35590822) ;
        p127.baseline_coords_type_SET((char)195) ;
        p127.rtk_health_SET((char)228) ;
        p127.time_last_baseline_ms_SET(1142475964L) ;
        p127.baseline_b_mm_SET(964387954) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_c_mm_GET() == -891629125);
            assert(pack.iar_num_hypotheses_GET() == 777925164);
            assert(pack.wn_GET() == (char)14738);
            assert(pack.rtk_rate_GET() == (char)132);
            assert(pack.baseline_coords_type_GET() == (char)253);
            assert(pack.rtk_health_GET() == (char)236);
            assert(pack.baseline_a_mm_GET() == -1459144725);
            assert(pack.nsats_GET() == (char)115);
            assert(pack.accuracy_GET() == 4086810019L);
            assert(pack.baseline_b_mm_GET() == -1730026189);
            assert(pack.rtk_receiver_id_GET() == (char)190);
            assert(pack.time_last_baseline_ms_GET() == 2601532036L);
            assert(pack.tow_GET() == 2054840684L);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.rtk_receiver_id_SET((char)190) ;
        p128.baseline_b_mm_SET(-1730026189) ;
        p128.rtk_rate_SET((char)132) ;
        p128.baseline_coords_type_SET((char)253) ;
        p128.accuracy_SET(4086810019L) ;
        p128.tow_SET(2054840684L) ;
        p128.time_last_baseline_ms_SET(2601532036L) ;
        p128.rtk_health_SET((char)236) ;
        p128.baseline_c_mm_SET(-891629125) ;
        p128.baseline_a_mm_SET(-1459144725) ;
        p128.wn_SET((char)14738) ;
        p128.iar_num_hypotheses_SET(777925164) ;
        p128.nsats_SET((char)115) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1234543703L);
            assert(pack.xmag_GET() == (short) -18337);
            assert(pack.ymag_GET() == (short) -6322);
            assert(pack.xacc_GET() == (short)32004);
            assert(pack.zmag_GET() == (short) -14725);
            assert(pack.xgyro_GET() == (short) -16187);
            assert(pack.yacc_GET() == (short)23884);
            assert(pack.ygyro_GET() == (short)17528);
            assert(pack.zacc_GET() == (short) -9295);
            assert(pack.zgyro_GET() == (short) -10096);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.xacc_SET((short)32004) ;
        p129.ygyro_SET((short)17528) ;
        p129.yacc_SET((short)23884) ;
        p129.zgyro_SET((short) -10096) ;
        p129.xgyro_SET((short) -16187) ;
        p129.xmag_SET((short) -18337) ;
        p129.zacc_SET((short) -9295) ;
        p129.ymag_SET((short) -6322) ;
        p129.zmag_SET((short) -14725) ;
        p129.time_boot_ms_SET(1234543703L) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.payload_GET() == (char)140);
            assert(pack.jpg_quality_GET() == (char)78);
            assert(pack.packets_GET() == (char)28929);
            assert(pack.height_GET() == (char)63541);
            assert(pack.type_GET() == (char)115);
            assert(pack.size_GET() == 216874141L);
            assert(pack.width_GET() == (char)9819);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.packets_SET((char)28929) ;
        p130.size_SET(216874141L) ;
        p130.payload_SET((char)140) ;
        p130.jpg_quality_SET((char)78) ;
        p130.height_SET((char)63541) ;
        p130.width_SET((char)9819) ;
        p130.type_SET((char)115) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)16, (char)131, (char)243, (char)76, (char)202, (char)97, (char)70, (char)222, (char)101, (char)68, (char)36, (char)149, (char)72, (char)45, (char)176, (char)15, (char)25, (char)125, (char)203, (char)102, (char)108, (char)35, (char)147, (char)140, (char)171, (char)19, (char)51, (char)36, (char)65, (char)166, (char)222, (char)112, (char)47, (char)209, (char)24, (char)232, (char)242, (char)3, (char)50, (char)27, (char)227, (char)237, (char)229, (char)170, (char)174, (char)202, (char)124, (char)18, (char)96, (char)143, (char)98, (char)151, (char)199, (char)38, (char)96, (char)31, (char)182, (char)112, (char)251, (char)136, (char)80, (char)7, (char)254, (char)21, (char)120, (char)149, (char)50, (char)82, (char)121, (char)29, (char)216, (char)31, (char)212, (char)174, (char)38, (char)17, (char)42, (char)5, (char)29, (char)219, (char)143, (char)87, (char)187, (char)120, (char)125, (char)21, (char)144, (char)167, (char)254, (char)60, (char)91, (char)129, (char)180, (char)221, (char)58, (char)12, (char)243, (char)146, (char)213, (char)98, (char)86, (char)57, (char)248, (char)49, (char)177, (char)6, (char)247, (char)197, (char)166, (char)164, (char)192, (char)56, (char)188, (char)108, (char)86, (char)209, (char)163, (char)5, (char)190, (char)196, (char)31, (char)88, (char)115, (char)68, (char)109, (char)10, (char)142, (char)63, (char)222, (char)100, (char)193, (char)214, (char)82, (char)94, (char)90, (char)223, (char)85, (char)181, (char)71, (char)83, (char)220, (char)137, (char)250, (char)182, (char)248, (char)131, (char)37, (char)183, (char)93, (char)178, (char)14, (char)116, (char)128, (char)115, (char)96, (char)41, (char)0, (char)187, (char)42, (char)19, (char)94, (char)53, (char)47, (char)221, (char)68, (char)99, (char)102, (char)33, (char)234, (char)154, (char)73, (char)152, (char)25, (char)240, (char)135, (char)127, (char)79, (char)77, (char)209, (char)120, (char)78, (char)124, (char)113, (char)125, (char)201, (char)167, (char)121, (char)143, (char)33, (char)76, (char)65, (char)110, (char)43, (char)174, (char)112, (char)17, (char)25, (char)45, (char)83, (char)171, (char)196, (char)127, (char)179, (char)31, (char)70, (char)225, (char)146, (char)0, (char)77, (char)81, (char)251, (char)224, (char)181, (char)198, (char)82, (char)234, (char)30, (char)251, (char)77, (char)26, (char)3, (char)166, (char)26, (char)48, (char)229, (char)196, (char)25, (char)216, (char)36, (char)53, (char)55, (char)70, (char)25, (char)135, (char)8, (char)213, (char)86, (char)60, (char)198, (char)168, (char)101, (char)216, (char)169, (char)86, (char)12, (char)9, (char)41, (char)208, (char)229, (char)156, (char)100, (char)132, (char)99}));
            assert(pack.seqnr_GET() == (char)33532);
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)33532) ;
        p131.data__SET(new char[] {(char)16, (char)131, (char)243, (char)76, (char)202, (char)97, (char)70, (char)222, (char)101, (char)68, (char)36, (char)149, (char)72, (char)45, (char)176, (char)15, (char)25, (char)125, (char)203, (char)102, (char)108, (char)35, (char)147, (char)140, (char)171, (char)19, (char)51, (char)36, (char)65, (char)166, (char)222, (char)112, (char)47, (char)209, (char)24, (char)232, (char)242, (char)3, (char)50, (char)27, (char)227, (char)237, (char)229, (char)170, (char)174, (char)202, (char)124, (char)18, (char)96, (char)143, (char)98, (char)151, (char)199, (char)38, (char)96, (char)31, (char)182, (char)112, (char)251, (char)136, (char)80, (char)7, (char)254, (char)21, (char)120, (char)149, (char)50, (char)82, (char)121, (char)29, (char)216, (char)31, (char)212, (char)174, (char)38, (char)17, (char)42, (char)5, (char)29, (char)219, (char)143, (char)87, (char)187, (char)120, (char)125, (char)21, (char)144, (char)167, (char)254, (char)60, (char)91, (char)129, (char)180, (char)221, (char)58, (char)12, (char)243, (char)146, (char)213, (char)98, (char)86, (char)57, (char)248, (char)49, (char)177, (char)6, (char)247, (char)197, (char)166, (char)164, (char)192, (char)56, (char)188, (char)108, (char)86, (char)209, (char)163, (char)5, (char)190, (char)196, (char)31, (char)88, (char)115, (char)68, (char)109, (char)10, (char)142, (char)63, (char)222, (char)100, (char)193, (char)214, (char)82, (char)94, (char)90, (char)223, (char)85, (char)181, (char)71, (char)83, (char)220, (char)137, (char)250, (char)182, (char)248, (char)131, (char)37, (char)183, (char)93, (char)178, (char)14, (char)116, (char)128, (char)115, (char)96, (char)41, (char)0, (char)187, (char)42, (char)19, (char)94, (char)53, (char)47, (char)221, (char)68, (char)99, (char)102, (char)33, (char)234, (char)154, (char)73, (char)152, (char)25, (char)240, (char)135, (char)127, (char)79, (char)77, (char)209, (char)120, (char)78, (char)124, (char)113, (char)125, (char)201, (char)167, (char)121, (char)143, (char)33, (char)76, (char)65, (char)110, (char)43, (char)174, (char)112, (char)17, (char)25, (char)45, (char)83, (char)171, (char)196, (char)127, (char)179, (char)31, (char)70, (char)225, (char)146, (char)0, (char)77, (char)81, (char)251, (char)224, (char)181, (char)198, (char)82, (char)234, (char)30, (char)251, (char)77, (char)26, (char)3, (char)166, (char)26, (char)48, (char)229, (char)196, (char)25, (char)216, (char)36, (char)53, (char)55, (char)70, (char)25, (char)135, (char)8, (char)213, (char)86, (char)60, (char)198, (char)168, (char)101, (char)216, (char)169, (char)86, (char)12, (char)9, (char)41, (char)208, (char)229, (char)156, (char)100, (char)132, (char)99}, 0) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.covariance_GET() == (char)138);
            assert(pack.current_distance_GET() == (char)61462);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_PITCH_90);
            assert(pack.time_boot_ms_GET() == 2461535929L);
            assert(pack.id_GET() == (char)227);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            assert(pack.max_distance_GET() == (char)12988);
            assert(pack.min_distance_GET() == (char)18960);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.time_boot_ms_SET(2461535929L) ;
        p132.min_distance_SET((char)18960) ;
        p132.current_distance_SET((char)61462) ;
        p132.max_distance_SET((char)12988) ;
        p132.covariance_SET((char)138) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        p132.id_SET((char)227) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_PITCH_90) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mask_GET() == 4590942505563865444L);
            assert(pack.grid_spacing_GET() == (char)27138);
            assert(pack.lon_GET() == 1447734217);
            assert(pack.lat_GET() == -63267933);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(-63267933) ;
        p133.mask_SET(4590942505563865444L) ;
        p133.lon_SET(1447734217) ;
        p133.grid_spacing_SET((char)27138) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -20487, (short)4460, (short)8717, (short)23896, (short)31351, (short) -13994, (short)1573, (short) -29949, (short)16858, (short) -23955, (short)16109, (short)10320, (short) -4209, (short)2356, (short)22742, (short) -14767}));
            assert(pack.grid_spacing_GET() == (char)21500);
            assert(pack.lon_GET() == 2076629063);
            assert(pack.gridbit_GET() == (char)181);
            assert(pack.lat_GET() == 1033725802);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lon_SET(2076629063) ;
        p134.grid_spacing_SET((char)21500) ;
        p134.gridbit_SET((char)181) ;
        p134.data__SET(new short[] {(short) -20487, (short)4460, (short)8717, (short)23896, (short)31351, (short) -13994, (short)1573, (short) -29949, (short)16858, (short) -23955, (short)16109, (short)10320, (short) -4209, (short)2356, (short)22742, (short) -14767}, 0) ;
        p134.lat_SET(1033725802) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 375660559);
            assert(pack.lat_GET() == 6900303);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(6900303) ;
        p135.lon_SET(375660559) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.current_height_GET() == -1.5475912E38F);
            assert(pack.lat_GET() == 1706648295);
            assert(pack.loaded_GET() == (char)62744);
            assert(pack.spacing_GET() == (char)994);
            assert(pack.terrain_height_GET() == 7.413587E37F);
            assert(pack.lon_GET() == 1785447192);
            assert(pack.pending_GET() == (char)21671);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.pending_SET((char)21671) ;
        p136.spacing_SET((char)994) ;
        p136.lon_SET(1785447192) ;
        p136.current_height_SET(-1.5475912E38F) ;
        p136.loaded_SET((char)62744) ;
        p136.terrain_height_SET(7.413587E37F) ;
        p136.lat_SET(1706648295) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)1548);
            assert(pack.press_abs_GET() == 4.1146047E37F);
            assert(pack.press_diff_GET() == 1.8429273E38F);
            assert(pack.time_boot_ms_GET() == 3760242962L);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.press_diff_SET(1.8429273E38F) ;
        p137.time_boot_ms_SET(3760242962L) ;
        p137.temperature_SET((short)1548) ;
        p137.press_abs_SET(4.1146047E37F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.4581414E38F, 1.4562614E38F, 1.945856E38F, -2.992353E38F}));
            assert(pack.z_GET() == -1.0293545E38F);
            assert(pack.y_GET() == -2.8999234E38F);
            assert(pack.x_GET() == 5.470913E37F);
            assert(pack.time_usec_GET() == 4745584106334842177L);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(4745584106334842177L) ;
        p138.x_SET(5.470913E37F) ;
        p138.q_SET(new float[] {-1.4581414E38F, 1.4562614E38F, 1.945856E38F, -2.992353E38F}, 0) ;
        p138.y_SET(-2.8999234E38F) ;
        p138.z_SET(-1.0293545E38F) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)61);
            assert(pack.target_component_GET() == (char)82);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-1.742751E38F, -1.6259748E38F, -2.654707E38F, 2.3472906E38F, 2.0358148E37F, 1.2439909E38F, 1.7759632E37F, -6.6797555E37F}));
            assert(pack.time_usec_GET() == 4491973346917579528L);
            assert(pack.target_system_GET() == (char)211);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.controls_SET(new float[] {-1.742751E38F, -1.6259748E38F, -2.654707E38F, 2.3472906E38F, 2.0358148E37F, 1.2439909E38F, 1.7759632E37F, -6.6797555E37F}, 0) ;
        p139.time_usec_SET(4491973346917579528L) ;
        p139.target_component_SET((char)82) ;
        p139.group_mlx_SET((char)61) ;
        p139.target_system_SET((char)211) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5692100406593922309L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {3.1750818E38F, -2.7488715E38F, 5.0785343E37F, 2.341107E38F, 1.680721E38F, 2.4801122E38F, -2.1152202E38F, -1.6109973E38F}));
            assert(pack.group_mlx_GET() == (char)64);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.group_mlx_SET((char)64) ;
        p140.controls_SET(new float[] {3.1750818E38F, -2.7488715E38F, 5.0785343E37F, 2.341107E38F, 1.680721E38F, 2.4801122E38F, -2.1152202E38F, -1.6109973E38F}, 0) ;
        p140.time_usec_SET(5692100406593922309L) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.bottom_clearance_GET() == 1.0704876E38F);
            assert(pack.altitude_amsl_GET() == -8.544297E37F);
            assert(pack.time_usec_GET() == 8683808676262828819L);
            assert(pack.altitude_terrain_GET() == -1.5084645E38F);
            assert(pack.altitude_local_GET() == -2.8420203E38F);
            assert(pack.altitude_relative_GET() == -2.4164211E38F);
            assert(pack.altitude_monotonic_GET() == 7.437344E37F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_relative_SET(-2.4164211E38F) ;
        p141.altitude_amsl_SET(-8.544297E37F) ;
        p141.altitude_local_SET(-2.8420203E38F) ;
        p141.bottom_clearance_SET(1.0704876E38F) ;
        p141.altitude_terrain_SET(-1.5084645E38F) ;
        p141.altitude_monotonic_SET(7.437344E37F) ;
        p141.time_usec_SET(8683808676262828819L) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.uri_type_GET() == (char)39);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)10, (char)133, (char)1, (char)242, (char)149, (char)123, (char)3, (char)200, (char)34, (char)207, (char)166, (char)254, (char)4, (char)46, (char)154, (char)160, (char)249, (char)172, (char)8, (char)122, (char)155, (char)145, (char)251, (char)234, (char)2, (char)255, (char)128, (char)185, (char)138, (char)115, (char)122, (char)99, (char)217, (char)40, (char)105, (char)93, (char)20, (char)13, (char)15, (char)255, (char)26, (char)168, (char)77, (char)71, (char)70, (char)203, (char)181, (char)25, (char)244, (char)37, (char)234, (char)237, (char)241, (char)118, (char)126, (char)57, (char)123, (char)197, (char)255, (char)251, (char)204, (char)117, (char)12, (char)247, (char)63, (char)63, (char)121, (char)105, (char)111, (char)126, (char)85, (char)40, (char)79, (char)139, (char)23, (char)132, (char)249, (char)173, (char)162, (char)220, (char)229, (char)75, (char)193, (char)240, (char)53, (char)205, (char)156, (char)248, (char)28, (char)75, (char)221, (char)237, (char)65, (char)209, (char)6, (char)132, (char)52, (char)135, (char)221, (char)53, (char)218, (char)172, (char)101, (char)51, (char)202, (char)102, (char)38, (char)10, (char)86, (char)116, (char)44, (char)214, (char)221, (char)243, (char)235, (char)157, (char)133, (char)68, (char)122, (char)125}));
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)120, (char)251, (char)16, (char)91, (char)238, (char)4, (char)196, (char)76, (char)132, (char)249, (char)86, (char)64, (char)116, (char)178, (char)185, (char)120, (char)65, (char)62, (char)22, (char)115, (char)68, (char)8, (char)190, (char)254, (char)114, (char)196, (char)28, (char)47, (char)99, (char)203, (char)74, (char)120, (char)65, (char)49, (char)41, (char)159, (char)5, (char)34, (char)146, (char)97, (char)120, (char)199, (char)162, (char)251, (char)117, (char)80, (char)181, (char)28, (char)159, (char)221, (char)218, (char)30, (char)187, (char)12, (char)179, (char)122, (char)9, (char)235, (char)67, (char)128, (char)7, (char)64, (char)239, (char)174, (char)117, (char)75, (char)150, (char)100, (char)241, (char)250, (char)175, (char)59, (char)119, (char)245, (char)40, (char)183, (char)129, (char)80, (char)94, (char)83, (char)208, (char)65, (char)236, (char)188, (char)88, (char)202, (char)153, (char)168, (char)190, (char)183, (char)18, (char)172, (char)122, (char)161, (char)42, (char)3, (char)13, (char)194, (char)191, (char)243, (char)166, (char)100, (char)11, (char)15, (char)210, (char)2, (char)59, (char)225, (char)140, (char)130, (char)231, (char)99, (char)246, (char)76, (char)3, (char)235, (char)77, (char)196, (char)239, (char)174}));
            assert(pack.transfer_type_GET() == (char)10);
            assert(pack.request_id_GET() == (char)210);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.uri_SET(new char[] {(char)120, (char)251, (char)16, (char)91, (char)238, (char)4, (char)196, (char)76, (char)132, (char)249, (char)86, (char)64, (char)116, (char)178, (char)185, (char)120, (char)65, (char)62, (char)22, (char)115, (char)68, (char)8, (char)190, (char)254, (char)114, (char)196, (char)28, (char)47, (char)99, (char)203, (char)74, (char)120, (char)65, (char)49, (char)41, (char)159, (char)5, (char)34, (char)146, (char)97, (char)120, (char)199, (char)162, (char)251, (char)117, (char)80, (char)181, (char)28, (char)159, (char)221, (char)218, (char)30, (char)187, (char)12, (char)179, (char)122, (char)9, (char)235, (char)67, (char)128, (char)7, (char)64, (char)239, (char)174, (char)117, (char)75, (char)150, (char)100, (char)241, (char)250, (char)175, (char)59, (char)119, (char)245, (char)40, (char)183, (char)129, (char)80, (char)94, (char)83, (char)208, (char)65, (char)236, (char)188, (char)88, (char)202, (char)153, (char)168, (char)190, (char)183, (char)18, (char)172, (char)122, (char)161, (char)42, (char)3, (char)13, (char)194, (char)191, (char)243, (char)166, (char)100, (char)11, (char)15, (char)210, (char)2, (char)59, (char)225, (char)140, (char)130, (char)231, (char)99, (char)246, (char)76, (char)3, (char)235, (char)77, (char)196, (char)239, (char)174}, 0) ;
        p142.storage_SET(new char[] {(char)10, (char)133, (char)1, (char)242, (char)149, (char)123, (char)3, (char)200, (char)34, (char)207, (char)166, (char)254, (char)4, (char)46, (char)154, (char)160, (char)249, (char)172, (char)8, (char)122, (char)155, (char)145, (char)251, (char)234, (char)2, (char)255, (char)128, (char)185, (char)138, (char)115, (char)122, (char)99, (char)217, (char)40, (char)105, (char)93, (char)20, (char)13, (char)15, (char)255, (char)26, (char)168, (char)77, (char)71, (char)70, (char)203, (char)181, (char)25, (char)244, (char)37, (char)234, (char)237, (char)241, (char)118, (char)126, (char)57, (char)123, (char)197, (char)255, (char)251, (char)204, (char)117, (char)12, (char)247, (char)63, (char)63, (char)121, (char)105, (char)111, (char)126, (char)85, (char)40, (char)79, (char)139, (char)23, (char)132, (char)249, (char)173, (char)162, (char)220, (char)229, (char)75, (char)193, (char)240, (char)53, (char)205, (char)156, (char)248, (char)28, (char)75, (char)221, (char)237, (char)65, (char)209, (char)6, (char)132, (char)52, (char)135, (char)221, (char)53, (char)218, (char)172, (char)101, (char)51, (char)202, (char)102, (char)38, (char)10, (char)86, (char)116, (char)44, (char)214, (char)221, (char)243, (char)235, (char)157, (char)133, (char)68, (char)122, (char)125}, 0) ;
        p142.uri_type_SET((char)39) ;
        p142.request_id_SET((char)210) ;
        p142.transfer_type_SET((char)10) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)1221);
            assert(pack.time_boot_ms_GET() == 1207361091L);
            assert(pack.press_diff_GET() == -2.04176E38F);
            assert(pack.press_abs_GET() == 3.2078172E38F);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.temperature_SET((short)1221) ;
        p143.press_diff_SET(-2.04176E38F) ;
        p143.time_boot_ms_SET(1207361091L) ;
        p143.press_abs_SET(3.2078172E38F) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.vel_GET(),  new float[] {-3.216533E38F, 2.0428102E38F, -2.8550996E38F}));
            assert(pack.alt_GET() == -2.975677E38F);
            assert(pack.timestamp_GET() == 5848954207545844589L);
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-5.245943E37F, 1.6728228E36F, -2.238141E38F}));
            assert(pack.lat_GET() == 226348018);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {1.151593E38F, 1.9103797E38F, 1.3725576E37F}));
            assert(pack.est_capabilities_GET() == (char)245);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {3.2835398E38F, 3.2934966E38F, 6.9110034E37F}));
            assert(pack.lon_GET() == -777923622);
            assert(pack.custom_state_GET() == 1198690666030838470L);
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {2.7642464E38F, -2.838299E38F, 1.6114388E38F, 3.1138592E38F}));
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.acc_SET(new float[] {3.2835398E38F, 3.2934966E38F, 6.9110034E37F}, 0) ;
        p144.lat_SET(226348018) ;
        p144.vel_SET(new float[] {-3.216533E38F, 2.0428102E38F, -2.8550996E38F}, 0) ;
        p144.attitude_q_SET(new float[] {2.7642464E38F, -2.838299E38F, 1.6114388E38F, 3.1138592E38F}, 0) ;
        p144.rates_SET(new float[] {-5.245943E37F, 1.6728228E36F, -2.238141E38F}, 0) ;
        p144.est_capabilities_SET((char)245) ;
        p144.lon_SET(-777923622) ;
        p144.position_cov_SET(new float[] {1.151593E38F, 1.9103797E38F, 1.3725576E37F}, 0) ;
        p144.alt_SET(-2.975677E38F) ;
        p144.timestamp_SET(5848954207545844589L) ;
        p144.custom_state_SET(1198690666030838470L) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.airspeed_GET() == 3.1160493E38F);
            assert(pack.x_pos_GET() == 1.7153414E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {9.80839E36F, -2.6414228E38F, -2.2163748E37F}));
            assert(pack.yaw_rate_GET() == -2.5745803E38F);
            assert(pack.y_pos_GET() == 2.5724223E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.5296169E38F, -2.4760301E38F, 3.3533823E38F, -2.9220754E38F}));
            assert(pack.x_vel_GET() == 1.7634095E37F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {1.6263336E38F, 3.3167522E38F, 1.0775576E38F}));
            assert(pack.y_vel_GET() == -9.781606E37F);
            assert(pack.x_acc_GET() == 5.944929E37F);
            assert(pack.time_usec_GET() == 5573537983514249219L);
            assert(pack.roll_rate_GET() == -1.3892044E38F);
            assert(pack.pitch_rate_GET() == 2.693956E38F);
            assert(pack.z_pos_GET() == 2.8464859E38F);
            assert(pack.z_vel_GET() == -4.1639435E37F);
            assert(pack.y_acc_GET() == -1.0446159E38F);
            assert(pack.z_acc_GET() == -6.851946E37F);
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.z_acc_SET(-6.851946E37F) ;
        p146.x_acc_SET(5.944929E37F) ;
        p146.time_usec_SET(5573537983514249219L) ;
        p146.y_acc_SET(-1.0446159E38F) ;
        p146.x_pos_SET(1.7153414E38F) ;
        p146.vel_variance_SET(new float[] {1.6263336E38F, 3.3167522E38F, 1.0775576E38F}, 0) ;
        p146.y_vel_SET(-9.781606E37F) ;
        p146.z_vel_SET(-4.1639435E37F) ;
        p146.q_SET(new float[] {2.5296169E38F, -2.4760301E38F, 3.3533823E38F, -2.9220754E38F}, 0) ;
        p146.airspeed_SET(3.1160493E38F) ;
        p146.x_vel_SET(1.7634095E37F) ;
        p146.roll_rate_SET(-1.3892044E38F) ;
        p146.yaw_rate_SET(-2.5745803E38F) ;
        p146.z_pos_SET(2.8464859E38F) ;
        p146.pitch_rate_SET(2.693956E38F) ;
        p146.pos_variance_SET(new float[] {9.80839E36F, -2.6414228E38F, -2.2163748E37F}, 0) ;
        p146.y_pos_SET(2.5724223E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)5832, (char)55851, (char)23011, (char)44241, (char)30256, (char)63329, (char)59833, (char)37668, (char)25670, (char)1040}));
            assert(pack.current_consumed_GET() == -852790650);
            assert(pack.energy_consumed_GET() == 1253766364);
            assert(pack.current_battery_GET() == (short) -15745);
            assert(pack.id_GET() == (char)62);
            assert(pack.battery_remaining_GET() == (byte) - 2);
            assert(pack.temperature_GET() == (short) -27011);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.energy_consumed_SET(1253766364) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION) ;
        p147.id_SET((char)62) ;
        p147.current_battery_SET((short) -15745) ;
        p147.voltages_SET(new char[] {(char)5832, (char)55851, (char)23011, (char)44241, (char)30256, (char)63329, (char)59833, (char)37668, (char)25670, (char)1040}, 0) ;
        p147.current_consumed_SET(-852790650) ;
        p147.temperature_SET((short) -27011) ;
        p147.battery_remaining_SET((byte) - 2) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)48, (char)3, (char)219, (char)94, (char)95, (char)238, (char)56, (char)77}));
            assert(pack.middleware_sw_version_GET() == 3031703407L);
            assert(pack.product_id_GET() == (char)64596);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)107, (char)100, (char)181, (char)130, (char)123, (char)205, (char)224, (char)79}));
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)48, (char)170, (char)167, (char)51, (char)196, (char)47, (char)25, (char)107, (char)138, (char)30, (char)204, (char)219, (char)212, (char)107, (char)117, (char)62, (char)237, (char)215}));
            assert(pack.flight_sw_version_GET() == 3342579954L);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)222, (char)121, (char)84, (char)163, (char)205, (char)23, (char)161, (char)144}));
            assert(pack.os_sw_version_GET() == 35692749L);
            assert(pack.vendor_id_GET() == (char)26245);
            assert(pack.board_version_GET() == 3778299431L);
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN));
            assert(pack.uid_GET() == 7875218100336589740L);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.middleware_custom_version_SET(new char[] {(char)107, (char)100, (char)181, (char)130, (char)123, (char)205, (char)224, (char)79}, 0) ;
        p148.os_sw_version_SET(35692749L) ;
        p148.uid_SET(7875218100336589740L) ;
        p148.flight_sw_version_SET(3342579954L) ;
        p148.flight_custom_version_SET(new char[] {(char)222, (char)121, (char)84, (char)163, (char)205, (char)23, (char)161, (char)144}, 0) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN)) ;
        p148.product_id_SET((char)64596) ;
        p148.vendor_id_SET((char)26245) ;
        p148.os_custom_version_SET(new char[] {(char)48, (char)3, (char)219, (char)94, (char)95, (char)238, (char)56, (char)77}, 0) ;
        p148.board_version_SET(3778299431L) ;
        p148.middleware_sw_version_SET(3031703407L) ;
        p148.uid2_SET(new char[] {(char)48, (char)170, (char)167, (char)51, (char)196, (char)47, (char)25, (char)107, (char)138, (char)30, (char)204, (char)219, (char)212, (char)107, (char)117, (char)62, (char)237, (char)215}, 0, PH) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.z_TRY(ph) == -2.8232897E38F);
            assert(pack.angle_y_GET() == 7.601049E37F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
            assert(pack.y_TRY(ph) == 2.7905273E38F);
            assert(pack.angle_x_GET() == 1.429734E38F);
            assert(pack.size_y_GET() == -1.8149337E38F);
            assert(pack.size_x_GET() == 2.2030956E38F);
            assert(pack.x_TRY(ph) == 2.627584E38F);
            assert(pack.target_num_GET() == (char)182);
            assert(pack.distance_GET() == 2.4753393E38F);
            assert(pack.time_usec_GET() == 3881307793904796112L);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {1.9756608E38F, 1.825604E38F, 3.259021E38F, 3.2521355E38F}));
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.position_valid_TRY(ph) == (char)36);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.target_num_SET((char)182) ;
        p149.x_SET(2.627584E38F, PH) ;
        p149.angle_y_SET(7.601049E37F) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p149.distance_SET(2.4753393E38F) ;
        p149.size_y_SET(-1.8149337E38F) ;
        p149.q_SET(new float[] {1.9756608E38F, 1.825604E38F, 3.259021E38F, 3.2521355E38F}, 0, PH) ;
        p149.z_SET(-2.8232897E38F, PH) ;
        p149.position_valid_SET((char)36, PH) ;
        p149.angle_x_SET(1.429734E38F) ;
        p149.time_usec_SET(3881307793904796112L) ;
        p149.size_x_SET(2.2030956E38F) ;
        p149.y_SET(2.7905273E38F, PH) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_SENSOR_OFFSETS.add((src, ph, pack) ->
        {
            assert(pack.accel_cal_y_GET() == -2.2436842E38F);
            assert(pack.mag_declination_GET() == -1.3693592E37F);
            assert(pack.raw_press_GET() == 1515132249);
            assert(pack.raw_temp_GET() == 1439823425);
            assert(pack.gyro_cal_x_GET() == 1.8335838E38F);
            assert(pack.mag_ofs_y_GET() == (short) -22885);
            assert(pack.accel_cal_x_GET() == 2.648169E38F);
            assert(pack.mag_ofs_z_GET() == (short)29674);
            assert(pack.gyro_cal_z_GET() == 1.1642036E38F);
            assert(pack.gyro_cal_y_GET() == 2.0124166E38F);
            assert(pack.accel_cal_z_GET() == 2.6061034E38F);
            assert(pack.mag_ofs_x_GET() == (short)11908);
        });
        GroundControl.SENSOR_OFFSETS p150 = CommunicationChannel.new_SENSOR_OFFSETS();
        PH.setPack(p150);
        p150.raw_press_SET(1515132249) ;
        p150.accel_cal_y_SET(-2.2436842E38F) ;
        p150.mag_ofs_y_SET((short) -22885) ;
        p150.mag_ofs_z_SET((short)29674) ;
        p150.mag_ofs_x_SET((short)11908) ;
        p150.mag_declination_SET(-1.3693592E37F) ;
        p150.gyro_cal_z_SET(1.1642036E38F) ;
        p150.gyro_cal_y_SET(2.0124166E38F) ;
        p150.gyro_cal_x_SET(1.8335838E38F) ;
        p150.accel_cal_z_SET(2.6061034E38F) ;
        p150.raw_temp_SET(1439823425) ;
        p150.accel_cal_x_SET(2.648169E38F) ;
        CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_MAG_OFFSETS.add((src, ph, pack) ->
        {
            assert(pack.mag_ofs_x_GET() == (short)32172);
            assert(pack.mag_ofs_y_GET() == (short)31917);
            assert(pack.target_component_GET() == (char)22);
            assert(pack.mag_ofs_z_GET() == (short)12641);
            assert(pack.target_system_GET() == (char)172);
        });
        GroundControl.SET_MAG_OFFSETS p151 = CommunicationChannel.new_SET_MAG_OFFSETS();
        PH.setPack(p151);
        p151.target_system_SET((char)172) ;
        p151.mag_ofs_y_SET((short)31917) ;
        p151.target_component_SET((char)22) ;
        p151.mag_ofs_z_SET((short)12641) ;
        p151.mag_ofs_x_SET((short)32172) ;
        CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MEMINFO.add((src, ph, pack) ->
        {
            assert(pack.brkval_GET() == (char)19604);
            assert(pack.freemem32_TRY(ph) == 462674975L);
            assert(pack.freemem_GET() == (char)45716);
        });
        GroundControl.MEMINFO p152 = CommunicationChannel.new_MEMINFO();
        PH.setPack(p152);
        p152.freemem_SET((char)45716) ;
        p152.brkval_SET((char)19604) ;
        p152.freemem32_SET(462674975L, PH) ;
        CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AP_ADC.add((src, ph, pack) ->
        {
            assert(pack.adc3_GET() == (char)56081);
            assert(pack.adc2_GET() == (char)41239);
            assert(pack.adc4_GET() == (char)56573);
            assert(pack.adc6_GET() == (char)31260);
            assert(pack.adc1_GET() == (char)1564);
            assert(pack.adc5_GET() == (char)56513);
        });
        GroundControl.AP_ADC p153 = CommunicationChannel.new_AP_ADC();
        PH.setPack(p153);
        p153.adc2_SET((char)41239) ;
        p153.adc5_SET((char)56513) ;
        p153.adc3_SET((char)56081) ;
        p153.adc1_SET((char)1564) ;
        p153.adc6_SET((char)31260) ;
        p153.adc4_SET((char)56573) ;
        CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DIGICAM_CONFIGURE.add((src, ph, pack) ->
        {
            assert(pack.shutter_speed_GET() == (char)18352);
            assert(pack.aperture_GET() == (char)246);
            assert(pack.exposure_type_GET() == (char)45);
            assert(pack.command_id_GET() == (char)144);
            assert(pack.target_system_GET() == (char)94);
            assert(pack.iso_GET() == (char)140);
            assert(pack.extra_value_GET() == 2.194571E38F);
            assert(pack.mode_GET() == (char)196);
            assert(pack.engine_cut_off_GET() == (char)89);
            assert(pack.target_component_GET() == (char)195);
            assert(pack.extra_param_GET() == (char)237);
        });
        GroundControl.DIGICAM_CONFIGURE p154 = CommunicationChannel.new_DIGICAM_CONFIGURE();
        PH.setPack(p154);
        p154.aperture_SET((char)246) ;
        p154.extra_value_SET(2.194571E38F) ;
        p154.extra_param_SET((char)237) ;
        p154.iso_SET((char)140) ;
        p154.target_system_SET((char)94) ;
        p154.engine_cut_off_SET((char)89) ;
        p154.shutter_speed_SET((char)18352) ;
        p154.mode_SET((char)196) ;
        p154.target_component_SET((char)195) ;
        p154.exposure_type_SET((char)45) ;
        p154.command_id_SET((char)144) ;
        CommunicationChannel.instance.send(p154);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DIGICAM_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.extra_param_GET() == (char)138);
            assert(pack.zoom_pos_GET() == (char)158);
            assert(pack.command_id_GET() == (char)117);
            assert(pack.session_GET() == (char)190);
            assert(pack.target_component_GET() == (char)46);
            assert(pack.extra_value_GET() == 3.2836846E38F);
            assert(pack.zoom_step_GET() == (byte)50);
            assert(pack.shot_GET() == (char)79);
            assert(pack.focus_lock_GET() == (char)97);
            assert(pack.target_system_GET() == (char)214);
        });
        GroundControl.DIGICAM_CONTROL p155 = CommunicationChannel.new_DIGICAM_CONTROL();
        PH.setPack(p155);
        p155.target_system_SET((char)214) ;
        p155.extra_param_SET((char)138) ;
        p155.extra_value_SET(3.2836846E38F) ;
        p155.shot_SET((char)79) ;
        p155.focus_lock_SET((char)97) ;
        p155.zoom_step_SET((byte)50) ;
        p155.target_component_SET((char)46) ;
        p155.session_SET((char)190) ;
        p155.command_id_SET((char)117) ;
        p155.zoom_pos_SET((char)158) ;
        CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_CONFIGURE.add((src, ph, pack) ->
        {
            assert(pack.stab_yaw_GET() == (char)105);
            assert(pack.mount_mode_GET() == MAV_MOUNT_MODE.MAV_MOUNT_MODE_GPS_POINT);
            assert(pack.stab_roll_GET() == (char)106);
            assert(pack.target_system_GET() == (char)103);
            assert(pack.stab_pitch_GET() == (char)208);
            assert(pack.target_component_GET() == (char)149);
        });
        GroundControl.MOUNT_CONFIGURE p156 = CommunicationChannel.new_MOUNT_CONFIGURE();
        PH.setPack(p156);
        p156.stab_pitch_SET((char)208) ;
        p156.stab_yaw_SET((char)105) ;
        p156.target_component_SET((char)149) ;
        p156.mount_mode_SET(MAV_MOUNT_MODE.MAV_MOUNT_MODE_GPS_POINT) ;
        p156.stab_roll_SET((char)106) ;
        p156.target_system_SET((char)103) ;
        CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.input_c_GET() == 1334526100);
            assert(pack.target_component_GET() == (char)70);
            assert(pack.input_b_GET() == -1243593639);
            assert(pack.input_a_GET() == -607591792);
            assert(pack.target_system_GET() == (char)185);
            assert(pack.save_position_GET() == (char)184);
        });
        GroundControl.MOUNT_CONTROL p157 = CommunicationChannel.new_MOUNT_CONTROL();
        PH.setPack(p157);
        p157.input_a_SET(-607591792) ;
        p157.target_component_SET((char)70) ;
        p157.input_c_SET(1334526100) ;
        p157.target_system_SET((char)185) ;
        p157.save_position_SET((char)184) ;
        p157.input_b_SET(-1243593639) ;
        CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pointing_b_GET() == -498380982);
            assert(pack.pointing_c_GET() == -601258353);
            assert(pack.target_system_GET() == (char)107);
            assert(pack.target_component_GET() == (char)152);
            assert(pack.pointing_a_GET() == 1743421402);
        });
        GroundControl.MOUNT_STATUS p158 = CommunicationChannel.new_MOUNT_STATUS();
        PH.setPack(p158);
        p158.pointing_a_SET(1743421402) ;
        p158.pointing_b_SET(-498380982) ;
        p158.pointing_c_SET(-601258353) ;
        p158.target_component_SET((char)152) ;
        p158.target_system_SET((char)107) ;
        CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FENCE_POINT.add((src, ph, pack) ->
        {
            assert(pack.idx_GET() == (char)98);
            assert(pack.count_GET() == (char)203);
            assert(pack.lng_GET() == -2.71394E37F);
            assert(pack.target_component_GET() == (char)139);
            assert(pack.target_system_GET() == (char)210);
            assert(pack.lat_GET() == 1.5204539E38F);
        });
        GroundControl.FENCE_POINT p160 = CommunicationChannel.new_FENCE_POINT();
        PH.setPack(p160);
        p160.target_component_SET((char)139) ;
        p160.lat_SET(1.5204539E38F) ;
        p160.count_SET((char)203) ;
        p160.idx_SET((char)98) ;
        p160.target_system_SET((char)210) ;
        p160.lng_SET(-2.71394E37F) ;
        CommunicationChannel.instance.send(p160);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FENCE_FETCH_POINT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)124);
            assert(pack.idx_GET() == (char)166);
            assert(pack.target_system_GET() == (char)240);
        });
        GroundControl.FENCE_FETCH_POINT p161 = CommunicationChannel.new_FENCE_FETCH_POINT();
        PH.setPack(p161);
        p161.idx_SET((char)166) ;
        p161.target_system_SET((char)240) ;
        p161.target_component_SET((char)124) ;
        CommunicationChannel.instance.send(p161);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FENCE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.breach_count_GET() == (char)47432);
            assert(pack.breach_time_GET() == 3684969980L);
            assert(pack.breach_type_GET() == FENCE_BREACH.FENCE_BREACH_MAXALT);
            assert(pack.breach_status_GET() == (char)242);
        });
        GroundControl.FENCE_STATUS p162 = CommunicationChannel.new_FENCE_STATUS();
        PH.setPack(p162);
        p162.breach_type_SET(FENCE_BREACH.FENCE_BREACH_MAXALT) ;
        p162.breach_status_SET((char)242) ;
        p162.breach_time_SET(3684969980L) ;
        p162.breach_count_SET((char)47432) ;
        CommunicationChannel.instance.send(p162);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AHRS.add((src, ph, pack) ->
        {
            assert(pack.accel_weight_GET() == -2.4879673E38F);
            assert(pack.omegaIy_GET() == -1.3450593E38F);
            assert(pack.error_yaw_GET() == -1.0349545E38F);
            assert(pack.omegaIz_GET() == -2.7010026E38F);
            assert(pack.renorm_val_GET() == -1.647543E38F);
            assert(pack.omegaIx_GET() == 7.9913865E37F);
            assert(pack.error_rp_GET() == -2.6188505E38F);
        });
        GroundControl.AHRS p163 = CommunicationChannel.new_AHRS();
        PH.setPack(p163);
        p163.error_rp_SET(-2.6188505E38F) ;
        p163.renorm_val_SET(-1.647543E38F) ;
        p163.omegaIx_SET(7.9913865E37F) ;
        p163.omegaIy_SET(-1.3450593E38F) ;
        p163.accel_weight_SET(-2.4879673E38F) ;
        p163.error_yaw_SET(-1.0349545E38F) ;
        p163.omegaIz_SET(-2.7010026E38F) ;
        CommunicationChannel.instance.send(p163);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SIMSTATE.add((src, ph, pack) ->
        {
            assert(pack.lng_GET() == 1578032927);
            assert(pack.ygyro_GET() == 2.8115083E37F);
            assert(pack.xgyro_GET() == 3.0876437E38F);
            assert(pack.yacc_GET() == 1.3423716E38F);
            assert(pack.xacc_GET() == -1.5737912E38F);
            assert(pack.roll_GET() == 1.1535499E38F);
            assert(pack.zacc_GET() == 3.2429597E38F);
            assert(pack.zgyro_GET() == -1.851154E38F);
            assert(pack.pitch_GET() == 3.315517E38F);
            assert(pack.lat_GET() == -1711117823);
            assert(pack.yaw_GET() == 1.0787796E38F);
        });
        GroundControl.SIMSTATE p164 = CommunicationChannel.new_SIMSTATE();
        PH.setPack(p164);
        p164.lat_SET(-1711117823) ;
        p164.pitch_SET(3.315517E38F) ;
        p164.zacc_SET(3.2429597E38F) ;
        p164.lng_SET(1578032927) ;
        p164.xacc_SET(-1.5737912E38F) ;
        p164.yaw_SET(1.0787796E38F) ;
        p164.yacc_SET(1.3423716E38F) ;
        p164.xgyro_SET(3.0876437E38F) ;
        p164.ygyro_SET(2.8115083E37F) ;
        p164.zgyro_SET(-1.851154E38F) ;
        p164.roll_SET(1.1535499E38F) ;
        CommunicationChannel.instance.send(p164);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HWSTATUS.add((src, ph, pack) ->
        {
            assert(pack.Vcc_GET() == (char)12073);
            assert(pack.I2Cerr_GET() == (char)240);
        });
        GroundControl.HWSTATUS p165 = CommunicationChannel.new_HWSTATUS();
        PH.setPack(p165);
        p165.Vcc_SET((char)12073) ;
        p165.I2Cerr_SET((char)240) ;
        CommunicationChannel.instance.send(p165);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RADIO.add((src, ph, pack) ->
        {
            assert(pack.remnoise_GET() == (char)105);
            assert(pack.txbuf_GET() == (char)130);
            assert(pack.rxerrors_GET() == (char)13545);
            assert(pack.rssi_GET() == (char)32);
            assert(pack.remrssi_GET() == (char)36);
            assert(pack.noise_GET() == (char)249);
            assert(pack.fixed__GET() == (char)28036);
        });
        GroundControl.RADIO p166 = CommunicationChannel.new_RADIO();
        PH.setPack(p166);
        p166.txbuf_SET((char)130) ;
        p166.remnoise_SET((char)105) ;
        p166.rssi_SET((char)32) ;
        p166.remrssi_SET((char)36) ;
        p166.rxerrors_SET((char)13545) ;
        p166.fixed__SET((char)28036) ;
        p166.noise_SET((char)249) ;
        CommunicationChannel.instance.send(p166);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LIMITS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.last_clear_GET() == 1269719651L);
            assert(pack.last_recovery_GET() == 3054015524L);
            assert(pack.breach_count_GET() == (char)44035);
            assert(pack.last_trigger_GET() == 2310393743L);
            assert(pack.mods_enabled_GET() == (LIMIT_MODULE.LIMIT_GEOFENCE |
                                               LIMIT_MODULE.LIMIT_ALTITUDE));
            assert(pack.last_action_GET() == 2361424643L);
            assert(pack.limits_state_GET() == LIMITS_STATE.LIMITS_RECOVERING);
            assert(pack.mods_triggered_GET() == (LIMIT_MODULE.LIMIT_GPSLOCK));
            assert(pack.mods_required_GET() == (LIMIT_MODULE.LIMIT_ALTITUDE |
                                                LIMIT_MODULE.LIMIT_GPSLOCK));
        });
        GroundControl.LIMITS_STATUS p167 = CommunicationChannel.new_LIMITS_STATUS();
        PH.setPack(p167);
        p167.limits_state_SET(LIMITS_STATE.LIMITS_RECOVERING) ;
        p167.last_clear_SET(1269719651L) ;
        p167.mods_required_SET((LIMIT_MODULE.LIMIT_ALTITUDE |
                                LIMIT_MODULE.LIMIT_GPSLOCK)) ;
        p167.mods_triggered_SET((LIMIT_MODULE.LIMIT_GPSLOCK)) ;
        p167.last_action_SET(2361424643L) ;
        p167.mods_enabled_SET((LIMIT_MODULE.LIMIT_GEOFENCE |
                               LIMIT_MODULE.LIMIT_ALTITUDE)) ;
        p167.last_trigger_SET(2310393743L) ;
        p167.breach_count_SET((char)44035) ;
        p167.last_recovery_SET(3054015524L) ;
        CommunicationChannel.instance.send(p167);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIND.add((src, ph, pack) ->
        {
            assert(pack.speed_z_GET() == -2.9601361E38F);
            assert(pack.direction_GET() == 2.3840454E37F);
            assert(pack.speed_GET() == 4.9042755E37F);
        });
        GroundControl.WIND p168 = CommunicationChannel.new_WIND();
        PH.setPack(p168);
        p168.direction_SET(2.3840454E37F) ;
        p168.speed_z_SET(-2.9601361E38F) ;
        p168.speed_SET(4.9042755E37F) ;
        CommunicationChannel.instance.send(p168);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DATA16.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)210);
            assert(pack.type_GET() == (char)9);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)233, (char)168, (char)227, (char)58, (char)231, (char)82, (char)65, (char)50, (char)207, (char)188, (char)77, (char)209, (char)213, (char)151, (char)8, (char)31}));
        });
        GroundControl.DATA16 p169 = CommunicationChannel.new_DATA16();
        PH.setPack(p169);
        p169.type_SET((char)9) ;
        p169.len_SET((char)210) ;
        p169.data__SET(new char[] {(char)233, (char)168, (char)227, (char)58, (char)231, (char)82, (char)65, (char)50, (char)207, (char)188, (char)77, (char)209, (char)213, (char)151, (char)8, (char)31}, 0) ;
        CommunicationChannel.instance.send(p169);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DATA32.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)120);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)197, (char)25, (char)99, (char)82, (char)217, (char)232, (char)14, (char)13, (char)161, (char)61, (char)153, (char)203, (char)13, (char)121, (char)252, (char)44, (char)78, (char)59, (char)73, (char)117, (char)29, (char)3, (char)89, (char)21, (char)115, (char)138, (char)15, (char)167, (char)158, (char)241, (char)41, (char)102}));
            assert(pack.type_GET() == (char)223);
        });
        GroundControl.DATA32 p170 = CommunicationChannel.new_DATA32();
        PH.setPack(p170);
        p170.type_SET((char)223) ;
        p170.data__SET(new char[] {(char)197, (char)25, (char)99, (char)82, (char)217, (char)232, (char)14, (char)13, (char)161, (char)61, (char)153, (char)203, (char)13, (char)121, (char)252, (char)44, (char)78, (char)59, (char)73, (char)117, (char)29, (char)3, (char)89, (char)21, (char)115, (char)138, (char)15, (char)167, (char)158, (char)241, (char)41, (char)102}, 0) ;
        p170.len_SET((char)120) ;
        CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DATA64.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)92);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)235, (char)157, (char)218, (char)173, (char)188, (char)64, (char)208, (char)235, (char)249, (char)166, (char)219, (char)72, (char)134, (char)215, (char)235, (char)5, (char)122, (char)35, (char)200, (char)118, (char)213, (char)170, (char)142, (char)136, (char)49, (char)37, (char)36, (char)189, (char)242, (char)220, (char)146, (char)127, (char)29, (char)25, (char)165, (char)5, (char)168, (char)124, (char)226, (char)167, (char)74, (char)13, (char)186, (char)95, (char)133, (char)115, (char)79, (char)118, (char)252, (char)150, (char)129, (char)218, (char)191, (char)80, (char)56, (char)48, (char)202, (char)119, (char)58, (char)209, (char)193, (char)178, (char)174, (char)178}));
            assert(pack.len_GET() == (char)91);
        });
        GroundControl.DATA64 p171 = CommunicationChannel.new_DATA64();
        PH.setPack(p171);
        p171.data__SET(new char[] {(char)235, (char)157, (char)218, (char)173, (char)188, (char)64, (char)208, (char)235, (char)249, (char)166, (char)219, (char)72, (char)134, (char)215, (char)235, (char)5, (char)122, (char)35, (char)200, (char)118, (char)213, (char)170, (char)142, (char)136, (char)49, (char)37, (char)36, (char)189, (char)242, (char)220, (char)146, (char)127, (char)29, (char)25, (char)165, (char)5, (char)168, (char)124, (char)226, (char)167, (char)74, (char)13, (char)186, (char)95, (char)133, (char)115, (char)79, (char)118, (char)252, (char)150, (char)129, (char)218, (char)191, (char)80, (char)56, (char)48, (char)202, (char)119, (char)58, (char)209, (char)193, (char)178, (char)174, (char)178}, 0) ;
        p171.type_SET((char)92) ;
        p171.len_SET((char)91) ;
        CommunicationChannel.instance.send(p171);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DATA96.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)61, (char)65, (char)192, (char)99, (char)45, (char)131, (char)27, (char)15, (char)66, (char)212, (char)154, (char)183, (char)43, (char)206, (char)138, (char)37, (char)216, (char)92, (char)204, (char)185, (char)76, (char)60, (char)71, (char)39, (char)72, (char)248, (char)176, (char)246, (char)84, (char)140, (char)206, (char)203, (char)244, (char)160, (char)185, (char)195, (char)239, (char)156, (char)53, (char)176, (char)42, (char)254, (char)34, (char)50, (char)144, (char)176, (char)106, (char)74, (char)4, (char)155, (char)129, (char)171, (char)155, (char)237, (char)208, (char)125, (char)175, (char)121, (char)231, (char)95, (char)70, (char)140, (char)167, (char)41, (char)51, (char)95, (char)1, (char)86, (char)88, (char)129, (char)192, (char)241, (char)111, (char)218, (char)124, (char)246, (char)64, (char)234, (char)39, (char)215, (char)64, (char)118, (char)36, (char)166, (char)38, (char)93, (char)145, (char)212, (char)45, (char)146, (char)100, (char)75, (char)210, (char)2, (char)227, (char)165}));
            assert(pack.type_GET() == (char)197);
            assert(pack.len_GET() == (char)121);
        });
        GroundControl.DATA96 p172 = CommunicationChannel.new_DATA96();
        PH.setPack(p172);
        p172.data__SET(new char[] {(char)61, (char)65, (char)192, (char)99, (char)45, (char)131, (char)27, (char)15, (char)66, (char)212, (char)154, (char)183, (char)43, (char)206, (char)138, (char)37, (char)216, (char)92, (char)204, (char)185, (char)76, (char)60, (char)71, (char)39, (char)72, (char)248, (char)176, (char)246, (char)84, (char)140, (char)206, (char)203, (char)244, (char)160, (char)185, (char)195, (char)239, (char)156, (char)53, (char)176, (char)42, (char)254, (char)34, (char)50, (char)144, (char)176, (char)106, (char)74, (char)4, (char)155, (char)129, (char)171, (char)155, (char)237, (char)208, (char)125, (char)175, (char)121, (char)231, (char)95, (char)70, (char)140, (char)167, (char)41, (char)51, (char)95, (char)1, (char)86, (char)88, (char)129, (char)192, (char)241, (char)111, (char)218, (char)124, (char)246, (char)64, (char)234, (char)39, (char)215, (char)64, (char)118, (char)36, (char)166, (char)38, (char)93, (char)145, (char)212, (char)45, (char)146, (char)100, (char)75, (char)210, (char)2, (char)227, (char)165}, 0) ;
        p172.len_SET((char)121) ;
        p172.type_SET((char)197) ;
        CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RANGEFINDER.add((src, ph, pack) ->
        {
            assert(pack.voltage_GET() == -1.1968031E38F);
            assert(pack.distance_GET() == -1.7310185E38F);
        });
        GroundControl.RANGEFINDER p173 = CommunicationChannel.new_RANGEFINDER();
        PH.setPack(p173);
        p173.voltage_SET(-1.1968031E38F) ;
        p173.distance_SET(-1.7310185E38F) ;
        CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AIRSPEED_AUTOCAL.add((src, ph, pack) ->
        {
            assert(pack.Pby_GET() == -3.2406695E36F);
            assert(pack.Pax_GET() == 1.0413672E38F);
            assert(pack.ratio_GET() == 1.6484585E37F);
            assert(pack.Pcz_GET() == 9.52986E37F);
            assert(pack.vz_GET() == 3.0214434E37F);
            assert(pack.state_y_GET() == -3.2989702E38F);
            assert(pack.vy_GET() == 1.1141737E38F);
            assert(pack.state_z_GET() == -8.917306E37F);
            assert(pack.vx_GET() == -2.163786E37F);
            assert(pack.diff_pressure_GET() == 4.6774284E37F);
            assert(pack.EAS2TAS_GET() == 5.056917E37F);
            assert(pack.state_x_GET() == -8.763791E35F);
        });
        GroundControl.AIRSPEED_AUTOCAL p174 = CommunicationChannel.new_AIRSPEED_AUTOCAL();
        PH.setPack(p174);
        p174.diff_pressure_SET(4.6774284E37F) ;
        p174.vy_SET(1.1141737E38F) ;
        p174.Pby_SET(-3.2406695E36F) ;
        p174.vx_SET(-2.163786E37F) ;
        p174.state_x_SET(-8.763791E35F) ;
        p174.state_z_SET(-8.917306E37F) ;
        p174.Pcz_SET(9.52986E37F) ;
        p174.ratio_SET(1.6484585E37F) ;
        p174.vz_SET(3.0214434E37F) ;
        p174.EAS2TAS_SET(5.056917E37F) ;
        p174.Pax_SET(1.0413672E38F) ;
        p174.state_y_SET(-3.2989702E38F) ;
        CommunicationChannel.instance.send(p174);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RALLY_POINT.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)41);
            assert(pack.idx_GET() == (char)187);
            assert(pack.lat_GET() == 1093837561);
            assert(pack.target_system_GET() == (char)243);
            assert(pack.break_alt_GET() == (short)28654);
            assert(pack.target_component_GET() == (char)117);
            assert(pack.land_dir_GET() == (char)11381);
            assert(pack.lng_GET() == -1278911083);
            assert(pack.alt_GET() == (short) -8558);
            assert(pack.flags_GET() == RALLY_FLAGS.FAVORABLE_WIND);
        });
        GroundControl.RALLY_POINT p175 = CommunicationChannel.new_RALLY_POINT();
        PH.setPack(p175);
        p175.target_system_SET((char)243) ;
        p175.lng_SET(-1278911083) ;
        p175.alt_SET((short) -8558) ;
        p175.break_alt_SET((short)28654) ;
        p175.target_component_SET((char)117) ;
        p175.lat_SET(1093837561) ;
        p175.flags_SET(RALLY_FLAGS.FAVORABLE_WIND) ;
        p175.count_SET((char)41) ;
        p175.land_dir_SET((char)11381) ;
        p175.idx_SET((char)187) ;
        CommunicationChannel.instance.send(p175);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RALLY_FETCH_POINT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)104);
            assert(pack.idx_GET() == (char)111);
            assert(pack.target_system_GET() == (char)182);
        });
        GroundControl.RALLY_FETCH_POINT p176 = CommunicationChannel.new_RALLY_FETCH_POINT();
        PH.setPack(p176);
        p176.target_component_SET((char)104) ;
        p176.target_system_SET((char)182) ;
        p176.idx_SET((char)111) ;
        CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_COMPASSMOT_STATUS.add((src, ph, pack) ->
        {
            assert(pack.current_GET() == -1.9500141E36F);
            assert(pack.throttle_GET() == (char)39239);
            assert(pack.CompensationZ_GET() == -2.7008886E38F);
            assert(pack.CompensationY_GET() == 2.7833291E38F);
            assert(pack.interference_GET() == (char)860);
            assert(pack.CompensationX_GET() == 2.3716485E38F);
        });
        GroundControl.COMPASSMOT_STATUS p177 = CommunicationChannel.new_COMPASSMOT_STATUS();
        PH.setPack(p177);
        p177.CompensationZ_SET(-2.7008886E38F) ;
        p177.current_SET(-1.9500141E36F) ;
        p177.CompensationY_SET(2.7833291E38F) ;
        p177.throttle_SET((char)39239) ;
        p177.CompensationX_SET(2.3716485E38F) ;
        p177.interference_SET((char)860) ;
        CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AHRS2.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 3.3980557E38F);
            assert(pack.altitude_GET() == 1.1967004E38F);
            assert(pack.yaw_GET() == -5.7237457E37F);
            assert(pack.lat_GET() == 946118570);
            assert(pack.roll_GET() == -1.7698098E38F);
            assert(pack.lng_GET() == -975462858);
        });
        GroundControl.AHRS2 p178 = CommunicationChannel.new_AHRS2();
        PH.setPack(p178);
        p178.yaw_SET(-5.7237457E37F) ;
        p178.roll_SET(-1.7698098E38F) ;
        p178.altitude_SET(1.1967004E38F) ;
        p178.pitch_SET(3.3980557E38F) ;
        p178.lat_SET(946118570) ;
        p178.lng_SET(-975462858) ;
        CommunicationChannel.instance.send(p178);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_STATUS.add((src, ph, pack) ->
        {
            assert(pack.p4_GET() == 2.3151375E38F);
            assert(pack.time_usec_GET() == 5203911896583327149L);
            assert(pack.target_system_GET() == (char)55);
            assert(pack.img_idx_GET() == (char)46484);
            assert(pack.cam_idx_GET() == (char)225);
            assert(pack.p1_GET() == 5.5045103E36F);
            assert(pack.event_id_GET() == CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_DISCONNECT);
            assert(pack.p2_GET() == -1.4421027E38F);
            assert(pack.p3_GET() == 2.657891E38F);
        });
        GroundControl.CAMERA_STATUS p179 = CommunicationChannel.new_CAMERA_STATUS();
        PH.setPack(p179);
        p179.img_idx_SET((char)46484) ;
        p179.cam_idx_SET((char)225) ;
        p179.p2_SET(-1.4421027E38F) ;
        p179.event_id_SET(CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_DISCONNECT) ;
        p179.time_usec_SET(5203911896583327149L) ;
        p179.p1_SET(5.5045103E36F) ;
        p179.p3_SET(2.657891E38F) ;
        p179.target_system_SET((char)55) ;
        p179.p4_SET(2.3151375E38F) ;
        CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_FEEDBACK.add((src, ph, pack) ->
        {
            assert(pack.cam_idx_GET() == (char)193);
            assert(pack.lng_GET() == -1590249080);
            assert(pack.foc_len_GET() == -1.991666E38F);
            assert(pack.roll_GET() == -6.015623E37F);
            assert(pack.img_idx_GET() == (char)3684);
            assert(pack.target_system_GET() == (char)93);
            assert(pack.flags_GET() == CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_PHOTO);
            assert(pack.yaw_GET() == 3.2410295E38F);
            assert(pack.alt_rel_GET() == -3.0965728E37F);
            assert(pack.pitch_GET() == -1.0677763E38F);
            assert(pack.alt_msl_GET() == 2.37183E38F);
            assert(pack.lat_GET() == 1528329111);
            assert(pack.time_usec_GET() == 8110922246997437132L);
        });
        GroundControl.CAMERA_FEEDBACK p180 = CommunicationChannel.new_CAMERA_FEEDBACK();
        PH.setPack(p180);
        p180.lng_SET(-1590249080) ;
        p180.img_idx_SET((char)3684) ;
        p180.lat_SET(1528329111) ;
        p180.foc_len_SET(-1.991666E38F) ;
        p180.yaw_SET(3.2410295E38F) ;
        p180.roll_SET(-6.015623E37F) ;
        p180.pitch_SET(-1.0677763E38F) ;
        p180.flags_SET(CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_PHOTO) ;
        p180.target_system_SET((char)93) ;
        p180.alt_rel_SET(-3.0965728E37F) ;
        p180.alt_msl_SET(2.37183E38F) ;
        p180.time_usec_SET(8110922246997437132L) ;
        p180.cam_idx_SET((char)193) ;
        CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BATTERY2.add((src, ph, pack) ->
        {
            assert(pack.voltage_GET() == (char)38238);
            assert(pack.current_battery_GET() == (short) -9020);
        });
        GroundControl.BATTERY2 p181 = CommunicationChannel.new_BATTERY2();
        PH.setPack(p181);
        p181.current_battery_SET((short) -9020) ;
        p181.voltage_SET((char)38238) ;
        CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AHRS3.add((src, ph, pack) ->
        {
            assert(pack.v2_GET() == 5.9294184E37F);
            assert(pack.altitude_GET() == 2.5196645E37F);
            assert(pack.yaw_GET() == 1.7336296E38F);
            assert(pack.v4_GET() == -2.4571693E38F);
            assert(pack.pitch_GET() == 2.3202088E37F);
            assert(pack.v1_GET() == 2.313892E38F);
            assert(pack.lng_GET() == -1076435822);
            assert(pack.lat_GET() == -1601770179);
            assert(pack.v3_GET() == -4.3309875E36F);
            assert(pack.roll_GET() == -1.5792081E38F);
        });
        GroundControl.AHRS3 p182 = CommunicationChannel.new_AHRS3();
        PH.setPack(p182);
        p182.roll_SET(-1.5792081E38F) ;
        p182.yaw_SET(1.7336296E38F) ;
        p182.v4_SET(-2.4571693E38F) ;
        p182.v3_SET(-4.3309875E36F) ;
        p182.v2_SET(5.9294184E37F) ;
        p182.v1_SET(2.313892E38F) ;
        p182.lat_SET(-1601770179) ;
        p182.lng_SET(-1076435822) ;
        p182.altitude_SET(2.5196645E37F) ;
        p182.pitch_SET(2.3202088E37F) ;
        CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)220);
            assert(pack.target_component_GET() == (char)149);
        });
        GroundControl.AUTOPILOT_VERSION_REQUEST p183 = CommunicationChannel.new_AUTOPILOT_VERSION_REQUEST();
        PH.setPack(p183);
        p183.target_system_SET((char)220) ;
        p183.target_component_SET((char)149) ;
        CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_REMOTE_LOG_DATA_BLOCK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)69);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)145, (char)231, (char)226, (char)156, (char)197, (char)102, (char)132, (char)204, (char)115, (char)154, (char)105, (char)4, (char)80, (char)134, (char)189, (char)89, (char)226, (char)40, (char)186, (char)51, (char)71, (char)212, (char)98, (char)115, (char)216, (char)166, (char)71, (char)68, (char)101, (char)10, (char)30, (char)72, (char)159, (char)183, (char)180, (char)191, (char)197, (char)215, (char)146, (char)114, (char)149, (char)137, (char)15, (char)134, (char)180, (char)243, (char)207, (char)196, (char)8, (char)232, (char)128, (char)141, (char)146, (char)97, (char)112, (char)54, (char)148, (char)92, (char)119, (char)20, (char)76, (char)189, (char)234, (char)107, (char)188, (char)116, (char)11, (char)209, (char)63, (char)165, (char)246, (char)81, (char)210, (char)59, (char)146, (char)253, (char)118, (char)10, (char)152, (char)53, (char)247, (char)71, (char)152, (char)8, (char)168, (char)35, (char)72, (char)125, (char)240, (char)232, (char)144, (char)152, (char)31, (char)157, (char)109, (char)42, (char)172, (char)230, (char)244, (char)207, (char)221, (char)170, (char)177, (char)244, (char)77, (char)60, (char)36, (char)243, (char)120, (char)70, (char)113, (char)204, (char)101, (char)128, (char)253, (char)142, (char)180, (char)174, (char)252, (char)190, (char)148, (char)78, (char)177, (char)219, (char)51, (char)15, (char)76, (char)84, (char)240, (char)184, (char)142, (char)67, (char)225, (char)35, (char)139, (char)3, (char)67, (char)225, (char)184, (char)131, (char)80, (char)131, (char)54, (char)75, (char)176, (char)50, (char)27, (char)251, (char)147, (char)105, (char)84, (char)98, (char)18, (char)69, (char)141, (char)184, (char)210, (char)184, (char)55, (char)236, (char)253, (char)17, (char)16, (char)56, (char)236, (char)220, (char)94, (char)8, (char)31, (char)64, (char)167, (char)173, (char)101, (char)25, (char)231, (char)127, (char)126, (char)248, (char)88, (char)249, (char)168, (char)164, (char)86, (char)238, (char)153, (char)122, (char)19, (char)73, (char)56, (char)109, (char)111, (char)152, (char)154, (char)224, (char)183, (char)203, (char)81, (char)104, (char)162, (char)116}));
            assert(pack.seqno_GET() == MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP);
            assert(pack.target_system_GET() == (char)91);
        });
        GroundControl.REMOTE_LOG_DATA_BLOCK p184 = CommunicationChannel.new_REMOTE_LOG_DATA_BLOCK();
        PH.setPack(p184);
        p184.seqno_SET(MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP) ;
        p184.target_system_SET((char)91) ;
        p184.target_component_SET((char)69) ;
        p184.data__SET(new char[] {(char)145, (char)231, (char)226, (char)156, (char)197, (char)102, (char)132, (char)204, (char)115, (char)154, (char)105, (char)4, (char)80, (char)134, (char)189, (char)89, (char)226, (char)40, (char)186, (char)51, (char)71, (char)212, (char)98, (char)115, (char)216, (char)166, (char)71, (char)68, (char)101, (char)10, (char)30, (char)72, (char)159, (char)183, (char)180, (char)191, (char)197, (char)215, (char)146, (char)114, (char)149, (char)137, (char)15, (char)134, (char)180, (char)243, (char)207, (char)196, (char)8, (char)232, (char)128, (char)141, (char)146, (char)97, (char)112, (char)54, (char)148, (char)92, (char)119, (char)20, (char)76, (char)189, (char)234, (char)107, (char)188, (char)116, (char)11, (char)209, (char)63, (char)165, (char)246, (char)81, (char)210, (char)59, (char)146, (char)253, (char)118, (char)10, (char)152, (char)53, (char)247, (char)71, (char)152, (char)8, (char)168, (char)35, (char)72, (char)125, (char)240, (char)232, (char)144, (char)152, (char)31, (char)157, (char)109, (char)42, (char)172, (char)230, (char)244, (char)207, (char)221, (char)170, (char)177, (char)244, (char)77, (char)60, (char)36, (char)243, (char)120, (char)70, (char)113, (char)204, (char)101, (char)128, (char)253, (char)142, (char)180, (char)174, (char)252, (char)190, (char)148, (char)78, (char)177, (char)219, (char)51, (char)15, (char)76, (char)84, (char)240, (char)184, (char)142, (char)67, (char)225, (char)35, (char)139, (char)3, (char)67, (char)225, (char)184, (char)131, (char)80, (char)131, (char)54, (char)75, (char)176, (char)50, (char)27, (char)251, (char)147, (char)105, (char)84, (char)98, (char)18, (char)69, (char)141, (char)184, (char)210, (char)184, (char)55, (char)236, (char)253, (char)17, (char)16, (char)56, (char)236, (char)220, (char)94, (char)8, (char)31, (char)64, (char)167, (char)173, (char)101, (char)25, (char)231, (char)127, (char)126, (char)248, (char)88, (char)249, (char)168, (char)164, (char)86, (char)238, (char)153, (char)122, (char)19, (char)73, (char)56, (char)109, (char)111, (char)152, (char)154, (char)224, (char)183, (char)203, (char)81, (char)104, (char)162, (char)116}, 0) ;
        CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_REMOTE_LOG_BLOCK_STATUS.add((src, ph, pack) ->
        {
            assert(pack.status_GET() == MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK);
            assert(pack.seqno_GET() == 3713539949L);
            assert(pack.target_component_GET() == (char)115);
            assert(pack.target_system_GET() == (char)0);
        });
        GroundControl.REMOTE_LOG_BLOCK_STATUS p185 = CommunicationChannel.new_REMOTE_LOG_BLOCK_STATUS();
        PH.setPack(p185);
        p185.target_system_SET((char)0) ;
        p185.seqno_SET(3713539949L) ;
        p185.status_SET(MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK) ;
        p185.target_component_SET((char)115) ;
        CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LED_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)15);
            assert(pack.instance_GET() == (char)71);
            assert(Arrays.equals(pack.custom_bytes_GET(),  new char[] {(char)198, (char)238, (char)12, (char)175, (char)197, (char)82, (char)226, (char)188, (char)39, (char)78, (char)13, (char)166, (char)125, (char)120, (char)83, (char)123, (char)191, (char)31, (char)58, (char)178, (char)103, (char)70, (char)70, (char)152}));
            assert(pack.pattern_GET() == (char)129);
            assert(pack.custom_len_GET() == (char)77);
            assert(pack.target_system_GET() == (char)110);
        });
        GroundControl.LED_CONTROL p186 = CommunicationChannel.new_LED_CONTROL();
        PH.setPack(p186);
        p186.custom_len_SET((char)77) ;
        p186.target_component_SET((char)15) ;
        p186.instance_SET((char)71) ;
        p186.target_system_SET((char)110) ;
        p186.custom_bytes_SET(new char[] {(char)198, (char)238, (char)12, (char)175, (char)197, (char)82, (char)226, (char)188, (char)39, (char)78, (char)13, (char)166, (char)125, (char)120, (char)83, (char)123, (char)191, (char)31, (char)58, (char)178, (char)103, (char)70, (char)70, (char)152}, 0) ;
        p186.pattern_SET((char)129) ;
        CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MAG_CAL_PROGRESS.add((src, ph, pack) ->
        {
            assert(pack.compass_id_GET() == (char)0);
            assert(pack.direction_y_GET() == -1.9533283E38F);
            assert(pack.completion_pct_GET() == (char)105);
            assert(pack.direction_x_GET() == -1.1336051E38F);
            assert(pack.cal_mask_GET() == (char)108);
            assert(Arrays.equals(pack.completion_mask_GET(),  new char[] {(char)102, (char)140, (char)239, (char)236, (char)58, (char)74, (char)151, (char)37, (char)2, (char)17}));
            assert(pack.cal_status_GET() == MAG_CAL_STATUS.MAG_CAL_NOT_STARTED);
            assert(pack.attempt_GET() == (char)103);
            assert(pack.direction_z_GET() == 1.7386073E38F);
        });
        GroundControl.MAG_CAL_PROGRESS p191 = CommunicationChannel.new_MAG_CAL_PROGRESS();
        PH.setPack(p191);
        p191.attempt_SET((char)103) ;
        p191.completion_mask_SET(new char[] {(char)102, (char)140, (char)239, (char)236, (char)58, (char)74, (char)151, (char)37, (char)2, (char)17}, 0) ;
        p191.direction_y_SET(-1.9533283E38F) ;
        p191.direction_z_SET(1.7386073E38F) ;
        p191.compass_id_SET((char)0) ;
        p191.completion_pct_SET((char)105) ;
        p191.direction_x_SET(-1.1336051E38F) ;
        p191.cal_status_SET(MAG_CAL_STATUS.MAG_CAL_NOT_STARTED) ;
        p191.cal_mask_SET((char)108) ;
        CommunicationChannel.instance.send(p191);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MAG_CAL_REPORT.add((src, ph, pack) ->
        {
            assert(pack.cal_mask_GET() == (char)83);
            assert(pack.ofs_y_GET() == 2.0569498E37F);
            assert(pack.autosaved_GET() == (char)155);
            assert(pack.diag_z_GET() == -4.6295547E36F);
            assert(pack.diag_y_GET() == 1.5221754E38F);
            assert(pack.ofs_x_GET() == 2.5042022E38F);
            assert(pack.offdiag_y_GET() == -1.631227E38F);
            assert(pack.offdiag_x_GET() == 1.4212573E38F);
            assert(pack.ofs_z_GET() == -2.447175E38F);
            assert(pack.compass_id_GET() == (char)38);
            assert(pack.fitness_GET() == 2.68583E38F);
            assert(pack.cal_status_GET() == MAG_CAL_STATUS.MAG_CAL_WAITING_TO_START);
            assert(pack.diag_x_GET() == -2.7661315E38F);
            assert(pack.offdiag_z_GET() == -6.428535E37F);
        });
        GroundControl.MAG_CAL_REPORT p192 = CommunicationChannel.new_MAG_CAL_REPORT();
        PH.setPack(p192);
        p192.offdiag_y_SET(-1.631227E38F) ;
        p192.ofs_y_SET(2.0569498E37F) ;
        p192.ofs_x_SET(2.5042022E38F) ;
        p192.autosaved_SET((char)155) ;
        p192.diag_x_SET(-2.7661315E38F) ;
        p192.cal_mask_SET((char)83) ;
        p192.offdiag_z_SET(-6.428535E37F) ;
        p192.ofs_z_SET(-2.447175E38F) ;
        p192.diag_y_SET(1.5221754E38F) ;
        p192.diag_z_SET(-4.6295547E36F) ;
        p192.offdiag_x_SET(1.4212573E38F) ;
        p192.fitness_SET(2.68583E38F) ;
        p192.cal_status_SET(MAG_CAL_STATUS.MAG_CAL_WAITING_TO_START) ;
        p192.compass_id_SET((char)38) ;
        CommunicationChannel.instance.send(p192);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_EKF_STATUS_REPORT.add((src, ph, pack) ->
        {
            assert(pack.pos_vert_variance_GET() == 4.537413E37F);
            assert(pack.velocity_variance_GET() == 1.1665115E38F);
            assert(pack.terrain_alt_variance_GET() == -1.015551E38F);
            assert(pack.compass_variance_GET() == -1.8752826E38F);
            assert(pack.flags_GET() == (EKF_STATUS_FLAGS.EKF_VELOCITY_HORIZ |
                                        EKF_STATUS_FLAGS.EKF_POS_VERT_ABS |
                                        EKF_STATUS_FLAGS.EKF_POS_HORIZ_ABS |
                                        EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_REL |
                                        EKF_STATUS_FLAGS.EKF_POS_VERT_AGL));
            assert(pack.pos_horiz_variance_GET() == 1.5511017E38F);
        });
        GroundControl.EKF_STATUS_REPORT p193 = CommunicationChannel.new_EKF_STATUS_REPORT();
        PH.setPack(p193);
        p193.pos_vert_variance_SET(4.537413E37F) ;
        p193.compass_variance_SET(-1.8752826E38F) ;
        p193.flags_SET((EKF_STATUS_FLAGS.EKF_VELOCITY_HORIZ |
                        EKF_STATUS_FLAGS.EKF_POS_VERT_ABS |
                        EKF_STATUS_FLAGS.EKF_POS_HORIZ_ABS |
                        EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_REL |
                        EKF_STATUS_FLAGS.EKF_POS_VERT_AGL)) ;
        p193.terrain_alt_variance_SET(-1.015551E38F) ;
        p193.velocity_variance_SET(1.1665115E38F) ;
        p193.pos_horiz_variance_SET(1.5511017E38F) ;
        CommunicationChannel.instance.send(p193);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PID_TUNING.add((src, ph, pack) ->
        {
            assert(pack.axis_GET() == PID_TUNING_AXIS.PID_TUNING_LANDING);
            assert(pack.FF_GET() == -2.3234881E38F);
            assert(pack.I_GET() == -6.3348273E37F);
            assert(pack.D_GET() == -7.816279E37F);
            assert(pack.P_GET() == -1.613884E38F);
            assert(pack.achieved_GET() == 1.8748928E38F);
            assert(pack.desired_GET() == -2.2744347E38F);
        });
        GroundControl.PID_TUNING p194 = CommunicationChannel.new_PID_TUNING();
        PH.setPack(p194);
        p194.I_SET(-6.3348273E37F) ;
        p194.FF_SET(-2.3234881E38F) ;
        p194.achieved_SET(1.8748928E38F) ;
        p194.axis_SET(PID_TUNING_AXIS.PID_TUNING_LANDING) ;
        p194.D_SET(-7.816279E37F) ;
        p194.desired_SET(-2.2744347E38F) ;
        p194.P_SET(-1.613884E38F) ;
        CommunicationChannel.instance.send(p194);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GIMBAL_REPORT.add((src, ph, pack) ->
        {
            assert(pack.delta_velocity_y_GET() == -1.1466771E38F);
            assert(pack.joint_roll_GET() == -2.9806611E38F);
            assert(pack.delta_time_GET() == 1.0546941E36F);
            assert(pack.delta_angle_x_GET() == 1.6249461E38F);
            assert(pack.joint_az_GET() == 1.2080473E38F);
            assert(pack.delta_angle_z_GET() == -3.1618642E38F);
            assert(pack.delta_angle_y_GET() == -3.0658016E38F);
            assert(pack.delta_velocity_x_GET() == 6.7379955E37F);
            assert(pack.delta_velocity_z_GET() == -2.0745553E37F);
            assert(pack.target_system_GET() == (char)97);
            assert(pack.joint_el_GET() == 1.5762334E38F);
            assert(pack.target_component_GET() == (char)210);
        });
        GroundControl.GIMBAL_REPORT p200 = CommunicationChannel.new_GIMBAL_REPORT();
        PH.setPack(p200);
        p200.delta_velocity_x_SET(6.7379955E37F) ;
        p200.joint_el_SET(1.5762334E38F) ;
        p200.delta_angle_y_SET(-3.0658016E38F) ;
        p200.delta_time_SET(1.0546941E36F) ;
        p200.delta_angle_x_SET(1.6249461E38F) ;
        p200.delta_velocity_z_SET(-2.0745553E37F) ;
        p200.joint_az_SET(1.2080473E38F) ;
        p200.delta_velocity_y_SET(-1.1466771E38F) ;
        p200.target_system_SET((char)97) ;
        p200.joint_roll_SET(-2.9806611E38F) ;
        p200.delta_angle_z_SET(-3.1618642E38F) ;
        p200.target_component_SET((char)210) ;
        CommunicationChannel.instance.send(p200);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GIMBAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)120);
            assert(pack.target_system_GET() == (char)137);
            assert(pack.demanded_rate_x_GET() == 2.9545305E38F);
            assert(pack.demanded_rate_y_GET() == -1.4512274E38F);
            assert(pack.demanded_rate_z_GET() == 1.061196E38F);
        });
        GroundControl.GIMBAL_CONTROL p201 = CommunicationChannel.new_GIMBAL_CONTROL();
        PH.setPack(p201);
        p201.demanded_rate_x_SET(2.9545305E38F) ;
        p201.demanded_rate_z_SET(1.061196E38F) ;
        p201.target_component_SET((char)120) ;
        p201.demanded_rate_y_SET(-1.4512274E38F) ;
        p201.target_system_SET((char)137) ;
        CommunicationChannel.instance.send(p201);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GIMBAL_TORQUE_CMD_REPORT.add((src, ph, pack) ->
        {
            assert(pack.el_torque_cmd_GET() == (short)32686);
            assert(pack.target_system_GET() == (char)129);
            assert(pack.target_component_GET() == (char)91);
            assert(pack.rl_torque_cmd_GET() == (short) -21085);
            assert(pack.az_torque_cmd_GET() == (short)10262);
        });
        GroundControl.GIMBAL_TORQUE_CMD_REPORT p214 = CommunicationChannel.new_GIMBAL_TORQUE_CMD_REPORT();
        PH.setPack(p214);
        p214.target_system_SET((char)129) ;
        p214.rl_torque_cmd_SET((short) -21085) ;
        p214.el_torque_cmd_SET((short)32686) ;
        p214.az_torque_cmd_SET((short)10262) ;
        p214.target_component_SET((char)91) ;
        CommunicationChannel.instance.send(p214);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GOPRO_HEARTBEAT.add((src, ph, pack) ->
        {
            assert(pack.status_GET() == GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_ERROR);
            assert(pack.capture_mode_GET() == GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_TIME_LAPSE);
            assert(pack.flags_GET() == GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING);
        });
        GroundControl.GOPRO_HEARTBEAT p215 = CommunicationChannel.new_GOPRO_HEARTBEAT();
        PH.setPack(p215);
        p215.status_SET(GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_ERROR) ;
        p215.capture_mode_SET(GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_TIME_LAPSE) ;
        p215.flags_SET(GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING) ;
        CommunicationChannel.instance.send(p215);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GOPRO_GET_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)79);
            assert(pack.cmd_id_GET() == GOPRO_COMMAND.GOPRO_COMMAND_BATTERY);
            assert(pack.target_component_GET() == (char)71);
        });
        GroundControl.GOPRO_GET_REQUEST p216 = CommunicationChannel.new_GOPRO_GET_REQUEST();
        PH.setPack(p216);
        p216.target_component_SET((char)71) ;
        p216.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_BATTERY) ;
        p216.target_system_SET((char)79) ;
        CommunicationChannel.instance.send(p216);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GOPRO_GET_RESPONSE.add((src, ph, pack) ->
        {
            assert(pack.cmd_id_GET() == GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_WHITE_BALANCE);
            assert(Arrays.equals(pack.value_GET(),  new char[] {(char)99, (char)186, (char)25, (char)55}));
            assert(pack.status_GET() == GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS);
        });
        GroundControl.GOPRO_GET_RESPONSE p217 = CommunicationChannel.new_GOPRO_GET_RESPONSE();
        PH.setPack(p217);
        p217.value_SET(new char[] {(char)99, (char)186, (char)25, (char)55}, 0) ;
        p217.status_SET(GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS) ;
        p217.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_WHITE_BALANCE) ;
        CommunicationChannel.instance.send(p217);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GOPRO_SET_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.value_GET(),  new char[] {(char)142, (char)66, (char)40, (char)249}));
            assert(pack.target_component_GET() == (char)255);
            assert(pack.target_system_GET() == (char)124);
            assert(pack.cmd_id_GET() == GOPRO_COMMAND.GOPRO_COMMAND_TIME);
        });
        GroundControl.GOPRO_SET_REQUEST p218 = CommunicationChannel.new_GOPRO_SET_REQUEST();
        PH.setPack(p218);
        p218.value_SET(new char[] {(char)142, (char)66, (char)40, (char)249}, 0) ;
        p218.target_component_SET((char)255) ;
        p218.target_system_SET((char)124) ;
        p218.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_TIME) ;
        CommunicationChannel.instance.send(p218);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GOPRO_SET_RESPONSE.add((src, ph, pack) ->
        {
            assert(pack.cmd_id_GET() == GOPRO_COMMAND.GOPRO_COMMAND_PHOTO_RESOLUTION);
            assert(pack.status_GET() == GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS);
        });
        GroundControl.GOPRO_SET_RESPONSE p219 = CommunicationChannel.new_GOPRO_SET_RESPONSE();
        PH.setPack(p219);
        p219.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_PHOTO_RESOLUTION) ;
        p219.status_SET(GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS) ;
        CommunicationChannel.instance.send(p219);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RPM.add((src, ph, pack) ->
        {
            assert(pack.rpm2_GET() == -8.266629E36F);
            assert(pack.rpm1_GET() == -3.9788558E37F);
        });
        GroundControl.RPM p226 = CommunicationChannel.new_RPM();
        PH.setPack(p226);
        p226.rpm2_SET(-8.266629E36F) ;
        p226.rpm1_SET(-3.9788558E37F) ;
        CommunicationChannel.instance.send(p226);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        CommunicationChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3153336557457367852L);
            assert(pack.tas_ratio_GET() == -2.7609895E38F);
            assert(pack.pos_horiz_accuracy_GET() == 2.9904973E38F);
            assert(pack.hagl_ratio_GET() == 2.3860537E38F);
            assert(pack.vel_ratio_GET() == 2.5628226E38F);
            assert(pack.pos_vert_accuracy_GET() == 1.830441E38F);
            assert(pack.pos_vert_ratio_GET() == -6.688263E37F);
            assert(pack.pos_horiz_ratio_GET() == -6.3484576E37F);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE));
            assert(pack.mag_ratio_GET() == 3.0529983E38F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.pos_vert_ratio_SET(-6.688263E37F) ;
        p230.mag_ratio_SET(3.0529983E38F) ;
        p230.tas_ratio_SET(-2.7609895E38F) ;
        p230.time_usec_SET(3153336557457367852L) ;
        p230.hagl_ratio_SET(2.3860537E38F) ;
        p230.pos_vert_accuracy_SET(1.830441E38F) ;
        p230.pos_horiz_accuracy_SET(2.9904973E38F) ;
        p230.pos_horiz_ratio_SET(-6.3484576E37F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE)) ;
        p230.vel_ratio_SET(2.5628226E38F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_y_GET() == 7.2646237E37F);
            assert(pack.time_usec_GET() == 2446628072000167013L);
            assert(pack.vert_accuracy_GET() == -7.507471E37F);
            assert(pack.wind_x_GET() == -2.3710597E38F);
            assert(pack.var_horiz_GET() == -7.629312E37F);
            assert(pack.horiz_accuracy_GET() == -8.864322E37F);
            assert(pack.wind_z_GET() == 2.040943E38F);
            assert(pack.wind_alt_GET() == -3.0288272E38F);
            assert(pack.var_vert_GET() == -1.0738294E38F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.var_vert_SET(-1.0738294E38F) ;
        p231.wind_y_SET(7.2646237E37F) ;
        p231.time_usec_SET(2446628072000167013L) ;
        p231.wind_x_SET(-2.3710597E38F) ;
        p231.var_horiz_SET(-7.629312E37F) ;
        p231.wind_z_SET(2.040943E38F) ;
        p231.wind_alt_SET(-3.0288272E38F) ;
        p231.horiz_accuracy_SET(-8.864322E37F) ;
        p231.vert_accuracy_SET(-7.507471E37F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -1.5153988E38F);
            assert(pack.ve_GET() == -3.2773146E37F);
            assert(pack.vn_GET() == -2.900602E38F);
            assert(pack.vdop_GET() == -3.400306E38F);
            assert(pack.speed_accuracy_GET() == -1.8243307E38F);
            assert(pack.lat_GET() == -890296583);
            assert(pack.satellites_visible_GET() == (char)228);
            assert(pack.lon_GET() == -399560248);
            assert(pack.time_week_ms_GET() == 2018514218L);
            assert(pack.hdop_GET() == 2.5319836E38F);
            assert(pack.vd_GET() == 1.6629948E38F);
            assert(pack.vert_accuracy_GET() == -2.2779044E38F);
            assert(pack.gps_id_GET() == (char)177);
            assert(pack.time_week_GET() == (char)45261);
            assert(pack.horiz_accuracy_GET() == 1.6692177E37F);
            assert(pack.time_usec_GET() == 8006814271726556036L);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY));
            assert(pack.fix_type_GET() == (char)11);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.speed_accuracy_SET(-1.8243307E38F) ;
        p232.vert_accuracy_SET(-2.2779044E38F) ;
        p232.time_week_ms_SET(2018514218L) ;
        p232.lat_SET(-890296583) ;
        p232.satellites_visible_SET((char)228) ;
        p232.alt_SET(-1.5153988E38F) ;
        p232.time_usec_SET(8006814271726556036L) ;
        p232.vn_SET(-2.900602E38F) ;
        p232.ve_SET(-3.2773146E37F) ;
        p232.time_week_SET((char)45261) ;
        p232.lon_SET(-399560248) ;
        p232.fix_type_SET((char)11) ;
        p232.vd_SET(1.6629948E38F) ;
        p232.hdop_SET(2.5319836E38F) ;
        p232.horiz_accuracy_SET(1.6692177E37F) ;
        p232.vdop_SET(-3.400306E38F) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY)) ;
        p232.gps_id_SET((char)177) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (char)195);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)114, (char)50, (char)107, (char)63, (char)215, (char)81, (char)97, (char)122, (char)116, (char)152, (char)141, (char)58, (char)115, (char)46, (char)12, (char)65, (char)7, (char)194, (char)38, (char)88, (char)85, (char)243, (char)188, (char)16, (char)71, (char)135, (char)127, (char)123, (char)141, (char)219, (char)166, (char)223, (char)94, (char)15, (char)158, (char)27, (char)131, (char)124, (char)118, (char)79, (char)105, (char)253, (char)45, (char)243, (char)91, (char)212, (char)156, (char)66, (char)237, (char)127, (char)191, (char)206, (char)131, (char)228, (char)66, (char)19, (char)163, (char)192, (char)183, (char)237, (char)221, (char)141, (char)57, (char)18, (char)137, (char)236, (char)44, (char)206, (char)179, (char)245, (char)90, (char)187, (char)94, (char)200, (char)124, (char)164, (char)76, (char)129, (char)70, (char)184, (char)170, (char)121, (char)139, (char)214, (char)53, (char)233, (char)60, (char)84, (char)77, (char)56, (char)173, (char)78, (char)162, (char)134, (char)90, (char)103, (char)132, (char)245, (char)241, (char)205, (char)170, (char)37, (char)65, (char)46, (char)182, (char)25, (char)74, (char)19, (char)204, (char)230, (char)218, (char)137, (char)72, (char)204, (char)64, (char)115, (char)154, (char)90, (char)48, (char)113, (char)182, (char)18, (char)255, (char)215, (char)40, (char)97, (char)86, (char)34, (char)125, (char)233, (char)186, (char)137, (char)15, (char)112, (char)110, (char)248, (char)241, (char)2, (char)19, (char)179, (char)236, (char)174, (char)231, (char)163, (char)210, (char)27, (char)82, (char)120, (char)5, (char)76, (char)29, (char)66, (char)175, (char)176, (char)94, (char)168, (char)50, (char)123, (char)159, (char)214, (char)119, (char)246, (char)60, (char)163, (char)62, (char)142, (char)194, (char)177, (char)97, (char)236, (char)85, (char)77, (char)238, (char)208, (char)90, (char)222, (char)164, (char)90, (char)20, (char)38}));
            assert(pack.len_GET() == (char)180);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)195) ;
        p233.len_SET((char)180) ;
        p233.data__SET(new char[] {(char)114, (char)50, (char)107, (char)63, (char)215, (char)81, (char)97, (char)122, (char)116, (char)152, (char)141, (char)58, (char)115, (char)46, (char)12, (char)65, (char)7, (char)194, (char)38, (char)88, (char)85, (char)243, (char)188, (char)16, (char)71, (char)135, (char)127, (char)123, (char)141, (char)219, (char)166, (char)223, (char)94, (char)15, (char)158, (char)27, (char)131, (char)124, (char)118, (char)79, (char)105, (char)253, (char)45, (char)243, (char)91, (char)212, (char)156, (char)66, (char)237, (char)127, (char)191, (char)206, (char)131, (char)228, (char)66, (char)19, (char)163, (char)192, (char)183, (char)237, (char)221, (char)141, (char)57, (char)18, (char)137, (char)236, (char)44, (char)206, (char)179, (char)245, (char)90, (char)187, (char)94, (char)200, (char)124, (char)164, (char)76, (char)129, (char)70, (char)184, (char)170, (char)121, (char)139, (char)214, (char)53, (char)233, (char)60, (char)84, (char)77, (char)56, (char)173, (char)78, (char)162, (char)134, (char)90, (char)103, (char)132, (char)245, (char)241, (char)205, (char)170, (char)37, (char)65, (char)46, (char)182, (char)25, (char)74, (char)19, (char)204, (char)230, (char)218, (char)137, (char)72, (char)204, (char)64, (char)115, (char)154, (char)90, (char)48, (char)113, (char)182, (char)18, (char)255, (char)215, (char)40, (char)97, (char)86, (char)34, (char)125, (char)233, (char)186, (char)137, (char)15, (char)112, (char)110, (char)248, (char)241, (char)2, (char)19, (char)179, (char)236, (char)174, (char)231, (char)163, (char)210, (char)27, (char)82, (char)120, (char)5, (char)76, (char)29, (char)66, (char)175, (char)176, (char)94, (char)168, (char)50, (char)123, (char)159, (char)214, (char)119, (char)246, (char)60, (char)163, (char)62, (char)142, (char)194, (char)177, (char)97, (char)236, (char)85, (char)77, (char)238, (char)208, (char)90, (char)222, (char)164, (char)90, (char)20, (char)38}, 0) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == -1399820793);
            assert(pack.airspeed_GET() == (char)205);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED));
            assert(pack.roll_GET() == (short)17421);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
            assert(pack.pitch_GET() == (short) -11400);
            assert(pack.airspeed_sp_GET() == (char)9);
            assert(pack.groundspeed_GET() == (char)56);
            assert(pack.heading_GET() == (char)64548);
            assert(pack.throttle_GET() == (byte)92);
            assert(pack.climb_rate_GET() == (byte) - 62);
            assert(pack.altitude_sp_GET() == (short)5651);
            assert(pack.wp_num_GET() == (char)147);
            assert(pack.longitude_GET() == 976831389);
            assert(pack.custom_mode_GET() == 569279377L);
            assert(pack.failsafe_GET() == (char)234);
            assert(pack.altitude_amsl_GET() == (short)13855);
            assert(pack.temperature_GET() == (byte) - 100);
            assert(pack.wp_distance_GET() == (char)57152);
            assert(pack.temperature_air_GET() == (byte)1);
            assert(pack.battery_remaining_GET() == (char)191);
            assert(pack.heading_sp_GET() == (short) -19333);
            assert(pack.gps_nsat_GET() == (char)9);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.wp_num_SET((char)147) ;
        p234.custom_mode_SET(569279377L) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED) ;
        p234.altitude_amsl_SET((short)13855) ;
        p234.temperature_air_SET((byte)1) ;
        p234.longitude_SET(976831389) ;
        p234.failsafe_SET((char)234) ;
        p234.roll_SET((short)17421) ;
        p234.battery_remaining_SET((char)191) ;
        p234.latitude_SET(-1399820793) ;
        p234.gps_nsat_SET((char)9) ;
        p234.airspeed_SET((char)205) ;
        p234.altitude_sp_SET((short)5651) ;
        p234.groundspeed_SET((char)56) ;
        p234.temperature_SET((byte) - 100) ;
        p234.wp_distance_SET((char)57152) ;
        p234.heading_SET((char)64548) ;
        p234.climb_rate_SET((byte) - 62) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED)) ;
        p234.pitch_SET((short) -11400) ;
        p234.heading_sp_SET((short) -19333) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS) ;
        p234.airspeed_sp_SET((char)9) ;
        p234.throttle_SET((byte)92) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.clipping_0_GET() == 1818676676L);
            assert(pack.vibration_y_GET() == -2.325555E38F);
            assert(pack.clipping_1_GET() == 1408155835L);
            assert(pack.vibration_x_GET() == 1.1980528E38F);
            assert(pack.clipping_2_GET() == 2688757079L);
            assert(pack.vibration_z_GET() == -2.0091968E38F);
            assert(pack.time_usec_GET() == 3157509113707506869L);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(3157509113707506869L) ;
        p241.clipping_1_SET(1408155835L) ;
        p241.vibration_y_SET(-2.325555E38F) ;
        p241.vibration_x_SET(1.1980528E38F) ;
        p241.clipping_2_SET(2688757079L) ;
        p241.vibration_z_SET(-2.0091968E38F) ;
        p241.clipping_0_SET(1818676676L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == 956201147);
            assert(pack.latitude_GET() == 1675282775);
            assert(pack.y_GET() == -6.328612E37F);
            assert(pack.z_GET() == -2.780153E38F);
            assert(pack.approach_x_GET() == 1.9123534E37F);
            assert(pack.approach_z_GET() == 1.1033103E38F);
            assert(pack.approach_y_GET() == 3.1771176E38F);
            assert(pack.time_usec_TRY(ph) == 3557841725531923442L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-5.4714766E37F, 9.342564E37F, -2.9177494E38F, 6.689733E37F}));
            assert(pack.altitude_GET() == -591265172);
            assert(pack.x_GET() == -8.644552E37F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.approach_x_SET(1.9123534E37F) ;
        p242.time_usec_SET(3557841725531923442L, PH) ;
        p242.latitude_SET(1675282775) ;
        p242.x_SET(-8.644552E37F) ;
        p242.longitude_SET(956201147) ;
        p242.approach_z_SET(1.1033103E38F) ;
        p242.z_SET(-2.780153E38F) ;
        p242.altitude_SET(-591265172) ;
        p242.approach_y_SET(3.1771176E38F) ;
        p242.y_SET(-6.328612E37F) ;
        p242.q_SET(new float[] {-5.4714766E37F, 9.342564E37F, -2.9177494E38F, 6.689733E37F}, 0) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 1085646671);
            assert(pack.approach_z_GET() == -2.6174944E38F);
            assert(pack.y_GET() == 2.0028245E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-8.4436523E37F, 1.74962E38F, 1.1348366E38F, 2.7422497E38F}));
            assert(pack.approach_y_GET() == 3.0393452E38F);
            assert(pack.altitude_GET() == -1558425315);
            assert(pack.z_GET() == 7.3670265E37F);
            assert(pack.time_usec_TRY(ph) == 920024715011780212L);
            assert(pack.target_system_GET() == (char)232);
            assert(pack.approach_x_GET() == 6.7104575E37F);
            assert(pack.x_GET() == 6.931291E37F);
            assert(pack.longitude_GET() == 1447875585);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.altitude_SET(-1558425315) ;
        p243.q_SET(new float[] {-8.4436523E37F, 1.74962E38F, 1.1348366E38F, 2.7422497E38F}, 0) ;
        p243.y_SET(2.0028245E38F) ;
        p243.z_SET(7.3670265E37F) ;
        p243.longitude_SET(1447875585) ;
        p243.latitude_SET(1085646671) ;
        p243.target_system_SET((char)232) ;
        p243.x_SET(6.931291E37F) ;
        p243.approach_x_SET(6.7104575E37F) ;
        p243.approach_y_SET(3.0393452E38F) ;
        p243.time_usec_SET(920024715011780212L, PH) ;
        p243.approach_z_SET(-2.6174944E38F) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.interval_us_GET() == 1489758255);
            assert(pack.message_id_GET() == (char)62443);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(1489758255) ;
        p244.message_id_SET((char)62443) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            assert(pack.squawk_GET() == (char)25820);
            assert(pack.lat_GET() == 1045193314);
            assert(pack.ICAO_address_GET() == 3821651925L);
            assert(pack.callsign_LEN(ph) == 9);
            assert(pack.callsign_TRY(ph).equals("PhhxmfgvU"));
            assert(pack.heading_GET() == (char)47616);
            assert(pack.ver_velocity_GET() == (short)21796);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE));
            assert(pack.altitude_GET() == -807274057);
            assert(pack.hor_velocity_GET() == (char)48621);
            assert(pack.lon_GET() == -717411443);
            assert(pack.tslc_GET() == (char)219);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SERVICE_SURFACE);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.hor_velocity_SET((char)48621) ;
        p246.tslc_SET((char)219) ;
        p246.heading_SET((char)47616) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE)) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH) ;
        p246.lat_SET(1045193314) ;
        p246.squawk_SET((char)25820) ;
        p246.ICAO_address_SET(3821651925L) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SERVICE_SURFACE) ;
        p246.lon_SET(-717411443) ;
        p246.callsign_SET("PhhxmfgvU", PH) ;
        p246.altitude_SET(-807274057) ;
        p246.ver_velocity_SET((short)21796) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.time_to_minimum_delta_GET() == 1.4467013E38F);
            assert(pack.threat_level_GET() == (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE));
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
            assert(pack.horizontal_minimum_delta_GET() == -1.8693437E38F);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
            assert(pack.id_GET() == 3430014036L);
            assert(pack.altitude_minimum_delta_GET() == 3.6918524E37F);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.altitude_minimum_delta_SET(3.6918524E37F) ;
        p247.threat_level_SET((MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE)) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        p247.id_SET(3430014036L) ;
        p247.time_to_minimum_delta_SET(1.4467013E38F) ;
        p247.horizontal_minimum_delta_SET(-1.8693437E38F) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.message_type_GET() == (char)58833);
            assert(pack.target_network_GET() == (char)47);
            assert(pack.target_system_GET() == (char)132);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)68, (char)219, (char)47, (char)213, (char)232, (char)32, (char)217, (char)151, (char)41, (char)23, (char)116, (char)55, (char)134, (char)81, (char)85, (char)29, (char)88, (char)70, (char)229, (char)106, (char)251, (char)203, (char)242, (char)172, (char)47, (char)225, (char)98, (char)252, (char)120, (char)139, (char)117, (char)182, (char)12, (char)131, (char)0, (char)67, (char)209, (char)101, (char)201, (char)93, (char)249, (char)101, (char)192, (char)152, (char)134, (char)124, (char)81, (char)76, (char)154, (char)200, (char)234, (char)165, (char)62, (char)145, (char)49, (char)221, (char)147, (char)251, (char)160, (char)8, (char)34, (char)209, (char)12, (char)149, (char)85, (char)65, (char)209, (char)1, (char)75, (char)171, (char)194, (char)12, (char)67, (char)53, (char)168, (char)208, (char)44, (char)115, (char)130, (char)29, (char)220, (char)108, (char)0, (char)123, (char)107, (char)130, (char)222, (char)26, (char)35, (char)73, (char)100, (char)154, (char)56, (char)134, (char)22, (char)39, (char)47, (char)130, (char)217, (char)169, (char)22, (char)255, (char)78, (char)56, (char)96, (char)188, (char)159, (char)216, (char)74, (char)218, (char)227, (char)74, (char)9, (char)63, (char)78, (char)70, (char)134, (char)207, (char)186, (char)234, (char)205, (char)233, (char)74, (char)247, (char)69, (char)238, (char)32, (char)18, (char)216, (char)231, (char)45, (char)102, (char)205, (char)217, (char)172, (char)125, (char)121, (char)242, (char)193, (char)230, (char)125, (char)219, (char)10, (char)81, (char)28, (char)207, (char)12, (char)122, (char)39, (char)50, (char)97, (char)191, (char)63, (char)196, (char)248, (char)80, (char)43, (char)219, (char)179, (char)137, (char)2, (char)201, (char)139, (char)130, (char)247, (char)222, (char)133, (char)28, (char)238, (char)37, (char)203, (char)109, (char)240, (char)91, (char)122, (char)94, (char)34, (char)137, (char)42, (char)114, (char)175, (char)13, (char)81, (char)43, (char)158, (char)75, (char)29, (char)8, (char)19, (char)73, (char)197, (char)148, (char)131, (char)63, (char)122, (char)52, (char)71, (char)220, (char)229, (char)21, (char)100, (char)175, (char)99, (char)118, (char)169, (char)59, (char)46, (char)151, (char)135, (char)115, (char)226, (char)92, (char)40, (char)231, (char)41, (char)247, (char)106, (char)134, (char)117, (char)89, (char)56, (char)130, (char)209, (char)100, (char)250, (char)229, (char)94, (char)18, (char)203, (char)160, (char)94, (char)191, (char)141, (char)240, (char)98, (char)81, (char)155, (char)17, (char)11, (char)131, (char)187, (char)59, (char)165, (char)111, (char)66, (char)41, (char)157, (char)229, (char)157}));
            assert(pack.target_component_GET() == (char)21);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.message_type_SET((char)58833) ;
        p248.target_network_SET((char)47) ;
        p248.target_component_SET((char)21) ;
        p248.target_system_SET((char)132) ;
        p248.payload_SET(new char[] {(char)68, (char)219, (char)47, (char)213, (char)232, (char)32, (char)217, (char)151, (char)41, (char)23, (char)116, (char)55, (char)134, (char)81, (char)85, (char)29, (char)88, (char)70, (char)229, (char)106, (char)251, (char)203, (char)242, (char)172, (char)47, (char)225, (char)98, (char)252, (char)120, (char)139, (char)117, (char)182, (char)12, (char)131, (char)0, (char)67, (char)209, (char)101, (char)201, (char)93, (char)249, (char)101, (char)192, (char)152, (char)134, (char)124, (char)81, (char)76, (char)154, (char)200, (char)234, (char)165, (char)62, (char)145, (char)49, (char)221, (char)147, (char)251, (char)160, (char)8, (char)34, (char)209, (char)12, (char)149, (char)85, (char)65, (char)209, (char)1, (char)75, (char)171, (char)194, (char)12, (char)67, (char)53, (char)168, (char)208, (char)44, (char)115, (char)130, (char)29, (char)220, (char)108, (char)0, (char)123, (char)107, (char)130, (char)222, (char)26, (char)35, (char)73, (char)100, (char)154, (char)56, (char)134, (char)22, (char)39, (char)47, (char)130, (char)217, (char)169, (char)22, (char)255, (char)78, (char)56, (char)96, (char)188, (char)159, (char)216, (char)74, (char)218, (char)227, (char)74, (char)9, (char)63, (char)78, (char)70, (char)134, (char)207, (char)186, (char)234, (char)205, (char)233, (char)74, (char)247, (char)69, (char)238, (char)32, (char)18, (char)216, (char)231, (char)45, (char)102, (char)205, (char)217, (char)172, (char)125, (char)121, (char)242, (char)193, (char)230, (char)125, (char)219, (char)10, (char)81, (char)28, (char)207, (char)12, (char)122, (char)39, (char)50, (char)97, (char)191, (char)63, (char)196, (char)248, (char)80, (char)43, (char)219, (char)179, (char)137, (char)2, (char)201, (char)139, (char)130, (char)247, (char)222, (char)133, (char)28, (char)238, (char)37, (char)203, (char)109, (char)240, (char)91, (char)122, (char)94, (char)34, (char)137, (char)42, (char)114, (char)175, (char)13, (char)81, (char)43, (char)158, (char)75, (char)29, (char)8, (char)19, (char)73, (char)197, (char)148, (char)131, (char)63, (char)122, (char)52, (char)71, (char)220, (char)229, (char)21, (char)100, (char)175, (char)99, (char)118, (char)169, (char)59, (char)46, (char)151, (char)135, (char)115, (char)226, (char)92, (char)40, (char)231, (char)41, (char)247, (char)106, (char)134, (char)117, (char)89, (char)56, (char)130, (char)209, (char)100, (char)250, (char)229, (char)94, (char)18, (char)203, (char)160, (char)94, (char)191, (char)141, (char)240, (char)98, (char)81, (char)155, (char)17, (char)11, (char)131, (char)187, (char)59, (char)165, (char)111, (char)66, (char)41, (char)157, (char)229, (char)157}, 0) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)166);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)37, (byte) - 50, (byte)35, (byte) - 123, (byte) - 104, (byte) - 24, (byte) - 46, (byte)67, (byte)23, (byte) - 42, (byte)63, (byte)67, (byte)121, (byte)127, (byte)55, (byte) - 11, (byte) - 27, (byte)126, (byte)107, (byte)16, (byte)72, (byte)84, (byte)16, (byte) - 19, (byte) - 45, (byte)91, (byte)108, (byte) - 76, (byte) - 71, (byte)113, (byte)85, (byte)123}));
            assert(pack.ver_GET() == (char)87);
            assert(pack.address_GET() == (char)21467);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)21467) ;
        p249.value_SET(new byte[] {(byte)37, (byte) - 50, (byte)35, (byte) - 123, (byte) - 104, (byte) - 24, (byte) - 46, (byte)67, (byte)23, (byte) - 42, (byte)63, (byte)67, (byte)121, (byte)127, (byte)55, (byte) - 11, (byte) - 27, (byte)126, (byte)107, (byte)16, (byte)72, (byte)84, (byte)16, (byte) - 19, (byte) - 45, (byte)91, (byte)108, (byte) - 76, (byte) - 71, (byte)113, (byte)85, (byte)123}, 0) ;
        p249.type_SET((char)166) ;
        p249.ver_SET((char)87) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -3.6512417E37F);
            assert(pack.y_GET() == 1.345481E38F);
            assert(pack.x_GET() == 2.0788551E38F);
            assert(pack.time_usec_GET() == 7589549924700793065L);
            assert(pack.name_LEN(ph) == 7);
            assert(pack.name_TRY(ph).equals("gypmUHq"));
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.time_usec_SET(7589549924700793065L) ;
        p250.z_SET(-3.6512417E37F) ;
        p250.y_SET(1.345481E38F) ;
        p250.name_SET("gypmUHq", PH) ;
        p250.x_SET(2.0788551E38F) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1120179783L);
            assert(pack.value_GET() == 1.7218833E38F);
            assert(pack.name_LEN(ph) == 6);
            assert(pack.name_TRY(ph).equals("KIePkp"));
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.name_SET("KIePkp", PH) ;
        p251.time_boot_ms_SET(1120179783L) ;
        p251.value_SET(1.7218833E38F) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 3);
            assert(pack.name_TRY(ph).equals("nGu"));
            assert(pack.value_GET() == -1291625996);
            assert(pack.time_boot_ms_GET() == 1475509588L);
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(1475509588L) ;
        p252.value_SET(-1291625996) ;
        p252.name_SET("nGu", PH) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_INFO);
            assert(pack.text_LEN(ph) == 1);
            assert(pack.text_TRY(ph).equals("f"));
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_INFO) ;
        p253.text_SET("f", PH) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 164985467L);
            assert(pack.value_GET() == -1.8341582E38F);
            assert(pack.ind_GET() == (char)18);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(164985467L) ;
        p254.value_SET(-1.8341582E38F) ;
        p254.ind_SET((char)18) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)12, (char)247, (char)237, (char)213, (char)119, (char)217, (char)51, (char)2, (char)105, (char)170, (char)198, (char)201, (char)245, (char)238, (char)244, (char)194, (char)166, (char)113, (char)248, (char)96, (char)51, (char)215, (char)227, (char)132, (char)115, (char)130, (char)29, (char)4, (char)117, (char)140, (char)232, (char)72}));
            assert(pack.target_component_GET() == (char)5);
            assert(pack.target_system_GET() == (char)210);
            assert(pack.initial_timestamp_GET() == 490685544847180487L);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.secret_key_SET(new char[] {(char)12, (char)247, (char)237, (char)213, (char)119, (char)217, (char)51, (char)2, (char)105, (char)170, (char)198, (char)201, (char)245, (char)238, (char)244, (char)194, (char)166, (char)113, (char)248, (char)96, (char)51, (char)215, (char)227, (char)132, (char)115, (char)130, (char)29, (char)4, (char)117, (char)140, (char)232, (char)72}, 0) ;
        p256.target_system_SET((char)210) ;
        p256.target_component_SET((char)5) ;
        p256.initial_timestamp_SET(490685544847180487L) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.last_change_ms_GET() == 2224733031L);
            assert(pack.state_GET() == (char)137);
            assert(pack.time_boot_ms_GET() == 3706871212L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.last_change_ms_SET(2224733031L) ;
        p257.time_boot_ms_SET(3706871212L) ;
        p257.state_SET((char)137) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)218);
            assert(pack.tune_LEN(ph) == 9);
            assert(pack.tune_TRY(ph).equals("licmeziuq"));
            assert(pack.target_system_GET() == (char)130);
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)130) ;
        p258.tune_SET("licmeziuq", PH) ;
        p258.target_component_SET((char)218) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)70, (char)144, (char)110, (char)212, (char)10, (char)16, (char)82, (char)93, (char)120, (char)29, (char)176, (char)90, (char)134, (char)120, (char)233, (char)203, (char)65, (char)240, (char)96, (char)206, (char)53, (char)116, (char)104, (char)167, (char)209, (char)17, (char)194, (char)223, (char)142, (char)121, (char)215, (char)196}));
            assert(pack.focal_length_GET() == 5.853219E37F);
            assert(pack.resolution_h_GET() == (char)22287);
            assert(pack.cam_definition_uri_LEN(ph) == 119);
            assert(pack.cam_definition_uri_TRY(ph).equals("kwatixjbhzZomuphmcugatiwnuyyRpLjvKxvwVemTqzwdvqplvmtubcCapcfemirsjmsnzqbWslhggmLrnQajblhiOhsoSicmwiabsnsmPlgnssjldnxjgr"));
            assert(pack.resolution_v_GET() == (char)46354);
            assert(pack.sensor_size_h_GET() == -5.056505E36F);
            assert(pack.time_boot_ms_GET() == 2428670330L);
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO));
            assert(pack.cam_definition_version_GET() == (char)28277);
            assert(pack.lens_id_GET() == (char)79);
            assert(pack.sensor_size_v_GET() == 3.2158888E38F);
            assert(pack.firmware_version_GET() == 3528745553L);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)166, (char)190, (char)77, (char)219, (char)81, (char)217, (char)193, (char)105, (char)210, (char)118, (char)115, (char)49, (char)140, (char)37, (char)63, (char)102, (char)71, (char)157, (char)58, (char)26, (char)209, (char)197, (char)139, (char)75, (char)241, (char)29, (char)145, (char)26, (char)140, (char)60, (char)146, (char)152}));
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.sensor_size_v_SET(3.2158888E38F) ;
        p259.focal_length_SET(5.853219E37F) ;
        p259.sensor_size_h_SET(-5.056505E36F) ;
        p259.firmware_version_SET(3528745553L) ;
        p259.resolution_v_SET((char)46354) ;
        p259.time_boot_ms_SET(2428670330L) ;
        p259.model_name_SET(new char[] {(char)70, (char)144, (char)110, (char)212, (char)10, (char)16, (char)82, (char)93, (char)120, (char)29, (char)176, (char)90, (char)134, (char)120, (char)233, (char)203, (char)65, (char)240, (char)96, (char)206, (char)53, (char)116, (char)104, (char)167, (char)209, (char)17, (char)194, (char)223, (char)142, (char)121, (char)215, (char)196}, 0) ;
        p259.cam_definition_uri_SET("kwatixjbhzZomuphmcugatiwnuyyRpLjvKxvwVemTqzwdvqplvmtubcCapcfemirsjmsnzqbWslhggmLrnQajblhiOhsoSicmwiabsnsmPlgnssjldnxjgr", PH) ;
        p259.vendor_name_SET(new char[] {(char)166, (char)190, (char)77, (char)219, (char)81, (char)217, (char)193, (char)105, (char)210, (char)118, (char)115, (char)49, (char)140, (char)37, (char)63, (char)102, (char)71, (char)157, (char)58, (char)26, (char)209, (char)197, (char)139, (char)75, (char)241, (char)29, (char)145, (char)26, (char)140, (char)60, (char)146, (char)152}, 0) ;
        p259.lens_id_SET((char)79) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO)) ;
        p259.resolution_h_SET((char)22287) ;
        p259.cam_definition_version_SET((char)28277) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.mode_id_GET() == (CAMERA_MODE.CAMERA_MODE_IMAGE |
                                          CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY));
            assert(pack.time_boot_ms_GET() == 1771997582L);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.mode_id_SET((CAMERA_MODE.CAMERA_MODE_IMAGE |
                          CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY)) ;
        p260.time_boot_ms_SET(1771997582L) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2233220976L);
            assert(pack.available_capacity_GET() == 1.4329158E38F);
            assert(pack.storage_count_GET() == (char)83);
            assert(pack.write_speed_GET() == -2.8799807E38F);
            assert(pack.status_GET() == (char)195);
            assert(pack.read_speed_GET() == -1.890848E38F);
            assert(pack.storage_id_GET() == (char)67);
            assert(pack.total_capacity_GET() == 1.6573317E38F);
            assert(pack.used_capacity_GET() == 1.1840719E38F);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.status_SET((char)195) ;
        p261.storage_count_SET((char)83) ;
        p261.available_capacity_SET(1.4329158E38F) ;
        p261.used_capacity_SET(1.1840719E38F) ;
        p261.storage_id_SET((char)67) ;
        p261.total_capacity_SET(1.6573317E38F) ;
        p261.write_speed_SET(-2.8799807E38F) ;
        p261.time_boot_ms_SET(2233220976L) ;
        p261.read_speed_SET(-1.890848E38F) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.image_interval_GET() == -2.6772588E38F);
            assert(pack.time_boot_ms_GET() == 85633702L);
            assert(pack.recording_time_ms_GET() == 3095426708L);
            assert(pack.video_status_GET() == (char)142);
            assert(pack.available_capacity_GET() == 2.3929035E38F);
            assert(pack.image_status_GET() == (char)153);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.recording_time_ms_SET(3095426708L) ;
        p262.time_boot_ms_SET(85633702L) ;
        p262.video_status_SET((char)142) ;
        p262.image_interval_SET(-2.6772588E38F) ;
        p262.image_status_SET((char)153) ;
        p262.available_capacity_SET(2.3929035E38F) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.file_url_LEN(ph) == 34);
            assert(pack.file_url_TRY(ph).equals("izsejqhnimCLdbxjkpdpbyVxzsquhdgxFE"));
            assert(pack.camera_id_GET() == (char)122);
            assert(pack.relative_alt_GET() == -1318756373);
            assert(pack.image_index_GET() == -1002101888);
            assert(pack.time_utc_GET() == 2997344717045127899L);
            assert(pack.alt_GET() == -1921599942);
            assert(pack.capture_result_GET() == (byte)88);
            assert(pack.lon_GET() == -818112080);
            assert(pack.time_boot_ms_GET() == 1585535133L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.3076339E38F, -7.0550917E37F, -2.7822673E38F, 1.1223888E38F}));
            assert(pack.lat_GET() == 1761723634);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.relative_alt_SET(-1318756373) ;
        p263.camera_id_SET((char)122) ;
        p263.alt_SET(-1921599942) ;
        p263.time_boot_ms_SET(1585535133L) ;
        p263.file_url_SET("izsejqhnimCLdbxjkpdpbyVxzsquhdgxFE", PH) ;
        p263.capture_result_SET((byte)88) ;
        p263.image_index_SET(-1002101888) ;
        p263.q_SET(new float[] {3.3076339E38F, -7.0550917E37F, -2.7822673E38F, 1.1223888E38F}, 0) ;
        p263.time_utc_SET(2997344717045127899L) ;
        p263.lon_SET(-818112080) ;
        p263.lat_SET(1761723634) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.takeoff_time_utc_GET() == 7543922004133882630L);
            assert(pack.flight_uuid_GET() == 3398763081401635787L);
            assert(pack.time_boot_ms_GET() == 1572123928L);
            assert(pack.arming_time_utc_GET() == 635934975901117926L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.arming_time_utc_SET(635934975901117926L) ;
        p264.time_boot_ms_SET(1572123928L) ;
        p264.flight_uuid_SET(3398763081401635787L) ;
        p264.takeoff_time_utc_SET(7543922004133882630L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3341525337L);
            assert(pack.pitch_GET() == -2.8911198E37F);
            assert(pack.roll_GET() == 1.417244E38F);
            assert(pack.yaw_GET() == -1.6035064E38F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.yaw_SET(-1.6035064E38F) ;
        p265.roll_SET(1.417244E38F) ;
        p265.time_boot_ms_SET(3341525337L) ;
        p265.pitch_SET(-2.8911198E37F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.first_message_offset_GET() == (char)76);
            assert(pack.target_component_GET() == (char)13);
            assert(pack.target_system_GET() == (char)129);
            assert(pack.length_GET() == (char)170);
            assert(pack.sequence_GET() == (char)60254);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)7, (char)79, (char)9, (char)168, (char)58, (char)130, (char)9, (char)167, (char)229, (char)189, (char)69, (char)177, (char)218, (char)148, (char)154, (char)15, (char)250, (char)201, (char)219, (char)40, (char)47, (char)145, (char)240, (char)252, (char)205, (char)54, (char)30, (char)10, (char)175, (char)60, (char)89, (char)166, (char)2, (char)232, (char)98, (char)97, (char)110, (char)169, (char)205, (char)183, (char)82, (char)32, (char)229, (char)32, (char)66, (char)128, (char)25, (char)65, (char)213, (char)217, (char)104, (char)41, (char)168, (char)199, (char)163, (char)253, (char)142, (char)228, (char)81, (char)249, (char)235, (char)232, (char)211, (char)32, (char)114, (char)179, (char)6, (char)28, (char)133, (char)222, (char)200, (char)224, (char)215, (char)158, (char)46, (char)95, (char)127, (char)89, (char)73, (char)124, (char)67, (char)203, (char)133, (char)73, (char)90, (char)206, (char)42, (char)243, (char)129, (char)117, (char)26, (char)69, (char)181, (char)252, (char)180, (char)72, (char)24, (char)89, (char)133, (char)89, (char)215, (char)170, (char)91, (char)252, (char)76, (char)173, (char)87, (char)168, (char)181, (char)195, (char)18, (char)94, (char)163, (char)99, (char)148, (char)22, (char)13, (char)173, (char)86, (char)192, (char)140, (char)223, (char)193, (char)129, (char)0, (char)200, (char)185, (char)85, (char)127, (char)56, (char)201, (char)107, (char)101, (char)141, (char)109, (char)220, (char)156, (char)18, (char)250, (char)179, (char)214, (char)81, (char)161, (char)39, (char)170, (char)55, (char)112, (char)87, (char)143, (char)161, (char)28, (char)4, (char)253, (char)205, (char)48, (char)24, (char)38, (char)234, (char)148, (char)245, (char)114, (char)165, (char)139, (char)78, (char)233, (char)80, (char)109, (char)190, (char)129, (char)220, (char)99, (char)85, (char)7, (char)14, (char)235, (char)92, (char)160, (char)193, (char)158, (char)76, (char)244, (char)64, (char)181, (char)129, (char)134, (char)234, (char)7, (char)193, (char)54, (char)80, (char)89, (char)142, (char)64, (char)16, (char)199, (char)127, (char)186, (char)133, (char)125, (char)212, (char)61, (char)66, (char)198, (char)196, (char)233, (char)245, (char)207, (char)89, (char)25, (char)108, (char)203, (char)95, (char)70, (char)255, (char)27, (char)235, (char)231, (char)251, (char)50, (char)215, (char)220, (char)28, (char)113, (char)208, (char)100, (char)177, (char)121, (char)50, (char)29, (char)167, (char)6, (char)122, (char)57, (char)204, (char)49, (char)209, (char)165, (char)109, (char)155, (char)164, (char)12, (char)56, (char)113, (char)165, (char)82, (char)223, (char)147, (char)170, (char)140}));
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_system_SET((char)129) ;
        p266.length_SET((char)170) ;
        p266.target_component_SET((char)13) ;
        p266.sequence_SET((char)60254) ;
        p266.first_message_offset_SET((char)76) ;
        p266.data__SET(new char[] {(char)7, (char)79, (char)9, (char)168, (char)58, (char)130, (char)9, (char)167, (char)229, (char)189, (char)69, (char)177, (char)218, (char)148, (char)154, (char)15, (char)250, (char)201, (char)219, (char)40, (char)47, (char)145, (char)240, (char)252, (char)205, (char)54, (char)30, (char)10, (char)175, (char)60, (char)89, (char)166, (char)2, (char)232, (char)98, (char)97, (char)110, (char)169, (char)205, (char)183, (char)82, (char)32, (char)229, (char)32, (char)66, (char)128, (char)25, (char)65, (char)213, (char)217, (char)104, (char)41, (char)168, (char)199, (char)163, (char)253, (char)142, (char)228, (char)81, (char)249, (char)235, (char)232, (char)211, (char)32, (char)114, (char)179, (char)6, (char)28, (char)133, (char)222, (char)200, (char)224, (char)215, (char)158, (char)46, (char)95, (char)127, (char)89, (char)73, (char)124, (char)67, (char)203, (char)133, (char)73, (char)90, (char)206, (char)42, (char)243, (char)129, (char)117, (char)26, (char)69, (char)181, (char)252, (char)180, (char)72, (char)24, (char)89, (char)133, (char)89, (char)215, (char)170, (char)91, (char)252, (char)76, (char)173, (char)87, (char)168, (char)181, (char)195, (char)18, (char)94, (char)163, (char)99, (char)148, (char)22, (char)13, (char)173, (char)86, (char)192, (char)140, (char)223, (char)193, (char)129, (char)0, (char)200, (char)185, (char)85, (char)127, (char)56, (char)201, (char)107, (char)101, (char)141, (char)109, (char)220, (char)156, (char)18, (char)250, (char)179, (char)214, (char)81, (char)161, (char)39, (char)170, (char)55, (char)112, (char)87, (char)143, (char)161, (char)28, (char)4, (char)253, (char)205, (char)48, (char)24, (char)38, (char)234, (char)148, (char)245, (char)114, (char)165, (char)139, (char)78, (char)233, (char)80, (char)109, (char)190, (char)129, (char)220, (char)99, (char)85, (char)7, (char)14, (char)235, (char)92, (char)160, (char)193, (char)158, (char)76, (char)244, (char)64, (char)181, (char)129, (char)134, (char)234, (char)7, (char)193, (char)54, (char)80, (char)89, (char)142, (char)64, (char)16, (char)199, (char)127, (char)186, (char)133, (char)125, (char)212, (char)61, (char)66, (char)198, (char)196, (char)233, (char)245, (char)207, (char)89, (char)25, (char)108, (char)203, (char)95, (char)70, (char)255, (char)27, (char)235, (char)231, (char)251, (char)50, (char)215, (char)220, (char)28, (char)113, (char)208, (char)100, (char)177, (char)121, (char)50, (char)29, (char)167, (char)6, (char)122, (char)57, (char)204, (char)49, (char)209, (char)165, (char)109, (char)155, (char)164, (char)12, (char)56, (char)113, (char)165, (char)82, (char)223, (char)147, (char)170, (char)140}, 0) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)200);
            assert(pack.sequence_GET() == (char)5372);
            assert(pack.target_system_GET() == (char)243);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)213, (char)140, (char)78, (char)174, (char)166, (char)228, (char)230, (char)83, (char)116, (char)35, (char)4, (char)122, (char)248, (char)81, (char)98, (char)112, (char)214, (char)167, (char)193, (char)56, (char)114, (char)53, (char)183, (char)181, (char)249, (char)37, (char)180, (char)68, (char)159, (char)143, (char)145, (char)248, (char)209, (char)221, (char)60, (char)161, (char)129, (char)95, (char)204, (char)196, (char)118, (char)60, (char)229, (char)182, (char)118, (char)241, (char)29, (char)130, (char)44, (char)92, (char)138, (char)192, (char)147, (char)205, (char)156, (char)164, (char)44, (char)45, (char)150, (char)132, (char)78, (char)234, (char)25, (char)26, (char)217, (char)210, (char)12, (char)113, (char)208, (char)154, (char)240, (char)185, (char)23, (char)219, (char)12, (char)167, (char)224, (char)82, (char)179, (char)24, (char)199, (char)91, (char)242, (char)244, (char)44, (char)33, (char)103, (char)154, (char)237, (char)143, (char)42, (char)234, (char)221, (char)164, (char)245, (char)83, (char)233, (char)192, (char)30, (char)192, (char)67, (char)248, (char)23, (char)182, (char)194, (char)212, (char)241, (char)124, (char)102, (char)130, (char)29, (char)71, (char)134, (char)154, (char)40, (char)247, (char)90, (char)98, (char)37, (char)67, (char)30, (char)120, (char)240, (char)170, (char)21, (char)221, (char)61, (char)26, (char)203, (char)184, (char)193, (char)238, (char)234, (char)117, (char)213, (char)66, (char)177, (char)115, (char)0, (char)42, (char)184, (char)101, (char)215, (char)18, (char)173, (char)219, (char)185, (char)124, (char)232, (char)44, (char)175, (char)164, (char)245, (char)167, (char)48, (char)182, (char)190, (char)101, (char)117, (char)104, (char)191, (char)169, (char)250, (char)66, (char)194, (char)57, (char)52, (char)2, (char)96, (char)215, (char)182, (char)142, (char)56, (char)234, (char)90, (char)19, (char)190, (char)142, (char)211, (char)170, (char)145, (char)112, (char)117, (char)18, (char)28, (char)41, (char)227, (char)150, (char)183, (char)149, (char)254, (char)0, (char)135, (char)236, (char)0, (char)209, (char)151, (char)189, (char)17, (char)239, (char)108, (char)126, (char)179, (char)83, (char)224, (char)9, (char)165, (char)22, (char)244, (char)16, (char)124, (char)131, (char)13, (char)173, (char)39, (char)163, (char)240, (char)248, (char)39, (char)232, (char)3, (char)139, (char)231, (char)4, (char)85, (char)28, (char)149, (char)234, (char)213, (char)200, (char)145, (char)216, (char)78, (char)26, (char)255, (char)228, (char)234, (char)122, (char)119, (char)43, (char)110, (char)3, (char)189, (char)70, (char)61, (char)89, (char)155, (char)235, (char)239}));
            assert(pack.first_message_offset_GET() == (char)68);
            assert(pack.length_GET() == (char)204);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.sequence_SET((char)5372) ;
        p267.target_component_SET((char)200) ;
        p267.target_system_SET((char)243) ;
        p267.first_message_offset_SET((char)68) ;
        p267.length_SET((char)204) ;
        p267.data__SET(new char[] {(char)213, (char)140, (char)78, (char)174, (char)166, (char)228, (char)230, (char)83, (char)116, (char)35, (char)4, (char)122, (char)248, (char)81, (char)98, (char)112, (char)214, (char)167, (char)193, (char)56, (char)114, (char)53, (char)183, (char)181, (char)249, (char)37, (char)180, (char)68, (char)159, (char)143, (char)145, (char)248, (char)209, (char)221, (char)60, (char)161, (char)129, (char)95, (char)204, (char)196, (char)118, (char)60, (char)229, (char)182, (char)118, (char)241, (char)29, (char)130, (char)44, (char)92, (char)138, (char)192, (char)147, (char)205, (char)156, (char)164, (char)44, (char)45, (char)150, (char)132, (char)78, (char)234, (char)25, (char)26, (char)217, (char)210, (char)12, (char)113, (char)208, (char)154, (char)240, (char)185, (char)23, (char)219, (char)12, (char)167, (char)224, (char)82, (char)179, (char)24, (char)199, (char)91, (char)242, (char)244, (char)44, (char)33, (char)103, (char)154, (char)237, (char)143, (char)42, (char)234, (char)221, (char)164, (char)245, (char)83, (char)233, (char)192, (char)30, (char)192, (char)67, (char)248, (char)23, (char)182, (char)194, (char)212, (char)241, (char)124, (char)102, (char)130, (char)29, (char)71, (char)134, (char)154, (char)40, (char)247, (char)90, (char)98, (char)37, (char)67, (char)30, (char)120, (char)240, (char)170, (char)21, (char)221, (char)61, (char)26, (char)203, (char)184, (char)193, (char)238, (char)234, (char)117, (char)213, (char)66, (char)177, (char)115, (char)0, (char)42, (char)184, (char)101, (char)215, (char)18, (char)173, (char)219, (char)185, (char)124, (char)232, (char)44, (char)175, (char)164, (char)245, (char)167, (char)48, (char)182, (char)190, (char)101, (char)117, (char)104, (char)191, (char)169, (char)250, (char)66, (char)194, (char)57, (char)52, (char)2, (char)96, (char)215, (char)182, (char)142, (char)56, (char)234, (char)90, (char)19, (char)190, (char)142, (char)211, (char)170, (char)145, (char)112, (char)117, (char)18, (char)28, (char)41, (char)227, (char)150, (char)183, (char)149, (char)254, (char)0, (char)135, (char)236, (char)0, (char)209, (char)151, (char)189, (char)17, (char)239, (char)108, (char)126, (char)179, (char)83, (char)224, (char)9, (char)165, (char)22, (char)244, (char)16, (char)124, (char)131, (char)13, (char)173, (char)39, (char)163, (char)240, (char)248, (char)39, (char)232, (char)3, (char)139, (char)231, (char)4, (char)85, (char)28, (char)149, (char)234, (char)213, (char)200, (char)145, (char)216, (char)78, (char)26, (char)255, (char)228, (char)234, (char)122, (char)119, (char)43, (char)110, (char)3, (char)189, (char)70, (char)61, (char)89, (char)155, (char)235, (char)239}, 0) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)131);
            assert(pack.target_system_GET() == (char)160);
            assert(pack.sequence_GET() == (char)38710);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)160) ;
        p268.target_component_SET((char)131) ;
        p268.sequence_SET((char)38710) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.resolution_v_GET() == (char)27152);
            assert(pack.resolution_h_GET() == (char)31702);
            assert(pack.framerate_GET() == 1.9923555E38F);
            assert(pack.uri_LEN(ph) == 25);
            assert(pack.uri_TRY(ph).equals("bnjaemWoutqqwheksifnhfgGk"));
            assert(pack.status_GET() == (char)6);
            assert(pack.camera_id_GET() == (char)157);
            assert(pack.rotation_GET() == (char)50606);
            assert(pack.bitrate_GET() == 2183130530L);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)157) ;
        p269.resolution_h_SET((char)31702) ;
        p269.bitrate_SET(2183130530L) ;
        p269.resolution_v_SET((char)27152) ;
        p269.uri_SET("bnjaemWoutqqwheksifnhfgGk", PH) ;
        p269.rotation_SET((char)50606) ;
        p269.status_SET((char)6) ;
        p269.framerate_SET(1.9923555E38F) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.resolution_v_GET() == (char)2806);
            assert(pack.target_system_GET() == (char)71);
            assert(pack.camera_id_GET() == (char)68);
            assert(pack.resolution_h_GET() == (char)16580);
            assert(pack.target_component_GET() == (char)133);
            assert(pack.uri_LEN(ph) == 62);
            assert(pack.uri_TRY(ph).equals("bauettllbtanegbjWwzuovzjelaoTqSodkylsqTxrkyepsckbsmsuQbmbalzrw"));
            assert(pack.framerate_GET() == 4.0778347E37F);
            assert(pack.bitrate_GET() == 2659216265L);
            assert(pack.rotation_GET() == (char)1410);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_component_SET((char)133) ;
        p270.resolution_v_SET((char)2806) ;
        p270.uri_SET("bauettllbtanegbjWwzuovzjelaoTqSodkylsqTxrkyepsckbsmsuQbmbalzrw", PH) ;
        p270.resolution_h_SET((char)16580) ;
        p270.target_system_SET((char)71) ;
        p270.rotation_SET((char)1410) ;
        p270.camera_id_SET((char)68) ;
        p270.bitrate_SET(2659216265L) ;
        p270.framerate_SET(4.0778347E37F) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.password_LEN(ph) == 56);
            assert(pack.password_TRY(ph).equals("jkazihofbdkuznbudojxoAtBrgfienpjdMNjukbkEupkgbryeqzrobkg"));
            assert(pack.ssid_LEN(ph) == 27);
            assert(pack.ssid_TRY(ph).equals("bvwxbcwomwzholfaSfwwihxobqy"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("bvwxbcwomwzholfaSfwwihxobqy", PH) ;
        p299.password_SET("jkazihofbdkuznbudojxoAtBrgfienpjdMNjukbkEupkgbryeqzrobkg", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.max_version_GET() == (char)51402);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)76, (char)19, (char)106, (char)97, (char)138, (char)97, (char)98, (char)83}));
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)117, (char)255, (char)41, (char)122, (char)7, (char)16, (char)150, (char)197}));
            assert(pack.version_GET() == (char)58960);
            assert(pack.min_version_GET() == (char)11068);
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.library_version_hash_SET(new char[] {(char)76, (char)19, (char)106, (char)97, (char)138, (char)97, (char)98, (char)83}, 0) ;
        p300.min_version_SET((char)11068) ;
        p300.version_SET((char)58960) ;
        p300.spec_version_hash_SET(new char[] {(char)117, (char)255, (char)41, (char)122, (char)7, (char)16, (char)150, (char)197}, 0) ;
        p300.max_version_SET((char)51402) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.sub_mode_GET() == (char)18);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
            assert(pack.time_usec_GET() == 3846846577525162890L);
            assert(pack.vendor_specific_status_code_GET() == (char)2635);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
            assert(pack.uptime_sec_GET() == 4206500514L);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL) ;
        p310.uptime_sec_SET(4206500514L) ;
        p310.sub_mode_SET((char)18) ;
        p310.time_usec_SET(3846846577525162890L) ;
        p310.vendor_specific_status_code_SET((char)2635) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.sw_version_major_GET() == (char)148);
            assert(pack.sw_vcs_commit_GET() == 740411201L);
            assert(pack.time_usec_GET() == 3287982984615228946L);
            assert(pack.sw_version_minor_GET() == (char)72);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)251, (char)115, (char)170, (char)120, (char)45, (char)232, (char)76, (char)201, (char)168, (char)244, (char)50, (char)175, (char)72, (char)166, (char)85, (char)234}));
            assert(pack.name_LEN(ph) == 63);
            assert(pack.name_TRY(ph).equals("pxhgcnmqebbkseubnaoJjxhbtpllbgxirPkqidtikcyzHzmdsIkhwhavsajtprt"));
            assert(pack.uptime_sec_GET() == 3668119804L);
            assert(pack.hw_version_major_GET() == (char)164);
            assert(pack.hw_version_minor_GET() == (char)153);
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.hw_unique_id_SET(new char[] {(char)251, (char)115, (char)170, (char)120, (char)45, (char)232, (char)76, (char)201, (char)168, (char)244, (char)50, (char)175, (char)72, (char)166, (char)85, (char)234}, 0) ;
        p311.hw_version_minor_SET((char)153) ;
        p311.hw_version_major_SET((char)164) ;
        p311.sw_vcs_commit_SET(740411201L) ;
        p311.sw_version_major_SET((char)148) ;
        p311.name_SET("pxhgcnmqebbkseubnaoJjxhbtpllbgxirPkqidtikcyzHzmdsIkhwhavsajtprt", PH) ;
        p311.time_usec_SET(3287982984615228946L) ;
        p311.sw_version_minor_SET((char)72) ;
        p311.uptime_sec_SET(3668119804L) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)71);
            assert(pack.param_index_GET() == (short)24700);
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("cuxh"));
            assert(pack.target_component_GET() == (char)54);
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.param_index_SET((short)24700) ;
        p320.target_component_SET((char)54) ;
        p320.target_system_SET((char)71) ;
        p320.param_id_SET("cuxh", PH) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)119);
            assert(pack.target_component_GET() == (char)97);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)119) ;
        p321.target_component_SET((char)97) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)63995);
            assert(pack.param_value_LEN(ph) == 61);
            assert(pack.param_value_TRY(ph).equals("JttrlfitatglpksbfymyhsmbIkrAaHakaNirOfxgsgdtaokishhxmuHaktdel"));
            assert(pack.param_id_LEN(ph) == 3);
            assert(pack.param_id_TRY(ph).equals("Ivz"));
            assert(pack.param_index_GET() == (char)3812);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_index_SET((char)3812) ;
        p322.param_id_SET("Ivz", PH) ;
        p322.param_count_SET((char)63995) ;
        p322.param_value_SET("JttrlfitatglpksbfymyhsmbIkrAaHakaNirOfxgsgdtaokishhxmuHaktdel", PH) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 80);
            assert(pack.param_value_TRY(ph).equals("kBlrrzpgFkuqysiAjnnjtfdbozzundmznfmwoikoycrUpapkccqgzLpalvellktpjvodfVZrtwIcefjy"));
            assert(pack.param_id_LEN(ph) == 14);
            assert(pack.param_id_TRY(ph).equals("mqjckydyrXgjdh"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64);
            assert(pack.target_component_GET() == (char)27);
            assert(pack.target_system_GET() == (char)90);
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_id_SET("mqjckydyrXgjdh", PH) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64) ;
        p323.param_value_SET("kBlrrzpgFkuqysiAjnnjtfdbozzundmznfmwoikoycrUpapkccqgzLpalvellktpjvodfVZrtwIcefjy", PH) ;
        p323.target_system_SET((char)90) ;
        p323.target_component_SET((char)27) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 76);
            assert(pack.param_value_TRY(ph).equals("aftJfTnjcdWqddiyPxOfvsedKglqyeoecjmsufnewulcpQjqnYrjdneiflluzgzzjgxDottdzeia"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("p"));
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_value_SET("aftJfTnjcdWqddiyPxOfvsedKglqyeoecjmsufnewulcpQjqnYrjdneiflluzgzzjgxDottdzeia", PH) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED) ;
        p324.param_id_SET("p", PH) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)60165, (char)28255, (char)42967, (char)9159, (char)63056, (char)58812, (char)32298, (char)49963, (char)3537, (char)5470, (char)46668, (char)35732, (char)29006, (char)40684, (char)59097, (char)29124, (char)19678, (char)54634, (char)20021, (char)28117, (char)30332, (char)18535, (char)54839, (char)17043, (char)38636, (char)14264, (char)25079, (char)31459, (char)2102, (char)27341, (char)47622, (char)10647, (char)36534, (char)2603, (char)61192, (char)33512, (char)14033, (char)40388, (char)61199, (char)22395, (char)19007, (char)32217, (char)52033, (char)16787, (char)65299, (char)11469, (char)46193, (char)9470, (char)1799, (char)18454, (char)18107, (char)35781, (char)25312, (char)61284, (char)35270, (char)51989, (char)55347, (char)13737, (char)1917, (char)11162, (char)3476, (char)33, (char)23312, (char)15570, (char)35659, (char)18605, (char)21263, (char)22017, (char)22705, (char)17693, (char)63835, (char)65220}));
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            assert(pack.min_distance_GET() == (char)17388);
            assert(pack.max_distance_GET() == (char)11216);
            assert(pack.increment_GET() == (char)59);
            assert(pack.time_usec_GET() == 2840665684543347654L);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.max_distance_SET((char)11216) ;
        p330.distances_SET(new char[] {(char)60165, (char)28255, (char)42967, (char)9159, (char)63056, (char)58812, (char)32298, (char)49963, (char)3537, (char)5470, (char)46668, (char)35732, (char)29006, (char)40684, (char)59097, (char)29124, (char)19678, (char)54634, (char)20021, (char)28117, (char)30332, (char)18535, (char)54839, (char)17043, (char)38636, (char)14264, (char)25079, (char)31459, (char)2102, (char)27341, (char)47622, (char)10647, (char)36534, (char)2603, (char)61192, (char)33512, (char)14033, (char)40388, (char)61199, (char)22395, (char)19007, (char)32217, (char)52033, (char)16787, (char)65299, (char)11469, (char)46193, (char)9470, (char)1799, (char)18454, (char)18107, (char)35781, (char)25312, (char)61284, (char)35270, (char)51989, (char)55347, (char)13737, (char)1917, (char)11162, (char)3476, (char)33, (char)23312, (char)15570, (char)35659, (char)18605, (char)21263, (char)22017, (char)22705, (char)17693, (char)63835, (char)65220}, 0) ;
        p330.min_distance_SET((char)17388) ;
        p330.increment_SET((char)59) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        p330.time_usec_SET(2840665684543347654L) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVIONIX_ADSB_OUT_CFG.add((src, ph, pack) ->
        {
            assert(pack.gpsOffsetLat_GET() == UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_6M);
            assert(pack.gpsOffsetLon_GET() == UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA);
            assert(pack.emitterType_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_PARACHUTE);
            assert(pack.ICAO_GET() == 3361882483L);
            assert(pack.aircraftSize_GET() == UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25_34M);
            assert(pack.stallSpeed_GET() == (char)53472);
            assert(pack.rfSelect_GET() == (UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED));
            assert(pack.callsign_LEN(ph) == 5);
            assert(pack.callsign_TRY(ph).equals("uzntb"));
        });
        GroundControl.UAVIONIX_ADSB_OUT_CFG p10001 = CommunicationChannel.new_UAVIONIX_ADSB_OUT_CFG();
        PH.setPack(p10001);
        p10001.callsign_SET("uzntb", PH) ;
        p10001.stallSpeed_SET((char)53472) ;
        p10001.gpsOffsetLon_SET(UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA) ;
        p10001.gpsOffsetLat_SET(UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_6M) ;
        p10001.aircraftSize_SET(UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25_34M) ;
        p10001.ICAO_SET(3361882483L) ;
        p10001.emitterType_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_PARACHUTE) ;
        p10001.rfSelect_SET((UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED)) ;
        CommunicationChannel.instance.send(p10001);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVIONIX_ADSB_OUT_DYNAMIC.add((src, ph, pack) ->
        {
            assert(pack.utcTime_GET() == 141739758L);
            assert(pack.accuracyVert_GET() == (char)34912);
            assert(pack.gpsAlt_GET() == 409926227);
            assert(pack.accuracyHor_GET() == 1005308971L);
            assert(pack.emergencyStatus_GET() == UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_LIFEGUARD_EMERGENCY);
            assert(pack.VelEW_GET() == (short) -27282);
            assert(pack.squawk_GET() == (char)39511);
            assert(pack.accuracyVel_GET() == (char)13190);
            assert(pack.velVert_GET() == (short)71);
            assert(pack.baroAltMSL_GET() == 781406259);
            assert(pack.velNS_GET() == (short)21175);
            assert(pack.state_GET() == (UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND |
                                        UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED |
                                        UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE));
            assert(pack.numSats_GET() == (char)8);
            assert(pack.gpsFix_GET() == UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_1);
            assert(pack.gpsLon_GET() == 362301787);
            assert(pack.gpsLat_GET() == -1769071124);
        });
        GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC p10002 = CommunicationChannel.new_UAVIONIX_ADSB_OUT_DYNAMIC();
        PH.setPack(p10002);
        p10002.gpsLon_SET(362301787) ;
        p10002.gpsLat_SET(-1769071124) ;
        p10002.baroAltMSL_SET(781406259) ;
        p10002.accuracyVert_SET((char)34912) ;
        p10002.emergencyStatus_SET(UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_LIFEGUARD_EMERGENCY) ;
        p10002.gpsAlt_SET(409926227) ;
        p10002.VelEW_SET((short) -27282) ;
        p10002.accuracyVel_SET((char)13190) ;
        p10002.accuracyHor_SET(1005308971L) ;
        p10002.state_SET((UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND |
                          UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED |
                          UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE)) ;
        p10002.gpsFix_SET(UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_1) ;
        p10002.squawk_SET((char)39511) ;
        p10002.velVert_SET((short)71) ;
        p10002.utcTime_SET(141739758L) ;
        p10002.velNS_SET((short)21175) ;
        p10002.numSats_SET((char)8) ;
        CommunicationChannel.instance.send(p10002);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT.add((src, ph, pack) ->
        {
            assert(pack.rfHealth_GET() == (UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_TX));
        });
        GroundControl.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT p10003 = CommunicationChannel.new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();
        PH.setPack(p10003);
        p10003.rfHealth_SET((UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_TX)) ;
        CommunicationChannel.instance.send(p10003);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEVICE_OP_READ.add((src, ph, pack) ->
        {
            assert(pack.bustype_GET() == DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_SPI);
            assert(pack.regstart_GET() == (char)236);
            assert(pack.busname_LEN(ph) == 1);
            assert(pack.busname_TRY(ph).equals("v"));
            assert(pack.request_id_GET() == 913619270L);
            assert(pack.bus_GET() == (char)174);
            assert(pack.target_system_GET() == (char)172);
            assert(pack.count_GET() == (char)158);
            assert(pack.target_component_GET() == (char)70);
            assert(pack.address_GET() == (char)153);
        });
        GroundControl.DEVICE_OP_READ p11000 = CommunicationChannel.new_DEVICE_OP_READ();
        PH.setPack(p11000);
        p11000.address_SET((char)153) ;
        p11000.target_component_SET((char)70) ;
        p11000.regstart_SET((char)236) ;
        p11000.bustype_SET(DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_SPI) ;
        p11000.count_SET((char)158) ;
        p11000.busname_SET("v", PH) ;
        p11000.request_id_SET(913619270L) ;
        p11000.bus_SET((char)174) ;
        p11000.target_system_SET((char)172) ;
        CommunicationChannel.instance.send(p11000);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEVICE_OP_READ_REPLY.add((src, ph, pack) ->
        {
            assert(pack.regstart_GET() == (char)115);
            assert(pack.result_GET() == (char)196);
            assert(pack.request_id_GET() == 3411397057L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)41, (char)160, (char)214, (char)29, (char)12, (char)112, (char)74, (char)19, (char)84, (char)109, (char)212, (char)20, (char)195, (char)65, (char)202, (char)215, (char)176, (char)119, (char)217, (char)57, (char)115, (char)172, (char)136, (char)236, (char)15, (char)51, (char)6, (char)237, (char)22, (char)24, (char)155, (char)143, (char)96, (char)126, (char)170, (char)132, (char)238, (char)191, (char)12, (char)101, (char)233, (char)105, (char)115, (char)5, (char)177, (char)221, (char)90, (char)177, (char)205, (char)141, (char)39, (char)157, (char)53, (char)98, (char)240, (char)128, (char)61, (char)34, (char)103, (char)165, (char)35, (char)243, (char)181, (char)38, (char)24, (char)169, (char)206, (char)108, (char)214, (char)44, (char)226, (char)35, (char)24, (char)181, (char)53, (char)152, (char)104, (char)92, (char)60, (char)219, (char)48, (char)215, (char)244, (char)130, (char)222, (char)51, (char)54, (char)17, (char)240, (char)246, (char)143, (char)31, (char)142, (char)179, (char)161, (char)236, (char)117, (char)133, (char)203, (char)212, (char)237, (char)246, (char)2, (char)143, (char)225, (char)121, (char)157, (char)198, (char)6, (char)104, (char)180, (char)134, (char)231, (char)3, (char)164, (char)243, (char)83, (char)60, (char)32, (char)44, (char)40, (char)77, (char)201, (char)94, (char)213, (char)112, (char)76, (char)221}));
            assert(pack.count_GET() == (char)25);
        });
        GroundControl.DEVICE_OP_READ_REPLY p11001 = CommunicationChannel.new_DEVICE_OP_READ_REPLY();
        PH.setPack(p11001);
        p11001.data__SET(new char[] {(char)41, (char)160, (char)214, (char)29, (char)12, (char)112, (char)74, (char)19, (char)84, (char)109, (char)212, (char)20, (char)195, (char)65, (char)202, (char)215, (char)176, (char)119, (char)217, (char)57, (char)115, (char)172, (char)136, (char)236, (char)15, (char)51, (char)6, (char)237, (char)22, (char)24, (char)155, (char)143, (char)96, (char)126, (char)170, (char)132, (char)238, (char)191, (char)12, (char)101, (char)233, (char)105, (char)115, (char)5, (char)177, (char)221, (char)90, (char)177, (char)205, (char)141, (char)39, (char)157, (char)53, (char)98, (char)240, (char)128, (char)61, (char)34, (char)103, (char)165, (char)35, (char)243, (char)181, (char)38, (char)24, (char)169, (char)206, (char)108, (char)214, (char)44, (char)226, (char)35, (char)24, (char)181, (char)53, (char)152, (char)104, (char)92, (char)60, (char)219, (char)48, (char)215, (char)244, (char)130, (char)222, (char)51, (char)54, (char)17, (char)240, (char)246, (char)143, (char)31, (char)142, (char)179, (char)161, (char)236, (char)117, (char)133, (char)203, (char)212, (char)237, (char)246, (char)2, (char)143, (char)225, (char)121, (char)157, (char)198, (char)6, (char)104, (char)180, (char)134, (char)231, (char)3, (char)164, (char)243, (char)83, (char)60, (char)32, (char)44, (char)40, (char)77, (char)201, (char)94, (char)213, (char)112, (char)76, (char)221}, 0) ;
        p11001.request_id_SET(3411397057L) ;
        p11001.regstart_SET((char)115) ;
        p11001.result_SET((char)196) ;
        p11001.count_SET((char)25) ;
        CommunicationChannel.instance.send(p11001);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEVICE_OP_WRITE.add((src, ph, pack) ->
        {
            assert(pack.bustype_GET() == DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C);
            assert(pack.target_component_GET() == (char)79);
            assert(pack.busname_LEN(ph) == 36);
            assert(pack.busname_TRY(ph).equals("dadNjzffEpolpgwtnIdgasunxZAhizWnlmlu"));
            assert(pack.bus_GET() == (char)148);
            assert(pack.target_system_GET() == (char)45);
            assert(pack.count_GET() == (char)172);
            assert(pack.request_id_GET() == 1849950911L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)38, (char)81, (char)170, (char)156, (char)109, (char)202, (char)150, (char)169, (char)113, (char)235, (char)39, (char)255, (char)66, (char)156, (char)154, (char)108, (char)91, (char)138, (char)116, (char)126, (char)32, (char)137, (char)2, (char)63, (char)91, (char)25, (char)53, (char)170, (char)61, (char)155, (char)160, (char)4, (char)220, (char)219, (char)236, (char)171, (char)179, (char)29, (char)221, (char)7, (char)55, (char)170, (char)42, (char)195, (char)41, (char)45, (char)223, (char)101, (char)17, (char)13, (char)68, (char)64, (char)53, (char)113, (char)148, (char)76, (char)181, (char)230, (char)175, (char)1, (char)64, (char)127, (char)66, (char)46, (char)83, (char)70, (char)45, (char)87, (char)33, (char)194, (char)119, (char)84, (char)187, (char)63, (char)222, (char)119, (char)137, (char)73, (char)95, (char)194, (char)149, (char)149, (char)222, (char)196, (char)34, (char)72, (char)185, (char)96, (char)249, (char)168, (char)202, (char)42, (char)28, (char)109, (char)119, (char)93, (char)152, (char)2, (char)209, (char)189, (char)250, (char)70, (char)142, (char)22, (char)47, (char)220, (char)205, (char)42, (char)96, (char)64, (char)243, (char)246, (char)220, (char)20, (char)171, (char)5, (char)25, (char)37, (char)61, (char)12, (char)54, (char)13, (char)8, (char)199, (char)171, (char)241, (char)89, (char)167}));
            assert(pack.regstart_GET() == (char)144);
            assert(pack.address_GET() == (char)147);
        });
        GroundControl.DEVICE_OP_WRITE p11002 = CommunicationChannel.new_DEVICE_OP_WRITE();
        PH.setPack(p11002);
        p11002.address_SET((char)147) ;
        p11002.regstart_SET((char)144) ;
        p11002.bustype_SET(DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C) ;
        p11002.target_system_SET((char)45) ;
        p11002.target_component_SET((char)79) ;
        p11002.busname_SET("dadNjzffEpolpgwtnIdgasunxZAhizWnlmlu", PH) ;
        p11002.count_SET((char)172) ;
        p11002.bus_SET((char)148) ;
        p11002.request_id_SET(1849950911L) ;
        p11002.data__SET(new char[] {(char)38, (char)81, (char)170, (char)156, (char)109, (char)202, (char)150, (char)169, (char)113, (char)235, (char)39, (char)255, (char)66, (char)156, (char)154, (char)108, (char)91, (char)138, (char)116, (char)126, (char)32, (char)137, (char)2, (char)63, (char)91, (char)25, (char)53, (char)170, (char)61, (char)155, (char)160, (char)4, (char)220, (char)219, (char)236, (char)171, (char)179, (char)29, (char)221, (char)7, (char)55, (char)170, (char)42, (char)195, (char)41, (char)45, (char)223, (char)101, (char)17, (char)13, (char)68, (char)64, (char)53, (char)113, (char)148, (char)76, (char)181, (char)230, (char)175, (char)1, (char)64, (char)127, (char)66, (char)46, (char)83, (char)70, (char)45, (char)87, (char)33, (char)194, (char)119, (char)84, (char)187, (char)63, (char)222, (char)119, (char)137, (char)73, (char)95, (char)194, (char)149, (char)149, (char)222, (char)196, (char)34, (char)72, (char)185, (char)96, (char)249, (char)168, (char)202, (char)42, (char)28, (char)109, (char)119, (char)93, (char)152, (char)2, (char)209, (char)189, (char)250, (char)70, (char)142, (char)22, (char)47, (char)220, (char)205, (char)42, (char)96, (char)64, (char)243, (char)246, (char)220, (char)20, (char)171, (char)5, (char)25, (char)37, (char)61, (char)12, (char)54, (char)13, (char)8, (char)199, (char)171, (char)241, (char)89, (char)167}, 0) ;
        CommunicationChannel.instance.send(p11002);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEVICE_OP_WRITE_REPLY.add((src, ph, pack) ->
        {
            assert(pack.request_id_GET() == 207104920L);
            assert(pack.result_GET() == (char)193);
        });
        GroundControl.DEVICE_OP_WRITE_REPLY p11003 = CommunicationChannel.new_DEVICE_OP_WRITE_REPLY();
        PH.setPack(p11003);
        p11003.result_SET((char)193) ;
        p11003.request_id_SET(207104920L) ;
        CommunicationChannel.instance.send(p11003);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ADAP_TUNING.add((src, ph, pack) ->
        {
            assert(pack.sigma_GET() == 1.5393043E38F);
            assert(pack.f_GET() == 4.835808E37F);
            assert(pack.sigma_dot_GET() == 1.2640435E38F);
            assert(pack.theta_GET() == -2.8990494E38F);
            assert(pack.omega_GET() == 1.2749977E38F);
            assert(pack.achieved_GET() == 1.2558101E38F);
            assert(pack.desired_GET() == -2.0566096E38F);
            assert(pack.axis_GET() == PID_TUNING_AXIS.PID_TUNING_ROLL);
            assert(pack.theta_dot_GET() == -2.6204827E38F);
            assert(pack.u_GET() == 6.420069E37F);
            assert(pack.error_GET() == -1.9577184E38F);
            assert(pack.omega_dot_GET() == -1.6722425E38F);
            assert(pack.f_dot_GET() == -1.7364016E38F);
        });
        GroundControl.ADAP_TUNING p11010 = CommunicationChannel.new_ADAP_TUNING();
        PH.setPack(p11010);
        p11010.axis_SET(PID_TUNING_AXIS.PID_TUNING_ROLL) ;
        p11010.u_SET(6.420069E37F) ;
        p11010.desired_SET(-2.0566096E38F) ;
        p11010.theta_SET(-2.8990494E38F) ;
        p11010.error_SET(-1.9577184E38F) ;
        p11010.sigma_SET(1.5393043E38F) ;
        p11010.omega_SET(1.2749977E38F) ;
        p11010.omega_dot_SET(-1.6722425E38F) ;
        p11010.f_dot_SET(-1.7364016E38F) ;
        p11010.achieved_SET(1.2558101E38F) ;
        p11010.f_SET(4.835808E37F) ;
        p11010.sigma_dot_SET(1.2640435E38F) ;
        p11010.theta_dot_SET(-2.6204827E38F) ;
        CommunicationChannel.instance.send(p11010);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VISION_POSITION_DELTA.add((src, ph, pack) ->
        {
            assert(pack.time_delta_usec_GET() == 8283148838991203889L);
            assert(pack.time_usec_GET() == 8225626828053590239L);
            assert(Arrays.equals(pack.position_delta_GET(),  new float[] {3.1558577E38F, 5.6066457E37F, -2.5040298E38F}));
            assert(pack.confidence_GET() == 2.0259136E38F);
            assert(Arrays.equals(pack.angle_delta_GET(),  new float[] {1.8200455E38F, -1.390831E38F, 1.8231929E38F}));
        });
        GroundControl.VISION_POSITION_DELTA p11011 = CommunicationChannel.new_VISION_POSITION_DELTA();
        PH.setPack(p11011);
        p11011.angle_delta_SET(new float[] {1.8200455E38F, -1.390831E38F, 1.8231929E38F}, 0) ;
        p11011.position_delta_SET(new float[] {3.1558577E38F, 5.6066457E37F, -2.5040298E38F}, 0) ;
        p11011.time_delta_usec_SET(8283148838991203889L) ;
        p11011.confidence_SET(2.0259136E38F) ;
        p11011.time_usec_SET(8225626828053590239L) ;
        CommunicationChannel.instance.send(p11011);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
    }

}
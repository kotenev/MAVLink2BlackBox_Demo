
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
            long id = id__A(src);
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
                case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED:
                    id = 0;
                    break;
                case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                    id = 1;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
                    id = 2;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
                    id = 3;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
                    id = 4;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                    id = 5;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND:
                    id = 6;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                    id = 7;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
                    id = 8;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
                    id = 9;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FOLLOW:
                    id = 10;
                    break;
                case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
                    id = 11;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
                    id = 12;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW:
                    id = 13;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
                    id = 14;
                    break;
                case MAV_CMD.MAV_CMD_NAV_ROI:
                    id = 15;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
                    id = 16;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
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
                case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                    id = 57;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                    id = 58;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                    id = 59;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                    id = 60;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                    id = 61;
                    break;
                case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                    id = 62;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAST:
                    id = 63;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                    id = 64;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                    id = 65;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                    id = 66;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                    id = 67;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                    id = 68;
                    break;
                case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                    id = 69;
                    break;
                case MAV_CMD.MAV_CMD_MISSION_START:
                    id = 70;
                    break;
                case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                    id = 71;
                    break;
                case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                    id = 72;
                    break;
                case MAV_CMD.MAV_CMD_START_RX_PAIR:
                    id = 73;
                    break;
                case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                    id = 74;
                    break;
                case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                    id = 75;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                    id = 76;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                    id = 77;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                    id = 78;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                    id = 79;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                    id = 80;
                    break;
                case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                    id = 81;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                    id = 82;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                    id = 83;
                    break;
                case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                    id = 84;
                    break;
                case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                    id = 85;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                    id = 86;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                    id = 87;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                    id = 88;
                    break;
                case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                    id = 89;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                    id = 90;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                    id = 91;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                    id = 92;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                    id = 93;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                    id = 94;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_START:
                    id = 95;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_STOP:
                    id = 96;
                    break;
                case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                    id = 97;
                    break;
                case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                    id = 98;
                    break;
                case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                    id = 99;
                    break;
                case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                    id = 100;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                    id = 101;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                    id = 102;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_GATE:
                    id = 103;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                    id = 104;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                    id = 105;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                    id = 106;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                    id = 107;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                    id = 108;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                    id = 109;
                    break;
                case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                    id = 110;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                    id = 111;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                    id = 112;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                    id = 113;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                    id = 114;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                    id = 115;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                    id = 116;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                    id = 117;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                    id = 118;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                    id = 119;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                    id = 120;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                    id = 121;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                    id = 122;
                    break;
                case MAV_CMD.MAV_CMD_USER_1:
                    id = 123;
                    break;
                case MAV_CMD.MAV_CMD_USER_2:
                    id = 124;
                    break;
                case MAV_CMD.MAV_CMD_USER_3:
                    id = 125;
                    break;
                case MAV_CMD.MAV_CMD_USER_4:
                    id = 126;
                    break;
                case MAV_CMD.MAV_CMD_USER_5:
                    id = 127;
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
                case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED:
                    id = 0;
                    break;
                case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                    id = 1;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
                    id = 2;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
                    id = 3;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
                    id = 4;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                    id = 5;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND:
                    id = 6;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                    id = 7;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
                    id = 8;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
                    id = 9;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FOLLOW:
                    id = 10;
                    break;
                case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
                    id = 11;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
                    id = 12;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW:
                    id = 13;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
                    id = 14;
                    break;
                case MAV_CMD.MAV_CMD_NAV_ROI:
                    id = 15;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
                    id = 16;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
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
                case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                    id = 57;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                    id = 58;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                    id = 59;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                    id = 60;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                    id = 61;
                    break;
                case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                    id = 62;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAST:
                    id = 63;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                    id = 64;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                    id = 65;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                    id = 66;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                    id = 67;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                    id = 68;
                    break;
                case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                    id = 69;
                    break;
                case MAV_CMD.MAV_CMD_MISSION_START:
                    id = 70;
                    break;
                case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                    id = 71;
                    break;
                case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                    id = 72;
                    break;
                case MAV_CMD.MAV_CMD_START_RX_PAIR:
                    id = 73;
                    break;
                case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                    id = 74;
                    break;
                case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                    id = 75;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                    id = 76;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                    id = 77;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                    id = 78;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                    id = 79;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                    id = 80;
                    break;
                case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                    id = 81;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                    id = 82;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                    id = 83;
                    break;
                case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                    id = 84;
                    break;
                case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                    id = 85;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                    id = 86;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                    id = 87;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                    id = 88;
                    break;
                case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                    id = 89;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                    id = 90;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                    id = 91;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                    id = 92;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                    id = 93;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                    id = 94;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_START:
                    id = 95;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_STOP:
                    id = 96;
                    break;
                case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                    id = 97;
                    break;
                case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                    id = 98;
                    break;
                case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                    id = 99;
                    break;
                case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                    id = 100;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                    id = 101;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                    id = 102;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_GATE:
                    id = 103;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                    id = 104;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                    id = 105;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                    id = 106;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                    id = 107;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                    id = 108;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                    id = 109;
                    break;
                case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                    id = 110;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                    id = 111;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                    id = 112;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                    id = 113;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                    id = 114;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                    id = 115;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                    id = 116;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                    id = 117;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                    id = 118;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                    id = 119;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                    id = 120;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                    id = 121;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                    id = 122;
                    break;
                case MAV_CMD.MAV_CMD_USER_1:
                    id = 123;
                    break;
                case MAV_CMD.MAV_CMD_USER_2:
                    id = 124;
                    break;
                case MAV_CMD.MAV_CMD_USER_3:
                    id = 125;
                    break;
                case MAV_CMD.MAV_CMD_USER_4:
                    id = 126;
                    break;
                case MAV_CMD.MAV_CMD_USER_5:
                    id = 127;
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
                case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED:
                    id = 0;
                    break;
                case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                    id = 1;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
                    id = 2;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
                    id = 3;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
                    id = 4;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                    id = 5;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND:
                    id = 6;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                    id = 7;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
                    id = 8;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
                    id = 9;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FOLLOW:
                    id = 10;
                    break;
                case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
                    id = 11;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
                    id = 12;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW:
                    id = 13;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
                    id = 14;
                    break;
                case MAV_CMD.MAV_CMD_NAV_ROI:
                    id = 15;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
                    id = 16;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
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
                case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                    id = 57;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                    id = 58;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                    id = 59;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                    id = 60;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                    id = 61;
                    break;
                case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                    id = 62;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAST:
                    id = 63;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                    id = 64;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                    id = 65;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                    id = 66;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                    id = 67;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                    id = 68;
                    break;
                case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                    id = 69;
                    break;
                case MAV_CMD.MAV_CMD_MISSION_START:
                    id = 70;
                    break;
                case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                    id = 71;
                    break;
                case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                    id = 72;
                    break;
                case MAV_CMD.MAV_CMD_START_RX_PAIR:
                    id = 73;
                    break;
                case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                    id = 74;
                    break;
                case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                    id = 75;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                    id = 76;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                    id = 77;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                    id = 78;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                    id = 79;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                    id = 80;
                    break;
                case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                    id = 81;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                    id = 82;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                    id = 83;
                    break;
                case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                    id = 84;
                    break;
                case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                    id = 85;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                    id = 86;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                    id = 87;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                    id = 88;
                    break;
                case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                    id = 89;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                    id = 90;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                    id = 91;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                    id = 92;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                    id = 93;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                    id = 94;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_START:
                    id = 95;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_STOP:
                    id = 96;
                    break;
                case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                    id = 97;
                    break;
                case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                    id = 98;
                    break;
                case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                    id = 99;
                    break;
                case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                    id = 100;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                    id = 101;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                    id = 102;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_GATE:
                    id = 103;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                    id = 104;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                    id = 105;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                    id = 106;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                    id = 107;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                    id = 108;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                    id = 109;
                    break;
                case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                    id = 110;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                    id = 111;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                    id = 112;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                    id = 113;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                    id = 114;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                    id = 115;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                    id = 116;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                    id = 117;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                    id = 118;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                    id = 119;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                    id = 120;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                    id = 121;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                    id = 122;
                    break;
                case MAV_CMD.MAV_CMD_USER_1:
                    id = 123;
                    break;
                case MAV_CMD.MAV_CMD_USER_2:
                    id = 124;
                    break;
                case MAV_CMD.MAV_CMD_USER_3:
                    id = 125;
                    break;
                case MAV_CMD.MAV_CMD_USER_4:
                    id = 126;
                    break;
                case MAV_CMD.MAV_CMD_USER_5:
                    id = 127;
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
                case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED:
                    id = 0;
                    break;
                case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                    id = 1;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
                    id = 2;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
                    id = 3;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
                    id = 4;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                    id = 5;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND:
                    id = 6;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                    id = 7;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
                    id = 8;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
                    id = 9;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FOLLOW:
                    id = 10;
                    break;
                case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
                    id = 11;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
                    id = 12;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW:
                    id = 13;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
                    id = 14;
                    break;
                case MAV_CMD.MAV_CMD_NAV_ROI:
                    id = 15;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
                    id = 16;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
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
                case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                    id = 57;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                    id = 58;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                    id = 59;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                    id = 60;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                    id = 61;
                    break;
                case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                    id = 62;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAST:
                    id = 63;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                    id = 64;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                    id = 65;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                    id = 66;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                    id = 67;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                    id = 68;
                    break;
                case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                    id = 69;
                    break;
                case MAV_CMD.MAV_CMD_MISSION_START:
                    id = 70;
                    break;
                case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                    id = 71;
                    break;
                case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                    id = 72;
                    break;
                case MAV_CMD.MAV_CMD_START_RX_PAIR:
                    id = 73;
                    break;
                case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                    id = 74;
                    break;
                case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                    id = 75;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                    id = 76;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                    id = 77;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                    id = 78;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                    id = 79;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                    id = 80;
                    break;
                case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                    id = 81;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                    id = 82;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                    id = 83;
                    break;
                case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                    id = 84;
                    break;
                case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                    id = 85;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                    id = 86;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                    id = 87;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                    id = 88;
                    break;
                case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                    id = 89;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                    id = 90;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                    id = 91;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                    id = 92;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                    id = 93;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                    id = 94;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_START:
                    id = 95;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_STOP:
                    id = 96;
                    break;
                case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                    id = 97;
                    break;
                case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                    id = 98;
                    break;
                case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                    id = 99;
                    break;
                case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                    id = 100;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                    id = 101;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                    id = 102;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_GATE:
                    id = 103;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                    id = 104;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                    id = 105;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                    id = 106;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                    id = 107;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                    id = 108;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                    id = 109;
                    break;
                case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                    id = 110;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                    id = 111;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                    id = 112;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                    id = 113;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                    id = 114;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                    id = 115;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                    id = 116;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                    id = 117;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                    id = 118;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                    id = 119;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                    id = 120;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                    id = 121;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                    id = 122;
                    break;
                case MAV_CMD.MAV_CMD_USER_1:
                    id = 123;
                    break;
                case MAV_CMD.MAV_CMD_USER_2:
                    id = 124;
                    break;
                case MAV_CMD.MAV_CMD_USER_3:
                    id = 125;
                    break;
                case MAV_CMD.MAV_CMD_USER_4:
                    id = 126;
                    break;
                case MAV_CMD.MAV_CMD_USER_5:
                    id = 127;
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
                case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED:
                    id = 0;
                    break;
                case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                    id = 1;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
                    id = 2;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
                    id = 3;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
                    id = 4;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                    id = 5;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND:
                    id = 6;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                    id = 7;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
                    id = 8;
                    break;
                case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
                    id = 9;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FOLLOW:
                    id = 10;
                    break;
                case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
                    id = 11;
                    break;
                case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
                    id = 12;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW:
                    id = 13;
                    break;
                case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
                    id = 14;
                    break;
                case MAV_CMD.MAV_CMD_NAV_ROI:
                    id = 15;
                    break;
                case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
                    id = 16;
                    break;
                case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
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
                case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                    id = 57;
                    break;
                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                    id = 58;
                    break;
                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                    id = 59;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                    id = 60;
                    break;
                case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                    id = 61;
                    break;
                case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                    id = 62;
                    break;
                case MAV_CMD.MAV_CMD_DO_LAST:
                    id = 63;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                    id = 64;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                    id = 65;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                    id = 66;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                    id = 67;
                    break;
                case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                    id = 68;
                    break;
                case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                    id = 69;
                    break;
                case MAV_CMD.MAV_CMD_MISSION_START:
                    id = 70;
                    break;
                case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                    id = 71;
                    break;
                case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                    id = 72;
                    break;
                case MAV_CMD.MAV_CMD_START_RX_PAIR:
                    id = 73;
                    break;
                case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                    id = 74;
                    break;
                case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                    id = 75;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                    id = 76;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                    id = 77;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                    id = 78;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                    id = 79;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                    id = 80;
                    break;
                case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                    id = 81;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                    id = 82;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                    id = 83;
                    break;
                case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                    id = 84;
                    break;
                case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                    id = 85;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                    id = 86;
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                    id = 87;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                    id = 88;
                    break;
                case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                    id = 89;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                    id = 90;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                    id = 91;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                    id = 92;
                    break;
                case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                    id = 93;
                    break;
                case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                    id = 94;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_START:
                    id = 95;
                    break;
                case MAV_CMD.MAV_CMD_LOGGING_STOP:
                    id = 96;
                    break;
                case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                    id = 97;
                    break;
                case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                    id = 98;
                    break;
                case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                    id = 99;
                    break;
                case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                    id = 100;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                    id = 101;
                    break;
                case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                    id = 102;
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_GATE:
                    id = 103;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                    id = 104;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                    id = 105;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                    id = 106;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                    id = 107;
                    break;
                case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                    id = 108;
                    break;
                case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                    id = 109;
                    break;
                case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                    id = 110;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                    id = 111;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                    id = 112;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                    id = 113;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                    id = 114;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                    id = 115;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                    id = 116;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                    id = 117;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                    id = 118;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                    id = 119;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                    id = 120;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                    id = 121;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                    id = 122;
                    break;
                case MAV_CMD.MAV_CMD_USER_1:
                    id = 123;
                    break;
                case MAV_CMD.MAV_CMD_USER_2:
                    id = 124;
                    break;
                case MAV_CMD.MAV_CMD_USER_3:
                    id = 125;
                    break;
                case MAV_CMD.MAV_CMD_USER_4:
                    id = 126;
                    break;
                case MAV_CMD.MAV_CMD_USER_5:
                    id = 127;
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
    public static class FLEXIFUNCTION_SET extends GroundControl.FLEXIFUNCTION_SET
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
    }
    public static class FLEXIFUNCTION_READ_REQ extends GroundControl.FLEXIFUNCTION_READ_REQ
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public short read_req_type_GET()//Type of flexifunction data requested
        {  return (short)((short) get_bytes(data,  2, 2)); }
        public short data_index_GET()//index into data where needed
        {  return (short)((short) get_bytes(data,  4, 2)); }
    }
    public static class FLEXIFUNCTION_BUFFER_FUNCTION extends GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION
    {
        public char func_index_GET()//Function index
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char func_count_GET()//Total count of functions
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public char data_address_GET()//Address in the flexifunction data, Set to 0xFFFF to use address in target memory
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public char data_size_GET()//Size of the
        {  return (char)((char) get_bytes(data,  6, 2)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  9, 1)); }
        public byte[] data__GET(byte[]  dst_ch, int pos)  //Settings data
        {
            for(int BYTE = 10, dst_max = pos + 48; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (byte)((byte) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public byte[] data__GET()//Settings data
        {return data__GET(new byte[48], 0);}
    }
    public static class FLEXIFUNCTION_BUFFER_FUNCTION_ACK extends GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION_ACK
    {
        public char func_index_GET()//Function index
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char result_GET()//result of acknowledge, 0=fail, 1=good
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  5, 1)); }
    }
    public static class FLEXIFUNCTION_DIRECTORY extends GroundControl.FLEXIFUNCTION_DIRECTORY
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char directory_type_GET()//0=inputs, 1=outputs
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char start_index_GET()//index of first directory entry to write
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public char count_GET()//count of directory entries to write
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public byte[] directory_data_GET(byte[]  dst_ch, int pos)  //Settings data
        {
            for(int BYTE = 5, dst_max = pos + 48; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (byte)((byte) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public byte[] directory_data_GET()//Settings data
        {return directory_data_GET(new byte[48], 0);}
    }
    public static class FLEXIFUNCTION_DIRECTORY_ACK extends GroundControl.FLEXIFUNCTION_DIRECTORY_ACK
    {
        public char result_GET()//result of acknowledge, 0=fail, 1=good
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public char directory_type_GET()//0=inputs, 1=outputs
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public char start_index_GET()//index of first directory entry to write
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public char count_GET()//count of directory entries to write
        {  return (char)((char) get_bytes(data,  6, 1)); }
    }
    public static class FLEXIFUNCTION_COMMAND extends GroundControl.FLEXIFUNCTION_COMMAND
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char command_type_GET()//Flexifunction command type
        {  return (char)((char) get_bytes(data,  2, 1)); }
    }
    public static class FLEXIFUNCTION_COMMAND_ACK extends GroundControl.FLEXIFUNCTION_COMMAND_ACK
    {
        public char command_type_GET()//Command acknowledged
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char result_GET()//result of acknowledge
        {  return (char)((char) get_bytes(data,  2, 2)); }
    }
    public static class SERIAL_UDB_EXTRA_F2_A extends GroundControl.SERIAL_UDB_EXTRA_F2_A
    {
        public char sue_waypoint_index_GET()//Serial UDB Extra Waypoint Index
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char sue_cog_GET()//Serial UDB Extra GPS Course Over Ground
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public char sue_cpu_load_GET()//Serial UDB Extra CPU Load
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public char sue_air_speed_3DIMU_GET()//Serial UDB Extra 3D IMU Air Speed
        {  return (char)((char) get_bytes(data,  6, 2)); }
        public long sue_time_GET()//Serial UDB Extra Time
        {  return (get_bytes(data,  8, 4)); }
        public char sue_status_GET()//Serial UDB Extra Status
        {  return (char)((char) get_bytes(data,  12, 1)); }
        public int sue_latitude_GET()//Serial UDB Extra Latitude
        {  return (int)((int) get_bytes(data,  13, 4)); }
        public int sue_longitude_GET()//Serial UDB Extra Longitude
        {  return (int)((int) get_bytes(data,  17, 4)); }
        public int sue_altitude_GET()//Serial UDB Extra Altitude
        {  return (int)((int) get_bytes(data,  21, 4)); }
        public short sue_rmat0_GET()//Serial UDB Extra Rmat 0
        {  return (short)((short) get_bytes(data,  25, 2)); }
        public short sue_rmat1_GET()//Serial UDB Extra Rmat 1
        {  return (short)((short) get_bytes(data,  27, 2)); }
        public short sue_rmat2_GET()//Serial UDB Extra Rmat 2
        {  return (short)((short) get_bytes(data,  29, 2)); }
        public short sue_rmat3_GET()//Serial UDB Extra Rmat 3
        {  return (short)((short) get_bytes(data,  31, 2)); }
        public short sue_rmat4_GET()//Serial UDB Extra Rmat 4
        {  return (short)((short) get_bytes(data,  33, 2)); }
        public short sue_rmat5_GET()//Serial UDB Extra Rmat 5
        {  return (short)((short) get_bytes(data,  35, 2)); }
        public short sue_rmat6_GET()//Serial UDB Extra Rmat 6
        {  return (short)((short) get_bytes(data,  37, 2)); }
        public short sue_rmat7_GET()//Serial UDB Extra Rmat 7
        {  return (short)((short) get_bytes(data,  39, 2)); }
        public short sue_rmat8_GET()//Serial UDB Extra Rmat 8
        {  return (short)((short) get_bytes(data,  41, 2)); }
        public short sue_sog_GET()//Serial UDB Extra Speed Over Ground
        {  return (short)((short) get_bytes(data,  43, 2)); }
        public short sue_estimated_wind_0_GET()//Serial UDB Extra Estimated Wind 0
        {  return (short)((short) get_bytes(data,  45, 2)); }
        public short sue_estimated_wind_1_GET()//Serial UDB Extra Estimated Wind 1
        {  return (short)((short) get_bytes(data,  47, 2)); }
        public short sue_estimated_wind_2_GET()//Serial UDB Extra Estimated Wind 2
        {  return (short)((short) get_bytes(data,  49, 2)); }
        public short sue_magFieldEarth0_GET()//Serial UDB Extra Magnetic Field Earth 0
        {  return (short)((short) get_bytes(data,  51, 2)); }
        public short sue_magFieldEarth1_GET()//Serial UDB Extra Magnetic Field Earth 1
        {  return (short)((short) get_bytes(data,  53, 2)); }
        public short sue_magFieldEarth2_GET()//Serial UDB Extra Magnetic Field Earth 2
        {  return (short)((short) get_bytes(data,  55, 2)); }
        public short sue_svs_GET()//Serial UDB Extra Number of Sattelites in View
        {  return (short)((short) get_bytes(data,  57, 2)); }
        public short sue_hdop_GET()//Serial UDB Extra GPS Horizontal Dilution of Precision
        {  return (short)((short) get_bytes(data,  59, 2)); }
    }
    public static class SERIAL_UDB_EXTRA_F2_B extends GroundControl.SERIAL_UDB_EXTRA_F2_B
    {
        public long sue_time_GET()//Serial UDB Extra Time
        {  return (get_bytes(data,  0, 4)); }
        public long sue_flags_GET()//Serial UDB Extra Status Flags
        {  return (get_bytes(data,  4, 4)); }
        public short sue_pwm_input_1_GET()//Serial UDB Extra PWM Input Channel 1
        {  return (short)((short) get_bytes(data,  8, 2)); }
        public short sue_pwm_input_2_GET()//Serial UDB Extra PWM Input Channel 2
        {  return (short)((short) get_bytes(data,  10, 2)); }
        public short sue_pwm_input_3_GET()//Serial UDB Extra PWM Input Channel 3
        {  return (short)((short) get_bytes(data,  12, 2)); }
        public short sue_pwm_input_4_GET()//Serial UDB Extra PWM Input Channel 4
        {  return (short)((short) get_bytes(data,  14, 2)); }
        public short sue_pwm_input_5_GET()//Serial UDB Extra PWM Input Channel 5
        {  return (short)((short) get_bytes(data,  16, 2)); }
        public short sue_pwm_input_6_GET()//Serial UDB Extra PWM Input Channel 6
        {  return (short)((short) get_bytes(data,  18, 2)); }
        public short sue_pwm_input_7_GET()//Serial UDB Extra PWM Input Channel 7
        {  return (short)((short) get_bytes(data,  20, 2)); }
        public short sue_pwm_input_8_GET()//Serial UDB Extra PWM Input Channel 8
        {  return (short)((short) get_bytes(data,  22, 2)); }
        public short sue_pwm_input_9_GET()//Serial UDB Extra PWM Input Channel 9
        {  return (short)((short) get_bytes(data,  24, 2)); }
        public short sue_pwm_input_10_GET()//Serial UDB Extra PWM Input Channel 10
        {  return (short)((short) get_bytes(data,  26, 2)); }
        public short sue_pwm_input_11_GET()//Serial UDB Extra PWM Input Channel 11
        {  return (short)((short) get_bytes(data,  28, 2)); }
        public short sue_pwm_input_12_GET()//Serial UDB Extra PWM Input Channel 12
        {  return (short)((short) get_bytes(data,  30, 2)); }
        public short sue_pwm_output_1_GET()//Serial UDB Extra PWM Output Channel 1
        {  return (short)((short) get_bytes(data,  32, 2)); }
        public short sue_pwm_output_2_GET()//Serial UDB Extra PWM Output Channel 2
        {  return (short)((short) get_bytes(data,  34, 2)); }
        public short sue_pwm_output_3_GET()//Serial UDB Extra PWM Output Channel 3
        {  return (short)((short) get_bytes(data,  36, 2)); }
        public short sue_pwm_output_4_GET()//Serial UDB Extra PWM Output Channel 4
        {  return (short)((short) get_bytes(data,  38, 2)); }
        public short sue_pwm_output_5_GET()//Serial UDB Extra PWM Output Channel 5
        {  return (short)((short) get_bytes(data,  40, 2)); }
        public short sue_pwm_output_6_GET()//Serial UDB Extra PWM Output Channel 6
        {  return (short)((short) get_bytes(data,  42, 2)); }
        public short sue_pwm_output_7_GET()//Serial UDB Extra PWM Output Channel 7
        {  return (short)((short) get_bytes(data,  44, 2)); }
        public short sue_pwm_output_8_GET()//Serial UDB Extra PWM Output Channel 8
        {  return (short)((short) get_bytes(data,  46, 2)); }
        public short sue_pwm_output_9_GET()//Serial UDB Extra PWM Output Channel 9
        {  return (short)((short) get_bytes(data,  48, 2)); }
        public short sue_pwm_output_10_GET()//Serial UDB Extra PWM Output Channel 10
        {  return (short)((short) get_bytes(data,  50, 2)); }
        public short sue_pwm_output_11_GET()//Serial UDB Extra PWM Output Channel 11
        {  return (short)((short) get_bytes(data,  52, 2)); }
        public short sue_pwm_output_12_GET()//Serial UDB Extra PWM Output Channel 12
        {  return (short)((short) get_bytes(data,  54, 2)); }
        public short sue_imu_location_x_GET()//Serial UDB Extra IMU Location X
        {  return (short)((short) get_bytes(data,  56, 2)); }
        public short sue_imu_location_y_GET()//Serial UDB Extra IMU Location Y
        {  return (short)((short) get_bytes(data,  58, 2)); }
        public short sue_imu_location_z_GET()//Serial UDB Extra IMU Location Z
        {  return (short)((short) get_bytes(data,  60, 2)); }
        public short sue_location_error_earth_x_GET()//Serial UDB Location Error Earth X
        {  return (short)((short) get_bytes(data,  62, 2)); }
        public short sue_location_error_earth_y_GET()//Serial UDB Location Error Earth Y
        {  return (short)((short) get_bytes(data,  64, 2)); }
        public short sue_location_error_earth_z_GET()//Serial UDB Location Error Earth Z
        {  return (short)((short) get_bytes(data,  66, 2)); }
        public short sue_osc_fails_GET()//Serial UDB Extra Oscillator Failure Count
        {  return (short)((short) get_bytes(data,  68, 2)); }
        public short sue_imu_velocity_x_GET()//Serial UDB Extra IMU Velocity X
        {  return (short)((short) get_bytes(data,  70, 2)); }
        public short sue_imu_velocity_y_GET()//Serial UDB Extra IMU Velocity Y
        {  return (short)((short) get_bytes(data,  72, 2)); }
        public short sue_imu_velocity_z_GET()//Serial UDB Extra IMU Velocity Z
        {  return (short)((short) get_bytes(data,  74, 2)); }
        public short sue_waypoint_goal_x_GET()//Serial UDB Extra Current Waypoint Goal X
        {  return (short)((short) get_bytes(data,  76, 2)); }
        public short sue_waypoint_goal_y_GET()//Serial UDB Extra Current Waypoint Goal Y
        {  return (short)((short) get_bytes(data,  78, 2)); }
        public short sue_waypoint_goal_z_GET()//Serial UDB Extra Current Waypoint Goal Z
        {  return (short)((short) get_bytes(data,  80, 2)); }
        public short sue_aero_x_GET()//Aeroforce in UDB X Axis
        {  return (short)((short) get_bytes(data,  82, 2)); }
        public short sue_aero_y_GET()//Aeroforce in UDB Y Axis
        {  return (short)((short) get_bytes(data,  84, 2)); }
        public short sue_aero_z_GET()//Aeroforce in UDB Z axis
        {  return (short)((short) get_bytes(data,  86, 2)); }
        public short sue_barom_temp_GET()//SUE barometer temperature
        {  return (short)((short) get_bytes(data,  88, 2)); }
        public int sue_barom_press_GET()//SUE barometer pressure
        {  return (int)((int) get_bytes(data,  90, 4)); }
        public int sue_barom_alt_GET()//SUE barometer altitude
        {  return (int)((int) get_bytes(data,  94, 4)); }
        public short sue_bat_volt_GET()//SUE battery voltage
        {  return (short)((short) get_bytes(data,  98, 2)); }
        public short sue_bat_amp_GET()//SUE battery current
        {  return (short)((short) get_bytes(data,  100, 2)); }
        public short sue_bat_amp_hours_GET()//SUE battery milli amp hours used
        {  return (short)((short) get_bytes(data,  102, 2)); }
        public short sue_desired_height_GET()//Sue autopilot desired height
        {  return (short)((short) get_bytes(data,  104, 2)); }
        public short sue_memory_stack_free_GET()//Serial UDB Extra Stack Memory Free
        {  return (short)((short) get_bytes(data,  106, 2)); }
    }
    public static class SERIAL_UDB_EXTRA_F4 extends GroundControl.SERIAL_UDB_EXTRA_F4
    {
        public char sue_ROLL_STABILIZATION_AILERONS_GET()//Serial UDB Extra Roll Stabilization with Ailerons Enabled
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char sue_ROLL_STABILIZATION_RUDDER_GET()//Serial UDB Extra Roll Stabilization with Rudder Enabled
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char sue_PITCH_STABILIZATION_GET()//Serial UDB Extra Pitch Stabilization Enabled
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char sue_YAW_STABILIZATION_RUDDER_GET()//Serial UDB Extra Yaw Stabilization using Rudder Enabled
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public char sue_YAW_STABILIZATION_AILERON_GET()//Serial UDB Extra Yaw Stabilization using Ailerons Enabled
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public char sue_AILERON_NAVIGATION_GET()//Serial UDB Extra Navigation with Ailerons Enabled
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public char sue_RUDDER_NAVIGATION_GET()//Serial UDB Extra Navigation with Rudder Enabled
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public char sue_ALTITUDEHOLD_STABILIZED_GET()//Serial UDB Extra Type of Alitude Hold when in Stabilized Mode
        {  return (char)((char) get_bytes(data,  7, 1)); }
        public char sue_ALTITUDEHOLD_WAYPOINT_GET()//Serial UDB Extra Type of Alitude Hold when in Waypoint Mode
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public char sue_RACING_MODE_GET()//Serial UDB Extra Firmware racing mode enabled
        {  return (char)((char) get_bytes(data,  9, 1)); }
    }
    public static class SERIAL_UDB_EXTRA_F5 extends GroundControl.SERIAL_UDB_EXTRA_F5
    {
        public float sue_YAWKP_AILERON_GET()//Serial UDB YAWKP_AILERON Gain for Proporional control of navigation
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float sue_YAWKD_AILERON_GET()//Serial UDB YAWKD_AILERON Gain for Rate control of navigation
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float sue_ROLLKP_GET()//Serial UDB Extra ROLLKP Gain for Proportional control of roll stabilization
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float sue_ROLLKD_GET()//Serial UDB Extra ROLLKD Gain for Rate control of roll stabilization
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
    }
    public static class SERIAL_UDB_EXTRA_F6 extends GroundControl.SERIAL_UDB_EXTRA_F6
    {
        public float sue_PITCHGAIN_GET()//Serial UDB Extra PITCHGAIN Proportional Control
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float sue_PITCHKD_GET()//Serial UDB Extra Pitch Rate Control
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float sue_RUDDER_ELEV_MIX_GET()//Serial UDB Extra Rudder to Elevator Mix
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float sue_ROLL_ELEV_MIX_GET()//Serial UDB Extra Roll to Elevator Mix
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float sue_ELEVATOR_BOOST_GET()//Gain For Boosting Manual Elevator control When Plane Stabilized
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
    }
    public static class SERIAL_UDB_EXTRA_F7 extends GroundControl.SERIAL_UDB_EXTRA_F7
    {
        public float sue_YAWKP_RUDDER_GET()//Serial UDB YAWKP_RUDDER Gain for Proporional control of navigation
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float sue_YAWKD_RUDDER_GET()//Serial UDB YAWKD_RUDDER Gain for Rate control of navigation
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float sue_ROLLKP_RUDDER_GET()//Serial UDB Extra ROLLKP_RUDDER Gain for Proportional control of roll stabilization
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float sue_ROLLKD_RUDDER_GET()//Serial UDB Extra ROLLKD_RUDDER Gain for Rate control of roll stabilization
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float sue_RUDDER_BOOST_GET()//SERIAL UDB EXTRA Rudder Boost Gain to Manual Control when stabilized
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float sue_RTL_PITCH_DOWN_GET()//Serial UDB Extra Return To Landing - Angle to Pitch Plane Down
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
    }
    public static class SERIAL_UDB_EXTRA_F8 extends GroundControl.SERIAL_UDB_EXTRA_F8
    {
        public float sue_HEIGHT_TARGET_MAX_GET()//Serial UDB Extra HEIGHT_TARGET_MAX
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float sue_HEIGHT_TARGET_MIN_GET()//Serial UDB Extra HEIGHT_TARGET_MIN
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float sue_ALT_HOLD_THROTTLE_MIN_GET()//Serial UDB Extra ALT_HOLD_THROTTLE_MIN
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float sue_ALT_HOLD_THROTTLE_MAX_GET()//Serial UDB Extra ALT_HOLD_THROTTLE_MAX
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float sue_ALT_HOLD_PITCH_MIN_GET()//Serial UDB Extra ALT_HOLD_PITCH_MIN
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float sue_ALT_HOLD_PITCH_MAX_GET()//Serial UDB Extra ALT_HOLD_PITCH_MAX
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float sue_ALT_HOLD_PITCH_HIGH_GET()//Serial UDB Extra ALT_HOLD_PITCH_HIGH
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
    }
    public static class SERIAL_UDB_EXTRA_F13 extends GroundControl.SERIAL_UDB_EXTRA_F13
    {
        public short sue_week_no_GET()//Serial UDB Extra GPS Week Number
        {  return (short)((short) get_bytes(data,  0, 2)); }
        public int sue_lat_origin_GET()//Serial UDB Extra MP Origin Latitude
        {  return (int)((int) get_bytes(data,  2, 4)); }
        public int sue_lon_origin_GET()//Serial UDB Extra MP Origin Longitude
        {  return (int)((int) get_bytes(data,  6, 4)); }
        public int sue_alt_origin_GET()//Serial UDB Extra MP Origin Altitude Above Sea Level
        {  return (int)((int) get_bytes(data,  10, 4)); }
    }
    public static class SERIAL_UDB_EXTRA_F14 extends GroundControl.SERIAL_UDB_EXTRA_F14
    {
        public long sue_TRAP_SOURCE_GET()//Serial UDB Extra Type Program Address of Last Trap
        {  return (get_bytes(data,  0, 4)); }
        public char sue_WIND_ESTIMATION_GET()//Serial UDB Extra Wind Estimation Enabled
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public char sue_GPS_TYPE_GET()//Serial UDB Extra Type of GPS Unit
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public char sue_DR_GET()//Serial UDB Extra Dead Reckoning Enabled
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public char sue_BOARD_TYPE_GET()//Serial UDB Extra Type of UDB Hardware
        {  return (char)((char) get_bytes(data,  7, 1)); }
        public char sue_AIRFRAME_GET()//Serial UDB Extra Type of Airframe
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public short sue_RCON_GET()//Serial UDB Extra Reboot Register of DSPIC
        {  return (short)((short) get_bytes(data,  9, 2)); }
        public short sue_TRAP_FLAGS_GET()//Serial UDB Extra  Last dspic Trap Flags
        {  return (short)((short) get_bytes(data,  11, 2)); }
        public short sue_osc_fail_count_GET()//Serial UDB Extra Number of Ocillator Failures
        {  return (short)((short) get_bytes(data,  13, 2)); }
        public char sue_CLOCK_CONFIG_GET()//Serial UDB Extra UDB Internal Clock Configuration
        {  return (char)((char) get_bytes(data,  15, 1)); }
        public char sue_FLIGHT_PLAN_TYPE_GET()//Serial UDB Extra Type of Flight Plan
        {  return (char)((char) get_bytes(data,  16, 1)); }
    }
    public static class SERIAL_UDB_EXTRA_F15 extends GroundControl.SERIAL_UDB_EXTRA_F15
    {
        public char[] sue_ID_VEHICLE_MODEL_NAME_GET(char[]  dst_ch, int pos)  //Serial UDB Extra Model Name Of Vehicle
        {
            for(int BYTE = 0, dst_max = pos + 40; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] sue_ID_VEHICLE_MODEL_NAME_GET()//Serial UDB Extra Model Name Of Vehicle
        {return sue_ID_VEHICLE_MODEL_NAME_GET(new char[40], 0);} public char[] sue_ID_VEHICLE_REGISTRATION_GET(char[]  dst_ch, int pos)  //Serial UDB Extra Registraton Number of Vehicle
        {
            for(int BYTE = 40, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] sue_ID_VEHICLE_REGISTRATION_GET()//Serial UDB Extra Registraton Number of Vehicle
        {return sue_ID_VEHICLE_REGISTRATION_GET(new char[20], 0);}
    }
    public static class SERIAL_UDB_EXTRA_F16 extends GroundControl.SERIAL_UDB_EXTRA_F16
    {
        public char[] sue_ID_LEAD_PILOT_GET(char[]  dst_ch, int pos)  //Serial UDB Extra Name of Expected Lead Pilot
        {
            for(int BYTE = 0, dst_max = pos + 40; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] sue_ID_LEAD_PILOT_GET()//Serial UDB Extra Name of Expected Lead Pilot
        {return sue_ID_LEAD_PILOT_GET(new char[40], 0);} public char[] sue_ID_DIY_DRONES_URL_GET(char[]  dst_ch, int pos)  //Serial UDB Extra URL of Lead Pilot or Team
        {
            for(int BYTE = 40, dst_max = pos + 70; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] sue_ID_DIY_DRONES_URL_GET()//Serial UDB Extra URL of Lead Pilot or Team
        {return sue_ID_DIY_DRONES_URL_GET(new char[70], 0);}
    }
    public static class ALTITUDES extends GroundControl.ALTITUDES
    {
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public int alt_gps_GET()//GPS altitude in meters, expressed as * 1000 (millimeters), above MSL
        {  return (int)((int) get_bytes(data,  4, 4)); }
        public int alt_imu_GET()//IMU altitude above ground in meters, expressed as * 1000 (millimeters)
        {  return (int)((int) get_bytes(data,  8, 4)); }
        public int alt_barometric_GET()//barometeric altitude above ground in meters, expressed as * 1000 (millimeters)
        {  return (int)((int) get_bytes(data,  12, 4)); }
        public int alt_optical_flow_GET()//Optical flow altitude above ground in meters, expressed as * 1000 (millimeters)
        {  return (int)((int) get_bytes(data,  16, 4)); }
        public int alt_range_finder_GET()//Rangefinder Altitude above ground in meters, expressed as * 1000 (millimeters)
        {  return (int)((int) get_bytes(data,  20, 4)); }
        public int alt_extra_GET()//Extra altitude above ground in meters, expressed as * 1000 (millimeters)
        {  return (int)((int) get_bytes(data,  24, 4)); }
    }
    public static class AIRSPEEDS extends GroundControl.AIRSPEEDS
    {
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public short airspeed_imu_GET()//Airspeed estimate from IMU, cm/s
        {  return (short)((short) get_bytes(data,  4, 2)); }
        public short airspeed_pitot_GET()//Pitot measured forward airpseed, cm/s
        {  return (short)((short) get_bytes(data,  6, 2)); }
        public short airspeed_hot_wire_GET()//Hot wire anenometer measured airspeed, cm/s
        {  return (short)((short) get_bytes(data,  8, 2)); }
        public short airspeed_ultrasonic_GET()//Ultrasonic measured airspeed, cm/s
        {  return (short)((short) get_bytes(data,  10, 2)); }
        public short aoa_GET()//Angle of attack sensor, degrees * 10
        {  return (short)((short) get_bytes(data,  12, 2)); }
        public short aoy_GET()//Yaw angle sensor, degrees * 10
        {  return (short)((short) get_bytes(data,  14, 2)); }
    }
    public static class SERIAL_UDB_EXTRA_F17 extends GroundControl.SERIAL_UDB_EXTRA_F17
    {
        public float sue_feed_forward_GET()//SUE Feed Forward Gain
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float sue_turn_rate_nav_GET()//SUE Max Turn Rate when Navigating
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float sue_turn_rate_fbw_GET()//SUE Max Turn Rate in Fly By Wire Mode
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
    }
    public static class SERIAL_UDB_EXTRA_F18 extends GroundControl.SERIAL_UDB_EXTRA_F18
    {
        public float angle_of_attack_normal_GET()//SUE Angle of Attack Normal
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float angle_of_attack_inverted_GET()//SUE Angle of Attack Inverted
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float elevator_trim_normal_GET()//SUE Elevator Trim Normal
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float elevator_trim_inverted_GET()//SUE Elevator Trim Inverted
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float reference_speed_GET()//SUE reference_speed
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
    }
    public static class SERIAL_UDB_EXTRA_F19 extends GroundControl.SERIAL_UDB_EXTRA_F19
    {
        public char sue_aileron_output_channel_GET()//SUE aileron output channel
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char sue_aileron_reversed_GET()//SUE aileron reversed
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char sue_elevator_output_channel_GET()//SUE elevator output channel
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char sue_elevator_reversed_GET()//SUE elevator reversed
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public char sue_throttle_output_channel_GET()//SUE throttle output channel
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public char sue_throttle_reversed_GET()//SUE throttle reversed
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public char sue_rudder_output_channel_GET()//SUE rudder output channel
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public char sue_rudder_reversed_GET()//SUE rudder reversed
        {  return (char)((char) get_bytes(data,  7, 1)); }
    }
    public static class SERIAL_UDB_EXTRA_F20 extends GroundControl.SERIAL_UDB_EXTRA_F20
    {
        public char sue_number_of_inputs_GET()//SUE Number of Input Channels
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public short sue_trim_value_input_1_GET()//SUE UDB PWM Trim Value on Input 1
        {  return (short)((short) get_bytes(data,  1, 2)); }
        public short sue_trim_value_input_2_GET()//SUE UDB PWM Trim Value on Input 2
        {  return (short)((short) get_bytes(data,  3, 2)); }
        public short sue_trim_value_input_3_GET()//SUE UDB PWM Trim Value on Input 3
        {  return (short)((short) get_bytes(data,  5, 2)); }
        public short sue_trim_value_input_4_GET()//SUE UDB PWM Trim Value on Input 4
        {  return (short)((short) get_bytes(data,  7, 2)); }
        public short sue_trim_value_input_5_GET()//SUE UDB PWM Trim Value on Input 5
        {  return (short)((short) get_bytes(data,  9, 2)); }
        public short sue_trim_value_input_6_GET()//SUE UDB PWM Trim Value on Input 6
        {  return (short)((short) get_bytes(data,  11, 2)); }
        public short sue_trim_value_input_7_GET()//SUE UDB PWM Trim Value on Input 7
        {  return (short)((short) get_bytes(data,  13, 2)); }
        public short sue_trim_value_input_8_GET()//SUE UDB PWM Trim Value on Input 8
        {  return (short)((short) get_bytes(data,  15, 2)); }
        public short sue_trim_value_input_9_GET()//SUE UDB PWM Trim Value on Input 9
        {  return (short)((short) get_bytes(data,  17, 2)); }
        public short sue_trim_value_input_10_GET()//SUE UDB PWM Trim Value on Input 10
        {  return (short)((short) get_bytes(data,  19, 2)); }
        public short sue_trim_value_input_11_GET()//SUE UDB PWM Trim Value on Input 11
        {  return (short)((short) get_bytes(data,  21, 2)); }
        public short sue_trim_value_input_12_GET()//SUE UDB PWM Trim Value on Input 12
        {  return (short)((short) get_bytes(data,  23, 2)); }
    }
    public static class SERIAL_UDB_EXTRA_F21 extends GroundControl.SERIAL_UDB_EXTRA_F21
    {
        public short sue_accel_x_offset_GET()//SUE X accelerometer offset
        {  return (short)((short) get_bytes(data,  0, 2)); }
        public short sue_accel_y_offset_GET()//SUE Y accelerometer offset
        {  return (short)((short) get_bytes(data,  2, 2)); }
        public short sue_accel_z_offset_GET()//SUE Z accelerometer offset
        {  return (short)((short) get_bytes(data,  4, 2)); }
        public short sue_gyro_x_offset_GET()//SUE X gyro offset
        {  return (short)((short) get_bytes(data,  6, 2)); }
        public short sue_gyro_y_offset_GET()//SUE Y gyro offset
        {  return (short)((short) get_bytes(data,  8, 2)); }
        public short sue_gyro_z_offset_GET()//SUE Z gyro offset
        {  return (short)((short) get_bytes(data,  10, 2)); }
    }
    public static class SERIAL_UDB_EXTRA_F22 extends GroundControl.SERIAL_UDB_EXTRA_F22
    {
        public short sue_accel_x_at_calibration_GET()//SUE X accelerometer at calibration time
        {  return (short)((short) get_bytes(data,  0, 2)); }
        public short sue_accel_y_at_calibration_GET()//SUE Y accelerometer at calibration time
        {  return (short)((short) get_bytes(data,  2, 2)); }
        public short sue_accel_z_at_calibration_GET()//SUE Z accelerometer at calibration time
        {  return (short)((short) get_bytes(data,  4, 2)); }
        public short sue_gyro_x_at_calibration_GET()//SUE X gyro at calibration time
        {  return (short)((short) get_bytes(data,  6, 2)); }
        public short sue_gyro_y_at_calibration_GET()//SUE Y gyro at calibration time
        {  return (short)((short) get_bytes(data,  8, 2)); }
        public short sue_gyro_z_at_calibration_GET()//SUE Z gyro at calibration time
        {  return (short)((short) get_bytes(data,  10, 2)); }
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
        {  return  0 + (int)get_bits(data, 200, 1); }
        public @ADSB_EMITTER_TYPE int emitter_type_GET()//Type from ADSB_EMITTER_TYPE enum
        {  return  0 + (int)get_bits(data, 201, 5); }
        public @ADSB_FLAGS int flags_GET()//Flags to indicate various statuses including valid data fields
        {  return  1 + (int)get_bits(data, 206, 7); }
        public String callsign_TRY(Bounds.Inside ph)//The callsign, 8+null
        {
            if(ph.field_bit !=  213 && !try_visit_field(ph, 213)  ||  !try_visit_item(ph, 0)) return null;
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
            return (ph.field_bit !=  213 && !try_visit_field(ph, 213)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
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
        {  return  0 + (int)get_bits(data, 128, 1); }
        public @MAV_COLLISION_ACTION int action_GET()//Action that is being taken to avoid this collision
        {  return  0 + (int)get_bits(data, 129, 3); }
        public @MAV_COLLISION_THREAT_LEVEL int threat_level_GET()//How concerned the aircraft is about this collision
        {  return  0 + (int)get_bits(data, 132, 3); }
    }
    public static class V2_EXTENSION extends GroundControl.V2_EXTENSION
    {
        /**
        *A code that identifies the software component that understands this message (analogous to usb device classes
        *	 or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension
        *	 and the corresponding entry should be added to https:github.com/mavlink/mavlink/extension-message-ids.xml.
        *	 Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...).
        *	 Message_types greater than 32767 are considered local experiments and should not be checked in to any
        *	 widely distributed codebase*/
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
        *	 and other fields.  The entire content of this block is opaque unless you understand any the encoding
        *	 message_type.  The particular encoding used can be extension specific and might not always be documented
        *	 as part of the mavlink specification*/
        public char[] payload_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 5, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *Variable length payload. The length is defined by the remaining message length when subtracting the header
        *	 and other fields.  The entire content of this block is opaque unless you understand any the encoding
        *	 message_type.  The particular encoding used can be extension specific and might not always be documented
        *	 as part of the mavlink specification*/
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
        {  return  0 + (int)get_bits(data, 0, 3); }
        public String text_TRY(Bounds.Inside ph)//Status text message, without null termination character
        {
            if(ph.field_bit !=  3 && !try_visit_field(ph, 3)  ||  !try_visit_item(ph, 0)) return null;
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
            return (ph.field_bit !=  3 && !try_visit_field(ph, 3)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
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
        {  return  1 + (int)get_bits(data, 728, 6); }
        public String cam_definition_uri_TRY(Bounds.Inside ph)//Camera definition URI (if any, otherwise only basic functions will be available).
        {
            if(ph.field_bit !=  734 && !try_visit_field(ph, 734)  ||  !try_visit_item(ph, 0)) return null;
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
            return (ph.field_bit !=  734 && !try_visit_field(ph, 734)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class CAMERA_SETTINGS extends GroundControl.CAMERA_SETTINGS
    {
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public @CAMERA_MODE int mode_id_GET()//Camera mode (CAMERA_MODE)
        {  return  0 + (int)get_bits(data, 32, 3); }
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
        *	 set and capture in progress*/
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

        static final Collection<OnReceive.Handler<FLEXIFUNCTION_SET, Channel>> on_FLEXIFUNCTION_SET = new OnReceive<>();
        static final Collection<OnReceive.Handler<FLEXIFUNCTION_READ_REQ, Channel>> on_FLEXIFUNCTION_READ_REQ = new OnReceive<>();
        static final Collection<OnReceive.Handler<FLEXIFUNCTION_BUFFER_FUNCTION, Channel>> on_FLEXIFUNCTION_BUFFER_FUNCTION = new OnReceive<>();
        static final Collection<OnReceive.Handler<FLEXIFUNCTION_BUFFER_FUNCTION_ACK, Channel>> on_FLEXIFUNCTION_BUFFER_FUNCTION_ACK = new OnReceive<>();
        static final Collection<OnReceive.Handler<FLEXIFUNCTION_DIRECTORY, Channel>> on_FLEXIFUNCTION_DIRECTORY = new OnReceive<>();
        static final Collection<OnReceive.Handler<FLEXIFUNCTION_DIRECTORY_ACK, Channel>> on_FLEXIFUNCTION_DIRECTORY_ACK = new OnReceive<>();
        static final Collection<OnReceive.Handler<FLEXIFUNCTION_COMMAND, Channel>> on_FLEXIFUNCTION_COMMAND = new OnReceive<>();
        static final Collection<OnReceive.Handler<FLEXIFUNCTION_COMMAND_ACK, Channel>> on_FLEXIFUNCTION_COMMAND_ACK = new OnReceive<>();
        static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F2_A, Channel>> on_SERIAL_UDB_EXTRA_F2_A = new OnReceive<>();
        static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F2_B, Channel>> on_SERIAL_UDB_EXTRA_F2_B = new OnReceive<>();
        static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F4, Channel>> on_SERIAL_UDB_EXTRA_F4 = new OnReceive<>();
        static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F5, Channel>> on_SERIAL_UDB_EXTRA_F5 = new OnReceive<>();
        static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F6, Channel>> on_SERIAL_UDB_EXTRA_F6 = new OnReceive<>();
        static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F7, Channel>> on_SERIAL_UDB_EXTRA_F7 = new OnReceive<>();
        static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F8, Channel>> on_SERIAL_UDB_EXTRA_F8 = new OnReceive<>();
        static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F13, Channel>> on_SERIAL_UDB_EXTRA_F13 = new OnReceive<>();
        static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F14, Channel>> on_SERIAL_UDB_EXTRA_F14 = new OnReceive<>();
        static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F15, Channel>> on_SERIAL_UDB_EXTRA_F15 = new OnReceive<>();
        static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F16, Channel>> on_SERIAL_UDB_EXTRA_F16 = new OnReceive<>();
        static final Collection<OnReceive.Handler<ALTITUDES, Channel>> on_ALTITUDES = new OnReceive<>();
        static final Collection<OnReceive.Handler<AIRSPEEDS, Channel>> on_AIRSPEEDS = new OnReceive<>();
        static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F17, Channel>> on_SERIAL_UDB_EXTRA_F17 = new OnReceive<>();
        static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F18, Channel>> on_SERIAL_UDB_EXTRA_F18 = new OnReceive<>();
        static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F19, Channel>> on_SERIAL_UDB_EXTRA_F19 = new OnReceive<>();
        static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F20, Channel>> on_SERIAL_UDB_EXTRA_F20 = new OnReceive<>();
        static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F21, Channel>> on_SERIAL_UDB_EXTRA_F21 = new OnReceive<>();
        static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F22, Channel>> on_SERIAL_UDB_EXTRA_F22 = new OnReceive<>();
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
                case 150:
                    if(pack == null) return new FLEXIFUNCTION_SET();
                    ((OnReceive) on_FLEXIFUNCTION_SET).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 151:
                    if(pack == null) return new FLEXIFUNCTION_READ_REQ();
                    ((OnReceive) on_FLEXIFUNCTION_READ_REQ).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 152:
                    if(pack == null) return new FLEXIFUNCTION_BUFFER_FUNCTION();
                    ((OnReceive) on_FLEXIFUNCTION_BUFFER_FUNCTION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 153:
                    if(pack == null) return new FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
                    ((OnReceive) on_FLEXIFUNCTION_BUFFER_FUNCTION_ACK).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 155:
                    if(pack == null) return new FLEXIFUNCTION_DIRECTORY();
                    ((OnReceive) on_FLEXIFUNCTION_DIRECTORY).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 156:
                    if(pack == null) return new FLEXIFUNCTION_DIRECTORY_ACK();
                    ((OnReceive) on_FLEXIFUNCTION_DIRECTORY_ACK).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 157:
                    if(pack == null) return new FLEXIFUNCTION_COMMAND();
                    ((OnReceive) on_FLEXIFUNCTION_COMMAND).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 158:
                    if(pack == null) return new FLEXIFUNCTION_COMMAND_ACK();
                    ((OnReceive) on_FLEXIFUNCTION_COMMAND_ACK).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 170:
                    if(pack == null) return new SERIAL_UDB_EXTRA_F2_A();
                    ((OnReceive) on_SERIAL_UDB_EXTRA_F2_A).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 171:
                    if(pack == null) return new SERIAL_UDB_EXTRA_F2_B();
                    ((OnReceive) on_SERIAL_UDB_EXTRA_F2_B).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 172:
                    if(pack == null) return new SERIAL_UDB_EXTRA_F4();
                    ((OnReceive) on_SERIAL_UDB_EXTRA_F4).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 173:
                    if(pack == null) return new SERIAL_UDB_EXTRA_F5();
                    ((OnReceive) on_SERIAL_UDB_EXTRA_F5).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 174:
                    if(pack == null) return new SERIAL_UDB_EXTRA_F6();
                    ((OnReceive) on_SERIAL_UDB_EXTRA_F6).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 175:
                    if(pack == null) return new SERIAL_UDB_EXTRA_F7();
                    ((OnReceive) on_SERIAL_UDB_EXTRA_F7).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 176:
                    if(pack == null) return new SERIAL_UDB_EXTRA_F8();
                    ((OnReceive) on_SERIAL_UDB_EXTRA_F8).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 177:
                    if(pack == null) return new SERIAL_UDB_EXTRA_F13();
                    ((OnReceive) on_SERIAL_UDB_EXTRA_F13).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 178:
                    if(pack == null) return new SERIAL_UDB_EXTRA_F14();
                    ((OnReceive) on_SERIAL_UDB_EXTRA_F14).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 179:
                    if(pack == null) return new SERIAL_UDB_EXTRA_F15();
                    ((OnReceive) on_SERIAL_UDB_EXTRA_F15).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 180:
                    if(pack == null) return new SERIAL_UDB_EXTRA_F16();
                    ((OnReceive) on_SERIAL_UDB_EXTRA_F16).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 181:
                    if(pack == null) return new ALTITUDES();
                    ((OnReceive) on_ALTITUDES).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 182:
                    if(pack == null) return new AIRSPEEDS();
                    ((OnReceive) on_AIRSPEEDS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 183:
                    if(pack == null) return new SERIAL_UDB_EXTRA_F17();
                    ((OnReceive) on_SERIAL_UDB_EXTRA_F17).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 184:
                    if(pack == null) return new SERIAL_UDB_EXTRA_F18();
                    ((OnReceive) on_SERIAL_UDB_EXTRA_F18).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 185:
                    if(pack == null) return new SERIAL_UDB_EXTRA_F19();
                    ((OnReceive) on_SERIAL_UDB_EXTRA_F19).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 186:
                    if(pack == null) return new SERIAL_UDB_EXTRA_F20();
                    ((OnReceive) on_SERIAL_UDB_EXTRA_F20).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 187:
                    if(pack == null) return new SERIAL_UDB_EXTRA_F21();
                    ((OnReceive) on_SERIAL_UDB_EXTRA_F21).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 188:
                    if(pack == null) return new SERIAL_UDB_EXTRA_F22();
                    ((OnReceive) on_SERIAL_UDB_EXTRA_F22).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
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
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_EMERGENCY);
            assert(pack.mavlink_version_GET() == (char)239);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_VTOL_RESERVED5);
            assert(pack.custom_mode_GET() == 3353874973L);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_MISSION_FULL);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.custom_mode_SET(3353874973L) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_EMERGENCY) ;
        p0.mavlink_version_SET((char)239) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED)) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_MISSION_FULL) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_VTOL_RESERVED5) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.current_battery_GET() == (short) -27601);
            assert(pack.battery_remaining_GET() == (byte)89);
            assert(pack.errors_count1_GET() == (char)27362);
            assert(pack.load_GET() == (char)55369);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
            assert(pack.errors_count3_GET() == (char)32074);
            assert(pack.errors_comm_GET() == (char)52001);
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.errors_count4_GET() == (char)31324);
            assert(pack.drop_rate_comm_GET() == (char)32955);
            assert(pack.errors_count2_GET() == (char)13633);
            assert(pack.voltage_battery_GET() == (char)32671);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.current_battery_SET((short) -27601) ;
        p1.errors_count3_SET((char)32074) ;
        p1.load_SET((char)55369) ;
        p1.errors_comm_SET((char)52001) ;
        p1.drop_rate_comm_SET((char)32955) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL)) ;
        p1.errors_count2_SET((char)13633) ;
        p1.errors_count4_SET((char)31324) ;
        p1.voltage_battery_SET((char)32671) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        p1.errors_count1_SET((char)27362) ;
        p1.battery_remaining_SET((byte)89) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 6293782507574892064L);
            assert(pack.time_boot_ms_GET() == 2036836946L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(2036836946L) ;
        p2.time_unix_usec_SET(6293782507574892064L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.afy_GET() == 3.0594958E38F);
            assert(pack.yaw_rate_GET() == 2.1273745E38F);
            assert(pack.vx_GET() == 4.2778614E37F);
            assert(pack.afx_GET() == 3.4908202E37F);
            assert(pack.vy_GET() == -4.4387093E36F);
            assert(pack.x_GET() == 1.1348428E38F);
            assert(pack.z_GET() == 2.50398E38F);
            assert(pack.time_boot_ms_GET() == 1086942785L);
            assert(pack.y_GET() == -1.2334093E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.yaw_GET() == -1.3223728E38F);
            assert(pack.afz_GET() == -3.3368823E38F);
            assert(pack.type_mask_GET() == (char)45429);
            assert(pack.vz_GET() == -3.0552905E38F);
        });
        POSITION_TARGET_LOCAL_NED p3 = new POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.vx_SET(4.2778614E37F) ;
        p3.afx_SET(3.4908202E37F) ;
        p3.vz_SET(-3.0552905E38F) ;
        p3.time_boot_ms_SET(1086942785L) ;
        p3.type_mask_SET((char)45429) ;
        p3.x_SET(1.1348428E38F) ;
        p3.afz_SET(-3.3368823E38F) ;
        p3.afy_SET(3.0594958E38F) ;
        p3.vy_SET(-4.4387093E36F) ;
        p3.y_SET(-1.2334093E38F) ;
        p3.z_SET(2.50398E38F) ;
        p3.yaw_rate_SET(2.1273745E38F) ;
        p3.yaw_SET(-1.3223728E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        TestChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4555202748351770902L);
            assert(pack.target_system_GET() == (char)109);
            assert(pack.target_component_GET() == (char)88);
            assert(pack.seq_GET() == 3308918536L);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.time_usec_SET(4555202748351770902L) ;
        p4.seq_SET(3308918536L) ;
        p4.target_component_SET((char)88) ;
        p4.target_system_SET((char)109) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == (char)126);
            assert(pack.control_request_GET() == (char)2);
            assert(pack.passkey_LEN(ph) == 6);
            assert(pack.passkey_TRY(ph).equals("krsfsa"));
            assert(pack.target_system_GET() == (char)216);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.control_request_SET((char)2) ;
        p5.passkey_SET("krsfsa", PH) ;
        p5.version_SET((char)126) ;
        p5.target_system_SET((char)216) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.control_request_GET() == (char)97);
            assert(pack.ack_GET() == (char)238);
            assert(pack.gcs_system_id_GET() == (char)166);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.control_request_SET((char)97) ;
        p6.ack_SET((char)238) ;
        p6.gcs_system_id_SET((char)166) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 18);
            assert(pack.key_TRY(ph).equals("aLdquxrysqmhgkjShZ"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("aLdquxrysqmhgkjShZ", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.custom_mode_GET() == 1362538850L);
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
            assert(pack.target_system_GET() == (char)4);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_GUIDED_DISARMED) ;
        p11.custom_mode_SET(1362538850L) ;
        p11.target_system_SET((char)4) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)211);
            assert(pack.param_id_LEN(ph) == 14);
            assert(pack.param_id_TRY(ph).equals("xjuhcfomThoyvi"));
            assert(pack.param_index_GET() == (short)22881);
            assert(pack.target_component_GET() == (char)76);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.param_index_SET((short)22881) ;
        p20.target_component_SET((char)76) ;
        p20.target_system_SET((char)211) ;
        p20.param_id_SET("xjuhcfomThoyvi", PH) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)143);
            assert(pack.target_component_GET() == (char)126);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)143) ;
        p21.target_component_SET((char)126) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_value_GET() == -6.0818975E37F);
            assert(pack.param_index_GET() == (char)40435);
            assert(pack.param_count_GET() == (char)12468);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
            assert(pack.param_id_LEN(ph) == 14);
            assert(pack.param_id_TRY(ph).equals("nyfqcwscjnztii"));
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_count_SET((char)12468) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32) ;
        p22.param_id_SET("nyfqcwscjnztii", PH) ;
        p22.param_value_SET(-6.0818975E37F) ;
        p22.param_index_SET((char)40435) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)207);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
            assert(pack.param_value_GET() == -2.9346754E38F);
            assert(pack.target_system_GET() == (char)116);
            assert(pack.param_id_LEN(ph) == 3);
            assert(pack.param_id_TRY(ph).equals("tvr"));
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.target_component_SET((char)207) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16) ;
        p23.param_id_SET("tvr", PH) ;
        p23.target_system_SET((char)116) ;
        p23.param_value_SET(-2.9346754E38F) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7879713789008077068L);
            assert(pack.h_acc_TRY(ph) == 3299856503L);
            assert(pack.lon_GET() == 1160287416);
            assert(pack.satellites_visible_GET() == (char)86);
            assert(pack.alt_ellipsoid_TRY(ph) == -1103042032);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
            assert(pack.cog_GET() == (char)58437);
            assert(pack.alt_GET() == 1817983631);
            assert(pack.vel_GET() == (char)20925);
            assert(pack.eph_GET() == (char)28027);
            assert(pack.epv_GET() == (char)53103);
            assert(pack.lat_GET() == 1378025407);
            assert(pack.vel_acc_TRY(ph) == 4136080594L);
            assert(pack.hdg_acc_TRY(ph) == 1462871338L);
            assert(pack.v_acc_TRY(ph) == 2574710562L);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.h_acc_SET(3299856503L, PH) ;
        p24.time_usec_SET(7879713789008077068L) ;
        p24.satellites_visible_SET((char)86) ;
        p24.lon_SET(1160287416) ;
        p24.vel_SET((char)20925) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX) ;
        p24.v_acc_SET(2574710562L, PH) ;
        p24.alt_ellipsoid_SET(-1103042032, PH) ;
        p24.vel_acc_SET(4136080594L, PH) ;
        p24.lat_SET(1378025407) ;
        p24.hdg_acc_SET(1462871338L, PH) ;
        p24.cog_SET((char)58437) ;
        p24.alt_SET(1817983631) ;
        p24.epv_SET((char)53103) ;
        p24.eph_SET((char)28027) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)92, (char)7, (char)0, (char)140, (char)18, (char)208, (char)155, (char)148, (char)135, (char)191, (char)110, (char)124, (char)164, (char)190, (char)104, (char)108, (char)108, (char)83, (char)1, (char)136}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)153, (char)107, (char)241, (char)191, (char)150, (char)60, (char)88, (char)168, (char)112, (char)57, (char)179, (char)40, (char)88, (char)70, (char)220, (char)8, (char)205, (char)40, (char)183, (char)121}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)239, (char)105, (char)85, (char)105, (char)62, (char)246, (char)145, (char)70, (char)150, (char)2, (char)215, (char)110, (char)76, (char)39, (char)117, (char)98, (char)108, (char)68, (char)12, (char)137}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)151, (char)195, (char)166, (char)1, (char)63, (char)252, (char)124, (char)54, (char)74, (char)182, (char)21, (char)190, (char)164, (char)160, (char)234, (char)188, (char)179, (char)71, (char)61, (char)75}));
            assert(pack.satellites_visible_GET() == (char)222);
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)130, (char)68, (char)146, (char)59, (char)98, (char)148, (char)10, (char)44, (char)71, (char)180, (char)6, (char)217, (char)19, (char)68, (char)36, (char)126, (char)74, (char)208, (char)144, (char)133}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_used_SET(new char[] {(char)92, (char)7, (char)0, (char)140, (char)18, (char)208, (char)155, (char)148, (char)135, (char)191, (char)110, (char)124, (char)164, (char)190, (char)104, (char)108, (char)108, (char)83, (char)1, (char)136}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)239, (char)105, (char)85, (char)105, (char)62, (char)246, (char)145, (char)70, (char)150, (char)2, (char)215, (char)110, (char)76, (char)39, (char)117, (char)98, (char)108, (char)68, (char)12, (char)137}, 0) ;
        p25.satellites_visible_SET((char)222) ;
        p25.satellite_prn_SET(new char[] {(char)151, (char)195, (char)166, (char)1, (char)63, (char)252, (char)124, (char)54, (char)74, (char)182, (char)21, (char)190, (char)164, (char)160, (char)234, (char)188, (char)179, (char)71, (char)61, (char)75}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)153, (char)107, (char)241, (char)191, (char)150, (char)60, (char)88, (char)168, (char)112, (char)57, (char)179, (char)40, (char)88, (char)70, (char)220, (char)8, (char)205, (char)40, (char)183, (char)121}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)130, (char)68, (char)146, (char)59, (char)98, (char)148, (char)10, (char)44, (char)71, (char)180, (char)6, (char)217, (char)19, (char)68, (char)36, (char)126, (char)74, (char)208, (char)144, (char)133}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.zmag_GET() == (short) -22095);
            assert(pack.time_boot_ms_GET() == 3488588822L);
            assert(pack.yacc_GET() == (short)6695);
            assert(pack.zacc_GET() == (short) -7985);
            assert(pack.zgyro_GET() == (short)11395);
            assert(pack.ygyro_GET() == (short) -21277);
            assert(pack.xgyro_GET() == (short) -31802);
            assert(pack.xmag_GET() == (short)3470);
            assert(pack.ymag_GET() == (short)4919);
            assert(pack.xacc_GET() == (short)25517);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.zmag_SET((short) -22095) ;
        p26.time_boot_ms_SET(3488588822L) ;
        p26.yacc_SET((short)6695) ;
        p26.xacc_SET((short)25517) ;
        p26.xmag_SET((short)3470) ;
        p26.xgyro_SET((short) -31802) ;
        p26.zgyro_SET((short)11395) ;
        p26.ymag_SET((short)4919) ;
        p26.zacc_SET((short) -7985) ;
        p26.ygyro_SET((short) -21277) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.xmag_GET() == (short) -460);
            assert(pack.zacc_GET() == (short) -18750);
            assert(pack.yacc_GET() == (short) -19885);
            assert(pack.zgyro_GET() == (short)2574);
            assert(pack.time_usec_GET() == 8734732291403079039L);
            assert(pack.ymag_GET() == (short)23668);
            assert(pack.zmag_GET() == (short)29599);
            assert(pack.ygyro_GET() == (short) -29760);
            assert(pack.xgyro_GET() == (short) -3200);
            assert(pack.xacc_GET() == (short)22441);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.zmag_SET((short)29599) ;
        p27.zgyro_SET((short)2574) ;
        p27.ygyro_SET((short) -29760) ;
        p27.xgyro_SET((short) -3200) ;
        p27.zacc_SET((short) -18750) ;
        p27.time_usec_SET(8734732291403079039L) ;
        p27.ymag_SET((short)23668) ;
        p27.xacc_SET((short)22441) ;
        p27.xmag_SET((short) -460) ;
        p27.yacc_SET((short) -19885) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff2_GET() == (short)28120);
            assert(pack.temperature_GET() == (short) -14450);
            assert(pack.press_abs_GET() == (short) -30721);
            assert(pack.time_usec_GET() == 6493074779815204510L);
            assert(pack.press_diff1_GET() == (short)8087);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.temperature_SET((short) -14450) ;
        p28.time_usec_SET(6493074779815204510L) ;
        p28.press_abs_SET((short) -30721) ;
        p28.press_diff2_SET((short)28120) ;
        p28.press_diff1_SET((short)8087) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)32075);
            assert(pack.press_diff_GET() == -4.5827496E37F);
            assert(pack.press_abs_GET() == -1.8870413E38F);
            assert(pack.time_boot_ms_GET() == 1918981029L);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.time_boot_ms_SET(1918981029L) ;
        p29.press_diff_SET(-4.5827496E37F) ;
        p29.press_abs_SET(-1.8870413E38F) ;
        p29.temperature_SET((short)32075) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == 2.0555257E38F);
            assert(pack.yaw_GET() == -3.0433555E38F);
            assert(pack.roll_GET() == 1.7001983E38F);
            assert(pack.pitch_GET() == 3.2150824E38F);
            assert(pack.pitchspeed_GET() == -1.2751418E38F);
            assert(pack.time_boot_ms_GET() == 3243898503L);
            assert(pack.rollspeed_GET() == 2.5616485E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.yawspeed_SET(2.0555257E38F) ;
        p30.pitch_SET(3.2150824E38F) ;
        p30.time_boot_ms_SET(3243898503L) ;
        p30.roll_SET(1.7001983E38F) ;
        p30.rollspeed_SET(2.5616485E38F) ;
        p30.pitchspeed_SET(-1.2751418E38F) ;
        p30.yaw_SET(-3.0433555E38F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == 8.2999514E37F);
            assert(pack.time_boot_ms_GET() == 4199380357L);
            assert(pack.rollspeed_GET() == -4.6808353E37F);
            assert(pack.q2_GET() == 8.38273E37F);
            assert(pack.q4_GET() == -2.8737818E38F);
            assert(pack.q1_GET() == 1.8335957E38F);
            assert(pack.pitchspeed_GET() == 8.860123E37F);
            assert(pack.q3_GET() == 8.406224E37F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q1_SET(1.8335957E38F) ;
        p31.rollspeed_SET(-4.6808353E37F) ;
        p31.q3_SET(8.406224E37F) ;
        p31.q4_SET(-2.8737818E38F) ;
        p31.time_boot_ms_SET(4199380357L) ;
        p31.q2_SET(8.38273E37F) ;
        p31.pitchspeed_SET(8.860123E37F) ;
        p31.yawspeed_SET(8.2999514E37F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 1.1766955E37F);
            assert(pack.vx_GET() == 1.5901531E38F);
            assert(pack.vz_GET() == -2.37026E38F);
            assert(pack.z_GET() == -1.3009859E38F);
            assert(pack.vy_GET() == 1.4299548E38F);
            assert(pack.y_GET() == 5.3185493E37F);
            assert(pack.time_boot_ms_GET() == 2394106617L);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.vx_SET(1.5901531E38F) ;
        p32.vy_SET(1.4299548E38F) ;
        p32.time_boot_ms_SET(2394106617L) ;
        p32.vz_SET(-2.37026E38F) ;
        p32.z_SET(-1.3009859E38F) ;
        p32.y_SET(5.3185493E37F) ;
        p32.x_SET(1.1766955E37F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == (short)24174);
            assert(pack.hdg_GET() == (char)19166);
            assert(pack.alt_GET() == 730352637);
            assert(pack.vx_GET() == (short)7617);
            assert(pack.relative_alt_GET() == 427456477);
            assert(pack.vy_GET() == (short) -19115);
            assert(pack.lon_GET() == 365608273);
            assert(pack.lat_GET() == 289539720);
            assert(pack.time_boot_ms_GET() == 2378185335L);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.time_boot_ms_SET(2378185335L) ;
        p33.vy_SET((short) -19115) ;
        p33.relative_alt_SET(427456477) ;
        p33.hdg_SET((char)19166) ;
        p33.alt_SET(730352637) ;
        p33.lon_SET(365608273) ;
        p33.vz_SET((short)24174) ;
        p33.lat_SET(289539720) ;
        p33.vx_SET((short)7617) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan4_scaled_GET() == (short) -17185);
            assert(pack.chan6_scaled_GET() == (short)5762);
            assert(pack.port_GET() == (char)166);
            assert(pack.chan3_scaled_GET() == (short)21976);
            assert(pack.chan2_scaled_GET() == (short)25168);
            assert(pack.chan8_scaled_GET() == (short)1688);
            assert(pack.rssi_GET() == (char)185);
            assert(pack.chan1_scaled_GET() == (short)2971);
            assert(pack.chan7_scaled_GET() == (short) -7683);
            assert(pack.time_boot_ms_GET() == 2213589542L);
            assert(pack.chan5_scaled_GET() == (short) -21718);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan6_scaled_SET((short)5762) ;
        p34.chan5_scaled_SET((short) -21718) ;
        p34.chan1_scaled_SET((short)2971) ;
        p34.chan7_scaled_SET((short) -7683) ;
        p34.port_SET((char)166) ;
        p34.chan2_scaled_SET((short)25168) ;
        p34.chan3_scaled_SET((short)21976) ;
        p34.chan8_scaled_SET((short)1688) ;
        p34.rssi_SET((char)185) ;
        p34.chan4_scaled_SET((short) -17185) ;
        p34.time_boot_ms_SET(2213589542L) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan7_raw_GET() == (char)8059);
            assert(pack.port_GET() == (char)128);
            assert(pack.chan8_raw_GET() == (char)47179);
            assert(pack.chan5_raw_GET() == (char)12000);
            assert(pack.chan3_raw_GET() == (char)30218);
            assert(pack.chan6_raw_GET() == (char)10861);
            assert(pack.chan2_raw_GET() == (char)11930);
            assert(pack.rssi_GET() == (char)49);
            assert(pack.time_boot_ms_GET() == 3156414920L);
            assert(pack.chan1_raw_GET() == (char)53015);
            assert(pack.chan4_raw_GET() == (char)45221);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan7_raw_SET((char)8059) ;
        p35.rssi_SET((char)49) ;
        p35.chan8_raw_SET((char)47179) ;
        p35.chan2_raw_SET((char)11930) ;
        p35.port_SET((char)128) ;
        p35.chan3_raw_SET((char)30218) ;
        p35.chan4_raw_SET((char)45221) ;
        p35.chan5_raw_SET((char)12000) ;
        p35.time_boot_ms_SET(3156414920L) ;
        p35.chan6_raw_SET((char)10861) ;
        p35.chan1_raw_SET((char)53015) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo7_raw_GET() == (char)27333);
            assert(pack.servo8_raw_GET() == (char)31279);
            assert(pack.servo15_raw_TRY(ph) == (char)55106);
            assert(pack.servo10_raw_TRY(ph) == (char)45324);
            assert(pack.servo11_raw_TRY(ph) == (char)19354);
            assert(pack.time_usec_GET() == 1655337744L);
            assert(pack.servo5_raw_GET() == (char)46705);
            assert(pack.servo12_raw_TRY(ph) == (char)8957);
            assert(pack.servo6_raw_GET() == (char)18249);
            assert(pack.servo14_raw_TRY(ph) == (char)30182);
            assert(pack.servo2_raw_GET() == (char)22904);
            assert(pack.servo3_raw_GET() == (char)36974);
            assert(pack.servo1_raw_GET() == (char)63208);
            assert(pack.servo4_raw_GET() == (char)11174);
            assert(pack.port_GET() == (char)151);
            assert(pack.servo16_raw_TRY(ph) == (char)22111);
            assert(pack.servo9_raw_TRY(ph) == (char)36581);
            assert(pack.servo13_raw_TRY(ph) == (char)27950);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo12_raw_SET((char)8957, PH) ;
        p36.servo2_raw_SET((char)22904) ;
        p36.servo7_raw_SET((char)27333) ;
        p36.servo16_raw_SET((char)22111, PH) ;
        p36.servo9_raw_SET((char)36581, PH) ;
        p36.servo11_raw_SET((char)19354, PH) ;
        p36.port_SET((char)151) ;
        p36.time_usec_SET(1655337744L) ;
        p36.servo15_raw_SET((char)55106, PH) ;
        p36.servo4_raw_SET((char)11174) ;
        p36.servo1_raw_SET((char)63208) ;
        p36.servo5_raw_SET((char)46705) ;
        p36.servo3_raw_SET((char)36974) ;
        p36.servo10_raw_SET((char)45324, PH) ;
        p36.servo14_raw_SET((char)30182, PH) ;
        p36.servo8_raw_SET((char)31279) ;
        p36.servo13_raw_SET((char)27950, PH) ;
        p36.servo6_raw_SET((char)18249) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)132);
            assert(pack.start_index_GET() == (short)19224);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_component_GET() == (char)214);
            assert(pack.end_index_GET() == (short) -23121);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_system_SET((char)132) ;
        p37.target_component_SET((char)214) ;
        p37.start_index_SET((short)19224) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p37.end_index_SET((short) -23121) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_index_GET() == (short) -14005);
            assert(pack.start_index_GET() == (short)26279);
            assert(pack.target_system_GET() == (char)140);
            assert(pack.target_component_GET() == (char)20);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_component_SET((char)20) ;
        p38.target_system_SET((char)140) ;
        p38.end_index_SET((short) -14005) ;
        p38.start_index_SET((short)26279) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 2.6210913E38F);
            assert(pack.param4_GET() == -1.4089345E38F);
            assert(pack.autocontinue_GET() == (char)190);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.param1_GET() == -6.531674E37F);
            assert(pack.target_system_GET() == (char)55);
            assert(pack.param2_GET() == -2.2690767E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_component_GET() == (char)51);
            assert(pack.param3_GET() == -2.6272774E37F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_TAKEOFF);
            assert(pack.current_GET() == (char)19);
            assert(pack.y_GET() == 9.147048E37F);
            assert(pack.z_GET() == 1.159419E38F);
            assert(pack.seq_GET() == (char)2480);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.param3_SET(-2.6272774E37F) ;
        p39.command_SET(MAV_CMD.MAV_CMD_NAV_TAKEOFF) ;
        p39.target_system_SET((char)55) ;
        p39.autocontinue_SET((char)190) ;
        p39.y_SET(9.147048E37F) ;
        p39.param4_SET(-1.4089345E38F) ;
        p39.z_SET(1.159419E38F) ;
        p39.target_component_SET((char)51) ;
        p39.seq_SET((char)2480) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p39.x_SET(2.6210913E38F) ;
        p39.param2_SET(-2.2690767E38F) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p39.param1_SET(-6.531674E37F) ;
        p39.current_SET((char)19) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)60722);
            assert(pack.target_component_GET() == (char)143);
            assert(pack.target_system_GET() == (char)3);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.seq_SET((char)60722) ;
        p40.target_system_SET((char)3) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p40.target_component_SET((char)143) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)48);
            assert(pack.seq_GET() == (char)64822);
            assert(pack.target_system_GET() == (char)161);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_component_SET((char)48) ;
        p41.seq_SET((char)64822) ;
        p41.target_system_SET((char)161) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)13956);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)13956) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)162);
            assert(pack.target_system_GET() == (char)65);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_component_SET((char)162) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p43.target_system_SET((char)65) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)40);
            assert(pack.count_GET() == (char)24126);
            assert(pack.target_component_GET() == (char)126);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.target_system_SET((char)40) ;
        p44.target_component_SET((char)126) ;
        p44.count_SET((char)24126) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)26);
            assert(pack.target_system_GET() == (char)73);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_component_SET((char)26) ;
        p45.target_system_SET((char)73) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)4201);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)4201) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)28);
            assert(pack.target_system_GET() == (char)60);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM7);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.target_component_SET((char)28) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM7) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p47.target_system_SET((char)60) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)56);
            assert(pack.latitude_GET() == -13080021);
            assert(pack.altitude_GET() == 934886536);
            assert(pack.longitude_GET() == -480700025);
            assert(pack.time_usec_TRY(ph) == 1625055642380738117L);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.time_usec_SET(1625055642380738117L, PH) ;
        p48.latitude_SET(-13080021) ;
        p48.longitude_SET(-480700025) ;
        p48.altitude_SET(934886536) ;
        p48.target_system_SET((char)56) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == -667234925);
            assert(pack.time_usec_TRY(ph) == 6312479353783452526L);
            assert(pack.altitude_GET() == 1879019146);
            assert(pack.longitude_GET() == 725353289);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.altitude_SET(1879019146) ;
        p49.time_usec_SET(6312479353783452526L, PH) ;
        p49.latitude_SET(-667234925) ;
        p49.longitude_SET(725353289) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)150);
            assert(pack.param_value_max_GET() == 2.373733E38F);
            assert(pack.param_index_GET() == (short) -511);
            assert(pack.target_system_GET() == (char)22);
            assert(pack.param_id_LEN(ph) == 7);
            assert(pack.param_id_TRY(ph).equals("taelRkg"));
            assert(pack.parameter_rc_channel_index_GET() == (char)130);
            assert(pack.param_value_min_GET() == -2.2048014E38F);
            assert(pack.scale_GET() == -3.0532805E38F);
            assert(pack.param_value0_GET() == 2.89825E38F);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.target_system_SET((char)22) ;
        p50.scale_SET(-3.0532805E38F) ;
        p50.param_value_max_SET(2.373733E38F) ;
        p50.parameter_rc_channel_index_SET((char)130) ;
        p50.param_value0_SET(2.89825E38F) ;
        p50.param_value_min_SET(-2.2048014E38F) ;
        p50.target_component_SET((char)150) ;
        p50.param_index_SET((short) -511) ;
        p50.param_id_SET("taelRkg", PH) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)77);
            assert(pack.target_component_GET() == (char)16);
            assert(pack.seq_GET() == (char)22343);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_component_SET((char)16) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p51.target_system_SET((char)77) ;
        p51.seq_SET((char)22343) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.target_system_GET() == (char)207);
            assert(pack.p1z_GET() == -2.902684E38F);
            assert(pack.target_component_GET() == (char)186);
            assert(pack.p2x_GET() == 1.9588834E38F);
            assert(pack.p1x_GET() == 1.9891963E38F);
            assert(pack.p1y_GET() == -1.0714105E38F);
            assert(pack.p2z_GET() == 1.15949E38F);
            assert(pack.p2y_GET() == -2.4664002E38F);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p1x_SET(1.9891963E38F) ;
        p54.target_system_SET((char)207) ;
        p54.p1y_SET(-1.0714105E38F) ;
        p54.p2z_SET(1.15949E38F) ;
        p54.p1z_SET(-2.902684E38F) ;
        p54.target_component_SET((char)186) ;
        p54.p2y_SET(-2.4664002E38F) ;
        p54.p2x_SET(1.9588834E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1x_GET() == 5.225856E37F);
            assert(pack.p2z_GET() == 1.6105432E38F);
            assert(pack.p2x_GET() == -3.337005E38F);
            assert(pack.p1z_GET() == 1.8056809E38F);
            assert(pack.p1y_GET() == 8.600718E37F);
            assert(pack.p2y_GET() == -4.6491633E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p2y_SET(-4.6491633E37F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p55.p2x_SET(-3.337005E38F) ;
        p55.p2z_SET(1.6105432E38F) ;
        p55.p1z_SET(1.8056809E38F) ;
        p55.p1y_SET(8.600718E37F) ;
        p55.p1x_SET(5.225856E37F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.8712685E36F, 1.655513E38F, 8.0561057E37F, -1.8939964E38F, 8.179876E37F, -2.5393778E38F, -4.5511456E37F, -2.6127814E38F, -3.90579E37F}));
            assert(pack.pitchspeed_GET() == -2.392459E38F);
            assert(pack.rollspeed_GET() == -5.4045026E37F);
            assert(pack.time_usec_GET() == 5001732056105731580L);
            assert(pack.yawspeed_GET() == 1.5322247E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.1418665E38F, -1.749868E38F, 1.5834127E38F, 1.671658E38F}));
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.q_SET(new float[] {2.1418665E38F, -1.749868E38F, 1.5834127E38F, 1.671658E38F}, 0) ;
        p61.pitchspeed_SET(-2.392459E38F) ;
        p61.yawspeed_SET(1.5322247E38F) ;
        p61.rollspeed_SET(-5.4045026E37F) ;
        p61.covariance_SET(new float[] {-2.8712685E36F, 1.655513E38F, 8.0561057E37F, -1.8939964E38F, 8.179876E37F, -2.5393778E38F, -4.5511456E37F, -2.6127814E38F, -3.90579E37F}, 0) ;
        p61.time_usec_SET(5001732056105731580L) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.target_bearing_GET() == (short)11816);
            assert(pack.xtrack_error_GET() == -1.5831471E38F);
            assert(pack.alt_error_GET() == -6.9962666E37F);
            assert(pack.nav_bearing_GET() == (short) -15342);
            assert(pack.nav_roll_GET() == -1.7364154E38F);
            assert(pack.nav_pitch_GET() == -1.5653353E38F);
            assert(pack.aspd_error_GET() == 1.5299677E38F);
            assert(pack.wp_dist_GET() == (char)12895);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.target_bearing_SET((short)11816) ;
        p62.nav_bearing_SET((short) -15342) ;
        p62.aspd_error_SET(1.5299677E38F) ;
        p62.nav_pitch_SET(-1.5653353E38F) ;
        p62.alt_error_SET(-6.9962666E37F) ;
        p62.xtrack_error_SET(-1.5831471E38F) ;
        p62.wp_dist_SET((char)12895) ;
        p62.nav_roll_SET(-1.7364154E38F) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == -1.5874267E38F);
            assert(pack.relative_alt_GET() == -359125297);
            assert(pack.time_usec_GET() == 6484526536755431127L);
            assert(pack.vy_GET() == -2.7970139E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.1622209E38F, 1.8235284E38F, 3.0431738E38F, -1.7274177E38F, 1.3595758E38F, 1.810659E38F, -1.1119068E38F, 2.5050232E38F, -9.489648E37F, -2.5611298E38F, -2.0877459E37F, -3.0620486E38F, -7.959091E37F, -2.6540894E37F, 1.9092096E38F, -2.2371098E38F, 2.6504495E38F, -7.396175E37F, -9.299453E37F, -9.780108E37F, -3.140041E38F, -4.9646846E37F, -1.2635876E38F, -7.531087E37F, 3.2567343E38F, -1.9493527E38F, -4.7035217E37F, 3.2505519E38F, -2.0138423E38F, -3.129965E37F, -1.7482779E37F, -2.3489432E38F, 2.7680565E38F, 8.627351E37F, 3.3551943E38F, 8.4017154E37F}));
            assert(pack.lon_GET() == -839554734);
            assert(pack.vx_GET() == -2.779461E38F);
            assert(pack.alt_GET() == 1037836);
            assert(pack.lat_GET() == 305066719);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.time_usec_SET(6484526536755431127L) ;
        p63.vy_SET(-2.7970139E38F) ;
        p63.lon_SET(-839554734) ;
        p63.relative_alt_SET(-359125297) ;
        p63.vz_SET(-1.5874267E38F) ;
        p63.vx_SET(-2.779461E38F) ;
        p63.alt_SET(1037836) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS) ;
        p63.lat_SET(305066719) ;
        p63.covariance_SET(new float[] {-2.1622209E38F, 1.8235284E38F, 3.0431738E38F, -1.7274177E38F, 1.3595758E38F, 1.810659E38F, -1.1119068E38F, 2.5050232E38F, -9.489648E37F, -2.5611298E38F, -2.0877459E37F, -3.0620486E38F, -7.959091E37F, -2.6540894E37F, 1.9092096E38F, -2.2371098E38F, 2.6504495E38F, -7.396175E37F, -9.299453E37F, -9.780108E37F, -3.140041E38F, -4.9646846E37F, -1.2635876E38F, -7.531087E37F, 3.2567343E38F, -1.9493527E38F, -4.7035217E37F, 3.2505519E38F, -2.0138423E38F, -3.129965E37F, -1.7482779E37F, -2.3489432E38F, 2.7680565E38F, 8.627351E37F, 3.3551943E38F, 8.4017154E37F}, 0) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 7.6164236E37F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            assert(pack.y_GET() == 5.546459E36F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {9.4902434E36F, -3.1705214E37F, -7.08678E36F, 7.879629E37F, -7.778835E37F, -2.071978E38F, -2.4436935E38F, 3.0825954E37F, -3.0331626E38F, 1.5104571E38F, 1.5170247E38F, 6.197477E37F, 1.1500923E38F, 7.276888E37F, -7.06344E37F, -1.818327E38F, -8.008463E36F, 2.4842717E37F, 1.9829368E38F, 2.372639E38F, 2.7307372E38F, -1.2705483E38F, 2.7900535E38F, -3.4291744E37F, 3.3231475E38F, 4.5620316E37F, -5.561626E37F, -2.2841216E38F, 1.5739326E38F, 2.706278E38F, 2.5168049E38F, -2.0721854E38F, 1.2237067E38F, -1.1608641E38F, -3.2100714E38F, -2.8990129E38F, -8.927458E37F, -1.8058916E38F, 2.7107387E37F, 2.4993683E38F, 8.686685E37F, 1.8629144E38F, -1.873008E38F, -2.4104618E38F, -1.3671662E38F}));
            assert(pack.vx_GET() == 1.1566026E37F);
            assert(pack.az_GET() == -1.4382871E38F);
            assert(pack.vy_GET() == 2.4320944E38F);
            assert(pack.x_GET() == -2.1028206E38F);
            assert(pack.vz_GET() == -1.532526E38F);
            assert(pack.ax_GET() == -8.010972E37F);
            assert(pack.ay_GET() == 8.122923E37F);
            assert(pack.time_usec_GET() == 2070834105847222958L);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.time_usec_SET(2070834105847222958L) ;
        p64.az_SET(-1.4382871E38F) ;
        p64.vz_SET(-1.532526E38F) ;
        p64.ay_SET(8.122923E37F) ;
        p64.y_SET(5.546459E36F) ;
        p64.ax_SET(-8.010972E37F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        p64.covariance_SET(new float[] {9.4902434E36F, -3.1705214E37F, -7.08678E36F, 7.879629E37F, -7.778835E37F, -2.071978E38F, -2.4436935E38F, 3.0825954E37F, -3.0331626E38F, 1.5104571E38F, 1.5170247E38F, 6.197477E37F, 1.1500923E38F, 7.276888E37F, -7.06344E37F, -1.818327E38F, -8.008463E36F, 2.4842717E37F, 1.9829368E38F, 2.372639E38F, 2.7307372E38F, -1.2705483E38F, 2.7900535E38F, -3.4291744E37F, 3.3231475E38F, 4.5620316E37F, -5.561626E37F, -2.2841216E38F, 1.5739326E38F, 2.706278E38F, 2.5168049E38F, -2.0721854E38F, 1.2237067E38F, -1.1608641E38F, -3.2100714E38F, -2.8990129E38F, -8.927458E37F, -1.8058916E38F, 2.7107387E37F, 2.4993683E38F, 8.686685E37F, 1.8629144E38F, -1.873008E38F, -2.4104618E38F, -1.3671662E38F}, 0) ;
        p64.vy_SET(2.4320944E38F) ;
        p64.x_SET(-2.1028206E38F) ;
        p64.vx_SET(1.1566026E37F) ;
        p64.z_SET(7.6164236E37F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan9_raw_GET() == (char)20961);
            assert(pack.chan18_raw_GET() == (char)5971);
            assert(pack.chan1_raw_GET() == (char)6651);
            assert(pack.chan8_raw_GET() == (char)35680);
            assert(pack.rssi_GET() == (char)150);
            assert(pack.chan17_raw_GET() == (char)64461);
            assert(pack.chan4_raw_GET() == (char)22979);
            assert(pack.chan2_raw_GET() == (char)18025);
            assert(pack.chan10_raw_GET() == (char)63721);
            assert(pack.chan16_raw_GET() == (char)62332);
            assert(pack.chan3_raw_GET() == (char)1316);
            assert(pack.chan6_raw_GET() == (char)49584);
            assert(pack.time_boot_ms_GET() == 3147189015L);
            assert(pack.chan15_raw_GET() == (char)17671);
            assert(pack.chan13_raw_GET() == (char)26713);
            assert(pack.chan7_raw_GET() == (char)14084);
            assert(pack.chan14_raw_GET() == (char)44627);
            assert(pack.chan12_raw_GET() == (char)43554);
            assert(pack.chan5_raw_GET() == (char)62253);
            assert(pack.chancount_GET() == (char)90);
            assert(pack.chan11_raw_GET() == (char)37390);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan8_raw_SET((char)35680) ;
        p65.chan13_raw_SET((char)26713) ;
        p65.chan16_raw_SET((char)62332) ;
        p65.chan5_raw_SET((char)62253) ;
        p65.chan1_raw_SET((char)6651) ;
        p65.chancount_SET((char)90) ;
        p65.chan2_raw_SET((char)18025) ;
        p65.chan12_raw_SET((char)43554) ;
        p65.chan18_raw_SET((char)5971) ;
        p65.rssi_SET((char)150) ;
        p65.chan7_raw_SET((char)14084) ;
        p65.chan10_raw_SET((char)63721) ;
        p65.chan4_raw_SET((char)22979) ;
        p65.chan11_raw_SET((char)37390) ;
        p65.chan9_raw_SET((char)20961) ;
        p65.chan6_raw_SET((char)49584) ;
        p65.chan17_raw_SET((char)64461) ;
        p65.chan15_raw_SET((char)17671) ;
        p65.time_boot_ms_SET(3147189015L) ;
        p65.chan3_raw_SET((char)1316) ;
        p65.chan14_raw_SET((char)44627) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)144);
            assert(pack.req_stream_id_GET() == (char)233);
            assert(pack.start_stop_GET() == (char)169);
            assert(pack.target_component_GET() == (char)67);
            assert(pack.req_message_rate_GET() == (char)50561);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_component_SET((char)67) ;
        p66.target_system_SET((char)144) ;
        p66.req_stream_id_SET((char)233) ;
        p66.req_message_rate_SET((char)50561) ;
        p66.start_stop_SET((char)169) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.stream_id_GET() == (char)252);
            assert(pack.on_off_GET() == (char)240);
            assert(pack.message_rate_GET() == (char)47752);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.on_off_SET((char)240) ;
        p67.message_rate_SET((char)47752) ;
        p67.stream_id_SET((char)252) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.buttons_GET() == (char)30345);
            assert(pack.target_GET() == (char)42);
            assert(pack.r_GET() == (short)6072);
            assert(pack.z_GET() == (short)30920);
            assert(pack.x_GET() == (short)3380);
            assert(pack.y_GET() == (short)27723);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.z_SET((short)30920) ;
        p69.r_SET((short)6072) ;
        p69.target_SET((char)42) ;
        p69.y_SET((short)27723) ;
        p69.buttons_SET((char)30345) ;
        p69.x_SET((short)3380) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan7_raw_GET() == (char)23608);
            assert(pack.chan2_raw_GET() == (char)32072);
            assert(pack.chan5_raw_GET() == (char)22238);
            assert(pack.target_system_GET() == (char)95);
            assert(pack.target_component_GET() == (char)14);
            assert(pack.chan4_raw_GET() == (char)40716);
            assert(pack.chan1_raw_GET() == (char)44282);
            assert(pack.chan8_raw_GET() == (char)47814);
            assert(pack.chan3_raw_GET() == (char)4874);
            assert(pack.chan6_raw_GET() == (char)52889);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan3_raw_SET((char)4874) ;
        p70.chan2_raw_SET((char)32072) ;
        p70.chan4_raw_SET((char)40716) ;
        p70.chan5_raw_SET((char)22238) ;
        p70.chan8_raw_SET((char)47814) ;
        p70.target_component_SET((char)14) ;
        p70.target_system_SET((char)95) ;
        p70.chan6_raw_SET((char)52889) ;
        p70.chan7_raw_SET((char)23608) ;
        p70.chan1_raw_SET((char)44282) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.autocontinue_GET() == (char)98);
            assert(pack.seq_GET() == (char)50835);
            assert(pack.x_GET() == -1308990635);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.current_GET() == (char)38);
            assert(pack.target_system_GET() == (char)160);
            assert(pack.z_GET() == -2.1689052E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_CONDITION_GATE);
            assert(pack.param4_GET() == -3.1185312E38F);
            assert(pack.y_GET() == 55816971);
            assert(pack.param3_GET() == -5.6830637E37F);
            assert(pack.target_component_GET() == (char)176);
            assert(pack.param1_GET() == -1.1821796E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.param2_GET() == -2.6374872E38F);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.target_system_SET((char)160) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p73.param3_SET(-5.6830637E37F) ;
        p73.param2_SET(-2.6374872E38F) ;
        p73.param4_SET(-3.1185312E38F) ;
        p73.command_SET(MAV_CMD.MAV_CMD_CONDITION_GATE) ;
        p73.current_SET((char)38) ;
        p73.y_SET(55816971) ;
        p73.seq_SET((char)50835) ;
        p73.target_component_SET((char)176) ;
        p73.z_SET(-2.1689052E38F) ;
        p73.x_SET(-1308990635) ;
        p73.autocontinue_SET((char)98) ;
        p73.param1_SET(-1.1821796E38F) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.groundspeed_GET() == -2.1193323E38F);
            assert(pack.climb_GET() == 1.2216387E38F);
            assert(pack.heading_GET() == (short) -2796);
            assert(pack.throttle_GET() == (char)44087);
            assert(pack.alt_GET() == -1.5656639E38F);
            assert(pack.airspeed_GET() == 2.0333695E37F);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.throttle_SET((char)44087) ;
        p74.groundspeed_SET(-2.1193323E38F) ;
        p74.heading_SET((short) -2796) ;
        p74.airspeed_SET(2.0333695E37F) ;
        p74.climb_SET(1.2216387E38F) ;
        p74.alt_SET(-1.5656639E38F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.current_GET() == (char)73);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_FOLLOW);
            assert(pack.param3_GET() == -3.2150345E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.y_GET() == -1091541492);
            assert(pack.param2_GET() == 1.2182217E38F);
            assert(pack.target_component_GET() == (char)62);
            assert(pack.autocontinue_GET() == (char)8);
            assert(pack.target_system_GET() == (char)213);
            assert(pack.x_GET() == 1673941402);
            assert(pack.param1_GET() == 2.8989989E38F);
            assert(pack.z_GET() == 8.2509005E37F);
            assert(pack.param4_GET() == 1.2158155E38F);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.param2_SET(1.2182217E38F) ;
        p75.autocontinue_SET((char)8) ;
        p75.param4_SET(1.2158155E38F) ;
        p75.target_system_SET((char)213) ;
        p75.y_SET(-1091541492) ;
        p75.command_SET(MAV_CMD.MAV_CMD_DO_FOLLOW) ;
        p75.x_SET(1673941402) ;
        p75.target_component_SET((char)62) ;
        p75.current_SET((char)73) ;
        p75.param3_SET(-3.2150345E38F) ;
        p75.z_SET(8.2509005E37F) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p75.param1_SET(2.8989989E38F) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)184);
            assert(pack.confirmation_GET() == (char)119);
            assert(pack.param1_GET() == -1.0032267E38F);
            assert(pack.param7_GET() == -2.3795962E38F);
            assert(pack.param5_GET() == 1.4197512E38F);
            assert(pack.param2_GET() == 5.326157E37F);
            assert(pack.param3_GET() == 1.8457155E38F);
            assert(pack.param4_GET() == -9.393748E37F);
            assert(pack.param6_GET() == 2.3889297E38F);
            assert(pack.target_component_GET() == (char)212);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_REPEAT_SERVO);
        });
        COMMAND_LONG p76 = new COMMAND_LONG();
        PH.setPack(p76);
        p76.param4_SET(-9.393748E37F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_DO_REPEAT_SERVO) ;
        p76.param2_SET(5.326157E37F) ;
        p76.param3_SET(1.8457155E38F) ;
        p76.param5_SET(1.4197512E38F) ;
        p76.param7_SET(-2.3795962E38F) ;
        p76.target_system_SET((char)184) ;
        p76.confirmation_SET((char)119) ;
        p76.target_component_SET((char)212) ;
        p76.param1_SET(-1.0032267E38F) ;
        p76.param6_SET(2.3889297E38F) ;
        TestChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.result_param2_TRY(ph) == 945768250);
            assert(pack.target_component_TRY(ph) == (char)4);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_ACCEPTED);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_REPEAT_RELAY);
            assert(pack.progress_TRY(ph) == (char)44);
            assert(pack.target_system_TRY(ph) == (char)211);
        });
        COMMAND_ACK p77 = new COMMAND_ACK();
        PH.setPack(p77);
        p77.result_param2_SET(945768250, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_DO_REPEAT_RELAY) ;
        p77.progress_SET((char)44, PH) ;
        p77.target_system_SET((char)211, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_ACCEPTED) ;
        p77.target_component_SET((char)4, PH) ;
        TestChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.mode_switch_GET() == (char)229);
            assert(pack.yaw_GET() == -1.0862329E38F);
            assert(pack.roll_GET() == 1.3892397E38F);
            assert(pack.time_boot_ms_GET() == 3463198141L);
            assert(pack.thrust_GET() == 3.0448785E38F);
            assert(pack.manual_override_switch_GET() == (char)200);
            assert(pack.pitch_GET() == 1.8741478E37F);
        });
        MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.roll_SET(1.3892397E38F) ;
        p81.manual_override_switch_SET((char)200) ;
        p81.yaw_SET(-1.0862329E38F) ;
        p81.pitch_SET(1.8741478E37F) ;
        p81.time_boot_ms_SET(3463198141L) ;
        p81.thrust_SET(3.0448785E38F) ;
        p81.mode_switch_SET((char)229) ;
        TestChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.thrust_GET() == -2.20025E38F);
            assert(pack.target_system_GET() == (char)255);
            assert(pack.time_boot_ms_GET() == 979690890L);
            assert(pack.body_yaw_rate_GET() == -1.9545128E38F);
            assert(pack.type_mask_GET() == (char)77);
            assert(pack.target_component_GET() == (char)190);
            assert(pack.body_roll_rate_GET() == 1.2388615E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.264345E38F, -3.3823798E38F, 1.6803232E38F, 1.6998821E38F}));
            assert(pack.body_pitch_rate_GET() == -2.5354576E38F);
        });
        SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.type_mask_SET((char)77) ;
        p82.body_pitch_rate_SET(-2.5354576E38F) ;
        p82.body_roll_rate_SET(1.2388615E38F) ;
        p82.time_boot_ms_SET(979690890L) ;
        p82.thrust_SET(-2.20025E38F) ;
        p82.q_SET(new float[] {-1.264345E38F, -3.3823798E38F, 1.6803232E38F, 1.6998821E38F}, 0) ;
        p82.target_system_SET((char)255) ;
        p82.target_component_SET((char)190) ;
        p82.body_yaw_rate_SET(-1.9545128E38F) ;
        TestChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)66);
            assert(pack.body_roll_rate_GET() == 5.3757386E37F);
            assert(pack.time_boot_ms_GET() == 1592312413L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.8275054E38F, -9.083192E37F, -1.5051397E38F, 3.1666445E38F}));
            assert(pack.thrust_GET() == 1.0153713E38F);
            assert(pack.body_pitch_rate_GET() == -1.2034884E38F);
            assert(pack.body_yaw_rate_GET() == 2.7665294E38F);
        });
        ATTITUDE_TARGET p83 = new ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.body_roll_rate_SET(5.3757386E37F) ;
        p83.type_mask_SET((char)66) ;
        p83.body_yaw_rate_SET(2.7665294E38F) ;
        p83.thrust_SET(1.0153713E38F) ;
        p83.body_pitch_rate_SET(-1.2034884E38F) ;
        p83.q_SET(new float[] {-2.8275054E38F, -9.083192E37F, -1.5051397E38F, 3.1666445E38F}, 0) ;
        p83.time_boot_ms_SET(1592312413L) ;
        TestChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.afx_GET() == 1.1308968E38F);
            assert(pack.time_boot_ms_GET() == 3281160261L);
            assert(pack.afz_GET() == 4.74753E37F);
            assert(pack.y_GET() == 4.2749316E37F);
            assert(pack.target_component_GET() == (char)233);
            assert(pack.vx_GET() == 1.6575185E38F);
            assert(pack.yaw_rate_GET() == 3.3340604E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.yaw_GET() == -2.2509201E38F);
            assert(pack.z_GET() == -7.865325E37F);
            assert(pack.vz_GET() == 9.680953E37F);
            assert(pack.target_system_GET() == (char)123);
            assert(pack.x_GET() == -1.3037528E38F);
            assert(pack.afy_GET() == 5.989196E36F);
            assert(pack.type_mask_GET() == (char)43098);
            assert(pack.vy_GET() == 2.854739E38F);
        });
        SET_POSITION_TARGET_LOCAL_NED p84 = new SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.target_component_SET((char)233) ;
        p84.vx_SET(1.6575185E38F) ;
        p84.yaw_SET(-2.2509201E38F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p84.afy_SET(5.989196E36F) ;
        p84.x_SET(-1.3037528E38F) ;
        p84.afz_SET(4.74753E37F) ;
        p84.target_system_SET((char)123) ;
        p84.time_boot_ms_SET(3281160261L) ;
        p84.type_mask_SET((char)43098) ;
        p84.yaw_rate_SET(3.3340604E38F) ;
        p84.y_SET(4.2749316E37F) ;
        p84.z_SET(-7.865325E37F) ;
        p84.vy_SET(2.854739E38F) ;
        p84.afx_SET(1.1308968E38F) ;
        p84.vz_SET(9.680953E37F) ;
        TestChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)188);
            assert(pack.vz_GET() == -2.911509E38F);
            assert(pack.afy_GET() == 6.684728E37F);
            assert(pack.vx_GET() == 1.9331008E38F);
            assert(pack.vy_GET() == -2.2371129E38F);
            assert(pack.type_mask_GET() == (char)32563);
            assert(pack.yaw_rate_GET() == -4.3150598E36F);
            assert(pack.lat_int_GET() == 1344840983);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.yaw_GET() == 8.619764E37F);
            assert(pack.alt_GET() == 1.5508203E38F);
            assert(pack.afx_GET() == -3.535654E37F);
            assert(pack.time_boot_ms_GET() == 1849557445L);
            assert(pack.target_system_GET() == (char)170);
            assert(pack.lon_int_GET() == 544017078);
            assert(pack.afz_GET() == -1.4481851E38F);
        });
        SET_POSITION_TARGET_GLOBAL_INT p86 = new SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.alt_SET(1.5508203E38F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p86.lon_int_SET(544017078) ;
        p86.vx_SET(1.9331008E38F) ;
        p86.time_boot_ms_SET(1849557445L) ;
        p86.yaw_rate_SET(-4.3150598E36F) ;
        p86.yaw_SET(8.619764E37F) ;
        p86.afz_SET(-1.4481851E38F) ;
        p86.type_mask_SET((char)32563) ;
        p86.vy_SET(-2.2371129E38F) ;
        p86.afx_SET(-3.535654E37F) ;
        p86.afy_SET(6.684728E37F) ;
        p86.target_component_SET((char)188) ;
        p86.vz_SET(-2.911509E38F) ;
        p86.target_system_SET((char)170) ;
        p86.lat_int_SET(1344840983) ;
        TestChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == -6.0399102E35F);
            assert(pack.time_boot_ms_GET() == 2513376248L);
            assert(pack.lat_int_GET() == 1246764491);
            assert(pack.yaw_rate_GET() == 9.440905E37F);
            assert(pack.afx_GET() == 1.1481213E38F);
            assert(pack.vx_GET() == -1.1352255E37F);
            assert(pack.alt_GET() == 2.854411E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.vy_GET() == -1.5659289E38F);
            assert(pack.yaw_GET() == 2.8837502E38F);
            assert(pack.afy_GET() == 7.7068304E37F);
            assert(pack.lon_int_GET() == -2029773390);
            assert(pack.afz_GET() == 4.2531944E37F);
            assert(pack.type_mask_GET() == (char)41914);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.alt_SET(2.854411E38F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p87.lon_int_SET(-2029773390) ;
        p87.vy_SET(-1.5659289E38F) ;
        p87.yaw_SET(2.8837502E38F) ;
        p87.afz_SET(4.2531944E37F) ;
        p87.vx_SET(-1.1352255E37F) ;
        p87.yaw_rate_SET(9.440905E37F) ;
        p87.time_boot_ms_SET(2513376248L) ;
        p87.afx_SET(1.1481213E38F) ;
        p87.lat_int_SET(1246764491) ;
        p87.afy_SET(7.7068304E37F) ;
        p87.type_mask_SET((char)41914) ;
        p87.vz_SET(-6.0399102E35F) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -1.0142311E38F);
            assert(pack.z_GET() == -3.3846768E38F);
            assert(pack.y_GET() == -2.058351E38F);
            assert(pack.time_boot_ms_GET() == 4292351607L);
            assert(pack.yaw_GET() == -2.9444398E38F);
            assert(pack.x_GET() == -1.2130189E38F);
            assert(pack.pitch_GET() == 1.7895716E38F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.time_boot_ms_SET(4292351607L) ;
        p89.roll_SET(-1.0142311E38F) ;
        p89.z_SET(-3.3846768E38F) ;
        p89.pitch_SET(1.7895716E38F) ;
        p89.yaw_SET(-2.9444398E38F) ;
        p89.y_SET(-2.058351E38F) ;
        p89.x_SET(-1.2130189E38F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 352649607);
            assert(pack.vx_GET() == (short)4696);
            assert(pack.roll_GET() == -3.0364327E38F);
            assert(pack.rollspeed_GET() == -1.197666E38F);
            assert(pack.yawspeed_GET() == 1.0002846E38F);
            assert(pack.vy_GET() == (short)19160);
            assert(pack.yaw_GET() == 9.677567E37F);
            assert(pack.vz_GET() == (short)28546);
            assert(pack.lat_GET() == 1099971216);
            assert(pack.xacc_GET() == (short)13623);
            assert(pack.yacc_GET() == (short)29599);
            assert(pack.lon_GET() == 1376094277);
            assert(pack.pitchspeed_GET() == 2.0607715E38F);
            assert(pack.zacc_GET() == (short) -14175);
            assert(pack.time_usec_GET() == 4239146476171690300L);
            assert(pack.pitch_GET() == 9.744733E37F);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.pitch_SET(9.744733E37F) ;
        p90.yacc_SET((short)29599) ;
        p90.vz_SET((short)28546) ;
        p90.yaw_SET(9.677567E37F) ;
        p90.alt_SET(352649607) ;
        p90.vy_SET((short)19160) ;
        p90.zacc_SET((short) -14175) ;
        p90.yawspeed_SET(1.0002846E38F) ;
        p90.pitchspeed_SET(2.0607715E38F) ;
        p90.rollspeed_SET(-1.197666E38F) ;
        p90.vx_SET((short)4696) ;
        p90.xacc_SET((short)13623) ;
        p90.time_usec_SET(4239146476171690300L) ;
        p90.roll_SET(-3.0364327E38F) ;
        p90.lon_SET(1376094277) ;
        p90.lat_SET(1099971216) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.aux1_GET() == -2.4995737E38F);
            assert(pack.aux4_GET() == -1.8262523E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_AUTO_DISARMED);
            assert(pack.pitch_elevator_GET() == 2.992688E38F);
            assert(pack.yaw_rudder_GET() == -3.2653325E38F);
            assert(pack.aux2_GET() == 1.5015126E38F);
            assert(pack.throttle_GET() == -1.3165813E38F);
            assert(pack.roll_ailerons_GET() == -2.2899873E38F);
            assert(pack.nav_mode_GET() == (char)23);
            assert(pack.aux3_GET() == 2.4596809E38F);
            assert(pack.time_usec_GET() == 951310939620657769L);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.aux1_SET(-2.4995737E38F) ;
        p91.time_usec_SET(951310939620657769L) ;
        p91.roll_ailerons_SET(-2.2899873E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_AUTO_DISARMED) ;
        p91.aux4_SET(-1.8262523E38F) ;
        p91.nav_mode_SET((char)23) ;
        p91.throttle_SET(-1.3165813E38F) ;
        p91.pitch_elevator_SET(2.992688E38F) ;
        p91.aux2_SET(1.5015126E38F) ;
        p91.aux3_SET(2.4596809E38F) ;
        p91.yaw_rudder_SET(-3.2653325E38F) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan5_raw_GET() == (char)41525);
            assert(pack.chan1_raw_GET() == (char)29531);
            assert(pack.chan6_raw_GET() == (char)15966);
            assert(pack.chan4_raw_GET() == (char)42784);
            assert(pack.time_usec_GET() == 991535857129907884L);
            assert(pack.chan7_raw_GET() == (char)52847);
            assert(pack.chan3_raw_GET() == (char)25613);
            assert(pack.chan12_raw_GET() == (char)25723);
            assert(pack.chan11_raw_GET() == (char)3244);
            assert(pack.chan2_raw_GET() == (char)52507);
            assert(pack.chan10_raw_GET() == (char)15580);
            assert(pack.rssi_GET() == (char)170);
            assert(pack.chan8_raw_GET() == (char)49030);
            assert(pack.chan9_raw_GET() == (char)38662);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan3_raw_SET((char)25613) ;
        p92.time_usec_SET(991535857129907884L) ;
        p92.chan1_raw_SET((char)29531) ;
        p92.chan11_raw_SET((char)3244) ;
        p92.chan8_raw_SET((char)49030) ;
        p92.rssi_SET((char)170) ;
        p92.chan9_raw_SET((char)38662) ;
        p92.chan4_raw_SET((char)42784) ;
        p92.chan6_raw_SET((char)15966) ;
        p92.chan2_raw_SET((char)52507) ;
        p92.chan5_raw_SET((char)41525) ;
        p92.chan7_raw_SET((char)52847) ;
        p92.chan10_raw_SET((char)15580) ;
        p92.chan12_raw_SET((char)25723) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
            assert(pack.flags_GET() == 7971136432317851336L);
            assert(pack.time_usec_GET() == 5166957201425147625L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-3.3461522E38F, -2.5209978E37F, 1.7473387E38F, -2.2249568E38F, -1.1601712E38F, 1.1881024E38F, 1.571799E38F, 2.1579194E38F, 6.25235E37F, -2.4687593E38F, 1.7045035E38F, -2.8350284E38F, -5.060853E37F, -1.6886123E38F, 2.822712E38F, -2.4217958E38F}));
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.controls_SET(new float[] {-3.3461522E38F, -2.5209978E37F, 1.7473387E38F, -2.2249568E38F, -1.1601712E38F, 1.1881024E38F, 1.571799E38F, 2.1579194E38F, 6.25235E37F, -2.4687593E38F, 1.7045035E38F, -2.8350284E38F, -5.060853E37F, -1.6886123E38F, 2.822712E38F, -2.4217958E38F}, 0) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_STABILIZE_DISARMED) ;
        p93.time_usec_SET(5166957201425147625L) ;
        p93.flags_SET(7971136432317851336L) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.quality_GET() == (char)14);
            assert(pack.ground_distance_GET() == -5.669474E37F);
            assert(pack.flow_comp_m_y_GET() == -1.0238152E38F);
            assert(pack.flow_rate_x_TRY(ph) == 1.1149292E38F);
            assert(pack.flow_rate_y_TRY(ph) == 2.65803E38F);
            assert(pack.flow_comp_m_x_GET() == -5.861733E37F);
            assert(pack.sensor_id_GET() == (char)255);
            assert(pack.flow_x_GET() == (short)4619);
            assert(pack.time_usec_GET() == 6982661722659161285L);
            assert(pack.flow_y_GET() == (short)6069);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.flow_comp_m_x_SET(-5.861733E37F) ;
        p100.flow_comp_m_y_SET(-1.0238152E38F) ;
        p100.flow_x_SET((short)4619) ;
        p100.flow_rate_y_SET(2.65803E38F, PH) ;
        p100.flow_rate_x_SET(1.1149292E38F, PH) ;
        p100.flow_y_SET((short)6069) ;
        p100.time_usec_SET(6982661722659161285L) ;
        p100.sensor_id_SET((char)255) ;
        p100.ground_distance_SET(-5.669474E37F) ;
        p100.quality_SET((char)14) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 3976812401556256096L);
            assert(pack.x_GET() == 2.6656548E38F);
            assert(pack.pitch_GET() == -9.530939E37F);
            assert(pack.y_GET() == 2.5440888E38F);
            assert(pack.z_GET() == -2.4862441E38F);
            assert(pack.roll_GET() == -1.7300966E38F);
            assert(pack.yaw_GET() == -2.0290936E38F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.x_SET(2.6656548E38F) ;
        p101.usec_SET(3976812401556256096L) ;
        p101.z_SET(-2.4862441E38F) ;
        p101.y_SET(2.5440888E38F) ;
        p101.roll_SET(-1.7300966E38F) ;
        p101.pitch_SET(-9.530939E37F) ;
        p101.yaw_SET(-2.0290936E38F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -1.2348913E38F);
            assert(pack.pitch_GET() == 5.480784E37F);
            assert(pack.y_GET() == -2.3760675E38F);
            assert(pack.roll_GET() == 1.2309018E38F);
            assert(pack.yaw_GET() == 3.1335175E38F);
            assert(pack.x_GET() == -1.4465491E38F);
            assert(pack.usec_GET() == 5934704068396325091L);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.roll_SET(1.2309018E38F) ;
        p102.pitch_SET(5.480784E37F) ;
        p102.z_SET(-1.2348913E38F) ;
        p102.yaw_SET(3.1335175E38F) ;
        p102.y_SET(-2.3760675E38F) ;
        p102.x_SET(-1.4465491E38F) ;
        p102.usec_SET(5934704068396325091L) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 6441072308472074537L);
            assert(pack.z_GET() == 2.5651234E38F);
            assert(pack.y_GET() == -2.148506E38F);
            assert(pack.x_GET() == 3.35943E38F);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.z_SET(2.5651234E38F) ;
        p103.usec_SET(6441072308472074537L) ;
        p103.y_SET(-2.148506E38F) ;
        p103.x_SET(3.35943E38F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -2.1921697E38F);
            assert(pack.x_GET() == 2.9323243E38F);
            assert(pack.usec_GET() == 6495486211930250911L);
            assert(pack.yaw_GET() == 1.8017806E38F);
            assert(pack.roll_GET() == -1.0436405E38F);
            assert(pack.z_GET() == 1.1047057E38F);
            assert(pack.pitch_GET() == -2.852337E38F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.roll_SET(-1.0436405E38F) ;
        p104.x_SET(2.9323243E38F) ;
        p104.usec_SET(6495486211930250911L) ;
        p104.y_SET(-2.1921697E38F) ;
        p104.z_SET(1.1047057E38F) ;
        p104.yaw_SET(1.8017806E38F) ;
        p104.pitch_SET(-2.852337E38F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.ymag_GET() == 2.940577E37F);
            assert(pack.yacc_GET() == -3.0963968E38F);
            assert(pack.temperature_GET() == 3.142865E38F);
            assert(pack.zgyro_GET() == -2.1641514E38F);
            assert(pack.abs_pressure_GET() == -2.3884058E38F);
            assert(pack.fields_updated_GET() == (char)18707);
            assert(pack.diff_pressure_GET() == 1.8233756E38F);
            assert(pack.ygyro_GET() == -2.559916E37F);
            assert(pack.xacc_GET() == 1.1797528E38F);
            assert(pack.xmag_GET() == 2.5764354E37F);
            assert(pack.pressure_alt_GET() == 2.7791858E37F);
            assert(pack.time_usec_GET() == 2927368427626400121L);
            assert(pack.zacc_GET() == 3.2855611E38F);
            assert(pack.zmag_GET() == 1.7342446E38F);
            assert(pack.xgyro_GET() == -9.606916E37F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.fields_updated_SET((char)18707) ;
        p105.pressure_alt_SET(2.7791858E37F) ;
        p105.xgyro_SET(-9.606916E37F) ;
        p105.zgyro_SET(-2.1641514E38F) ;
        p105.diff_pressure_SET(1.8233756E38F) ;
        p105.ygyro_SET(-2.559916E37F) ;
        p105.xmag_SET(2.5764354E37F) ;
        p105.abs_pressure_SET(-2.3884058E38F) ;
        p105.zacc_SET(3.2855611E38F) ;
        p105.ymag_SET(2.940577E37F) ;
        p105.temperature_SET(3.142865E38F) ;
        p105.zmag_SET(1.7342446E38F) ;
        p105.yacc_SET(-3.0963968E38F) ;
        p105.xacc_SET(1.1797528E38F) ;
        p105.time_usec_SET(2927368427626400121L) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integrated_zgyro_GET() == 9.179527E37F);
            assert(pack.integrated_y_GET() == -3.0070135E38F);
            assert(pack.integrated_xgyro_GET() == -3.468967E37F);
            assert(pack.integrated_ygyro_GET() == -8.2679803E37F);
            assert(pack.temperature_GET() == (short) -13910);
            assert(pack.time_usec_GET() == 971724160377900705L);
            assert(pack.integration_time_us_GET() == 2920589574L);
            assert(pack.quality_GET() == (char)157);
            assert(pack.sensor_id_GET() == (char)55);
            assert(pack.distance_GET() == -7.5186304E37F);
            assert(pack.time_delta_distance_us_GET() == 132831013L);
            assert(pack.integrated_x_GET() == -1.848204E38F);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integrated_ygyro_SET(-8.2679803E37F) ;
        p106.integrated_y_SET(-3.0070135E38F) ;
        p106.integrated_zgyro_SET(9.179527E37F) ;
        p106.distance_SET(-7.5186304E37F) ;
        p106.time_delta_distance_us_SET(132831013L) ;
        p106.sensor_id_SET((char)55) ;
        p106.time_usec_SET(971724160377900705L) ;
        p106.integrated_xgyro_SET(-3.468967E37F) ;
        p106.integration_time_us_SET(2920589574L) ;
        p106.quality_SET((char)157) ;
        p106.integrated_x_SET(-1.848204E38F) ;
        p106.temperature_SET((short) -13910) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == 1.8815912E38F);
            assert(pack.zmag_GET() == 2.141126E38F);
            assert(pack.ymag_GET() == -1.9619118E37F);
            assert(pack.ygyro_GET() == -1.5539174E38F);
            assert(pack.pressure_alt_GET() == 8.616412E37F);
            assert(pack.yacc_GET() == -8.820104E37F);
            assert(pack.fields_updated_GET() == 3895478214L);
            assert(pack.zgyro_GET() == -6.4782645E37F);
            assert(pack.xmag_GET() == 5.561025E37F);
            assert(pack.time_usec_GET() == 262007647736335157L);
            assert(pack.abs_pressure_GET() == 8.1304606E36F);
            assert(pack.diff_pressure_GET() == 4.2042522E37F);
            assert(pack.temperature_GET() == 2.2825465E38F);
            assert(pack.xgyro_GET() == -1.0185471E38F);
            assert(pack.zacc_GET() == 3.0661876E38F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.zmag_SET(2.141126E38F) ;
        p107.pressure_alt_SET(8.616412E37F) ;
        p107.yacc_SET(-8.820104E37F) ;
        p107.xgyro_SET(-1.0185471E38F) ;
        p107.temperature_SET(2.2825465E38F) ;
        p107.diff_pressure_SET(4.2042522E37F) ;
        p107.xmag_SET(5.561025E37F) ;
        p107.zacc_SET(3.0661876E38F) ;
        p107.fields_updated_SET(3895478214L) ;
        p107.xacc_SET(1.8815912E38F) ;
        p107.time_usec_SET(262007647736335157L) ;
        p107.zgyro_SET(-6.4782645E37F) ;
        p107.abs_pressure_SET(8.1304606E36F) ;
        p107.ymag_SET(-1.9619118E37F) ;
        p107.ygyro_SET(-1.5539174E38F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.std_dev_horz_GET() == 2.4826592E38F);
            assert(pack.vd_GET() == -4.2943064E37F);
            assert(pack.vn_GET() == 2.7866877E38F);
            assert(pack.xacc_GET() == -2.918061E38F);
            assert(pack.xgyro_GET() == 1.5282361E38F);
            assert(pack.lat_GET() == -9.206358E37F);
            assert(pack.ygyro_GET() == -1.9713782E38F);
            assert(pack.zacc_GET() == -2.1468856E38F);
            assert(pack.q3_GET() == -2.0001105E38F);
            assert(pack.lon_GET() == -1.0089991E38F);
            assert(pack.roll_GET() == -3.1078146E38F);
            assert(pack.ve_GET() == 3.201447E38F);
            assert(pack.q1_GET() == 1.9204688E38F);
            assert(pack.q2_GET() == 3.3253524E38F);
            assert(pack.std_dev_vert_GET() == -4.6904345E37F);
            assert(pack.zgyro_GET() == 2.8738613E38F);
            assert(pack.yacc_GET() == 1.9301505E37F);
            assert(pack.yaw_GET() == -2.2805152E38F);
            assert(pack.q4_GET() == 6.771341E37F);
            assert(pack.alt_GET() == -7.682443E37F);
            assert(pack.pitch_GET() == -2.0094897E38F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.q1_SET(1.9204688E38F) ;
        p108.ygyro_SET(-1.9713782E38F) ;
        p108.q4_SET(6.771341E37F) ;
        p108.vd_SET(-4.2943064E37F) ;
        p108.zgyro_SET(2.8738613E38F) ;
        p108.q3_SET(-2.0001105E38F) ;
        p108.vn_SET(2.7866877E38F) ;
        p108.pitch_SET(-2.0094897E38F) ;
        p108.std_dev_horz_SET(2.4826592E38F) ;
        p108.q2_SET(3.3253524E38F) ;
        p108.yacc_SET(1.9301505E37F) ;
        p108.ve_SET(3.201447E38F) ;
        p108.std_dev_vert_SET(-4.6904345E37F) ;
        p108.lat_SET(-9.206358E37F) ;
        p108.alt_SET(-7.682443E37F) ;
        p108.yaw_SET(-2.2805152E38F) ;
        p108.roll_SET(-3.1078146E38F) ;
        p108.zacc_SET(-2.1468856E38F) ;
        p108.xacc_SET(-2.918061E38F) ;
        p108.xgyro_SET(1.5282361E38F) ;
        p108.lon_SET(-1.0089991E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.noise_GET() == (char)220);
            assert(pack.remrssi_GET() == (char)134);
            assert(pack.rxerrors_GET() == (char)2012);
            assert(pack.remnoise_GET() == (char)204);
            assert(pack.rssi_GET() == (char)79);
            assert(pack.txbuf_GET() == (char)215);
            assert(pack.fixed__GET() == (char)58570);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)79) ;
        p109.remnoise_SET((char)204) ;
        p109.remrssi_SET((char)134) ;
        p109.fixed__SET((char)58570) ;
        p109.rxerrors_SET((char)2012) ;
        p109.noise_SET((char)220) ;
        p109.txbuf_SET((char)215) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)64);
            assert(pack.target_system_GET() == (char)165);
            assert(pack.target_network_GET() == (char)137);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)10, (char)83, (char)28, (char)75, (char)65, (char)28, (char)168, (char)186, (char)8, (char)53, (char)34, (char)197, (char)68, (char)99, (char)186, (char)1, (char)244, (char)239, (char)29, (char)181, (char)165, (char)89, (char)239, (char)170, (char)66, (char)64, (char)26, (char)69, (char)35, (char)25, (char)78, (char)13, (char)134, (char)75, (char)91, (char)27, (char)232, (char)4, (char)111, (char)194, (char)58, (char)79, (char)103, (char)209, (char)238, (char)165, (char)153, (char)38, (char)136, (char)94, (char)162, (char)213, (char)221, (char)42, (char)61, (char)103, (char)172, (char)156, (char)144, (char)142, (char)114, (char)194, (char)246, (char)226, (char)192, (char)98, (char)63, (char)35, (char)100, (char)27, (char)208, (char)229, (char)163, (char)139, (char)214, (char)58, (char)194, (char)35, (char)117, (char)213, (char)62, (char)23, (char)175, (char)231, (char)204, (char)32, (char)199, (char)101, (char)215, (char)34, (char)76, (char)131, (char)95, (char)221, (char)30, (char)100, (char)168, (char)197, (char)226, (char)26, (char)212, (char)32, (char)75, (char)222, (char)90, (char)66, (char)255, (char)187, (char)138, (char)121, (char)131, (char)247, (char)8, (char)41, (char)176, (char)88, (char)31, (char)32, (char)94, (char)75, (char)205, (char)245, (char)252, (char)238, (char)195, (char)161, (char)159, (char)116, (char)54, (char)111, (char)200, (char)168, (char)154, (char)65, (char)42, (char)127, (char)35, (char)135, (char)196, (char)199, (char)73, (char)255, (char)210, (char)123, (char)213, (char)32, (char)23, (char)247, (char)222, (char)225, (char)175, (char)69, (char)162, (char)115, (char)186, (char)139, (char)150, (char)45, (char)194, (char)23, (char)132, (char)55, (char)246, (char)63, (char)40, (char)105, (char)230, (char)63, (char)223, (char)149, (char)132, (char)230, (char)44, (char)41, (char)217, (char)219, (char)110, (char)100, (char)163, (char)187, (char)0, (char)66, (char)117, (char)208, (char)124, (char)36, (char)50, (char)132, (char)179, (char)116, (char)128, (char)5, (char)216, (char)55, (char)79, (char)50, (char)62, (char)234, (char)208, (char)186, (char)118, (char)49, (char)120, (char)39, (char)162, (char)7, (char)117, (char)78, (char)45, (char)136, (char)146, (char)106, (char)90, (char)104, (char)78, (char)10, (char)90, (char)10, (char)96, (char)126, (char)23, (char)140, (char)82, (char)95, (char)137, (char)105, (char)231, (char)111, (char)254, (char)209, (char)175, (char)226, (char)156, (char)45, (char)91, (char)163, (char)118, (char)110, (char)82, (char)8, (char)58, (char)111, (char)196, (char)28, (char)88, (char)188, (char)238, (char)138, (char)225, (char)88, (char)177}));
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.payload_SET(new char[] {(char)10, (char)83, (char)28, (char)75, (char)65, (char)28, (char)168, (char)186, (char)8, (char)53, (char)34, (char)197, (char)68, (char)99, (char)186, (char)1, (char)244, (char)239, (char)29, (char)181, (char)165, (char)89, (char)239, (char)170, (char)66, (char)64, (char)26, (char)69, (char)35, (char)25, (char)78, (char)13, (char)134, (char)75, (char)91, (char)27, (char)232, (char)4, (char)111, (char)194, (char)58, (char)79, (char)103, (char)209, (char)238, (char)165, (char)153, (char)38, (char)136, (char)94, (char)162, (char)213, (char)221, (char)42, (char)61, (char)103, (char)172, (char)156, (char)144, (char)142, (char)114, (char)194, (char)246, (char)226, (char)192, (char)98, (char)63, (char)35, (char)100, (char)27, (char)208, (char)229, (char)163, (char)139, (char)214, (char)58, (char)194, (char)35, (char)117, (char)213, (char)62, (char)23, (char)175, (char)231, (char)204, (char)32, (char)199, (char)101, (char)215, (char)34, (char)76, (char)131, (char)95, (char)221, (char)30, (char)100, (char)168, (char)197, (char)226, (char)26, (char)212, (char)32, (char)75, (char)222, (char)90, (char)66, (char)255, (char)187, (char)138, (char)121, (char)131, (char)247, (char)8, (char)41, (char)176, (char)88, (char)31, (char)32, (char)94, (char)75, (char)205, (char)245, (char)252, (char)238, (char)195, (char)161, (char)159, (char)116, (char)54, (char)111, (char)200, (char)168, (char)154, (char)65, (char)42, (char)127, (char)35, (char)135, (char)196, (char)199, (char)73, (char)255, (char)210, (char)123, (char)213, (char)32, (char)23, (char)247, (char)222, (char)225, (char)175, (char)69, (char)162, (char)115, (char)186, (char)139, (char)150, (char)45, (char)194, (char)23, (char)132, (char)55, (char)246, (char)63, (char)40, (char)105, (char)230, (char)63, (char)223, (char)149, (char)132, (char)230, (char)44, (char)41, (char)217, (char)219, (char)110, (char)100, (char)163, (char)187, (char)0, (char)66, (char)117, (char)208, (char)124, (char)36, (char)50, (char)132, (char)179, (char)116, (char)128, (char)5, (char)216, (char)55, (char)79, (char)50, (char)62, (char)234, (char)208, (char)186, (char)118, (char)49, (char)120, (char)39, (char)162, (char)7, (char)117, (char)78, (char)45, (char)136, (char)146, (char)106, (char)90, (char)104, (char)78, (char)10, (char)90, (char)10, (char)96, (char)126, (char)23, (char)140, (char)82, (char)95, (char)137, (char)105, (char)231, (char)111, (char)254, (char)209, (char)175, (char)226, (char)156, (char)45, (char)91, (char)163, (char)118, (char)110, (char)82, (char)8, (char)58, (char)111, (char)196, (char)28, (char)88, (char)188, (char)238, (char)138, (char)225, (char)88, (char)177}, 0) ;
        p110.target_network_SET((char)137) ;
        p110.target_component_SET((char)64) ;
        p110.target_system_SET((char)165) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.ts1_GET() == 499370985469555565L);
            assert(pack.tc1_GET() == -7393292873780282413L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(-7393292873780282413L) ;
        p111.ts1_SET(499370985469555565L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 2365368459001877091L);
            assert(pack.seq_GET() == 3031502704L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(2365368459001877091L) ;
        p112.seq_SET(3031502704L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 1982217501);
            assert(pack.epv_GET() == (char)15905);
            assert(pack.ve_GET() == (short)16706);
            assert(pack.lon_GET() == -795614356);
            assert(pack.vd_GET() == (short)16581);
            assert(pack.eph_GET() == (char)7542);
            assert(pack.fix_type_GET() == (char)6);
            assert(pack.time_usec_GET() == 4888905835924353039L);
            assert(pack.lat_GET() == 408032526);
            assert(pack.vn_GET() == (short)3053);
            assert(pack.vel_GET() == (char)45364);
            assert(pack.cog_GET() == (char)54118);
            assert(pack.satellites_visible_GET() == (char)228);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.lat_SET(408032526) ;
        p113.ve_SET((short)16706) ;
        p113.fix_type_SET((char)6) ;
        p113.epv_SET((char)15905) ;
        p113.vd_SET((short)16581) ;
        p113.satellites_visible_SET((char)228) ;
        p113.vn_SET((short)3053) ;
        p113.cog_SET((char)54118) ;
        p113.alt_SET(1982217501) ;
        p113.time_usec_SET(4888905835924353039L) ;
        p113.lon_SET(-795614356) ;
        p113.eph_SET((char)7542) ;
        p113.vel_SET((char)45364) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.distance_GET() == -2.0536088E38F);
            assert(pack.integrated_zgyro_GET() == 2.8927203E38F);
            assert(pack.integrated_xgyro_GET() == 1.9855656E38F);
            assert(pack.temperature_GET() == (short) -15250);
            assert(pack.integrated_x_GET() == -2.8781263E38F);
            assert(pack.quality_GET() == (char)27);
            assert(pack.sensor_id_GET() == (char)73);
            assert(pack.integrated_y_GET() == 8.60013E37F);
            assert(pack.time_usec_GET() == 4551214819544126319L);
            assert(pack.time_delta_distance_us_GET() == 2283756953L);
            assert(pack.integrated_ygyro_GET() == -1.0679429E38F);
            assert(pack.integration_time_us_GET() == 312666149L);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.temperature_SET((short) -15250) ;
        p114.time_usec_SET(4551214819544126319L) ;
        p114.integrated_zgyro_SET(2.8927203E38F) ;
        p114.integration_time_us_SET(312666149L) ;
        p114.quality_SET((char)27) ;
        p114.time_delta_distance_us_SET(2283756953L) ;
        p114.distance_SET(-2.0536088E38F) ;
        p114.integrated_xgyro_SET(1.9855656E38F) ;
        p114.integrated_y_SET(8.60013E37F) ;
        p114.integrated_ygyro_SET(-1.0679429E38F) ;
        p114.integrated_x_SET(-2.8781263E38F) ;
        p114.sensor_id_SET((char)73) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 1940275675);
            assert(pack.zacc_GET() == (short) -2879);
            assert(pack.pitchspeed_GET() == -1.6687119E38F);
            assert(pack.yawspeed_GET() == -7.093365E37F);
            assert(pack.alt_GET() == 2037942640);
            assert(pack.vx_GET() == (short) -4119);
            assert(pack.yacc_GET() == (short)23579);
            assert(pack.ind_airspeed_GET() == (char)24861);
            assert(pack.xacc_GET() == (short) -12924);
            assert(pack.vz_GET() == (short)30510);
            assert(pack.true_airspeed_GET() == (char)20213);
            assert(pack.time_usec_GET() == 719642884591014326L);
            assert(pack.lat_GET() == 270442379);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {1.5139641E37F, -2.420372E38F, 3.360659E38F, -1.624399E38F}));
            assert(pack.rollspeed_GET() == -2.118701E38F);
            assert(pack.vy_GET() == (short)6625);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.lat_SET(270442379) ;
        p115.time_usec_SET(719642884591014326L) ;
        p115.vz_SET((short)30510) ;
        p115.lon_SET(1940275675) ;
        p115.vy_SET((short)6625) ;
        p115.rollspeed_SET(-2.118701E38F) ;
        p115.attitude_quaternion_SET(new float[] {1.5139641E37F, -2.420372E38F, 3.360659E38F, -1.624399E38F}, 0) ;
        p115.yawspeed_SET(-7.093365E37F) ;
        p115.true_airspeed_SET((char)20213) ;
        p115.vx_SET((short) -4119) ;
        p115.alt_SET(2037942640) ;
        p115.yacc_SET((short)23579) ;
        p115.zacc_SET((short) -2879) ;
        p115.ind_airspeed_SET((char)24861) ;
        p115.pitchspeed_SET(-1.6687119E38F) ;
        p115.xacc_SET((short) -12924) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == (short)2911);
            assert(pack.zacc_GET() == (short) -20507);
            assert(pack.xmag_GET() == (short) -11318);
            assert(pack.yacc_GET() == (short)16701);
            assert(pack.ygyro_GET() == (short)28473);
            assert(pack.time_boot_ms_GET() == 3997952874L);
            assert(pack.xgyro_GET() == (short) -2915);
            assert(pack.xacc_GET() == (short)18812);
            assert(pack.zmag_GET() == (short) -24213);
            assert(pack.ymag_GET() == (short) -3325);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.yacc_SET((short)16701) ;
        p116.time_boot_ms_SET(3997952874L) ;
        p116.zacc_SET((short) -20507) ;
        p116.xmag_SET((short) -11318) ;
        p116.zgyro_SET((short)2911) ;
        p116.xgyro_SET((short) -2915) ;
        p116.xacc_SET((short)18812) ;
        p116.ymag_SET((short) -3325) ;
        p116.ygyro_SET((short)28473) ;
        p116.zmag_SET((short) -24213) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)121);
            assert(pack.start_GET() == (char)45584);
            assert(pack.end_GET() == (char)53391);
            assert(pack.target_system_GET() == (char)83);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.end_SET((char)53391) ;
        p117.target_system_SET((char)83) ;
        p117.target_component_SET((char)121) ;
        p117.start_SET((char)45584) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.time_utc_GET() == 2711686466L);
            assert(pack.num_logs_GET() == (char)65087);
            assert(pack.last_log_num_GET() == (char)1100);
            assert(pack.id_GET() == (char)1355);
            assert(pack.size_GET() == 1584198568L);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.last_log_num_SET((char)1100) ;
        p118.num_logs_SET((char)65087) ;
        p118.id_SET((char)1355) ;
        p118.time_utc_SET(2711686466L) ;
        p118.size_SET(1584198568L) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)231);
            assert(pack.id_GET() == (char)55808);
            assert(pack.count_GET() == 2392469813L);
            assert(pack.ofs_GET() == 330738595L);
            assert(pack.target_system_GET() == (char)145);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)145) ;
        p119.id_SET((char)55808) ;
        p119.target_component_SET((char)231) ;
        p119.count_SET(2392469813L) ;
        p119.ofs_SET(330738595L) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)60, (char)118, (char)73, (char)200, (char)134, (char)115, (char)219, (char)216, (char)211, (char)148, (char)53, (char)50, (char)152, (char)52, (char)128, (char)109, (char)62, (char)233, (char)142, (char)182, (char)160, (char)39, (char)61, (char)76, (char)45, (char)82, (char)88, (char)140, (char)252, (char)175, (char)223, (char)117, (char)102, (char)222, (char)81, (char)147, (char)221, (char)147, (char)11, (char)96, (char)112, (char)239, (char)105, (char)247, (char)118, (char)144, (char)221, (char)151, (char)21, (char)75, (char)206, (char)38, (char)221, (char)160, (char)194, (char)31, (char)130, (char)240, (char)127, (char)18, (char)73, (char)170, (char)11, (char)130, (char)5, (char)111, (char)253, (char)27, (char)153, (char)75, (char)96, (char)244, (char)195, (char)156, (char)230, (char)53, (char)212, (char)254, (char)87, (char)214, (char)33, (char)61, (char)196, (char)184, (char)7, (char)75, (char)96, (char)92, (char)245, (char)156}));
            assert(pack.id_GET() == (char)7707);
            assert(pack.ofs_GET() == 1511356778L);
            assert(pack.count_GET() == (char)139);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)7707) ;
        p120.ofs_SET(1511356778L) ;
        p120.count_SET((char)139) ;
        p120.data__SET(new char[] {(char)60, (char)118, (char)73, (char)200, (char)134, (char)115, (char)219, (char)216, (char)211, (char)148, (char)53, (char)50, (char)152, (char)52, (char)128, (char)109, (char)62, (char)233, (char)142, (char)182, (char)160, (char)39, (char)61, (char)76, (char)45, (char)82, (char)88, (char)140, (char)252, (char)175, (char)223, (char)117, (char)102, (char)222, (char)81, (char)147, (char)221, (char)147, (char)11, (char)96, (char)112, (char)239, (char)105, (char)247, (char)118, (char)144, (char)221, (char)151, (char)21, (char)75, (char)206, (char)38, (char)221, (char)160, (char)194, (char)31, (char)130, (char)240, (char)127, (char)18, (char)73, (char)170, (char)11, (char)130, (char)5, (char)111, (char)253, (char)27, (char)153, (char)75, (char)96, (char)244, (char)195, (char)156, (char)230, (char)53, (char)212, (char)254, (char)87, (char)214, (char)33, (char)61, (char)196, (char)184, (char)7, (char)75, (char)96, (char)92, (char)245, (char)156}, 0) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)103);
            assert(pack.target_component_GET() == (char)81);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)103) ;
        p121.target_component_SET((char)81) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)112);
            assert(pack.target_system_GET() == (char)168);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)168) ;
        p122.target_component_SET((char)112) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)239, (char)172, (char)113, (char)35, (char)135, (char)171, (char)188, (char)239, (char)139, (char)254, (char)190, (char)110, (char)137, (char)160, (char)115, (char)9, (char)75, (char)219, (char)31, (char)132, (char)86, (char)176, (char)10, (char)127, (char)145, (char)221, (char)253, (char)69, (char)156, (char)46, (char)32, (char)124, (char)181, (char)182, (char)72, (char)162, (char)95, (char)26, (char)80, (char)46, (char)115, (char)94, (char)0, (char)247, (char)135, (char)31, (char)3, (char)247, (char)142, (char)137, (char)112, (char)102, (char)133, (char)90, (char)77, (char)248, (char)234, (char)21, (char)141, (char)61, (char)18, (char)149, (char)69, (char)193, (char)113, (char)40, (char)48, (char)6, (char)0, (char)210, (char)153, (char)128, (char)107, (char)56, (char)183, (char)196, (char)244, (char)111, (char)66, (char)210, (char)240, (char)220, (char)69, (char)131, (char)191, (char)114, (char)68, (char)182, (char)161, (char)115, (char)163, (char)110, (char)177, (char)76, (char)246, (char)67, (char)137, (char)163, (char)86, (char)241, (char)187, (char)39, (char)244, (char)155, (char)202, (char)19, (char)213, (char)235, (char)176, (char)156}));
            assert(pack.target_system_GET() == (char)51);
            assert(pack.len_GET() == (char)107);
            assert(pack.target_component_GET() == (char)58);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)51) ;
        p123.target_component_SET((char)58) ;
        p123.data__SET(new char[] {(char)239, (char)172, (char)113, (char)35, (char)135, (char)171, (char)188, (char)239, (char)139, (char)254, (char)190, (char)110, (char)137, (char)160, (char)115, (char)9, (char)75, (char)219, (char)31, (char)132, (char)86, (char)176, (char)10, (char)127, (char)145, (char)221, (char)253, (char)69, (char)156, (char)46, (char)32, (char)124, (char)181, (char)182, (char)72, (char)162, (char)95, (char)26, (char)80, (char)46, (char)115, (char)94, (char)0, (char)247, (char)135, (char)31, (char)3, (char)247, (char)142, (char)137, (char)112, (char)102, (char)133, (char)90, (char)77, (char)248, (char)234, (char)21, (char)141, (char)61, (char)18, (char)149, (char)69, (char)193, (char)113, (char)40, (char)48, (char)6, (char)0, (char)210, (char)153, (char)128, (char)107, (char)56, (char)183, (char)196, (char)244, (char)111, (char)66, (char)210, (char)240, (char)220, (char)69, (char)131, (char)191, (char)114, (char)68, (char)182, (char)161, (char)115, (char)163, (char)110, (char)177, (char)76, (char)246, (char)67, (char)137, (char)163, (char)86, (char)241, (char)187, (char)39, (char)244, (char)155, (char)202, (char)19, (char)213, (char)235, (char)176, (char)156}, 0) ;
        p123.len_SET((char)107) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -637543600);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
            assert(pack.alt_GET() == -1506572806);
            assert(pack.satellites_visible_GET() == (char)233);
            assert(pack.eph_GET() == (char)40517);
            assert(pack.time_usec_GET() == 3990544241031601367L);
            assert(pack.lon_GET() == -81254388);
            assert(pack.epv_GET() == (char)31635);
            assert(pack.dgps_age_GET() == 1183957045L);
            assert(pack.cog_GET() == (char)24271);
            assert(pack.dgps_numch_GET() == (char)158);
            assert(pack.vel_GET() == (char)47416);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX) ;
        p124.epv_SET((char)31635) ;
        p124.time_usec_SET(3990544241031601367L) ;
        p124.dgps_numch_SET((char)158) ;
        p124.dgps_age_SET(1183957045L) ;
        p124.lon_SET(-81254388) ;
        p124.vel_SET((char)47416) ;
        p124.satellites_visible_SET((char)233) ;
        p124.eph_SET((char)40517) ;
        p124.alt_SET(-1506572806) ;
        p124.cog_SET((char)24271) ;
        p124.lat_SET(-637543600) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID));
            assert(pack.Vservo_GET() == (char)30021);
            assert(pack.Vcc_GET() == (char)36958);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vservo_SET((char)30021) ;
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID)) ;
        p125.Vcc_SET((char)36958) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING));
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)122, (char)173, (char)253, (char)115, (char)216, (char)215, (char)61, (char)253, (char)250, (char)108, (char)140, (char)249, (char)78, (char)130, (char)85, (char)145, (char)175, (char)183, (char)94, (char)146, (char)95, (char)181, (char)217, (char)228, (char)248, (char)101, (char)235, (char)229, (char)92, (char)85, (char)51, (char)39, (char)66, (char)180, (char)16, (char)60, (char)134, (char)198, (char)100, (char)60, (char)30, (char)99, (char)182, (char)214, (char)93, (char)197, (char)189, (char)136, (char)48, (char)114, (char)208, (char)204, (char)254, (char)112, (char)69, (char)36, (char)53, (char)145, (char)209, (char)108, (char)253, (char)14, (char)100, (char)224, (char)5, (char)249, (char)204, (char)142, (char)202, (char)112}));
            assert(pack.timeout_GET() == (char)3406);
            assert(pack.baudrate_GET() == 4004175418L);
            assert(pack.count_GET() == (char)80);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2) ;
        p126.data__SET(new char[] {(char)122, (char)173, (char)253, (char)115, (char)216, (char)215, (char)61, (char)253, (char)250, (char)108, (char)140, (char)249, (char)78, (char)130, (char)85, (char)145, (char)175, (char)183, (char)94, (char)146, (char)95, (char)181, (char)217, (char)228, (char)248, (char)101, (char)235, (char)229, (char)92, (char)85, (char)51, (char)39, (char)66, (char)180, (char)16, (char)60, (char)134, (char)198, (char)100, (char)60, (char)30, (char)99, (char)182, (char)214, (char)93, (char)197, (char)189, (char)136, (char)48, (char)114, (char)208, (char)204, (char)254, (char)112, (char)69, (char)36, (char)53, (char)145, (char)209, (char)108, (char)253, (char)14, (char)100, (char)224, (char)5, (char)249, (char)204, (char)142, (char)202, (char)112}, 0) ;
        p126.baudrate_SET(4004175418L) ;
        p126.count_SET((char)80) ;
        p126.timeout_SET((char)3406) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING)) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.accuracy_GET() == 2738542831L);
            assert(pack.baseline_a_mm_GET() == 1779721845);
            assert(pack.tow_GET() == 6812369L);
            assert(pack.wn_GET() == (char)30879);
            assert(pack.rtk_receiver_id_GET() == (char)92);
            assert(pack.nsats_GET() == (char)228);
            assert(pack.rtk_rate_GET() == (char)39);
            assert(pack.iar_num_hypotheses_GET() == 1533181702);
            assert(pack.baseline_c_mm_GET() == -128389460);
            assert(pack.baseline_b_mm_GET() == 169594437);
            assert(pack.time_last_baseline_ms_GET() == 2028282288L);
            assert(pack.baseline_coords_type_GET() == (char)55);
            assert(pack.rtk_health_GET() == (char)170);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.rtk_receiver_id_SET((char)92) ;
        p127.baseline_b_mm_SET(169594437) ;
        p127.baseline_coords_type_SET((char)55) ;
        p127.nsats_SET((char)228) ;
        p127.baseline_a_mm_SET(1779721845) ;
        p127.time_last_baseline_ms_SET(2028282288L) ;
        p127.rtk_health_SET((char)170) ;
        p127.wn_SET((char)30879) ;
        p127.rtk_rate_SET((char)39) ;
        p127.tow_SET(6812369L) ;
        p127.baseline_c_mm_SET(-128389460) ;
        p127.iar_num_hypotheses_SET(1533181702) ;
        p127.accuracy_SET(2738542831L) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.iar_num_hypotheses_GET() == 467001497);
            assert(pack.rtk_rate_GET() == (char)131);
            assert(pack.baseline_c_mm_GET() == 573427216);
            assert(pack.wn_GET() == (char)45614);
            assert(pack.baseline_a_mm_GET() == 1389033392);
            assert(pack.rtk_receiver_id_GET() == (char)85);
            assert(pack.time_last_baseline_ms_GET() == 3795864209L);
            assert(pack.nsats_GET() == (char)135);
            assert(pack.rtk_health_GET() == (char)10);
            assert(pack.accuracy_GET() == 3861563399L);
            assert(pack.baseline_coords_type_GET() == (char)96);
            assert(pack.baseline_b_mm_GET() == 1858230694);
            assert(pack.tow_GET() == 740326122L);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.tow_SET(740326122L) ;
        p128.time_last_baseline_ms_SET(3795864209L) ;
        p128.baseline_b_mm_SET(1858230694) ;
        p128.wn_SET((char)45614) ;
        p128.rtk_health_SET((char)10) ;
        p128.accuracy_SET(3861563399L) ;
        p128.rtk_rate_SET((char)131) ;
        p128.rtk_receiver_id_SET((char)85) ;
        p128.baseline_coords_type_SET((char)96) ;
        p128.iar_num_hypotheses_SET(467001497) ;
        p128.baseline_c_mm_SET(573427216) ;
        p128.nsats_SET((char)135) ;
        p128.baseline_a_mm_SET(1389033392) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short) -5313);
            assert(pack.xgyro_GET() == (short)15572);
            assert(pack.xacc_GET() == (short)16306);
            assert(pack.zmag_GET() == (short) -7612);
            assert(pack.zacc_GET() == (short)31615);
            assert(pack.time_boot_ms_GET() == 1559701957L);
            assert(pack.zgyro_GET() == (short) -8199);
            assert(pack.ymag_GET() == (short)3269);
            assert(pack.xmag_GET() == (short) -16280);
            assert(pack.ygyro_GET() == (short)11839);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.zgyro_SET((short) -8199) ;
        p129.time_boot_ms_SET(1559701957L) ;
        p129.ygyro_SET((short)11839) ;
        p129.zmag_SET((short) -7612) ;
        p129.zacc_SET((short)31615) ;
        p129.xmag_SET((short) -16280) ;
        p129.xacc_SET((short)16306) ;
        p129.xgyro_SET((short)15572) ;
        p129.ymag_SET((short)3269) ;
        p129.yacc_SET((short) -5313) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.payload_GET() == (char)7);
            assert(pack.height_GET() == (char)20255);
            assert(pack.width_GET() == (char)26607);
            assert(pack.packets_GET() == (char)55638);
            assert(pack.jpg_quality_GET() == (char)212);
            assert(pack.size_GET() == 216615317L);
            assert(pack.type_GET() == (char)193);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.packets_SET((char)55638) ;
        p130.jpg_quality_SET((char)212) ;
        p130.height_SET((char)20255) ;
        p130.size_SET(216615317L) ;
        p130.width_SET((char)26607) ;
        p130.type_SET((char)193) ;
        p130.payload_SET((char)7) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)173, (char)51, (char)125, (char)73, (char)67, (char)31, (char)5, (char)106, (char)152, (char)182, (char)135, (char)125, (char)170, (char)112, (char)37, (char)150, (char)100, (char)87, (char)130, (char)79, (char)250, (char)75, (char)100, (char)169, (char)157, (char)230, (char)108, (char)70, (char)222, (char)178, (char)159, (char)182, (char)31, (char)204, (char)81, (char)227, (char)225, (char)133, (char)157, (char)26, (char)210, (char)29, (char)169, (char)54, (char)128, (char)223, (char)211, (char)240, (char)193, (char)255, (char)148, (char)18, (char)123, (char)195, (char)168, (char)62, (char)52, (char)116, (char)181, (char)153, (char)62, (char)44, (char)61, (char)149, (char)0, (char)103, (char)248, (char)233, (char)151, (char)17, (char)129, (char)255, (char)227, (char)159, (char)61, (char)251, (char)146, (char)111, (char)246, (char)192, (char)132, (char)89, (char)220, (char)132, (char)89, (char)102, (char)96, (char)236, (char)160, (char)144, (char)83, (char)167, (char)202, (char)204, (char)149, (char)11, (char)57, (char)72, (char)100, (char)137, (char)67, (char)238, (char)4, (char)169, (char)102, (char)0, (char)212, (char)229, (char)171, (char)210, (char)5, (char)235, (char)215, (char)82, (char)123, (char)75, (char)62, (char)105, (char)141, (char)165, (char)230, (char)237, (char)31, (char)93, (char)46, (char)16, (char)197, (char)149, (char)80, (char)64, (char)8, (char)114, (char)102, (char)40, (char)99, (char)243, (char)39, (char)255, (char)216, (char)108, (char)2, (char)118, (char)159, (char)34, (char)214, (char)236, (char)214, (char)231, (char)182, (char)183, (char)172, (char)88, (char)222, (char)79, (char)228, (char)127, (char)116, (char)123, (char)44, (char)147, (char)36, (char)15, (char)190, (char)249, (char)245, (char)235, (char)190, (char)175, (char)61, (char)14, (char)232, (char)149, (char)101, (char)254, (char)27, (char)36, (char)223, (char)114, (char)250, (char)78, (char)106, (char)15, (char)5, (char)226, (char)53, (char)243, (char)137, (char)136, (char)73, (char)16, (char)78, (char)175, (char)205, (char)121, (char)48, (char)239, (char)108, (char)177, (char)138, (char)121, (char)15, (char)105, (char)35, (char)184, (char)240, (char)139, (char)70, (char)148, (char)27, (char)24, (char)62, (char)144, (char)213, (char)237, (char)210, (char)151, (char)79, (char)55, (char)233, (char)123, (char)190, (char)124, (char)199, (char)172, (char)65, (char)214, (char)25, (char)155, (char)79, (char)163, (char)49, (char)191, (char)162, (char)10, (char)218, (char)92, (char)231, (char)69, (char)172, (char)248, (char)27, (char)110, (char)24, (char)90, (char)35, (char)7, (char)116, (char)200, (char)75, (char)147, (char)149, (char)145, (char)207}));
            assert(pack.seqnr_GET() == (char)18518);
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)18518) ;
        p131.data__SET(new char[] {(char)173, (char)51, (char)125, (char)73, (char)67, (char)31, (char)5, (char)106, (char)152, (char)182, (char)135, (char)125, (char)170, (char)112, (char)37, (char)150, (char)100, (char)87, (char)130, (char)79, (char)250, (char)75, (char)100, (char)169, (char)157, (char)230, (char)108, (char)70, (char)222, (char)178, (char)159, (char)182, (char)31, (char)204, (char)81, (char)227, (char)225, (char)133, (char)157, (char)26, (char)210, (char)29, (char)169, (char)54, (char)128, (char)223, (char)211, (char)240, (char)193, (char)255, (char)148, (char)18, (char)123, (char)195, (char)168, (char)62, (char)52, (char)116, (char)181, (char)153, (char)62, (char)44, (char)61, (char)149, (char)0, (char)103, (char)248, (char)233, (char)151, (char)17, (char)129, (char)255, (char)227, (char)159, (char)61, (char)251, (char)146, (char)111, (char)246, (char)192, (char)132, (char)89, (char)220, (char)132, (char)89, (char)102, (char)96, (char)236, (char)160, (char)144, (char)83, (char)167, (char)202, (char)204, (char)149, (char)11, (char)57, (char)72, (char)100, (char)137, (char)67, (char)238, (char)4, (char)169, (char)102, (char)0, (char)212, (char)229, (char)171, (char)210, (char)5, (char)235, (char)215, (char)82, (char)123, (char)75, (char)62, (char)105, (char)141, (char)165, (char)230, (char)237, (char)31, (char)93, (char)46, (char)16, (char)197, (char)149, (char)80, (char)64, (char)8, (char)114, (char)102, (char)40, (char)99, (char)243, (char)39, (char)255, (char)216, (char)108, (char)2, (char)118, (char)159, (char)34, (char)214, (char)236, (char)214, (char)231, (char)182, (char)183, (char)172, (char)88, (char)222, (char)79, (char)228, (char)127, (char)116, (char)123, (char)44, (char)147, (char)36, (char)15, (char)190, (char)249, (char)245, (char)235, (char)190, (char)175, (char)61, (char)14, (char)232, (char)149, (char)101, (char)254, (char)27, (char)36, (char)223, (char)114, (char)250, (char)78, (char)106, (char)15, (char)5, (char)226, (char)53, (char)243, (char)137, (char)136, (char)73, (char)16, (char)78, (char)175, (char)205, (char)121, (char)48, (char)239, (char)108, (char)177, (char)138, (char)121, (char)15, (char)105, (char)35, (char)184, (char)240, (char)139, (char)70, (char)148, (char)27, (char)24, (char)62, (char)144, (char)213, (char)237, (char)210, (char)151, (char)79, (char)55, (char)233, (char)123, (char)190, (char)124, (char)199, (char)172, (char)65, (char)214, (char)25, (char)155, (char)79, (char)163, (char)49, (char)191, (char)162, (char)10, (char)218, (char)92, (char)231, (char)69, (char)172, (char)248, (char)27, (char)110, (char)24, (char)90, (char)35, (char)7, (char)116, (char)200, (char)75, (char)147, (char)149, (char)145, (char)207}, 0) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.covariance_GET() == (char)125);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_YAW_135);
            assert(pack.max_distance_GET() == (char)11340);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            assert(pack.current_distance_GET() == (char)20088);
            assert(pack.id_GET() == (char)103);
            assert(pack.min_distance_GET() == (char)10005);
            assert(pack.time_boot_ms_GET() == 2329760040L);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.covariance_SET((char)125) ;
        p132.min_distance_SET((char)10005) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_YAW_135) ;
        p132.id_SET((char)103) ;
        p132.current_distance_SET((char)20088) ;
        p132.max_distance_SET((char)11340) ;
        p132.time_boot_ms_SET(2329760040L) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mask_GET() == 7515728699174162144L);
            assert(pack.lon_GET() == -1529339034);
            assert(pack.lat_GET() == -1913588911);
            assert(pack.grid_spacing_GET() == (char)54791);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.grid_spacing_SET((char)54791) ;
        p133.lon_SET(-1529339034) ;
        p133.mask_SET(7515728699174162144L) ;
        p133.lat_SET(-1913588911) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.gridbit_GET() == (char)77);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -459, (short)14197, (short)31219, (short)19077, (short) -19716, (short)21997, (short) -32152, (short)32114, (short)24222, (short)20447, (short)23101, (short)30511, (short) -20569, (short)17772, (short)28631, (short) -4021}));
            assert(pack.grid_spacing_GET() == (char)8529);
            assert(pack.lon_GET() == -1539738775);
            assert(pack.lat_GET() == -1010168904);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.gridbit_SET((char)77) ;
        p134.lon_SET(-1539738775) ;
        p134.lat_SET(-1010168904) ;
        p134.grid_spacing_SET((char)8529) ;
        p134.data__SET(new short[] {(short) -459, (short)14197, (short)31219, (short)19077, (short) -19716, (short)21997, (short) -32152, (short)32114, (short)24222, (short)20447, (short)23101, (short)30511, (short) -20569, (short)17772, (short)28631, (short) -4021}, 0) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -870532850);
            assert(pack.lat_GET() == 1585124845);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lon_SET(-870532850) ;
        p135.lat_SET(1585124845) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 178687372);
            assert(pack.terrain_height_GET() == -2.7141146E38F);
            assert(pack.pending_GET() == (char)10172);
            assert(pack.current_height_GET() == 7.8322335E37F);
            assert(pack.spacing_GET() == (char)38915);
            assert(pack.lon_GET() == 2072014684);
            assert(pack.loaded_GET() == (char)56655);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.terrain_height_SET(-2.7141146E38F) ;
        p136.spacing_SET((char)38915) ;
        p136.current_height_SET(7.8322335E37F) ;
        p136.lat_SET(178687372) ;
        p136.pending_SET((char)10172) ;
        p136.lon_SET(2072014684) ;
        p136.loaded_SET((char)56655) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == 2.28626E38F);
            assert(pack.press_diff_GET() == 6.4227074E35F);
            assert(pack.temperature_GET() == (short)20595);
            assert(pack.time_boot_ms_GET() == 1086353709L);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(1086353709L) ;
        p137.press_abs_SET(2.28626E38F) ;
        p137.press_diff_SET(6.4227074E35F) ;
        p137.temperature_SET((short)20595) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6449303545004288017L);
            assert(pack.z_GET() == 9.853515E37F);
            assert(pack.x_GET() == -1.4317266E38F);
            assert(pack.y_GET() == 1.5690786E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.3163105E38F, -5.06546E37F, -2.5829223E38F, 2.4611945E38F}));
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(6449303545004288017L) ;
        p138.y_SET(1.5690786E38F) ;
        p138.q_SET(new float[] {-3.3163105E38F, -5.06546E37F, -2.5829223E38F, 2.4611945E38F}, 0) ;
        p138.z_SET(9.853515E37F) ;
        p138.x_SET(-1.4317266E38F) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)80);
            assert(pack.target_component_GET() == (char)47);
            assert(pack.target_system_GET() == (char)205);
            assert(pack.time_usec_GET() == 2665601047869213153L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.0548831E38F, 2.927232E38F, -9.032848E37F, -8.605805E37F, 7.114106E37F, -8.530167E37F, 7.870036E37F, 5.3915137E37F}));
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.controls_SET(new float[] {2.0548831E38F, 2.927232E38F, -9.032848E37F, -8.605805E37F, 7.114106E37F, -8.530167E37F, 7.870036E37F, 5.3915137E37F}, 0) ;
        p139.target_component_SET((char)47) ;
        p139.target_system_SET((char)205) ;
        p139.time_usec_SET(2665601047869213153L) ;
        p139.group_mlx_SET((char)80) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)68);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {3.2357345E38F, 2.073287E38F, -4.0251712E37F, 2.8747625E36F, 3.1728956E38F, -3.2516265E37F, 1.9593641E38F, 1.834331E38F}));
            assert(pack.time_usec_GET() == 813854189795392226L);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.group_mlx_SET((char)68) ;
        p140.time_usec_SET(813854189795392226L) ;
        p140.controls_SET(new float[] {3.2357345E38F, 2.073287E38F, -4.0251712E37F, 2.8747625E36F, 3.1728956E38F, -3.2516265E37F, 1.9593641E38F, 1.834331E38F}, 0) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_amsl_GET() == 3.0483746E38F);
            assert(pack.altitude_terrain_GET() == 2.2514566E38F);
            assert(pack.altitude_local_GET() == -1.0642922E38F);
            assert(pack.bottom_clearance_GET() == 2.818075E38F);
            assert(pack.altitude_monotonic_GET() == -5.7148295E37F);
            assert(pack.time_usec_GET() == 4964433325498633000L);
            assert(pack.altitude_relative_GET() == -8.0571527E37F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_local_SET(-1.0642922E38F) ;
        p141.altitude_amsl_SET(3.0483746E38F) ;
        p141.time_usec_SET(4964433325498633000L) ;
        p141.altitude_terrain_SET(2.2514566E38F) ;
        p141.bottom_clearance_SET(2.818075E38F) ;
        p141.altitude_monotonic_SET(-5.7148295E37F) ;
        p141.altitude_relative_SET(-8.0571527E37F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.request_id_GET() == (char)89);
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)13, (char)56, (char)65, (char)173, (char)3, (char)95, (char)175, (char)18, (char)248, (char)97, (char)5, (char)47, (char)178, (char)225, (char)180, (char)198, (char)245, (char)177, (char)146, (char)150, (char)219, (char)97, (char)200, (char)217, (char)188, (char)129, (char)173, (char)24, (char)121, (char)174, (char)213, (char)108, (char)210, (char)94, (char)202, (char)140, (char)151, (char)26, (char)136, (char)59, (char)217, (char)44, (char)37, (char)243, (char)19, (char)107, (char)3, (char)246, (char)205, (char)190, (char)227, (char)27, (char)11, (char)192, (char)47, (char)84, (char)153, (char)79, (char)231, (char)201, (char)219, (char)78, (char)207, (char)201, (char)150, (char)79, (char)202, (char)80, (char)244, (char)229, (char)21, (char)37, (char)25, (char)195, (char)117, (char)217, (char)76, (char)163, (char)168, (char)214, (char)105, (char)249, (char)249, (char)220, (char)25, (char)86, (char)11, (char)213, (char)161, (char)178, (char)168, (char)29, (char)64, (char)52, (char)77, (char)215, (char)230, (char)143, (char)2, (char)149, (char)159, (char)117, (char)65, (char)100, (char)84, (char)50, (char)250, (char)141, (char)249, (char)171, (char)83, (char)216, (char)173, (char)189, (char)228, (char)141, (char)158, (char)190, (char)131, (char)66}));
            assert(pack.uri_type_GET() == (char)41);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)246, (char)235, (char)176, (char)206, (char)153, (char)238, (char)238, (char)193, (char)76, (char)194, (char)130, (char)140, (char)207, (char)31, (char)143, (char)159, (char)238, (char)85, (char)142, (char)34, (char)30, (char)139, (char)182, (char)75, (char)156, (char)30, (char)233, (char)24, (char)18, (char)228, (char)212, (char)104, (char)72, (char)32, (char)165, (char)120, (char)44, (char)62, (char)29, (char)139, (char)230, (char)62, (char)81, (char)75, (char)66, (char)163, (char)203, (char)139, (char)128, (char)64, (char)231, (char)112, (char)233, (char)4, (char)120, (char)226, (char)203, (char)51, (char)185, (char)64, (char)55, (char)163, (char)70, (char)175, (char)8, (char)113, (char)127, (char)26, (char)220, (char)32, (char)124, (char)125, (char)135, (char)135, (char)118, (char)83, (char)237, (char)118, (char)165, (char)13, (char)12, (char)44, (char)54, (char)248, (char)99, (char)19, (char)145, (char)88, (char)227, (char)169, (char)38, (char)178, (char)81, (char)121, (char)188, (char)240, (char)246, (char)152, (char)108, (char)136, (char)94, (char)11, (char)88, (char)240, (char)113, (char)157, (char)35, (char)205, (char)153, (char)122, (char)108, (char)93, (char)212, (char)89, (char)206, (char)129, (char)153, (char)49, (char)32, (char)175}));
            assert(pack.transfer_type_GET() == (char)115);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.uri_SET(new char[] {(char)13, (char)56, (char)65, (char)173, (char)3, (char)95, (char)175, (char)18, (char)248, (char)97, (char)5, (char)47, (char)178, (char)225, (char)180, (char)198, (char)245, (char)177, (char)146, (char)150, (char)219, (char)97, (char)200, (char)217, (char)188, (char)129, (char)173, (char)24, (char)121, (char)174, (char)213, (char)108, (char)210, (char)94, (char)202, (char)140, (char)151, (char)26, (char)136, (char)59, (char)217, (char)44, (char)37, (char)243, (char)19, (char)107, (char)3, (char)246, (char)205, (char)190, (char)227, (char)27, (char)11, (char)192, (char)47, (char)84, (char)153, (char)79, (char)231, (char)201, (char)219, (char)78, (char)207, (char)201, (char)150, (char)79, (char)202, (char)80, (char)244, (char)229, (char)21, (char)37, (char)25, (char)195, (char)117, (char)217, (char)76, (char)163, (char)168, (char)214, (char)105, (char)249, (char)249, (char)220, (char)25, (char)86, (char)11, (char)213, (char)161, (char)178, (char)168, (char)29, (char)64, (char)52, (char)77, (char)215, (char)230, (char)143, (char)2, (char)149, (char)159, (char)117, (char)65, (char)100, (char)84, (char)50, (char)250, (char)141, (char)249, (char)171, (char)83, (char)216, (char)173, (char)189, (char)228, (char)141, (char)158, (char)190, (char)131, (char)66}, 0) ;
        p142.storage_SET(new char[] {(char)246, (char)235, (char)176, (char)206, (char)153, (char)238, (char)238, (char)193, (char)76, (char)194, (char)130, (char)140, (char)207, (char)31, (char)143, (char)159, (char)238, (char)85, (char)142, (char)34, (char)30, (char)139, (char)182, (char)75, (char)156, (char)30, (char)233, (char)24, (char)18, (char)228, (char)212, (char)104, (char)72, (char)32, (char)165, (char)120, (char)44, (char)62, (char)29, (char)139, (char)230, (char)62, (char)81, (char)75, (char)66, (char)163, (char)203, (char)139, (char)128, (char)64, (char)231, (char)112, (char)233, (char)4, (char)120, (char)226, (char)203, (char)51, (char)185, (char)64, (char)55, (char)163, (char)70, (char)175, (char)8, (char)113, (char)127, (char)26, (char)220, (char)32, (char)124, (char)125, (char)135, (char)135, (char)118, (char)83, (char)237, (char)118, (char)165, (char)13, (char)12, (char)44, (char)54, (char)248, (char)99, (char)19, (char)145, (char)88, (char)227, (char)169, (char)38, (char)178, (char)81, (char)121, (char)188, (char)240, (char)246, (char)152, (char)108, (char)136, (char)94, (char)11, (char)88, (char)240, (char)113, (char)157, (char)35, (char)205, (char)153, (char)122, (char)108, (char)93, (char)212, (char)89, (char)206, (char)129, (char)153, (char)49, (char)32, (char)175}, 0) ;
        p142.request_id_SET((char)89) ;
        p142.transfer_type_SET((char)115) ;
        p142.uri_type_SET((char)41) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == 1.701206E38F);
            assert(pack.temperature_GET() == (short) -11167);
            assert(pack.press_abs_GET() == 3.0977042E38F);
            assert(pack.time_boot_ms_GET() == 1240753581L);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.temperature_SET((short) -11167) ;
        p143.time_boot_ms_SET(1240753581L) ;
        p143.press_diff_SET(1.701206E38F) ;
        p143.press_abs_SET(3.0977042E38F) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1967992998);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-1.9586983E38F, -1.7514035E38F, 2.6043046E38F}));
            assert(pack.timestamp_GET() == 180730568717707660L);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {6.4206995E37F, 2.7726647E37F, 4.828655E37F}));
            assert(pack.alt_GET() == -8.729736E37F);
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-1.9007181E38F, -1.4161715E38F, 8.686438E37F}));
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-4.2268268E37F, 1.6815388E37F, -1.8214837E38F, 6.1950095E37F}));
            assert(pack.est_capabilities_GET() == (char)210);
            assert(pack.lon_GET() == 484473015);
            assert(Arrays.equals(pack.vel_GET(),  new float[] {-2.30392E37F, 3.4003375E38F, -3.9124958E36F}));
            assert(pack.custom_state_GET() == 3387912660413058546L);
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.est_capabilities_SET((char)210) ;
        p144.vel_SET(new float[] {-2.30392E37F, 3.4003375E38F, -3.9124958E36F}, 0) ;
        p144.alt_SET(-8.729736E37F) ;
        p144.position_cov_SET(new float[] {-1.9586983E38F, -1.7514035E38F, 2.6043046E38F}, 0) ;
        p144.timestamp_SET(180730568717707660L) ;
        p144.custom_state_SET(3387912660413058546L) ;
        p144.lon_SET(484473015) ;
        p144.lat_SET(-1967992998) ;
        p144.attitude_q_SET(new float[] {-4.2268268E37F, 1.6815388E37F, -1.8214837E38F, 6.1950095E37F}, 0) ;
        p144.acc_SET(new float[] {6.4206995E37F, 2.7726647E37F, 4.828655E37F}, 0) ;
        p144.rates_SET(new float[] {-1.9007181E38F, -1.4161715E38F, 8.686438E37F}, 0) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.z_acc_GET() == 3.0563798E38F);
            assert(pack.y_pos_GET() == 2.1405315E38F);
            assert(pack.x_acc_GET() == -2.262717E37F);
            assert(pack.y_vel_GET() == -6.905982E35F);
            assert(pack.y_acc_GET() == -2.6181761E38F);
            assert(pack.yaw_rate_GET() == -1.3472289E38F);
            assert(pack.roll_rate_GET() == -1.3464837E38F);
            assert(pack.x_pos_GET() == 2.885548E38F);
            assert(pack.airspeed_GET() == 1.3006303E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {2.4212475E38F, 1.5058427E38F, -1.2693475E38F}));
            assert(pack.x_vel_GET() == -2.9847657E38F);
            assert(pack.pitch_rate_GET() == -8.2968685E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.3894411E38F, 1.3648233E38F, 2.0926881E38F, -2.711394E37F}));
            assert(pack.z_pos_GET() == 1.1018647E38F);
            assert(pack.z_vel_GET() == -1.7273188E37F);
            assert(pack.time_usec_GET() == 2717256788527209716L);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {-2.639061E38F, -1.8930484E38F, -3.1924192E38F}));
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.q_SET(new float[] {3.3894411E38F, 1.3648233E38F, 2.0926881E38F, -2.711394E37F}, 0) ;
        p146.x_acc_SET(-2.262717E37F) ;
        p146.yaw_rate_SET(-1.3472289E38F) ;
        p146.roll_rate_SET(-1.3464837E38F) ;
        p146.z_acc_SET(3.0563798E38F) ;
        p146.x_pos_SET(2.885548E38F) ;
        p146.pos_variance_SET(new float[] {-2.639061E38F, -1.8930484E38F, -3.1924192E38F}, 0) ;
        p146.vel_variance_SET(new float[] {2.4212475E38F, 1.5058427E38F, -1.2693475E38F}, 0) ;
        p146.z_vel_SET(-1.7273188E37F) ;
        p146.time_usec_SET(2717256788527209716L) ;
        p146.z_pos_SET(1.1018647E38F) ;
        p146.y_vel_SET(-6.905982E35F) ;
        p146.y_acc_SET(-2.6181761E38F) ;
        p146.y_pos_SET(2.1405315E38F) ;
        p146.airspeed_SET(1.3006303E38F) ;
        p146.pitch_rate_SET(-8.2968685E37F) ;
        p146.x_vel_SET(-2.9847657E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.energy_consumed_GET() == 1658092930);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
            assert(pack.battery_remaining_GET() == (byte)100);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)5284, (char)56725, (char)19056, (char)1348, (char)32864, (char)28056, (char)11121, (char)4424, (char)61259, (char)35255}));
            assert(pack.id_GET() == (char)43);
            assert(pack.current_consumed_GET() == -657584905);
            assert(pack.current_battery_GET() == (short)2895);
            assert(pack.temperature_GET() == (short)17390);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE) ;
        p147.current_consumed_SET(-657584905) ;
        p147.battery_remaining_SET((byte)100) ;
        p147.voltages_SET(new char[] {(char)5284, (char)56725, (char)19056, (char)1348, (char)32864, (char)28056, (char)11121, (char)4424, (char)61259, (char)35255}, 0) ;
        p147.id_SET((char)43) ;
        p147.temperature_SET((short)17390) ;
        p147.current_battery_SET((short)2895) ;
        p147.energy_consumed_SET(1658092930) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.flight_sw_version_GET() == 980355084L);
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY));
            assert(pack.os_sw_version_GET() == 2861522356L);
            assert(pack.uid_GET() == 6251055558089617908L);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)11, (char)219, (char)234, (char)93, (char)104, (char)38, (char)111, (char)72}));
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)201, (char)66, (char)149, (char)128, (char)46, (char)177, (char)51, (char)98}));
            assert(pack.product_id_GET() == (char)40306);
            assert(pack.board_version_GET() == 421921417L);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)178, (char)86, (char)160, (char)235, (char)129, (char)143, (char)165, (char)150}));
            assert(pack.middleware_sw_version_GET() == 4051283451L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)118, (char)147, (char)31, (char)132, (char)20, (char)117, (char)233, (char)245, (char)42, (char)173, (char)60, (char)156, (char)10, (char)89, (char)176, (char)23, (char)255, (char)220}));
            assert(pack.vendor_id_GET() == (char)17659);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.middleware_sw_version_SET(4051283451L) ;
        p148.os_sw_version_SET(2861522356L) ;
        p148.vendor_id_SET((char)17659) ;
        p148.flight_custom_version_SET(new char[] {(char)178, (char)86, (char)160, (char)235, (char)129, (char)143, (char)165, (char)150}, 0) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY)) ;
        p148.os_custom_version_SET(new char[] {(char)11, (char)219, (char)234, (char)93, (char)104, (char)38, (char)111, (char)72}, 0) ;
        p148.board_version_SET(421921417L) ;
        p148.uid_SET(6251055558089617908L) ;
        p148.flight_sw_version_SET(980355084L) ;
        p148.uid2_SET(new char[] {(char)118, (char)147, (char)31, (char)132, (char)20, (char)117, (char)233, (char)245, (char)42, (char)173, (char)60, (char)156, (char)10, (char)89, (char)176, (char)23, (char)255, (char)220}, 0, PH) ;
        p148.product_id_SET((char)40306) ;
        p148.middleware_custom_version_SET(new char[] {(char)201, (char)66, (char)149, (char)128, (char)46, (char)177, (char)51, (char)98}, 0) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.x_TRY(ph) == 4.5887436E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.position_valid_TRY(ph) == (char)124);
            assert(pack.size_x_GET() == -3.2024892E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
            assert(pack.time_usec_GET() == 3666383658329777176L);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {-2.54701E38F, 1.5194789E38F, -8.049281E37F, -1.9938846E38F}));
            assert(pack.z_TRY(ph) == 1.7721875E38F);
            assert(pack.size_y_GET() == 1.275985E38F);
            assert(pack.angle_y_GET() == -1.2160211E38F);
            assert(pack.angle_x_GET() == -5.9866894E37F);
            assert(pack.y_TRY(ph) == -1.7017583E38F);
            assert(pack.target_num_GET() == (char)30);
            assert(pack.distance_GET() == 2.7222647E38F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.x_SET(4.5887436E37F, PH) ;
        p149.size_y_SET(1.275985E38F) ;
        p149.position_valid_SET((char)124, PH) ;
        p149.angle_x_SET(-5.9866894E37F) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p149.angle_y_SET(-1.2160211E38F) ;
        p149.distance_SET(2.7222647E38F) ;
        p149.q_SET(new float[] {-2.54701E38F, 1.5194789E38F, -8.049281E37F, -1.9938846E38F}, 0, PH) ;
        p149.z_SET(1.7721875E38F, PH) ;
        p149.size_x_SET(-3.2024892E38F) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON) ;
        p149.target_num_SET((char)30) ;
        p149.time_usec_SET(3666383658329777176L) ;
        p149.y_SET(-1.7017583E38F, PH) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)234);
            assert(pack.target_system_GET() == (char)114);
        });
        GroundControl.FLEXIFUNCTION_SET p150 = CommunicationChannel.new_FLEXIFUNCTION_SET();
        PH.setPack(p150);
        p150.target_system_SET((char)114) ;
        p150.target_component_SET((char)234) ;
        CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_READ_REQ.add((src, ph, pack) ->
        {
            assert(pack.data_index_GET() == (short)21826);
            assert(pack.target_system_GET() == (char)62);
            assert(pack.target_component_GET() == (char)190);
            assert(pack.read_req_type_GET() == (short) -29488);
        });
        GroundControl.FLEXIFUNCTION_READ_REQ p151 = CommunicationChannel.new_FLEXIFUNCTION_READ_REQ();
        PH.setPack(p151);
        p151.read_req_type_SET((short) -29488) ;
        p151.data_index_SET((short)21826) ;
        p151.target_system_SET((char)62) ;
        p151.target_component_SET((char)190) ;
        CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_BUFFER_FUNCTION.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)209);
            assert(pack.func_count_GET() == (char)64121);
            assert(pack.data_address_GET() == (char)34540);
            assert(Arrays.equals(pack.data__GET(),  new byte[] {(byte)104, (byte) - 116, (byte)124, (byte)42, (byte)16, (byte)41, (byte) - 124, (byte) - 30, (byte) - 6, (byte) - 105, (byte) - 61, (byte) - 57, (byte) - 62, (byte) - 34, (byte) - 87, (byte)42, (byte)105, (byte)18, (byte) - 114, (byte) - 81, (byte)17, (byte) - 14, (byte) - 49, (byte)102, (byte) - 46, (byte)82, (byte)27, (byte) - 6, (byte) - 29, (byte) - 48, (byte)109, (byte) - 84, (byte) - 87, (byte)108, (byte) - 77, (byte)92, (byte) - 69, (byte)2, (byte)120, (byte) - 32, (byte) - 111, (byte)115, (byte) - 27, (byte)92, (byte)30, (byte)66, (byte)109, (byte)25}));
            assert(pack.func_index_GET() == (char)19868);
            assert(pack.data_size_GET() == (char)25940);
            assert(pack.target_component_GET() == (char)182);
        });
        GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION p152 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION();
        PH.setPack(p152);
        p152.func_count_SET((char)64121) ;
        p152.data_size_SET((char)25940) ;
        p152.target_component_SET((char)182) ;
        p152.data__SET(new byte[] {(byte)104, (byte) - 116, (byte)124, (byte)42, (byte)16, (byte)41, (byte) - 124, (byte) - 30, (byte) - 6, (byte) - 105, (byte) - 61, (byte) - 57, (byte) - 62, (byte) - 34, (byte) - 87, (byte)42, (byte)105, (byte)18, (byte) - 114, (byte) - 81, (byte)17, (byte) - 14, (byte) - 49, (byte)102, (byte) - 46, (byte)82, (byte)27, (byte) - 6, (byte) - 29, (byte) - 48, (byte)109, (byte) - 84, (byte) - 87, (byte)108, (byte) - 77, (byte)92, (byte) - 69, (byte)2, (byte)120, (byte) - 32, (byte) - 111, (byte)115, (byte) - 27, (byte)92, (byte)30, (byte)66, (byte)109, (byte)25}, 0) ;
        p152.target_system_SET((char)209) ;
        p152.func_index_SET((char)19868) ;
        p152.data_address_SET((char)34540) ;
        CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_BUFFER_FUNCTION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)77);
            assert(pack.result_GET() == (char)46460);
            assert(pack.func_index_GET() == (char)39189);
            assert(pack.target_system_GET() == (char)30);
        });
        GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION_ACK p153 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
        PH.setPack(p153);
        p153.target_system_SET((char)30) ;
        p153.result_SET((char)46460) ;
        p153.func_index_SET((char)39189) ;
        p153.target_component_SET((char)77) ;
        CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_DIRECTORY.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)243);
            assert(Arrays.equals(pack.directory_data_GET(),  new byte[] {(byte)33, (byte)0, (byte) - 121, (byte)70, (byte)11, (byte) - 73, (byte) - 2, (byte)8, (byte) - 55, (byte) - 52, (byte) - 56, (byte) - 123, (byte)60, (byte)3, (byte) - 99, (byte)17, (byte) - 89, (byte)83, (byte) - 103, (byte)68, (byte)30, (byte)88, (byte) - 20, (byte) - 121, (byte) - 83, (byte) - 112, (byte) - 103, (byte) - 35, (byte) - 98, (byte) - 122, (byte)35, (byte) - 116, (byte)109, (byte) - 35, (byte) - 73, (byte)121, (byte)41, (byte)41, (byte)73, (byte)15, (byte) - 57, (byte)89, (byte) - 73, (byte) - 55, (byte)123, (byte) - 68, (byte)19, (byte)53}));
            assert(pack.target_system_GET() == (char)47);
            assert(pack.count_GET() == (char)86);
            assert(pack.start_index_GET() == (char)113);
            assert(pack.directory_type_GET() == (char)198);
        });
        GroundControl.FLEXIFUNCTION_DIRECTORY p155 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY();
        PH.setPack(p155);
        p155.count_SET((char)86) ;
        p155.directory_type_SET((char)198) ;
        p155.directory_data_SET(new byte[] {(byte)33, (byte)0, (byte) - 121, (byte)70, (byte)11, (byte) - 73, (byte) - 2, (byte)8, (byte) - 55, (byte) - 52, (byte) - 56, (byte) - 123, (byte)60, (byte)3, (byte) - 99, (byte)17, (byte) - 89, (byte)83, (byte) - 103, (byte)68, (byte)30, (byte)88, (byte) - 20, (byte) - 121, (byte) - 83, (byte) - 112, (byte) - 103, (byte) - 35, (byte) - 98, (byte) - 122, (byte)35, (byte) - 116, (byte)109, (byte) - 35, (byte) - 73, (byte)121, (byte)41, (byte)41, (byte)73, (byte)15, (byte) - 57, (byte)89, (byte) - 73, (byte) - 55, (byte)123, (byte) - 68, (byte)19, (byte)53}, 0) ;
        p155.start_index_SET((char)113) ;
        p155.target_system_SET((char)47) ;
        p155.target_component_SET((char)243) ;
        CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_DIRECTORY_ACK.add((src, ph, pack) ->
        {
            assert(pack.start_index_GET() == (char)210);
            assert(pack.target_component_GET() == (char)81);
            assert(pack.target_system_GET() == (char)120);
            assert(pack.directory_type_GET() == (char)33);
            assert(pack.count_GET() == (char)37);
            assert(pack.result_GET() == (char)36072);
        });
        GroundControl.FLEXIFUNCTION_DIRECTORY_ACK p156 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY_ACK();
        PH.setPack(p156);
        p156.start_index_SET((char)210) ;
        p156.directory_type_SET((char)33) ;
        p156.target_component_SET((char)81) ;
        p156.target_system_SET((char)120) ;
        p156.result_SET((char)36072) ;
        p156.count_SET((char)37) ;
        CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_COMMAND.add((src, ph, pack) ->
        {
            assert(pack.command_type_GET() == (char)247);
            assert(pack.target_component_GET() == (char)209);
            assert(pack.target_system_GET() == (char)25);
        });
        GroundControl.FLEXIFUNCTION_COMMAND p157 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND();
        PH.setPack(p157);
        p157.command_type_SET((char)247) ;
        p157.target_system_SET((char)25) ;
        p157.target_component_SET((char)209) ;
        CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.command_type_GET() == (char)43596);
            assert(pack.result_GET() == (char)59631);
        });
        GroundControl.FLEXIFUNCTION_COMMAND_ACK p158 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND_ACK();
        PH.setPack(p158);
        p158.result_SET((char)59631) ;
        p158.command_type_SET((char)43596) ;
        CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F2_A.add((src, ph, pack) ->
        {
            assert(pack.sue_rmat0_GET() == (short) -21385);
            assert(pack.sue_cpu_load_GET() == (char)11613);
            assert(pack.sue_status_GET() == (char)96);
            assert(pack.sue_rmat5_GET() == (short)12385);
            assert(pack.sue_rmat7_GET() == (short)7339);
            assert(pack.sue_sog_GET() == (short)2258);
            assert(pack.sue_longitude_GET() == -1636991973);
            assert(pack.sue_estimated_wind_1_GET() == (short) -1996);
            assert(pack.sue_hdop_GET() == (short) -10952);
            assert(pack.sue_rmat6_GET() == (short)27300);
            assert(pack.sue_latitude_GET() == -131127560);
            assert(pack.sue_time_GET() == 2943445312L);
            assert(pack.sue_waypoint_index_GET() == (char)22510);
            assert(pack.sue_rmat4_GET() == (short)24848);
            assert(pack.sue_rmat8_GET() == (short)21860);
            assert(pack.sue_svs_GET() == (short) -531);
            assert(pack.sue_magFieldEarth0_GET() == (short) -28832);
            assert(pack.sue_altitude_GET() == 1532296227);
            assert(pack.sue_rmat2_GET() == (short)19219);
            assert(pack.sue_estimated_wind_2_GET() == (short) -13067);
            assert(pack.sue_cog_GET() == (char)39898);
            assert(pack.sue_air_speed_3DIMU_GET() == (char)4995);
            assert(pack.sue_magFieldEarth1_GET() == (short)28551);
            assert(pack.sue_rmat3_GET() == (short)7257);
            assert(pack.sue_estimated_wind_0_GET() == (short)26493);
            assert(pack.sue_magFieldEarth2_GET() == (short) -18828);
            assert(pack.sue_rmat1_GET() == (short)6778);
        });
        GroundControl.SERIAL_UDB_EXTRA_F2_A p170 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_A();
        PH.setPack(p170);
        p170.sue_magFieldEarth0_SET((short) -28832) ;
        p170.sue_estimated_wind_1_SET((short) -1996) ;
        p170.sue_estimated_wind_0_SET((short)26493) ;
        p170.sue_magFieldEarth1_SET((short)28551) ;
        p170.sue_hdop_SET((short) -10952) ;
        p170.sue_longitude_SET(-1636991973) ;
        p170.sue_rmat5_SET((short)12385) ;
        p170.sue_time_SET(2943445312L) ;
        p170.sue_status_SET((char)96) ;
        p170.sue_rmat7_SET((short)7339) ;
        p170.sue_altitude_SET(1532296227) ;
        p170.sue_svs_SET((short) -531) ;
        p170.sue_cpu_load_SET((char)11613) ;
        p170.sue_rmat0_SET((short) -21385) ;
        p170.sue_latitude_SET(-131127560) ;
        p170.sue_rmat8_SET((short)21860) ;
        p170.sue_rmat1_SET((short)6778) ;
        p170.sue_magFieldEarth2_SET((short) -18828) ;
        p170.sue_sog_SET((short)2258) ;
        p170.sue_rmat2_SET((short)19219) ;
        p170.sue_rmat6_SET((short)27300) ;
        p170.sue_air_speed_3DIMU_SET((char)4995) ;
        p170.sue_cog_SET((char)39898) ;
        p170.sue_rmat4_SET((short)24848) ;
        p170.sue_rmat3_SET((short)7257) ;
        p170.sue_estimated_wind_2_SET((short) -13067) ;
        p170.sue_waypoint_index_SET((char)22510) ;
        CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F2_B.add((src, ph, pack) ->
        {
            assert(pack.sue_imu_velocity_y_GET() == (short) -19150);
            assert(pack.sue_time_GET() == 1684537750L);
            assert(pack.sue_aero_z_GET() == (short) -30022);
            assert(pack.sue_pwm_output_11_GET() == (short) -367);
            assert(pack.sue_pwm_output_12_GET() == (short) -11037);
            assert(pack.sue_barom_press_GET() == -111193018);
            assert(pack.sue_desired_height_GET() == (short)28593);
            assert(pack.sue_aero_x_GET() == (short) -2216);
            assert(pack.sue_imu_velocity_z_GET() == (short)29039);
            assert(pack.sue_pwm_input_10_GET() == (short)27421);
            assert(pack.sue_pwm_input_11_GET() == (short)7941);
            assert(pack.sue_imu_location_y_GET() == (short) -29006);
            assert(pack.sue_waypoint_goal_z_GET() == (short) -31646);
            assert(pack.sue_pwm_output_9_GET() == (short)316);
            assert(pack.sue_imu_location_x_GET() == (short)16070);
            assert(pack.sue_location_error_earth_z_GET() == (short) -11983);
            assert(pack.sue_osc_fails_GET() == (short)24015);
            assert(pack.sue_pwm_output_3_GET() == (short)29813);
            assert(pack.sue_memory_stack_free_GET() == (short) -22933);
            assert(pack.sue_pwm_input_7_GET() == (short)4179);
            assert(pack.sue_bat_volt_GET() == (short) -5013);
            assert(pack.sue_location_error_earth_x_GET() == (short)8796);
            assert(pack.sue_barom_alt_GET() == -853746477);
            assert(pack.sue_pwm_input_9_GET() == (short)29146);
            assert(pack.sue_bat_amp_GET() == (short)24075);
            assert(pack.sue_pwm_input_3_GET() == (short) -10275);
            assert(pack.sue_pwm_output_5_GET() == (short)238);
            assert(pack.sue_pwm_input_4_GET() == (short)25079);
            assert(pack.sue_waypoint_goal_x_GET() == (short)18424);
            assert(pack.sue_pwm_input_6_GET() == (short)12602);
            assert(pack.sue_aero_y_GET() == (short) -6729);
            assert(pack.sue_pwm_input_12_GET() == (short)12298);
            assert(pack.sue_bat_amp_hours_GET() == (short)17439);
            assert(pack.sue_pwm_output_8_GET() == (short)12749);
            assert(pack.sue_imu_location_z_GET() == (short) -3740);
            assert(pack.sue_pwm_output_7_GET() == (short) -23561);
            assert(pack.sue_pwm_output_1_GET() == (short) -7521);
            assert(pack.sue_barom_temp_GET() == (short) -7065);
            assert(pack.sue_pwm_output_6_GET() == (short) -22959);
            assert(pack.sue_flags_GET() == 4290133093L);
            assert(pack.sue_pwm_output_10_GET() == (short) -3494);
            assert(pack.sue_waypoint_goal_y_GET() == (short) -9061);
            assert(pack.sue_imu_velocity_x_GET() == (short)28674);
            assert(pack.sue_pwm_output_4_GET() == (short) -15654);
            assert(pack.sue_pwm_input_2_GET() == (short) -10175);
            assert(pack.sue_location_error_earth_y_GET() == (short)16362);
            assert(pack.sue_pwm_input_5_GET() == (short)17024);
            assert(pack.sue_pwm_input_1_GET() == (short)8134);
            assert(pack.sue_pwm_output_2_GET() == (short)17270);
            assert(pack.sue_pwm_input_8_GET() == (short)17250);
        });
        GroundControl.SERIAL_UDB_EXTRA_F2_B p171 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_B();
        PH.setPack(p171);
        p171.sue_imu_location_z_SET((short) -3740) ;
        p171.sue_pwm_output_2_SET((short)17270) ;
        p171.sue_pwm_output_11_SET((short) -367) ;
        p171.sue_pwm_input_10_SET((short)27421) ;
        p171.sue_pwm_input_6_SET((short)12602) ;
        p171.sue_pwm_input_9_SET((short)29146) ;
        p171.sue_desired_height_SET((short)28593) ;
        p171.sue_pwm_output_7_SET((short) -23561) ;
        p171.sue_osc_fails_SET((short)24015) ;
        p171.sue_location_error_earth_x_SET((short)8796) ;
        p171.sue_aero_z_SET((short) -30022) ;
        p171.sue_imu_location_y_SET((short) -29006) ;
        p171.sue_waypoint_goal_z_SET((short) -31646) ;
        p171.sue_pwm_output_1_SET((short) -7521) ;
        p171.sue_pwm_input_4_SET((short)25079) ;
        p171.sue_pwm_input_5_SET((short)17024) ;
        p171.sue_pwm_output_4_SET((short) -15654) ;
        p171.sue_flags_SET(4290133093L) ;
        p171.sue_pwm_output_5_SET((short)238) ;
        p171.sue_pwm_output_12_SET((short) -11037) ;
        p171.sue_waypoint_goal_x_SET((short)18424) ;
        p171.sue_pwm_input_7_SET((short)4179) ;
        p171.sue_bat_volt_SET((short) -5013) ;
        p171.sue_imu_velocity_z_SET((short)29039) ;
        p171.sue_location_error_earth_y_SET((short)16362) ;
        p171.sue_pwm_output_9_SET((short)316) ;
        p171.sue_pwm_output_8_SET((short)12749) ;
        p171.sue_pwm_input_2_SET((short) -10175) ;
        p171.sue_barom_temp_SET((short) -7065) ;
        p171.sue_barom_alt_SET(-853746477) ;
        p171.sue_bat_amp_SET((short)24075) ;
        p171.sue_imu_velocity_x_SET((short)28674) ;
        p171.sue_pwm_output_10_SET((short) -3494) ;
        p171.sue_pwm_input_3_SET((short) -10275) ;
        p171.sue_pwm_input_12_SET((short)12298) ;
        p171.sue_barom_press_SET(-111193018) ;
        p171.sue_pwm_output_6_SET((short) -22959) ;
        p171.sue_imu_location_x_SET((short)16070) ;
        p171.sue_bat_amp_hours_SET((short)17439) ;
        p171.sue_memory_stack_free_SET((short) -22933) ;
        p171.sue_pwm_input_8_SET((short)17250) ;
        p171.sue_pwm_input_11_SET((short)7941) ;
        p171.sue_location_error_earth_z_SET((short) -11983) ;
        p171.sue_pwm_input_1_SET((short)8134) ;
        p171.sue_waypoint_goal_y_SET((short) -9061) ;
        p171.sue_pwm_output_3_SET((short)29813) ;
        p171.sue_time_SET(1684537750L) ;
        p171.sue_imu_velocity_y_SET((short) -19150) ;
        p171.sue_aero_x_SET((short) -2216) ;
        p171.sue_aero_y_SET((short) -6729) ;
        CommunicationChannel.instance.send(p171);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F4.add((src, ph, pack) ->
        {
            assert(pack.sue_RUDDER_NAVIGATION_GET() == (char)204);
            assert(pack.sue_ALTITUDEHOLD_WAYPOINT_GET() == (char)194);
            assert(pack.sue_ROLL_STABILIZATION_AILERONS_GET() == (char)177);
            assert(pack.sue_AILERON_NAVIGATION_GET() == (char)124);
            assert(pack.sue_ALTITUDEHOLD_STABILIZED_GET() == (char)1);
            assert(pack.sue_RACING_MODE_GET() == (char)89);
            assert(pack.sue_ROLL_STABILIZATION_RUDDER_GET() == (char)209);
            assert(pack.sue_YAW_STABILIZATION_AILERON_GET() == (char)230);
            assert(pack.sue_YAW_STABILIZATION_RUDDER_GET() == (char)84);
            assert(pack.sue_PITCH_STABILIZATION_GET() == (char)182);
        });
        GroundControl.SERIAL_UDB_EXTRA_F4 p172 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F4();
        PH.setPack(p172);
        p172.sue_PITCH_STABILIZATION_SET((char)182) ;
        p172.sue_ALTITUDEHOLD_WAYPOINT_SET((char)194) ;
        p172.sue_RUDDER_NAVIGATION_SET((char)204) ;
        p172.sue_ALTITUDEHOLD_STABILIZED_SET((char)1) ;
        p172.sue_YAW_STABILIZATION_AILERON_SET((char)230) ;
        p172.sue_AILERON_NAVIGATION_SET((char)124) ;
        p172.sue_RACING_MODE_SET((char)89) ;
        p172.sue_ROLL_STABILIZATION_RUDDER_SET((char)209) ;
        p172.sue_YAW_STABILIZATION_RUDDER_SET((char)84) ;
        p172.sue_ROLL_STABILIZATION_AILERONS_SET((char)177) ;
        CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F5.add((src, ph, pack) ->
        {
            assert(pack.sue_YAWKP_AILERON_GET() == 6.5846995E36F);
            assert(pack.sue_YAWKD_AILERON_GET() == -1.2524787E38F);
            assert(pack.sue_ROLLKP_GET() == 3.5752234E36F);
            assert(pack.sue_ROLLKD_GET() == 2.2111988E38F);
        });
        GroundControl.SERIAL_UDB_EXTRA_F5 p173 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F5();
        PH.setPack(p173);
        p173.sue_YAWKP_AILERON_SET(6.5846995E36F) ;
        p173.sue_ROLLKD_SET(2.2111988E38F) ;
        p173.sue_ROLLKP_SET(3.5752234E36F) ;
        p173.sue_YAWKD_AILERON_SET(-1.2524787E38F) ;
        CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F6.add((src, ph, pack) ->
        {
            assert(pack.sue_PITCHKD_GET() == 2.8977101E38F);
            assert(pack.sue_RUDDER_ELEV_MIX_GET() == 1.3072872E38F);
            assert(pack.sue_ROLL_ELEV_MIX_GET() == 1.244926E38F);
            assert(pack.sue_PITCHGAIN_GET() == 2.5360575E37F);
            assert(pack.sue_ELEVATOR_BOOST_GET() == -3.2386895E38F);
        });
        GroundControl.SERIAL_UDB_EXTRA_F6 p174 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F6();
        PH.setPack(p174);
        p174.sue_RUDDER_ELEV_MIX_SET(1.3072872E38F) ;
        p174.sue_ROLL_ELEV_MIX_SET(1.244926E38F) ;
        p174.sue_PITCHKD_SET(2.8977101E38F) ;
        p174.sue_ELEVATOR_BOOST_SET(-3.2386895E38F) ;
        p174.sue_PITCHGAIN_SET(2.5360575E37F) ;
        CommunicationChannel.instance.send(p174);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F7.add((src, ph, pack) ->
        {
            assert(pack.sue_ROLLKD_RUDDER_GET() == -2.3026031E38F);
            assert(pack.sue_YAWKP_RUDDER_GET() == 2.1654898E38F);
            assert(pack.sue_RTL_PITCH_DOWN_GET() == 1.3094918E38F);
            assert(pack.sue_RUDDER_BOOST_GET() == -5.523895E37F);
            assert(pack.sue_ROLLKP_RUDDER_GET() == 2.6685191E38F);
            assert(pack.sue_YAWKD_RUDDER_GET() == 2.121466E38F);
        });
        GroundControl.SERIAL_UDB_EXTRA_F7 p175 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F7();
        PH.setPack(p175);
        p175.sue_ROLLKP_RUDDER_SET(2.6685191E38F) ;
        p175.sue_RTL_PITCH_DOWN_SET(1.3094918E38F) ;
        p175.sue_YAWKP_RUDDER_SET(2.1654898E38F) ;
        p175.sue_ROLLKD_RUDDER_SET(-2.3026031E38F) ;
        p175.sue_RUDDER_BOOST_SET(-5.523895E37F) ;
        p175.sue_YAWKD_RUDDER_SET(2.121466E38F) ;
        CommunicationChannel.instance.send(p175);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F8.add((src, ph, pack) ->
        {
            assert(pack.sue_ALT_HOLD_PITCH_MIN_GET() == 2.6234936E38F);
            assert(pack.sue_ALT_HOLD_THROTTLE_MIN_GET() == -3.111382E38F);
            assert(pack.sue_HEIGHT_TARGET_MIN_GET() == -5.2857506E37F);
            assert(pack.sue_ALT_HOLD_PITCH_MAX_GET() == -2.958971E38F);
            assert(pack.sue_ALT_HOLD_THROTTLE_MAX_GET() == -3.0224147E38F);
            assert(pack.sue_HEIGHT_TARGET_MAX_GET() == 2.8476142E38F);
            assert(pack.sue_ALT_HOLD_PITCH_HIGH_GET() == 3.027404E38F);
        });
        GroundControl.SERIAL_UDB_EXTRA_F8 p176 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F8();
        PH.setPack(p176);
        p176.sue_HEIGHT_TARGET_MIN_SET(-5.2857506E37F) ;
        p176.sue_ALT_HOLD_THROTTLE_MIN_SET(-3.111382E38F) ;
        p176.sue_ALT_HOLD_PITCH_MAX_SET(-2.958971E38F) ;
        p176.sue_ALT_HOLD_PITCH_MIN_SET(2.6234936E38F) ;
        p176.sue_HEIGHT_TARGET_MAX_SET(2.8476142E38F) ;
        p176.sue_ALT_HOLD_THROTTLE_MAX_SET(-3.0224147E38F) ;
        p176.sue_ALT_HOLD_PITCH_HIGH_SET(3.027404E38F) ;
        CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F13.add((src, ph, pack) ->
        {
            assert(pack.sue_lat_origin_GET() == 457447820);
            assert(pack.sue_alt_origin_GET() == -1037648643);
            assert(pack.sue_lon_origin_GET() == 995127093);
            assert(pack.sue_week_no_GET() == (short)11643);
        });
        GroundControl.SERIAL_UDB_EXTRA_F13 p177 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F13();
        PH.setPack(p177);
        p177.sue_lon_origin_SET(995127093) ;
        p177.sue_lat_origin_SET(457447820) ;
        p177.sue_week_no_SET((short)11643) ;
        p177.sue_alt_origin_SET(-1037648643) ;
        CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F14.add((src, ph, pack) ->
        {
            assert(pack.sue_TRAP_FLAGS_GET() == (short)7062);
            assert(pack.sue_WIND_ESTIMATION_GET() == (char)7);
            assert(pack.sue_CLOCK_CONFIG_GET() == (char)222);
            assert(pack.sue_osc_fail_count_GET() == (short)24655);
            assert(pack.sue_RCON_GET() == (short)9433);
            assert(pack.sue_AIRFRAME_GET() == (char)224);
            assert(pack.sue_GPS_TYPE_GET() == (char)235);
            assert(pack.sue_DR_GET() == (char)90);
            assert(pack.sue_FLIGHT_PLAN_TYPE_GET() == (char)225);
            assert(pack.sue_BOARD_TYPE_GET() == (char)14);
            assert(pack.sue_TRAP_SOURCE_GET() == 3507923840L);
        });
        GroundControl.SERIAL_UDB_EXTRA_F14 p178 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F14();
        PH.setPack(p178);
        p178.sue_BOARD_TYPE_SET((char)14) ;
        p178.sue_WIND_ESTIMATION_SET((char)7) ;
        p178.sue_DR_SET((char)90) ;
        p178.sue_GPS_TYPE_SET((char)235) ;
        p178.sue_CLOCK_CONFIG_SET((char)222) ;
        p178.sue_osc_fail_count_SET((short)24655) ;
        p178.sue_TRAP_SOURCE_SET(3507923840L) ;
        p178.sue_AIRFRAME_SET((char)224) ;
        p178.sue_TRAP_FLAGS_SET((short)7062) ;
        p178.sue_RCON_SET((short)9433) ;
        p178.sue_FLIGHT_PLAN_TYPE_SET((char)225) ;
        CommunicationChannel.instance.send(p178);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F15.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.sue_ID_VEHICLE_MODEL_NAME_GET(),  new char[] {(char)229, (char)120, (char)20, (char)36, (char)211, (char)238, (char)113, (char)40, (char)90, (char)111, (char)13, (char)244, (char)26, (char)107, (char)101, (char)131, (char)27, (char)11, (char)169, (char)136, (char)144, (char)216, (char)88, (char)174, (char)44, (char)1, (char)175, (char)3, (char)161, (char)156, (char)16, (char)79, (char)178, (char)194, (char)237, (char)45, (char)129, (char)193, (char)69, (char)81}));
            assert(Arrays.equals(pack.sue_ID_VEHICLE_REGISTRATION_GET(),  new char[] {(char)117, (char)119, (char)70, (char)118, (char)233, (char)206, (char)17, (char)182, (char)116, (char)18, (char)156, (char)171, (char)247, (char)199, (char)214, (char)95, (char)233, (char)152, (char)165, (char)83}));
        });
        GroundControl.SERIAL_UDB_EXTRA_F15 p179 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F15();
        PH.setPack(p179);
        p179.sue_ID_VEHICLE_MODEL_NAME_SET(new char[] {(char)229, (char)120, (char)20, (char)36, (char)211, (char)238, (char)113, (char)40, (char)90, (char)111, (char)13, (char)244, (char)26, (char)107, (char)101, (char)131, (char)27, (char)11, (char)169, (char)136, (char)144, (char)216, (char)88, (char)174, (char)44, (char)1, (char)175, (char)3, (char)161, (char)156, (char)16, (char)79, (char)178, (char)194, (char)237, (char)45, (char)129, (char)193, (char)69, (char)81}, 0) ;
        p179.sue_ID_VEHICLE_REGISTRATION_SET(new char[] {(char)117, (char)119, (char)70, (char)118, (char)233, (char)206, (char)17, (char)182, (char)116, (char)18, (char)156, (char)171, (char)247, (char)199, (char)214, (char)95, (char)233, (char)152, (char)165, (char)83}, 0) ;
        CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F16.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.sue_ID_LEAD_PILOT_GET(),  new char[] {(char)3, (char)152, (char)152, (char)204, (char)254, (char)209, (char)162, (char)128, (char)53, (char)90, (char)3, (char)44, (char)65, (char)117, (char)155, (char)233, (char)185, (char)196, (char)112, (char)44, (char)18, (char)79, (char)61, (char)140, (char)246, (char)110, (char)70, (char)203, (char)219, (char)153, (char)23, (char)144, (char)248, (char)45, (char)59, (char)31, (char)186, (char)22, (char)160, (char)99}));
            assert(Arrays.equals(pack.sue_ID_DIY_DRONES_URL_GET(),  new char[] {(char)111, (char)183, (char)97, (char)157, (char)236, (char)152, (char)47, (char)52, (char)252, (char)237, (char)227, (char)122, (char)96, (char)110, (char)156, (char)24, (char)22, (char)54, (char)129, (char)118, (char)226, (char)32, (char)248, (char)118, (char)216, (char)212, (char)177, (char)237, (char)121, (char)213, (char)147, (char)245, (char)64, (char)69, (char)136, (char)64, (char)42, (char)183, (char)54, (char)107, (char)47, (char)71, (char)39, (char)43, (char)206, (char)97, (char)199, (char)20, (char)84, (char)42, (char)70, (char)144, (char)109, (char)184, (char)41, (char)138, (char)148, (char)162, (char)224, (char)157, (char)101, (char)90, (char)201, (char)236, (char)87, (char)9, (char)188, (char)233, (char)130, (char)154}));
        });
        GroundControl.SERIAL_UDB_EXTRA_F16 p180 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F16();
        PH.setPack(p180);
        p180.sue_ID_LEAD_PILOT_SET(new char[] {(char)3, (char)152, (char)152, (char)204, (char)254, (char)209, (char)162, (char)128, (char)53, (char)90, (char)3, (char)44, (char)65, (char)117, (char)155, (char)233, (char)185, (char)196, (char)112, (char)44, (char)18, (char)79, (char)61, (char)140, (char)246, (char)110, (char)70, (char)203, (char)219, (char)153, (char)23, (char)144, (char)248, (char)45, (char)59, (char)31, (char)186, (char)22, (char)160, (char)99}, 0) ;
        p180.sue_ID_DIY_DRONES_URL_SET(new char[] {(char)111, (char)183, (char)97, (char)157, (char)236, (char)152, (char)47, (char)52, (char)252, (char)237, (char)227, (char)122, (char)96, (char)110, (char)156, (char)24, (char)22, (char)54, (char)129, (char)118, (char)226, (char)32, (char)248, (char)118, (char)216, (char)212, (char)177, (char)237, (char)121, (char)213, (char)147, (char)245, (char)64, (char)69, (char)136, (char)64, (char)42, (char)183, (char)54, (char)107, (char)47, (char)71, (char)39, (char)43, (char)206, (char)97, (char)199, (char)20, (char)84, (char)42, (char)70, (char)144, (char)109, (char)184, (char)41, (char)138, (char)148, (char)162, (char)224, (char)157, (char)101, (char)90, (char)201, (char)236, (char)87, (char)9, (char)188, (char)233, (char)130, (char)154}, 0) ;
        CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ALTITUDES.add((src, ph, pack) ->
        {
            assert(pack.alt_range_finder_GET() == 1559786297);
            assert(pack.alt_imu_GET() == 2056845078);
            assert(pack.alt_gps_GET() == -1935972269);
            assert(pack.alt_optical_flow_GET() == 649633389);
            assert(pack.alt_barometric_GET() == 1669921671);
            assert(pack.alt_extra_GET() == 392289172);
            assert(pack.time_boot_ms_GET() == 4091623317L);
        });
        GroundControl.ALTITUDES p181 = CommunicationChannel.new_ALTITUDES();
        PH.setPack(p181);
        p181.alt_gps_SET(-1935972269) ;
        p181.alt_range_finder_SET(1559786297) ;
        p181.time_boot_ms_SET(4091623317L) ;
        p181.alt_imu_SET(2056845078) ;
        p181.alt_optical_flow_SET(649633389) ;
        p181.alt_extra_SET(392289172) ;
        p181.alt_barometric_SET(1669921671) ;
        CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_AIRSPEEDS.add((src, ph, pack) ->
        {
            assert(pack.aoa_GET() == (short) -30734);
            assert(pack.airspeed_pitot_GET() == (short) -30554);
            assert(pack.airspeed_ultrasonic_GET() == (short) -10914);
            assert(pack.aoy_GET() == (short)25522);
            assert(pack.airspeed_hot_wire_GET() == (short) -22086);
            assert(pack.airspeed_imu_GET() == (short) -11504);
            assert(pack.time_boot_ms_GET() == 3279528744L);
        });
        GroundControl.AIRSPEEDS p182 = CommunicationChannel.new_AIRSPEEDS();
        PH.setPack(p182);
        p182.aoy_SET((short)25522) ;
        p182.airspeed_hot_wire_SET((short) -22086) ;
        p182.airspeed_imu_SET((short) -11504) ;
        p182.aoa_SET((short) -30734) ;
        p182.airspeed_pitot_SET((short) -30554) ;
        p182.time_boot_ms_SET(3279528744L) ;
        p182.airspeed_ultrasonic_SET((short) -10914) ;
        CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F17.add((src, ph, pack) ->
        {
            assert(pack.sue_turn_rate_nav_GET() == -1.3087264E38F);
            assert(pack.sue_turn_rate_fbw_GET() == -2.2197388E38F);
            assert(pack.sue_feed_forward_GET() == 7.326029E37F);
        });
        GroundControl.SERIAL_UDB_EXTRA_F17 p183 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F17();
        PH.setPack(p183);
        p183.sue_turn_rate_nav_SET(-1.3087264E38F) ;
        p183.sue_turn_rate_fbw_SET(-2.2197388E38F) ;
        p183.sue_feed_forward_SET(7.326029E37F) ;
        CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F18.add((src, ph, pack) ->
        {
            assert(pack.elevator_trim_inverted_GET() == -2.3808336E38F);
            assert(pack.elevator_trim_normal_GET() == -6.6568856E37F);
            assert(pack.reference_speed_GET() == -2.129442E38F);
            assert(pack.angle_of_attack_normal_GET() == 1.6001898E38F);
            assert(pack.angle_of_attack_inverted_GET() == 1.4553416E38F);
        });
        GroundControl.SERIAL_UDB_EXTRA_F18 p184 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F18();
        PH.setPack(p184);
        p184.elevator_trim_inverted_SET(-2.3808336E38F) ;
        p184.angle_of_attack_normal_SET(1.6001898E38F) ;
        p184.reference_speed_SET(-2.129442E38F) ;
        p184.elevator_trim_normal_SET(-6.6568856E37F) ;
        p184.angle_of_attack_inverted_SET(1.4553416E38F) ;
        CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F19.add((src, ph, pack) ->
        {
            assert(pack.sue_rudder_output_channel_GET() == (char)202);
            assert(pack.sue_throttle_output_channel_GET() == (char)220);
            assert(pack.sue_aileron_output_channel_GET() == (char)8);
            assert(pack.sue_elevator_reversed_GET() == (char)28);
            assert(pack.sue_aileron_reversed_GET() == (char)130);
            assert(pack.sue_rudder_reversed_GET() == (char)206);
            assert(pack.sue_throttle_reversed_GET() == (char)179);
            assert(pack.sue_elevator_output_channel_GET() == (char)1);
        });
        GroundControl.SERIAL_UDB_EXTRA_F19 p185 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F19();
        PH.setPack(p185);
        p185.sue_throttle_output_channel_SET((char)220) ;
        p185.sue_elevator_output_channel_SET((char)1) ;
        p185.sue_rudder_output_channel_SET((char)202) ;
        p185.sue_aileron_output_channel_SET((char)8) ;
        p185.sue_elevator_reversed_SET((char)28) ;
        p185.sue_throttle_reversed_SET((char)179) ;
        p185.sue_aileron_reversed_SET((char)130) ;
        p185.sue_rudder_reversed_SET((char)206) ;
        CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F20.add((src, ph, pack) ->
        {
            assert(pack.sue_trim_value_input_1_GET() == (short)7729);
            assert(pack.sue_number_of_inputs_GET() == (char)16);
            assert(pack.sue_trim_value_input_11_GET() == (short) -28125);
            assert(pack.sue_trim_value_input_6_GET() == (short) -27581);
            assert(pack.sue_trim_value_input_8_GET() == (short)15784);
            assert(pack.sue_trim_value_input_12_GET() == (short) -16989);
            assert(pack.sue_trim_value_input_2_GET() == (short) -32449);
            assert(pack.sue_trim_value_input_5_GET() == (short) -15920);
            assert(pack.sue_trim_value_input_9_GET() == (short) -22378);
            assert(pack.sue_trim_value_input_10_GET() == (short) -14493);
            assert(pack.sue_trim_value_input_3_GET() == (short)6075);
            assert(pack.sue_trim_value_input_7_GET() == (short)16735);
            assert(pack.sue_trim_value_input_4_GET() == (short) -21867);
        });
        GroundControl.SERIAL_UDB_EXTRA_F20 p186 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F20();
        PH.setPack(p186);
        p186.sue_trim_value_input_3_SET((short)6075) ;
        p186.sue_number_of_inputs_SET((char)16) ;
        p186.sue_trim_value_input_4_SET((short) -21867) ;
        p186.sue_trim_value_input_8_SET((short)15784) ;
        p186.sue_trim_value_input_2_SET((short) -32449) ;
        p186.sue_trim_value_input_5_SET((short) -15920) ;
        p186.sue_trim_value_input_7_SET((short)16735) ;
        p186.sue_trim_value_input_6_SET((short) -27581) ;
        p186.sue_trim_value_input_9_SET((short) -22378) ;
        p186.sue_trim_value_input_1_SET((short)7729) ;
        p186.sue_trim_value_input_12_SET((short) -16989) ;
        p186.sue_trim_value_input_11_SET((short) -28125) ;
        p186.sue_trim_value_input_10_SET((short) -14493) ;
        CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F21.add((src, ph, pack) ->
        {
            assert(pack.sue_gyro_x_offset_GET() == (short) -14362);
            assert(pack.sue_gyro_z_offset_GET() == (short)13832);
            assert(pack.sue_accel_x_offset_GET() == (short) -2805);
            assert(pack.sue_accel_y_offset_GET() == (short)813);
            assert(pack.sue_gyro_y_offset_GET() == (short)147);
            assert(pack.sue_accel_z_offset_GET() == (short)7402);
        });
        GroundControl.SERIAL_UDB_EXTRA_F21 p187 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F21();
        PH.setPack(p187);
        p187.sue_accel_y_offset_SET((short)813) ;
        p187.sue_gyro_z_offset_SET((short)13832) ;
        p187.sue_gyro_x_offset_SET((short) -14362) ;
        p187.sue_accel_x_offset_SET((short) -2805) ;
        p187.sue_gyro_y_offset_SET((short)147) ;
        p187.sue_accel_z_offset_SET((short)7402) ;
        CommunicationChannel.instance.send(p187);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F22.add((src, ph, pack) ->
        {
            assert(pack.sue_accel_x_at_calibration_GET() == (short) -10066);
            assert(pack.sue_gyro_z_at_calibration_GET() == (short)6437);
            assert(pack.sue_accel_z_at_calibration_GET() == (short)17729);
            assert(pack.sue_gyro_y_at_calibration_GET() == (short)28892);
            assert(pack.sue_accel_y_at_calibration_GET() == (short)16274);
            assert(pack.sue_gyro_x_at_calibration_GET() == (short)25113);
        });
        GroundControl.SERIAL_UDB_EXTRA_F22 p188 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F22();
        PH.setPack(p188);
        p188.sue_accel_z_at_calibration_SET((short)17729) ;
        p188.sue_gyro_x_at_calibration_SET((short)25113) ;
        p188.sue_accel_y_at_calibration_SET((short)16274) ;
        p188.sue_gyro_z_at_calibration_SET((short)6437) ;
        p188.sue_accel_x_at_calibration_SET((short) -10066) ;
        p188.sue_gyro_y_at_calibration_SET((short)28892) ;
        CommunicationChannel.instance.send(p188);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        CommunicationChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pos_horiz_ratio_GET() == 1.9275974E38F);
            assert(pack.vel_ratio_GET() == 2.9332747E38F);
            assert(pack.hagl_ratio_GET() == -2.4590957E38F);
            assert(pack.pos_vert_ratio_GET() == 2.110891E38F);
            assert(pack.pos_vert_accuracy_GET() == -3.189904E38F);
            assert(pack.time_usec_GET() == 3906405792172300192L);
            assert(pack.mag_ratio_GET() == 2.4046754E38F);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE));
            assert(pack.pos_horiz_accuracy_GET() == -2.5823925E38F);
            assert(pack.tas_ratio_GET() == -2.88624E38F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.pos_vert_ratio_SET(2.110891E38F) ;
        p230.mag_ratio_SET(2.4046754E38F) ;
        p230.tas_ratio_SET(-2.88624E38F) ;
        p230.time_usec_SET(3906405792172300192L) ;
        p230.pos_horiz_accuracy_SET(-2.5823925E38F) ;
        p230.hagl_ratio_SET(-2.4590957E38F) ;
        p230.vel_ratio_SET(2.9332747E38F) ;
        p230.pos_horiz_ratio_SET(1.9275974E38F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE)) ;
        p230.pos_vert_accuracy_SET(-3.189904E38F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.horiz_accuracy_GET() == 1.3624262E38F);
            assert(pack.time_usec_GET() == 8176733171900295096L);
            assert(pack.wind_x_GET() == 5.2943473E37F);
            assert(pack.vert_accuracy_GET() == -2.1805414E38F);
            assert(pack.wind_alt_GET() == -1.9738906E38F);
            assert(pack.wind_z_GET() == -1.8483634E38F);
            assert(pack.wind_y_GET() == -1.4052333E38F);
            assert(pack.var_vert_GET() == 2.3245791E37F);
            assert(pack.var_horiz_GET() == 2.2040172E38F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.var_vert_SET(2.3245791E37F) ;
        p231.time_usec_SET(8176733171900295096L) ;
        p231.wind_alt_SET(-1.9738906E38F) ;
        p231.vert_accuracy_SET(-2.1805414E38F) ;
        p231.wind_x_SET(5.2943473E37F) ;
        p231.var_horiz_SET(2.2040172E38F) ;
        p231.horiz_accuracy_SET(1.3624262E38F) ;
        p231.wind_y_SET(-1.4052333E38F) ;
        p231.wind_z_SET(-1.8483634E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.satellites_visible_GET() == (char)23);
            assert(pack.alt_GET() == -2.4651701E38F);
            assert(pack.time_week_ms_GET() == 3142047270L);
            assert(pack.fix_type_GET() == (char)14);
            assert(pack.gps_id_GET() == (char)78);
            assert(pack.speed_accuracy_GET() == 2.6810906E38F);
            assert(pack.ve_GET() == -8.086151E37F);
            assert(pack.vert_accuracy_GET() == 1.532488E38F);
            assert(pack.time_usec_GET() == 7759496825242501122L);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY));
            assert(pack.lat_GET() == 366959722);
            assert(pack.lon_GET() == 1858892100);
            assert(pack.vdop_GET() == -1.1452531E38F);
            assert(pack.horiz_accuracy_GET() == 2.812877E38F);
            assert(pack.vd_GET() == 2.5728352E38F);
            assert(pack.vn_GET() == -1.5713086E38F);
            assert(pack.hdop_GET() == -1.2402542E38F);
            assert(pack.time_week_GET() == (char)14863);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.lat_SET(366959722) ;
        p232.satellites_visible_SET((char)23) ;
        p232.speed_accuracy_SET(2.6810906E38F) ;
        p232.horiz_accuracy_SET(2.812877E38F) ;
        p232.vn_SET(-1.5713086E38F) ;
        p232.time_week_SET((char)14863) ;
        p232.lon_SET(1858892100) ;
        p232.time_week_ms_SET(3142047270L) ;
        p232.ve_SET(-8.086151E37F) ;
        p232.time_usec_SET(7759496825242501122L) ;
        p232.alt_SET(-2.4651701E38F) ;
        p232.vert_accuracy_SET(1.532488E38F) ;
        p232.hdop_SET(-1.2402542E38F) ;
        p232.fix_type_SET((char)14) ;
        p232.vdop_SET(-1.1452531E38F) ;
        p232.gps_id_SET((char)78) ;
        p232.vd_SET(2.5728352E38F) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY)) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (char)190);
            assert(pack.len_GET() == (char)125);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)84, (char)189, (char)55, (char)203, (char)131, (char)198, (char)29, (char)241, (char)20, (char)50, (char)66, (char)188, (char)199, (char)247, (char)102, (char)215, (char)74, (char)65, (char)217, (char)118, (char)91, (char)162, (char)66, (char)245, (char)179, (char)119, (char)41, (char)139, (char)122, (char)157, (char)52, (char)237, (char)96, (char)112, (char)63, (char)222, (char)129, (char)207, (char)137, (char)36, (char)85, (char)71, (char)77, (char)60, (char)60, (char)223, (char)6, (char)46, (char)98, (char)230, (char)242, (char)59, (char)159, (char)217, (char)167, (char)185, (char)59, (char)254, (char)206, (char)215, (char)113, (char)122, (char)13, (char)162, (char)242, (char)109, (char)46, (char)36, (char)88, (char)136, (char)246, (char)2, (char)109, (char)251, (char)190, (char)13, (char)214, (char)29, (char)97, (char)197, (char)250, (char)50, (char)19, (char)29, (char)222, (char)52, (char)55, (char)221, (char)57, (char)112, (char)173, (char)130, (char)192, (char)216, (char)151, (char)220, (char)129, (char)5, (char)41, (char)157, (char)124, (char)184, (char)251, (char)193, (char)105, (char)59, (char)19, (char)64, (char)159, (char)140, (char)121, (char)206, (char)13, (char)220, (char)116, (char)3, (char)119, (char)197, (char)118, (char)39, (char)173, (char)237, (char)216, (char)105, (char)116, (char)107, (char)246, (char)191, (char)80, (char)212, (char)188, (char)26, (char)36, (char)52, (char)250, (char)60, (char)37, (char)149, (char)235, (char)174, (char)157, (char)164, (char)240, (char)197, (char)86, (char)180, (char)130, (char)186, (char)186, (char)4, (char)110, (char)20, (char)8, (char)10, (char)160, (char)220, (char)135, (char)1, (char)151, (char)217, (char)198, (char)178, (char)208, (char)231, (char)86, (char)220, (char)151, (char)84, (char)198, (char)248, (char)209, (char)27, (char)85, (char)216, (char)171, (char)114, (char)231, (char)237, (char)197, (char)199}));
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)190) ;
        p233.len_SET((char)125) ;
        p233.data__SET(new char[] {(char)84, (char)189, (char)55, (char)203, (char)131, (char)198, (char)29, (char)241, (char)20, (char)50, (char)66, (char)188, (char)199, (char)247, (char)102, (char)215, (char)74, (char)65, (char)217, (char)118, (char)91, (char)162, (char)66, (char)245, (char)179, (char)119, (char)41, (char)139, (char)122, (char)157, (char)52, (char)237, (char)96, (char)112, (char)63, (char)222, (char)129, (char)207, (char)137, (char)36, (char)85, (char)71, (char)77, (char)60, (char)60, (char)223, (char)6, (char)46, (char)98, (char)230, (char)242, (char)59, (char)159, (char)217, (char)167, (char)185, (char)59, (char)254, (char)206, (char)215, (char)113, (char)122, (char)13, (char)162, (char)242, (char)109, (char)46, (char)36, (char)88, (char)136, (char)246, (char)2, (char)109, (char)251, (char)190, (char)13, (char)214, (char)29, (char)97, (char)197, (char)250, (char)50, (char)19, (char)29, (char)222, (char)52, (char)55, (char)221, (char)57, (char)112, (char)173, (char)130, (char)192, (char)216, (char)151, (char)220, (char)129, (char)5, (char)41, (char)157, (char)124, (char)184, (char)251, (char)193, (char)105, (char)59, (char)19, (char)64, (char)159, (char)140, (char)121, (char)206, (char)13, (char)220, (char)116, (char)3, (char)119, (char)197, (char)118, (char)39, (char)173, (char)237, (char)216, (char)105, (char)116, (char)107, (char)246, (char)191, (char)80, (char)212, (char)188, (char)26, (char)36, (char)52, (char)250, (char)60, (char)37, (char)149, (char)235, (char)174, (char)157, (char)164, (char)240, (char)197, (char)86, (char)180, (char)130, (char)186, (char)186, (char)4, (char)110, (char)20, (char)8, (char)10, (char)160, (char)220, (char)135, (char)1, (char)151, (char)217, (char)198, (char)178, (char)208, (char)231, (char)86, (char)220, (char)151, (char)84, (char)198, (char)248, (char)209, (char)27, (char)85, (char)216, (char)171, (char)114, (char)231, (char)237, (char)197, (char)199}, 0) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.throttle_GET() == (byte) - 19);
            assert(pack.pitch_GET() == (short)3638);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
            assert(pack.roll_GET() == (short)4520);
            assert(pack.gps_nsat_GET() == (char)87);
            assert(pack.wp_distance_GET() == (char)16247);
            assert(pack.battery_remaining_GET() == (char)226);
            assert(pack.airspeed_GET() == (char)35);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            assert(pack.airspeed_sp_GET() == (char)88);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
            assert(pack.failsafe_GET() == (char)157);
            assert(pack.longitude_GET() == -1585906877);
            assert(pack.latitude_GET() == 23652167);
            assert(pack.custom_mode_GET() == 2656664260L);
            assert(pack.altitude_amsl_GET() == (short)18193);
            assert(pack.temperature_air_GET() == (byte)33);
            assert(pack.groundspeed_GET() == (char)183);
            assert(pack.temperature_GET() == (byte)3);
            assert(pack.heading_sp_GET() == (short)8542);
            assert(pack.altitude_sp_GET() == (short)14897);
            assert(pack.wp_num_GET() == (char)105);
            assert(pack.heading_GET() == (char)57296);
            assert(pack.climb_rate_GET() == (byte) - 5);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.gps_nsat_SET((char)87) ;
        p234.temperature_air_SET((byte)33) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED)) ;
        p234.heading_sp_SET((short)8542) ;
        p234.climb_rate_SET((byte) - 5) ;
        p234.altitude_sp_SET((short)14897) ;
        p234.temperature_SET((byte)3) ;
        p234.airspeed_SET((char)35) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        p234.wp_num_SET((char)105) ;
        p234.longitude_SET(-1585906877) ;
        p234.heading_SET((char)57296) ;
        p234.airspeed_sp_SET((char)88) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS) ;
        p234.groundspeed_SET((char)183) ;
        p234.latitude_SET(23652167) ;
        p234.throttle_SET((byte) - 19) ;
        p234.custom_mode_SET(2656664260L) ;
        p234.battery_remaining_SET((char)226) ;
        p234.roll_SET((short)4520) ;
        p234.failsafe_SET((char)157) ;
        p234.altitude_amsl_SET((short)18193) ;
        p234.wp_distance_SET((char)16247) ;
        p234.pitch_SET((short)3638) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.clipping_1_GET() == 2610532677L);
            assert(pack.clipping_2_GET() == 2610346831L);
            assert(pack.vibration_z_GET() == 2.6618146E38F);
            assert(pack.vibration_x_GET() == -9.901004E37F);
            assert(pack.vibration_y_GET() == 2.3243055E38F);
            assert(pack.clipping_0_GET() == 3578341559L);
            assert(pack.time_usec_GET() == 7713094667975268939L);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.vibration_y_SET(2.3243055E38F) ;
        p241.clipping_0_SET(3578341559L) ;
        p241.time_usec_SET(7713094667975268939L) ;
        p241.vibration_z_SET(2.6618146E38F) ;
        p241.clipping_2_SET(2610346831L) ;
        p241.clipping_1_SET(2610532677L) ;
        p241.vibration_x_SET(-9.901004E37F) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == -1567263852);
            assert(pack.z_GET() == 4.8469346E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-5.7481733E37F, 1.9259079E38F, 2.1764628E37F, 1.3995193E38F}));
            assert(pack.latitude_GET() == 490293509);
            assert(pack.approach_y_GET() == 3.037667E38F);
            assert(pack.approach_z_GET() == -1.9387022E38F);
            assert(pack.approach_x_GET() == 2.44337E38F);
            assert(pack.x_GET() == 2.7380547E38F);
            assert(pack.longitude_GET() == -1892921184);
            assert(pack.y_GET() == 2.032452E38F);
            assert(pack.time_usec_TRY(ph) == 9046380933732793736L);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.x_SET(2.7380547E38F) ;
        p242.y_SET(2.032452E38F) ;
        p242.longitude_SET(-1892921184) ;
        p242.approach_z_SET(-1.9387022E38F) ;
        p242.q_SET(new float[] {-5.7481733E37F, 1.9259079E38F, 2.1764628E37F, 1.3995193E38F}, 0) ;
        p242.latitude_SET(490293509) ;
        p242.time_usec_SET(9046380933732793736L, PH) ;
        p242.z_SET(4.8469346E37F) ;
        p242.altitude_SET(-1567263852) ;
        p242.approach_x_SET(2.44337E38F) ;
        p242.approach_y_SET(3.037667E38F) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == -1691856978);
            assert(pack.x_GET() == -2.1825996E38F);
            assert(pack.z_GET() == -1.581987E38F);
            assert(pack.y_GET() == 3.0900208E38F);
            assert(pack.altitude_GET() == -1369613143);
            assert(pack.approach_z_GET() == 3.0884802E38F);
            assert(pack.longitude_GET() == 626928852);
            assert(pack.approach_x_GET() == 1.3455023E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-4.290624E37F, -5.5095954E37F, -3.2135042E37F, -1.0108872E38F}));
            assert(pack.approach_y_GET() == -2.8182191E38F);
            assert(pack.time_usec_TRY(ph) == 554330540270228639L);
            assert(pack.target_system_GET() == (char)1);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.z_SET(-1.581987E38F) ;
        p243.approach_y_SET(-2.8182191E38F) ;
        p243.q_SET(new float[] {-4.290624E37F, -5.5095954E37F, -3.2135042E37F, -1.0108872E38F}, 0) ;
        p243.y_SET(3.0900208E38F) ;
        p243.latitude_SET(-1691856978) ;
        p243.approach_x_SET(1.3455023E38F) ;
        p243.x_SET(-2.1825996E38F) ;
        p243.longitude_SET(626928852) ;
        p243.time_usec_SET(554330540270228639L, PH) ;
        p243.altitude_SET(-1369613143) ;
        p243.approach_z_SET(3.0884802E38F) ;
        p243.target_system_SET((char)1) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.interval_us_GET() == -220266921);
            assert(pack.message_id_GET() == (char)8894);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)8894) ;
        p244.interval_us_SET(-220266921) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            assert(pack.lat_GET() == 1193219509);
            assert(pack.squawk_GET() == (char)22115);
            assert(pack.altitude_GET() == -346561681);
            assert(pack.hor_velocity_GET() == (char)42061);
            assert(pack.ver_velocity_GET() == (short) -15511);
            assert(pack.tslc_GET() == (char)15);
            assert(pack.lon_GET() == -1368882970);
            assert(pack.callsign_LEN(ph) == 7);
            assert(pack.callsign_TRY(ph).equals("yoYxzge"));
            assert(pack.ICAO_address_GET() == 1491688944L);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED));
            assert(pack.heading_GET() == (char)45197);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SMALL);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH) ;
        p246.tslc_SET((char)15) ;
        p246.heading_SET((char)45197) ;
        p246.lon_SET(-1368882970) ;
        p246.ICAO_address_SET(1491688944L) ;
        p246.altitude_SET(-346561681) ;
        p246.hor_velocity_SET((char)42061) ;
        p246.ver_velocity_SET((short) -15511) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SMALL) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED)) ;
        p246.lat_SET(1193219509) ;
        p246.callsign_SET("yoYxzge", PH) ;
        p246.squawk_SET((char)22115) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == 1791052293L);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY);
            assert(pack.horizontal_minimum_delta_GET() == 1.7211925E38F);
            assert(pack.threat_level_GET() == (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE));
            assert(pack.time_to_minimum_delta_GET() == -2.1020693E38F);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
            assert(pack.altitude_minimum_delta_GET() == 3.378084E38F);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.horizontal_minimum_delta_SET(1.7211925E38F) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY) ;
        p247.time_to_minimum_delta_SET(-2.1020693E38F) ;
        p247.altitude_minimum_delta_SET(3.378084E38F) ;
        p247.threat_level_SET((MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE)) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        p247.id_SET(1791052293L) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)125, (char)125, (char)5, (char)141, (char)246, (char)26, (char)72, (char)181, (char)198, (char)41, (char)45, (char)203, (char)214, (char)41, (char)187, (char)178, (char)102, (char)218, (char)213, (char)215, (char)110, (char)203, (char)243, (char)110, (char)107, (char)145, (char)201, (char)11, (char)3, (char)84, (char)0, (char)98, (char)96, (char)250, (char)26, (char)102, (char)233, (char)207, (char)52, (char)158, (char)43, (char)81, (char)15, (char)52, (char)60, (char)135, (char)149, (char)175, (char)102, (char)180, (char)41, (char)5, (char)231, (char)33, (char)75, (char)249, (char)41, (char)11, (char)171, (char)207, (char)31, (char)87, (char)236, (char)138, (char)58, (char)70, (char)243, (char)73, (char)74, (char)109, (char)182, (char)117, (char)154, (char)5, (char)255, (char)209, (char)6, (char)61, (char)239, (char)217, (char)0, (char)247, (char)80, (char)37, (char)228, (char)74, (char)196, (char)242, (char)183, (char)72, (char)144, (char)10, (char)207, (char)33, (char)73, (char)35, (char)176, (char)111, (char)66, (char)161, (char)57, (char)11, (char)215, (char)238, (char)189, (char)169, (char)24, (char)14, (char)54, (char)185, (char)41, (char)239, (char)228, (char)218, (char)184, (char)158, (char)89, (char)176, (char)105, (char)249, (char)97, (char)41, (char)51, (char)210, (char)169, (char)244, (char)58, (char)133, (char)123, (char)221, (char)34, (char)53, (char)76, (char)17, (char)102, (char)54, (char)88, (char)210, (char)211, (char)62, (char)43, (char)135, (char)203, (char)222, (char)136, (char)14, (char)125, (char)214, (char)116, (char)93, (char)119, (char)185, (char)209, (char)212, (char)199, (char)35, (char)105, (char)93, (char)203, (char)244, (char)194, (char)76, (char)228, (char)30, (char)221, (char)147, (char)32, (char)36, (char)214, (char)17, (char)86, (char)231, (char)31, (char)175, (char)151, (char)145, (char)226, (char)172, (char)170, (char)148, (char)142, (char)65, (char)247, (char)228, (char)219, (char)70, (char)97, (char)1, (char)113, (char)201, (char)85, (char)226, (char)89, (char)190, (char)20, (char)154, (char)21, (char)237, (char)44, (char)239, (char)143, (char)36, (char)146, (char)135, (char)179, (char)50, (char)232, (char)124, (char)56, (char)141, (char)182, (char)39, (char)231, (char)159, (char)238, (char)76, (char)237, (char)62, (char)193, (char)121, (char)64, (char)148, (char)245, (char)190, (char)142, (char)167, (char)237, (char)177, (char)124, (char)150, (char)221, (char)4, (char)117, (char)150, (char)32, (char)120, (char)210, (char)177, (char)244, (char)91, (char)64, (char)98, (char)21, (char)234, (char)91, (char)232, (char)118, (char)122, (char)143}));
            assert(pack.target_component_GET() == (char)116);
            assert(pack.target_system_GET() == (char)169);
            assert(pack.target_network_GET() == (char)149);
            assert(pack.message_type_GET() == (char)46473);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.message_type_SET((char)46473) ;
        p248.target_system_SET((char)169) ;
        p248.target_component_SET((char)116) ;
        p248.payload_SET(new char[] {(char)125, (char)125, (char)5, (char)141, (char)246, (char)26, (char)72, (char)181, (char)198, (char)41, (char)45, (char)203, (char)214, (char)41, (char)187, (char)178, (char)102, (char)218, (char)213, (char)215, (char)110, (char)203, (char)243, (char)110, (char)107, (char)145, (char)201, (char)11, (char)3, (char)84, (char)0, (char)98, (char)96, (char)250, (char)26, (char)102, (char)233, (char)207, (char)52, (char)158, (char)43, (char)81, (char)15, (char)52, (char)60, (char)135, (char)149, (char)175, (char)102, (char)180, (char)41, (char)5, (char)231, (char)33, (char)75, (char)249, (char)41, (char)11, (char)171, (char)207, (char)31, (char)87, (char)236, (char)138, (char)58, (char)70, (char)243, (char)73, (char)74, (char)109, (char)182, (char)117, (char)154, (char)5, (char)255, (char)209, (char)6, (char)61, (char)239, (char)217, (char)0, (char)247, (char)80, (char)37, (char)228, (char)74, (char)196, (char)242, (char)183, (char)72, (char)144, (char)10, (char)207, (char)33, (char)73, (char)35, (char)176, (char)111, (char)66, (char)161, (char)57, (char)11, (char)215, (char)238, (char)189, (char)169, (char)24, (char)14, (char)54, (char)185, (char)41, (char)239, (char)228, (char)218, (char)184, (char)158, (char)89, (char)176, (char)105, (char)249, (char)97, (char)41, (char)51, (char)210, (char)169, (char)244, (char)58, (char)133, (char)123, (char)221, (char)34, (char)53, (char)76, (char)17, (char)102, (char)54, (char)88, (char)210, (char)211, (char)62, (char)43, (char)135, (char)203, (char)222, (char)136, (char)14, (char)125, (char)214, (char)116, (char)93, (char)119, (char)185, (char)209, (char)212, (char)199, (char)35, (char)105, (char)93, (char)203, (char)244, (char)194, (char)76, (char)228, (char)30, (char)221, (char)147, (char)32, (char)36, (char)214, (char)17, (char)86, (char)231, (char)31, (char)175, (char)151, (char)145, (char)226, (char)172, (char)170, (char)148, (char)142, (char)65, (char)247, (char)228, (char)219, (char)70, (char)97, (char)1, (char)113, (char)201, (char)85, (char)226, (char)89, (char)190, (char)20, (char)154, (char)21, (char)237, (char)44, (char)239, (char)143, (char)36, (char)146, (char)135, (char)179, (char)50, (char)232, (char)124, (char)56, (char)141, (char)182, (char)39, (char)231, (char)159, (char)238, (char)76, (char)237, (char)62, (char)193, (char)121, (char)64, (char)148, (char)245, (char)190, (char)142, (char)167, (char)237, (char)177, (char)124, (char)150, (char)221, (char)4, (char)117, (char)150, (char)32, (char)120, (char)210, (char)177, (char)244, (char)91, (char)64, (char)98, (char)21, (char)234, (char)91, (char)232, (char)118, (char)122, (char)143}, 0) ;
        p248.target_network_SET((char)149) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte) - 96, (byte) - 58, (byte)54, (byte)122, (byte)31, (byte)37, (byte) - 47, (byte) - 55, (byte) - 34, (byte) - 24, (byte) - 106, (byte) - 84, (byte) - 66, (byte)86, (byte) - 108, (byte)12, (byte)62, (byte) - 8, (byte)56, (byte)69, (byte) - 58, (byte)107, (byte) - 84, (byte)11, (byte) - 86, (byte)72, (byte)77, (byte)118, (byte)47, (byte) - 116, (byte) - 46, (byte)52}));
            assert(pack.ver_GET() == (char)24);
            assert(pack.type_GET() == (char)173);
            assert(pack.address_GET() == (char)23128);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.value_SET(new byte[] {(byte) - 96, (byte) - 58, (byte)54, (byte)122, (byte)31, (byte)37, (byte) - 47, (byte) - 55, (byte) - 34, (byte) - 24, (byte) - 106, (byte) - 84, (byte) - 66, (byte)86, (byte) - 108, (byte)12, (byte)62, (byte) - 8, (byte)56, (byte)69, (byte) - 58, (byte)107, (byte) - 84, (byte)11, (byte) - 86, (byte)72, (byte)77, (byte)118, (byte)47, (byte) - 116, (byte) - 46, (byte)52}, 0) ;
        p249.type_SET((char)173) ;
        p249.address_SET((char)23128) ;
        p249.ver_SET((char)24) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -1.2714492E38F);
            assert(pack.y_GET() == 2.104418E38F);
            assert(pack.name_LEN(ph) == 8);
            assert(pack.name_TRY(ph).equals("nGgsbluk"));
            assert(pack.z_GET() == -1.6945086E38F);
            assert(pack.time_usec_GET() == 6144693043248909140L);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("nGgsbluk", PH) ;
        p250.x_SET(-1.2714492E38F) ;
        p250.time_usec_SET(6144693043248909140L) ;
        p250.y_SET(2.104418E38F) ;
        p250.z_SET(-1.6945086E38F) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == 7.0096586E37F);
            assert(pack.name_LEN(ph) == 7);
            assert(pack.name_TRY(ph).equals("zDWtafh"));
            assert(pack.time_boot_ms_GET() == 1196864634L);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.name_SET("zDWtafh", PH) ;
        p251.time_boot_ms_SET(1196864634L) ;
        p251.value_SET(7.0096586E37F) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 10);
            assert(pack.name_TRY(ph).equals("qbTzzosTza"));
            assert(pack.time_boot_ms_GET() == 3882420540L);
            assert(pack.value_GET() == 1313710806);
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.name_SET("qbTzzosTza", PH) ;
        p252.time_boot_ms_SET(3882420540L) ;
        p252.value_SET(1313710806) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 6);
            assert(pack.text_TRY(ph).equals("oGaqkv"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_ALERT);
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("oGaqkv", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_ALERT) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.ind_GET() == (char)222);
            assert(pack.time_boot_ms_GET() == 1692731003L);
            assert(pack.value_GET() == -2.6782247E37F);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.value_SET(-2.6782247E37F) ;
        p254.ind_SET((char)222) ;
        p254.time_boot_ms_SET(1692731003L) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.initial_timestamp_GET() == 6628466647797272473L);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)49, (char)175, (char)158, (char)55, (char)181, (char)18, (char)24, (char)11, (char)112, (char)63, (char)164, (char)38, (char)41, (char)151, (char)212, (char)78, (char)224, (char)64, (char)50, (char)16, (char)40, (char)181, (char)211, (char)37, (char)157, (char)182, (char)12, (char)59, (char)56, (char)36, (char)8, (char)99}));
            assert(pack.target_component_GET() == (char)240);
            assert(pack.target_system_GET() == (char)29);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.initial_timestamp_SET(6628466647797272473L) ;
        p256.secret_key_SET(new char[] {(char)49, (char)175, (char)158, (char)55, (char)181, (char)18, (char)24, (char)11, (char)112, (char)63, (char)164, (char)38, (char)41, (char)151, (char)212, (char)78, (char)224, (char)64, (char)50, (char)16, (char)40, (char)181, (char)211, (char)37, (char)157, (char)182, (char)12, (char)59, (char)56, (char)36, (char)8, (char)99}, 0) ;
        p256.target_system_SET((char)29) ;
        p256.target_component_SET((char)240) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.state_GET() == (char)245);
            assert(pack.last_change_ms_GET() == 4161644247L);
            assert(pack.time_boot_ms_GET() == 213447317L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.last_change_ms_SET(4161644247L) ;
        p257.time_boot_ms_SET(213447317L) ;
        p257.state_SET((char)245) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)138);
            assert(pack.target_system_GET() == (char)239);
            assert(pack.tune_LEN(ph) == 13);
            assert(pack.tune_TRY(ph).equals("uffZPdgtjhlfg"));
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_component_SET((char)138) ;
        p258.tune_SET("uffZPdgtjhlfg", PH) ;
        p258.target_system_SET((char)239) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.sensor_size_h_GET() == 2.3274866E37F);
            assert(pack.lens_id_GET() == (char)166);
            assert(pack.resolution_v_GET() == (char)2232);
            assert(pack.time_boot_ms_GET() == 4023293419L);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)15, (char)157, (char)254, (char)161, (char)21, (char)55, (char)178, (char)149, (char)96, (char)72, (char)246, (char)218, (char)241, (char)145, (char)176, (char)227, (char)250, (char)149, (char)136, (char)10, (char)150, (char)30, (char)114, (char)209, (char)194, (char)41, (char)146, (char)19, (char)29, (char)73, (char)67, (char)19}));
            assert(pack.cam_definition_uri_LEN(ph) == 34);
            assert(pack.cam_definition_uri_TRY(ph).equals("ssxmlcsiXcmrknlgztupkfyebpepcrtudu"));
            assert(pack.focal_length_GET() == -1.7875078E38F);
            assert(pack.sensor_size_v_GET() == -3.0547684E38F);
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE));
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)221, (char)226, (char)55, (char)247, (char)220, (char)188, (char)62, (char)161, (char)185, (char)32, (char)221, (char)103, (char)36, (char)89, (char)129, (char)185, (char)129, (char)242, (char)49, (char)179, (char)164, (char)41, (char)142, (char)132, (char)157, (char)178, (char)238, (char)126, (char)3, (char)191, (char)4, (char)158}));
            assert(pack.firmware_version_GET() == 2386916878L);
            assert(pack.cam_definition_version_GET() == (char)54863);
            assert(pack.resolution_h_GET() == (char)62452);
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.sensor_size_h_SET(2.3274866E37F) ;
        p259.focal_length_SET(-1.7875078E38F) ;
        p259.resolution_v_SET((char)2232) ;
        p259.resolution_h_SET((char)62452) ;
        p259.sensor_size_v_SET(-3.0547684E38F) ;
        p259.model_name_SET(new char[] {(char)15, (char)157, (char)254, (char)161, (char)21, (char)55, (char)178, (char)149, (char)96, (char)72, (char)246, (char)218, (char)241, (char)145, (char)176, (char)227, (char)250, (char)149, (char)136, (char)10, (char)150, (char)30, (char)114, (char)209, (char)194, (char)41, (char)146, (char)19, (char)29, (char)73, (char)67, (char)19}, 0) ;
        p259.lens_id_SET((char)166) ;
        p259.cam_definition_version_SET((char)54863) ;
        p259.firmware_version_SET(2386916878L) ;
        p259.cam_definition_uri_SET("ssxmlcsiXcmrknlgztupkfyebpepcrtudu", PH) ;
        p259.time_boot_ms_SET(4023293419L) ;
        p259.vendor_name_SET(new char[] {(char)221, (char)226, (char)55, (char)247, (char)220, (char)188, (char)62, (char)161, (char)185, (char)32, (char)221, (char)103, (char)36, (char)89, (char)129, (char)185, (char)129, (char)242, (char)49, (char)179, (char)164, (char)41, (char)142, (char)132, (char)157, (char)178, (char)238, (char)126, (char)3, (char)191, (char)4, (char)158}, 0) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE)) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2498966968L);
            assert(pack.mode_id_GET() == (CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY));
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.mode_id_SET((CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY)) ;
        p260.time_boot_ms_SET(2498966968L) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.total_capacity_GET() == 2.415365E38F);
            assert(pack.storage_count_GET() == (char)110);
            assert(pack.time_boot_ms_GET() == 2343918383L);
            assert(pack.available_capacity_GET() == 2.238986E38F);
            assert(pack.storage_id_GET() == (char)51);
            assert(pack.write_speed_GET() == 7.376891E37F);
            assert(pack.read_speed_GET() == -1.2357381E38F);
            assert(pack.used_capacity_GET() == 1.3186231E38F);
            assert(pack.status_GET() == (char)214);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.write_speed_SET(7.376891E37F) ;
        p261.available_capacity_SET(2.238986E38F) ;
        p261.read_speed_SET(-1.2357381E38F) ;
        p261.time_boot_ms_SET(2343918383L) ;
        p261.used_capacity_SET(1.3186231E38F) ;
        p261.storage_count_SET((char)110) ;
        p261.status_SET((char)214) ;
        p261.total_capacity_SET(2.415365E38F) ;
        p261.storage_id_SET((char)51) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3947808863L);
            assert(pack.video_status_GET() == (char)6);
            assert(pack.recording_time_ms_GET() == 4262753695L);
            assert(pack.available_capacity_GET() == 4.5959205E37F);
            assert(pack.image_interval_GET() == -2.5603445E38F);
            assert(pack.image_status_GET() == (char)156);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.available_capacity_SET(4.5959205E37F) ;
        p262.recording_time_ms_SET(4262753695L) ;
        p262.image_status_SET((char)156) ;
        p262.time_boot_ms_SET(3947808863L) ;
        p262.video_status_SET((char)6) ;
        p262.image_interval_SET(-2.5603445E38F) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2526592427L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.2713697E38F, 7.0424623E37F, -6.0947936E37F, -2.1066023E38F}));
            assert(pack.image_index_GET() == -639184043);
            assert(pack.lon_GET() == 513656430);
            assert(pack.time_utc_GET() == 697135830239316750L);
            assert(pack.file_url_LEN(ph) == 195);
            assert(pack.file_url_TRY(ph).equals("FvbgvxhKhieXizegzqwqtruubvkuwpgkyxtqladvjnopnKmVDkgqdsabqoohpddmngzpdkhcccbOolqafwGuefvGzaRbkdqtnrkppkiktnqzuyAddwbhkIlaehzUfdMhTvmjojylummnzGgiufhtidmDPmyfkpmLsuffMzoxdsxrvamdulBphpsonsdjtbpafmm"));
            assert(pack.relative_alt_GET() == 1640922871);
            assert(pack.lat_GET() == 292443361);
            assert(pack.alt_GET() == -135488909);
            assert(pack.capture_result_GET() == (byte)101);
            assert(pack.camera_id_GET() == (char)121);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.file_url_SET("FvbgvxhKhieXizegzqwqtruubvkuwpgkyxtqladvjnopnKmVDkgqdsabqoohpddmngzpdkhcccbOolqafwGuefvGzaRbkdqtnrkppkiktnqzuyAddwbhkIlaehzUfdMhTvmjojylummnzGgiufhtidmDPmyfkpmLsuffMzoxdsxrvamdulBphpsonsdjtbpafmm", PH) ;
        p263.relative_alt_SET(1640922871) ;
        p263.time_boot_ms_SET(2526592427L) ;
        p263.capture_result_SET((byte)101) ;
        p263.camera_id_SET((char)121) ;
        p263.time_utc_SET(697135830239316750L) ;
        p263.lat_SET(292443361) ;
        p263.image_index_SET(-639184043) ;
        p263.lon_SET(513656430) ;
        p263.q_SET(new float[] {2.2713697E38F, 7.0424623E37F, -6.0947936E37F, -2.1066023E38F}, 0) ;
        p263.alt_SET(-135488909) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.flight_uuid_GET() == 568323124470525102L);
            assert(pack.arming_time_utc_GET() == 8390190437143687263L);
            assert(pack.time_boot_ms_GET() == 4197818864L);
            assert(pack.takeoff_time_utc_GET() == 4814834877092435978L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.time_boot_ms_SET(4197818864L) ;
        p264.takeoff_time_utc_SET(4814834877092435978L) ;
        p264.flight_uuid_SET(568323124470525102L) ;
        p264.arming_time_utc_SET(8390190437143687263L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -2.6541007E38F);
            assert(pack.yaw_GET() == 2.160668E38F);
            assert(pack.time_boot_ms_GET() == 180170997L);
            assert(pack.pitch_GET() == -2.4148012E38F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(180170997L) ;
        p265.yaw_SET(2.160668E38F) ;
        p265.pitch_SET(-2.4148012E38F) ;
        p265.roll_SET(-2.6541007E38F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)11384);
            assert(pack.first_message_offset_GET() == (char)216);
            assert(pack.target_component_GET() == (char)73);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)184, (char)16, (char)246, (char)242, (char)202, (char)35, (char)54, (char)2, (char)96, (char)182, (char)58, (char)27, (char)255, (char)156, (char)152, (char)200, (char)86, (char)200, (char)117, (char)100, (char)76, (char)140, (char)190, (char)72, (char)151, (char)189, (char)163, (char)140, (char)83, (char)244, (char)122, (char)218, (char)210, (char)233, (char)151, (char)80, (char)121, (char)63, (char)140, (char)187, (char)240, (char)65, (char)227, (char)241, (char)134, (char)29, (char)180, (char)221, (char)78, (char)179, (char)170, (char)251, (char)220, (char)163, (char)182, (char)112, (char)166, (char)87, (char)127, (char)51, (char)143, (char)171, (char)2, (char)104, (char)98, (char)46, (char)65, (char)157, (char)164, (char)181, (char)247, (char)14, (char)150, (char)57, (char)62, (char)222, (char)188, (char)207, (char)187, (char)112, (char)203, (char)101, (char)108, (char)40, (char)119, (char)39, (char)71, (char)147, (char)0, (char)64, (char)199, (char)131, (char)71, (char)27, (char)134, (char)16, (char)61, (char)141, (char)115, (char)160, (char)211, (char)211, (char)95, (char)220, (char)40, (char)17, (char)185, (char)196, (char)84, (char)55, (char)96, (char)22, (char)116, (char)164, (char)201, (char)112, (char)134, (char)35, (char)131, (char)29, (char)226, (char)210, (char)38, (char)204, (char)121, (char)167, (char)96, (char)179, (char)90, (char)101, (char)175, (char)229, (char)179, (char)107, (char)17, (char)183, (char)152, (char)16, (char)23, (char)142, (char)19, (char)138, (char)65, (char)58, (char)235, (char)91, (char)180, (char)173, (char)187, (char)196, (char)41, (char)143, (char)6, (char)37, (char)23, (char)66, (char)118, (char)158, (char)23, (char)27, (char)207, (char)122, (char)69, (char)42, (char)152, (char)218, (char)119, (char)144, (char)225, (char)227, (char)57, (char)216, (char)212, (char)220, (char)11, (char)85, (char)74, (char)169, (char)132, (char)194, (char)244, (char)169, (char)245, (char)197, (char)193, (char)130, (char)13, (char)92, (char)95, (char)143, (char)236, (char)196, (char)106, (char)30, (char)216, (char)5, (char)169, (char)53, (char)163, (char)104, (char)165, (char)184, (char)84, (char)172, (char)199, (char)75, (char)225, (char)87, (char)248, (char)44, (char)208, (char)125, (char)55, (char)78, (char)112, (char)143, (char)112, (char)104, (char)186, (char)197, (char)185, (char)99, (char)101, (char)66, (char)250, (char)48, (char)157, (char)198, (char)3, (char)98, (char)135, (char)78, (char)247, (char)210, (char)238, (char)216, (char)232, (char)145, (char)206, (char)183, (char)27, (char)238, (char)233, (char)152, (char)172, (char)38, (char)117, (char)105, (char)111}));
            assert(pack.length_GET() == (char)230);
            assert(pack.target_system_GET() == (char)115);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.first_message_offset_SET((char)216) ;
        p266.data__SET(new char[] {(char)184, (char)16, (char)246, (char)242, (char)202, (char)35, (char)54, (char)2, (char)96, (char)182, (char)58, (char)27, (char)255, (char)156, (char)152, (char)200, (char)86, (char)200, (char)117, (char)100, (char)76, (char)140, (char)190, (char)72, (char)151, (char)189, (char)163, (char)140, (char)83, (char)244, (char)122, (char)218, (char)210, (char)233, (char)151, (char)80, (char)121, (char)63, (char)140, (char)187, (char)240, (char)65, (char)227, (char)241, (char)134, (char)29, (char)180, (char)221, (char)78, (char)179, (char)170, (char)251, (char)220, (char)163, (char)182, (char)112, (char)166, (char)87, (char)127, (char)51, (char)143, (char)171, (char)2, (char)104, (char)98, (char)46, (char)65, (char)157, (char)164, (char)181, (char)247, (char)14, (char)150, (char)57, (char)62, (char)222, (char)188, (char)207, (char)187, (char)112, (char)203, (char)101, (char)108, (char)40, (char)119, (char)39, (char)71, (char)147, (char)0, (char)64, (char)199, (char)131, (char)71, (char)27, (char)134, (char)16, (char)61, (char)141, (char)115, (char)160, (char)211, (char)211, (char)95, (char)220, (char)40, (char)17, (char)185, (char)196, (char)84, (char)55, (char)96, (char)22, (char)116, (char)164, (char)201, (char)112, (char)134, (char)35, (char)131, (char)29, (char)226, (char)210, (char)38, (char)204, (char)121, (char)167, (char)96, (char)179, (char)90, (char)101, (char)175, (char)229, (char)179, (char)107, (char)17, (char)183, (char)152, (char)16, (char)23, (char)142, (char)19, (char)138, (char)65, (char)58, (char)235, (char)91, (char)180, (char)173, (char)187, (char)196, (char)41, (char)143, (char)6, (char)37, (char)23, (char)66, (char)118, (char)158, (char)23, (char)27, (char)207, (char)122, (char)69, (char)42, (char)152, (char)218, (char)119, (char)144, (char)225, (char)227, (char)57, (char)216, (char)212, (char)220, (char)11, (char)85, (char)74, (char)169, (char)132, (char)194, (char)244, (char)169, (char)245, (char)197, (char)193, (char)130, (char)13, (char)92, (char)95, (char)143, (char)236, (char)196, (char)106, (char)30, (char)216, (char)5, (char)169, (char)53, (char)163, (char)104, (char)165, (char)184, (char)84, (char)172, (char)199, (char)75, (char)225, (char)87, (char)248, (char)44, (char)208, (char)125, (char)55, (char)78, (char)112, (char)143, (char)112, (char)104, (char)186, (char)197, (char)185, (char)99, (char)101, (char)66, (char)250, (char)48, (char)157, (char)198, (char)3, (char)98, (char)135, (char)78, (char)247, (char)210, (char)238, (char)216, (char)232, (char)145, (char)206, (char)183, (char)27, (char)238, (char)233, (char)152, (char)172, (char)38, (char)117, (char)105, (char)111}, 0) ;
        p266.target_system_SET((char)115) ;
        p266.length_SET((char)230) ;
        p266.sequence_SET((char)11384) ;
        p266.target_component_SET((char)73) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)23643);
            assert(pack.length_GET() == (char)137);
            assert(pack.target_system_GET() == (char)251);
            assert(pack.first_message_offset_GET() == (char)23);
            assert(pack.target_component_GET() == (char)67);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)237, (char)108, (char)30, (char)109, (char)132, (char)247, (char)183, (char)37, (char)217, (char)104, (char)118, (char)240, (char)158, (char)20, (char)146, (char)136, (char)182, (char)31, (char)2, (char)225, (char)107, (char)15, (char)212, (char)79, (char)132, (char)233, (char)174, (char)98, (char)3, (char)58, (char)255, (char)31, (char)5, (char)183, (char)187, (char)80, (char)77, (char)128, (char)155, (char)151, (char)28, (char)12, (char)64, (char)191, (char)24, (char)43, (char)160, (char)145, (char)63, (char)61, (char)48, (char)185, (char)50, (char)189, (char)31, (char)85, (char)78, (char)1, (char)163, (char)82, (char)231, (char)215, (char)228, (char)128, (char)125, (char)188, (char)45, (char)76, (char)98, (char)212, (char)144, (char)73, (char)182, (char)83, (char)110, (char)164, (char)95, (char)143, (char)127, (char)104, (char)13, (char)42, (char)7, (char)168, (char)101, (char)165, (char)44, (char)68, (char)26, (char)239, (char)78, (char)190, (char)42, (char)243, (char)44, (char)14, (char)251, (char)120, (char)186, (char)17, (char)168, (char)21, (char)37, (char)72, (char)173, (char)202, (char)84, (char)77, (char)198, (char)43, (char)30, (char)32, (char)115, (char)92, (char)175, (char)31, (char)187, (char)59, (char)225, (char)237, (char)236, (char)98, (char)31, (char)225, (char)105, (char)3, (char)241, (char)192, (char)127, (char)72, (char)48, (char)18, (char)115, (char)160, (char)250, (char)241, (char)165, (char)190, (char)137, (char)172, (char)70, (char)147, (char)99, (char)250, (char)201, (char)202, (char)117, (char)49, (char)184, (char)120, (char)63, (char)162, (char)86, (char)79, (char)126, (char)78, (char)128, (char)209, (char)251, (char)69, (char)191, (char)34, (char)120, (char)195, (char)118, (char)95, (char)61, (char)178, (char)113, (char)207, (char)221, (char)62, (char)11, (char)158, (char)208, (char)216, (char)11, (char)83, (char)71, (char)37, (char)55, (char)34, (char)196, (char)19, (char)177, (char)67, (char)68, (char)98, (char)1, (char)63, (char)189, (char)8, (char)52, (char)130, (char)163, (char)233, (char)12, (char)15, (char)135, (char)206, (char)186, (char)69, (char)205, (char)12, (char)194, (char)218, (char)208, (char)124, (char)194, (char)108, (char)19, (char)171, (char)215, (char)192, (char)38, (char)96, (char)241, (char)166, (char)159, (char)61, (char)80, (char)127, (char)23, (char)240, (char)156, (char)150, (char)181, (char)28, (char)125, (char)217, (char)32, (char)177, (char)38, (char)175, (char)20, (char)26, (char)240, (char)162, (char)186, (char)6, (char)156, (char)8, (char)83, (char)72, (char)181, (char)27, (char)115, (char)186, (char)45}));
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.length_SET((char)137) ;
        p267.first_message_offset_SET((char)23) ;
        p267.sequence_SET((char)23643) ;
        p267.target_system_SET((char)251) ;
        p267.data__SET(new char[] {(char)237, (char)108, (char)30, (char)109, (char)132, (char)247, (char)183, (char)37, (char)217, (char)104, (char)118, (char)240, (char)158, (char)20, (char)146, (char)136, (char)182, (char)31, (char)2, (char)225, (char)107, (char)15, (char)212, (char)79, (char)132, (char)233, (char)174, (char)98, (char)3, (char)58, (char)255, (char)31, (char)5, (char)183, (char)187, (char)80, (char)77, (char)128, (char)155, (char)151, (char)28, (char)12, (char)64, (char)191, (char)24, (char)43, (char)160, (char)145, (char)63, (char)61, (char)48, (char)185, (char)50, (char)189, (char)31, (char)85, (char)78, (char)1, (char)163, (char)82, (char)231, (char)215, (char)228, (char)128, (char)125, (char)188, (char)45, (char)76, (char)98, (char)212, (char)144, (char)73, (char)182, (char)83, (char)110, (char)164, (char)95, (char)143, (char)127, (char)104, (char)13, (char)42, (char)7, (char)168, (char)101, (char)165, (char)44, (char)68, (char)26, (char)239, (char)78, (char)190, (char)42, (char)243, (char)44, (char)14, (char)251, (char)120, (char)186, (char)17, (char)168, (char)21, (char)37, (char)72, (char)173, (char)202, (char)84, (char)77, (char)198, (char)43, (char)30, (char)32, (char)115, (char)92, (char)175, (char)31, (char)187, (char)59, (char)225, (char)237, (char)236, (char)98, (char)31, (char)225, (char)105, (char)3, (char)241, (char)192, (char)127, (char)72, (char)48, (char)18, (char)115, (char)160, (char)250, (char)241, (char)165, (char)190, (char)137, (char)172, (char)70, (char)147, (char)99, (char)250, (char)201, (char)202, (char)117, (char)49, (char)184, (char)120, (char)63, (char)162, (char)86, (char)79, (char)126, (char)78, (char)128, (char)209, (char)251, (char)69, (char)191, (char)34, (char)120, (char)195, (char)118, (char)95, (char)61, (char)178, (char)113, (char)207, (char)221, (char)62, (char)11, (char)158, (char)208, (char)216, (char)11, (char)83, (char)71, (char)37, (char)55, (char)34, (char)196, (char)19, (char)177, (char)67, (char)68, (char)98, (char)1, (char)63, (char)189, (char)8, (char)52, (char)130, (char)163, (char)233, (char)12, (char)15, (char)135, (char)206, (char)186, (char)69, (char)205, (char)12, (char)194, (char)218, (char)208, (char)124, (char)194, (char)108, (char)19, (char)171, (char)215, (char)192, (char)38, (char)96, (char)241, (char)166, (char)159, (char)61, (char)80, (char)127, (char)23, (char)240, (char)156, (char)150, (char)181, (char)28, (char)125, (char)217, (char)32, (char)177, (char)38, (char)175, (char)20, (char)26, (char)240, (char)162, (char)186, (char)6, (char)156, (char)8, (char)83, (char)72, (char)181, (char)27, (char)115, (char)186, (char)45}, 0) ;
        p267.target_component_SET((char)67) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)37594);
            assert(pack.target_system_GET() == (char)144);
            assert(pack.target_component_GET() == (char)12);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)144) ;
        p268.target_component_SET((char)12) ;
        p268.sequence_SET((char)37594) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.bitrate_GET() == 28624140L);
            assert(pack.framerate_GET() == -1.7420468E38F);
            assert(pack.uri_LEN(ph) == 172);
            assert(pack.uri_TRY(ph).equals("cxUmynxDeqxYsbvGjwioalhnVlhrzqhdscroqtsvowagciqhzvsGvpgxNKjtjqrmcpxegdybnnzfSygiinyolijuacvnvrJsMjoTbppaqmsfdgaBxjnpdlxarqghlshbzpiEmdbdoZyzdLzekvwDpgkoRuzaRaoJdomtfpoxfGmy"));
            assert(pack.camera_id_GET() == (char)153);
            assert(pack.resolution_v_GET() == (char)47670);
            assert(pack.rotation_GET() == (char)33181);
            assert(pack.status_GET() == (char)226);
            assert(pack.resolution_h_GET() == (char)18659);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.rotation_SET((char)33181) ;
        p269.framerate_SET(-1.7420468E38F) ;
        p269.bitrate_SET(28624140L) ;
        p269.resolution_v_SET((char)47670) ;
        p269.camera_id_SET((char)153) ;
        p269.uri_SET("cxUmynxDeqxYsbvGjwioalhnVlhrzqhdscroqtsvowagciqhzvsGvpgxNKjtjqrmcpxegdybnnzfSygiinyolijuacvnvrJsMjoTbppaqmsfdgaBxjnpdlxarqghlshbzpiEmdbdoZyzdLzekvwDpgkoRuzaRaoJdomtfpoxfGmy", PH) ;
        p269.status_SET((char)226) ;
        p269.resolution_h_SET((char)18659) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.uri_LEN(ph) == 227);
            assert(pack.uri_TRY(ph).equals("wflqbtmvyhtbbrijGqkQzvgqkuKhbuvzwrlslhpaPcrondaXGjfudtNjVxnmmzsmhsqdgdqcfqbojwmmcugdcaoFpkwhrajaxgibhcuwjcooqKinzexhnbdlgdweqgzpcagneqspivcltgtzlFsenndyzroPcsctxntsykVVpcBgpEwaucophovxsJbgkefszzrjkaodoykfijtslzijtArfqTszkhozpUq"));
            assert(pack.resolution_h_GET() == (char)43379);
            assert(pack.camera_id_GET() == (char)106);
            assert(pack.resolution_v_GET() == (char)35284);
            assert(pack.target_system_GET() == (char)247);
            assert(pack.target_component_GET() == (char)108);
            assert(pack.framerate_GET() == 3.2941042E37F);
            assert(pack.bitrate_GET() == 532137600L);
            assert(pack.rotation_GET() == (char)45288);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.camera_id_SET((char)106) ;
        p270.framerate_SET(3.2941042E37F) ;
        p270.bitrate_SET(532137600L) ;
        p270.uri_SET("wflqbtmvyhtbbrijGqkQzvgqkuKhbuvzwrlslhpaPcrondaXGjfudtNjVxnmmzsmhsqdgdqcfqbojwmmcugdcaoFpkwhrajaxgibhcuwjcooqKinzexhnbdlgdweqgzpcagneqspivcltgtzlFsenndyzroPcsctxntsykVVpcBgpEwaucophovxsJbgkefszzrjkaodoykfijtslzijtArfqTszkhozpUq", PH) ;
        p270.resolution_h_SET((char)43379) ;
        p270.target_system_SET((char)247) ;
        p270.rotation_SET((char)45288) ;
        p270.resolution_v_SET((char)35284) ;
        p270.target_component_SET((char)108) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.password_LEN(ph) == 7);
            assert(pack.password_TRY(ph).equals("dkmorvm"));
            assert(pack.ssid_LEN(ph) == 3);
            assert(pack.ssid_TRY(ph).equals("xcw"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("xcw", PH) ;
        p299.password_SET("dkmorvm", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.max_version_GET() == (char)47287);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)89, (char)39, (char)243, (char)246, (char)158, (char)46, (char)16, (char)140}));
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)13, (char)112, (char)82, (char)208, (char)96, (char)68, (char)183, (char)252}));
            assert(pack.min_version_GET() == (char)62583);
            assert(pack.version_GET() == (char)21016);
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.library_version_hash_SET(new char[] {(char)89, (char)39, (char)243, (char)246, (char)158, (char)46, (char)16, (char)140}, 0) ;
        p300.spec_version_hash_SET(new char[] {(char)13, (char)112, (char)82, (char)208, (char)96, (char)68, (char)183, (char)252}, 0) ;
        p300.version_SET((char)21016) ;
        p300.min_version_SET((char)62583) ;
        p300.max_version_SET((char)47287) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.vendor_specific_status_code_GET() == (char)61470);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
            assert(pack.uptime_sec_GET() == 1430518380L);
            assert(pack.sub_mode_GET() == (char)41);
            assert(pack.time_usec_GET() == 3998220834146836877L);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE) ;
        p310.sub_mode_SET((char)41) ;
        p310.vendor_specific_status_code_SET((char)61470) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK) ;
        p310.uptime_sec_SET(1430518380L) ;
        p310.time_usec_SET(3998220834146836877L) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.hw_version_major_GET() == (char)9);
            assert(pack.sw_version_major_GET() == (char)130);
            assert(pack.name_LEN(ph) == 44);
            assert(pack.name_TRY(ph).equals("kNvfgakFezwBeielbbucrrswarvzxhjjrxxwyxnludvt"));
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)109, (char)39, (char)70, (char)200, (char)58, (char)71, (char)149, (char)34, (char)48, (char)147, (char)140, (char)229, (char)92, (char)88, (char)123, (char)203}));
            assert(pack.time_usec_GET() == 6791416175210364946L);
            assert(pack.sw_vcs_commit_GET() == 4031116215L);
            assert(pack.hw_version_minor_GET() == (char)164);
            assert(pack.sw_version_minor_GET() == (char)176);
            assert(pack.uptime_sec_GET() == 528784020L);
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.hw_version_major_SET((char)9) ;
        p311.hw_unique_id_SET(new char[] {(char)109, (char)39, (char)70, (char)200, (char)58, (char)71, (char)149, (char)34, (char)48, (char)147, (char)140, (char)229, (char)92, (char)88, (char)123, (char)203}, 0) ;
        p311.name_SET("kNvfgakFezwBeielbbucrrswarvzxhjjrxxwyxnludvt", PH) ;
        p311.sw_version_major_SET((char)130) ;
        p311.time_usec_SET(6791416175210364946L) ;
        p311.sw_vcs_commit_SET(4031116215L) ;
        p311.hw_version_minor_SET((char)164) ;
        p311.uptime_sec_SET(528784020L) ;
        p311.sw_version_minor_SET((char)176) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)112);
            assert(pack.param_index_GET() == (short) -8790);
            assert(pack.target_component_GET() == (char)149);
            assert(pack.param_id_LEN(ph) == 6);
            assert(pack.param_id_TRY(ph).equals("sikhhh"));
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_component_SET((char)149) ;
        p320.param_id_SET("sikhhh", PH) ;
        p320.target_system_SET((char)112) ;
        p320.param_index_SET((short) -8790) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)87);
            assert(pack.target_component_GET() == (char)70);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)70) ;
        p321.target_system_SET((char)87) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)5242);
            assert(pack.param_value_LEN(ph) == 23);
            assert(pack.param_value_TRY(ph).equals("xdhjeekrepnoxjcwLemAqwa"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
            assert(pack.param_id_LEN(ph) == 15);
            assert(pack.param_id_TRY(ph).equals("odqryqvVyajohec"));
            assert(pack.param_index_GET() == (char)49193);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_index_SET((char)49193) ;
        p322.param_value_SET("xdhjeekrepnoxjcwLemAqwa", PH) ;
        p322.param_count_SET((char)5242) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16) ;
        p322.param_id_SET("odqryqvVyajohec", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("wojhrnhy"));
            assert(pack.target_system_GET() == (char)8);
            assert(pack.target_component_GET() == (char)179);
            assert(pack.param_value_LEN(ph) == 20);
            assert(pack.param_value_TRY(ph).equals("lbpmsatoudsygwRvfCgD"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_value_SET("lbpmsatoudsygwRvfCgD", PH) ;
        p323.param_id_SET("wojhrnhy", PH) ;
        p323.target_system_SET((char)8) ;
        p323.target_component_SET((char)179) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 98);
            assert(pack.param_value_TRY(ph).equals("tisBhLsNfxvtkpefltpkfyvlezpzpytFzxkeastnsabjojbharHyiaziplhvqnSrlPwnjublklzQhhwtxMzqqaynRlqawsyuej"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_IN_PROGRESS);
            assert(pack.param_id_LEN(ph) == 10);
            assert(pack.param_id_TRY(ph).equals("hfossroaii"));
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("hfossroaii", PH) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_IN_PROGRESS) ;
        p324.param_value_SET("tisBhLsNfxvtkpefltpkfyvlezpzpytFzxkeastnsabjojbharHyiaziplhvqnSrlPwnjublklzQhhwtxMzqqaynRlqawsyuej", PH) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.increment_GET() == (char)24);
            assert(pack.min_distance_GET() == (char)4637);
            assert(pack.time_usec_GET() == 336745438042612298L);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)17088, (char)11426, (char)63105, (char)29203, (char)33699, (char)2070, (char)24771, (char)20992, (char)64305, (char)6070, (char)7830, (char)62030, (char)18604, (char)40966, (char)41941, (char)36689, (char)15383, (char)28888, (char)57990, (char)57199, (char)27055, (char)31413, (char)12062, (char)62502, (char)63915, (char)5939, (char)41864, (char)53153, (char)52397, (char)12675, (char)9568, (char)8219, (char)25373, (char)59450, (char)41221, (char)16475, (char)16503, (char)52014, (char)12688, (char)49687, (char)51951, (char)53353, (char)34714, (char)46935, (char)18398, (char)37697, (char)29680, (char)15594, (char)44709, (char)31190, (char)2759, (char)27935, (char)26255, (char)21938, (char)1444, (char)35563, (char)62090, (char)37616, (char)39091, (char)47036, (char)29707, (char)22749, (char)46208, (char)40844, (char)37615, (char)35774, (char)6959, (char)63039, (char)37688, (char)6576, (char)52043, (char)26659}));
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
            assert(pack.max_distance_GET() == (char)35385);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.time_usec_SET(336745438042612298L) ;
        p330.increment_SET((char)24) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED) ;
        p330.max_distance_SET((char)35385) ;
        p330.distances_SET(new char[] {(char)17088, (char)11426, (char)63105, (char)29203, (char)33699, (char)2070, (char)24771, (char)20992, (char)64305, (char)6070, (char)7830, (char)62030, (char)18604, (char)40966, (char)41941, (char)36689, (char)15383, (char)28888, (char)57990, (char)57199, (char)27055, (char)31413, (char)12062, (char)62502, (char)63915, (char)5939, (char)41864, (char)53153, (char)52397, (char)12675, (char)9568, (char)8219, (char)25373, (char)59450, (char)41221, (char)16475, (char)16503, (char)52014, (char)12688, (char)49687, (char)51951, (char)53353, (char)34714, (char)46935, (char)18398, (char)37697, (char)29680, (char)15594, (char)44709, (char)31190, (char)2759, (char)27935, (char)26255, (char)21938, (char)1444, (char)35563, (char)62090, (char)37616, (char)39091, (char)47036, (char)29707, (char)22749, (char)46208, (char)40844, (char)37615, (char)35774, (char)6959, (char)63039, (char)37688, (char)6576, (char)52043, (char)26659}, 0) ;
        p330.min_distance_SET((char)4637) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
    }

}
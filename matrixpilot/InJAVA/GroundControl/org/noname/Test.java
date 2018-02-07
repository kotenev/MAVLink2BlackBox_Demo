
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
            long id = id__o(src);
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
        {  return  0 + (int)get_bits(data, 132, 2); }
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
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_UNINIT);
            assert(pack.mavlink_version_GET() == (char)156);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED));
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_FIXED_WING);
            assert(pack.custom_mode_GET() == 1877198821L);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.system_status_SET(MAV_STATE.MAV_STATE_UNINIT) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED)) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_FIXED_WING) ;
        p0.mavlink_version_SET((char)156) ;
        p0.custom_mode_SET(1877198821L) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.errors_count1_GET() == (char)39897);
            assert(pack.drop_rate_comm_GET() == (char)14573);
            assert(pack.errors_comm_GET() == (char)73);
            assert(pack.load_GET() == (char)54777);
            assert(pack.errors_count2_GET() == (char)41696);
            assert(pack.voltage_battery_GET() == (char)29087);
            assert(pack.battery_remaining_GET() == (byte)127);
            assert(pack.errors_count4_GET() == (char)53315);
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.current_battery_GET() == (short)23272);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY));
            assert(pack.errors_count3_GET() == (char)41207);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.battery_remaining_SET((byte)127) ;
        p1.load_SET((char)54777) ;
        p1.errors_comm_SET((char)73) ;
        p1.drop_rate_comm_SET((char)14573) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        p1.voltage_battery_SET((char)29087) ;
        p1.errors_count2_SET((char)41696) ;
        p1.errors_count4_SET((char)53315) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        p1.errors_count1_SET((char)39897) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY)) ;
        p1.current_battery_SET((short)23272) ;
        p1.errors_count3_SET((char)41207) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 6296952874045796488L);
            assert(pack.time_boot_ms_GET() == 3315490877L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(3315490877L) ;
        p2.time_unix_usec_SET(6296952874045796488L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == 3.2557431E38F);
            assert(pack.afx_GET() == -2.5611945E37F);
            assert(pack.z_GET() == -1.892161E38F);
            assert(pack.yaw_rate_GET() == -2.6410934E38F);
            assert(pack.afy_GET() == -7.1812417E37F);
            assert(pack.x_GET() == 2.1243666E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.yaw_GET() == -1.0458973E38F);
            assert(pack.vz_GET() == -2.4934574E38F);
            assert(pack.vy_GET() == -3.0104878E38F);
            assert(pack.y_GET() == -3.1060147E38F);
            assert(pack.type_mask_GET() == (char)9480);
            assert(pack.afz_GET() == 3.586893E37F);
            assert(pack.time_boot_ms_GET() == 199683713L);
        });
        POSITION_TARGET_LOCAL_NED p3 = new POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.yaw_rate_SET(-2.6410934E38F) ;
        p3.vz_SET(-2.4934574E38F) ;
        p3.z_SET(-1.892161E38F) ;
        p3.afz_SET(3.586893E37F) ;
        p3.type_mask_SET((char)9480) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p3.y_SET(-3.1060147E38F) ;
        p3.yaw_SET(-1.0458973E38F) ;
        p3.x_SET(2.1243666E38F) ;
        p3.vy_SET(-3.0104878E38F) ;
        p3.afx_SET(-2.5611945E37F) ;
        p3.afy_SET(-7.1812417E37F) ;
        p3.vx_SET(3.2557431E38F) ;
        p3.time_boot_ms_SET(199683713L) ;
        TestChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)150);
            assert(pack.time_usec_GET() == 3923127075918589557L);
            assert(pack.target_system_GET() == (char)81);
            assert(pack.seq_GET() == 3745239350L);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.target_system_SET((char)81) ;
        p4.target_component_SET((char)150) ;
        p4.time_usec_SET(3923127075918589557L) ;
        p4.seq_SET(3745239350L) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.passkey_LEN(ph) == 11);
            assert(pack.passkey_TRY(ph).equals("vmVowxjhytZ"));
            assert(pack.version_GET() == (char)183);
            assert(pack.target_system_GET() == (char)190);
            assert(pack.control_request_GET() == (char)45);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.target_system_SET((char)190) ;
        p5.version_SET((char)183) ;
        p5.control_request_SET((char)45) ;
        p5.passkey_SET("vmVowxjhytZ", PH) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.ack_GET() == (char)184);
            assert(pack.control_request_GET() == (char)190);
            assert(pack.gcs_system_id_GET() == (char)50);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.gcs_system_id_SET((char)50) ;
        p6.control_request_SET((char)190) ;
        p6.ack_SET((char)184) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 6);
            assert(pack.key_TRY(ph).equals("irtaxd"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("irtaxd", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.custom_mode_GET() == 1466577720L);
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_AUTO_ARMED);
            assert(pack.target_system_GET() == (char)42);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED) ;
        p11.target_system_SET((char)42) ;
        p11.custom_mode_SET(1466577720L) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 16);
            assert(pack.param_id_TRY(ph).equals("uNuusupyeeavtGnw"));
            assert(pack.target_system_GET() == (char)134);
            assert(pack.param_index_GET() == (short)27410);
            assert(pack.target_component_GET() == (char)124);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_system_SET((char)134) ;
        p20.target_component_SET((char)124) ;
        p20.param_id_SET("uNuusupyeeavtGnw", PH) ;
        p20.param_index_SET((short)27410) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)99);
            assert(pack.target_component_GET() == (char)172);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)172) ;
        p21.target_system_SET((char)99) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 14);
            assert(pack.param_id_TRY(ph).equals("jaacuvkpahazjr"));
            assert(pack.param_value_GET() == -1.3057976E38F);
            assert(pack.param_index_GET() == (char)3471);
            assert(pack.param_count_GET() == (char)56956);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32);
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_count_SET((char)56956) ;
        p22.param_value_SET(-1.3057976E38F) ;
        p22.param_index_SET((char)3471) ;
        p22.param_id_SET("jaacuvkpahazjr", PH) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 16);
            assert(pack.param_id_TRY(ph).equals("kxdqakkQduincnGD"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
            assert(pack.target_component_GET() == (char)91);
            assert(pack.param_value_GET() == -2.4955734E38F);
            assert(pack.target_system_GET() == (char)167);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.target_component_SET((char)91) ;
        p23.param_value_SET(-2.4955734E38F) ;
        p23.target_system_SET((char)167) ;
        p23.param_id_SET("kxdqakkQduincnGD", PH) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.vel_GET() == (char)6949);
            assert(pack.cog_GET() == (char)20205);
            assert(pack.lon_GET() == 744893445);
            assert(pack.eph_GET() == (char)16313);
            assert(pack.lat_GET() == -997213555);
            assert(pack.v_acc_TRY(ph) == 2422742041L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
            assert(pack.h_acc_TRY(ph) == 2508532150L);
            assert(pack.epv_GET() == (char)42405);
            assert(pack.vel_acc_TRY(ph) == 1532754258L);
            assert(pack.alt_GET() == -250222348);
            assert(pack.hdg_acc_TRY(ph) == 2377160513L);
            assert(pack.satellites_visible_GET() == (char)55);
            assert(pack.alt_ellipsoid_TRY(ph) == -1381887871);
            assert(pack.time_usec_GET() == 3751802628862054600L);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.cog_SET((char)20205) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX) ;
        p24.vel_acc_SET(1532754258L, PH) ;
        p24.lat_SET(-997213555) ;
        p24.v_acc_SET(2422742041L, PH) ;
        p24.alt_ellipsoid_SET(-1381887871, PH) ;
        p24.hdg_acc_SET(2377160513L, PH) ;
        p24.alt_SET(-250222348) ;
        p24.time_usec_SET(3751802628862054600L) ;
        p24.epv_SET((char)42405) ;
        p24.satellites_visible_SET((char)55) ;
        p24.eph_SET((char)16313) ;
        p24.vel_SET((char)6949) ;
        p24.lon_SET(744893445) ;
        p24.h_acc_SET(2508532150L, PH) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.satellites_visible_GET() == (char)110);
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)135, (char)99, (char)171, (char)155, (char)223, (char)255, (char)25, (char)155, (char)136, (char)194, (char)217, (char)33, (char)222, (char)109, (char)188, (char)14, (char)30, (char)84, (char)28, (char)226}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)60, (char)189, (char)231, (char)26, (char)229, (char)33, (char)7, (char)252, (char)179, (char)19, (char)238, (char)171, (char)142, (char)160, (char)39, (char)7, (char)223, (char)118, (char)11, (char)179}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)130, (char)168, (char)251, (char)206, (char)9, (char)225, (char)244, (char)130, (char)203, (char)46, (char)238, (char)163, (char)183, (char)205, (char)55, (char)39, (char)191, (char)177, (char)95, (char)247}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)0, (char)203, (char)125, (char)74, (char)22, (char)127, (char)94, (char)42, (char)48, (char)126, (char)108, (char)202, (char)144, (char)172, (char)221, (char)177, (char)107, (char)94, (char)159, (char)183}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)175, (char)29, (char)24, (char)209, (char)117, (char)255, (char)172, (char)59, (char)232, (char)170, (char)78, (char)3, (char)23, (char)183, (char)108, (char)86, (char)58, (char)191, (char)61, (char)175}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellites_visible_SET((char)110) ;
        p25.satellite_elevation_SET(new char[] {(char)175, (char)29, (char)24, (char)209, (char)117, (char)255, (char)172, (char)59, (char)232, (char)170, (char)78, (char)3, (char)23, (char)183, (char)108, (char)86, (char)58, (char)191, (char)61, (char)175}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)0, (char)203, (char)125, (char)74, (char)22, (char)127, (char)94, (char)42, (char)48, (char)126, (char)108, (char)202, (char)144, (char)172, (char)221, (char)177, (char)107, (char)94, (char)159, (char)183}, 0) ;
        p25.satellite_used_SET(new char[] {(char)135, (char)99, (char)171, (char)155, (char)223, (char)255, (char)25, (char)155, (char)136, (char)194, (char)217, (char)33, (char)222, (char)109, (char)188, (char)14, (char)30, (char)84, (char)28, (char)226}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)60, (char)189, (char)231, (char)26, (char)229, (char)33, (char)7, (char)252, (char)179, (char)19, (char)238, (char)171, (char)142, (char)160, (char)39, (char)7, (char)223, (char)118, (char)11, (char)179}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)130, (char)168, (char)251, (char)206, (char)9, (char)225, (char)244, (char)130, (char)203, (char)46, (char)238, (char)163, (char)183, (char)205, (char)55, (char)39, (char)191, (char)177, (char)95, (char)247}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short)4916);
            assert(pack.time_boot_ms_GET() == 616472282L);
            assert(pack.ymag_GET() == (short)24779);
            assert(pack.yacc_GET() == (short)25596);
            assert(pack.xmag_GET() == (short) -21551);
            assert(pack.zgyro_GET() == (short) -5859);
            assert(pack.xgyro_GET() == (short)11594);
            assert(pack.zmag_GET() == (short) -1029);
            assert(pack.ygyro_GET() == (short)2720);
            assert(pack.zacc_GET() == (short) -2212);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.yacc_SET((short)25596) ;
        p26.ymag_SET((short)24779) ;
        p26.zacc_SET((short) -2212) ;
        p26.time_boot_ms_SET(616472282L) ;
        p26.zmag_SET((short) -1029) ;
        p26.xmag_SET((short) -21551) ;
        p26.xacc_SET((short)4916) ;
        p26.zgyro_SET((short) -5859) ;
        p26.ygyro_SET((short)2720) ;
        p26.xgyro_SET((short)11594) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == (short) -18751);
            assert(pack.yacc_GET() == (short) -12487);
            assert(pack.xgyro_GET() == (short)13038);
            assert(pack.ygyro_GET() == (short) -4273);
            assert(pack.time_usec_GET() == 511595073797739658L);
            assert(pack.ymag_GET() == (short) -10993);
            assert(pack.xmag_GET() == (short) -29112);
            assert(pack.xacc_GET() == (short)19596);
            assert(pack.zacc_GET() == (short) -9794);
            assert(pack.zmag_GET() == (short)6724);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.zgyro_SET((short) -18751) ;
        p27.xmag_SET((short) -29112) ;
        p27.zmag_SET((short)6724) ;
        p27.yacc_SET((short) -12487) ;
        p27.time_usec_SET(511595073797739658L) ;
        p27.ymag_SET((short) -10993) ;
        p27.zacc_SET((short) -9794) ;
        p27.xacc_SET((short)19596) ;
        p27.xgyro_SET((short)13038) ;
        p27.ygyro_SET((short) -4273) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4345198416429298289L);
            assert(pack.press_diff2_GET() == (short) -1359);
            assert(pack.press_diff1_GET() == (short) -3639);
            assert(pack.temperature_GET() == (short) -23805);
            assert(pack.press_abs_GET() == (short) -23645);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_diff1_SET((short) -3639) ;
        p28.time_usec_SET(4345198416429298289L) ;
        p28.temperature_SET((short) -23805) ;
        p28.press_abs_SET((short) -23645) ;
        p28.press_diff2_SET((short) -1359) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)29662);
            assert(pack.press_diff_GET() == -3.7629184E36F);
            assert(pack.time_boot_ms_GET() == 2025527018L);
            assert(pack.press_abs_GET() == 1.4150556E38F);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.temperature_SET((short)29662) ;
        p29.press_abs_SET(1.4150556E38F) ;
        p29.press_diff_SET(-3.7629184E36F) ;
        p29.time_boot_ms_SET(2025527018L) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2588310232L);
            assert(pack.pitchspeed_GET() == -1.4461877E38F);
            assert(pack.pitch_GET() == 1.5877172E38F);
            assert(pack.rollspeed_GET() == -2.5028118E37F);
            assert(pack.yawspeed_GET() == -2.9613271E38F);
            assert(pack.roll_GET() == 2.109723E38F);
            assert(pack.yaw_GET() == 3.9127998E37F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.roll_SET(2.109723E38F) ;
        p30.pitch_SET(1.5877172E38F) ;
        p30.yawspeed_SET(-2.9613271E38F) ;
        p30.rollspeed_SET(-2.5028118E37F) ;
        p30.pitchspeed_SET(-1.4461877E38F) ;
        p30.yaw_SET(3.9127998E37F) ;
        p30.time_boot_ms_SET(2588310232L) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q4_GET() == -3.9547274E37F);
            assert(pack.time_boot_ms_GET() == 4034443889L);
            assert(pack.pitchspeed_GET() == -1.971826E38F);
            assert(pack.q3_GET() == -8.619937E37F);
            assert(pack.rollspeed_GET() == 1.1242637E38F);
            assert(pack.q1_GET() == -9.432099E37F);
            assert(pack.q2_GET() == 2.9452186E38F);
            assert(pack.yawspeed_GET() == 3.0843984E38F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.rollspeed_SET(1.1242637E38F) ;
        p31.yawspeed_SET(3.0843984E38F) ;
        p31.q3_SET(-8.619937E37F) ;
        p31.pitchspeed_SET(-1.971826E38F) ;
        p31.q4_SET(-3.9547274E37F) ;
        p31.q1_SET(-9.432099E37F) ;
        p31.q2_SET(2.9452186E38F) ;
        p31.time_boot_ms_SET(4034443889L) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == 2.9307212E38F);
            assert(pack.x_GET() == 1.6309853E38F);
            assert(pack.z_GET() == -2.5915443E38F);
            assert(pack.vy_GET() == 3.1338083E38F);
            assert(pack.y_GET() == 2.264517E38F);
            assert(pack.vz_GET() == 6.7573053E37F);
            assert(pack.time_boot_ms_GET() == 3265587278L);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.time_boot_ms_SET(3265587278L) ;
        p32.x_SET(1.6309853E38F) ;
        p32.y_SET(2.264517E38F) ;
        p32.vy_SET(3.1338083E38F) ;
        p32.vz_SET(6.7573053E37F) ;
        p32.vx_SET(2.9307212E38F) ;
        p32.z_SET(-2.5915443E38F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == (short)7780);
            assert(pack.alt_GET() == 1421468730);
            assert(pack.lat_GET() == -1725378732);
            assert(pack.vz_GET() == (short)3734);
            assert(pack.hdg_GET() == (char)18519);
            assert(pack.relative_alt_GET() == -402350839);
            assert(pack.lon_GET() == 449796716);
            assert(pack.time_boot_ms_GET() == 3234282348L);
            assert(pack.vy_GET() == (short) -14598);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.time_boot_ms_SET(3234282348L) ;
        p33.vx_SET((short)7780) ;
        p33.alt_SET(1421468730) ;
        p33.lon_SET(449796716) ;
        p33.relative_alt_SET(-402350839) ;
        p33.vy_SET((short) -14598) ;
        p33.lat_SET(-1725378732) ;
        p33.vz_SET((short)3734) ;
        p33.hdg_SET((char)18519) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan5_scaled_GET() == (short) -10938);
            assert(pack.chan7_scaled_GET() == (short)7898);
            assert(pack.port_GET() == (char)227);
            assert(pack.chan4_scaled_GET() == (short) -18071);
            assert(pack.rssi_GET() == (char)14);
            assert(pack.chan3_scaled_GET() == (short)7836);
            assert(pack.chan6_scaled_GET() == (short)30386);
            assert(pack.chan1_scaled_GET() == (short) -30703);
            assert(pack.time_boot_ms_GET() == 3723493498L);
            assert(pack.chan2_scaled_GET() == (short)15550);
            assert(pack.chan8_scaled_GET() == (short)31107);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan8_scaled_SET((short)31107) ;
        p34.chan1_scaled_SET((short) -30703) ;
        p34.chan3_scaled_SET((short)7836) ;
        p34.chan5_scaled_SET((short) -10938) ;
        p34.chan6_scaled_SET((short)30386) ;
        p34.port_SET((char)227) ;
        p34.chan4_scaled_SET((short) -18071) ;
        p34.chan7_scaled_SET((short)7898) ;
        p34.rssi_SET((char)14) ;
        p34.time_boot_ms_SET(3723493498L) ;
        p34.chan2_scaled_SET((short)15550) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.rssi_GET() == (char)21);
            assert(pack.chan6_raw_GET() == (char)43699);
            assert(pack.chan5_raw_GET() == (char)7540);
            assert(pack.chan1_raw_GET() == (char)2307);
            assert(pack.time_boot_ms_GET() == 3145787921L);
            assert(pack.chan4_raw_GET() == (char)41486);
            assert(pack.chan3_raw_GET() == (char)57875);
            assert(pack.chan7_raw_GET() == (char)55500);
            assert(pack.chan2_raw_GET() == (char)6037);
            assert(pack.port_GET() == (char)222);
            assert(pack.chan8_raw_GET() == (char)47922);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan2_raw_SET((char)6037) ;
        p35.chan4_raw_SET((char)41486) ;
        p35.chan8_raw_SET((char)47922) ;
        p35.chan1_raw_SET((char)2307) ;
        p35.chan6_raw_SET((char)43699) ;
        p35.chan7_raw_SET((char)55500) ;
        p35.time_boot_ms_SET(3145787921L) ;
        p35.port_SET((char)222) ;
        p35.chan5_raw_SET((char)7540) ;
        p35.chan3_raw_SET((char)57875) ;
        p35.rssi_SET((char)21) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo15_raw_TRY(ph) == (char)65223);
            assert(pack.servo5_raw_GET() == (char)36874);
            assert(pack.servo3_raw_GET() == (char)6436);
            assert(pack.servo11_raw_TRY(ph) == (char)37790);
            assert(pack.servo9_raw_TRY(ph) == (char)22190);
            assert(pack.servo4_raw_GET() == (char)12414);
            assert(pack.servo8_raw_GET() == (char)58355);
            assert(pack.time_usec_GET() == 1528052938L);
            assert(pack.servo16_raw_TRY(ph) == (char)3336);
            assert(pack.servo12_raw_TRY(ph) == (char)59812);
            assert(pack.servo10_raw_TRY(ph) == (char)5737);
            assert(pack.servo1_raw_GET() == (char)34594);
            assert(pack.servo14_raw_TRY(ph) == (char)5185);
            assert(pack.servo7_raw_GET() == (char)38516);
            assert(pack.servo6_raw_GET() == (char)63640);
            assert(pack.servo2_raw_GET() == (char)46918);
            assert(pack.port_GET() == (char)51);
            assert(pack.servo13_raw_TRY(ph) == (char)37526);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.port_SET((char)51) ;
        p36.servo11_raw_SET((char)37790, PH) ;
        p36.servo14_raw_SET((char)5185, PH) ;
        p36.servo5_raw_SET((char)36874) ;
        p36.servo10_raw_SET((char)5737, PH) ;
        p36.servo15_raw_SET((char)65223, PH) ;
        p36.servo8_raw_SET((char)58355) ;
        p36.servo4_raw_SET((char)12414) ;
        p36.servo2_raw_SET((char)46918) ;
        p36.servo7_raw_SET((char)38516) ;
        p36.servo12_raw_SET((char)59812, PH) ;
        p36.servo13_raw_SET((char)37526, PH) ;
        p36.time_usec_SET(1528052938L) ;
        p36.servo1_raw_SET((char)34594) ;
        p36.servo16_raw_SET((char)3336, PH) ;
        p36.servo6_raw_SET((char)63640) ;
        p36.servo3_raw_SET((char)6436) ;
        p36.servo9_raw_SET((char)22190, PH) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)3);
            assert(pack.end_index_GET() == (short)27722);
            assert(pack.target_system_GET() == (char)173);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.start_index_GET() == (short)5817);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p37.end_index_SET((short)27722) ;
        p37.target_system_SET((char)173) ;
        p37.start_index_SET((short)5817) ;
        p37.target_component_SET((char)3) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)148);
            assert(pack.start_index_GET() == (short) -14086);
            assert(pack.end_index_GET() == (short) -13215);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)171);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.end_index_SET((short) -13215) ;
        p38.start_index_SET((short) -14086) ;
        p38.target_component_SET((char)148) ;
        p38.target_system_SET((char)171) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)72);
            assert(pack.param1_GET() == 4.4889862E36F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_CONDITION_DISTANCE);
            assert(pack.x_GET() == 6.557194E37F);
            assert(pack.z_GET() == 6.8586337E37F);
            assert(pack.current_GET() == (char)202);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.y_GET() == 2.2939797E38F);
            assert(pack.target_system_GET() == (char)34);
            assert(pack.seq_GET() == (char)44679);
            assert(pack.autocontinue_GET() == (char)89);
            assert(pack.param4_GET() == 2.4553303E38F);
            assert(pack.param3_GET() == 2.7495171E38F);
            assert(pack.param2_GET() == 1.4174154E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.command_SET(MAV_CMD.MAV_CMD_CONDITION_DISTANCE) ;
        p39.target_system_SET((char)34) ;
        p39.autocontinue_SET((char)89) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p39.param1_SET(4.4889862E36F) ;
        p39.z_SET(6.8586337E37F) ;
        p39.seq_SET((char)44679) ;
        p39.param2_SET(1.4174154E38F) ;
        p39.current_SET((char)202) ;
        p39.y_SET(2.2939797E38F) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p39.x_SET(6.557194E37F) ;
        p39.param3_SET(2.7495171E38F) ;
        p39.param4_SET(2.4553303E38F) ;
        p39.target_component_SET((char)72) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)56);
            assert(pack.seq_GET() == (char)40159);
            assert(pack.target_component_GET() == (char)87);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_system_SET((char)56) ;
        p40.seq_SET((char)40159) ;
        p40.target_component_SET((char)87) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)37046);
            assert(pack.target_system_GET() == (char)201);
            assert(pack.target_component_GET() == (char)38);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.seq_SET((char)37046) ;
        p41.target_system_SET((char)201) ;
        p41.target_component_SET((char)38) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)54570);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)54570) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)108);
            assert(pack.target_component_GET() == (char)1);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_component_SET((char)1) ;
        p43.target_system_SET((char)108) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)135);
            assert(pack.count_GET() == (char)38735);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)49);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.target_component_SET((char)135) ;
        p44.target_system_SET((char)49) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p44.count_SET((char)38735) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)140);
            assert(pack.target_system_GET() == (char)80);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)80) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p45.target_component_SET((char)140) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)32846);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)32846) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM5_X);
            assert(pack.target_component_GET() == (char)127);
            assert(pack.target_system_GET() == (char)83);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.target_system_SET((char)83) ;
        p47.target_component_SET((char)127) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM5_X) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)49);
            assert(pack.time_usec_TRY(ph) == 4213623506531055216L);
            assert(pack.altitude_GET() == -61956415);
            assert(pack.latitude_GET() == 1014114606);
            assert(pack.longitude_GET() == -1897844153);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.time_usec_SET(4213623506531055216L, PH) ;
        p48.altitude_SET(-61956415) ;
        p48.latitude_SET(1014114606) ;
        p48.target_system_SET((char)49) ;
        p48.longitude_SET(-1897844153) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == 1361199336);
            assert(pack.altitude_GET() == 1014813930);
            assert(pack.time_usec_TRY(ph) == 3212017143800000395L);
            assert(pack.latitude_GET() == 335057160);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.altitude_SET(1014813930) ;
        p49.longitude_SET(1361199336) ;
        p49.latitude_SET(335057160) ;
        p49.time_usec_SET(3212017143800000395L, PH) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)19);
            assert(pack.param_index_GET() == (short) -7469);
            assert(pack.param_value0_GET() == 1.04818067E37F);
            assert(pack.parameter_rc_channel_index_GET() == (char)190);
            assert(pack.target_component_GET() == (char)72);
            assert(pack.scale_GET() == 1.4231319E38F);
            assert(pack.param_id_LEN(ph) == 10);
            assert(pack.param_id_TRY(ph).equals("xhrkbvbzRg"));
            assert(pack.param_value_max_GET() == 9.504948E37F);
            assert(pack.param_value_min_GET() == 1.818923E38F);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.scale_SET(1.4231319E38F) ;
        p50.param_id_SET("xhrkbvbzRg", PH) ;
        p50.target_system_SET((char)19) ;
        p50.param_value_min_SET(1.818923E38F) ;
        p50.param_index_SET((short) -7469) ;
        p50.param_value_max_SET(9.504948E37F) ;
        p50.parameter_rc_channel_index_SET((char)190) ;
        p50.param_value0_SET(1.04818067E37F) ;
        p50.target_component_SET((char)72) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)123);
            assert(pack.seq_GET() == (char)31004);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)242);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p51.seq_SET((char)31004) ;
        p51.target_system_SET((char)242) ;
        p51.target_component_SET((char)123) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)44);
            assert(pack.p2x_GET() == -2.971606E38F);
            assert(pack.p1y_GET() == 1.4323021E38F);
            assert(pack.p2z_GET() == 5.7969824E37F);
            assert(pack.target_system_GET() == (char)13);
            assert(pack.p2y_GET() == 1.1177955E38F);
            assert(pack.p1z_GET() == -2.8592265E38F);
            assert(pack.p1x_GET() == -3.181949E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p1x_SET(-3.181949E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p54.p2x_SET(-2.971606E38F) ;
        p54.p2y_SET(1.1177955E38F) ;
        p54.p2z_SET(5.7969824E37F) ;
        p54.target_system_SET((char)13) ;
        p54.target_component_SET((char)44) ;
        p54.p1z_SET(-2.8592265E38F) ;
        p54.p1y_SET(1.4323021E38F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.p2y_GET() == -2.1014667E38F);
            assert(pack.p2x_GET() == -1.0487377E38F);
            assert(pack.p2z_GET() == 4.613416E37F);
            assert(pack.p1z_GET() == -4.4012073E37F);
            assert(pack.p1x_GET() == -3.2671471E38F);
            assert(pack.p1y_GET() == -2.043574E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p1x_SET(-3.2671471E38F) ;
        p55.p2z_SET(4.613416E37F) ;
        p55.p2x_SET(-1.0487377E38F) ;
        p55.p1y_SET(-2.043574E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p55.p2y_SET(-2.1014667E38F) ;
        p55.p1z_SET(-4.4012073E37F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3407234073593077686L);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {3.3671447E38F, -2.8357257E38F, -5.7159106E37F, 2.979593E38F, 5.9413155E36F, 2.5681895E38F, -1.718542E38F, 6.743784E37F, -1.9965335E38F}));
            assert(pack.rollspeed_GET() == 4.1596005E37F);
            assert(pack.pitchspeed_GET() == -2.705588E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-8.64942E37F, 1.582806E38F, 1.6054189E38F, -1.8392426E37F}));
            assert(pack.yawspeed_GET() == 3.3161198E38F);
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.time_usec_SET(3407234073593077686L) ;
        p61.covariance_SET(new float[] {3.3671447E38F, -2.8357257E38F, -5.7159106E37F, 2.979593E38F, 5.9413155E36F, 2.5681895E38F, -1.718542E38F, 6.743784E37F, -1.9965335E38F}, 0) ;
        p61.yawspeed_SET(3.3161198E38F) ;
        p61.rollspeed_SET(4.1596005E37F) ;
        p61.q_SET(new float[] {-8.64942E37F, 1.582806E38F, 1.6054189E38F, -1.8392426E37F}, 0) ;
        p61.pitchspeed_SET(-2.705588E38F) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.alt_error_GET() == 1.4954583E38F);
            assert(pack.target_bearing_GET() == (short)8626);
            assert(pack.aspd_error_GET() == -1.4104799E38F);
            assert(pack.xtrack_error_GET() == -3.3624162E38F);
            assert(pack.wp_dist_GET() == (char)7038);
            assert(pack.nav_pitch_GET() == -6.195282E37F);
            assert(pack.nav_bearing_GET() == (short)19953);
            assert(pack.nav_roll_GET() == -2.356857E38F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.aspd_error_SET(-1.4104799E38F) ;
        p62.wp_dist_SET((char)7038) ;
        p62.nav_bearing_SET((short)19953) ;
        p62.nav_roll_SET(-2.356857E38F) ;
        p62.nav_pitch_SET(-6.195282E37F) ;
        p62.xtrack_error_SET(-3.3624162E38F) ;
        p62.alt_error_SET(1.4954583E38F) ;
        p62.target_bearing_SET((short)8626) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1468004612);
            assert(pack.lon_GET() == 1616631634);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {9.539491E37F, -2.6528313E38F, -1.1787028E38F, 1.9419034E38F, 1.0578739E38F, -1.985464E38F, 8.905795E37F, 1.1112934E38F, -9.916188E37F, -9.83623E37F, -2.145754E38F, -1.0204534E38F, 9.711183E37F, 2.0272416E38F, -3.2096106E38F, 1.5860183E37F, 2.2859432E38F, 7.458619E37F, 6.3178216E36F, -4.743958E37F, -3.376276E38F, -7.336708E37F, -2.2340265E37F, 8.954514E37F, -1.7975758E37F, -2.107921E38F, 1.0436591E38F, -1.0587358E38F, -3.3442168E38F, -2.7913897E38F, 9.52182E37F, 1.6411102E38F, 1.4764368E38F, -3.214876E38F, 2.9588283E38F, 2.2762202E38F}));
            assert(pack.vy_GET() == 1.6711668E38F);
            assert(pack.vx_GET() == 2.2273098E38F);
            assert(pack.vz_GET() == 2.245792E37F);
            assert(pack.time_usec_GET() == 4395966885102063306L);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
            assert(pack.alt_GET() == -445061826);
            assert(pack.relative_alt_GET() == -115856707);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.vx_SET(2.2273098E38F) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION) ;
        p63.covariance_SET(new float[] {9.539491E37F, -2.6528313E38F, -1.1787028E38F, 1.9419034E38F, 1.0578739E38F, -1.985464E38F, 8.905795E37F, 1.1112934E38F, -9.916188E37F, -9.83623E37F, -2.145754E38F, -1.0204534E38F, 9.711183E37F, 2.0272416E38F, -3.2096106E38F, 1.5860183E37F, 2.2859432E38F, 7.458619E37F, 6.3178216E36F, -4.743958E37F, -3.376276E38F, -7.336708E37F, -2.2340265E37F, 8.954514E37F, -1.7975758E37F, -2.107921E38F, 1.0436591E38F, -1.0587358E38F, -3.3442168E38F, -2.7913897E38F, 9.52182E37F, 1.6411102E38F, 1.4764368E38F, -3.214876E38F, 2.9588283E38F, 2.2762202E38F}, 0) ;
        p63.relative_alt_SET(-115856707) ;
        p63.time_usec_SET(4395966885102063306L) ;
        p63.vz_SET(2.245792E37F) ;
        p63.lat_SET(1468004612) ;
        p63.lon_SET(1616631634) ;
        p63.alt_SET(-445061826) ;
        p63.vy_SET(1.6711668E38F) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {1.6419161E38F, -2.8608314E38F, 1.0910383E38F, 3.2205655E38F, -3.361816E38F, 7.5810693E37F, 8.022726E37F, 3.2012043E38F, 1.6302597E38F, 3.1678264E38F, 2.1317837E38F, -1.9046199E38F, -2.939456E38F, 2.5538785E38F, -7.257274E37F, -1.7337625E38F, 1.6027232E38F, 7.779078E37F, -2.4488975E38F, 3.3836132E38F, 2.0629983E38F, -3.3428999E38F, -7.3355837E37F, 2.7470384E38F, 3.1762262E38F, 3.1962873E38F, -3.9484641E37F, -2.7366467E38F, -2.3774221E38F, 5.3510823E37F, 3.3349666E38F, -4.054088E37F, 2.3666739E38F, 1.6967898E38F, -1.3091608E38F, -1.8040921E38F, 2.8489616E38F, 2.4852886E38F, -3.061839E38F, 3.0409496E38F, 4.92449E37F, 2.9154481E38F, -2.0447281E38F, 6.9628757E37F, 1.240845E38F}));
            assert(pack.ay_GET() == 1.2598692E38F);
            assert(pack.vz_GET() == -1.5468611E38F);
            assert(pack.vy_GET() == 1.2221226E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
            assert(pack.x_GET() == 2.5491577E38F);
            assert(pack.az_GET() == 2.0954828E38F);
            assert(pack.time_usec_GET() == 1349342922170583441L);
            assert(pack.ax_GET() == 1.4285379E38F);
            assert(pack.y_GET() == 1.6318578E38F);
            assert(pack.z_GET() == 1.1728249E38F);
            assert(pack.vx_GET() == 2.7946812E38F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.covariance_SET(new float[] {1.6419161E38F, -2.8608314E38F, 1.0910383E38F, 3.2205655E38F, -3.361816E38F, 7.5810693E37F, 8.022726E37F, 3.2012043E38F, 1.6302597E38F, 3.1678264E38F, 2.1317837E38F, -1.9046199E38F, -2.939456E38F, 2.5538785E38F, -7.257274E37F, -1.7337625E38F, 1.6027232E38F, 7.779078E37F, -2.4488975E38F, 3.3836132E38F, 2.0629983E38F, -3.3428999E38F, -7.3355837E37F, 2.7470384E38F, 3.1762262E38F, 3.1962873E38F, -3.9484641E37F, -2.7366467E38F, -2.3774221E38F, 5.3510823E37F, 3.3349666E38F, -4.054088E37F, 2.3666739E38F, 1.6967898E38F, -1.3091608E38F, -1.8040921E38F, 2.8489616E38F, 2.4852886E38F, -3.061839E38F, 3.0409496E38F, 4.92449E37F, 2.9154481E38F, -2.0447281E38F, 6.9628757E37F, 1.240845E38F}, 0) ;
        p64.y_SET(1.6318578E38F) ;
        p64.ax_SET(1.4285379E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS) ;
        p64.time_usec_SET(1349342922170583441L) ;
        p64.ay_SET(1.2598692E38F) ;
        p64.z_SET(1.1728249E38F) ;
        p64.vx_SET(2.7946812E38F) ;
        p64.vy_SET(1.2221226E38F) ;
        p64.az_SET(2.0954828E38F) ;
        p64.vz_SET(-1.5468611E38F) ;
        p64.x_SET(2.5491577E38F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan14_raw_GET() == (char)29938);
            assert(pack.chan1_raw_GET() == (char)52437);
            assert(pack.chan10_raw_GET() == (char)55425);
            assert(pack.chan4_raw_GET() == (char)51579);
            assert(pack.chan15_raw_GET() == (char)53027);
            assert(pack.chan6_raw_GET() == (char)33080);
            assert(pack.chan12_raw_GET() == (char)6868);
            assert(pack.chan16_raw_GET() == (char)27024);
            assert(pack.rssi_GET() == (char)219);
            assert(pack.chan7_raw_GET() == (char)34564);
            assert(pack.time_boot_ms_GET() == 891408836L);
            assert(pack.chan3_raw_GET() == (char)14556);
            assert(pack.chan2_raw_GET() == (char)60966);
            assert(pack.chan17_raw_GET() == (char)24711);
            assert(pack.chancount_GET() == (char)63);
            assert(pack.chan11_raw_GET() == (char)21131);
            assert(pack.chan8_raw_GET() == (char)32421);
            assert(pack.chan9_raw_GET() == (char)59280);
            assert(pack.chan5_raw_GET() == (char)39947);
            assert(pack.chan18_raw_GET() == (char)49635);
            assert(pack.chan13_raw_GET() == (char)906);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan12_raw_SET((char)6868) ;
        p65.rssi_SET((char)219) ;
        p65.chan6_raw_SET((char)33080) ;
        p65.chan13_raw_SET((char)906) ;
        p65.chan16_raw_SET((char)27024) ;
        p65.chan17_raw_SET((char)24711) ;
        p65.chan4_raw_SET((char)51579) ;
        p65.chan2_raw_SET((char)60966) ;
        p65.chancount_SET((char)63) ;
        p65.chan5_raw_SET((char)39947) ;
        p65.chan3_raw_SET((char)14556) ;
        p65.chan15_raw_SET((char)53027) ;
        p65.chan18_raw_SET((char)49635) ;
        p65.chan10_raw_SET((char)55425) ;
        p65.chan11_raw_SET((char)21131) ;
        p65.chan7_raw_SET((char)34564) ;
        p65.chan8_raw_SET((char)32421) ;
        p65.chan9_raw_SET((char)59280) ;
        p65.time_boot_ms_SET(891408836L) ;
        p65.chan14_raw_SET((char)29938) ;
        p65.chan1_raw_SET((char)52437) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.req_stream_id_GET() == (char)42);
            assert(pack.req_message_rate_GET() == (char)3498);
            assert(pack.target_component_GET() == (char)56);
            assert(pack.start_stop_GET() == (char)83);
            assert(pack.target_system_GET() == (char)14);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.req_message_rate_SET((char)3498) ;
        p66.req_stream_id_SET((char)42) ;
        p66.target_system_SET((char)14) ;
        p66.start_stop_SET((char)83) ;
        p66.target_component_SET((char)56) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.on_off_GET() == (char)151);
            assert(pack.message_rate_GET() == (char)18871);
            assert(pack.stream_id_GET() == (char)181);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.stream_id_SET((char)181) ;
        p67.message_rate_SET((char)18871) ;
        p67.on_off_SET((char)151) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == (short)14112);
            assert(pack.target_GET() == (char)223);
            assert(pack.x_GET() == (short) -24578);
            assert(pack.r_GET() == (short)14806);
            assert(pack.z_GET() == (short)5978);
            assert(pack.buttons_GET() == (char)4513);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.x_SET((short) -24578) ;
        p69.r_SET((short)14806) ;
        p69.target_SET((char)223) ;
        p69.z_SET((short)5978) ;
        p69.y_SET((short)14112) ;
        p69.buttons_SET((char)4513) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan8_raw_GET() == (char)22690);
            assert(pack.chan3_raw_GET() == (char)30800);
            assert(pack.chan4_raw_GET() == (char)13920);
            assert(pack.target_component_GET() == (char)63);
            assert(pack.target_system_GET() == (char)225);
            assert(pack.chan2_raw_GET() == (char)34660);
            assert(pack.chan7_raw_GET() == (char)29272);
            assert(pack.chan5_raw_GET() == (char)23595);
            assert(pack.chan1_raw_GET() == (char)27480);
            assert(pack.chan6_raw_GET() == (char)57387);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan4_raw_SET((char)13920) ;
        p70.chan8_raw_SET((char)22690) ;
        p70.chan7_raw_SET((char)29272) ;
        p70.chan3_raw_SET((char)30800) ;
        p70.chan2_raw_SET((char)34660) ;
        p70.chan6_raw_SET((char)57387) ;
        p70.chan5_raw_SET((char)23595) ;
        p70.chan1_raw_SET((char)27480) ;
        p70.target_component_SET((char)63) ;
        p70.target_system_SET((char)225) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.param1_GET() == -3.2976916E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.param4_GET() == 8.0242633E37F);
            assert(pack.param2_GET() == 2.4373104E38F);
            assert(pack.z_GET() == 1.5018475E37F);
            assert(pack.current_GET() == (char)123);
            assert(pack.seq_GET() == (char)20637);
            assert(pack.autocontinue_GET() == (char)211);
            assert(pack.param3_GET() == 1.6887791E38F);
            assert(pack.x_GET() == -611775191);
            assert(pack.target_component_GET() == (char)93);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.y_GET() == 2137055143);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_LOITER_TURNS);
            assert(pack.target_system_GET() == (char)47);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.seq_SET((char)20637) ;
        p73.x_SET(-611775191) ;
        p73.target_component_SET((char)93) ;
        p73.param4_SET(8.0242633E37F) ;
        p73.z_SET(1.5018475E37F) ;
        p73.y_SET(2137055143) ;
        p73.autocontinue_SET((char)211) ;
        p73.param2_SET(2.4373104E38F) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p73.command_SET(MAV_CMD.MAV_CMD_NAV_LOITER_TURNS) ;
        p73.current_SET((char)123) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p73.param3_SET(1.6887791E38F) ;
        p73.param1_SET(-3.2976916E38F) ;
        p73.target_system_SET((char)47) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.airspeed_GET() == -3.1149676E38F);
            assert(pack.throttle_GET() == (char)19798);
            assert(pack.climb_GET() == 1.3316886E38F);
            assert(pack.alt_GET() == 1.6369459E38F);
            assert(pack.groundspeed_GET() == 5.000109E37F);
            assert(pack.heading_GET() == (short)11777);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.airspeed_SET(-3.1149676E38F) ;
        p74.climb_SET(1.3316886E38F) ;
        p74.groundspeed_SET(5.000109E37F) ;
        p74.throttle_SET((char)19798) ;
        p74.heading_SET((short)11777) ;
        p74.alt_SET(1.6369459E38F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 163376011);
            assert(pack.param1_GET() == 1.9703375E38F);
            assert(pack.target_system_GET() == (char)52);
            assert(pack.param4_GET() == 1.8934654E38F);
            assert(pack.param2_GET() == -2.8228084E38F);
            assert(pack.autocontinue_GET() == (char)114);
            assert(pack.x_GET() == 844313526);
            assert(pack.target_component_GET() == (char)155);
            assert(pack.param3_GET() == -1.4113424E38F);
            assert(pack.z_GET() == 1.0593239E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.current_GET() == (char)29);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.param2_SET(-2.8228084E38F) ;
        p75.param3_SET(-1.4113424E38F) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p75.z_SET(1.0593239E38F) ;
        p75.param4_SET(1.8934654E38F) ;
        p75.target_system_SET((char)52) ;
        p75.target_component_SET((char)155) ;
        p75.param1_SET(1.9703375E38F) ;
        p75.y_SET(163376011) ;
        p75.autocontinue_SET((char)114) ;
        p75.command_SET(MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD) ;
        p75.x_SET(844313526) ;
        p75.current_SET((char)29) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param7_GET() == 2.5649598E38F);
            assert(pack.param1_GET() == 1.645175E38F);
            assert(pack.param5_GET() == 8.609616E37F);
            assert(pack.param3_GET() == 8.4860993E37F);
            assert(pack.param6_GET() == -2.7922753E38F);
            assert(pack.target_component_GET() == (char)123);
            assert(pack.target_system_GET() == (char)133);
            assert(pack.param2_GET() == 2.7656729E38F);
            assert(pack.param4_GET() == 2.3156617E37F);
            assert(pack.confirmation_GET() == (char)27);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT);
        });
        COMMAND_LONG p76 = new COMMAND_LONG();
        PH.setPack(p76);
        p76.param2_SET(2.7656729E38F) ;
        p76.param5_SET(8.609616E37F) ;
        p76.param7_SET(2.5649598E38F) ;
        p76.target_system_SET((char)133) ;
        p76.param4_SET(2.3156617E37F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT) ;
        p76.target_component_SET((char)123) ;
        p76.param3_SET(8.4860993E37F) ;
        p76.param6_SET(-2.7922753E38F) ;
        p76.confirmation_SET((char)27) ;
        p76.param1_SET(1.645175E38F) ;
        TestChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.progress_TRY(ph) == (char)16);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_IN_PROGRESS);
            assert(pack.target_component_TRY(ph) == (char)76);
            assert(pack.result_param2_TRY(ph) == 900177155);
            assert(pack.target_system_TRY(ph) == (char)221);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH);
        });
        COMMAND_ACK p77 = new COMMAND_ACK();
        PH.setPack(p77);
        p77.command_SET(MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH) ;
        p77.progress_SET((char)16, PH) ;
        p77.target_component_SET((char)76, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_IN_PROGRESS) ;
        p77.result_param2_SET(900177155, PH) ;
        p77.target_system_SET((char)221, PH) ;
        TestChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 916441753L);
            assert(pack.pitch_GET() == 1.9932587E38F);
            assert(pack.manual_override_switch_GET() == (char)7);
            assert(pack.mode_switch_GET() == (char)221);
            assert(pack.thrust_GET() == -3.3827901E38F);
            assert(pack.roll_GET() == -1.821534E38F);
            assert(pack.yaw_GET() == -2.7809265E38F);
        });
        MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.roll_SET(-1.821534E38F) ;
        p81.pitch_SET(1.9932587E38F) ;
        p81.thrust_SET(-3.3827901E38F) ;
        p81.mode_switch_SET((char)221) ;
        p81.yaw_SET(-2.7809265E38F) ;
        p81.manual_override_switch_SET((char)7) ;
        p81.time_boot_ms_SET(916441753L) ;
        TestChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)159);
            assert(pack.time_boot_ms_GET() == 1985854270L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.0629743E38F, 7.5667494E37F, -2.5141258E38F, 2.9082036E38F}));
            assert(pack.body_yaw_rate_GET() == -7.462951E37F);
            assert(pack.body_roll_rate_GET() == -1.2211125E38F);
            assert(pack.body_pitch_rate_GET() == 6.0793937E37F);
            assert(pack.thrust_GET() == -1.8716013E38F);
            assert(pack.type_mask_GET() == (char)63);
            assert(pack.target_component_GET() == (char)66);
        });
        SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.body_pitch_rate_SET(6.0793937E37F) ;
        p82.thrust_SET(-1.8716013E38F) ;
        p82.type_mask_SET((char)63) ;
        p82.q_SET(new float[] {1.0629743E38F, 7.5667494E37F, -2.5141258E38F, 2.9082036E38F}, 0) ;
        p82.body_yaw_rate_SET(-7.462951E37F) ;
        p82.time_boot_ms_SET(1985854270L) ;
        p82.target_system_SET((char)159) ;
        p82.target_component_SET((char)66) ;
        p82.body_roll_rate_SET(-1.2211125E38F) ;
        TestChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_roll_rate_GET() == 1.3908988E38F);
            assert(pack.type_mask_GET() == (char)56);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.666648E38F, 2.8523015E38F, -4.991987E36F, 4.4662788E36F}));
            assert(pack.time_boot_ms_GET() == 804137338L);
            assert(pack.thrust_GET() == 2.4027447E38F);
            assert(pack.body_pitch_rate_GET() == 2.3483422E38F);
            assert(pack.body_yaw_rate_GET() == -4.1491558E37F);
        });
        ATTITUDE_TARGET p83 = new ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.type_mask_SET((char)56) ;
        p83.body_yaw_rate_SET(-4.1491558E37F) ;
        p83.thrust_SET(2.4027447E38F) ;
        p83.body_pitch_rate_SET(2.3483422E38F) ;
        p83.time_boot_ms_SET(804137338L) ;
        p83.q_SET(new float[] {1.666648E38F, 2.8523015E38F, -4.991987E36F, 4.4662788E36F}, 0) ;
        p83.body_roll_rate_SET(1.3908988E38F) ;
        TestChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == 9.995082E37F);
            assert(pack.y_GET() == 1.7134673E38F);
            assert(pack.yaw_GET() == 3.144332E38F);
            assert(pack.afz_GET() == -6.843681E37F);
            assert(pack.z_GET() == 9.798969E37F);
            assert(pack.vy_GET() == 2.4782723E38F);
            assert(pack.target_system_GET() == (char)132);
            assert(pack.afy_GET() == 2.524685E38F);
            assert(pack.vz_GET() == -7.751337E37F);
            assert(pack.vx_GET() == 1.4750728E38F);
            assert(pack.target_component_GET() == (char)213);
            assert(pack.afx_GET() == 6.168401E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.time_boot_ms_GET() == 1138677927L);
            assert(pack.x_GET() == 4.42224E37F);
            assert(pack.type_mask_GET() == (char)33027);
        });
        SET_POSITION_TARGET_LOCAL_NED p84 = new SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.y_SET(1.7134673E38F) ;
        p84.afz_SET(-6.843681E37F) ;
        p84.target_system_SET((char)132) ;
        p84.vy_SET(2.4782723E38F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p84.target_component_SET((char)213) ;
        p84.yaw_rate_SET(9.995082E37F) ;
        p84.time_boot_ms_SET(1138677927L) ;
        p84.afx_SET(6.168401E37F) ;
        p84.type_mask_SET((char)33027) ;
        p84.vx_SET(1.4750728E38F) ;
        p84.x_SET(4.42224E37F) ;
        p84.vz_SET(-7.751337E37F) ;
        p84.afy_SET(2.524685E38F) ;
        p84.z_SET(9.798969E37F) ;
        p84.yaw_SET(3.144332E38F) ;
        TestChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == 7.109557E37F);
            assert(pack.type_mask_GET() == (char)18397);
            assert(pack.vx_GET() == 1.2513721E37F);
            assert(pack.target_component_GET() == (char)1);
            assert(pack.lat_int_GET() == -2006561883);
            assert(pack.afz_GET() == 1.995714E38F);
            assert(pack.yaw_GET() == 1.4690081E38F);
            assert(pack.target_system_GET() == (char)149);
            assert(pack.time_boot_ms_GET() == 1290488007L);
            assert(pack.alt_GET() == 1.8972843E38F);
            assert(pack.vz_GET() == -2.8991976E38F);
            assert(pack.lon_int_GET() == 1471442272);
            assert(pack.afy_GET() == 2.5777286E38F);
            assert(pack.afx_GET() == 2.7444274E38F);
            assert(pack.vy_GET() == 2.702372E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
        });
        SET_POSITION_TARGET_GLOBAL_INT p86 = new SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.time_boot_ms_SET(1290488007L) ;
        p86.lon_int_SET(1471442272) ;
        p86.alt_SET(1.8972843E38F) ;
        p86.target_component_SET((char)1) ;
        p86.lat_int_SET(-2006561883) ;
        p86.type_mask_SET((char)18397) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p86.target_system_SET((char)149) ;
        p86.afx_SET(2.7444274E38F) ;
        p86.yaw_SET(1.4690081E38F) ;
        p86.yaw_rate_SET(7.109557E37F) ;
        p86.afy_SET(2.5777286E38F) ;
        p86.vy_SET(2.702372E38F) ;
        p86.vx_SET(1.2513721E37F) ;
        p86.vz_SET(-2.8991976E38F) ;
        p86.afz_SET(1.995714E38F) ;
        TestChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.lat_int_GET() == 195525960);
            assert(pack.yaw_GET() == -2.0699617E38F);
            assert(pack.afx_GET() == -2.0859566E38F);
            assert(pack.lon_int_GET() == 355763266);
            assert(pack.type_mask_GET() == (char)62421);
            assert(pack.afy_GET() == -2.6146066E38F);
            assert(pack.vz_GET() == -2.5817037E38F);
            assert(pack.vy_GET() == 3.0454077E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.afz_GET() == -2.2340034E38F);
            assert(pack.yaw_rate_GET() == -2.9928841E38F);
            assert(pack.alt_GET() == -5.925824E37F);
            assert(pack.vx_GET() == -6.9077263E37F);
            assert(pack.time_boot_ms_GET() == 1709218329L);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.lat_int_SET(195525960) ;
        p87.time_boot_ms_SET(1709218329L) ;
        p87.afy_SET(-2.6146066E38F) ;
        p87.yaw_rate_SET(-2.9928841E38F) ;
        p87.vy_SET(3.0454077E38F) ;
        p87.afz_SET(-2.2340034E38F) ;
        p87.vx_SET(-6.9077263E37F) ;
        p87.type_mask_SET((char)62421) ;
        p87.lon_int_SET(355763266) ;
        p87.afx_SET(-2.0859566E38F) ;
        p87.yaw_SET(-2.0699617E38F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p87.alt_SET(-5.925824E37F) ;
        p87.vz_SET(-2.5817037E38F) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == 3.8751613E36F);
            assert(pack.yaw_GET() == -2.9895037E38F);
            assert(pack.time_boot_ms_GET() == 3966499388L);
            assert(pack.z_GET() == 3.1032425E38F);
            assert(pack.x_GET() == -1.1340821E38F);
            assert(pack.pitch_GET() == -1.9229584E38F);
            assert(pack.y_GET() == -1.0572785E38F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.x_SET(-1.1340821E38F) ;
        p89.z_SET(3.1032425E38F) ;
        p89.time_boot_ms_SET(3966499388L) ;
        p89.pitch_SET(-1.9229584E38F) ;
        p89.y_SET(-1.0572785E38F) ;
        p89.yaw_SET(-2.9895037E38F) ;
        p89.roll_SET(3.8751613E36F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == (short) -14426);
            assert(pack.yaw_GET() == -2.6592608E38F);
            assert(pack.pitch_GET() == -3.229407E38F);
            assert(pack.lon_GET() == 30252320);
            assert(pack.lat_GET() == 1730935379);
            assert(pack.time_usec_GET() == 783016335449419299L);
            assert(pack.pitchspeed_GET() == 9.971436E37F);
            assert(pack.roll_GET() == -2.9601165E38F);
            assert(pack.yawspeed_GET() == 3.1417738E38F);
            assert(pack.rollspeed_GET() == -1.1854458E38F);
            assert(pack.xacc_GET() == (short)17839);
            assert(pack.yacc_GET() == (short) -27028);
            assert(pack.zacc_GET() == (short)5142);
            assert(pack.vy_GET() == (short) -26247);
            assert(pack.alt_GET() == -2078933282);
            assert(pack.vz_GET() == (short) -27734);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.xacc_SET((short)17839) ;
        p90.yawspeed_SET(3.1417738E38F) ;
        p90.vy_SET((short) -26247) ;
        p90.yaw_SET(-2.6592608E38F) ;
        p90.pitchspeed_SET(9.971436E37F) ;
        p90.roll_SET(-2.9601165E38F) ;
        p90.alt_SET(-2078933282) ;
        p90.vz_SET((short) -27734) ;
        p90.pitch_SET(-3.229407E38F) ;
        p90.zacc_SET((short)5142) ;
        p90.rollspeed_SET(-1.1854458E38F) ;
        p90.lon_SET(30252320) ;
        p90.time_usec_SET(783016335449419299L) ;
        p90.vx_SET((short) -14426) ;
        p90.lat_SET(1730935379) ;
        p90.yacc_SET((short) -27028) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.roll_ailerons_GET() == 1.3513689E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_ARMED);
            assert(pack.nav_mode_GET() == (char)114);
            assert(pack.aux1_GET() == 8.2034056E36F);
            assert(pack.aux3_GET() == -2.972944E37F);
            assert(pack.time_usec_GET() == 2147707267608688668L);
            assert(pack.aux4_GET() == -2.6653958E38F);
            assert(pack.aux2_GET() == -2.2666357E38F);
            assert(pack.throttle_GET() == 2.2055612E37F);
            assert(pack.yaw_rudder_GET() == 1.2831406E38F);
            assert(pack.pitch_elevator_GET() == 1.570311E38F);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.nav_mode_SET((char)114) ;
        p91.throttle_SET(2.2055612E37F) ;
        p91.time_usec_SET(2147707267608688668L) ;
        p91.yaw_rudder_SET(1.2831406E38F) ;
        p91.aux3_SET(-2.972944E37F) ;
        p91.aux4_SET(-2.6653958E38F) ;
        p91.pitch_elevator_SET(1.570311E38F) ;
        p91.aux2_SET(-2.2666357E38F) ;
        p91.roll_ailerons_SET(1.3513689E38F) ;
        p91.aux1_SET(8.2034056E36F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_GUIDED_ARMED) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5339595732478099190L);
            assert(pack.chan7_raw_GET() == (char)44606);
            assert(pack.chan9_raw_GET() == (char)29715);
            assert(pack.chan5_raw_GET() == (char)3581);
            assert(pack.chan10_raw_GET() == (char)12033);
            assert(pack.chan3_raw_GET() == (char)22675);
            assert(pack.chan4_raw_GET() == (char)62020);
            assert(pack.chan1_raw_GET() == (char)37675);
            assert(pack.rssi_GET() == (char)84);
            assert(pack.chan6_raw_GET() == (char)35611);
            assert(pack.chan2_raw_GET() == (char)25486);
            assert(pack.chan8_raw_GET() == (char)31126);
            assert(pack.chan12_raw_GET() == (char)31889);
            assert(pack.chan11_raw_GET() == (char)47670);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan7_raw_SET((char)44606) ;
        p92.chan3_raw_SET((char)22675) ;
        p92.chan8_raw_SET((char)31126) ;
        p92.rssi_SET((char)84) ;
        p92.chan1_raw_SET((char)37675) ;
        p92.chan10_raw_SET((char)12033) ;
        p92.chan2_raw_SET((char)25486) ;
        p92.chan11_raw_SET((char)47670) ;
        p92.chan4_raw_SET((char)62020) ;
        p92.chan6_raw_SET((char)35611) ;
        p92.chan9_raw_SET((char)29715) ;
        p92.time_usec_SET(5339595732478099190L) ;
        p92.chan5_raw_SET((char)3581) ;
        p92.chan12_raw_SET((char)31889) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.556087E38F, 1.8591958E38F, 1.6349547E38F, -7.402033E37F, -4.0094528E37F, 3.2237107E38F, 1.1031571E38F, 2.723084E37F, 3.2130069E38F, -1.726898E38F, 2.4931686E38F, 3.3417438E38F, 2.722025E38F, -1.9852905E38F, 7.5473103E37F, -9.476758E37F}));
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_MANUAL_DISARMED);
            assert(pack.time_usec_GET() == 3751522630966061397L);
            assert(pack.flags_GET() == 7048276546884907571L);
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.controls_SET(new float[] {2.556087E38F, 1.8591958E38F, 1.6349547E38F, -7.402033E37F, -4.0094528E37F, 3.2237107E38F, 1.1031571E38F, 2.723084E37F, 3.2130069E38F, -1.726898E38F, 2.4931686E38F, 3.3417438E38F, 2.722025E38F, -1.9852905E38F, 7.5473103E37F, -9.476758E37F}, 0) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_MANUAL_DISARMED) ;
        p93.time_usec_SET(3751522630966061397L) ;
        p93.flags_SET(7048276546884907571L) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_rate_x_TRY(ph) == 9.811711E37F);
            assert(pack.flow_y_GET() == (short) -2637);
            assert(pack.flow_comp_m_y_GET() == -8.584404E37F);
            assert(pack.time_usec_GET() == 1369134330104978004L);
            assert(pack.ground_distance_GET() == -3.1453176E38F);
            assert(pack.quality_GET() == (char)211);
            assert(pack.sensor_id_GET() == (char)204);
            assert(pack.flow_comp_m_x_GET() == -1.3292387E38F);
            assert(pack.flow_rate_y_TRY(ph) == 2.06767E38F);
            assert(pack.flow_x_GET() == (short) -11448);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.flow_comp_m_y_SET(-8.584404E37F) ;
        p100.flow_comp_m_x_SET(-1.3292387E38F) ;
        p100.flow_y_SET((short) -2637) ;
        p100.flow_x_SET((short) -11448) ;
        p100.flow_rate_y_SET(2.06767E38F, PH) ;
        p100.time_usec_SET(1369134330104978004L) ;
        p100.flow_rate_x_SET(9.811711E37F, PH) ;
        p100.ground_distance_SET(-3.1453176E38F) ;
        p100.sensor_id_SET((char)204) ;
        p100.quality_SET((char)211) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 2656787438487580145L);
            assert(pack.z_GET() == -2.2484298E38F);
            assert(pack.x_GET() == -1.211131E38F);
            assert(pack.yaw_GET() == 1.6141391E38F);
            assert(pack.y_GET() == -3.7103E37F);
            assert(pack.roll_GET() == 3.645368E37F);
            assert(pack.pitch_GET() == -1.4346329E38F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.usec_SET(2656787438487580145L) ;
        p101.y_SET(-3.7103E37F) ;
        p101.x_SET(-1.211131E38F) ;
        p101.z_SET(-2.2484298E38F) ;
        p101.roll_SET(3.645368E37F) ;
        p101.yaw_SET(1.6141391E38F) ;
        p101.pitch_SET(-1.4346329E38F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == 2.9334443E38F);
            assert(pack.x_GET() == 2.472586E38F);
            assert(pack.usec_GET() == 7170455524080234822L);
            assert(pack.yaw_GET() == 5.11259E37F);
            assert(pack.pitch_GET() == 1.4302197E38F);
            assert(pack.y_GET() == -2.3289867E38F);
            assert(pack.z_GET() == 7.5062896E37F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.yaw_SET(5.11259E37F) ;
        p102.pitch_SET(1.4302197E38F) ;
        p102.y_SET(-2.3289867E38F) ;
        p102.x_SET(2.472586E38F) ;
        p102.z_SET(7.5062896E37F) ;
        p102.roll_SET(2.9334443E38F) ;
        p102.usec_SET(7170455524080234822L) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 1.5441144E38F);
            assert(pack.x_GET() == 1.0177404E38F);
            assert(pack.usec_GET() == 2743838947622726432L);
            assert(pack.z_GET() == 3.2284116E38F);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.z_SET(3.2284116E38F) ;
        p103.y_SET(1.5441144E38F) ;
        p103.x_SET(1.0177404E38F) ;
        p103.usec_SET(2743838947622726432L) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 3.0907616E38F);
            assert(pack.x_GET() == -2.2830789E38F);
            assert(pack.pitch_GET() == 1.3328292E38F);
            assert(pack.yaw_GET() == -2.7422647E38F);
            assert(pack.z_GET() == -3.1128162E38F);
            assert(pack.roll_GET() == 1.9468694E38F);
            assert(pack.usec_GET() == 8202020415218772088L);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.y_SET(3.0907616E38F) ;
        p104.z_SET(-3.1128162E38F) ;
        p104.yaw_SET(-2.7422647E38F) ;
        p104.x_SET(-2.2830789E38F) ;
        p104.usec_SET(8202020415218772088L) ;
        p104.roll_SET(1.9468694E38F) ;
        p104.pitch_SET(1.3328292E38F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.fields_updated_GET() == (char)5986);
            assert(pack.time_usec_GET() == 758757279703944735L);
            assert(pack.zacc_GET() == -6.8699417E37F);
            assert(pack.xmag_GET() == 3.1088561E38F);
            assert(pack.ymag_GET() == 2.0448046E38F);
            assert(pack.pressure_alt_GET() == -2.620222E38F);
            assert(pack.zgyro_GET() == -6.4680766E37F);
            assert(pack.diff_pressure_GET() == -1.1368744E38F);
            assert(pack.abs_pressure_GET() == 1.1269517E38F);
            assert(pack.xgyro_GET() == 3.1899365E37F);
            assert(pack.temperature_GET() == 1.3831615E38F);
            assert(pack.zmag_GET() == 3.1919665E37F);
            assert(pack.xacc_GET() == -1.2434675E37F);
            assert(pack.ygyro_GET() == 1.7324445E38F);
            assert(pack.yacc_GET() == -1.6714342E38F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.ygyro_SET(1.7324445E38F) ;
        p105.xacc_SET(-1.2434675E37F) ;
        p105.time_usec_SET(758757279703944735L) ;
        p105.zmag_SET(3.1919665E37F) ;
        p105.fields_updated_SET((char)5986) ;
        p105.zgyro_SET(-6.4680766E37F) ;
        p105.temperature_SET(1.3831615E38F) ;
        p105.xmag_SET(3.1088561E38F) ;
        p105.diff_pressure_SET(-1.1368744E38F) ;
        p105.abs_pressure_SET(1.1269517E38F) ;
        p105.xgyro_SET(3.1899365E37F) ;
        p105.pressure_alt_SET(-2.620222E38F) ;
        p105.yacc_SET(-1.6714342E38F) ;
        p105.ymag_SET(2.0448046E38F) ;
        p105.zacc_SET(-6.8699417E37F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.time_delta_distance_us_GET() == 2055938616L);
            assert(pack.distance_GET() == -2.6285127E38F);
            assert(pack.integrated_xgyro_GET() == -7.871131E37F);
            assert(pack.temperature_GET() == (short) -25266);
            assert(pack.integrated_zgyro_GET() == -1.898757E38F);
            assert(pack.integrated_y_GET() == -2.0090029E38F);
            assert(pack.quality_GET() == (char)74);
            assert(pack.sensor_id_GET() == (char)95);
            assert(pack.integrated_ygyro_GET() == -1.6299282E38F);
            assert(pack.time_usec_GET() == 7988848250634164985L);
            assert(pack.integrated_x_GET() == 3.3422038E38F);
            assert(pack.integration_time_us_GET() == 2217879009L);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.distance_SET(-2.6285127E38F) ;
        p106.integrated_zgyro_SET(-1.898757E38F) ;
        p106.integrated_xgyro_SET(-7.871131E37F) ;
        p106.integrated_y_SET(-2.0090029E38F) ;
        p106.time_delta_distance_us_SET(2055938616L) ;
        p106.sensor_id_SET((char)95) ;
        p106.quality_SET((char)74) ;
        p106.integrated_x_SET(3.3422038E38F) ;
        p106.integration_time_us_SET(2217879009L) ;
        p106.temperature_SET((short) -25266) ;
        p106.integrated_ygyro_SET(-1.6299282E38F) ;
        p106.time_usec_SET(7988848250634164985L) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == -1.1488023E38F);
            assert(pack.ymag_GET() == 2.2473098E38F);
            assert(pack.diff_pressure_GET() == 1.4529659E38F);
            assert(pack.zacc_GET() == 1.190036E38F);
            assert(pack.ygyro_GET() == 3.2634194E38F);
            assert(pack.fields_updated_GET() == 2672677032L);
            assert(pack.yacc_GET() == -1.9680912E38F);
            assert(pack.temperature_GET() == -1.008232E38F);
            assert(pack.xgyro_GET() == -9.655945E37F);
            assert(pack.zmag_GET() == 1.8280765E38F);
            assert(pack.abs_pressure_GET() == -2.6549989E38F);
            assert(pack.pressure_alt_GET() == -5.4286554E37F);
            assert(pack.time_usec_GET() == 94085489705796240L);
            assert(pack.zgyro_GET() == -2.4995829E38F);
            assert(pack.xmag_GET() == 1.4317858E38F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.pressure_alt_SET(-5.4286554E37F) ;
        p107.yacc_SET(-1.9680912E38F) ;
        p107.zmag_SET(1.8280765E38F) ;
        p107.zacc_SET(1.190036E38F) ;
        p107.abs_pressure_SET(-2.6549989E38F) ;
        p107.time_usec_SET(94085489705796240L) ;
        p107.diff_pressure_SET(1.4529659E38F) ;
        p107.temperature_SET(-1.008232E38F) ;
        p107.ymag_SET(2.2473098E38F) ;
        p107.xacc_SET(-1.1488023E38F) ;
        p107.zgyro_SET(-2.4995829E38F) ;
        p107.xgyro_SET(-9.655945E37F) ;
        p107.ygyro_SET(3.2634194E38F) ;
        p107.fields_updated_SET(2672677032L) ;
        p107.xmag_SET(1.4317858E38F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.vd_GET() == -1.133156E38F);
            assert(pack.q4_GET() == -1.1645531E38F);
            assert(pack.zacc_GET() == -1.1325966E38F);
            assert(pack.yacc_GET() == -2.3305487E38F);
            assert(pack.pitch_GET() == 2.7106343E37F);
            assert(pack.xgyro_GET() == 1.0504353E38F);
            assert(pack.yaw_GET() == 1.5700931E38F);
            assert(pack.ve_GET() == 1.6346504E38F);
            assert(pack.vn_GET() == -1.8193952E38F);
            assert(pack.roll_GET() == -9.242842E37F);
            assert(pack.q1_GET() == 4.285381E37F);
            assert(pack.q3_GET() == -2.3131354E38F);
            assert(pack.alt_GET() == 3.3516765E38F);
            assert(pack.ygyro_GET() == 1.3579728E38F);
            assert(pack.zgyro_GET() == 1.3431242E38F);
            assert(pack.lat_GET() == 2.3449776E38F);
            assert(pack.xacc_GET() == -5.7286226E37F);
            assert(pack.q2_GET() == -1.3478774E38F);
            assert(pack.std_dev_horz_GET() == 2.6649703E38F);
            assert(pack.std_dev_vert_GET() == 1.399859E37F);
            assert(pack.lon_GET() == -9.625164E36F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.ve_SET(1.6346504E38F) ;
        p108.vn_SET(-1.8193952E38F) ;
        p108.q1_SET(4.285381E37F) ;
        p108.yaw_SET(1.5700931E38F) ;
        p108.pitch_SET(2.7106343E37F) ;
        p108.zgyro_SET(1.3431242E38F) ;
        p108.lon_SET(-9.625164E36F) ;
        p108.xgyro_SET(1.0504353E38F) ;
        p108.q2_SET(-1.3478774E38F) ;
        p108.xacc_SET(-5.7286226E37F) ;
        p108.yacc_SET(-2.3305487E38F) ;
        p108.q4_SET(-1.1645531E38F) ;
        p108.q3_SET(-2.3131354E38F) ;
        p108.std_dev_vert_SET(1.399859E37F) ;
        p108.std_dev_horz_SET(2.6649703E38F) ;
        p108.ygyro_SET(1.3579728E38F) ;
        p108.zacc_SET(-1.1325966E38F) ;
        p108.roll_SET(-9.242842E37F) ;
        p108.vd_SET(-1.133156E38F) ;
        p108.alt_SET(3.3516765E38F) ;
        p108.lat_SET(2.3449776E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.remnoise_GET() == (char)190);
            assert(pack.txbuf_GET() == (char)0);
            assert(pack.fixed__GET() == (char)8939);
            assert(pack.rxerrors_GET() == (char)57598);
            assert(pack.rssi_GET() == (char)19);
            assert(pack.remrssi_GET() == (char)244);
            assert(pack.noise_GET() == (char)42);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.remrssi_SET((char)244) ;
        p109.noise_SET((char)42) ;
        p109.rssi_SET((char)19) ;
        p109.remnoise_SET((char)190) ;
        p109.txbuf_SET((char)0) ;
        p109.fixed__SET((char)8939) ;
        p109.rxerrors_SET((char)57598) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)75);
            assert(pack.target_system_GET() == (char)244);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)43, (char)123, (char)67, (char)143, (char)125, (char)118, (char)12, (char)114, (char)84, (char)41, (char)97, (char)119, (char)21, (char)189, (char)60, (char)36, (char)23, (char)198, (char)33, (char)143, (char)95, (char)101, (char)92, (char)250, (char)199, (char)9, (char)19, (char)138, (char)47, (char)37, (char)99, (char)95, (char)229, (char)145, (char)227, (char)53, (char)212, (char)237, (char)72, (char)97, (char)162, (char)58, (char)216, (char)20, (char)75, (char)176, (char)42, (char)176, (char)52, (char)125, (char)124, (char)231, (char)49, (char)224, (char)163, (char)231, (char)7, (char)213, (char)51, (char)124, (char)191, (char)151, (char)193, (char)98, (char)255, (char)47, (char)17, (char)110, (char)233, (char)161, (char)63, (char)110, (char)188, (char)215, (char)48, (char)191, (char)228, (char)138, (char)15, (char)230, (char)194, (char)24, (char)202, (char)60, (char)52, (char)117, (char)212, (char)61, (char)170, (char)108, (char)43, (char)202, (char)54, (char)167, (char)183, (char)110, (char)69, (char)147, (char)187, (char)54, (char)108, (char)255, (char)80, (char)248, (char)212, (char)152, (char)196, (char)240, (char)242, (char)19, (char)128, (char)198, (char)183, (char)110, (char)221, (char)139, (char)149, (char)218, (char)46, (char)151, (char)48, (char)146, (char)138, (char)53, (char)67, (char)31, (char)95, (char)96, (char)244, (char)89, (char)250, (char)61, (char)184, (char)39, (char)88, (char)103, (char)95, (char)147, (char)48, (char)38, (char)72, (char)96, (char)202, (char)152, (char)11, (char)224, (char)211, (char)170, (char)211, (char)207, (char)50, (char)170, (char)66, (char)176, (char)131, (char)148, (char)4, (char)143, (char)64, (char)61, (char)71, (char)42, (char)41, (char)222, (char)28, (char)223, (char)3, (char)55, (char)107, (char)3, (char)61, (char)121, (char)195, (char)109, (char)121, (char)162, (char)7, (char)94, (char)112, (char)49, (char)112, (char)83, (char)132, (char)235, (char)247, (char)4, (char)76, (char)15, (char)22, (char)77, (char)140, (char)118, (char)14, (char)184, (char)221, (char)102, (char)139, (char)185, (char)211, (char)49, (char)10, (char)21, (char)62, (char)0, (char)196, (char)57, (char)138, (char)192, (char)107, (char)222, (char)47, (char)224, (char)138, (char)24, (char)22, (char)57, (char)95, (char)58, (char)151, (char)248, (char)241, (char)143, (char)247, (char)44, (char)195, (char)244, (char)47, (char)198, (char)173, (char)241, (char)229, (char)235, (char)225, (char)159, (char)76, (char)30, (char)220, (char)228, (char)57, (char)195, (char)144, (char)238, (char)71, (char)142, (char)219, (char)19, (char)242, (char)13, (char)114, (char)196, (char)139}));
            assert(pack.target_component_GET() == (char)77);
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)75) ;
        p110.target_system_SET((char)244) ;
        p110.payload_SET(new char[] {(char)43, (char)123, (char)67, (char)143, (char)125, (char)118, (char)12, (char)114, (char)84, (char)41, (char)97, (char)119, (char)21, (char)189, (char)60, (char)36, (char)23, (char)198, (char)33, (char)143, (char)95, (char)101, (char)92, (char)250, (char)199, (char)9, (char)19, (char)138, (char)47, (char)37, (char)99, (char)95, (char)229, (char)145, (char)227, (char)53, (char)212, (char)237, (char)72, (char)97, (char)162, (char)58, (char)216, (char)20, (char)75, (char)176, (char)42, (char)176, (char)52, (char)125, (char)124, (char)231, (char)49, (char)224, (char)163, (char)231, (char)7, (char)213, (char)51, (char)124, (char)191, (char)151, (char)193, (char)98, (char)255, (char)47, (char)17, (char)110, (char)233, (char)161, (char)63, (char)110, (char)188, (char)215, (char)48, (char)191, (char)228, (char)138, (char)15, (char)230, (char)194, (char)24, (char)202, (char)60, (char)52, (char)117, (char)212, (char)61, (char)170, (char)108, (char)43, (char)202, (char)54, (char)167, (char)183, (char)110, (char)69, (char)147, (char)187, (char)54, (char)108, (char)255, (char)80, (char)248, (char)212, (char)152, (char)196, (char)240, (char)242, (char)19, (char)128, (char)198, (char)183, (char)110, (char)221, (char)139, (char)149, (char)218, (char)46, (char)151, (char)48, (char)146, (char)138, (char)53, (char)67, (char)31, (char)95, (char)96, (char)244, (char)89, (char)250, (char)61, (char)184, (char)39, (char)88, (char)103, (char)95, (char)147, (char)48, (char)38, (char)72, (char)96, (char)202, (char)152, (char)11, (char)224, (char)211, (char)170, (char)211, (char)207, (char)50, (char)170, (char)66, (char)176, (char)131, (char)148, (char)4, (char)143, (char)64, (char)61, (char)71, (char)42, (char)41, (char)222, (char)28, (char)223, (char)3, (char)55, (char)107, (char)3, (char)61, (char)121, (char)195, (char)109, (char)121, (char)162, (char)7, (char)94, (char)112, (char)49, (char)112, (char)83, (char)132, (char)235, (char)247, (char)4, (char)76, (char)15, (char)22, (char)77, (char)140, (char)118, (char)14, (char)184, (char)221, (char)102, (char)139, (char)185, (char)211, (char)49, (char)10, (char)21, (char)62, (char)0, (char)196, (char)57, (char)138, (char)192, (char)107, (char)222, (char)47, (char)224, (char)138, (char)24, (char)22, (char)57, (char)95, (char)58, (char)151, (char)248, (char)241, (char)143, (char)247, (char)44, (char)195, (char)244, (char)47, (char)198, (char)173, (char)241, (char)229, (char)235, (char)225, (char)159, (char)76, (char)30, (char)220, (char)228, (char)57, (char)195, (char)144, (char)238, (char)71, (char)142, (char)219, (char)19, (char)242, (char)13, (char)114, (char)196, (char)139}, 0) ;
        p110.target_component_SET((char)77) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.ts1_GET() == -2445189278974876071L);
            assert(pack.tc1_GET() == 430885946515863410L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(-2445189278974876071L) ;
        p111.tc1_SET(430885946515863410L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 8202430039620797331L);
            assert(pack.seq_GET() == 1588309206L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(8202430039620797331L) ;
        p112.seq_SET(1588309206L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 1555010188);
            assert(pack.fix_type_GET() == (char)128);
            assert(pack.lon_GET() == -128190245);
            assert(pack.vel_GET() == (char)11339);
            assert(pack.satellites_visible_GET() == (char)43);
            assert(pack.lat_GET() == 1108442128);
            assert(pack.eph_GET() == (char)19847);
            assert(pack.vd_GET() == (short) -9749);
            assert(pack.ve_GET() == (short) -21620);
            assert(pack.vn_GET() == (short)28517);
            assert(pack.cog_GET() == (char)616);
            assert(pack.time_usec_GET() == 7129440067850045002L);
            assert(pack.epv_GET() == (char)11003);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.fix_type_SET((char)128) ;
        p113.vel_SET((char)11339) ;
        p113.time_usec_SET(7129440067850045002L) ;
        p113.epv_SET((char)11003) ;
        p113.alt_SET(1555010188) ;
        p113.ve_SET((short) -21620) ;
        p113.lat_SET(1108442128) ;
        p113.lon_SET(-128190245) ;
        p113.eph_SET((char)19847) ;
        p113.vd_SET((short) -9749) ;
        p113.vn_SET((short)28517) ;
        p113.cog_SET((char)616) ;
        p113.satellites_visible_SET((char)43) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integrated_xgyro_GET() == 7.3561004E37F);
            assert(pack.integration_time_us_GET() == 1409206807L);
            assert(pack.sensor_id_GET() == (char)176);
            assert(pack.integrated_zgyro_GET() == -3.8060608E37F);
            assert(pack.quality_GET() == (char)91);
            assert(pack.integrated_x_GET() == -7.562972E37F);
            assert(pack.integrated_y_GET() == 4.604889E37F);
            assert(pack.time_usec_GET() == 5057157198232529788L);
            assert(pack.integrated_ygyro_GET() == -1.1928959E38F);
            assert(pack.distance_GET() == -1.3547291E38F);
            assert(pack.temperature_GET() == (short)1443);
            assert(pack.time_delta_distance_us_GET() == 882124496L);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.sensor_id_SET((char)176) ;
        p114.integrated_y_SET(4.604889E37F) ;
        p114.distance_SET(-1.3547291E38F) ;
        p114.time_usec_SET(5057157198232529788L) ;
        p114.integrated_ygyro_SET(-1.1928959E38F) ;
        p114.time_delta_distance_us_SET(882124496L) ;
        p114.quality_SET((char)91) ;
        p114.integrated_xgyro_SET(7.3561004E37F) ;
        p114.integration_time_us_SET(1409206807L) ;
        p114.integrated_zgyro_SET(-3.8060608E37F) ;
        p114.integrated_x_SET(-7.562972E37F) ;
        p114.temperature_SET((short)1443) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == (short)22019);
            assert(pack.true_airspeed_GET() == (char)28491);
            assert(pack.alt_GET() == -188228758);
            assert(pack.ind_airspeed_GET() == (char)59392);
            assert(pack.rollspeed_GET() == 1.3868384E38F);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {-3.8068508E37F, 1.4900139E38F, 3.085717E38F, -3.2921943E38F}));
            assert(pack.zacc_GET() == (short) -26265);
            assert(pack.vz_GET() == (short)4098);
            assert(pack.yacc_GET() == (short)9473);
            assert(pack.yawspeed_GET() == -1.4256405E38F);
            assert(pack.vx_GET() == (short) -16279);
            assert(pack.lat_GET() == 1837883670);
            assert(pack.lon_GET() == 712707951);
            assert(pack.xacc_GET() == (short) -21983);
            assert(pack.pitchspeed_GET() == -2.8005454E38F);
            assert(pack.time_usec_GET() == 2949578273576722788L);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.vy_SET((short)22019) ;
        p115.vz_SET((short)4098) ;
        p115.yawspeed_SET(-1.4256405E38F) ;
        p115.attitude_quaternion_SET(new float[] {-3.8068508E37F, 1.4900139E38F, 3.085717E38F, -3.2921943E38F}, 0) ;
        p115.vx_SET((short) -16279) ;
        p115.rollspeed_SET(1.3868384E38F) ;
        p115.yacc_SET((short)9473) ;
        p115.lon_SET(712707951) ;
        p115.true_airspeed_SET((char)28491) ;
        p115.xacc_SET((short) -21983) ;
        p115.lat_SET(1837883670) ;
        p115.ind_airspeed_SET((char)59392) ;
        p115.pitchspeed_SET(-2.8005454E38F) ;
        p115.zacc_SET((short) -26265) ;
        p115.alt_SET(-188228758) ;
        p115.time_usec_SET(2949578273576722788L) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.xmag_GET() == (short)8665);
            assert(pack.zmag_GET() == (short) -15969);
            assert(pack.xacc_GET() == (short) -32166);
            assert(pack.ygyro_GET() == (short) -31684);
            assert(pack.zacc_GET() == (short)19479);
            assert(pack.yacc_GET() == (short)30652);
            assert(pack.ymag_GET() == (short)4329);
            assert(pack.zgyro_GET() == (short)27734);
            assert(pack.xgyro_GET() == (short)16534);
            assert(pack.time_boot_ms_GET() == 1645456340L);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.ygyro_SET((short) -31684) ;
        p116.time_boot_ms_SET(1645456340L) ;
        p116.zgyro_SET((short)27734) ;
        p116.xacc_SET((short) -32166) ;
        p116.yacc_SET((short)30652) ;
        p116.zmag_SET((short) -15969) ;
        p116.xmag_SET((short)8665) ;
        p116.zacc_SET((short)19479) ;
        p116.ymag_SET((short)4329) ;
        p116.xgyro_SET((short)16534) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_GET() == (char)63399);
            assert(pack.target_component_GET() == (char)26);
            assert(pack.target_system_GET() == (char)33);
            assert(pack.end_GET() == (char)62306);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_component_SET((char)26) ;
        p117.start_SET((char)63399) ;
        p117.end_SET((char)62306) ;
        p117.target_system_SET((char)33) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.time_utc_GET() == 3869070001L);
            assert(pack.id_GET() == (char)41133);
            assert(pack.size_GET() == 4104402804L);
            assert(pack.num_logs_GET() == (char)31647);
            assert(pack.last_log_num_GET() == (char)61977);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.last_log_num_SET((char)61977) ;
        p118.id_SET((char)41133) ;
        p118.num_logs_SET((char)31647) ;
        p118.size_SET(4104402804L) ;
        p118.time_utc_SET(3869070001L) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)19);
            assert(pack.id_GET() == (char)8831);
            assert(pack.ofs_GET() == 3611536187L);
            assert(pack.count_GET() == 3341819805L);
            assert(pack.target_component_GET() == (char)142);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)19) ;
        p119.target_component_SET((char)142) ;
        p119.ofs_SET(3611536187L) ;
        p119.id_SET((char)8831) ;
        p119.count_SET(3341819805L) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)42, (char)35, (char)164, (char)237, (char)232, (char)96, (char)83, (char)196, (char)159, (char)209, (char)106, (char)18, (char)26, (char)112, (char)202, (char)228, (char)150, (char)179, (char)45, (char)111, (char)189, (char)183, (char)31, (char)192, (char)15, (char)229, (char)55, (char)71, (char)199, (char)218, (char)179, (char)219, (char)61, (char)254, (char)174, (char)160, (char)0, (char)66, (char)15, (char)58, (char)145, (char)90, (char)133, (char)226, (char)163, (char)138, (char)140, (char)143, (char)242, (char)200, (char)173, (char)114, (char)121, (char)127, (char)161, (char)42, (char)148, (char)17, (char)88, (char)42, (char)98, (char)52, (char)239, (char)198, (char)233, (char)3, (char)234, (char)32, (char)6, (char)129, (char)233, (char)159, (char)86, (char)112, (char)141, (char)108, (char)172, (char)240, (char)163, (char)43, (char)199, (char)10, (char)199, (char)224, (char)67, (char)176, (char)14, (char)217, (char)106, (char)130}));
            assert(pack.id_GET() == (char)46676);
            assert(pack.ofs_GET() == 3329424896L);
            assert(pack.count_GET() == (char)183);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.ofs_SET(3329424896L) ;
        p120.count_SET((char)183) ;
        p120.id_SET((char)46676) ;
        p120.data__SET(new char[] {(char)42, (char)35, (char)164, (char)237, (char)232, (char)96, (char)83, (char)196, (char)159, (char)209, (char)106, (char)18, (char)26, (char)112, (char)202, (char)228, (char)150, (char)179, (char)45, (char)111, (char)189, (char)183, (char)31, (char)192, (char)15, (char)229, (char)55, (char)71, (char)199, (char)218, (char)179, (char)219, (char)61, (char)254, (char)174, (char)160, (char)0, (char)66, (char)15, (char)58, (char)145, (char)90, (char)133, (char)226, (char)163, (char)138, (char)140, (char)143, (char)242, (char)200, (char)173, (char)114, (char)121, (char)127, (char)161, (char)42, (char)148, (char)17, (char)88, (char)42, (char)98, (char)52, (char)239, (char)198, (char)233, (char)3, (char)234, (char)32, (char)6, (char)129, (char)233, (char)159, (char)86, (char)112, (char)141, (char)108, (char)172, (char)240, (char)163, (char)43, (char)199, (char)10, (char)199, (char)224, (char)67, (char)176, (char)14, (char)217, (char)106, (char)130}, 0) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)113);
            assert(pack.target_component_GET() == (char)188);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_component_SET((char)188) ;
        p121.target_system_SET((char)113) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)80);
            assert(pack.target_system_GET() == (char)191);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)191) ;
        p122.target_component_SET((char)80) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)175, (char)85, (char)199, (char)165, (char)227, (char)214, (char)56, (char)129, (char)168, (char)74, (char)232, (char)115, (char)154, (char)77, (char)155, (char)202, (char)18, (char)249, (char)66, (char)15, (char)27, (char)199, (char)109, (char)36, (char)118, (char)46, (char)24, (char)163, (char)44, (char)74, (char)73, (char)51, (char)48, (char)58, (char)150, (char)197, (char)109, (char)95, (char)27, (char)127, (char)224, (char)75, (char)175, (char)2, (char)181, (char)188, (char)44, (char)25, (char)146, (char)178, (char)191, (char)238, (char)146, (char)231, (char)144, (char)25, (char)162, (char)119, (char)182, (char)173, (char)93, (char)223, (char)203, (char)132, (char)129, (char)141, (char)47, (char)184, (char)48, (char)155, (char)162, (char)94, (char)213, (char)213, (char)45, (char)221, (char)250, (char)100, (char)206, (char)214, (char)131, (char)135, (char)161, (char)203, (char)71, (char)152, (char)166, (char)5, (char)255, (char)182, (char)90, (char)143, (char)80, (char)7, (char)187, (char)62, (char)176, (char)211, (char)70, (char)67, (char)36, (char)109, (char)26, (char)185, (char)92, (char)88, (char)38, (char)205, (char)119, (char)27}));
            assert(pack.target_system_GET() == (char)21);
            assert(pack.target_component_GET() == (char)179);
            assert(pack.len_GET() == (char)141);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.data__SET(new char[] {(char)175, (char)85, (char)199, (char)165, (char)227, (char)214, (char)56, (char)129, (char)168, (char)74, (char)232, (char)115, (char)154, (char)77, (char)155, (char)202, (char)18, (char)249, (char)66, (char)15, (char)27, (char)199, (char)109, (char)36, (char)118, (char)46, (char)24, (char)163, (char)44, (char)74, (char)73, (char)51, (char)48, (char)58, (char)150, (char)197, (char)109, (char)95, (char)27, (char)127, (char)224, (char)75, (char)175, (char)2, (char)181, (char)188, (char)44, (char)25, (char)146, (char)178, (char)191, (char)238, (char)146, (char)231, (char)144, (char)25, (char)162, (char)119, (char)182, (char)173, (char)93, (char)223, (char)203, (char)132, (char)129, (char)141, (char)47, (char)184, (char)48, (char)155, (char)162, (char)94, (char)213, (char)213, (char)45, (char)221, (char)250, (char)100, (char)206, (char)214, (char)131, (char)135, (char)161, (char)203, (char)71, (char)152, (char)166, (char)5, (char)255, (char)182, (char)90, (char)143, (char)80, (char)7, (char)187, (char)62, (char)176, (char)211, (char)70, (char)67, (char)36, (char)109, (char)26, (char)185, (char)92, (char)88, (char)38, (char)205, (char)119, (char)27}, 0) ;
        p123.target_system_SET((char)21) ;
        p123.len_SET((char)141) ;
        p123.target_component_SET((char)179) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1384378838);
            assert(pack.alt_GET() == -1355111444);
            assert(pack.eph_GET() == (char)42542);
            assert(pack.satellites_visible_GET() == (char)115);
            assert(pack.vel_GET() == (char)65452);
            assert(pack.lon_GET() == -2021550188);
            assert(pack.time_usec_GET() == 547227287043383851L);
            assert(pack.epv_GET() == (char)7222);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.dgps_age_GET() == 3898468361L);
            assert(pack.dgps_numch_GET() == (char)174);
            assert(pack.cog_GET() == (char)56094);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.lon_SET(-2021550188) ;
        p124.lat_SET(1384378838) ;
        p124.eph_SET((char)42542) ;
        p124.alt_SET(-1355111444) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p124.dgps_numch_SET((char)174) ;
        p124.epv_SET((char)7222) ;
        p124.vel_SET((char)65452) ;
        p124.dgps_age_SET(3898468361L) ;
        p124.time_usec_SET(547227287043383851L) ;
        p124.satellites_visible_SET((char)115) ;
        p124.cog_SET((char)56094) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vcc_GET() == (char)16947);
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID));
            assert(pack.Vservo_GET() == (char)42486);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vservo_SET((char)42486) ;
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID)) ;
        p125.Vcc_SET((char)16947) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)114);
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING));
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)102, (char)160, (char)249, (char)152, (char)189, (char)172, (char)151, (char)147, (char)145, (char)219, (char)206, (char)251, (char)26, (char)177, (char)200, (char)112, (char)91, (char)224, (char)237, (char)66, (char)16, (char)149, (char)234, (char)49, (char)104, (char)253, (char)100, (char)245, (char)56, (char)160, (char)21, (char)92, (char)18, (char)114, (char)41, (char)243, (char)131, (char)145, (char)19, (char)231, (char)117, (char)58, (char)18, (char)97, (char)44, (char)11, (char)245, (char)148, (char)252, (char)14, (char)57, (char)95, (char)225, (char)123, (char)20, (char)116, (char)90, (char)130, (char)17, (char)145, (char)107, (char)67, (char)131, (char)74, (char)238, (char)44, (char)217, (char)204, (char)65, (char)82}));
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2);
            assert(pack.timeout_GET() == (char)41959);
            assert(pack.baudrate_GET() == 1302442832L);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.data__SET(new char[] {(char)102, (char)160, (char)249, (char)152, (char)189, (char)172, (char)151, (char)147, (char)145, (char)219, (char)206, (char)251, (char)26, (char)177, (char)200, (char)112, (char)91, (char)224, (char)237, (char)66, (char)16, (char)149, (char)234, (char)49, (char)104, (char)253, (char)100, (char)245, (char)56, (char)160, (char)21, (char)92, (char)18, (char)114, (char)41, (char)243, (char)131, (char)145, (char)19, (char)231, (char)117, (char)58, (char)18, (char)97, (char)44, (char)11, (char)245, (char)148, (char)252, (char)14, (char)57, (char)95, (char)225, (char)123, (char)20, (char)116, (char)90, (char)130, (char)17, (char)145, (char)107, (char)67, (char)131, (char)74, (char)238, (char)44, (char)217, (char)204, (char)65, (char)82}, 0) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING)) ;
        p126.count_SET((char)114) ;
        p126.timeout_SET((char)41959) ;
        p126.baudrate_SET(1302442832L) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.time_last_baseline_ms_GET() == 217190848L);
            assert(pack.nsats_GET() == (char)198);
            assert(pack.baseline_b_mm_GET() == 1286944177);
            assert(pack.accuracy_GET() == 612900893L);
            assert(pack.wn_GET() == (char)29027);
            assert(pack.rtk_rate_GET() == (char)143);
            assert(pack.baseline_a_mm_GET() == -1287849310);
            assert(pack.rtk_health_GET() == (char)64);
            assert(pack.iar_num_hypotheses_GET() == 1008659433);
            assert(pack.rtk_receiver_id_GET() == (char)194);
            assert(pack.baseline_c_mm_GET() == -367374494);
            assert(pack.baseline_coords_type_GET() == (char)56);
            assert(pack.tow_GET() == 3497392157L);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.tow_SET(3497392157L) ;
        p127.baseline_a_mm_SET(-1287849310) ;
        p127.rtk_rate_SET((char)143) ;
        p127.baseline_c_mm_SET(-367374494) ;
        p127.time_last_baseline_ms_SET(217190848L) ;
        p127.wn_SET((char)29027) ;
        p127.nsats_SET((char)198) ;
        p127.iar_num_hypotheses_SET(1008659433) ;
        p127.rtk_health_SET((char)64) ;
        p127.accuracy_SET(612900893L) ;
        p127.baseline_coords_type_SET((char)56) ;
        p127.rtk_receiver_id_SET((char)194) ;
        p127.baseline_b_mm_SET(1286944177) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.time_last_baseline_ms_GET() == 63611060L);
            assert(pack.nsats_GET() == (char)242);
            assert(pack.accuracy_GET() == 3785609432L);
            assert(pack.rtk_health_GET() == (char)66);
            assert(pack.rtk_receiver_id_GET() == (char)172);
            assert(pack.rtk_rate_GET() == (char)67);
            assert(pack.baseline_coords_type_GET() == (char)188);
            assert(pack.tow_GET() == 1959040530L);
            assert(pack.iar_num_hypotheses_GET() == 1650037626);
            assert(pack.baseline_a_mm_GET() == -674311103);
            assert(pack.baseline_c_mm_GET() == -310225998);
            assert(pack.baseline_b_mm_GET() == 1386507211);
            assert(pack.wn_GET() == (char)53026);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.tow_SET(1959040530L) ;
        p128.baseline_c_mm_SET(-310225998) ;
        p128.nsats_SET((char)242) ;
        p128.rtk_rate_SET((char)67) ;
        p128.iar_num_hypotheses_SET(1650037626) ;
        p128.accuracy_SET(3785609432L) ;
        p128.rtk_health_SET((char)66) ;
        p128.rtk_receiver_id_SET((char)172) ;
        p128.wn_SET((char)53026) ;
        p128.baseline_b_mm_SET(1386507211) ;
        p128.baseline_coords_type_SET((char)188) ;
        p128.baseline_a_mm_SET(-674311103) ;
        p128.time_last_baseline_ms_SET(63611060L) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short) -18227);
            assert(pack.time_boot_ms_GET() == 2427961056L);
            assert(pack.ymag_GET() == (short) -16115);
            assert(pack.zmag_GET() == (short) -1996);
            assert(pack.ygyro_GET() == (short) -10925);
            assert(pack.xgyro_GET() == (short) -7172);
            assert(pack.zgyro_GET() == (short) -26284);
            assert(pack.yacc_GET() == (short)27146);
            assert(pack.xmag_GET() == (short) -26473);
            assert(pack.zacc_GET() == (short)23916);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.yacc_SET((short)27146) ;
        p129.zacc_SET((short)23916) ;
        p129.ygyro_SET((short) -10925) ;
        p129.xacc_SET((short) -18227) ;
        p129.zmag_SET((short) -1996) ;
        p129.zgyro_SET((short) -26284) ;
        p129.xgyro_SET((short) -7172) ;
        p129.xmag_SET((short) -26473) ;
        p129.time_boot_ms_SET(2427961056L) ;
        p129.ymag_SET((short) -16115) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.jpg_quality_GET() == (char)146);
            assert(pack.width_GET() == (char)65270);
            assert(pack.height_GET() == (char)51423);
            assert(pack.size_GET() == 350148501L);
            assert(pack.type_GET() == (char)103);
            assert(pack.payload_GET() == (char)72);
            assert(pack.packets_GET() == (char)60111);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.type_SET((char)103) ;
        p130.size_SET(350148501L) ;
        p130.payload_SET((char)72) ;
        p130.jpg_quality_SET((char)146) ;
        p130.packets_SET((char)60111) ;
        p130.width_SET((char)65270) ;
        p130.height_SET((char)51423) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)5664);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)117, (char)70, (char)185, (char)223, (char)188, (char)10, (char)15, (char)83, (char)237, (char)167, (char)36, (char)191, (char)45, (char)227, (char)132, (char)6, (char)206, (char)218, (char)133, (char)157, (char)133, (char)56, (char)58, (char)255, (char)99, (char)222, (char)68, (char)232, (char)167, (char)56, (char)110, (char)8, (char)80, (char)83, (char)82, (char)105, (char)112, (char)144, (char)68, (char)172, (char)89, (char)183, (char)206, (char)49, (char)148, (char)52, (char)197, (char)183, (char)197, (char)239, (char)159, (char)103, (char)28, (char)110, (char)246, (char)61, (char)83, (char)229, (char)113, (char)101, (char)162, (char)240, (char)28, (char)3, (char)38, (char)220, (char)108, (char)147, (char)60, (char)91, (char)161, (char)196, (char)33, (char)13, (char)165, (char)108, (char)225, (char)66, (char)164, (char)8, (char)227, (char)243, (char)204, (char)190, (char)111, (char)253, (char)15, (char)251, (char)140, (char)179, (char)127, (char)128, (char)137, (char)67, (char)23, (char)245, (char)158, (char)213, (char)26, (char)133, (char)174, (char)166, (char)247, (char)77, (char)197, (char)220, (char)248, (char)69, (char)27, (char)221, (char)58, (char)255, (char)239, (char)56, (char)112, (char)74, (char)110, (char)108, (char)147, (char)233, (char)76, (char)138, (char)36, (char)72, (char)120, (char)223, (char)252, (char)8, (char)42, (char)44, (char)132, (char)143, (char)42, (char)114, (char)179, (char)161, (char)252, (char)246, (char)15, (char)218, (char)95, (char)216, (char)210, (char)67, (char)96, (char)92, (char)163, (char)139, (char)188, (char)110, (char)116, (char)96, (char)31, (char)203, (char)57, (char)145, (char)66, (char)17, (char)40, (char)76, (char)123, (char)206, (char)54, (char)185, (char)35, (char)97, (char)34, (char)154, (char)103, (char)46, (char)191, (char)134, (char)246, (char)217, (char)52, (char)47, (char)219, (char)172, (char)76, (char)160, (char)113, (char)133, (char)96, (char)176, (char)114, (char)114, (char)88, (char)114, (char)243, (char)117, (char)157, (char)118, (char)120, (char)255, (char)115, (char)24, (char)76, (char)226, (char)161, (char)100, (char)222, (char)5, (char)23, (char)203, (char)184, (char)124, (char)134, (char)49, (char)40, (char)183, (char)133, (char)77, (char)241, (char)243, (char)195, (char)201, (char)133, (char)117, (char)13, (char)235, (char)203, (char)123, (char)236, (char)190, (char)213, (char)170, (char)129, (char)236, (char)136, (char)83, (char)85, (char)117, (char)254, (char)204, (char)54, (char)194, (char)7, (char)236, (char)72, (char)198, (char)128, (char)86, (char)28, (char)87, (char)206, (char)191, (char)92, (char)103, (char)19, (char)51, (char)9, (char)151, (char)220}));
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.data__SET(new char[] {(char)117, (char)70, (char)185, (char)223, (char)188, (char)10, (char)15, (char)83, (char)237, (char)167, (char)36, (char)191, (char)45, (char)227, (char)132, (char)6, (char)206, (char)218, (char)133, (char)157, (char)133, (char)56, (char)58, (char)255, (char)99, (char)222, (char)68, (char)232, (char)167, (char)56, (char)110, (char)8, (char)80, (char)83, (char)82, (char)105, (char)112, (char)144, (char)68, (char)172, (char)89, (char)183, (char)206, (char)49, (char)148, (char)52, (char)197, (char)183, (char)197, (char)239, (char)159, (char)103, (char)28, (char)110, (char)246, (char)61, (char)83, (char)229, (char)113, (char)101, (char)162, (char)240, (char)28, (char)3, (char)38, (char)220, (char)108, (char)147, (char)60, (char)91, (char)161, (char)196, (char)33, (char)13, (char)165, (char)108, (char)225, (char)66, (char)164, (char)8, (char)227, (char)243, (char)204, (char)190, (char)111, (char)253, (char)15, (char)251, (char)140, (char)179, (char)127, (char)128, (char)137, (char)67, (char)23, (char)245, (char)158, (char)213, (char)26, (char)133, (char)174, (char)166, (char)247, (char)77, (char)197, (char)220, (char)248, (char)69, (char)27, (char)221, (char)58, (char)255, (char)239, (char)56, (char)112, (char)74, (char)110, (char)108, (char)147, (char)233, (char)76, (char)138, (char)36, (char)72, (char)120, (char)223, (char)252, (char)8, (char)42, (char)44, (char)132, (char)143, (char)42, (char)114, (char)179, (char)161, (char)252, (char)246, (char)15, (char)218, (char)95, (char)216, (char)210, (char)67, (char)96, (char)92, (char)163, (char)139, (char)188, (char)110, (char)116, (char)96, (char)31, (char)203, (char)57, (char)145, (char)66, (char)17, (char)40, (char)76, (char)123, (char)206, (char)54, (char)185, (char)35, (char)97, (char)34, (char)154, (char)103, (char)46, (char)191, (char)134, (char)246, (char)217, (char)52, (char)47, (char)219, (char)172, (char)76, (char)160, (char)113, (char)133, (char)96, (char)176, (char)114, (char)114, (char)88, (char)114, (char)243, (char)117, (char)157, (char)118, (char)120, (char)255, (char)115, (char)24, (char)76, (char)226, (char)161, (char)100, (char)222, (char)5, (char)23, (char)203, (char)184, (char)124, (char)134, (char)49, (char)40, (char)183, (char)133, (char)77, (char)241, (char)243, (char)195, (char)201, (char)133, (char)117, (char)13, (char)235, (char)203, (char)123, (char)236, (char)190, (char)213, (char)170, (char)129, (char)236, (char)136, (char)83, (char)85, (char)117, (char)254, (char)204, (char)54, (char)194, (char)7, (char)236, (char)72, (char)198, (char)128, (char)86, (char)28, (char)87, (char)206, (char)191, (char)92, (char)103, (char)19, (char)51, (char)9, (char)151, (char)220}, 0) ;
        p131.seqnr_SET((char)5664) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.covariance_GET() == (char)250);
            assert(pack.id_GET() == (char)227);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_225);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            assert(pack.current_distance_GET() == (char)7902);
            assert(pack.time_boot_ms_GET() == 579798069L);
            assert(pack.max_distance_GET() == (char)52877);
            assert(pack.min_distance_GET() == (char)35491);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR) ;
        p132.covariance_SET((char)250) ;
        p132.max_distance_SET((char)52877) ;
        p132.id_SET((char)227) ;
        p132.time_boot_ms_SET(579798069L) ;
        p132.current_distance_SET((char)7902) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_225) ;
        p132.min_distance_SET((char)35491) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.grid_spacing_GET() == (char)57114);
            assert(pack.lon_GET() == 1049937779);
            assert(pack.mask_GET() == 7810465713447636169L);
            assert(pack.lat_GET() == 1077831127);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(1077831127) ;
        p133.grid_spacing_SET((char)57114) ;
        p133.mask_SET(7810465713447636169L) ;
        p133.lon_SET(1049937779) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 1567863874);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -9291, (short)26281, (short)21279, (short) -26399, (short)27033, (short)22006, (short)12543, (short) -9068, (short) -25597, (short) -15995, (short) -26580, (short) -832, (short) -31572, (short) -13765, (short) -25609, (short) -14727}));
            assert(pack.gridbit_GET() == (char)43);
            assert(pack.lat_GET() == 722143904);
            assert(pack.grid_spacing_GET() == (char)35302);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.gridbit_SET((char)43) ;
        p134.data__SET(new short[] {(short) -9291, (short)26281, (short)21279, (short) -26399, (short)27033, (short)22006, (short)12543, (short) -9068, (short) -25597, (short) -15995, (short) -26580, (short) -832, (short) -31572, (short) -13765, (short) -25609, (short) -14727}, 0) ;
        p134.grid_spacing_SET((char)35302) ;
        p134.lon_SET(1567863874) ;
        p134.lat_SET(722143904) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 15737528);
            assert(pack.lon_GET() == 997315636);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(15737528) ;
        p135.lon_SET(997315636) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.loaded_GET() == (char)16438);
            assert(pack.current_height_GET() == -7.7757896E37F);
            assert(pack.spacing_GET() == (char)16144);
            assert(pack.lat_GET() == -259134510);
            assert(pack.terrain_height_GET() == 5.145986E37F);
            assert(pack.pending_GET() == (char)7322);
            assert(pack.lon_GET() == 484545099);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.spacing_SET((char)16144) ;
        p136.lat_SET(-259134510) ;
        p136.terrain_height_SET(5.145986E37F) ;
        p136.loaded_SET((char)16438) ;
        p136.lon_SET(484545099) ;
        p136.pending_SET((char)7322) ;
        p136.current_height_SET(-7.7757896E37F) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == 4.9466186E37F);
            assert(pack.time_boot_ms_GET() == 4177979338L);
            assert(pack.press_diff_GET() == 2.7141142E38F);
            assert(pack.temperature_GET() == (short)28368);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.press_diff_SET(2.7141142E38F) ;
        p137.time_boot_ms_SET(4177979338L) ;
        p137.temperature_SET((short)28368) ;
        p137.press_abs_SET(4.9466186E37F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 2.5352294E38F);
            assert(pack.y_GET() == 2.5642607E37F);
            assert(pack.time_usec_GET() == 5762041722683935661L);
            assert(pack.z_GET() == -1.8493211E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {4.425892E37F, 1.8031151E38F, 1.6718239E38F, -2.0194495E38F}));
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.q_SET(new float[] {4.425892E37F, 1.8031151E38F, 1.6718239E38F, -2.0194495E38F}, 0) ;
        p138.time_usec_SET(5762041722683935661L) ;
        p138.z_SET(-1.8493211E38F) ;
        p138.x_SET(2.5352294E38F) ;
        p138.y_SET(2.5642607E37F) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)87);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.0236981E38F, 4.652134E37F, -8.4488923E37F, 7.442756E37F, -7.1478543E37F, 1.8606446E38F, 2.104637E38F, -2.8801107E38F}));
            assert(pack.group_mlx_GET() == (char)247);
            assert(pack.target_system_GET() == (char)37);
            assert(pack.time_usec_GET() == 5032404832036855258L);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.controls_SET(new float[] {2.0236981E38F, 4.652134E37F, -8.4488923E37F, 7.442756E37F, -7.1478543E37F, 1.8606446E38F, 2.104637E38F, -2.8801107E38F}, 0) ;
        p139.group_mlx_SET((char)247) ;
        p139.target_component_SET((char)87) ;
        p139.time_usec_SET(5032404832036855258L) ;
        p139.target_system_SET((char)37) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)133);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.9862734E38F, 9.115977E37F, 2.1549038E38F, 1.6959589E38F, 2.2087816E38F, -6.822892E37F, 1.1365395E38F, 2.8846961E38F}));
            assert(pack.time_usec_GET() == 1207885867925808460L);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.group_mlx_SET((char)133) ;
        p140.time_usec_SET(1207885867925808460L) ;
        p140.controls_SET(new float[] {1.9862734E38F, 9.115977E37F, 2.1549038E38F, 1.6959589E38F, 2.2087816E38F, -6.822892E37F, 1.1365395E38F, 2.8846961E38F}, 0) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_local_GET() == 2.5999159E38F);
            assert(pack.altitude_monotonic_GET() == 6.6157524E37F);
            assert(pack.altitude_terrain_GET() == 2.2490515E38F);
            assert(pack.altitude_amsl_GET() == 1.5926474E38F);
            assert(pack.bottom_clearance_GET() == -1.9025669E38F);
            assert(pack.altitude_relative_GET() == 2.9946288E38F);
            assert(pack.time_usec_GET() == 1938118582071299648L);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_local_SET(2.5999159E38F) ;
        p141.time_usec_SET(1938118582071299648L) ;
        p141.altitude_amsl_SET(1.5926474E38F) ;
        p141.altitude_relative_SET(2.9946288E38F) ;
        p141.altitude_terrain_SET(2.2490515E38F) ;
        p141.altitude_monotonic_SET(6.6157524E37F) ;
        p141.bottom_clearance_SET(-1.9025669E38F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)53, (char)123, (char)153, (char)42, (char)84, (char)200, (char)218, (char)4, (char)132, (char)46, (char)109, (char)99, (char)161, (char)190, (char)129, (char)180, (char)112, (char)176, (char)59, (char)130, (char)50, (char)222, (char)111, (char)61, (char)219, (char)89, (char)113, (char)120, (char)53, (char)61, (char)33, (char)22, (char)35, (char)8, (char)18, (char)125, (char)205, (char)111, (char)7, (char)187, (char)39, (char)15, (char)6, (char)178, (char)193, (char)152, (char)57, (char)249, (char)160, (char)122, (char)218, (char)219, (char)141, (char)128, (char)248, (char)116, (char)139, (char)135, (char)208, (char)115, (char)85, (char)13, (char)31, (char)121, (char)212, (char)41, (char)75, (char)15, (char)198, (char)197, (char)214, (char)161, (char)40, (char)59, (char)19, (char)49, (char)131, (char)156, (char)59, (char)112, (char)88, (char)173, (char)242, (char)4, (char)130, (char)40, (char)22, (char)30, (char)122, (char)114, (char)144, (char)161, (char)0, (char)127, (char)165, (char)239, (char)100, (char)164, (char)203, (char)130, (char)225, (char)132, (char)127, (char)224, (char)207, (char)191, (char)212, (char)136, (char)132, (char)206, (char)147, (char)70, (char)122, (char)43, (char)209, (char)208, (char)66, (char)121, (char)253, (char)126}));
            assert(pack.transfer_type_GET() == (char)22);
            assert(pack.request_id_GET() == (char)120);
            assert(pack.uri_type_GET() == (char)39);
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)229, (char)232, (char)85, (char)219, (char)130, (char)112, (char)1, (char)18, (char)175, (char)115, (char)32, (char)80, (char)251, (char)71, (char)163, (char)87, (char)230, (char)35, (char)15, (char)230, (char)105, (char)158, (char)158, (char)58, (char)88, (char)78, (char)157, (char)99, (char)187, (char)203, (char)84, (char)133, (char)26, (char)198, (char)60, (char)127, (char)223, (char)15, (char)25, (char)208, (char)31, (char)146, (char)8, (char)68, (char)133, (char)219, (char)107, (char)245, (char)68, (char)153, (char)129, (char)171, (char)191, (char)207, (char)58, (char)82, (char)171, (char)108, (char)37, (char)247, (char)75, (char)108, (char)239, (char)193, (char)27, (char)240, (char)92, (char)97, (char)0, (char)129, (char)210, (char)205, (char)52, (char)250, (char)44, (char)81, (char)154, (char)116, (char)73, (char)131, (char)24, (char)21, (char)46, (char)227, (char)247, (char)83, (char)49, (char)179, (char)96, (char)138, (char)62, (char)197, (char)182, (char)173, (char)10, (char)27, (char)163, (char)201, (char)1, (char)200, (char)114, (char)105, (char)78, (char)254, (char)36, (char)213, (char)142, (char)97, (char)216, (char)73, (char)173, (char)149, (char)132, (char)143, (char)50, (char)128, (char)192, (char)33, (char)96, (char)43}));
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.request_id_SET((char)120) ;
        p142.uri_type_SET((char)39) ;
        p142.uri_SET(new char[] {(char)229, (char)232, (char)85, (char)219, (char)130, (char)112, (char)1, (char)18, (char)175, (char)115, (char)32, (char)80, (char)251, (char)71, (char)163, (char)87, (char)230, (char)35, (char)15, (char)230, (char)105, (char)158, (char)158, (char)58, (char)88, (char)78, (char)157, (char)99, (char)187, (char)203, (char)84, (char)133, (char)26, (char)198, (char)60, (char)127, (char)223, (char)15, (char)25, (char)208, (char)31, (char)146, (char)8, (char)68, (char)133, (char)219, (char)107, (char)245, (char)68, (char)153, (char)129, (char)171, (char)191, (char)207, (char)58, (char)82, (char)171, (char)108, (char)37, (char)247, (char)75, (char)108, (char)239, (char)193, (char)27, (char)240, (char)92, (char)97, (char)0, (char)129, (char)210, (char)205, (char)52, (char)250, (char)44, (char)81, (char)154, (char)116, (char)73, (char)131, (char)24, (char)21, (char)46, (char)227, (char)247, (char)83, (char)49, (char)179, (char)96, (char)138, (char)62, (char)197, (char)182, (char)173, (char)10, (char)27, (char)163, (char)201, (char)1, (char)200, (char)114, (char)105, (char)78, (char)254, (char)36, (char)213, (char)142, (char)97, (char)216, (char)73, (char)173, (char)149, (char)132, (char)143, (char)50, (char)128, (char)192, (char)33, (char)96, (char)43}, 0) ;
        p142.transfer_type_SET((char)22) ;
        p142.storage_SET(new char[] {(char)53, (char)123, (char)153, (char)42, (char)84, (char)200, (char)218, (char)4, (char)132, (char)46, (char)109, (char)99, (char)161, (char)190, (char)129, (char)180, (char)112, (char)176, (char)59, (char)130, (char)50, (char)222, (char)111, (char)61, (char)219, (char)89, (char)113, (char)120, (char)53, (char)61, (char)33, (char)22, (char)35, (char)8, (char)18, (char)125, (char)205, (char)111, (char)7, (char)187, (char)39, (char)15, (char)6, (char)178, (char)193, (char)152, (char)57, (char)249, (char)160, (char)122, (char)218, (char)219, (char)141, (char)128, (char)248, (char)116, (char)139, (char)135, (char)208, (char)115, (char)85, (char)13, (char)31, (char)121, (char)212, (char)41, (char)75, (char)15, (char)198, (char)197, (char)214, (char)161, (char)40, (char)59, (char)19, (char)49, (char)131, (char)156, (char)59, (char)112, (char)88, (char)173, (char)242, (char)4, (char)130, (char)40, (char)22, (char)30, (char)122, (char)114, (char)144, (char)161, (char)0, (char)127, (char)165, (char)239, (char)100, (char)164, (char)203, (char)130, (char)225, (char)132, (char)127, (char)224, (char)207, (char)191, (char)212, (char)136, (char)132, (char)206, (char)147, (char)70, (char)122, (char)43, (char)209, (char)208, (char)66, (char)121, (char)253, (char)126}, 0) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == -2.869997E37F);
            assert(pack.temperature_GET() == (short)4286);
            assert(pack.press_abs_GET() == 1.3820648E38F);
            assert(pack.time_boot_ms_GET() == 3010063861L);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.press_abs_SET(1.3820648E38F) ;
        p143.time_boot_ms_SET(3010063861L) ;
        p143.press_diff_SET(-2.869997E37F) ;
        p143.temperature_SET((short)4286) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1301641626);
            assert(pack.alt_GET() == 3.286162E38F);
            assert(Arrays.equals(pack.vel_GET(),  new float[] {-2.9521566E38F, -1.944033E38F, -2.0782138E38F}));
            assert(pack.timestamp_GET() == 744911886182509258L);
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {3.0677175E38F, -9.379781E37F, 2.7633657E38F, 2.0674063E38F}));
            assert(pack.custom_state_GET() == 128101950229359189L);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-5.4779607E36F, -5.9232166E36F, 3.208889E38F}));
            assert(pack.lon_GET() == 1868815626);
            assert(pack.est_capabilities_GET() == (char)205);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {1.1141723E38F, -9.998685E37F, 2.7446025E38F}));
            assert(Arrays.equals(pack.rates_GET(),  new float[] {1.9752203E38F, 3.1385455E38F, 9.735077E37F}));
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.timestamp_SET(744911886182509258L) ;
        p144.acc_SET(new float[] {1.1141723E38F, -9.998685E37F, 2.7446025E38F}, 0) ;
        p144.lat_SET(-1301641626) ;
        p144.alt_SET(3.286162E38F) ;
        p144.attitude_q_SET(new float[] {3.0677175E38F, -9.379781E37F, 2.7633657E38F, 2.0674063E38F}, 0) ;
        p144.position_cov_SET(new float[] {-5.4779607E36F, -5.9232166E36F, 3.208889E38F}, 0) ;
        p144.rates_SET(new float[] {1.9752203E38F, 3.1385455E38F, 9.735077E37F}, 0) ;
        p144.lon_SET(1868815626) ;
        p144.vel_SET(new float[] {-2.9521566E38F, -1.944033E38F, -2.0782138E38F}, 0) ;
        p144.est_capabilities_SET((char)205) ;
        p144.custom_state_SET(128101950229359189L) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.x_pos_GET() == 1.0771253E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.8250984E38F, -1.808125E38F, -3.0367298E38F, -1.4115872E38F}));
            assert(pack.x_acc_GET() == -5.7310605E37F);
            assert(pack.y_acc_GET() == -9.026213E37F);
            assert(pack.pitch_rate_GET() == -7.446146E37F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {3.099845E38F, 3.2516867E38F, 3.1159046E38F}));
            assert(pack.z_acc_GET() == -1.7184588E38F);
            assert(pack.z_vel_GET() == 2.8708138E38F);
            assert(pack.z_pos_GET() == -7.851552E36F);
            assert(pack.y_vel_GET() == 2.2306349E38F);
            assert(pack.y_pos_GET() == -2.0193984E38F);
            assert(pack.airspeed_GET() == -2.0090013E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {2.4451118E38F, 2.8458308E38F, -3.069658E38F}));
            assert(pack.time_usec_GET() == 6600777037324013051L);
            assert(pack.x_vel_GET() == -2.313586E38F);
            assert(pack.yaw_rate_GET() == 2.727821E36F);
            assert(pack.roll_rate_GET() == 1.0428589E38F);
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.y_vel_SET(2.2306349E38F) ;
        p146.z_acc_SET(-1.7184588E38F) ;
        p146.pitch_rate_SET(-7.446146E37F) ;
        p146.x_pos_SET(1.0771253E38F) ;
        p146.time_usec_SET(6600777037324013051L) ;
        p146.x_acc_SET(-5.7310605E37F) ;
        p146.x_vel_SET(-2.313586E38F) ;
        p146.y_acc_SET(-9.026213E37F) ;
        p146.q_SET(new float[] {-1.8250984E38F, -1.808125E38F, -3.0367298E38F, -1.4115872E38F}, 0) ;
        p146.airspeed_SET(-2.0090013E38F) ;
        p146.z_vel_SET(2.8708138E38F) ;
        p146.pos_variance_SET(new float[] {3.099845E38F, 3.2516867E38F, 3.1159046E38F}, 0) ;
        p146.roll_rate_SET(1.0428589E38F) ;
        p146.vel_variance_SET(new float[] {2.4451118E38F, 2.8458308E38F, -3.069658E38F}, 0) ;
        p146.yaw_rate_SET(2.727821E36F) ;
        p146.z_pos_SET(-7.851552E36F) ;
        p146.y_pos_SET(-2.0193984E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -25175);
            assert(pack.battery_remaining_GET() == (byte) - 52);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)10969, (char)53563, (char)48321, (char)18834, (char)34622, (char)22056, (char)51423, (char)8020, (char)64536, (char)43714}));
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
            assert(pack.current_consumed_GET() == -1803329729);
            assert(pack.current_battery_GET() == (short)32584);
            assert(pack.id_GET() == (char)250);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
            assert(pack.energy_consumed_GET() == 1585923521);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE) ;
        p147.voltages_SET(new char[] {(char)10969, (char)53563, (char)48321, (char)18834, (char)34622, (char)22056, (char)51423, (char)8020, (char)64536, (char)43714}, 0) ;
        p147.energy_consumed_SET(1585923521) ;
        p147.id_SET((char)250) ;
        p147.temperature_SET((short) -25175) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD) ;
        p147.battery_remaining_SET((byte) - 52) ;
        p147.current_consumed_SET(-1803329729) ;
        p147.current_battery_SET((short)32584) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.product_id_GET() == (char)3647);
            assert(pack.os_sw_version_GET() == 3604650724L);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)111, (char)144, (char)175, (char)185, (char)102, (char)215, (char)245, (char)254}));
            assert(pack.middleware_sw_version_GET() == 2307671273L);
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION));
            assert(pack.board_version_GET() == 916607887L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)243, (char)180, (char)217, (char)249, (char)140, (char)62, (char)66, (char)29, (char)3, (char)54, (char)117, (char)130, (char)59, (char)216, (char)208, (char)221, (char)174, (char)232}));
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)159, (char)19, (char)13, (char)225, (char)43, (char)81, (char)165, (char)41}));
            assert(pack.flight_sw_version_GET() == 1325949936L);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)241, (char)49, (char)60, (char)171, (char)69, (char)65, (char)163, (char)103}));
            assert(pack.vendor_id_GET() == (char)64945);
            assert(pack.uid_GET() == 6620667501350033071L);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.board_version_SET(916607887L) ;
        p148.flight_sw_version_SET(1325949936L) ;
        p148.vendor_id_SET((char)64945) ;
        p148.os_custom_version_SET(new char[] {(char)159, (char)19, (char)13, (char)225, (char)43, (char)81, (char)165, (char)41}, 0) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION)) ;
        p148.uid2_SET(new char[] {(char)243, (char)180, (char)217, (char)249, (char)140, (char)62, (char)66, (char)29, (char)3, (char)54, (char)117, (char)130, (char)59, (char)216, (char)208, (char)221, (char)174, (char)232}, 0, PH) ;
        p148.middleware_custom_version_SET(new char[] {(char)241, (char)49, (char)60, (char)171, (char)69, (char)65, (char)163, (char)103}, 0) ;
        p148.flight_custom_version_SET(new char[] {(char)111, (char)144, (char)175, (char)185, (char)102, (char)215, (char)245, (char)254}, 0) ;
        p148.os_sw_version_SET(3604650724L) ;
        p148.middleware_sw_version_SET(2307671273L) ;
        p148.uid_SET(6620667501350033071L) ;
        p148.product_id_SET((char)3647) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.angle_y_GET() == -1.3861902E38F);
            assert(pack.y_TRY(ph) == -1.1604294E38F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {4.4345247E37F, -2.5595358E38F, 3.148872E38F, -1.752305E38F}));
            assert(pack.size_x_GET() == 1.5284418E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.angle_x_GET() == -8.593302E37F);
            assert(pack.x_TRY(ph) == -3.223117E38F);
            assert(pack.time_usec_GET() == 548444226658786113L);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
            assert(pack.target_num_GET() == (char)108);
            assert(pack.position_valid_TRY(ph) == (char)218);
            assert(pack.size_y_GET() == -2.2758904E38F);
            assert(pack.z_TRY(ph) == 2.6411553E38F);
            assert(pack.distance_GET() == -6.888968E37F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p149.size_x_SET(1.5284418E38F) ;
        p149.size_y_SET(-2.2758904E38F) ;
        p149.x_SET(-3.223117E38F, PH) ;
        p149.y_SET(-1.1604294E38F, PH) ;
        p149.target_num_SET((char)108) ;
        p149.distance_SET(-6.888968E37F) ;
        p149.q_SET(new float[] {4.4345247E37F, -2.5595358E38F, 3.148872E38F, -1.752305E38F}, 0, PH) ;
        p149.position_valid_SET((char)218, PH) ;
        p149.time_usec_SET(548444226658786113L) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER) ;
        p149.angle_y_SET(-1.3861902E38F) ;
        p149.z_SET(2.6411553E38F, PH) ;
        p149.angle_x_SET(-8.593302E37F) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_SET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)226);
            assert(pack.target_component_GET() == (char)28);
        });
        GroundControl.FLEXIFUNCTION_SET p150 = CommunicationChannel.new_FLEXIFUNCTION_SET();
        PH.setPack(p150);
        p150.target_component_SET((char)28) ;
        p150.target_system_SET((char)226) ;
        CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_READ_REQ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)200);
            assert(pack.data_index_GET() == (short)21240);
            assert(pack.read_req_type_GET() == (short) -17500);
            assert(pack.target_component_GET() == (char)140);
        });
        GroundControl.FLEXIFUNCTION_READ_REQ p151 = CommunicationChannel.new_FLEXIFUNCTION_READ_REQ();
        PH.setPack(p151);
        p151.read_req_type_SET((short) -17500) ;
        p151.target_component_SET((char)140) ;
        p151.data_index_SET((short)21240) ;
        p151.target_system_SET((char)200) ;
        CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_BUFFER_FUNCTION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new byte[] {(byte)126, (byte) - 90, (byte)24, (byte) - 75, (byte)17, (byte) - 72, (byte) - 84, (byte) - 120, (byte)16, (byte)73, (byte)90, (byte)3, (byte)51, (byte) - 105, (byte)57, (byte) - 101, (byte)23, (byte)45, (byte) - 40, (byte)114, (byte)14, (byte) - 25, (byte)50, (byte) - 75, (byte) - 22, (byte) - 65, (byte) - 96, (byte)56, (byte) - 28, (byte)52, (byte)37, (byte)102, (byte) - 16, (byte) - 16, (byte) - 79, (byte) - 109, (byte)78, (byte) - 122, (byte) - 110, (byte)33, (byte) - 34, (byte) - 12, (byte) - 61, (byte) - 25, (byte)110, (byte)9, (byte)42, (byte)68}));
            assert(pack.func_index_GET() == (char)55434);
            assert(pack.target_component_GET() == (char)238);
            assert(pack.data_size_GET() == (char)57756);
            assert(pack.target_system_GET() == (char)243);
            assert(pack.func_count_GET() == (char)30721);
            assert(pack.data_address_GET() == (char)6548);
        });
        GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION p152 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION();
        PH.setPack(p152);
        p152.func_index_SET((char)55434) ;
        p152.func_count_SET((char)30721) ;
        p152.target_component_SET((char)238) ;
        p152.target_system_SET((char)243) ;
        p152.data__SET(new byte[] {(byte)126, (byte) - 90, (byte)24, (byte) - 75, (byte)17, (byte) - 72, (byte) - 84, (byte) - 120, (byte)16, (byte)73, (byte)90, (byte)3, (byte)51, (byte) - 105, (byte)57, (byte) - 101, (byte)23, (byte)45, (byte) - 40, (byte)114, (byte)14, (byte) - 25, (byte)50, (byte) - 75, (byte) - 22, (byte) - 65, (byte) - 96, (byte)56, (byte) - 28, (byte)52, (byte)37, (byte)102, (byte) - 16, (byte) - 16, (byte) - 79, (byte) - 109, (byte)78, (byte) - 122, (byte) - 110, (byte)33, (byte) - 34, (byte) - 12, (byte) - 61, (byte) - 25, (byte)110, (byte)9, (byte)42, (byte)68}, 0) ;
        p152.data_size_SET((char)57756) ;
        p152.data_address_SET((char)6548) ;
        CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_BUFFER_FUNCTION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)59);
            assert(pack.result_GET() == (char)25016);
            assert(pack.func_index_GET() == (char)53855);
            assert(pack.target_system_GET() == (char)120);
        });
        GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION_ACK p153 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
        PH.setPack(p153);
        p153.func_index_SET((char)53855) ;
        p153.target_component_SET((char)59) ;
        p153.target_system_SET((char)120) ;
        p153.result_SET((char)25016) ;
        CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_DIRECTORY.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)173);
            assert(pack.target_component_GET() == (char)174);
            assert(pack.start_index_GET() == (char)224);
            assert(pack.directory_type_GET() == (char)161);
            assert(pack.target_system_GET() == (char)26);
            assert(Arrays.equals(pack.directory_data_GET(),  new byte[] {(byte) - 12, (byte) - 55, (byte) - 88, (byte) - 122, (byte)40, (byte) - 107, (byte) - 32, (byte) - 65, (byte) - 48, (byte)24, (byte)93, (byte) - 3, (byte)46, (byte) - 94, (byte)18, (byte) - 74, (byte)6, (byte)37, (byte) - 27, (byte)69, (byte)49, (byte) - 109, (byte)53, (byte)29, (byte) - 1, (byte)59, (byte)83, (byte) - 11, (byte) - 76, (byte) - 54, (byte)75, (byte)101, (byte)7, (byte) - 33, (byte) - 125, (byte) - 118, (byte)101, (byte)23, (byte)14, (byte) - 3, (byte) - 72, (byte)83, (byte) - 87, (byte)12, (byte)59, (byte)36, (byte)97, (byte) - 83}));
        });
        GroundControl.FLEXIFUNCTION_DIRECTORY p155 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY();
        PH.setPack(p155);
        p155.count_SET((char)173) ;
        p155.target_system_SET((char)26) ;
        p155.start_index_SET((char)224) ;
        p155.directory_data_SET(new byte[] {(byte) - 12, (byte) - 55, (byte) - 88, (byte) - 122, (byte)40, (byte) - 107, (byte) - 32, (byte) - 65, (byte) - 48, (byte)24, (byte)93, (byte) - 3, (byte)46, (byte) - 94, (byte)18, (byte) - 74, (byte)6, (byte)37, (byte) - 27, (byte)69, (byte)49, (byte) - 109, (byte)53, (byte)29, (byte) - 1, (byte)59, (byte)83, (byte) - 11, (byte) - 76, (byte) - 54, (byte)75, (byte)101, (byte)7, (byte) - 33, (byte) - 125, (byte) - 118, (byte)101, (byte)23, (byte)14, (byte) - 3, (byte) - 72, (byte)83, (byte) - 87, (byte)12, (byte)59, (byte)36, (byte)97, (byte) - 83}, 0) ;
        p155.target_component_SET((char)174) ;
        p155.directory_type_SET((char)161) ;
        CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_DIRECTORY_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)179);
            assert(pack.directory_type_GET() == (char)202);
            assert(pack.target_component_GET() == (char)68);
            assert(pack.result_GET() == (char)27369);
            assert(pack.start_index_GET() == (char)27);
            assert(pack.count_GET() == (char)122);
        });
        GroundControl.FLEXIFUNCTION_DIRECTORY_ACK p156 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY_ACK();
        PH.setPack(p156);
        p156.start_index_SET((char)27) ;
        p156.target_system_SET((char)179) ;
        p156.target_component_SET((char)68) ;
        p156.directory_type_SET((char)202) ;
        p156.result_SET((char)27369) ;
        p156.count_SET((char)122) ;
        CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_COMMAND.add((src, ph, pack) ->
        {
            assert(pack.command_type_GET() == (char)225);
            assert(pack.target_component_GET() == (char)121);
            assert(pack.target_system_GET() == (char)61);
        });
        GroundControl.FLEXIFUNCTION_COMMAND p157 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND();
        PH.setPack(p157);
        p157.target_system_SET((char)61) ;
        p157.command_type_SET((char)225) ;
        p157.target_component_SET((char)121) ;
        CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.command_type_GET() == (char)1896);
            assert(pack.result_GET() == (char)26316);
        });
        GroundControl.FLEXIFUNCTION_COMMAND_ACK p158 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND_ACK();
        PH.setPack(p158);
        p158.command_type_SET((char)1896) ;
        p158.result_SET((char)26316) ;
        CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F2_A.add((src, ph, pack) ->
        {
            assert(pack.sue_rmat1_GET() == (short) -19278);
            assert(pack.sue_rmat7_GET() == (short)12014);
            assert(pack.sue_magFieldEarth0_GET() == (short)21128);
            assert(pack.sue_estimated_wind_0_GET() == (short) -19272);
            assert(pack.sue_longitude_GET() == 1644844853);
            assert(pack.sue_rmat8_GET() == (short)13241);
            assert(pack.sue_rmat2_GET() == (short)32029);
            assert(pack.sue_time_GET() == 10902249L);
            assert(pack.sue_rmat5_GET() == (short)30006);
            assert(pack.sue_sog_GET() == (short) -8215);
            assert(pack.sue_cog_GET() == (char)39310);
            assert(pack.sue_latitude_GET() == 857750539);
            assert(pack.sue_rmat6_GET() == (short)9489);
            assert(pack.sue_rmat3_GET() == (short)18874);
            assert(pack.sue_cpu_load_GET() == (char)10173);
            assert(pack.sue_estimated_wind_1_GET() == (short) -19433);
            assert(pack.sue_air_speed_3DIMU_GET() == (char)49420);
            assert(pack.sue_status_GET() == (char)185);
            assert(pack.sue_rmat4_GET() == (short)3754);
            assert(pack.sue_waypoint_index_GET() == (char)28626);
            assert(pack.sue_rmat0_GET() == (short)25076);
            assert(pack.sue_altitude_GET() == -2095822791);
            assert(pack.sue_magFieldEarth1_GET() == (short)5527);
            assert(pack.sue_hdop_GET() == (short) -11336);
            assert(pack.sue_magFieldEarth2_GET() == (short) -6383);
            assert(pack.sue_svs_GET() == (short) -17471);
            assert(pack.sue_estimated_wind_2_GET() == (short) -5247);
        });
        GroundControl.SERIAL_UDB_EXTRA_F2_A p170 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_A();
        PH.setPack(p170);
        p170.sue_estimated_wind_0_SET((short) -19272) ;
        p170.sue_altitude_SET(-2095822791) ;
        p170.sue_rmat5_SET((short)30006) ;
        p170.sue_magFieldEarth2_SET((short) -6383) ;
        p170.sue_estimated_wind_2_SET((short) -5247) ;
        p170.sue_air_speed_3DIMU_SET((char)49420) ;
        p170.sue_magFieldEarth1_SET((short)5527) ;
        p170.sue_rmat6_SET((short)9489) ;
        p170.sue_rmat0_SET((short)25076) ;
        p170.sue_rmat7_SET((short)12014) ;
        p170.sue_rmat3_SET((short)18874) ;
        p170.sue_status_SET((char)185) ;
        p170.sue_longitude_SET(1644844853) ;
        p170.sue_rmat2_SET((short)32029) ;
        p170.sue_magFieldEarth0_SET((short)21128) ;
        p170.sue_rmat8_SET((short)13241) ;
        p170.sue_svs_SET((short) -17471) ;
        p170.sue_cog_SET((char)39310) ;
        p170.sue_estimated_wind_1_SET((short) -19433) ;
        p170.sue_latitude_SET(857750539) ;
        p170.sue_rmat4_SET((short)3754) ;
        p170.sue_rmat1_SET((short) -19278) ;
        p170.sue_sog_SET((short) -8215) ;
        p170.sue_waypoint_index_SET((char)28626) ;
        p170.sue_time_SET(10902249L) ;
        p170.sue_hdop_SET((short) -11336) ;
        p170.sue_cpu_load_SET((char)10173) ;
        CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F2_B.add((src, ph, pack) ->
        {
            assert(pack.sue_imu_velocity_y_GET() == (short) -22606);
            assert(pack.sue_pwm_output_7_GET() == (short)4741);
            assert(pack.sue_pwm_output_9_GET() == (short) -29324);
            assert(pack.sue_pwm_output_8_GET() == (short) -31151);
            assert(pack.sue_pwm_input_6_GET() == (short)8517);
            assert(pack.sue_aero_z_GET() == (short) -10460);
            assert(pack.sue_pwm_output_3_GET() == (short)5560);
            assert(pack.sue_pwm_input_5_GET() == (short)18814);
            assert(pack.sue_pwm_output_1_GET() == (short) -17992);
            assert(pack.sue_pwm_input_2_GET() == (short) -28832);
            assert(pack.sue_location_error_earth_z_GET() == (short)13462);
            assert(pack.sue_location_error_earth_x_GET() == (short)28611);
            assert(pack.sue_imu_location_x_GET() == (short) -22582);
            assert(pack.sue_osc_fails_GET() == (short) -5252);
            assert(pack.sue_aero_y_GET() == (short)31948);
            assert(pack.sue_bat_volt_GET() == (short)21145);
            assert(pack.sue_pwm_input_7_GET() == (short) -32359);
            assert(pack.sue_pwm_input_11_GET() == (short) -17065);
            assert(pack.sue_pwm_input_9_GET() == (short) -3683);
            assert(pack.sue_bat_amp_hours_GET() == (short) -18649);
            assert(pack.sue_pwm_output_2_GET() == (short) -22701);
            assert(pack.sue_imu_velocity_x_GET() == (short)16236);
            assert(pack.sue_pwm_input_3_GET() == (short)12142);
            assert(pack.sue_pwm_input_8_GET() == (short)24319);
            assert(pack.sue_pwm_input_4_GET() == (short)17587);
            assert(pack.sue_bat_amp_GET() == (short)9418);
            assert(pack.sue_imu_location_z_GET() == (short) -30560);
            assert(pack.sue_pwm_output_5_GET() == (short) -32249);
            assert(pack.sue_pwm_output_4_GET() == (short) -26401);
            assert(pack.sue_barom_alt_GET() == -1254985418);
            assert(pack.sue_barom_temp_GET() == (short) -7121);
            assert(pack.sue_time_GET() == 4199833710L);
            assert(pack.sue_imu_velocity_z_GET() == (short) -12486);
            assert(pack.sue_imu_location_y_GET() == (short) -17363);
            assert(pack.sue_desired_height_GET() == (short)26408);
            assert(pack.sue_pwm_output_6_GET() == (short)12582);
            assert(pack.sue_waypoint_goal_x_GET() == (short)16535);
            assert(pack.sue_memory_stack_free_GET() == (short)4435);
            assert(pack.sue_pwm_input_1_GET() == (short)20160);
            assert(pack.sue_aero_x_GET() == (short)27715);
            assert(pack.sue_waypoint_goal_y_GET() == (short)2742);
            assert(pack.sue_waypoint_goal_z_GET() == (short) -5853);
            assert(pack.sue_pwm_input_12_GET() == (short) -16979);
            assert(pack.sue_pwm_output_12_GET() == (short) -31854);
            assert(pack.sue_pwm_input_10_GET() == (short)9064);
            assert(pack.sue_flags_GET() == 2931247894L);
            assert(pack.sue_pwm_output_10_GET() == (short) -24907);
            assert(pack.sue_pwm_output_11_GET() == (short)15059);
            assert(pack.sue_barom_press_GET() == -1492737473);
            assert(pack.sue_location_error_earth_y_GET() == (short) -22632);
        });
        GroundControl.SERIAL_UDB_EXTRA_F2_B p171 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_B();
        PH.setPack(p171);
        p171.sue_pwm_output_7_SET((short)4741) ;
        p171.sue_osc_fails_SET((short) -5252) ;
        p171.sue_location_error_earth_y_SET((short) -22632) ;
        p171.sue_pwm_output_5_SET((short) -32249) ;
        p171.sue_waypoint_goal_z_SET((short) -5853) ;
        p171.sue_pwm_output_10_SET((short) -24907) ;
        p171.sue_location_error_earth_x_SET((short)28611) ;
        p171.sue_time_SET(4199833710L) ;
        p171.sue_bat_amp_SET((short)9418) ;
        p171.sue_pwm_output_9_SET((short) -29324) ;
        p171.sue_location_error_earth_z_SET((short)13462) ;
        p171.sue_barom_press_SET(-1492737473) ;
        p171.sue_bat_volt_SET((short)21145) ;
        p171.sue_imu_velocity_z_SET((short) -12486) ;
        p171.sue_memory_stack_free_SET((short)4435) ;
        p171.sue_bat_amp_hours_SET((short) -18649) ;
        p171.sue_pwm_input_2_SET((short) -28832) ;
        p171.sue_pwm_output_12_SET((short) -31854) ;
        p171.sue_pwm_input_6_SET((short)8517) ;
        p171.sue_imu_velocity_x_SET((short)16236) ;
        p171.sue_pwm_output_8_SET((short) -31151) ;
        p171.sue_pwm_output_6_SET((short)12582) ;
        p171.sue_pwm_input_7_SET((short) -32359) ;
        p171.sue_aero_x_SET((short)27715) ;
        p171.sue_pwm_output_1_SET((short) -17992) ;
        p171.sue_desired_height_SET((short)26408) ;
        p171.sue_pwm_output_11_SET((short)15059) ;
        p171.sue_barom_temp_SET((short) -7121) ;
        p171.sue_pwm_input_5_SET((short)18814) ;
        p171.sue_flags_SET(2931247894L) ;
        p171.sue_pwm_input_1_SET((short)20160) ;
        p171.sue_imu_location_x_SET((short) -22582) ;
        p171.sue_pwm_input_9_SET((short) -3683) ;
        p171.sue_imu_location_z_SET((short) -30560) ;
        p171.sue_pwm_input_12_SET((short) -16979) ;
        p171.sue_pwm_input_8_SET((short)24319) ;
        p171.sue_barom_alt_SET(-1254985418) ;
        p171.sue_imu_location_y_SET((short) -17363) ;
        p171.sue_waypoint_goal_x_SET((short)16535) ;
        p171.sue_pwm_input_3_SET((short)12142) ;
        p171.sue_aero_z_SET((short) -10460) ;
        p171.sue_pwm_input_4_SET((short)17587) ;
        p171.sue_pwm_output_3_SET((short)5560) ;
        p171.sue_imu_velocity_y_SET((short) -22606) ;
        p171.sue_pwm_output_4_SET((short) -26401) ;
        p171.sue_pwm_output_2_SET((short) -22701) ;
        p171.sue_waypoint_goal_y_SET((short)2742) ;
        p171.sue_aero_y_SET((short)31948) ;
        p171.sue_pwm_input_10_SET((short)9064) ;
        p171.sue_pwm_input_11_SET((short) -17065) ;
        CommunicationChannel.instance.send(p171);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F4.add((src, ph, pack) ->
        {
            assert(pack.sue_PITCH_STABILIZATION_GET() == (char)239);
            assert(pack.sue_ROLL_STABILIZATION_AILERONS_GET() == (char)5);
            assert(pack.sue_ALTITUDEHOLD_WAYPOINT_GET() == (char)62);
            assert(pack.sue_YAW_STABILIZATION_AILERON_GET() == (char)243);
            assert(pack.sue_ALTITUDEHOLD_STABILIZED_GET() == (char)6);
            assert(pack.sue_YAW_STABILIZATION_RUDDER_GET() == (char)250);
            assert(pack.sue_RUDDER_NAVIGATION_GET() == (char)189);
            assert(pack.sue_RACING_MODE_GET() == (char)234);
            assert(pack.sue_AILERON_NAVIGATION_GET() == (char)63);
            assert(pack.sue_ROLL_STABILIZATION_RUDDER_GET() == (char)80);
        });
        GroundControl.SERIAL_UDB_EXTRA_F4 p172 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F4();
        PH.setPack(p172);
        p172.sue_ROLL_STABILIZATION_RUDDER_SET((char)80) ;
        p172.sue_AILERON_NAVIGATION_SET((char)63) ;
        p172.sue_ALTITUDEHOLD_WAYPOINT_SET((char)62) ;
        p172.sue_YAW_STABILIZATION_RUDDER_SET((char)250) ;
        p172.sue_ROLL_STABILIZATION_AILERONS_SET((char)5) ;
        p172.sue_ALTITUDEHOLD_STABILIZED_SET((char)6) ;
        p172.sue_RUDDER_NAVIGATION_SET((char)189) ;
        p172.sue_PITCH_STABILIZATION_SET((char)239) ;
        p172.sue_YAW_STABILIZATION_AILERON_SET((char)243) ;
        p172.sue_RACING_MODE_SET((char)234) ;
        CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F5.add((src, ph, pack) ->
        {
            assert(pack.sue_YAWKP_AILERON_GET() == -2.3896374E38F);
            assert(pack.sue_ROLLKD_GET() == -1.2274867E38F);
            assert(pack.sue_YAWKD_AILERON_GET() == 2.516374E38F);
            assert(pack.sue_ROLLKP_GET() == -1.6509139E38F);
        });
        GroundControl.SERIAL_UDB_EXTRA_F5 p173 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F5();
        PH.setPack(p173);
        p173.sue_YAWKD_AILERON_SET(2.516374E38F) ;
        p173.sue_ROLLKP_SET(-1.6509139E38F) ;
        p173.sue_YAWKP_AILERON_SET(-2.3896374E38F) ;
        p173.sue_ROLLKD_SET(-1.2274867E38F) ;
        CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F6.add((src, ph, pack) ->
        {
            assert(pack.sue_PITCHKD_GET() == 2.3755062E38F);
            assert(pack.sue_ELEVATOR_BOOST_GET() == -3.0884611E38F);
            assert(pack.sue_RUDDER_ELEV_MIX_GET() == -2.7009831E38F);
            assert(pack.sue_PITCHGAIN_GET() == 1.7491351E38F);
            assert(pack.sue_ROLL_ELEV_MIX_GET() == -4.6532786E37F);
        });
        GroundControl.SERIAL_UDB_EXTRA_F6 p174 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F6();
        PH.setPack(p174);
        p174.sue_ELEVATOR_BOOST_SET(-3.0884611E38F) ;
        p174.sue_PITCHGAIN_SET(1.7491351E38F) ;
        p174.sue_RUDDER_ELEV_MIX_SET(-2.7009831E38F) ;
        p174.sue_ROLL_ELEV_MIX_SET(-4.6532786E37F) ;
        p174.sue_PITCHKD_SET(2.3755062E38F) ;
        CommunicationChannel.instance.send(p174);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F7.add((src, ph, pack) ->
        {
            assert(pack.sue_YAWKP_RUDDER_GET() == 2.874864E38F);
            assert(pack.sue_RTL_PITCH_DOWN_GET() == -3.0375576E38F);
            assert(pack.sue_RUDDER_BOOST_GET() == -6.336518E37F);
            assert(pack.sue_ROLLKD_RUDDER_GET() == -3.9842198E37F);
            assert(pack.sue_ROLLKP_RUDDER_GET() == -1.7608663E38F);
            assert(pack.sue_YAWKD_RUDDER_GET() == -2.121938E38F);
        });
        GroundControl.SERIAL_UDB_EXTRA_F7 p175 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F7();
        PH.setPack(p175);
        p175.sue_YAWKD_RUDDER_SET(-2.121938E38F) ;
        p175.sue_RUDDER_BOOST_SET(-6.336518E37F) ;
        p175.sue_ROLLKP_RUDDER_SET(-1.7608663E38F) ;
        p175.sue_YAWKP_RUDDER_SET(2.874864E38F) ;
        p175.sue_ROLLKD_RUDDER_SET(-3.9842198E37F) ;
        p175.sue_RTL_PITCH_DOWN_SET(-3.0375576E38F) ;
        CommunicationChannel.instance.send(p175);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F8.add((src, ph, pack) ->
        {
            assert(pack.sue_ALT_HOLD_THROTTLE_MAX_GET() == 2.7023579E38F);
            assert(pack.sue_HEIGHT_TARGET_MAX_GET() == 3.0434431E38F);
            assert(pack.sue_ALT_HOLD_PITCH_HIGH_GET() == -2.4583033E38F);
            assert(pack.sue_ALT_HOLD_PITCH_MIN_GET() == 2.8382409E38F);
            assert(pack.sue_ALT_HOLD_PITCH_MAX_GET() == -9.638973E36F);
            assert(pack.sue_HEIGHT_TARGET_MIN_GET() == 1.7128215E38F);
            assert(pack.sue_ALT_HOLD_THROTTLE_MIN_GET() == 3.2628018E38F);
        });
        GroundControl.SERIAL_UDB_EXTRA_F8 p176 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F8();
        PH.setPack(p176);
        p176.sue_ALT_HOLD_THROTTLE_MIN_SET(3.2628018E38F) ;
        p176.sue_HEIGHT_TARGET_MAX_SET(3.0434431E38F) ;
        p176.sue_ALT_HOLD_PITCH_MIN_SET(2.8382409E38F) ;
        p176.sue_ALT_HOLD_PITCH_MAX_SET(-9.638973E36F) ;
        p176.sue_HEIGHT_TARGET_MIN_SET(1.7128215E38F) ;
        p176.sue_ALT_HOLD_THROTTLE_MAX_SET(2.7023579E38F) ;
        p176.sue_ALT_HOLD_PITCH_HIGH_SET(-2.4583033E38F) ;
        CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F13.add((src, ph, pack) ->
        {
            assert(pack.sue_lat_origin_GET() == -1090768086);
            assert(pack.sue_alt_origin_GET() == -288801143);
            assert(pack.sue_week_no_GET() == (short)18520);
            assert(pack.sue_lon_origin_GET() == 918965983);
        });
        GroundControl.SERIAL_UDB_EXTRA_F13 p177 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F13();
        PH.setPack(p177);
        p177.sue_alt_origin_SET(-288801143) ;
        p177.sue_week_no_SET((short)18520) ;
        p177.sue_lon_origin_SET(918965983) ;
        p177.sue_lat_origin_SET(-1090768086) ;
        CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F14.add((src, ph, pack) ->
        {
            assert(pack.sue_DR_GET() == (char)106);
            assert(pack.sue_AIRFRAME_GET() == (char)27);
            assert(pack.sue_osc_fail_count_GET() == (short) -24515);
            assert(pack.sue_BOARD_TYPE_GET() == (char)59);
            assert(pack.sue_FLIGHT_PLAN_TYPE_GET() == (char)11);
            assert(pack.sue_CLOCK_CONFIG_GET() == (char)190);
            assert(pack.sue_GPS_TYPE_GET() == (char)193);
            assert(pack.sue_TRAP_FLAGS_GET() == (short) -8833);
            assert(pack.sue_RCON_GET() == (short)27727);
            assert(pack.sue_TRAP_SOURCE_GET() == 2747569304L);
            assert(pack.sue_WIND_ESTIMATION_GET() == (char)191);
        });
        GroundControl.SERIAL_UDB_EXTRA_F14 p178 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F14();
        PH.setPack(p178);
        p178.sue_CLOCK_CONFIG_SET((char)190) ;
        p178.sue_GPS_TYPE_SET((char)193) ;
        p178.sue_TRAP_SOURCE_SET(2747569304L) ;
        p178.sue_WIND_ESTIMATION_SET((char)191) ;
        p178.sue_osc_fail_count_SET((short) -24515) ;
        p178.sue_BOARD_TYPE_SET((char)59) ;
        p178.sue_RCON_SET((short)27727) ;
        p178.sue_AIRFRAME_SET((char)27) ;
        p178.sue_TRAP_FLAGS_SET((short) -8833) ;
        p178.sue_FLIGHT_PLAN_TYPE_SET((char)11) ;
        p178.sue_DR_SET((char)106) ;
        CommunicationChannel.instance.send(p178);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F15.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.sue_ID_VEHICLE_MODEL_NAME_GET(),  new char[] {(char)242, (char)220, (char)6, (char)200, (char)225, (char)20, (char)68, (char)110, (char)61, (char)56, (char)172, (char)247, (char)118, (char)168, (char)52, (char)191, (char)114, (char)68, (char)44, (char)143, (char)64, (char)82, (char)126, (char)16, (char)229, (char)205, (char)137, (char)129, (char)104, (char)45, (char)237, (char)109, (char)207, (char)169, (char)7, (char)41, (char)45, (char)177, (char)133, (char)154}));
            assert(Arrays.equals(pack.sue_ID_VEHICLE_REGISTRATION_GET(),  new char[] {(char)246, (char)143, (char)97, (char)224, (char)12, (char)183, (char)205, (char)59, (char)200, (char)239, (char)212, (char)111, (char)31, (char)80, (char)7, (char)14, (char)254, (char)217, (char)217, (char)162}));
        });
        GroundControl.SERIAL_UDB_EXTRA_F15 p179 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F15();
        PH.setPack(p179);
        p179.sue_ID_VEHICLE_REGISTRATION_SET(new char[] {(char)246, (char)143, (char)97, (char)224, (char)12, (char)183, (char)205, (char)59, (char)200, (char)239, (char)212, (char)111, (char)31, (char)80, (char)7, (char)14, (char)254, (char)217, (char)217, (char)162}, 0) ;
        p179.sue_ID_VEHICLE_MODEL_NAME_SET(new char[] {(char)242, (char)220, (char)6, (char)200, (char)225, (char)20, (char)68, (char)110, (char)61, (char)56, (char)172, (char)247, (char)118, (char)168, (char)52, (char)191, (char)114, (char)68, (char)44, (char)143, (char)64, (char)82, (char)126, (char)16, (char)229, (char)205, (char)137, (char)129, (char)104, (char)45, (char)237, (char)109, (char)207, (char)169, (char)7, (char)41, (char)45, (char)177, (char)133, (char)154}, 0) ;
        CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F16.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.sue_ID_DIY_DRONES_URL_GET(),  new char[] {(char)85, (char)215, (char)39, (char)19, (char)182, (char)46, (char)68, (char)221, (char)51, (char)194, (char)171, (char)85, (char)109, (char)22, (char)18, (char)140, (char)135, (char)82, (char)52, (char)174, (char)241, (char)151, (char)143, (char)22, (char)112, (char)143, (char)96, (char)128, (char)165, (char)207, (char)181, (char)18, (char)146, (char)69, (char)211, (char)179, (char)64, (char)73, (char)188, (char)194, (char)149, (char)211, (char)10, (char)48, (char)66, (char)207, (char)218, (char)17, (char)161, (char)70, (char)151, (char)196, (char)73, (char)16, (char)108, (char)57, (char)34, (char)66, (char)35, (char)238, (char)103, (char)240, (char)184, (char)143, (char)3, (char)206, (char)47, (char)94, (char)74, (char)243}));
            assert(Arrays.equals(pack.sue_ID_LEAD_PILOT_GET(),  new char[] {(char)58, (char)139, (char)235, (char)249, (char)63, (char)137, (char)13, (char)78, (char)94, (char)156, (char)155, (char)107, (char)14, (char)37, (char)8, (char)61, (char)98, (char)191, (char)164, (char)186, (char)169, (char)119, (char)55, (char)150, (char)151, (char)251, (char)96, (char)162, (char)132, (char)48, (char)59, (char)196, (char)230, (char)218, (char)179, (char)22, (char)11, (char)43, (char)90, (char)176}));
        });
        GroundControl.SERIAL_UDB_EXTRA_F16 p180 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F16();
        PH.setPack(p180);
        p180.sue_ID_LEAD_PILOT_SET(new char[] {(char)58, (char)139, (char)235, (char)249, (char)63, (char)137, (char)13, (char)78, (char)94, (char)156, (char)155, (char)107, (char)14, (char)37, (char)8, (char)61, (char)98, (char)191, (char)164, (char)186, (char)169, (char)119, (char)55, (char)150, (char)151, (char)251, (char)96, (char)162, (char)132, (char)48, (char)59, (char)196, (char)230, (char)218, (char)179, (char)22, (char)11, (char)43, (char)90, (char)176}, 0) ;
        p180.sue_ID_DIY_DRONES_URL_SET(new char[] {(char)85, (char)215, (char)39, (char)19, (char)182, (char)46, (char)68, (char)221, (char)51, (char)194, (char)171, (char)85, (char)109, (char)22, (char)18, (char)140, (char)135, (char)82, (char)52, (char)174, (char)241, (char)151, (char)143, (char)22, (char)112, (char)143, (char)96, (char)128, (char)165, (char)207, (char)181, (char)18, (char)146, (char)69, (char)211, (char)179, (char)64, (char)73, (char)188, (char)194, (char)149, (char)211, (char)10, (char)48, (char)66, (char)207, (char)218, (char)17, (char)161, (char)70, (char)151, (char)196, (char)73, (char)16, (char)108, (char)57, (char)34, (char)66, (char)35, (char)238, (char)103, (char)240, (char)184, (char)143, (char)3, (char)206, (char)47, (char)94, (char)74, (char)243}, 0) ;
        CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ALTITUDES.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2002992863L);
            assert(pack.alt_imu_GET() == 818138333);
            assert(pack.alt_gps_GET() == -241150645);
            assert(pack.alt_barometric_GET() == -1157497250);
            assert(pack.alt_range_finder_GET() == 277695710);
            assert(pack.alt_optical_flow_GET() == -1515304194);
            assert(pack.alt_extra_GET() == -934198061);
        });
        GroundControl.ALTITUDES p181 = CommunicationChannel.new_ALTITUDES();
        PH.setPack(p181);
        p181.time_boot_ms_SET(2002992863L) ;
        p181.alt_gps_SET(-241150645) ;
        p181.alt_extra_SET(-934198061) ;
        p181.alt_barometric_SET(-1157497250) ;
        p181.alt_range_finder_SET(277695710) ;
        p181.alt_optical_flow_SET(-1515304194) ;
        p181.alt_imu_SET(818138333) ;
        CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_AIRSPEEDS.add((src, ph, pack) ->
        {
            assert(pack.airspeed_hot_wire_GET() == (short) -10710);
            assert(pack.airspeed_pitot_GET() == (short)15979);
            assert(pack.time_boot_ms_GET() == 632625683L);
            assert(pack.airspeed_imu_GET() == (short)3229);
            assert(pack.airspeed_ultrasonic_GET() == (short)19605);
            assert(pack.aoa_GET() == (short) -2854);
            assert(pack.aoy_GET() == (short) -19560);
        });
        GroundControl.AIRSPEEDS p182 = CommunicationChannel.new_AIRSPEEDS();
        PH.setPack(p182);
        p182.airspeed_imu_SET((short)3229) ;
        p182.airspeed_pitot_SET((short)15979) ;
        p182.airspeed_ultrasonic_SET((short)19605) ;
        p182.aoa_SET((short) -2854) ;
        p182.time_boot_ms_SET(632625683L) ;
        p182.airspeed_hot_wire_SET((short) -10710) ;
        p182.aoy_SET((short) -19560) ;
        CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F17.add((src, ph, pack) ->
        {
            assert(pack.sue_turn_rate_fbw_GET() == 1.5758404E38F);
            assert(pack.sue_feed_forward_GET() == 2.1764983E38F);
            assert(pack.sue_turn_rate_nav_GET() == 2.300857E38F);
        });
        GroundControl.SERIAL_UDB_EXTRA_F17 p183 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F17();
        PH.setPack(p183);
        p183.sue_feed_forward_SET(2.1764983E38F) ;
        p183.sue_turn_rate_fbw_SET(1.5758404E38F) ;
        p183.sue_turn_rate_nav_SET(2.300857E38F) ;
        CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F18.add((src, ph, pack) ->
        {
            assert(pack.angle_of_attack_normal_GET() == -3.165596E38F);
            assert(pack.elevator_trim_inverted_GET() == 1.6083277E38F);
            assert(pack.angle_of_attack_inverted_GET() == 1.9242626E38F);
            assert(pack.elevator_trim_normal_GET() == -1.5066669E38F);
            assert(pack.reference_speed_GET() == 2.9489947E37F);
        });
        GroundControl.SERIAL_UDB_EXTRA_F18 p184 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F18();
        PH.setPack(p184);
        p184.angle_of_attack_normal_SET(-3.165596E38F) ;
        p184.reference_speed_SET(2.9489947E37F) ;
        p184.elevator_trim_inverted_SET(1.6083277E38F) ;
        p184.elevator_trim_normal_SET(-1.5066669E38F) ;
        p184.angle_of_attack_inverted_SET(1.9242626E38F) ;
        CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F19.add((src, ph, pack) ->
        {
            assert(pack.sue_throttle_output_channel_GET() == (char)2);
            assert(pack.sue_rudder_output_channel_GET() == (char)170);
            assert(pack.sue_elevator_output_channel_GET() == (char)198);
            assert(pack.sue_elevator_reversed_GET() == (char)34);
            assert(pack.sue_aileron_output_channel_GET() == (char)136);
            assert(pack.sue_rudder_reversed_GET() == (char)217);
            assert(pack.sue_aileron_reversed_GET() == (char)145);
            assert(pack.sue_throttle_reversed_GET() == (char)215);
        });
        GroundControl.SERIAL_UDB_EXTRA_F19 p185 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F19();
        PH.setPack(p185);
        p185.sue_rudder_reversed_SET((char)217) ;
        p185.sue_elevator_reversed_SET((char)34) ;
        p185.sue_aileron_output_channel_SET((char)136) ;
        p185.sue_throttle_output_channel_SET((char)2) ;
        p185.sue_aileron_reversed_SET((char)145) ;
        p185.sue_throttle_reversed_SET((char)215) ;
        p185.sue_elevator_output_channel_SET((char)198) ;
        p185.sue_rudder_output_channel_SET((char)170) ;
        CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F20.add((src, ph, pack) ->
        {
            assert(pack.sue_number_of_inputs_GET() == (char)174);
            assert(pack.sue_trim_value_input_10_GET() == (short)14261);
            assert(pack.sue_trim_value_input_3_GET() == (short) -28727);
            assert(pack.sue_trim_value_input_2_GET() == (short) -11379);
            assert(pack.sue_trim_value_input_6_GET() == (short) -22727);
            assert(pack.sue_trim_value_input_9_GET() == (short)22276);
            assert(pack.sue_trim_value_input_1_GET() == (short)9527);
            assert(pack.sue_trim_value_input_8_GET() == (short) -4293);
            assert(pack.sue_trim_value_input_11_GET() == (short) -28629);
            assert(pack.sue_trim_value_input_4_GET() == (short) -20485);
            assert(pack.sue_trim_value_input_12_GET() == (short)32031);
            assert(pack.sue_trim_value_input_7_GET() == (short)23861);
            assert(pack.sue_trim_value_input_5_GET() == (short) -19314);
        });
        GroundControl.SERIAL_UDB_EXTRA_F20 p186 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F20();
        PH.setPack(p186);
        p186.sue_number_of_inputs_SET((char)174) ;
        p186.sue_trim_value_input_11_SET((short) -28629) ;
        p186.sue_trim_value_input_2_SET((short) -11379) ;
        p186.sue_trim_value_input_10_SET((short)14261) ;
        p186.sue_trim_value_input_4_SET((short) -20485) ;
        p186.sue_trim_value_input_7_SET((short)23861) ;
        p186.sue_trim_value_input_9_SET((short)22276) ;
        p186.sue_trim_value_input_5_SET((short) -19314) ;
        p186.sue_trim_value_input_3_SET((short) -28727) ;
        p186.sue_trim_value_input_8_SET((short) -4293) ;
        p186.sue_trim_value_input_12_SET((short)32031) ;
        p186.sue_trim_value_input_1_SET((short)9527) ;
        p186.sue_trim_value_input_6_SET((short) -22727) ;
        CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F21.add((src, ph, pack) ->
        {
            assert(pack.sue_gyro_x_offset_GET() == (short) -8555);
            assert(pack.sue_gyro_z_offset_GET() == (short)23527);
            assert(pack.sue_gyro_y_offset_GET() == (short)19193);
            assert(pack.sue_accel_x_offset_GET() == (short) -2232);
            assert(pack.sue_accel_y_offset_GET() == (short)32071);
            assert(pack.sue_accel_z_offset_GET() == (short) -28625);
        });
        GroundControl.SERIAL_UDB_EXTRA_F21 p187 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F21();
        PH.setPack(p187);
        p187.sue_accel_x_offset_SET((short) -2232) ;
        p187.sue_gyro_y_offset_SET((short)19193) ;
        p187.sue_accel_y_offset_SET((short)32071) ;
        p187.sue_gyro_x_offset_SET((short) -8555) ;
        p187.sue_gyro_z_offset_SET((short)23527) ;
        p187.sue_accel_z_offset_SET((short) -28625) ;
        CommunicationChannel.instance.send(p187);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F22.add((src, ph, pack) ->
        {
            assert(pack.sue_gyro_y_at_calibration_GET() == (short) -1009);
            assert(pack.sue_accel_z_at_calibration_GET() == (short) -4530);
            assert(pack.sue_gyro_z_at_calibration_GET() == (short) -7005);
            assert(pack.sue_accel_x_at_calibration_GET() == (short)27091);
            assert(pack.sue_gyro_x_at_calibration_GET() == (short)2973);
            assert(pack.sue_accel_y_at_calibration_GET() == (short) -20251);
        });
        GroundControl.SERIAL_UDB_EXTRA_F22 p188 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F22();
        PH.setPack(p188);
        p188.sue_gyro_x_at_calibration_SET((short)2973) ;
        p188.sue_gyro_z_at_calibration_SET((short) -7005) ;
        p188.sue_gyro_y_at_calibration_SET((short) -1009) ;
        p188.sue_accel_y_at_calibration_SET((short) -20251) ;
        p188.sue_accel_z_at_calibration_SET((short) -4530) ;
        p188.sue_accel_x_at_calibration_SET((short)27091) ;
        CommunicationChannel.instance.send(p188);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        CommunicationChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 1034866105323239743L);
            assert(pack.vel_ratio_GET() == 7.5594224E37F);
            assert(pack.mag_ratio_GET() == -1.854884E38F);
            assert(pack.pos_horiz_ratio_GET() == -2.331368E38F);
            assert(pack.pos_vert_ratio_GET() == 2.8942276E38F);
            assert(pack.pos_vert_accuracy_GET() == -2.6373292E38F);
            assert(pack.pos_horiz_accuracy_GET() == -1.7538863E38F);
            assert(pack.hagl_ratio_GET() == -1.5688677E38F);
            assert(pack.tas_ratio_GET() == 2.4508501E38F);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL));
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.time_usec_SET(1034866105323239743L) ;
        p230.mag_ratio_SET(-1.854884E38F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL)) ;
        p230.hagl_ratio_SET(-1.5688677E38F) ;
        p230.pos_vert_accuracy_SET(-2.6373292E38F) ;
        p230.pos_vert_ratio_SET(2.8942276E38F) ;
        p230.pos_horiz_ratio_SET(-2.331368E38F) ;
        p230.pos_horiz_accuracy_SET(-1.7538863E38F) ;
        p230.vel_ratio_SET(7.5594224E37F) ;
        p230.tas_ratio_SET(2.4508501E38F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_z_GET() == 4.2220597E37F);
            assert(pack.wind_alt_GET() == -3.063055E38F);
            assert(pack.var_horiz_GET() == -2.7304484E38F);
            assert(pack.wind_y_GET() == -2.4131954E38F);
            assert(pack.time_usec_GET() == 8084946085347959875L);
            assert(pack.wind_x_GET() == -3.1978561E38F);
            assert(pack.vert_accuracy_GET() == -3.2032145E38F);
            assert(pack.var_vert_GET() == -3.107365E38F);
            assert(pack.horiz_accuracy_GET() == 1.4378392E38F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_x_SET(-3.1978561E38F) ;
        p231.var_horiz_SET(-2.7304484E38F) ;
        p231.wind_z_SET(4.2220597E37F) ;
        p231.vert_accuracy_SET(-3.2032145E38F) ;
        p231.horiz_accuracy_SET(1.4378392E38F) ;
        p231.time_usec_SET(8084946085347959875L) ;
        p231.var_vert_SET(-3.107365E38F) ;
        p231.wind_alt_SET(-3.063055E38F) ;
        p231.wind_y_SET(-2.4131954E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.satellites_visible_GET() == (char)114);
            assert(pack.time_week_GET() == (char)24393);
            assert(pack.alt_GET() == 1.9591262E38F);
            assert(pack.vd_GET() == -2.3274584E38F);
            assert(pack.time_usec_GET() == 3416054188054113318L);
            assert(pack.lat_GET() == 282842805);
            assert(pack.hdop_GET() == 3.2685685E38F);
            assert(pack.gps_id_GET() == (char)95);
            assert(pack.fix_type_GET() == (char)152);
            assert(pack.vdop_GET() == 3.207424E38F);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP));
            assert(pack.horiz_accuracy_GET() == -1.2743096E37F);
            assert(pack.speed_accuracy_GET() == -1.4863032E38F);
            assert(pack.ve_GET() == 8.750394E37F);
            assert(pack.time_week_ms_GET() == 872362195L);
            assert(pack.vn_GET() == 1.3591139E38F);
            assert(pack.vert_accuracy_GET() == 2.242553E36F);
            assert(pack.lon_GET() == 1304727288);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.hdop_SET(3.2685685E38F) ;
        p232.fix_type_SET((char)152) ;
        p232.vert_accuracy_SET(2.242553E36F) ;
        p232.speed_accuracy_SET(-1.4863032E38F) ;
        p232.time_usec_SET(3416054188054113318L) ;
        p232.time_week_SET((char)24393) ;
        p232.horiz_accuracy_SET(-1.2743096E37F) ;
        p232.time_week_ms_SET(872362195L) ;
        p232.vd_SET(-2.3274584E38F) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP)) ;
        p232.vn_SET(1.3591139E38F) ;
        p232.satellites_visible_SET((char)114) ;
        p232.alt_SET(1.9591262E38F) ;
        p232.gps_id_SET((char)95) ;
        p232.vdop_SET(3.207424E38F) ;
        p232.lon_SET(1304727288) ;
        p232.lat_SET(282842805) ;
        p232.ve_SET(8.750394E37F) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)44, (char)149, (char)155, (char)24, (char)238, (char)177, (char)127, (char)120, (char)146, (char)84, (char)35, (char)168, (char)136, (char)122, (char)6, (char)191, (char)9, (char)242, (char)77, (char)239, (char)17, (char)200, (char)97, (char)99, (char)148, (char)22, (char)90, (char)95, (char)220, (char)143, (char)101, (char)141, (char)100, (char)150, (char)29, (char)54, (char)89, (char)253, (char)210, (char)10, (char)69, (char)20, (char)61, (char)95, (char)158, (char)62, (char)185, (char)195, (char)109, (char)31, (char)17, (char)244, (char)153, (char)223, (char)45, (char)155, (char)94, (char)125, (char)147, (char)130, (char)56, (char)120, (char)186, (char)226, (char)50, (char)236, (char)95, (char)151, (char)154, (char)145, (char)166, (char)245, (char)203, (char)67, (char)99, (char)55, (char)197, (char)118, (char)89, (char)120, (char)44, (char)158, (char)237, (char)80, (char)162, (char)15, (char)93, (char)57, (char)122, (char)198, (char)88, (char)20, (char)201, (char)56, (char)214, (char)94, (char)147, (char)9, (char)74, (char)118, (char)18, (char)22, (char)127, (char)14, (char)233, (char)91, (char)126, (char)187, (char)200, (char)229, (char)223, (char)36, (char)33, (char)241, (char)52, (char)245, (char)46, (char)23, (char)211, (char)41, (char)99, (char)161, (char)136, (char)220, (char)168, (char)87, (char)176, (char)157, (char)34, (char)145, (char)24, (char)39, (char)149, (char)166, (char)24, (char)37, (char)28, (char)101, (char)18, (char)228, (char)188, (char)98, (char)226, (char)66, (char)203, (char)199, (char)104, (char)171, (char)54, (char)177, (char)80, (char)232, (char)240, (char)150, (char)210, (char)28, (char)16, (char)164, (char)114, (char)225, (char)176, (char)66, (char)169, (char)211, (char)48, (char)217, (char)242, (char)122, (char)184, (char)84, (char)111, (char)232, (char)206, (char)170, (char)149, (char)209, (char)188, (char)163, (char)87, (char)175}));
            assert(pack.flags_GET() == (char)106);
            assert(pack.len_GET() == (char)154);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)106) ;
        p233.len_SET((char)154) ;
        p233.data__SET(new char[] {(char)44, (char)149, (char)155, (char)24, (char)238, (char)177, (char)127, (char)120, (char)146, (char)84, (char)35, (char)168, (char)136, (char)122, (char)6, (char)191, (char)9, (char)242, (char)77, (char)239, (char)17, (char)200, (char)97, (char)99, (char)148, (char)22, (char)90, (char)95, (char)220, (char)143, (char)101, (char)141, (char)100, (char)150, (char)29, (char)54, (char)89, (char)253, (char)210, (char)10, (char)69, (char)20, (char)61, (char)95, (char)158, (char)62, (char)185, (char)195, (char)109, (char)31, (char)17, (char)244, (char)153, (char)223, (char)45, (char)155, (char)94, (char)125, (char)147, (char)130, (char)56, (char)120, (char)186, (char)226, (char)50, (char)236, (char)95, (char)151, (char)154, (char)145, (char)166, (char)245, (char)203, (char)67, (char)99, (char)55, (char)197, (char)118, (char)89, (char)120, (char)44, (char)158, (char)237, (char)80, (char)162, (char)15, (char)93, (char)57, (char)122, (char)198, (char)88, (char)20, (char)201, (char)56, (char)214, (char)94, (char)147, (char)9, (char)74, (char)118, (char)18, (char)22, (char)127, (char)14, (char)233, (char)91, (char)126, (char)187, (char)200, (char)229, (char)223, (char)36, (char)33, (char)241, (char)52, (char)245, (char)46, (char)23, (char)211, (char)41, (char)99, (char)161, (char)136, (char)220, (char)168, (char)87, (char)176, (char)157, (char)34, (char)145, (char)24, (char)39, (char)149, (char)166, (char)24, (char)37, (char)28, (char)101, (char)18, (char)228, (char)188, (char)98, (char)226, (char)66, (char)203, (char)199, (char)104, (char)171, (char)54, (char)177, (char)80, (char)232, (char)240, (char)150, (char)210, (char)28, (char)16, (char)164, (char)114, (char)225, (char)176, (char)66, (char)169, (char)211, (char)48, (char)217, (char)242, (char)122, (char)184, (char)84, (char)111, (char)232, (char)206, (char)170, (char)149, (char)209, (char)188, (char)163, (char)87, (char)175}, 0) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == (short) -1404);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
            assert(pack.custom_mode_GET() == 2726737652L);
            assert(pack.temperature_GET() == (byte)127);
            assert(pack.climb_rate_GET() == (byte)44);
            assert(pack.wp_num_GET() == (char)40);
            assert(pack.altitude_sp_GET() == (short)3702);
            assert(pack.altitude_amsl_GET() == (short) -18574);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED));
            assert(pack.airspeed_sp_GET() == (char)248);
            assert(pack.groundspeed_GET() == (char)94);
            assert(pack.latitude_GET() == -548634826);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
            assert(pack.heading_GET() == (char)47209);
            assert(pack.wp_distance_GET() == (char)1079);
            assert(pack.pitch_GET() == (short)17367);
            assert(pack.heading_sp_GET() == (short)1287);
            assert(pack.airspeed_GET() == (char)164);
            assert(pack.temperature_air_GET() == (byte)95);
            assert(pack.gps_nsat_GET() == (char)49);
            assert(pack.longitude_GET() == -1104498017);
            assert(pack.throttle_GET() == (byte)109);
            assert(pack.failsafe_GET() == (char)172);
            assert(pack.battery_remaining_GET() == (char)143);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.altitude_sp_SET((short)3702) ;
        p234.heading_sp_SET((short)1287) ;
        p234.wp_distance_SET((char)1079) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED)) ;
        p234.wp_num_SET((char)40) ;
        p234.heading_SET((char)47209) ;
        p234.airspeed_sp_SET((char)248) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX) ;
        p234.altitude_amsl_SET((short) -18574) ;
        p234.throttle_SET((byte)109) ;
        p234.longitude_SET(-1104498017) ;
        p234.roll_SET((short) -1404) ;
        p234.temperature_air_SET((byte)95) ;
        p234.custom_mode_SET(2726737652L) ;
        p234.pitch_SET((short)17367) ;
        p234.battery_remaining_SET((char)143) ;
        p234.latitude_SET(-548634826) ;
        p234.failsafe_SET((char)172) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND) ;
        p234.groundspeed_SET((char)94) ;
        p234.gps_nsat_SET((char)49) ;
        p234.airspeed_SET((char)164) ;
        p234.temperature_SET((byte)127) ;
        p234.climb_rate_SET((byte)44) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.vibration_y_GET() == 2.601185E38F);
            assert(pack.clipping_2_GET() == 53708140L);
            assert(pack.clipping_0_GET() == 3210131468L);
            assert(pack.clipping_1_GET() == 249370299L);
            assert(pack.vibration_z_GET() == 3.5831777E37F);
            assert(pack.vibration_x_GET() == -2.2778118E38F);
            assert(pack.time_usec_GET() == 7316906156766270777L);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.clipping_1_SET(249370299L) ;
        p241.clipping_0_SET(3210131468L) ;
        p241.time_usec_SET(7316906156766270777L) ;
        p241.clipping_2_SET(53708140L) ;
        p241.vibration_y_SET(2.601185E38F) ;
        p241.vibration_z_SET(3.5831777E37F) ;
        p241.vibration_x_SET(-2.2778118E38F) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == -639135733);
            assert(pack.y_GET() == -1.3999978E38F);
            assert(pack.approach_y_GET() == -9.158813E37F);
            assert(pack.altitude_GET() == 1176095765);
            assert(pack.time_usec_TRY(ph) == 629648422154109297L);
            assert(pack.x_GET() == -3.210119E38F);
            assert(pack.approach_z_GET() == -3.0728579E38F);
            assert(pack.longitude_GET() == -1555592502);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.1813598E38F, -1.3300531E38F, 3.0683205E38F, 2.552449E38F}));
            assert(pack.z_GET() == 9.167235E37F);
            assert(pack.approach_x_GET() == 1.0377747E38F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.x_SET(-3.210119E38F) ;
        p242.altitude_SET(1176095765) ;
        p242.z_SET(9.167235E37F) ;
        p242.latitude_SET(-639135733) ;
        p242.time_usec_SET(629648422154109297L, PH) ;
        p242.approach_x_SET(1.0377747E38F) ;
        p242.longitude_SET(-1555592502) ;
        p242.y_SET(-1.3999978E38F) ;
        p242.approach_y_SET(-9.158813E37F) ;
        p242.q_SET(new float[] {-1.1813598E38F, -1.3300531E38F, 3.0683205E38F, 2.552449E38F}, 0) ;
        p242.approach_z_SET(-3.0728579E38F) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.approach_x_GET() == 2.1802988E38F);
            assert(pack.target_system_GET() == (char)207);
            assert(pack.z_GET() == 2.9467496E38F);
            assert(pack.y_GET() == 1.0317732E37F);
            assert(pack.longitude_GET() == 172477965);
            assert(pack.time_usec_TRY(ph) == 3573778406793520597L);
            assert(pack.approach_y_GET() == -3.0263402E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {6.396414E37F, -2.1658618E38F, -1.1271836E37F, 1.5139694E38F}));
            assert(pack.latitude_GET() == 1902297018);
            assert(pack.altitude_GET() == -2027351784);
            assert(pack.x_GET() == 2.045048E38F);
            assert(pack.approach_z_GET() == -2.3584599E38F);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.altitude_SET(-2027351784) ;
        p243.longitude_SET(172477965) ;
        p243.target_system_SET((char)207) ;
        p243.x_SET(2.045048E38F) ;
        p243.approach_y_SET(-3.0263402E38F) ;
        p243.y_SET(1.0317732E37F) ;
        p243.time_usec_SET(3573778406793520597L, PH) ;
        p243.approach_x_SET(2.1802988E38F) ;
        p243.latitude_SET(1902297018) ;
        p243.q_SET(new float[] {6.396414E37F, -2.1658618E38F, -1.1271836E37F, 1.5139694E38F}, 0) ;
        p243.z_SET(2.9467496E38F) ;
        p243.approach_z_SET(-2.3584599E38F) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.interval_us_GET() == 1648190504);
            assert(pack.message_id_GET() == (char)56916);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)56916) ;
        p244.interval_us_SET(1648190504) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.heading_GET() == (char)30506);
            assert(pack.squawk_GET() == (char)31362);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SPACE);
            assert(pack.lon_GET() == 236900095);
            assert(pack.lat_GET() == 1874863472);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED));
            assert(pack.altitude_GET() == 1704370043);
            assert(pack.callsign_LEN(ph) == 3);
            assert(pack.callsign_TRY(ph).equals("Jnl"));
            assert(pack.hor_velocity_GET() == (char)34781);
            assert(pack.tslc_GET() == (char)168);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            assert(pack.ICAO_address_GET() == 648801674L);
            assert(pack.ver_velocity_GET() == (short)8729);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.lat_SET(1874863472) ;
        p246.lon_SET(236900095) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED)) ;
        p246.tslc_SET((char)168) ;
        p246.hor_velocity_SET((char)34781) ;
        p246.squawk_SET((char)31362) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SPACE) ;
        p246.ICAO_address_SET(648801674L) ;
        p246.heading_SET((char)30506) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH) ;
        p246.altitude_SET(1704370043) ;
        p246.ver_velocity_SET((short)8729) ;
        p246.callsign_SET("Jnl", PH) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER);
            assert(pack.id_GET() == 1651242815L);
            assert(pack.time_to_minimum_delta_GET() == -1.8494164E38F);
            assert(pack.altitude_minimum_delta_GET() == -3.3029799E38F);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
            assert(pack.horizontal_minimum_delta_GET() == -2.846287E38F);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE) ;
        p247.horizontal_minimum_delta_SET(-2.846287E38F) ;
        p247.time_to_minimum_delta_SET(-1.8494164E38F) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT) ;
        p247.id_SET(1651242815L) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER) ;
        p247.altitude_minimum_delta_SET(-3.3029799E38F) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)9);
            assert(pack.target_system_GET() == (char)236);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)90, (char)71, (char)97, (char)32, (char)86, (char)233, (char)15, (char)101, (char)241, (char)25, (char)192, (char)135, (char)19, (char)204, (char)49, (char)120, (char)243, (char)226, (char)97, (char)26, (char)113, (char)27, (char)25, (char)66, (char)110, (char)205, (char)235, (char)74, (char)223, (char)52, (char)217, (char)60, (char)213, (char)63, (char)28, (char)19, (char)147, (char)58, (char)234, (char)85, (char)192, (char)79, (char)253, (char)212, (char)130, (char)154, (char)126, (char)156, (char)247, (char)36, (char)30, (char)43, (char)255, (char)152, (char)87, (char)109, (char)174, (char)91, (char)241, (char)175, (char)82, (char)25, (char)138, (char)70, (char)46, (char)126, (char)196, (char)71, (char)23, (char)51, (char)179, (char)228, (char)49, (char)44, (char)26, (char)52, (char)253, (char)204, (char)134, (char)232, (char)168, (char)213, (char)183, (char)7, (char)54, (char)22, (char)83, (char)255, (char)82, (char)60, (char)82, (char)168, (char)214, (char)152, (char)136, (char)149, (char)36, (char)12, (char)71, (char)68, (char)95, (char)13, (char)104, (char)53, (char)178, (char)254, (char)48, (char)58, (char)145, (char)246, (char)98, (char)73, (char)233, (char)94, (char)4, (char)116, (char)175, (char)7, (char)75, (char)17, (char)245, (char)109, (char)129, (char)195, (char)69, (char)151, (char)216, (char)210, (char)120, (char)98, (char)10, (char)224, (char)90, (char)33, (char)148, (char)144, (char)218, (char)27, (char)25, (char)41, (char)253, (char)178, (char)38, (char)28, (char)199, (char)39, (char)228, (char)162, (char)244, (char)179, (char)79, (char)198, (char)90, (char)21, (char)66, (char)97, (char)92, (char)10, (char)154, (char)53, (char)10, (char)78, (char)103, (char)83, (char)205, (char)98, (char)143, (char)164, (char)82, (char)148, (char)59, (char)50, (char)101, (char)161, (char)146, (char)100, (char)248, (char)168, (char)250, (char)120, (char)245, (char)107, (char)187, (char)249, (char)122, (char)67, (char)118, (char)220, (char)19, (char)39, (char)222, (char)52, (char)100, (char)208, (char)159, (char)83, (char)206, (char)159, (char)206, (char)13, (char)128, (char)155, (char)212, (char)110, (char)75, (char)5, (char)132, (char)64, (char)220, (char)14, (char)170, (char)142, (char)178, (char)23, (char)58, (char)31, (char)192, (char)43, (char)215, (char)169, (char)255, (char)40, (char)215, (char)11, (char)198, (char)244, (char)0, (char)16, (char)194, (char)65, (char)218, (char)78, (char)104, (char)41, (char)237, (char)77, (char)178, (char)113, (char)72, (char)30, (char)30, (char)110, (char)160, (char)165, (char)141, (char)115, (char)218, (char)147, (char)224}));
            assert(pack.target_component_GET() == (char)232);
            assert(pack.message_type_GET() == (char)14304);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)9) ;
        p248.payload_SET(new char[] {(char)90, (char)71, (char)97, (char)32, (char)86, (char)233, (char)15, (char)101, (char)241, (char)25, (char)192, (char)135, (char)19, (char)204, (char)49, (char)120, (char)243, (char)226, (char)97, (char)26, (char)113, (char)27, (char)25, (char)66, (char)110, (char)205, (char)235, (char)74, (char)223, (char)52, (char)217, (char)60, (char)213, (char)63, (char)28, (char)19, (char)147, (char)58, (char)234, (char)85, (char)192, (char)79, (char)253, (char)212, (char)130, (char)154, (char)126, (char)156, (char)247, (char)36, (char)30, (char)43, (char)255, (char)152, (char)87, (char)109, (char)174, (char)91, (char)241, (char)175, (char)82, (char)25, (char)138, (char)70, (char)46, (char)126, (char)196, (char)71, (char)23, (char)51, (char)179, (char)228, (char)49, (char)44, (char)26, (char)52, (char)253, (char)204, (char)134, (char)232, (char)168, (char)213, (char)183, (char)7, (char)54, (char)22, (char)83, (char)255, (char)82, (char)60, (char)82, (char)168, (char)214, (char)152, (char)136, (char)149, (char)36, (char)12, (char)71, (char)68, (char)95, (char)13, (char)104, (char)53, (char)178, (char)254, (char)48, (char)58, (char)145, (char)246, (char)98, (char)73, (char)233, (char)94, (char)4, (char)116, (char)175, (char)7, (char)75, (char)17, (char)245, (char)109, (char)129, (char)195, (char)69, (char)151, (char)216, (char)210, (char)120, (char)98, (char)10, (char)224, (char)90, (char)33, (char)148, (char)144, (char)218, (char)27, (char)25, (char)41, (char)253, (char)178, (char)38, (char)28, (char)199, (char)39, (char)228, (char)162, (char)244, (char)179, (char)79, (char)198, (char)90, (char)21, (char)66, (char)97, (char)92, (char)10, (char)154, (char)53, (char)10, (char)78, (char)103, (char)83, (char)205, (char)98, (char)143, (char)164, (char)82, (char)148, (char)59, (char)50, (char)101, (char)161, (char)146, (char)100, (char)248, (char)168, (char)250, (char)120, (char)245, (char)107, (char)187, (char)249, (char)122, (char)67, (char)118, (char)220, (char)19, (char)39, (char)222, (char)52, (char)100, (char)208, (char)159, (char)83, (char)206, (char)159, (char)206, (char)13, (char)128, (char)155, (char)212, (char)110, (char)75, (char)5, (char)132, (char)64, (char)220, (char)14, (char)170, (char)142, (char)178, (char)23, (char)58, (char)31, (char)192, (char)43, (char)215, (char)169, (char)255, (char)40, (char)215, (char)11, (char)198, (char)244, (char)0, (char)16, (char)194, (char)65, (char)218, (char)78, (char)104, (char)41, (char)237, (char)77, (char)178, (char)113, (char)72, (char)30, (char)30, (char)110, (char)160, (char)165, (char)141, (char)115, (char)218, (char)147, (char)224}, 0) ;
        p248.target_component_SET((char)232) ;
        p248.target_system_SET((char)236) ;
        p248.message_type_SET((char)14304) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)77);
            assert(pack.ver_GET() == (char)205);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)98, (byte) - 61, (byte) - 18, (byte)111, (byte)33, (byte)67, (byte)37, (byte) - 30, (byte)127, (byte)22, (byte) - 33, (byte) - 72, (byte)23, (byte) - 33, (byte)79, (byte) - 93, (byte) - 118, (byte) - 42, (byte)122, (byte) - 77, (byte) - 114, (byte) - 120, (byte)113, (byte)78, (byte)65, (byte)127, (byte)83, (byte) - 35, (byte) - 15, (byte)42, (byte) - 30, (byte)87}));
            assert(pack.address_GET() == (char)36081);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)36081) ;
        p249.value_SET(new byte[] {(byte)98, (byte) - 61, (byte) - 18, (byte)111, (byte)33, (byte)67, (byte)37, (byte) - 30, (byte)127, (byte)22, (byte) - 33, (byte) - 72, (byte)23, (byte) - 33, (byte)79, (byte) - 93, (byte) - 118, (byte) - 42, (byte)122, (byte) - 77, (byte) - 114, (byte) - 120, (byte)113, (byte)78, (byte)65, (byte)127, (byte)83, (byte) - 35, (byte) - 15, (byte)42, (byte) - 30, (byte)87}, 0) ;
        p249.ver_SET((char)205) ;
        p249.type_SET((char)77) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3737929455859519048L);
            assert(pack.x_GET() == 2.4381375E38F);
            assert(pack.name_LEN(ph) == 5);
            assert(pack.name_TRY(ph).equals("sXwep"));
            assert(pack.y_GET() == 1.1455777E38F);
            assert(pack.z_GET() == 1.8391892E38F);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.x_SET(2.4381375E38F) ;
        p250.y_SET(1.1455777E38F) ;
        p250.z_SET(1.8391892E38F) ;
        p250.time_usec_SET(3737929455859519048L) ;
        p250.name_SET("sXwep", PH) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 6);
            assert(pack.name_TRY(ph).equals("zYxeol"));
            assert(pack.time_boot_ms_GET() == 2379827261L);
            assert(pack.value_GET() == -2.5186988E38F);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.value_SET(-2.5186988E38F) ;
        p251.name_SET("zYxeol", PH) ;
        p251.time_boot_ms_SET(2379827261L) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2466669718L);
            assert(pack.value_GET() == 1249834062);
            assert(pack.name_LEN(ph) == 3);
            assert(pack.name_TRY(ph).equals("hxo"));
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.value_SET(1249834062) ;
        p252.name_SET("hxo", PH) ;
        p252.time_boot_ms_SET(2466669718L) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 31);
            assert(pack.text_TRY(ph).equals("zmarQrdsxoxaumgjCqfqxzauhbHzdaP"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_ALERT);
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("zmarQrdsxoxaumgjCqfqxzauhbHzdaP", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_ALERT) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2492751300L);
            assert(pack.ind_GET() == (char)177);
            assert(pack.value_GET() == -5.590525E37F);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.value_SET(-5.590525E37F) ;
        p254.ind_SET((char)177) ;
        p254.time_boot_ms_SET(2492751300L) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)220);
            assert(pack.target_system_GET() == (char)133);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)36, (char)60, (char)220, (char)175, (char)216, (char)237, (char)250, (char)119, (char)7, (char)13, (char)127, (char)87, (char)108, (char)182, (char)217, (char)154, (char)80, (char)247, (char)41, (char)48, (char)68, (char)79, (char)78, (char)168, (char)108, (char)72, (char)152, (char)82, (char)17, (char)63, (char)200, (char)205}));
            assert(pack.initial_timestamp_GET() == 7041630116804087782L);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_component_SET((char)220) ;
        p256.target_system_SET((char)133) ;
        p256.initial_timestamp_SET(7041630116804087782L) ;
        p256.secret_key_SET(new char[] {(char)36, (char)60, (char)220, (char)175, (char)216, (char)237, (char)250, (char)119, (char)7, (char)13, (char)127, (char)87, (char)108, (char)182, (char)217, (char)154, (char)80, (char)247, (char)41, (char)48, (char)68, (char)79, (char)78, (char)168, (char)108, (char)72, (char)152, (char)82, (char)17, (char)63, (char)200, (char)205}, 0) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.state_GET() == (char)160);
            assert(pack.time_boot_ms_GET() == 1920767438L);
            assert(pack.last_change_ms_GET() == 2788687040L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.last_change_ms_SET(2788687040L) ;
        p257.state_SET((char)160) ;
        p257.time_boot_ms_SET(1920767438L) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)141);
            assert(pack.target_component_GET() == (char)129);
            assert(pack.tune_LEN(ph) == 16);
            assert(pack.tune_TRY(ph).equals("phnDZitdojMmhwer"));
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)141) ;
        p258.tune_SET("phnDZitdojMmhwer", PH) ;
        p258.target_component_SET((char)129) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.lens_id_GET() == (char)208);
            assert(pack.firmware_version_GET() == 823131219L);
            assert(pack.cam_definition_uri_LEN(ph) == 58);
            assert(pack.cam_definition_uri_TRY(ph).equals("eoslsxtdetngZsllmlyJpavegaqmelfapWmqvpShympkibcwjdzwuezwzn"));
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE));
            assert(pack.resolution_v_GET() == (char)56477);
            assert(pack.cam_definition_version_GET() == (char)22307);
            assert(pack.sensor_size_h_GET() == 1.3732464E38F);
            assert(pack.sensor_size_v_GET() == 3.0571133E37F);
            assert(pack.time_boot_ms_GET() == 3281420175L);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)73, (char)164, (char)48, (char)215, (char)139, (char)114, (char)169, (char)164, (char)240, (char)174, (char)87, (char)76, (char)165, (char)28, (char)16, (char)20, (char)219, (char)22, (char)104, (char)175, (char)85, (char)10, (char)169, (char)139, (char)40, (char)46, (char)188, (char)130, (char)32, (char)101, (char)184, (char)53}));
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)58, (char)165, (char)145, (char)255, (char)133, (char)64, (char)250, (char)33, (char)30, (char)103, (char)138, (char)62, (char)253, (char)146, (char)249, (char)124, (char)7, (char)53, (char)34, (char)255, (char)169, (char)152, (char)87, (char)186, (char)36, (char)119, (char)125, (char)225, (char)72, (char)109, (char)27, (char)44}));
            assert(pack.focal_length_GET() == 2.243114E38F);
            assert(pack.resolution_h_GET() == (char)39185);
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.cam_definition_version_SET((char)22307) ;
        p259.resolution_v_SET((char)56477) ;
        p259.focal_length_SET(2.243114E38F) ;
        p259.firmware_version_SET(823131219L) ;
        p259.vendor_name_SET(new char[] {(char)58, (char)165, (char)145, (char)255, (char)133, (char)64, (char)250, (char)33, (char)30, (char)103, (char)138, (char)62, (char)253, (char)146, (char)249, (char)124, (char)7, (char)53, (char)34, (char)255, (char)169, (char)152, (char)87, (char)186, (char)36, (char)119, (char)125, (char)225, (char)72, (char)109, (char)27, (char)44}, 0) ;
        p259.sensor_size_v_SET(3.0571133E37F) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE)) ;
        p259.sensor_size_h_SET(1.3732464E38F) ;
        p259.cam_definition_uri_SET("eoslsxtdetngZsllmlyJpavegaqmelfapWmqvpShympkibcwjdzwuezwzn", PH) ;
        p259.lens_id_SET((char)208) ;
        p259.resolution_h_SET((char)39185) ;
        p259.time_boot_ms_SET(3281420175L) ;
        p259.model_name_SET(new char[] {(char)73, (char)164, (char)48, (char)215, (char)139, (char)114, (char)169, (char)164, (char)240, (char)174, (char)87, (char)76, (char)165, (char)28, (char)16, (char)20, (char)219, (char)22, (char)104, (char)175, (char)85, (char)10, (char)169, (char)139, (char)40, (char)46, (char)188, (char)130, (char)32, (char)101, (char)184, (char)53}, 0) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1150604985L);
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY) ;
        p260.time_boot_ms_SET(1150604985L) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.write_speed_GET() == -3.230974E38F);
            assert(pack.time_boot_ms_GET() == 1130173166L);
            assert(pack.used_capacity_GET() == -7.853931E37F);
            assert(pack.status_GET() == (char)244);
            assert(pack.total_capacity_GET() == -8.185651E37F);
            assert(pack.storage_id_GET() == (char)58);
            assert(pack.read_speed_GET() == -7.6392717E37F);
            assert(pack.storage_count_GET() == (char)254);
            assert(pack.available_capacity_GET() == -2.4121346E38F);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.write_speed_SET(-3.230974E38F) ;
        p261.storage_id_SET((char)58) ;
        p261.read_speed_SET(-7.6392717E37F) ;
        p261.time_boot_ms_SET(1130173166L) ;
        p261.status_SET((char)244) ;
        p261.used_capacity_SET(-7.853931E37F) ;
        p261.available_capacity_SET(-2.4121346E38F) ;
        p261.storage_count_SET((char)254) ;
        p261.total_capacity_SET(-8.185651E37F) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.video_status_GET() == (char)32);
            assert(pack.available_capacity_GET() == 9.344074E37F);
            assert(pack.recording_time_ms_GET() == 127255880L);
            assert(pack.time_boot_ms_GET() == 1489071416L);
            assert(pack.image_interval_GET() == 3.3928837E38F);
            assert(pack.image_status_GET() == (char)208);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.available_capacity_SET(9.344074E37F) ;
        p262.recording_time_ms_SET(127255880L) ;
        p262.video_status_SET((char)32) ;
        p262.time_boot_ms_SET(1489071416L) ;
        p262.image_status_SET((char)208) ;
        p262.image_interval_SET(3.3928837E38F) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.capture_result_GET() == (byte) - 67);
            assert(pack.file_url_LEN(ph) == 100);
            assert(pack.file_url_TRY(ph).equals("imqTpqiydfipzcbjocigtgwdIIVxlyqbiuujyjhlblgWnsOrxmkcpniibzeFehtteenayzchymmvspgzhmzyibsvyrgoNoYnsviP"));
            assert(pack.lon_GET() == 2017487983);
            assert(pack.time_boot_ms_GET() == 1792736816L);
            assert(pack.alt_GET() == -1748691203);
            assert(pack.image_index_GET() == -629020295);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.8164562E38F, -1.0346681E38F, 1.1089974E38F, 2.6878892E38F}));
            assert(pack.lat_GET() == 1423814233);
            assert(pack.time_utc_GET() == 9215221515406077157L);
            assert(pack.camera_id_GET() == (char)84);
            assert(pack.relative_alt_GET() == -1794591867);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.q_SET(new float[] {2.8164562E38F, -1.0346681E38F, 1.1089974E38F, 2.6878892E38F}, 0) ;
        p263.time_utc_SET(9215221515406077157L) ;
        p263.capture_result_SET((byte) - 67) ;
        p263.file_url_SET("imqTpqiydfipzcbjocigtgwdIIVxlyqbiuujyjhlblgWnsOrxmkcpniibzeFehtteenayzchymmvspgzhmzyibsvyrgoNoYnsviP", PH) ;
        p263.lat_SET(1423814233) ;
        p263.time_boot_ms_SET(1792736816L) ;
        p263.image_index_SET(-629020295) ;
        p263.lon_SET(2017487983) ;
        p263.alt_SET(-1748691203) ;
        p263.relative_alt_SET(-1794591867) ;
        p263.camera_id_SET((char)84) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.arming_time_utc_GET() == 7036834019384051585L);
            assert(pack.time_boot_ms_GET() == 3070363458L);
            assert(pack.flight_uuid_GET() == 9182558090771082530L);
            assert(pack.takeoff_time_utc_GET() == 3736972083410422572L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.flight_uuid_SET(9182558090771082530L) ;
        p264.takeoff_time_utc_SET(3736972083410422572L) ;
        p264.time_boot_ms_SET(3070363458L) ;
        p264.arming_time_utc_SET(7036834019384051585L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -1.3406438E38F);
            assert(pack.time_boot_ms_GET() == 3478670263L);
            assert(pack.roll_GET() == 3.297657E38F);
            assert(pack.yaw_GET() == -2.3551518E37F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.roll_SET(3.297657E38F) ;
        p265.pitch_SET(-1.3406438E38F) ;
        p265.time_boot_ms_SET(3478670263L) ;
        p265.yaw_SET(-2.3551518E37F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)63172);
            assert(pack.target_component_GET() == (char)151);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)33, (char)249, (char)166, (char)2, (char)14, (char)215, (char)231, (char)228, (char)62, (char)52, (char)49, (char)83, (char)171, (char)27, (char)167, (char)243, (char)41, (char)106, (char)48, (char)255, (char)95, (char)92, (char)10, (char)58, (char)1, (char)206, (char)163, (char)45, (char)59, (char)30, (char)27, (char)244, (char)194, (char)9, (char)235, (char)247, (char)0, (char)94, (char)150, (char)4, (char)172, (char)52, (char)163, (char)103, (char)29, (char)180, (char)85, (char)32, (char)32, (char)135, (char)176, (char)85, (char)253, (char)217, (char)216, (char)93, (char)80, (char)157, (char)158, (char)214, (char)244, (char)210, (char)204, (char)255, (char)11, (char)220, (char)151, (char)131, (char)62, (char)41, (char)204, (char)9, (char)79, (char)44, (char)82, (char)232, (char)32, (char)7, (char)67, (char)25, (char)68, (char)234, (char)62, (char)178, (char)144, (char)143, (char)60, (char)25, (char)235, (char)18, (char)131, (char)28, (char)219, (char)67, (char)24, (char)199, (char)68, (char)171, (char)91, (char)234, (char)21, (char)168, (char)28, (char)32, (char)207, (char)174, (char)128, (char)99, (char)124, (char)26, (char)242, (char)157, (char)136, (char)162, (char)210, (char)68, (char)191, (char)226, (char)250, (char)161, (char)69, (char)130, (char)157, (char)254, (char)229, (char)229, (char)217, (char)201, (char)63, (char)180, (char)243, (char)2, (char)81, (char)203, (char)145, (char)156, (char)52, (char)97, (char)196, (char)109, (char)211, (char)103, (char)192, (char)26, (char)240, (char)49, (char)234, (char)192, (char)104, (char)85, (char)212, (char)244, (char)228, (char)180, (char)44, (char)43, (char)50, (char)84, (char)228, (char)29, (char)36, (char)31, (char)128, (char)112, (char)217, (char)107, (char)175, (char)24, (char)67, (char)187, (char)17, (char)175, (char)224, (char)171, (char)96, (char)176, (char)27, (char)160, (char)8, (char)120, (char)150, (char)126, (char)10, (char)239, (char)153, (char)136, (char)179, (char)20, (char)51, (char)51, (char)134, (char)28, (char)197, (char)76, (char)2, (char)185, (char)160, (char)199, (char)165, (char)7, (char)86, (char)147, (char)91, (char)169, (char)139, (char)179, (char)61, (char)59, (char)139, (char)173, (char)117, (char)6, (char)4, (char)52, (char)73, (char)155, (char)96, (char)239, (char)103, (char)19, (char)92, (char)65, (char)219, (char)164, (char)26, (char)134, (char)168, (char)81, (char)187, (char)16, (char)130, (char)149, (char)134, (char)249, (char)196, (char)37, (char)153, (char)35, (char)127, (char)235, (char)105, (char)151, (char)100, (char)31, (char)117, (char)170, (char)57, (char)54, (char)22}));
            assert(pack.first_message_offset_GET() == (char)107);
            assert(pack.length_GET() == (char)14);
            assert(pack.target_system_GET() == (char)218);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_component_SET((char)151) ;
        p266.target_system_SET((char)218) ;
        p266.length_SET((char)14) ;
        p266.data__SET(new char[] {(char)33, (char)249, (char)166, (char)2, (char)14, (char)215, (char)231, (char)228, (char)62, (char)52, (char)49, (char)83, (char)171, (char)27, (char)167, (char)243, (char)41, (char)106, (char)48, (char)255, (char)95, (char)92, (char)10, (char)58, (char)1, (char)206, (char)163, (char)45, (char)59, (char)30, (char)27, (char)244, (char)194, (char)9, (char)235, (char)247, (char)0, (char)94, (char)150, (char)4, (char)172, (char)52, (char)163, (char)103, (char)29, (char)180, (char)85, (char)32, (char)32, (char)135, (char)176, (char)85, (char)253, (char)217, (char)216, (char)93, (char)80, (char)157, (char)158, (char)214, (char)244, (char)210, (char)204, (char)255, (char)11, (char)220, (char)151, (char)131, (char)62, (char)41, (char)204, (char)9, (char)79, (char)44, (char)82, (char)232, (char)32, (char)7, (char)67, (char)25, (char)68, (char)234, (char)62, (char)178, (char)144, (char)143, (char)60, (char)25, (char)235, (char)18, (char)131, (char)28, (char)219, (char)67, (char)24, (char)199, (char)68, (char)171, (char)91, (char)234, (char)21, (char)168, (char)28, (char)32, (char)207, (char)174, (char)128, (char)99, (char)124, (char)26, (char)242, (char)157, (char)136, (char)162, (char)210, (char)68, (char)191, (char)226, (char)250, (char)161, (char)69, (char)130, (char)157, (char)254, (char)229, (char)229, (char)217, (char)201, (char)63, (char)180, (char)243, (char)2, (char)81, (char)203, (char)145, (char)156, (char)52, (char)97, (char)196, (char)109, (char)211, (char)103, (char)192, (char)26, (char)240, (char)49, (char)234, (char)192, (char)104, (char)85, (char)212, (char)244, (char)228, (char)180, (char)44, (char)43, (char)50, (char)84, (char)228, (char)29, (char)36, (char)31, (char)128, (char)112, (char)217, (char)107, (char)175, (char)24, (char)67, (char)187, (char)17, (char)175, (char)224, (char)171, (char)96, (char)176, (char)27, (char)160, (char)8, (char)120, (char)150, (char)126, (char)10, (char)239, (char)153, (char)136, (char)179, (char)20, (char)51, (char)51, (char)134, (char)28, (char)197, (char)76, (char)2, (char)185, (char)160, (char)199, (char)165, (char)7, (char)86, (char)147, (char)91, (char)169, (char)139, (char)179, (char)61, (char)59, (char)139, (char)173, (char)117, (char)6, (char)4, (char)52, (char)73, (char)155, (char)96, (char)239, (char)103, (char)19, (char)92, (char)65, (char)219, (char)164, (char)26, (char)134, (char)168, (char)81, (char)187, (char)16, (char)130, (char)149, (char)134, (char)249, (char)196, (char)37, (char)153, (char)35, (char)127, (char)235, (char)105, (char)151, (char)100, (char)31, (char)117, (char)170, (char)57, (char)54, (char)22}, 0) ;
        p266.first_message_offset_SET((char)107) ;
        p266.sequence_SET((char)63172) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)17693);
            assert(pack.first_message_offset_GET() == (char)16);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)40, (char)216, (char)114, (char)102, (char)16, (char)242, (char)66, (char)196, (char)252, (char)62, (char)8, (char)165, (char)212, (char)22, (char)81, (char)74, (char)68, (char)251, (char)17, (char)9, (char)155, (char)93, (char)247, (char)225, (char)0, (char)22, (char)89, (char)67, (char)152, (char)116, (char)63, (char)55, (char)145, (char)105, (char)96, (char)13, (char)236, (char)240, (char)125, (char)203, (char)49, (char)150, (char)35, (char)204, (char)101, (char)184, (char)190, (char)43, (char)123, (char)173, (char)89, (char)193, (char)6, (char)58, (char)245, (char)182, (char)138, (char)213, (char)43, (char)231, (char)238, (char)24, (char)68, (char)71, (char)243, (char)94, (char)10, (char)219, (char)238, (char)52, (char)149, (char)183, (char)77, (char)55, (char)233, (char)255, (char)212, (char)95, (char)18, (char)113, (char)216, (char)187, (char)46, (char)33, (char)222, (char)249, (char)33, (char)205, (char)124, (char)227, (char)5, (char)77, (char)129, (char)188, (char)169, (char)246, (char)206, (char)45, (char)49, (char)248, (char)63, (char)157, (char)15, (char)141, (char)184, (char)91, (char)73, (char)241, (char)26, (char)45, (char)65, (char)71, (char)161, (char)104, (char)186, (char)153, (char)10, (char)210, (char)178, (char)62, (char)167, (char)46, (char)97, (char)129, (char)220, (char)140, (char)3, (char)47, (char)182, (char)43, (char)66, (char)157, (char)87, (char)62, (char)151, (char)34, (char)98, (char)193, (char)66, (char)165, (char)217, (char)191, (char)66, (char)4, (char)122, (char)249, (char)133, (char)223, (char)15, (char)91, (char)74, (char)23, (char)176, (char)138, (char)203, (char)220, (char)191, (char)46, (char)239, (char)78, (char)213, (char)188, (char)90, (char)34, (char)143, (char)40, (char)172, (char)30, (char)29, (char)29, (char)100, (char)83, (char)226, (char)196, (char)134, (char)235, (char)212, (char)38, (char)196, (char)7, (char)7, (char)1, (char)183, (char)116, (char)79, (char)190, (char)50, (char)104, (char)229, (char)212, (char)41, (char)224, (char)65, (char)207, (char)115, (char)201, (char)222, (char)149, (char)7, (char)103, (char)113, (char)214, (char)163, (char)156, (char)181, (char)245, (char)24, (char)244, (char)226, (char)125, (char)90, (char)135, (char)214, (char)203, (char)21, (char)32, (char)119, (char)236, (char)66, (char)155, (char)8, (char)98, (char)68, (char)207, (char)236, (char)0, (char)99, (char)121, (char)219, (char)11, (char)165, (char)166, (char)42, (char)67, (char)160, (char)90, (char)101, (char)182, (char)162, (char)202, (char)104, (char)37, (char)44, (char)155, (char)105, (char)232, (char)76, (char)66, (char)78}));
            assert(pack.length_GET() == (char)199);
            assert(pack.target_system_GET() == (char)128);
            assert(pack.target_component_GET() == (char)124);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.first_message_offset_SET((char)16) ;
        p267.length_SET((char)199) ;
        p267.sequence_SET((char)17693) ;
        p267.data__SET(new char[] {(char)40, (char)216, (char)114, (char)102, (char)16, (char)242, (char)66, (char)196, (char)252, (char)62, (char)8, (char)165, (char)212, (char)22, (char)81, (char)74, (char)68, (char)251, (char)17, (char)9, (char)155, (char)93, (char)247, (char)225, (char)0, (char)22, (char)89, (char)67, (char)152, (char)116, (char)63, (char)55, (char)145, (char)105, (char)96, (char)13, (char)236, (char)240, (char)125, (char)203, (char)49, (char)150, (char)35, (char)204, (char)101, (char)184, (char)190, (char)43, (char)123, (char)173, (char)89, (char)193, (char)6, (char)58, (char)245, (char)182, (char)138, (char)213, (char)43, (char)231, (char)238, (char)24, (char)68, (char)71, (char)243, (char)94, (char)10, (char)219, (char)238, (char)52, (char)149, (char)183, (char)77, (char)55, (char)233, (char)255, (char)212, (char)95, (char)18, (char)113, (char)216, (char)187, (char)46, (char)33, (char)222, (char)249, (char)33, (char)205, (char)124, (char)227, (char)5, (char)77, (char)129, (char)188, (char)169, (char)246, (char)206, (char)45, (char)49, (char)248, (char)63, (char)157, (char)15, (char)141, (char)184, (char)91, (char)73, (char)241, (char)26, (char)45, (char)65, (char)71, (char)161, (char)104, (char)186, (char)153, (char)10, (char)210, (char)178, (char)62, (char)167, (char)46, (char)97, (char)129, (char)220, (char)140, (char)3, (char)47, (char)182, (char)43, (char)66, (char)157, (char)87, (char)62, (char)151, (char)34, (char)98, (char)193, (char)66, (char)165, (char)217, (char)191, (char)66, (char)4, (char)122, (char)249, (char)133, (char)223, (char)15, (char)91, (char)74, (char)23, (char)176, (char)138, (char)203, (char)220, (char)191, (char)46, (char)239, (char)78, (char)213, (char)188, (char)90, (char)34, (char)143, (char)40, (char)172, (char)30, (char)29, (char)29, (char)100, (char)83, (char)226, (char)196, (char)134, (char)235, (char)212, (char)38, (char)196, (char)7, (char)7, (char)1, (char)183, (char)116, (char)79, (char)190, (char)50, (char)104, (char)229, (char)212, (char)41, (char)224, (char)65, (char)207, (char)115, (char)201, (char)222, (char)149, (char)7, (char)103, (char)113, (char)214, (char)163, (char)156, (char)181, (char)245, (char)24, (char)244, (char)226, (char)125, (char)90, (char)135, (char)214, (char)203, (char)21, (char)32, (char)119, (char)236, (char)66, (char)155, (char)8, (char)98, (char)68, (char)207, (char)236, (char)0, (char)99, (char)121, (char)219, (char)11, (char)165, (char)166, (char)42, (char)67, (char)160, (char)90, (char)101, (char)182, (char)162, (char)202, (char)104, (char)37, (char)44, (char)155, (char)105, (char)232, (char)76, (char)66, (char)78}, 0) ;
        p267.target_component_SET((char)124) ;
        p267.target_system_SET((char)128) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)12277);
            assert(pack.target_component_GET() == (char)135);
            assert(pack.target_system_GET() == (char)53);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_component_SET((char)135) ;
        p268.target_system_SET((char)53) ;
        p268.sequence_SET((char)12277) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.rotation_GET() == (char)7008);
            assert(pack.uri_LEN(ph) == 81);
            assert(pack.uri_TRY(ph).equals("cObcctanvffbiQttccGGyfzwylvdehNbyipcmlnRgkbakLxvDcbqhvXusvOQjuwjvrdxsijboxuxaJkBh"));
            assert(pack.status_GET() == (char)66);
            assert(pack.framerate_GET() == 2.1376506E38F);
            assert(pack.camera_id_GET() == (char)109);
            assert(pack.resolution_h_GET() == (char)31593);
            assert(pack.bitrate_GET() == 3044840752L);
            assert(pack.resolution_v_GET() == (char)35338);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.bitrate_SET(3044840752L) ;
        p269.uri_SET("cObcctanvffbiQttccGGyfzwylvdehNbyipcmlnRgkbakLxvDcbqhvXusvOQjuwjvrdxsijboxuxaJkBh", PH) ;
        p269.camera_id_SET((char)109) ;
        p269.framerate_SET(2.1376506E38F) ;
        p269.status_SET((char)66) ;
        p269.resolution_v_SET((char)35338) ;
        p269.rotation_SET((char)7008) ;
        p269.resolution_h_SET((char)31593) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.uri_LEN(ph) == 80);
            assert(pack.uri_TRY(ph).equals("ohvlLzroiubczimtvujwjadgocarxyOmwusmemyuqwbcythhqbgnzeuydcqqisxwzjximoljcqwSitkg"));
            assert(pack.resolution_v_GET() == (char)48680);
            assert(pack.resolution_h_GET() == (char)10540);
            assert(pack.bitrate_GET() == 1852527296L);
            assert(pack.camera_id_GET() == (char)99);
            assert(pack.target_component_GET() == (char)46);
            assert(pack.framerate_GET() == 4.2176903E37F);
            assert(pack.rotation_GET() == (char)41431);
            assert(pack.target_system_GET() == (char)171);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_component_SET((char)46) ;
        p270.resolution_v_SET((char)48680) ;
        p270.rotation_SET((char)41431) ;
        p270.camera_id_SET((char)99) ;
        p270.framerate_SET(4.2176903E37F) ;
        p270.bitrate_SET(1852527296L) ;
        p270.uri_SET("ohvlLzroiubczimtvujwjadgocarxyOmwusmemyuqwbcythhqbgnzeuydcqqisxwzjximoljcqwSitkg", PH) ;
        p270.resolution_h_SET((char)10540) ;
        p270.target_system_SET((char)171) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 17);
            assert(pack.ssid_TRY(ph).equals("qvzhhqnybvggbjCnz"));
            assert(pack.password_LEN(ph) == 55);
            assert(pack.password_TRY(ph).equals("bUjvlKqktMYussmoydunbzAhfluFPzekjnrlkhiqaMjxghiezvggokx"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("bUjvlKqktMYussmoydunbzAhfluFPzekjnrlkhiqaMjxghiezvggokx", PH) ;
        p299.ssid_SET("qvzhhqnybvggbjCnz", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)190, (char)125, (char)61, (char)247, (char)17, (char)72, (char)16, (char)104}));
            assert(pack.max_version_GET() == (char)64573);
            assert(pack.version_GET() == (char)54325);
            assert(pack.min_version_GET() == (char)51269);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)235, (char)211, (char)185, (char)92, (char)241, (char)28, (char)198, (char)185}));
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.library_version_hash_SET(new char[] {(char)235, (char)211, (char)185, (char)92, (char)241, (char)28, (char)198, (char)185}, 0) ;
        p300.max_version_SET((char)64573) ;
        p300.spec_version_hash_SET(new char[] {(char)190, (char)125, (char)61, (char)247, (char)17, (char)72, (char)16, (char)104}, 0) ;
        p300.version_SET((char)54325) ;
        p300.min_version_SET((char)51269) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
            assert(pack.time_usec_GET() == 7556939269590546458L);
            assert(pack.vendor_specific_status_code_GET() == (char)63195);
            assert(pack.uptime_sec_GET() == 638221136L);
            assert(pack.sub_mode_GET() == (char)52);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.time_usec_SET(7556939269590546458L) ;
        p310.sub_mode_SET((char)52) ;
        p310.vendor_specific_status_code_SET((char)63195) ;
        p310.uptime_sec_SET(638221136L) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.sw_version_major_GET() == (char)249);
            assert(pack.time_usec_GET() == 1370994011310019096L);
            assert(pack.hw_version_major_GET() == (char)50);
            assert(pack.hw_version_minor_GET() == (char)100);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)57, (char)88, (char)84, (char)227, (char)191, (char)72, (char)182, (char)225, (char)199, (char)201, (char)194, (char)213, (char)217, (char)65, (char)137, (char)224}));
            assert(pack.sw_vcs_commit_GET() == 1238501124L);
            assert(pack.uptime_sec_GET() == 1501536446L);
            assert(pack.sw_version_minor_GET() == (char)124);
            assert(pack.name_LEN(ph) == 61);
            assert(pack.name_TRY(ph).equals("hmbrcbparxJlutsnubPaxyxaagzmykqysxgSzpdrmdplcAvcqzqxnbomlcmlU"));
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.time_usec_SET(1370994011310019096L) ;
        p311.uptime_sec_SET(1501536446L) ;
        p311.hw_version_major_SET((char)50) ;
        p311.hw_unique_id_SET(new char[] {(char)57, (char)88, (char)84, (char)227, (char)191, (char)72, (char)182, (char)225, (char)199, (char)201, (char)194, (char)213, (char)217, (char)65, (char)137, (char)224}, 0) ;
        p311.hw_version_minor_SET((char)100) ;
        p311.sw_version_minor_SET((char)124) ;
        p311.sw_version_major_SET((char)249) ;
        p311.name_SET("hmbrcbparxJlutsnubPaxyxaagzmykqysxgSzpdrmdplcAvcqzqxnbomlcmlU", PH) ;
        p311.sw_vcs_commit_SET(1238501124L) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)119);
            assert(pack.target_component_GET() == (char)51);
            assert(pack.param_index_GET() == (short) -24406);
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("nvqarhxBE"));
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.param_index_SET((short) -24406) ;
        p320.target_system_SET((char)119) ;
        p320.target_component_SET((char)51) ;
        p320.param_id_SET("nvqarhxBE", PH) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)204);
            assert(pack.target_component_GET() == (char)140);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)204) ;
        p321.target_component_SET((char)140) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 71);
            assert(pack.param_value_TRY(ph).equals("pkaumbpYQfeifijygfehtpcjujqkztnghmqregvykrfjdlltmvjkmsaijmgndgoSntdztrv"));
            assert(pack.param_index_GET() == (char)28907);
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("nvdzutjbvwqa"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
            assert(pack.param_count_GET() == (char)38465);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_index_SET((char)28907) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32) ;
        p322.param_id_SET("nvdzutjbvwqa", PH) ;
        p322.param_count_SET((char)38465) ;
        p322.param_value_SET("pkaumbpYQfeifijygfehtpcjujqkztnghmqregvykrfjdlltmvjkmsaijmgndgoSntdztrv", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("rxEdsmdsl"));
            assert(pack.target_component_GET() == (char)243);
            assert(pack.target_system_GET() == (char)130);
            assert(pack.param_value_LEN(ph) == 18);
            assert(pack.param_value_TRY(ph).equals("gTNokgMcLRYdpfniji"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_component_SET((char)243) ;
        p323.param_id_SET("rxEdsmdsl", PH) ;
        p323.param_value_SET("gTNokgMcLRYdpfniji", PH) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16) ;
        p323.target_system_SET((char)130) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 47);
            assert(pack.param_value_TRY(ph).equals("hwnezfiMqxfvppvtfolhrzkoUaqmgiynnmwdDtrXitTgjuj"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_ACCEPTED);
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("Fhsfpwib"));
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("Fhsfpwib", PH) ;
        p324.param_value_SET("hwnezfiMqxfvppvtfolhrzkoUaqmgiynnmwdDtrXitTgjuj", PH) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_ACCEPTED) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 2815779146718601861L);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            assert(pack.min_distance_GET() == (char)62994);
            assert(pack.increment_GET() == (char)109);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)56869, (char)56962, (char)55056, (char)45831, (char)32859, (char)44003, (char)9566, (char)27125, (char)2182, (char)22910, (char)49994, (char)57745, (char)22301, (char)49684, (char)56367, (char)21174, (char)51648, (char)3771, (char)62817, (char)9542, (char)52170, (char)57653, (char)50574, (char)43085, (char)14722, (char)1392, (char)7399, (char)62935, (char)59051, (char)1362, (char)35992, (char)60581, (char)40238, (char)41669, (char)2767, (char)16932, (char)18528, (char)16721, (char)47675, (char)49329, (char)15775, (char)21227, (char)31039, (char)59080, (char)8831, (char)44603, (char)35646, (char)5908, (char)18155, (char)8865, (char)45097, (char)53436, (char)63428, (char)4118, (char)54701, (char)49786, (char)24163, (char)55640, (char)56937, (char)15965, (char)53789, (char)40825, (char)43198, (char)25157, (char)60422, (char)26920, (char)10791, (char)24622, (char)8219, (char)31976, (char)63355, (char)48699}));
            assert(pack.max_distance_GET() == (char)29348);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR) ;
        p330.min_distance_SET((char)62994) ;
        p330.distances_SET(new char[] {(char)56869, (char)56962, (char)55056, (char)45831, (char)32859, (char)44003, (char)9566, (char)27125, (char)2182, (char)22910, (char)49994, (char)57745, (char)22301, (char)49684, (char)56367, (char)21174, (char)51648, (char)3771, (char)62817, (char)9542, (char)52170, (char)57653, (char)50574, (char)43085, (char)14722, (char)1392, (char)7399, (char)62935, (char)59051, (char)1362, (char)35992, (char)60581, (char)40238, (char)41669, (char)2767, (char)16932, (char)18528, (char)16721, (char)47675, (char)49329, (char)15775, (char)21227, (char)31039, (char)59080, (char)8831, (char)44603, (char)35646, (char)5908, (char)18155, (char)8865, (char)45097, (char)53436, (char)63428, (char)4118, (char)54701, (char)49786, (char)24163, (char)55640, (char)56937, (char)15965, (char)53789, (char)40825, (char)43198, (char)25157, (char)60422, (char)26920, (char)10791, (char)24622, (char)8219, (char)31976, (char)63355, (char)48699}, 0) ;
        p330.increment_SET((char)109) ;
        p330.time_usec_SET(2815779146718601861L) ;
        p330.max_distance_SET((char)29348) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
    }

}
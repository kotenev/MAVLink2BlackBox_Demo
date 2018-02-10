
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
            long id = id__W(src);
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
            assert(pack.custom_mode_GET() == 684020035L);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED));
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_VTOL_RESERVED2);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_INVALID);
            assert(pack.mavlink_version_GET() == (char)78);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_UNINIT);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.custom_mode_SET(684020035L) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_INVALID) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED)) ;
        p0.mavlink_version_SET((char)78) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_VTOL_RESERVED2) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_UNINIT) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.battery_remaining_GET() == (byte) - 26);
            assert(pack.errors_count2_GET() == (char)44596);
            assert(pack.errors_count3_GET() == (char)10854);
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
            assert(pack.voltage_battery_GET() == (char)58861);
            assert(pack.current_battery_GET() == (short)7965);
            assert(pack.errors_count4_GET() == (char)39184);
            assert(pack.errors_comm_GET() == (char)62741);
            assert(pack.load_GET() == (char)40750);
            assert(pack.errors_count1_GET() == (char)56399);
            assert(pack.drop_rate_comm_GET() == (char)2336);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL));
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.errors_comm_SET((char)62741) ;
        p1.errors_count4_SET((char)39184) ;
        p1.load_SET((char)40750) ;
        p1.current_battery_SET((short)7965) ;
        p1.voltage_battery_SET((char)58861) ;
        p1.drop_rate_comm_SET((char)2336) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL)) ;
        p1.battery_remaining_SET((byte) - 26) ;
        p1.errors_count3_SET((char)10854) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL)) ;
        p1.errors_count1_SET((char)56399) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL)) ;
        p1.errors_count2_SET((char)44596) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 5103293913939463093L);
            assert(pack.time_boot_ms_GET() == 2484435886L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(2484435886L) ;
        p2.time_unix_usec_SET(5103293913939463093L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == 2.6695845E38F);
            assert(pack.z_GET() == 2.2488503E38F);
            assert(pack.vy_GET() == 1.6377567E38F);
            assert(pack.yaw_rate_GET() == -2.2639015E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.time_boot_ms_GET() == 3099889000L);
            assert(pack.x_GET() == -1.410884E38F);
            assert(pack.afy_GET() == -6.18094E37F);
            assert(pack.y_GET() == 3.1324403E38F);
            assert(pack.afz_GET() == 3.1905648E38F);
            assert(pack.type_mask_GET() == (char)40917);
            assert(pack.yaw_GET() == -3.3714417E38F);
            assert(pack.vx_GET() == 1.517937E38F);
            assert(pack.afx_GET() == 2.811435E38F);
        });
        POSITION_TARGET_LOCAL_NED p3 = new POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.afx_SET(2.811435E38F) ;
        p3.z_SET(2.2488503E38F) ;
        p3.yaw_rate_SET(-2.2639015E38F) ;
        p3.afz_SET(3.1905648E38F) ;
        p3.vy_SET(1.6377567E38F) ;
        p3.vx_SET(1.517937E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p3.yaw_SET(-3.3714417E38F) ;
        p3.x_SET(-1.410884E38F) ;
        p3.y_SET(3.1324403E38F) ;
        p3.type_mask_SET((char)40917) ;
        p3.vz_SET(2.6695845E38F) ;
        p3.afy_SET(-6.18094E37F) ;
        p3.time_boot_ms_SET(3099889000L) ;
        TestChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)113);
            assert(pack.target_component_GET() == (char)21);
            assert(pack.time_usec_GET() == 8180858069017002104L);
            assert(pack.seq_GET() == 3913675390L);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.seq_SET(3913675390L) ;
        p4.time_usec_SET(8180858069017002104L) ;
        p4.target_component_SET((char)21) ;
        p4.target_system_SET((char)113) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)200);
            assert(pack.passkey_LEN(ph) == 17);
            assert(pack.passkey_TRY(ph).equals("cqrcBazulygFjmlug"));
            assert(pack.version_GET() == (char)174);
            assert(pack.control_request_GET() == (char)50);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.passkey_SET("cqrcBazulygFjmlug", PH) ;
        p5.control_request_SET((char)50) ;
        p5.target_system_SET((char)200) ;
        p5.version_SET((char)174) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.ack_GET() == (char)93);
            assert(pack.gcs_system_id_GET() == (char)88);
            assert(pack.control_request_GET() == (char)145);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.gcs_system_id_SET((char)88) ;
        p6.ack_SET((char)93) ;
        p6.control_request_SET((char)145) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 26);
            assert(pack.key_TRY(ph).equals("sxFzZvebbzMmhgkggdotbzpgxO"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("sxFzZvebbzMmhgkggdotbzpgxO", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.custom_mode_GET() == 3693458112L);
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_AUTO_ARMED);
            assert(pack.target_system_GET() == (char)29);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED) ;
        p11.custom_mode_SET(3693458112L) ;
        p11.target_system_SET((char)29) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)41);
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("ubxki"));
            assert(pack.param_index_GET() == (short)2580);
            assert(pack.target_system_GET() == (char)175);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.param_index_SET((short)2580) ;
        p20.target_component_SET((char)41) ;
        p20.param_id_SET("ubxki", PH) ;
        p20.target_system_SET((char)175) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)53);
            assert(pack.target_component_GET() == (char)134);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)53) ;
        p21.target_component_SET((char)134) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)30890);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
            assert(pack.param_value_GET() == 1.4180325E38F);
            assert(pack.param_index_GET() == (char)62000);
            assert(pack.param_id_LEN(ph) == 14);
            assert(pack.param_id_TRY(ph).equals("ogtprffWazxbpc"));
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16) ;
        p22.param_index_SET((char)62000) ;
        p22.param_count_SET((char)30890) ;
        p22.param_id_SET("ogtprffWazxbpc", PH) ;
        p22.param_value_SET(1.4180325E38F) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
            assert(pack.param_value_GET() == 3.1667165E38F);
            assert(pack.param_id_LEN(ph) == 3);
            assert(pack.param_id_TRY(ph).equals("rph"));
            assert(pack.target_component_GET() == (char)40);
            assert(pack.target_system_GET() == (char)158);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.target_system_SET((char)158) ;
        p23.param_value_SET(3.1667165E38F) ;
        p23.target_component_SET((char)40) ;
        p23.param_id_SET("rph", PH) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.eph_GET() == (char)18875);
            assert(pack.lat_GET() == 624829304);
            assert(pack.alt_GET() == 1366926288);
            assert(pack.satellites_visible_GET() == (char)142);
            assert(pack.time_usec_GET() == 6521627366613030769L);
            assert(pack.vel_acc_TRY(ph) == 120114918L);
            assert(pack.hdg_acc_TRY(ph) == 731439579L);
            assert(pack.alt_ellipsoid_TRY(ph) == 1993493730);
            assert(pack.h_acc_TRY(ph) == 1811887688L);
            assert(pack.vel_GET() == (char)38735);
            assert(pack.lon_GET() == -1051707430);
            assert(pack.epv_GET() == (char)4260);
            assert(pack.cog_GET() == (char)58444);
            assert(pack.v_acc_TRY(ph) == 1305456827L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.satellites_visible_SET((char)142) ;
        p24.lon_SET(-1051707430) ;
        p24.eph_SET((char)18875) ;
        p24.lat_SET(624829304) ;
        p24.alt_SET(1366926288) ;
        p24.cog_SET((char)58444) ;
        p24.v_acc_SET(1305456827L, PH) ;
        p24.time_usec_SET(6521627366613030769L) ;
        p24.epv_SET((char)4260) ;
        p24.h_acc_SET(1811887688L, PH) ;
        p24.hdg_acc_SET(731439579L, PH) ;
        p24.vel_acc_SET(120114918L, PH) ;
        p24.vel_SET((char)38735) ;
        p24.alt_ellipsoid_SET(1993493730, PH) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)160, (char)161, (char)228, (char)28, (char)216, (char)129, (char)148, (char)17, (char)101, (char)102, (char)18, (char)218, (char)133, (char)168, (char)172, (char)186, (char)101, (char)239, (char)103, (char)132}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)118, (char)74, (char)132, (char)26, (char)129, (char)10, (char)100, (char)98, (char)131, (char)27, (char)26, (char)12, (char)55, (char)237, (char)209, (char)162, (char)44, (char)128, (char)11, (char)207}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)162, (char)114, (char)93, (char)89, (char)79, (char)221, (char)173, (char)90, (char)235, (char)228, (char)196, (char)202, (char)79, (char)238, (char)174, (char)37, (char)128, (char)242, (char)185, (char)38}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)51, (char)13, (char)173, (char)81, (char)234, (char)83, (char)10, (char)249, (char)118, (char)33, (char)46, (char)81, (char)88, (char)11, (char)125, (char)195, (char)149, (char)252, (char)33, (char)251}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)136, (char)114, (char)116, (char)154, (char)187, (char)133, (char)123, (char)34, (char)40, (char)50, (char)85, (char)36, (char)97, (char)22, (char)161, (char)0, (char)29, (char)156, (char)48, (char)102}));
            assert(pack.satellites_visible_GET() == (char)198);
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellites_visible_SET((char)198) ;
        p25.satellite_used_SET(new char[] {(char)51, (char)13, (char)173, (char)81, (char)234, (char)83, (char)10, (char)249, (char)118, (char)33, (char)46, (char)81, (char)88, (char)11, (char)125, (char)195, (char)149, (char)252, (char)33, (char)251}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)136, (char)114, (char)116, (char)154, (char)187, (char)133, (char)123, (char)34, (char)40, (char)50, (char)85, (char)36, (char)97, (char)22, (char)161, (char)0, (char)29, (char)156, (char)48, (char)102}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)162, (char)114, (char)93, (char)89, (char)79, (char)221, (char)173, (char)90, (char)235, (char)228, (char)196, (char)202, (char)79, (char)238, (char)174, (char)37, (char)128, (char)242, (char)185, (char)38}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)118, (char)74, (char)132, (char)26, (char)129, (char)10, (char)100, (char)98, (char)131, (char)27, (char)26, (char)12, (char)55, (char)237, (char)209, (char)162, (char)44, (char)128, (char)11, (char)207}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)160, (char)161, (char)228, (char)28, (char)216, (char)129, (char)148, (char)17, (char)101, (char)102, (char)18, (char)218, (char)133, (char)168, (char)172, (char)186, (char)101, (char)239, (char)103, (char)132}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.ymag_GET() == (short)30976);
            assert(pack.time_boot_ms_GET() == 2400852089L);
            assert(pack.zacc_GET() == (short) -13985);
            assert(pack.xacc_GET() == (short)20619);
            assert(pack.zgyro_GET() == (short)28690);
            assert(pack.xgyro_GET() == (short) -18584);
            assert(pack.zmag_GET() == (short)2143);
            assert(pack.ygyro_GET() == (short) -5456);
            assert(pack.xmag_GET() == (short) -4302);
            assert(pack.yacc_GET() == (short) -17143);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.xmag_SET((short) -4302) ;
        p26.yacc_SET((short) -17143) ;
        p26.zmag_SET((short)2143) ;
        p26.xacc_SET((short)20619) ;
        p26.ygyro_SET((short) -5456) ;
        p26.zacc_SET((short) -13985) ;
        p26.time_boot_ms_SET(2400852089L) ;
        p26.xgyro_SET((short) -18584) ;
        p26.ymag_SET((short)30976) ;
        p26.zgyro_SET((short)28690) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short) -13158);
            assert(pack.time_usec_GET() == 2550877163512093764L);
            assert(pack.ymag_GET() == (short)4672);
            assert(pack.yacc_GET() == (short)2056);
            assert(pack.zacc_GET() == (short) -29884);
            assert(pack.xacc_GET() == (short)4293);
            assert(pack.xgyro_GET() == (short) -12408);
            assert(pack.zgyro_GET() == (short)5827);
            assert(pack.xmag_GET() == (short)27537);
            assert(pack.zmag_GET() == (short)25551);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.xacc_SET((short)4293) ;
        p27.ygyro_SET((short) -13158) ;
        p27.zmag_SET((short)25551) ;
        p27.xgyro_SET((short) -12408) ;
        p27.time_usec_SET(2550877163512093764L) ;
        p27.xmag_SET((short)27537) ;
        p27.zgyro_SET((short)5827) ;
        p27.zacc_SET((short) -29884) ;
        p27.yacc_SET((short)2056) ;
        p27.ymag_SET((short)4672) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff2_GET() == (short) -19787);
            assert(pack.time_usec_GET() == 7489534169096888088L);
            assert(pack.press_abs_GET() == (short) -10806);
            assert(pack.press_diff1_GET() == (short)6587);
            assert(pack.temperature_GET() == (short)4766);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.temperature_SET((short)4766) ;
        p28.press_diff1_SET((short)6587) ;
        p28.time_usec_SET(7489534169096888088L) ;
        p28.press_abs_SET((short) -10806) ;
        p28.press_diff2_SET((short) -19787) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == 3.0004497E38F);
            assert(pack.time_boot_ms_GET() == 2098882978L);
            assert(pack.temperature_GET() == (short) -20924);
            assert(pack.press_diff_GET() == -1.2170482E38F);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.temperature_SET((short) -20924) ;
        p29.press_abs_SET(3.0004497E38F) ;
        p29.press_diff_SET(-1.2170482E38F) ;
        p29.time_boot_ms_SET(2098882978L) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == 2.2036215E38F);
            assert(pack.pitch_GET() == 3.0056602E38F);
            assert(pack.roll_GET() == 1.4898858E38F);
            assert(pack.time_boot_ms_GET() == 3810021820L);
            assert(pack.yawspeed_GET() == 2.6488016E37F);
            assert(pack.pitchspeed_GET() == -4.0343954E37F);
            assert(pack.yaw_GET() == 1.6824238E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.pitch_SET(3.0056602E38F) ;
        p30.pitchspeed_SET(-4.0343954E37F) ;
        p30.yaw_SET(1.6824238E38F) ;
        p30.time_boot_ms_SET(3810021820L) ;
        p30.yawspeed_SET(2.6488016E37F) ;
        p30.roll_SET(1.4898858E38F) ;
        p30.rollspeed_SET(2.2036215E38F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == 2.5965782E38F);
            assert(pack.q3_GET() == 5.7228505E36F);
            assert(pack.q1_GET() == 2.8103842E38F);
            assert(pack.rollspeed_GET() == 4.150552E37F);
            assert(pack.q2_GET() == 4.190351E37F);
            assert(pack.time_boot_ms_GET() == 2657609156L);
            assert(pack.q4_GET() == 1.9627844E38F);
            assert(pack.pitchspeed_GET() == -3.0294456E38F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q3_SET(5.7228505E36F) ;
        p31.pitchspeed_SET(-3.0294456E38F) ;
        p31.q1_SET(2.8103842E38F) ;
        p31.q4_SET(1.9627844E38F) ;
        p31.yawspeed_SET(2.5965782E38F) ;
        p31.rollspeed_SET(4.150552E37F) ;
        p31.time_boot_ms_SET(2657609156L) ;
        p31.q2_SET(4.190351E37F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == -2.2473413E38F);
            assert(pack.x_GET() == -3.8755308E36F);
            assert(pack.time_boot_ms_GET() == 2234792292L);
            assert(pack.vy_GET() == -2.1637717E38F);
            assert(pack.z_GET() == -2.9131222E36F);
            assert(pack.vx_GET() == 4.1969027E36F);
            assert(pack.y_GET() == -3.7912232E37F);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.vz_SET(-2.2473413E38F) ;
        p32.vx_SET(4.1969027E36F) ;
        p32.y_SET(-3.7912232E37F) ;
        p32.z_SET(-2.9131222E36F) ;
        p32.time_boot_ms_SET(2234792292L) ;
        p32.vy_SET(-2.1637717E38F) ;
        p32.x_SET(-3.8755308E36F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 467199676);
            assert(pack.vz_GET() == (short)3446);
            assert(pack.vy_GET() == (short)24489);
            assert(pack.lat_GET() == -1280758549);
            assert(pack.vx_GET() == (short) -30630);
            assert(pack.time_boot_ms_GET() == 3696709295L);
            assert(pack.relative_alt_GET() == 996326041);
            assert(pack.lon_GET() == -532796097);
            assert(pack.hdg_GET() == (char)25657);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.vz_SET((short)3446) ;
        p33.time_boot_ms_SET(3696709295L) ;
        p33.lon_SET(-532796097) ;
        p33.alt_SET(467199676) ;
        p33.vx_SET((short) -30630) ;
        p33.vy_SET((short)24489) ;
        p33.lat_SET(-1280758549) ;
        p33.hdg_SET((char)25657) ;
        p33.relative_alt_SET(996326041) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan3_scaled_GET() == (short)10724);
            assert(pack.chan1_scaled_GET() == (short) -25468);
            assert(pack.chan8_scaled_GET() == (short)5118);
            assert(pack.chan6_scaled_GET() == (short)32720);
            assert(pack.chan5_scaled_GET() == (short) -3304);
            assert(pack.rssi_GET() == (char)203);
            assert(pack.chan4_scaled_GET() == (short)26878);
            assert(pack.time_boot_ms_GET() == 3461828448L);
            assert(pack.chan2_scaled_GET() == (short)26185);
            assert(pack.chan7_scaled_GET() == (short)21277);
            assert(pack.port_GET() == (char)225);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.time_boot_ms_SET(3461828448L) ;
        p34.chan8_scaled_SET((short)5118) ;
        p34.rssi_SET((char)203) ;
        p34.port_SET((char)225) ;
        p34.chan2_scaled_SET((short)26185) ;
        p34.chan7_scaled_SET((short)21277) ;
        p34.chan5_scaled_SET((short) -3304) ;
        p34.chan6_scaled_SET((short)32720) ;
        p34.chan1_scaled_SET((short) -25468) ;
        p34.chan3_scaled_SET((short)10724) ;
        p34.chan4_scaled_SET((short)26878) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan6_raw_GET() == (char)63491);
            assert(pack.chan3_raw_GET() == (char)41176);
            assert(pack.chan8_raw_GET() == (char)4941);
            assert(pack.rssi_GET() == (char)182);
            assert(pack.time_boot_ms_GET() == 3390822961L);
            assert(pack.chan1_raw_GET() == (char)15018);
            assert(pack.chan4_raw_GET() == (char)20144);
            assert(pack.port_GET() == (char)24);
            assert(pack.chan7_raw_GET() == (char)25109);
            assert(pack.chan5_raw_GET() == (char)34039);
            assert(pack.chan2_raw_GET() == (char)7296);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan5_raw_SET((char)34039) ;
        p35.chan2_raw_SET((char)7296) ;
        p35.time_boot_ms_SET(3390822961L) ;
        p35.chan6_raw_SET((char)63491) ;
        p35.chan1_raw_SET((char)15018) ;
        p35.rssi_SET((char)182) ;
        p35.chan4_raw_SET((char)20144) ;
        p35.chan8_raw_SET((char)4941) ;
        p35.port_SET((char)24) ;
        p35.chan3_raw_SET((char)41176) ;
        p35.chan7_raw_SET((char)25109) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo3_raw_GET() == (char)59042);
            assert(pack.servo11_raw_TRY(ph) == (char)39095);
            assert(pack.servo10_raw_TRY(ph) == (char)29731);
            assert(pack.port_GET() == (char)131);
            assert(pack.time_usec_GET() == 1879054906L);
            assert(pack.servo15_raw_TRY(ph) == (char)24893);
            assert(pack.servo16_raw_TRY(ph) == (char)4384);
            assert(pack.servo8_raw_GET() == (char)28583);
            assert(pack.servo2_raw_GET() == (char)40880);
            assert(pack.servo12_raw_TRY(ph) == (char)44944);
            assert(pack.servo1_raw_GET() == (char)25455);
            assert(pack.servo14_raw_TRY(ph) == (char)65510);
            assert(pack.servo6_raw_GET() == (char)5300);
            assert(pack.servo4_raw_GET() == (char)49044);
            assert(pack.servo13_raw_TRY(ph) == (char)18279);
            assert(pack.servo9_raw_TRY(ph) == (char)11778);
            assert(pack.servo5_raw_GET() == (char)19125);
            assert(pack.servo7_raw_GET() == (char)28728);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo7_raw_SET((char)28728) ;
        p36.servo13_raw_SET((char)18279, PH) ;
        p36.servo12_raw_SET((char)44944, PH) ;
        p36.servo2_raw_SET((char)40880) ;
        p36.time_usec_SET(1879054906L) ;
        p36.servo4_raw_SET((char)49044) ;
        p36.servo1_raw_SET((char)25455) ;
        p36.servo6_raw_SET((char)5300) ;
        p36.servo8_raw_SET((char)28583) ;
        p36.servo5_raw_SET((char)19125) ;
        p36.servo16_raw_SET((char)4384, PH) ;
        p36.port_SET((char)131) ;
        p36.servo14_raw_SET((char)65510, PH) ;
        p36.servo10_raw_SET((char)29731, PH) ;
        p36.servo15_raw_SET((char)24893, PH) ;
        p36.servo11_raw_SET((char)39095, PH) ;
        p36.servo9_raw_SET((char)11778, PH) ;
        p36.servo3_raw_SET((char)59042) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)173);
            assert(pack.target_component_GET() == (char)117);
            assert(pack.end_index_GET() == (short)12812);
            assert(pack.start_index_GET() == (short) -7469);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.end_index_SET((short)12812) ;
        p37.target_component_SET((char)117) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p37.start_index_SET((short) -7469) ;
        p37.target_system_SET((char)173) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)102);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.end_index_GET() == (short) -16569);
            assert(pack.target_system_GET() == (char)224);
            assert(pack.start_index_GET() == (short)23692);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_component_SET((char)102) ;
        p38.end_index_SET((short) -16569) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p38.start_index_SET((short)23692) ;
        p38.target_system_SET((char)224) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)26588);
            assert(pack.target_system_GET() == (char)232);
            assert(pack.current_GET() == (char)179);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.autocontinue_GET() == (char)228);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_USER_3);
            assert(pack.y_GET() == -2.0553113E38F);
            assert(pack.param1_GET() == -2.829908E37F);
            assert(pack.target_component_GET() == (char)47);
            assert(pack.x_GET() == 1.7239438E38F);
            assert(pack.param3_GET() == 2.8898702E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.z_GET() == -1.5021975E38F);
            assert(pack.param2_GET() == -1.8148507E38F);
            assert(pack.param4_GET() == 1.7986258E38F);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.y_SET(-2.0553113E38F) ;
        p39.param4_SET(1.7986258E38F) ;
        p39.autocontinue_SET((char)228) ;
        p39.seq_SET((char)26588) ;
        p39.z_SET(-1.5021975E38F) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p39.param2_SET(-1.8148507E38F) ;
        p39.command_SET(MAV_CMD.MAV_CMD_USER_3) ;
        p39.target_component_SET((char)47) ;
        p39.target_system_SET((char)232) ;
        p39.x_SET(1.7239438E38F) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p39.current_SET((char)179) ;
        p39.param1_SET(-2.829908E37F) ;
        p39.param3_SET(2.8898702E38F) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)0);
            assert(pack.seq_GET() == (char)27593);
            assert(pack.target_component_GET() == (char)62);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.seq_SET((char)27593) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p40.target_component_SET((char)62) ;
        p40.target_system_SET((char)0) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)27548);
            assert(pack.target_component_GET() == (char)115);
            assert(pack.target_system_GET() == (char)84);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.seq_SET((char)27548) ;
        p41.target_component_SET((char)115) ;
        p41.target_system_SET((char)84) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)19748);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)19748) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)169);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)250);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_component_SET((char)169) ;
        p43.target_system_SET((char)250) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)81);
            assert(pack.count_GET() == (char)57545);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)187);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p44.target_system_SET((char)187) ;
        p44.count_SET((char)57545) ;
        p44.target_component_SET((char)81) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_component_GET() == (char)186);
            assert(pack.target_system_GET() == (char)55);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p45.target_system_SET((char)55) ;
        p45.target_component_SET((char)186) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)16061);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)16061) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)255);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)213);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED) ;
        p47.target_component_SET((char)213) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p47.target_system_SET((char)255) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 2106381360);
            assert(pack.altitude_GET() == -1164876310);
            assert(pack.time_usec_TRY(ph) == 6311195273025024213L);
            assert(pack.target_system_GET() == (char)219);
            assert(pack.longitude_GET() == 982897221);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.latitude_SET(2106381360) ;
        p48.altitude_SET(-1164876310) ;
        p48.time_usec_SET(6311195273025024213L, PH) ;
        p48.longitude_SET(982897221) ;
        p48.target_system_SET((char)219) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 792139197266163668L);
            assert(pack.latitude_GET() == -1234861892);
            assert(pack.longitude_GET() == 1214118533);
            assert(pack.altitude_GET() == 126871018);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.altitude_SET(126871018) ;
        p49.time_usec_SET(792139197266163668L, PH) ;
        p49.longitude_SET(1214118533) ;
        p49.latitude_SET(-1234861892) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (short)21864);
            assert(pack.scale_GET() == -2.0759298E38F);
            assert(pack.parameter_rc_channel_index_GET() == (char)156);
            assert(pack.target_component_GET() == (char)28);
            assert(pack.target_system_GET() == (char)214);
            assert(pack.param_id_LEN(ph) == 16);
            assert(pack.param_id_TRY(ph).equals("vytNuNvtkjrjjpbw"));
            assert(pack.param_value_min_GET() == 2.8458837E38F);
            assert(pack.param_value0_GET() == -6.1221683E37F);
            assert(pack.param_value_max_GET() == -2.2775517E38F);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.param_id_SET("vytNuNvtkjrjjpbw", PH) ;
        p50.parameter_rc_channel_index_SET((char)156) ;
        p50.param_value_max_SET(-2.2775517E38F) ;
        p50.param_value0_SET(-6.1221683E37F) ;
        p50.scale_SET(-2.0759298E38F) ;
        p50.param_value_min_SET(2.8458837E38F) ;
        p50.target_component_SET((char)28) ;
        p50.param_index_SET((short)21864) ;
        p50.target_system_SET((char)214) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)99);
            assert(pack.seq_GET() == (char)24859);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)127);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p51.target_component_SET((char)99) ;
        p51.seq_SET((char)24859) ;
        p51.target_system_SET((char)127) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2x_GET() == 3.124525E38F);
            assert(pack.p1z_GET() == -4.454032E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.target_system_GET() == (char)227);
            assert(pack.target_component_GET() == (char)252);
            assert(pack.p1y_GET() == -1.1635094E38F);
            assert(pack.p1x_GET() == -1.8177138E38F);
            assert(pack.p2y_GET() == -1.229365E38F);
            assert(pack.p2z_GET() == 2.9039634E38F);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p1x_SET(-1.8177138E38F) ;
        p54.p2y_SET(-1.229365E38F) ;
        p54.p1y_SET(-1.1635094E38F) ;
        p54.p2x_SET(3.124525E38F) ;
        p54.p2z_SET(2.9039634E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p54.p1z_SET(-4.454032E37F) ;
        p54.target_component_SET((char)252) ;
        p54.target_system_SET((char)227) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2x_GET() == -6.9337623E37F);
            assert(pack.p2y_GET() == 3.3906569E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.p1x_GET() == -2.5921911E38F);
            assert(pack.p2z_GET() == -2.2608796E38F);
            assert(pack.p1z_GET() == -6.611327E36F);
            assert(pack.p1y_GET() == -3.3939022E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p2x_SET(-6.9337623E37F) ;
        p55.p1y_SET(-3.3939022E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p55.p2y_SET(3.3906569E38F) ;
        p55.p1x_SET(-2.5921911E38F) ;
        p55.p1z_SET(-6.611327E36F) ;
        p55.p2z_SET(-2.2608796E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 1779008845916257921L);
            assert(pack.rollspeed_GET() == 1.1143323E37F);
            assert(pack.yawspeed_GET() == 1.5641994E38F);
            assert(pack.pitchspeed_GET() == -2.6527686E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.1210722E38F, -1.939879E37F, -1.9130563E38F, 3.0538342E38F}));
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.5630627E38F, -1.599932E38F, -3.31577E38F, 2.4251612E38F, -2.8284745E35F, 2.3968764E38F, 1.9292056E38F, 2.9037068E38F, 9.542518E37F}));
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.time_usec_SET(1779008845916257921L) ;
        p61.yawspeed_SET(1.5641994E38F) ;
        p61.q_SET(new float[] {3.1210722E38F, -1.939879E37F, -1.9130563E38F, 3.0538342E38F}, 0) ;
        p61.covariance_SET(new float[] {-2.5630627E38F, -1.599932E38F, -3.31577E38F, 2.4251612E38F, -2.8284745E35F, 2.3968764E38F, 1.9292056E38F, 2.9037068E38F, 9.542518E37F}, 0) ;
        p61.rollspeed_SET(1.1143323E37F) ;
        p61.pitchspeed_SET(-2.6527686E37F) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.nav_pitch_GET() == -2.9589928E38F);
            assert(pack.nav_roll_GET() == 1.5701561E37F);
            assert(pack.wp_dist_GET() == (char)48848);
            assert(pack.aspd_error_GET() == -1.8988256E38F);
            assert(pack.nav_bearing_GET() == (short) -24723);
            assert(pack.alt_error_GET() == 1.7954784E38F);
            assert(pack.target_bearing_GET() == (short) -9415);
            assert(pack.xtrack_error_GET() == -1.876497E38F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.target_bearing_SET((short) -9415) ;
        p62.wp_dist_SET((char)48848) ;
        p62.aspd_error_SET(-1.8988256E38F) ;
        p62.nav_pitch_SET(-2.9589928E38F) ;
        p62.nav_bearing_SET((short) -24723) ;
        p62.alt_error_SET(1.7954784E38F) ;
        p62.nav_roll_SET(1.5701561E37F) ;
        p62.xtrack_error_SET(-1.876497E38F) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == 2.2769256E38F);
            assert(pack.lon_GET() == -140762999);
            assert(pack.time_usec_GET() == 8375778278441085387L);
            assert(pack.vx_GET() == -1.808097E37F);
            assert(pack.vz_GET() == 3.3822591E38F);
            assert(pack.relative_alt_GET() == 1368375072);
            assert(pack.lat_GET() == 945019465);
            assert(pack.alt_GET() == 1641017963);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.1135683E38F, 3.3177246E36F, -1.9286484E38F, -1.0925061E38F, 9.5145626E36F, -9.408132E37F, -3.379621E38F, 3.0364641E38F, -3.3099868E38F, -2.326454E38F, -4.268579E37F, 2.0381941E38F, -2.807289E38F, 1.1626005E38F, -1.0245876E38F, -5.512513E37F, 1.7277358E38F, -1.1937772E38F, -3.3091595E38F, 1.192E38F, -1.7095422E38F, -9.83237E37F, 2.2613796E38F, 1.3545951E38F, -2.6326177E37F, 9.073216E37F, -1.4827112E38F, 8.931025E37F, -2.8486259E38F, -1.1174474E38F, -3.1218293E38F, -2.4585065E38F, -7.3307484E37F, 1.9310043E37F, 1.3736792E38F, -2.0791581E38F}));
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.relative_alt_SET(1368375072) ;
        p63.time_usec_SET(8375778278441085387L) ;
        p63.vx_SET(-1.808097E37F) ;
        p63.covariance_SET(new float[] {-1.1135683E38F, 3.3177246E36F, -1.9286484E38F, -1.0925061E38F, 9.5145626E36F, -9.408132E37F, -3.379621E38F, 3.0364641E38F, -3.3099868E38F, -2.326454E38F, -4.268579E37F, 2.0381941E38F, -2.807289E38F, 1.1626005E38F, -1.0245876E38F, -5.512513E37F, 1.7277358E38F, -1.1937772E38F, -3.3091595E38F, 1.192E38F, -1.7095422E38F, -9.83237E37F, 2.2613796E38F, 1.3545951E38F, -2.6326177E37F, 9.073216E37F, -1.4827112E38F, 8.931025E37F, -2.8486259E38F, -1.1174474E38F, -3.1218293E38F, -2.4585065E38F, -7.3307484E37F, 1.9310043E37F, 1.3736792E38F, -2.0791581E38F}, 0) ;
        p63.vz_SET(3.3822591E38F) ;
        p63.vy_SET(2.2769256E38F) ;
        p63.alt_SET(1641017963) ;
        p63.lat_SET(945019465) ;
        p63.lon_SET(-140762999) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -9.50816E37F);
            assert(pack.vy_GET() == 3.2104101E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.9232067E38F, -2.6520591E38F, 7.181877E37F, 3.1394073E38F, 1.3909825E38F, -8.704397E37F, 1.5382682E38F, 2.94041E38F, -3.1247989E38F, 1.1783419E38F, 3.1865199E38F, 2.2071821E38F, -2.8102773E38F, -2.929252E38F, 4.4085785E37F, -1.5907697E38F, -9.67105E37F, 2.874617E37F, -3.2340827E38F, -4.3535482E36F, 3.646862E37F, -3.0214178E38F, 1.6003987E38F, -1.4680887E38F, -3.2574018E38F, 2.9578296E38F, 8.717256E37F, -3.9477145E37F, -1.9828281E37F, 2.3001638E38F, -1.9377822E38F, -3.3913592E38F, -3.0188728E38F, 1.4019617E38F, 3.1159503E38F, 7.1127875E37F, -2.0085924E37F, 2.2577784E38F, -2.3383827E38F, -1.5299449E38F, -1.0358102E38F, -7.2374904E37F, 3.1000238E37F, -2.1522197E37F, -1.2510593E38F}));
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
            assert(pack.x_GET() == -1.0795319E38F);
            assert(pack.ay_GET() == -6.0928683E37F);
            assert(pack.y_GET() == -2.733653E38F);
            assert(pack.time_usec_GET() == 3769113456725606070L);
            assert(pack.vz_GET() == 2.9270831E38F);
            assert(pack.ax_GET() == -1.2838791E38F);
            assert(pack.az_GET() == 2.1260835E38F);
            assert(pack.vx_GET() == -6.5040247E37F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.ax_SET(-1.2838791E38F) ;
        p64.x_SET(-1.0795319E38F) ;
        p64.y_SET(-2.733653E38F) ;
        p64.ay_SET(-6.0928683E37F) ;
        p64.az_SET(2.1260835E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS) ;
        p64.vx_SET(-6.5040247E37F) ;
        p64.time_usec_SET(3769113456725606070L) ;
        p64.vz_SET(2.9270831E38F) ;
        p64.z_SET(-9.50816E37F) ;
        p64.covariance_SET(new float[] {-2.9232067E38F, -2.6520591E38F, 7.181877E37F, 3.1394073E38F, 1.3909825E38F, -8.704397E37F, 1.5382682E38F, 2.94041E38F, -3.1247989E38F, 1.1783419E38F, 3.1865199E38F, 2.2071821E38F, -2.8102773E38F, -2.929252E38F, 4.4085785E37F, -1.5907697E38F, -9.67105E37F, 2.874617E37F, -3.2340827E38F, -4.3535482E36F, 3.646862E37F, -3.0214178E38F, 1.6003987E38F, -1.4680887E38F, -3.2574018E38F, 2.9578296E38F, 8.717256E37F, -3.9477145E37F, -1.9828281E37F, 2.3001638E38F, -1.9377822E38F, -3.3913592E38F, -3.0188728E38F, 1.4019617E38F, 3.1159503E38F, 7.1127875E37F, -2.0085924E37F, 2.2577784E38F, -2.3383827E38F, -1.5299449E38F, -1.0358102E38F, -7.2374904E37F, 3.1000238E37F, -2.1522197E37F, -1.2510593E38F}, 0) ;
        p64.vy_SET(3.2104101E38F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan17_raw_GET() == (char)36127);
            assert(pack.chan1_raw_GET() == (char)52779);
            assert(pack.time_boot_ms_GET() == 241610168L);
            assert(pack.chan2_raw_GET() == (char)6211);
            assert(pack.chan11_raw_GET() == (char)15414);
            assert(pack.chancount_GET() == (char)51);
            assert(pack.chan16_raw_GET() == (char)58943);
            assert(pack.chan13_raw_GET() == (char)46915);
            assert(pack.chan6_raw_GET() == (char)46289);
            assert(pack.chan3_raw_GET() == (char)11216);
            assert(pack.chan18_raw_GET() == (char)36802);
            assert(pack.chan14_raw_GET() == (char)37881);
            assert(pack.chan12_raw_GET() == (char)38215);
            assert(pack.chan15_raw_GET() == (char)22430);
            assert(pack.chan4_raw_GET() == (char)27532);
            assert(pack.chan5_raw_GET() == (char)19111);
            assert(pack.chan10_raw_GET() == (char)26965);
            assert(pack.chan9_raw_GET() == (char)7294);
            assert(pack.chan7_raw_GET() == (char)3143);
            assert(pack.rssi_GET() == (char)86);
            assert(pack.chan8_raw_GET() == (char)4076);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan14_raw_SET((char)37881) ;
        p65.time_boot_ms_SET(241610168L) ;
        p65.chan13_raw_SET((char)46915) ;
        p65.chan2_raw_SET((char)6211) ;
        p65.chan7_raw_SET((char)3143) ;
        p65.chan4_raw_SET((char)27532) ;
        p65.chan6_raw_SET((char)46289) ;
        p65.chan8_raw_SET((char)4076) ;
        p65.chan18_raw_SET((char)36802) ;
        p65.chancount_SET((char)51) ;
        p65.chan11_raw_SET((char)15414) ;
        p65.chan9_raw_SET((char)7294) ;
        p65.chan17_raw_SET((char)36127) ;
        p65.chan1_raw_SET((char)52779) ;
        p65.chan10_raw_SET((char)26965) ;
        p65.chan15_raw_SET((char)22430) ;
        p65.chan16_raw_SET((char)58943) ;
        p65.rssi_SET((char)86) ;
        p65.chan5_raw_SET((char)19111) ;
        p65.chan3_raw_SET((char)11216) ;
        p65.chan12_raw_SET((char)38215) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.start_stop_GET() == (char)182);
            assert(pack.target_system_GET() == (char)25);
            assert(pack.req_stream_id_GET() == (char)118);
            assert(pack.target_component_GET() == (char)125);
            assert(pack.req_message_rate_GET() == (char)54956);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_system_SET((char)25) ;
        p66.target_component_SET((char)125) ;
        p66.req_message_rate_SET((char)54956) ;
        p66.req_stream_id_SET((char)118) ;
        p66.start_stop_SET((char)182) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.stream_id_GET() == (char)225);
            assert(pack.message_rate_GET() == (char)55364);
            assert(pack.on_off_GET() == (char)120);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.stream_id_SET((char)225) ;
        p67.message_rate_SET((char)55364) ;
        p67.on_off_SET((char)120) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.r_GET() == (short)7206);
            assert(pack.y_GET() == (short)16071);
            assert(pack.x_GET() == (short) -663);
            assert(pack.z_GET() == (short) -32182);
            assert(pack.target_GET() == (char)130);
            assert(pack.buttons_GET() == (char)52332);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.buttons_SET((char)52332) ;
        p69.target_SET((char)130) ;
        p69.y_SET((short)16071) ;
        p69.r_SET((short)7206) ;
        p69.x_SET((short) -663) ;
        p69.z_SET((short) -32182) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan7_raw_GET() == (char)2036);
            assert(pack.target_component_GET() == (char)172);
            assert(pack.chan5_raw_GET() == (char)4787);
            assert(pack.chan8_raw_GET() == (char)56970);
            assert(pack.chan3_raw_GET() == (char)63875);
            assert(pack.chan6_raw_GET() == (char)263);
            assert(pack.chan1_raw_GET() == (char)47166);
            assert(pack.chan4_raw_GET() == (char)34481);
            assert(pack.chan2_raw_GET() == (char)65169);
            assert(pack.target_system_GET() == (char)24);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan2_raw_SET((char)65169) ;
        p70.target_system_SET((char)24) ;
        p70.chan6_raw_SET((char)263) ;
        p70.chan5_raw_SET((char)4787) ;
        p70.target_component_SET((char)172) ;
        p70.chan1_raw_SET((char)47166) ;
        p70.chan8_raw_SET((char)56970) ;
        p70.chan7_raw_SET((char)2036) ;
        p70.chan4_raw_SET((char)34481) ;
        p70.chan3_raw_SET((char)63875) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST);
            assert(pack.x_GET() == 140397514);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.param2_GET() == 3.078928E38F);
            assert(pack.current_GET() == (char)207);
            assert(pack.target_component_GET() == (char)243);
            assert(pack.seq_GET() == (char)30201);
            assert(pack.target_system_GET() == (char)150);
            assert(pack.param1_GET() == 2.9332084E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.param3_GET() == -8.2405483E37F);
            assert(pack.y_GET() == 296444549);
            assert(pack.z_GET() == 1.1554946E38F);
            assert(pack.autocontinue_GET() == (char)180);
            assert(pack.param4_GET() == -7.582394E37F);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.param2_SET(3.078928E38F) ;
        p73.autocontinue_SET((char)180) ;
        p73.y_SET(296444549) ;
        p73.seq_SET((char)30201) ;
        p73.current_SET((char)207) ;
        p73.command_SET(MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST) ;
        p73.x_SET(140397514) ;
        p73.param1_SET(2.9332084E38F) ;
        p73.target_system_SET((char)150) ;
        p73.param4_SET(-7.582394E37F) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p73.target_component_SET((char)243) ;
        p73.param3_SET(-8.2405483E37F) ;
        p73.z_SET(1.1554946E38F) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 1.5642419E38F);
            assert(pack.climb_GET() == -2.9512022E38F);
            assert(pack.airspeed_GET() == -3.253796E38F);
            assert(pack.groundspeed_GET() == 4.584743E37F);
            assert(pack.heading_GET() == (short) -26483);
            assert(pack.throttle_GET() == (char)56556);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.throttle_SET((char)56556) ;
        p74.groundspeed_SET(4.584743E37F) ;
        p74.heading_SET((short) -26483) ;
        p74.airspeed_SET(-3.253796E38F) ;
        p74.alt_SET(1.5642419E38F) ;
        p74.climb_SET(-2.9512022E38F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.param2_GET() == 1.4977449E38F);
            assert(pack.autocontinue_GET() == (char)59);
            assert(pack.z_GET() == 2.8724945E37F);
            assert(pack.param3_GET() == 5.4422735E37F);
            assert(pack.target_system_GET() == (char)205);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT);
            assert(pack.x_GET() == 360867279);
            assert(pack.param1_GET() == 3.1254577E38F);
            assert(pack.target_component_GET() == (char)17);
            assert(pack.y_GET() == 1597646739);
            assert(pack.param4_GET() == -4.5510563E37F);
            assert(pack.current_GET() == (char)1);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.param2_SET(1.4977449E38F) ;
        p75.autocontinue_SET((char)59) ;
        p75.current_SET((char)1) ;
        p75.target_component_SET((char)17) ;
        p75.target_system_SET((char)205) ;
        p75.command_SET(MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT) ;
        p75.z_SET(2.8724945E37F) ;
        p75.y_SET(1597646739) ;
        p75.param3_SET(5.4422735E37F) ;
        p75.x_SET(360867279) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p75.param4_SET(-4.5510563E37F) ;
        p75.param1_SET(3.1254577E38F) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param1_GET() == 2.990126E38F);
            assert(pack.param2_GET() == 1.9608914E38F);
            assert(pack.param5_GET() == -9.0013524E36F);
            assert(pack.target_system_GET() == (char)72);
            assert(pack.param6_GET() == 2.0992401E38F);
            assert(pack.target_component_GET() == (char)132);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE);
            assert(pack.param3_GET() == -2.8367272E38F);
            assert(pack.confirmation_GET() == (char)108);
            assert(pack.param4_GET() == -3.5993547E37F);
            assert(pack.param7_GET() == 3.1182339E38F);
        });
        COMMAND_LONG p76 = new COMMAND_LONG();
        PH.setPack(p76);
        p76.param6_SET(2.0992401E38F) ;
        p76.target_component_SET((char)132) ;
        p76.param2_SET(1.9608914E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE) ;
        p76.param1_SET(2.990126E38F) ;
        p76.confirmation_SET((char)108) ;
        p76.param5_SET(-9.0013524E36F) ;
        p76.param7_SET(3.1182339E38F) ;
        p76.target_system_SET((char)72) ;
        p76.param4_SET(-3.5993547E37F) ;
        p76.param3_SET(-2.8367272E38F) ;
        TestChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_TRY(ph) == (char)43);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_IN_PROGRESS);
            assert(pack.result_param2_TRY(ph) == 595346410);
            assert(pack.progress_TRY(ph) == (char)85);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_JUMP);
            assert(pack.target_system_TRY(ph) == (char)185);
        });
        COMMAND_ACK p77 = new COMMAND_ACK();
        PH.setPack(p77);
        p77.result_param2_SET(595346410, PH) ;
        p77.progress_SET((char)85, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_DO_JUMP) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_IN_PROGRESS) ;
        p77.target_system_SET((char)185, PH) ;
        p77.target_component_SET((char)43, PH) ;
        TestChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -1.6307299E38F);
            assert(pack.manual_override_switch_GET() == (char)33);
            assert(pack.thrust_GET() == -8.2602466E37F);
            assert(pack.pitch_GET() == 1.789508E38F);
            assert(pack.time_boot_ms_GET() == 1683561401L);
            assert(pack.mode_switch_GET() == (char)164);
            assert(pack.roll_GET() == -2.5883837E38F);
        });
        MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.time_boot_ms_SET(1683561401L) ;
        p81.roll_SET(-2.5883837E38F) ;
        p81.yaw_SET(-1.6307299E38F) ;
        p81.mode_switch_SET((char)164) ;
        p81.manual_override_switch_SET((char)33) ;
        p81.pitch_SET(1.789508E38F) ;
        p81.thrust_SET(-8.2602466E37F) ;
        TestChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.thrust_GET() == -2.2285796E38F);
            assert(pack.body_roll_rate_GET() == 3.1935245E37F);
            assert(pack.target_component_GET() == (char)77);
            assert(pack.target_system_GET() == (char)178);
            assert(pack.time_boot_ms_GET() == 32107870L);
            assert(pack.body_pitch_rate_GET() == 2.1912894E38F);
            assert(pack.body_yaw_rate_GET() == -2.6976526E38F);
            assert(pack.type_mask_GET() == (char)161);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.4975945E38F, 2.1680001E38F, -3.00345E38F, 5.0510735E37F}));
        });
        SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.body_yaw_rate_SET(-2.6976526E38F) ;
        p82.target_component_SET((char)77) ;
        p82.thrust_SET(-2.2285796E38F) ;
        p82.target_system_SET((char)178) ;
        p82.time_boot_ms_SET(32107870L) ;
        p82.q_SET(new float[] {1.4975945E38F, 2.1680001E38F, -3.00345E38F, 5.0510735E37F}, 0) ;
        p82.body_roll_rate_SET(3.1935245E37F) ;
        p82.body_pitch_rate_SET(2.1912894E38F) ;
        p82.type_mask_SET((char)161) ;
        TestChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)231);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.7304542E38F, 2.3682593E38F, 6.16505E37F, 1.05402225E37F}));
            assert(pack.body_roll_rate_GET() == 3.1392803E38F);
            assert(pack.body_yaw_rate_GET() == 1.9164066E38F);
            assert(pack.time_boot_ms_GET() == 3941194260L);
            assert(pack.thrust_GET() == 2.702161E38F);
            assert(pack.body_pitch_rate_GET() == -2.59612E38F);
        });
        ATTITUDE_TARGET p83 = new ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.body_yaw_rate_SET(1.9164066E38F) ;
        p83.type_mask_SET((char)231) ;
        p83.body_roll_rate_SET(3.1392803E38F) ;
        p83.q_SET(new float[] {1.7304542E38F, 2.3682593E38F, 6.16505E37F, 1.05402225E37F}, 0) ;
        p83.body_pitch_rate_SET(-2.59612E38F) ;
        p83.time_boot_ms_SET(3941194260L) ;
        p83.thrust_SET(2.702161E38F) ;
        TestChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == 1.648717E38F);
            assert(pack.yaw_rate_GET() == -3.3254264E38F);
            assert(pack.afz_GET() == -3.256353E38F);
            assert(pack.yaw_GET() == 2.8124415E38F);
            assert(pack.vx_GET() == -1.418223E38F);
            assert(pack.z_GET() == 2.6180358E38F);
            assert(pack.vz_GET() == -1.4664456E38F);
            assert(pack.afx_GET() == -1.767346E38F);
            assert(pack.y_GET() == -9.633178E37F);
            assert(pack.x_GET() == 3.1056357E38F);
            assert(pack.target_component_GET() == (char)191);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.type_mask_GET() == (char)7274);
            assert(pack.afy_GET() == 2.2964772E38F);
            assert(pack.time_boot_ms_GET() == 4274549153L);
            assert(pack.target_system_GET() == (char)217);
        });
        SET_POSITION_TARGET_LOCAL_NED p84 = new SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.y_SET(-9.633178E37F) ;
        p84.vy_SET(1.648717E38F) ;
        p84.yaw_SET(2.8124415E38F) ;
        p84.afz_SET(-3.256353E38F) ;
        p84.afy_SET(2.2964772E38F) ;
        p84.vz_SET(-1.4664456E38F) ;
        p84.type_mask_SET((char)7274) ;
        p84.afx_SET(-1.767346E38F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p84.x_SET(3.1056357E38F) ;
        p84.target_component_SET((char)191) ;
        p84.target_system_SET((char)217) ;
        p84.yaw_rate_SET(-3.3254264E38F) ;
        p84.vx_SET(-1.418223E38F) ;
        p84.time_boot_ms_SET(4274549153L) ;
        p84.z_SET(2.6180358E38F) ;
        TestChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)42920);
            assert(pack.yaw_rate_GET() == -1.3796589E38F);
            assert(pack.target_system_GET() == (char)139);
            assert(pack.afx_GET() == 3.146819E38F);
            assert(pack.afy_GET() == 7.553004E36F);
            assert(pack.lat_int_GET() == -1871083374);
            assert(pack.lon_int_GET() == 1895726620);
            assert(pack.alt_GET() == 2.986505E38F);
            assert(pack.vx_GET() == 2.3181056E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.vz_GET() == 1.2174645E38F);
            assert(pack.vy_GET() == -1.2674784E38F);
            assert(pack.afz_GET() == -3.0323799E38F);
            assert(pack.target_component_GET() == (char)97);
            assert(pack.time_boot_ms_GET() == 2568968354L);
            assert(pack.yaw_GET() == -1.5061137E38F);
        });
        SET_POSITION_TARGET_GLOBAL_INT p86 = new SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.vx_SET(2.3181056E38F) ;
        p86.lon_int_SET(1895726620) ;
        p86.alt_SET(2.986505E38F) ;
        p86.afz_SET(-3.0323799E38F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p86.vy_SET(-1.2674784E38F) ;
        p86.time_boot_ms_SET(2568968354L) ;
        p86.type_mask_SET((char)42920) ;
        p86.yaw_rate_SET(-1.3796589E38F) ;
        p86.yaw_SET(-1.5061137E38F) ;
        p86.target_system_SET((char)139) ;
        p86.vz_SET(1.2174645E38F) ;
        p86.lat_int_SET(-1871083374) ;
        p86.afy_SET(7.553004E36F) ;
        p86.afx_SET(3.146819E38F) ;
        p86.target_component_SET((char)97) ;
        TestChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.afx_GET() == 1.6219121E38F);
            assert(pack.time_boot_ms_GET() == 490311503L);
            assert(pack.alt_GET() == -1.8080073E38F);
            assert(pack.type_mask_GET() == (char)20962);
            assert(pack.vx_GET() == -2.1808505E38F);
            assert(pack.afz_GET() == 1.223066E38F);
            assert(pack.lon_int_GET() == 1238084363);
            assert(pack.vz_GET() == 3.1281244E38F);
            assert(pack.vy_GET() == -2.8792688E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.yaw_GET() == -2.7745322E38F);
            assert(pack.lat_int_GET() == -741122530);
            assert(pack.yaw_rate_GET() == -3.5103674E37F);
            assert(pack.afy_GET() == 2.6287897E38F);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.vy_SET(-2.8792688E38F) ;
        p87.time_boot_ms_SET(490311503L) ;
        p87.afy_SET(2.6287897E38F) ;
        p87.lat_int_SET(-741122530) ;
        p87.afx_SET(1.6219121E38F) ;
        p87.type_mask_SET((char)20962) ;
        p87.yaw_SET(-2.7745322E38F) ;
        p87.lon_int_SET(1238084363) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p87.yaw_rate_SET(-3.5103674E37F) ;
        p87.alt_SET(-1.8080073E38F) ;
        p87.vx_SET(-2.1808505E38F) ;
        p87.vz_SET(3.1281244E38F) ;
        p87.afz_SET(1.223066E38F) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -2.4608082E38F);
            assert(pack.x_GET() == -1.9354996E38F);
            assert(pack.pitch_GET() == -2.662502E37F);
            assert(pack.roll_GET() == 2.5944095E36F);
            assert(pack.time_boot_ms_GET() == 487745326L);
            assert(pack.y_GET() == -2.3409008E38F);
            assert(pack.z_GET() == 2.213054E38F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.x_SET(-1.9354996E38F) ;
        p89.pitch_SET(-2.662502E37F) ;
        p89.yaw_SET(-2.4608082E38F) ;
        p89.time_boot_ms_SET(487745326L) ;
        p89.roll_SET(2.5944095E36F) ;
        p89.z_SET(2.213054E38F) ;
        p89.y_SET(-2.3409008E38F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == (short)22884);
            assert(pack.rollspeed_GET() == -4.651923E36F);
            assert(pack.roll_GET() == -9.2406265E36F);
            assert(pack.yaw_GET() == -1.2358204E37F);
            assert(pack.vz_GET() == (short) -7432);
            assert(pack.lat_GET() == -2077607183);
            assert(pack.time_usec_GET() == 7800934025789355752L);
            assert(pack.vx_GET() == (short) -21469);
            assert(pack.yawspeed_GET() == -9.404318E37F);
            assert(pack.lon_GET() == -2126617553);
            assert(pack.pitchspeed_GET() == -7.5068747E37F);
            assert(pack.yacc_GET() == (short)2375);
            assert(pack.xacc_GET() == (short)13238);
            assert(pack.pitch_GET() == 2.5814577E38F);
            assert(pack.zacc_GET() == (short)19082);
            assert(pack.alt_GET() == 1849889803);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.xacc_SET((short)13238) ;
        p90.yacc_SET((short)2375) ;
        p90.yawspeed_SET(-9.404318E37F) ;
        p90.lon_SET(-2126617553) ;
        p90.pitchspeed_SET(-7.5068747E37F) ;
        p90.vy_SET((short)22884) ;
        p90.alt_SET(1849889803) ;
        p90.roll_SET(-9.2406265E36F) ;
        p90.time_usec_SET(7800934025789355752L) ;
        p90.pitch_SET(2.5814577E38F) ;
        p90.rollspeed_SET(-4.651923E36F) ;
        p90.zacc_SET((short)19082) ;
        p90.yaw_SET(-1.2358204E37F) ;
        p90.vx_SET((short) -21469) ;
        p90.vz_SET((short) -7432) ;
        p90.lat_SET(-2077607183) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7683371126510622846L);
            assert(pack.yaw_rudder_GET() == 2.1831314E38F);
            assert(pack.aux4_GET() == 3.261251E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_DISARMED);
            assert(pack.aux3_GET() == -9.743203E37F);
            assert(pack.throttle_GET() == 1.7930311E38F);
            assert(pack.roll_ailerons_GET() == 1.65148E38F);
            assert(pack.pitch_elevator_GET() == -1.7124114E38F);
            assert(pack.nav_mode_GET() == (char)42);
            assert(pack.aux2_GET() == 2.6132147E38F);
            assert(pack.aux1_GET() == -2.901646E38F);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.aux4_SET(3.261251E38F) ;
        p91.throttle_SET(1.7930311E38F) ;
        p91.aux1_SET(-2.901646E38F) ;
        p91.time_usec_SET(7683371126510622846L) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_TEST_DISARMED) ;
        p91.yaw_rudder_SET(2.1831314E38F) ;
        p91.roll_ailerons_SET(1.65148E38F) ;
        p91.aux2_SET(2.6132147E38F) ;
        p91.aux3_SET(-9.743203E37F) ;
        p91.pitch_elevator_SET(-1.7124114E38F) ;
        p91.nav_mode_SET((char)42) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan12_raw_GET() == (char)25308);
            assert(pack.chan11_raw_GET() == (char)13799);
            assert(pack.chan6_raw_GET() == (char)52015);
            assert(pack.rssi_GET() == (char)114);
            assert(pack.chan7_raw_GET() == (char)18912);
            assert(pack.chan5_raw_GET() == (char)22270);
            assert(pack.chan2_raw_GET() == (char)38378);
            assert(pack.chan8_raw_GET() == (char)42203);
            assert(pack.chan9_raw_GET() == (char)33264);
            assert(pack.chan1_raw_GET() == (char)63717);
            assert(pack.chan4_raw_GET() == (char)36759);
            assert(pack.time_usec_GET() == 5663484082250282111L);
            assert(pack.chan10_raw_GET() == (char)19744);
            assert(pack.chan3_raw_GET() == (char)37821);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan5_raw_SET((char)22270) ;
        p92.chan6_raw_SET((char)52015) ;
        p92.chan7_raw_SET((char)18912) ;
        p92.chan9_raw_SET((char)33264) ;
        p92.chan2_raw_SET((char)38378) ;
        p92.rssi_SET((char)114) ;
        p92.chan4_raw_SET((char)36759) ;
        p92.chan1_raw_SET((char)63717) ;
        p92.chan10_raw_SET((char)19744) ;
        p92.chan3_raw_SET((char)37821) ;
        p92.chan11_raw_SET((char)13799) ;
        p92.chan8_raw_SET((char)42203) ;
        p92.chan12_raw_SET((char)25308) ;
        p92.time_usec_SET(5663484082250282111L) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6748393921063559948L);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_ARMED);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {3.4694423E37F, 1.7924255E38F, 3.1320993E38F, 1.2414877E38F, -1.134961E38F, -1.6409431E38F, -2.854185E38F, 5.0638737E37F, 2.419815E38F, 1.7606413E38F, 2.196017E38F, 2.241134E38F, -5.4005095E37F, 5.209742E37F, -1.0780487E38F, 3.1640605E37F}));
            assert(pack.flags_GET() == 567286506247169338L);
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.flags_SET(567286506247169338L) ;
        p93.controls_SET(new float[] {3.4694423E37F, 1.7924255E38F, 3.1320993E38F, 1.2414877E38F, -1.134961E38F, -1.6409431E38F, -2.854185E38F, 5.0638737E37F, 2.419815E38F, 1.7606413E38F, 2.196017E38F, 2.241134E38F, -5.4005095E37F, 5.209742E37F, -1.0780487E38F, 3.1640605E37F}, 0) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_TEST_ARMED) ;
        p93.time_usec_SET(6748393921063559948L) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.quality_GET() == (char)22);
            assert(pack.flow_x_GET() == (short) -27138);
            assert(pack.sensor_id_GET() == (char)180);
            assert(pack.flow_rate_y_TRY(ph) == -8.442743E37F);
            assert(pack.time_usec_GET() == 8905422712100399556L);
            assert(pack.flow_comp_m_x_GET() == -3.0287137E37F);
            assert(pack.flow_y_GET() == (short)20664);
            assert(pack.flow_rate_x_TRY(ph) == -2.309089E36F);
            assert(pack.flow_comp_m_y_GET() == -3.750012E37F);
            assert(pack.ground_distance_GET() == -8.2718775E37F);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.sensor_id_SET((char)180) ;
        p100.flow_x_SET((short) -27138) ;
        p100.flow_rate_y_SET(-8.442743E37F, PH) ;
        p100.quality_SET((char)22) ;
        p100.ground_distance_SET(-8.2718775E37F) ;
        p100.flow_y_SET((short)20664) ;
        p100.flow_comp_m_y_SET(-3.750012E37F) ;
        p100.flow_rate_x_SET(-2.309089E36F, PH) ;
        p100.time_usec_SET(8905422712100399556L) ;
        p100.flow_comp_m_x_SET(-3.0287137E37F) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 6912708478980509121L);
            assert(pack.roll_GET() == 2.927836E38F);
            assert(pack.yaw_GET() == 7.2554654E36F);
            assert(pack.pitch_GET() == -2.7945911E38F);
            assert(pack.z_GET() == 1.1288899E38F);
            assert(pack.x_GET() == -1.0242487E38F);
            assert(pack.y_GET() == 2.6310916E38F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.roll_SET(2.927836E38F) ;
        p101.y_SET(2.6310916E38F) ;
        p101.z_SET(1.1288899E38F) ;
        p101.x_SET(-1.0242487E38F) ;
        p101.yaw_SET(7.2554654E36F) ;
        p101.pitch_SET(-2.7945911E38F) ;
        p101.usec_SET(6912708478980509121L) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 2.4705468E38F);
            assert(pack.pitch_GET() == -1.0669886E38F);
            assert(pack.z_GET() == 1.0276276E38F);
            assert(pack.roll_GET() == 1.6944403E38F);
            assert(pack.yaw_GET() == -1.2757717E38F);
            assert(pack.usec_GET() == 2063218788716071716L);
            assert(pack.y_GET() == 1.9028245E38F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.y_SET(1.9028245E38F) ;
        p102.usec_SET(2063218788716071716L) ;
        p102.x_SET(2.4705468E38F) ;
        p102.roll_SET(1.6944403E38F) ;
        p102.z_SET(1.0276276E38F) ;
        p102.pitch_SET(-1.0669886E38F) ;
        p102.yaw_SET(-1.2757717E38F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 900643118034126684L);
            assert(pack.z_GET() == 5.225601E37F);
            assert(pack.y_GET() == 3.175498E38F);
            assert(pack.x_GET() == -3.2119058E38F);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.z_SET(5.225601E37F) ;
        p103.x_SET(-3.2119058E38F) ;
        p103.usec_SET(900643118034126684L) ;
        p103.y_SET(3.175498E38F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 1.8512335E38F);
            assert(pack.roll_GET() == 2.8527507E38F);
            assert(pack.yaw_GET() == 1.854141E38F);
            assert(pack.usec_GET() == 6612127558540808001L);
            assert(pack.z_GET() == 6.4941963E37F);
            assert(pack.pitch_GET() == 7.762448E37F);
            assert(pack.y_GET() == -2.8489727E38F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.roll_SET(2.8527507E38F) ;
        p104.y_SET(-2.8489727E38F) ;
        p104.x_SET(1.8512335E38F) ;
        p104.pitch_SET(7.762448E37F) ;
        p104.z_SET(6.4941963E37F) ;
        p104.usec_SET(6612127558540808001L) ;
        p104.yaw_SET(1.854141E38F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.pressure_alt_GET() == -1.3706509E38F);
            assert(pack.yacc_GET() == 2.5599386E38F);
            assert(pack.xgyro_GET() == 6.293949E37F);
            assert(pack.xacc_GET() == -5.4300686E37F);
            assert(pack.ygyro_GET() == -2.853148E38F);
            assert(pack.zacc_GET() == 2.1703436E38F);
            assert(pack.temperature_GET() == -3.059424E38F);
            assert(pack.diff_pressure_GET() == -1.8102038E38F);
            assert(pack.xmag_GET() == -2.3607814E38F);
            assert(pack.fields_updated_GET() == (char)51404);
            assert(pack.zgyro_GET() == -1.0233518E38F);
            assert(pack.zmag_GET() == -1.7018827E36F);
            assert(pack.abs_pressure_GET() == -1.2344833E38F);
            assert(pack.ymag_GET() == 3.3336708E38F);
            assert(pack.time_usec_GET() == 5487715379563603151L);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.temperature_SET(-3.059424E38F) ;
        p105.zmag_SET(-1.7018827E36F) ;
        p105.diff_pressure_SET(-1.8102038E38F) ;
        p105.time_usec_SET(5487715379563603151L) ;
        p105.zacc_SET(2.1703436E38F) ;
        p105.pressure_alt_SET(-1.3706509E38F) ;
        p105.xmag_SET(-2.3607814E38F) ;
        p105.zgyro_SET(-1.0233518E38F) ;
        p105.ymag_SET(3.3336708E38F) ;
        p105.xacc_SET(-5.4300686E37F) ;
        p105.xgyro_SET(6.293949E37F) ;
        p105.abs_pressure_SET(-1.2344833E38F) ;
        p105.ygyro_SET(-2.853148E38F) ;
        p105.fields_updated_SET((char)51404) ;
        p105.yacc_SET(2.5599386E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.distance_GET() == -5.8626797E37F);
            assert(pack.integrated_x_GET() == 1.2768466E38F);
            assert(pack.integrated_zgyro_GET() == -1.7525698E38F);
            assert(pack.quality_GET() == (char)39);
            assert(pack.time_usec_GET() == 3649750680026658427L);
            assert(pack.integration_time_us_GET() == 2839531907L);
            assert(pack.temperature_GET() == (short) -6957);
            assert(pack.time_delta_distance_us_GET() == 1486454789L);
            assert(pack.integrated_ygyro_GET() == 1.3541138E38F);
            assert(pack.integrated_xgyro_GET() == -3.1421566E38F);
            assert(pack.integrated_y_GET() == 3.4017856E38F);
            assert(pack.sensor_id_GET() == (char)42);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integrated_zgyro_SET(-1.7525698E38F) ;
        p106.integrated_ygyro_SET(1.3541138E38F) ;
        p106.quality_SET((char)39) ;
        p106.sensor_id_SET((char)42) ;
        p106.time_usec_SET(3649750680026658427L) ;
        p106.distance_SET(-5.8626797E37F) ;
        p106.temperature_SET((short) -6957) ;
        p106.integrated_y_SET(3.4017856E38F) ;
        p106.integrated_xgyro_SET(-3.1421566E38F) ;
        p106.time_delta_distance_us_SET(1486454789L) ;
        p106.integration_time_us_SET(2839531907L) ;
        p106.integrated_x_SET(1.2768466E38F) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.diff_pressure_GET() == 3.2451876E38F);
            assert(pack.zgyro_GET() == 1.5113048E38F);
            assert(pack.xgyro_GET() == -8.318715E37F);
            assert(pack.xmag_GET() == 2.9868102E38F);
            assert(pack.ymag_GET() == -1.5473752E38F);
            assert(pack.pressure_alt_GET() == 3.0361743E37F);
            assert(pack.temperature_GET() == -1.5861282E38F);
            assert(pack.yacc_GET() == 2.8117655E38F);
            assert(pack.zacc_GET() == -1.3889885E38F);
            assert(pack.ygyro_GET() == 2.082332E38F);
            assert(pack.abs_pressure_GET() == -4.721473E37F);
            assert(pack.xacc_GET() == 6.983001E37F);
            assert(pack.time_usec_GET() == 5456076947626217875L);
            assert(pack.zmag_GET() == 1.116376E38F);
            assert(pack.fields_updated_GET() == 1352592742L);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.temperature_SET(-1.5861282E38F) ;
        p107.time_usec_SET(5456076947626217875L) ;
        p107.ymag_SET(-1.5473752E38F) ;
        p107.zgyro_SET(1.5113048E38F) ;
        p107.zmag_SET(1.116376E38F) ;
        p107.zacc_SET(-1.3889885E38F) ;
        p107.ygyro_SET(2.082332E38F) ;
        p107.xgyro_SET(-8.318715E37F) ;
        p107.xacc_SET(6.983001E37F) ;
        p107.diff_pressure_SET(3.2451876E38F) ;
        p107.fields_updated_SET(1352592742L) ;
        p107.yacc_SET(2.8117655E38F) ;
        p107.abs_pressure_SET(-4.721473E37F) ;
        p107.xmag_SET(2.9868102E38F) ;
        p107.pressure_alt_SET(3.0361743E37F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.q3_GET() == 2.3943689E38F);
            assert(pack.xacc_GET() == 2.7858457E38F);
            assert(pack.q4_GET() == -2.161569E38F);
            assert(pack.roll_GET() == 1.9114877E38F);
            assert(pack.std_dev_horz_GET() == -9.807269E37F);
            assert(pack.zgyro_GET() == 2.1688058E38F);
            assert(pack.vd_GET() == -2.3098026E38F);
            assert(pack.q1_GET() == -5.6273043E37F);
            assert(pack.alt_GET() == -8.750215E37F);
            assert(pack.q2_GET() == -3.0801482E38F);
            assert(pack.lat_GET() == 2.5774837E38F);
            assert(pack.yacc_GET() == 1.0426011E38F);
            assert(pack.std_dev_vert_GET() == -1.4041707E38F);
            assert(pack.yaw_GET() == -3.044471E38F);
            assert(pack.ve_GET() == 1.4775347E38F);
            assert(pack.xgyro_GET() == 2.3351952E38F);
            assert(pack.ygyro_GET() == -9.012235E37F);
            assert(pack.vn_GET() == -6.277104E37F);
            assert(pack.pitch_GET() == 1.6598359E38F);
            assert(pack.lon_GET() == 2.8402513E38F);
            assert(pack.zacc_GET() == -2.24704E38F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.zacc_SET(-2.24704E38F) ;
        p108.alt_SET(-8.750215E37F) ;
        p108.xacc_SET(2.7858457E38F) ;
        p108.yacc_SET(1.0426011E38F) ;
        p108.vd_SET(-2.3098026E38F) ;
        p108.std_dev_horz_SET(-9.807269E37F) ;
        p108.q1_SET(-5.6273043E37F) ;
        p108.q4_SET(-2.161569E38F) ;
        p108.zgyro_SET(2.1688058E38F) ;
        p108.roll_SET(1.9114877E38F) ;
        p108.lat_SET(2.5774837E38F) ;
        p108.std_dev_vert_SET(-1.4041707E38F) ;
        p108.ygyro_SET(-9.012235E37F) ;
        p108.vn_SET(-6.277104E37F) ;
        p108.xgyro_SET(2.3351952E38F) ;
        p108.yaw_SET(-3.044471E38F) ;
        p108.ve_SET(1.4775347E38F) ;
        p108.q3_SET(2.3943689E38F) ;
        p108.q2_SET(-3.0801482E38F) ;
        p108.pitch_SET(1.6598359E38F) ;
        p108.lon_SET(2.8402513E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.fixed__GET() == (char)4950);
            assert(pack.txbuf_GET() == (char)116);
            assert(pack.remnoise_GET() == (char)161);
            assert(pack.rxerrors_GET() == (char)28720);
            assert(pack.remrssi_GET() == (char)217);
            assert(pack.noise_GET() == (char)112);
            assert(pack.rssi_GET() == (char)190);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.noise_SET((char)112) ;
        p109.remrssi_SET((char)217) ;
        p109.rxerrors_SET((char)28720) ;
        p109.fixed__SET((char)4950) ;
        p109.rssi_SET((char)190) ;
        p109.txbuf_SET((char)116) ;
        p109.remnoise_SET((char)161) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)221);
            assert(pack.target_network_GET() == (char)218);
            assert(pack.target_component_GET() == (char)220);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)46, (char)206, (char)23, (char)233, (char)64, (char)78, (char)119, (char)213, (char)132, (char)160, (char)224, (char)158, (char)29, (char)222, (char)61, (char)148, (char)112, (char)83, (char)90, (char)168, (char)252, (char)45, (char)152, (char)132, (char)218, (char)73, (char)157, (char)69, (char)151, (char)210, (char)72, (char)242, (char)181, (char)119, (char)108, (char)187, (char)99, (char)203, (char)216, (char)30, (char)205, (char)187, (char)131, (char)229, (char)169, (char)56, (char)6, (char)210, (char)124, (char)60, (char)114, (char)98, (char)245, (char)153, (char)98, (char)198, (char)146, (char)243, (char)79, (char)122, (char)111, (char)182, (char)83, (char)226, (char)112, (char)90, (char)207, (char)60, (char)51, (char)208, (char)51, (char)180, (char)48, (char)126, (char)63, (char)186, (char)65, (char)210, (char)140, (char)153, (char)160, (char)194, (char)143, (char)116, (char)206, (char)238, (char)40, (char)75, (char)181, (char)7, (char)126, (char)228, (char)30, (char)90, (char)135, (char)51, (char)44, (char)40, (char)140, (char)227, (char)40, (char)129, (char)110, (char)28, (char)75, (char)111, (char)193, (char)93, (char)79, (char)140, (char)216, (char)216, (char)26, (char)62, (char)231, (char)100, (char)80, (char)148, (char)238, (char)194, (char)11, (char)58, (char)158, (char)25, (char)179, (char)191, (char)36, (char)89, (char)10, (char)233, (char)170, (char)43, (char)185, (char)199, (char)124, (char)146, (char)97, (char)37, (char)149, (char)136, (char)43, (char)254, (char)122, (char)133, (char)94, (char)72, (char)116, (char)43, (char)188, (char)209, (char)100, (char)208, (char)11, (char)94, (char)166, (char)226, (char)126, (char)179, (char)92, (char)44, (char)171, (char)152, (char)10, (char)9, (char)108, (char)162, (char)12, (char)223, (char)208, (char)200, (char)169, (char)131, (char)39, (char)158, (char)231, (char)42, (char)29, (char)225, (char)102, (char)72, (char)233, (char)75, (char)79, (char)208, (char)141, (char)96, (char)44, (char)60, (char)109, (char)238, (char)140, (char)6, (char)176, (char)171, (char)2, (char)192, (char)119, (char)86, (char)162, (char)87, (char)95, (char)10, (char)235, (char)44, (char)96, (char)233, (char)228, (char)69, (char)21, (char)171, (char)22, (char)171, (char)221, (char)83, (char)8, (char)71, (char)176, (char)177, (char)187, (char)57, (char)163, (char)184, (char)146, (char)228, (char)110, (char)50, (char)79, (char)161, (char)212, (char)230, (char)127, (char)192, (char)97, (char)74, (char)71, (char)103, (char)131, (char)145, (char)141, (char)6, (char)69, (char)175, (char)29, (char)204, (char)255, (char)56, (char)179, (char)171, (char)174, (char)124, (char)147}));
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_system_SET((char)221) ;
        p110.target_network_SET((char)218) ;
        p110.target_component_SET((char)220) ;
        p110.payload_SET(new char[] {(char)46, (char)206, (char)23, (char)233, (char)64, (char)78, (char)119, (char)213, (char)132, (char)160, (char)224, (char)158, (char)29, (char)222, (char)61, (char)148, (char)112, (char)83, (char)90, (char)168, (char)252, (char)45, (char)152, (char)132, (char)218, (char)73, (char)157, (char)69, (char)151, (char)210, (char)72, (char)242, (char)181, (char)119, (char)108, (char)187, (char)99, (char)203, (char)216, (char)30, (char)205, (char)187, (char)131, (char)229, (char)169, (char)56, (char)6, (char)210, (char)124, (char)60, (char)114, (char)98, (char)245, (char)153, (char)98, (char)198, (char)146, (char)243, (char)79, (char)122, (char)111, (char)182, (char)83, (char)226, (char)112, (char)90, (char)207, (char)60, (char)51, (char)208, (char)51, (char)180, (char)48, (char)126, (char)63, (char)186, (char)65, (char)210, (char)140, (char)153, (char)160, (char)194, (char)143, (char)116, (char)206, (char)238, (char)40, (char)75, (char)181, (char)7, (char)126, (char)228, (char)30, (char)90, (char)135, (char)51, (char)44, (char)40, (char)140, (char)227, (char)40, (char)129, (char)110, (char)28, (char)75, (char)111, (char)193, (char)93, (char)79, (char)140, (char)216, (char)216, (char)26, (char)62, (char)231, (char)100, (char)80, (char)148, (char)238, (char)194, (char)11, (char)58, (char)158, (char)25, (char)179, (char)191, (char)36, (char)89, (char)10, (char)233, (char)170, (char)43, (char)185, (char)199, (char)124, (char)146, (char)97, (char)37, (char)149, (char)136, (char)43, (char)254, (char)122, (char)133, (char)94, (char)72, (char)116, (char)43, (char)188, (char)209, (char)100, (char)208, (char)11, (char)94, (char)166, (char)226, (char)126, (char)179, (char)92, (char)44, (char)171, (char)152, (char)10, (char)9, (char)108, (char)162, (char)12, (char)223, (char)208, (char)200, (char)169, (char)131, (char)39, (char)158, (char)231, (char)42, (char)29, (char)225, (char)102, (char)72, (char)233, (char)75, (char)79, (char)208, (char)141, (char)96, (char)44, (char)60, (char)109, (char)238, (char)140, (char)6, (char)176, (char)171, (char)2, (char)192, (char)119, (char)86, (char)162, (char)87, (char)95, (char)10, (char)235, (char)44, (char)96, (char)233, (char)228, (char)69, (char)21, (char)171, (char)22, (char)171, (char)221, (char)83, (char)8, (char)71, (char)176, (char)177, (char)187, (char)57, (char)163, (char)184, (char)146, (char)228, (char)110, (char)50, (char)79, (char)161, (char)212, (char)230, (char)127, (char)192, (char)97, (char)74, (char)71, (char)103, (char)131, (char)145, (char)141, (char)6, (char)69, (char)175, (char)29, (char)204, (char)255, (char)56, (char)179, (char)171, (char)174, (char)124, (char)147}, 0) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.ts1_GET() == 3872071450527626946L);
            assert(pack.tc1_GET() == 7212538119840243409L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(7212538119840243409L) ;
        p111.ts1_SET(3872071450527626946L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5660727223157052692L);
            assert(pack.seq_GET() == 1276629941L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(1276629941L) ;
        p112.time_usec_SET(5660727223157052692L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.fix_type_GET() == (char)6);
            assert(pack.time_usec_GET() == 3800316225227006257L);
            assert(pack.vd_GET() == (short) -10733);
            assert(pack.lon_GET() == 1647567601);
            assert(pack.ve_GET() == (short)19330);
            assert(pack.eph_GET() == (char)37207);
            assert(pack.vel_GET() == (char)21876);
            assert(pack.lat_GET() == 214679523);
            assert(pack.cog_GET() == (char)33128);
            assert(pack.alt_GET() == 1359773316);
            assert(pack.vn_GET() == (short)2621);
            assert(pack.epv_GET() == (char)22431);
            assert(pack.satellites_visible_GET() == (char)137);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.vn_SET((short)2621) ;
        p113.satellites_visible_SET((char)137) ;
        p113.epv_SET((char)22431) ;
        p113.lat_SET(214679523) ;
        p113.cog_SET((char)33128) ;
        p113.vel_SET((char)21876) ;
        p113.vd_SET((short) -10733) ;
        p113.ve_SET((short)19330) ;
        p113.fix_type_SET((char)6) ;
        p113.time_usec_SET(3800316225227006257L) ;
        p113.lon_SET(1647567601) ;
        p113.alt_SET(1359773316) ;
        p113.eph_SET((char)37207) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integration_time_us_GET() == 448772230L);
            assert(pack.integrated_zgyro_GET() == -1.9316196E38F);
            assert(pack.integrated_y_GET() == 4.5633556E37F);
            assert(pack.integrated_xgyro_GET() == 3.1968986E38F);
            assert(pack.time_delta_distance_us_GET() == 3073560150L);
            assert(pack.sensor_id_GET() == (char)196);
            assert(pack.integrated_x_GET() == 2.8813282E38F);
            assert(pack.temperature_GET() == (short) -5154);
            assert(pack.distance_GET() == 2.7369972E38F);
            assert(pack.time_usec_GET() == 5348573332496561187L);
            assert(pack.integrated_ygyro_GET() == 5.8230245E37F);
            assert(pack.quality_GET() == (char)56);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.integrated_ygyro_SET(5.8230245E37F) ;
        p114.time_delta_distance_us_SET(3073560150L) ;
        p114.sensor_id_SET((char)196) ;
        p114.distance_SET(2.7369972E38F) ;
        p114.integration_time_us_SET(448772230L) ;
        p114.quality_SET((char)56) ;
        p114.integrated_zgyro_SET(-1.9316196E38F) ;
        p114.integrated_y_SET(4.5633556E37F) ;
        p114.integrated_xgyro_SET(3.1968986E38F) ;
        p114.integrated_x_SET(2.8813282E38F) ;
        p114.time_usec_SET(5348573332496561187L) ;
        p114.temperature_SET((short) -5154) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == (short) -31560);
            assert(pack.vx_GET() == (short) -27498);
            assert(pack.time_usec_GET() == 5454255647696604777L);
            assert(pack.lon_GET() == -291430453);
            assert(pack.vz_GET() == (short) -17892);
            assert(pack.xacc_GET() == (short) -455);
            assert(pack.alt_GET() == -333954596);
            assert(pack.vy_GET() == (short) -21026);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {2.4059084E38F, -1.6816022E38F, 2.7139049E38F, 1.3952402E38F}));
            assert(pack.yawspeed_GET() == 2.191217E38F);
            assert(pack.yacc_GET() == (short) -8902);
            assert(pack.pitchspeed_GET() == -3.0149043E38F);
            assert(pack.rollspeed_GET() == -1.3059102E38F);
            assert(pack.true_airspeed_GET() == (char)28956);
            assert(pack.ind_airspeed_GET() == (char)16494);
            assert(pack.lat_GET() == -1926001069);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.xacc_SET((short) -455) ;
        p115.vx_SET((short) -27498) ;
        p115.lat_SET(-1926001069) ;
        p115.pitchspeed_SET(-3.0149043E38F) ;
        p115.zacc_SET((short) -31560) ;
        p115.rollspeed_SET(-1.3059102E38F) ;
        p115.vz_SET((short) -17892) ;
        p115.time_usec_SET(5454255647696604777L) ;
        p115.true_airspeed_SET((char)28956) ;
        p115.yawspeed_SET(2.191217E38F) ;
        p115.attitude_quaternion_SET(new float[] {2.4059084E38F, -1.6816022E38F, 2.7139049E38F, 1.3952402E38F}, 0) ;
        p115.ind_airspeed_SET((char)16494) ;
        p115.lon_SET(-291430453) ;
        p115.yacc_SET((short) -8902) ;
        p115.alt_SET(-333954596) ;
        p115.vy_SET((short) -21026) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.ymag_GET() == (short) -29976);
            assert(pack.zmag_GET() == (short) -32435);
            assert(pack.xacc_GET() == (short) -17828);
            assert(pack.zgyro_GET() == (short)26280);
            assert(pack.zacc_GET() == (short)11284);
            assert(pack.xmag_GET() == (short)15412);
            assert(pack.time_boot_ms_GET() == 2378665951L);
            assert(pack.yacc_GET() == (short) -7384);
            assert(pack.ygyro_GET() == (short)7794);
            assert(pack.xgyro_GET() == (short)32750);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.ygyro_SET((short)7794) ;
        p116.zmag_SET((short) -32435) ;
        p116.time_boot_ms_SET(2378665951L) ;
        p116.xgyro_SET((short)32750) ;
        p116.xmag_SET((short)15412) ;
        p116.zgyro_SET((short)26280) ;
        p116.ymag_SET((short) -29976) ;
        p116.yacc_SET((short) -7384) ;
        p116.xacc_SET((short) -17828) ;
        p116.zacc_SET((short)11284) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_GET() == (char)47157);
            assert(pack.target_component_GET() == (char)251);
            assert(pack.end_GET() == (char)2199);
            assert(pack.target_system_GET() == (char)137);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)137) ;
        p117.start_SET((char)47157) ;
        p117.end_SET((char)2199) ;
        p117.target_component_SET((char)251) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.size_GET() == 4228108593L);
            assert(pack.time_utc_GET() == 1030124031L);
            assert(pack.id_GET() == (char)10143);
            assert(pack.num_logs_GET() == (char)54846);
            assert(pack.last_log_num_GET() == (char)24396);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.last_log_num_SET((char)24396) ;
        p118.size_SET(4228108593L) ;
        p118.time_utc_SET(1030124031L) ;
        p118.num_logs_SET((char)54846) ;
        p118.id_SET((char)10143) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)124);
            assert(pack.count_GET() == 3648202226L);
            assert(pack.id_GET() == (char)32345);
            assert(pack.ofs_GET() == 3795048366L);
            assert(pack.target_system_GET() == (char)145);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)145) ;
        p119.id_SET((char)32345) ;
        p119.target_component_SET((char)124) ;
        p119.count_SET(3648202226L) ;
        p119.ofs_SET(3795048366L) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)118);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)201, (char)147, (char)94, (char)200, (char)218, (char)183, (char)166, (char)170, (char)169, (char)47, (char)122, (char)56, (char)251, (char)93, (char)208, (char)52, (char)151, (char)89, (char)4, (char)190, (char)8, (char)218, (char)189, (char)181, (char)85, (char)201, (char)137, (char)82, (char)79, (char)98, (char)62, (char)117, (char)157, (char)147, (char)58, (char)105, (char)235, (char)171, (char)243, (char)144, (char)92, (char)204, (char)160, (char)171, (char)20, (char)199, (char)144, (char)40, (char)200, (char)24, (char)37, (char)156, (char)100, (char)173, (char)183, (char)200, (char)226, (char)241, (char)31, (char)233, (char)181, (char)193, (char)85, (char)63, (char)123, (char)168, (char)205, (char)230, (char)198, (char)191, (char)13, (char)106, (char)100, (char)95, (char)5, (char)151, (char)11, (char)110, (char)119, (char)253, (char)252, (char)118, (char)99, (char)244, (char)76, (char)243, (char)114, (char)102, (char)14, (char)1}));
            assert(pack.ofs_GET() == 2685548805L);
            assert(pack.id_GET() == (char)54859);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)54859) ;
        p120.count_SET((char)118) ;
        p120.ofs_SET(2685548805L) ;
        p120.data__SET(new char[] {(char)201, (char)147, (char)94, (char)200, (char)218, (char)183, (char)166, (char)170, (char)169, (char)47, (char)122, (char)56, (char)251, (char)93, (char)208, (char)52, (char)151, (char)89, (char)4, (char)190, (char)8, (char)218, (char)189, (char)181, (char)85, (char)201, (char)137, (char)82, (char)79, (char)98, (char)62, (char)117, (char)157, (char)147, (char)58, (char)105, (char)235, (char)171, (char)243, (char)144, (char)92, (char)204, (char)160, (char)171, (char)20, (char)199, (char)144, (char)40, (char)200, (char)24, (char)37, (char)156, (char)100, (char)173, (char)183, (char)200, (char)226, (char)241, (char)31, (char)233, (char)181, (char)193, (char)85, (char)63, (char)123, (char)168, (char)205, (char)230, (char)198, (char)191, (char)13, (char)106, (char)100, (char)95, (char)5, (char)151, (char)11, (char)110, (char)119, (char)253, (char)252, (char)118, (char)99, (char)244, (char)76, (char)243, (char)114, (char)102, (char)14, (char)1}, 0) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)53);
            assert(pack.target_system_GET() == (char)219);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)219) ;
        p121.target_component_SET((char)53) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)218);
            assert(pack.target_system_GET() == (char)119);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)119) ;
        p122.target_component_SET((char)218) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)203);
            assert(pack.len_GET() == (char)178);
            assert(pack.target_system_GET() == (char)82);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)243, (char)144, (char)86, (char)247, (char)169, (char)26, (char)164, (char)216, (char)125, (char)179, (char)213, (char)4, (char)25, (char)177, (char)133, (char)235, (char)75, (char)153, (char)232, (char)12, (char)1, (char)173, (char)85, (char)102, (char)183, (char)151, (char)66, (char)9, (char)83, (char)19, (char)38, (char)91, (char)220, (char)87, (char)43, (char)26, (char)64, (char)192, (char)165, (char)154, (char)181, (char)101, (char)250, (char)198, (char)200, (char)172, (char)49, (char)146, (char)136, (char)232, (char)42, (char)207, (char)197, (char)112, (char)243, (char)66, (char)142, (char)78, (char)166, (char)196, (char)50, (char)61, (char)116, (char)232, (char)217, (char)221, (char)82, (char)219, (char)149, (char)61, (char)76, (char)21, (char)163, (char)92, (char)192, (char)24, (char)222, (char)228, (char)226, (char)65, (char)233, (char)79, (char)228, (char)231, (char)233, (char)162, (char)107, (char)187, (char)236, (char)188, (char)102, (char)77, (char)130, (char)76, (char)171, (char)55, (char)236, (char)124, (char)173, (char)134, (char)182, (char)123, (char)123, (char)136, (char)31, (char)102, (char)133, (char)95, (char)133, (char)155}));
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.data__SET(new char[] {(char)243, (char)144, (char)86, (char)247, (char)169, (char)26, (char)164, (char)216, (char)125, (char)179, (char)213, (char)4, (char)25, (char)177, (char)133, (char)235, (char)75, (char)153, (char)232, (char)12, (char)1, (char)173, (char)85, (char)102, (char)183, (char)151, (char)66, (char)9, (char)83, (char)19, (char)38, (char)91, (char)220, (char)87, (char)43, (char)26, (char)64, (char)192, (char)165, (char)154, (char)181, (char)101, (char)250, (char)198, (char)200, (char)172, (char)49, (char)146, (char)136, (char)232, (char)42, (char)207, (char)197, (char)112, (char)243, (char)66, (char)142, (char)78, (char)166, (char)196, (char)50, (char)61, (char)116, (char)232, (char)217, (char)221, (char)82, (char)219, (char)149, (char)61, (char)76, (char)21, (char)163, (char)92, (char)192, (char)24, (char)222, (char)228, (char)226, (char)65, (char)233, (char)79, (char)228, (char)231, (char)233, (char)162, (char)107, (char)187, (char)236, (char)188, (char)102, (char)77, (char)130, (char)76, (char)171, (char)55, (char)236, (char)124, (char)173, (char)134, (char)182, (char)123, (char)123, (char)136, (char)31, (char)102, (char)133, (char)95, (char)133, (char)155}, 0) ;
        p123.len_SET((char)178) ;
        p123.target_system_SET((char)82) ;
        p123.target_component_SET((char)203) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.eph_GET() == (char)4927);
            assert(pack.satellites_visible_GET() == (char)241);
            assert(pack.time_usec_GET() == 2126862005822432577L);
            assert(pack.vel_GET() == (char)18149);
            assert(pack.lat_GET() == -165967611);
            assert(pack.dgps_numch_GET() == (char)244);
            assert(pack.cog_GET() == (char)56880);
            assert(pack.dgps_age_GET() == 3156479430L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
            assert(pack.alt_GET() == 1135720059);
            assert(pack.epv_GET() == (char)65335);
            assert(pack.lon_GET() == -1045157719);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.vel_SET((char)18149) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS) ;
        p124.alt_SET(1135720059) ;
        p124.epv_SET((char)65335) ;
        p124.time_usec_SET(2126862005822432577L) ;
        p124.satellites_visible_SET((char)241) ;
        p124.dgps_age_SET(3156479430L) ;
        p124.eph_SET((char)4927) ;
        p124.cog_SET((char)56880) ;
        p124.lon_SET(-1045157719) ;
        p124.dgps_numch_SET((char)244) ;
        p124.lat_SET(-165967611) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vservo_GET() == (char)12541);
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT));
            assert(pack.Vcc_GET() == (char)54525);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vservo_SET((char)12541) ;
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT)) ;
        p125.Vcc_SET((char)54525) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.baudrate_GET() == 1215005543L);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1);
            assert(pack.timeout_GET() == (char)45437);
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING));
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)69, (char)218, (char)128, (char)98, (char)187, (char)243, (char)245, (char)246, (char)49, (char)132, (char)124, (char)231, (char)184, (char)173, (char)39, (char)150, (char)217, (char)31, (char)209, (char)193, (char)117, (char)214, (char)249, (char)105, (char)64, (char)132, (char)226, (char)131, (char)239, (char)128, (char)61, (char)21, (char)245, (char)108, (char)166, (char)74, (char)56, (char)17, (char)51, (char)139, (char)125, (char)13, (char)211, (char)170, (char)105, (char)228, (char)224, (char)247, (char)158, (char)176, (char)48, (char)202, (char)247, (char)188, (char)71, (char)70, (char)14, (char)182, (char)85, (char)215, (char)145, (char)225, (char)131, (char)126, (char)174, (char)100, (char)228, (char)42, (char)211, (char)141}));
            assert(pack.count_GET() == (char)221);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.timeout_SET((char)45437) ;
        p126.data__SET(new char[] {(char)69, (char)218, (char)128, (char)98, (char)187, (char)243, (char)245, (char)246, (char)49, (char)132, (char)124, (char)231, (char)184, (char)173, (char)39, (char)150, (char)217, (char)31, (char)209, (char)193, (char)117, (char)214, (char)249, (char)105, (char)64, (char)132, (char)226, (char)131, (char)239, (char)128, (char)61, (char)21, (char)245, (char)108, (char)166, (char)74, (char)56, (char)17, (char)51, (char)139, (char)125, (char)13, (char)211, (char)170, (char)105, (char)228, (char)224, (char)247, (char)158, (char)176, (char)48, (char)202, (char)247, (char)188, (char)71, (char)70, (char)14, (char)182, (char)85, (char)215, (char)145, (char)225, (char)131, (char)126, (char)174, (char)100, (char)228, (char)42, (char)211, (char)141}, 0) ;
        p126.count_SET((char)221) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1) ;
        p126.baudrate_SET(1215005543L) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING)) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.accuracy_GET() == 519103536L);
            assert(pack.tow_GET() == 2140406152L);
            assert(pack.baseline_c_mm_GET() == 1829180664);
            assert(pack.nsats_GET() == (char)66);
            assert(pack.iar_num_hypotheses_GET() == 170590173);
            assert(pack.wn_GET() == (char)63695);
            assert(pack.rtk_health_GET() == (char)250);
            assert(pack.time_last_baseline_ms_GET() == 2810283235L);
            assert(pack.rtk_rate_GET() == (char)139);
            assert(pack.baseline_coords_type_GET() == (char)235);
            assert(pack.baseline_b_mm_GET() == 1820139906);
            assert(pack.rtk_receiver_id_GET() == (char)102);
            assert(pack.baseline_a_mm_GET() == -1125435114);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.wn_SET((char)63695) ;
        p127.baseline_c_mm_SET(1829180664) ;
        p127.baseline_a_mm_SET(-1125435114) ;
        p127.rtk_health_SET((char)250) ;
        p127.accuracy_SET(519103536L) ;
        p127.iar_num_hypotheses_SET(170590173) ;
        p127.tow_SET(2140406152L) ;
        p127.rtk_rate_SET((char)139) ;
        p127.nsats_SET((char)66) ;
        p127.time_last_baseline_ms_SET(2810283235L) ;
        p127.baseline_coords_type_SET((char)235) ;
        p127.rtk_receiver_id_SET((char)102) ;
        p127.baseline_b_mm_SET(1820139906) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.accuracy_GET() == 1253610087L);
            assert(pack.baseline_c_mm_GET() == -1478161710);
            assert(pack.rtk_receiver_id_GET() == (char)222);
            assert(pack.rtk_health_GET() == (char)147);
            assert(pack.iar_num_hypotheses_GET() == 2125302919);
            assert(pack.baseline_b_mm_GET() == 1852388952);
            assert(pack.time_last_baseline_ms_GET() == 893753806L);
            assert(pack.baseline_coords_type_GET() == (char)195);
            assert(pack.nsats_GET() == (char)160);
            assert(pack.tow_GET() == 1694577649L);
            assert(pack.rtk_rate_GET() == (char)68);
            assert(pack.wn_GET() == (char)24257);
            assert(pack.baseline_a_mm_GET() == 776155090);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.rtk_health_SET((char)147) ;
        p128.rtk_rate_SET((char)68) ;
        p128.rtk_receiver_id_SET((char)222) ;
        p128.tow_SET(1694577649L) ;
        p128.wn_SET((char)24257) ;
        p128.baseline_b_mm_SET(1852388952) ;
        p128.baseline_c_mm_SET(-1478161710) ;
        p128.iar_num_hypotheses_SET(2125302919) ;
        p128.nsats_SET((char)160) ;
        p128.baseline_coords_type_SET((char)195) ;
        p128.baseline_a_mm_SET(776155090) ;
        p128.accuracy_SET(1253610087L) ;
        p128.time_last_baseline_ms_SET(893753806L) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short) -22809);
            assert(pack.xacc_GET() == (short)32456);
            assert(pack.xgyro_GET() == (short) -14799);
            assert(pack.time_boot_ms_GET() == 4246603241L);
            assert(pack.xmag_GET() == (short)12856);
            assert(pack.zgyro_GET() == (short) -298);
            assert(pack.zmag_GET() == (short) -24229);
            assert(pack.zacc_GET() == (short)27080);
            assert(pack.ygyro_GET() == (short) -26238);
            assert(pack.ymag_GET() == (short) -12864);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.ygyro_SET((short) -26238) ;
        p129.xgyro_SET((short) -14799) ;
        p129.time_boot_ms_SET(4246603241L) ;
        p129.ymag_SET((short) -12864) ;
        p129.xmag_SET((short)12856) ;
        p129.zgyro_SET((short) -298) ;
        p129.xacc_SET((short)32456) ;
        p129.zmag_SET((short) -24229) ;
        p129.yacc_SET((short) -22809) ;
        p129.zacc_SET((short)27080) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.jpg_quality_GET() == (char)136);
            assert(pack.payload_GET() == (char)87);
            assert(pack.size_GET() == 2286457177L);
            assert(pack.width_GET() == (char)36561);
            assert(pack.height_GET() == (char)1342);
            assert(pack.packets_GET() == (char)56255);
            assert(pack.type_GET() == (char)249);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.type_SET((char)249) ;
        p130.packets_SET((char)56255) ;
        p130.size_SET(2286457177L) ;
        p130.payload_SET((char)87) ;
        p130.width_SET((char)36561) ;
        p130.height_SET((char)1342) ;
        p130.jpg_quality_SET((char)136) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)79, (char)98, (char)251, (char)14, (char)80, (char)196, (char)11, (char)252, (char)219, (char)94, (char)13, (char)82, (char)250, (char)159, (char)110, (char)39, (char)155, (char)245, (char)210, (char)110, (char)91, (char)186, (char)215, (char)177, (char)138, (char)65, (char)198, (char)99, (char)168, (char)14, (char)19, (char)230, (char)209, (char)122, (char)195, (char)38, (char)241, (char)176, (char)30, (char)112, (char)143, (char)143, (char)224, (char)228, (char)239, (char)245, (char)89, (char)28, (char)61, (char)141, (char)107, (char)110, (char)215, (char)240, (char)2, (char)185, (char)50, (char)110, (char)97, (char)7, (char)194, (char)190, (char)172, (char)99, (char)46, (char)48, (char)244, (char)103, (char)175, (char)103, (char)254, (char)216, (char)240, (char)151, (char)53, (char)64, (char)2, (char)211, (char)6, (char)16, (char)191, (char)187, (char)248, (char)81, (char)127, (char)202, (char)68, (char)32, (char)178, (char)175, (char)145, (char)12, (char)10, (char)86, (char)235, (char)33, (char)11, (char)113, (char)39, (char)9, (char)66, (char)5, (char)225, (char)53, (char)148, (char)93, (char)195, (char)41, (char)188, (char)110, (char)27, (char)46, (char)11, (char)157, (char)245, (char)27, (char)3, (char)142, (char)166, (char)53, (char)51, (char)78, (char)216, (char)191, (char)12, (char)246, (char)201, (char)237, (char)159, (char)234, (char)167, (char)116, (char)50, (char)182, (char)18, (char)47, (char)71, (char)61, (char)194, (char)19, (char)234, (char)162, (char)244, (char)240, (char)72, (char)68, (char)60, (char)122, (char)88, (char)100, (char)116, (char)20, (char)70, (char)28, (char)208, (char)2, (char)115, (char)25, (char)69, (char)228, (char)180, (char)113, (char)41, (char)194, (char)216, (char)223, (char)201, (char)176, (char)131, (char)149, (char)207, (char)205, (char)69, (char)17, (char)230, (char)9, (char)150, (char)17, (char)88, (char)47, (char)7, (char)29, (char)135, (char)225, (char)12, (char)108, (char)36, (char)12, (char)20, (char)121, (char)204, (char)165, (char)226, (char)147, (char)197, (char)76, (char)80, (char)28, (char)249, (char)91, (char)12, (char)137, (char)137, (char)218, (char)233, (char)163, (char)100, (char)134, (char)44, (char)139, (char)185, (char)206, (char)4, (char)177, (char)78, (char)136, (char)10, (char)132, (char)211, (char)23, (char)165, (char)214, (char)102, (char)202, (char)180, (char)179, (char)70, (char)230, (char)84, (char)251, (char)26, (char)54, (char)189, (char)246, (char)86, (char)217, (char)10, (char)49, (char)176, (char)242, (char)18, (char)9, (char)151, (char)10, (char)65, (char)16, (char)41, (char)206, (char)19, (char)117, (char)83, (char)160, (char)165}));
            assert(pack.seqnr_GET() == (char)38160);
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.data__SET(new char[] {(char)79, (char)98, (char)251, (char)14, (char)80, (char)196, (char)11, (char)252, (char)219, (char)94, (char)13, (char)82, (char)250, (char)159, (char)110, (char)39, (char)155, (char)245, (char)210, (char)110, (char)91, (char)186, (char)215, (char)177, (char)138, (char)65, (char)198, (char)99, (char)168, (char)14, (char)19, (char)230, (char)209, (char)122, (char)195, (char)38, (char)241, (char)176, (char)30, (char)112, (char)143, (char)143, (char)224, (char)228, (char)239, (char)245, (char)89, (char)28, (char)61, (char)141, (char)107, (char)110, (char)215, (char)240, (char)2, (char)185, (char)50, (char)110, (char)97, (char)7, (char)194, (char)190, (char)172, (char)99, (char)46, (char)48, (char)244, (char)103, (char)175, (char)103, (char)254, (char)216, (char)240, (char)151, (char)53, (char)64, (char)2, (char)211, (char)6, (char)16, (char)191, (char)187, (char)248, (char)81, (char)127, (char)202, (char)68, (char)32, (char)178, (char)175, (char)145, (char)12, (char)10, (char)86, (char)235, (char)33, (char)11, (char)113, (char)39, (char)9, (char)66, (char)5, (char)225, (char)53, (char)148, (char)93, (char)195, (char)41, (char)188, (char)110, (char)27, (char)46, (char)11, (char)157, (char)245, (char)27, (char)3, (char)142, (char)166, (char)53, (char)51, (char)78, (char)216, (char)191, (char)12, (char)246, (char)201, (char)237, (char)159, (char)234, (char)167, (char)116, (char)50, (char)182, (char)18, (char)47, (char)71, (char)61, (char)194, (char)19, (char)234, (char)162, (char)244, (char)240, (char)72, (char)68, (char)60, (char)122, (char)88, (char)100, (char)116, (char)20, (char)70, (char)28, (char)208, (char)2, (char)115, (char)25, (char)69, (char)228, (char)180, (char)113, (char)41, (char)194, (char)216, (char)223, (char)201, (char)176, (char)131, (char)149, (char)207, (char)205, (char)69, (char)17, (char)230, (char)9, (char)150, (char)17, (char)88, (char)47, (char)7, (char)29, (char)135, (char)225, (char)12, (char)108, (char)36, (char)12, (char)20, (char)121, (char)204, (char)165, (char)226, (char)147, (char)197, (char)76, (char)80, (char)28, (char)249, (char)91, (char)12, (char)137, (char)137, (char)218, (char)233, (char)163, (char)100, (char)134, (char)44, (char)139, (char)185, (char)206, (char)4, (char)177, (char)78, (char)136, (char)10, (char)132, (char)211, (char)23, (char)165, (char)214, (char)102, (char)202, (char)180, (char)179, (char)70, (char)230, (char)84, (char)251, (char)26, (char)54, (char)189, (char)246, (char)86, (char)217, (char)10, (char)49, (char)176, (char)242, (char)18, (char)9, (char)151, (char)10, (char)65, (char)16, (char)41, (char)206, (char)19, (char)117, (char)83, (char)160, (char)165}, 0) ;
        p131.seqnr_SET((char)38160) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.current_distance_GET() == (char)18806);
            assert(pack.covariance_GET() == (char)9);
            assert(pack.id_GET() == (char)38);
            assert(pack.max_distance_GET() == (char)30313);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_270);
            assert(pack.min_distance_GET() == (char)222);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            assert(pack.time_boot_ms_GET() == 997277135L);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.current_distance_SET((char)18806) ;
        p132.min_distance_SET((char)222) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_270) ;
        p132.id_SET((char)38) ;
        p132.max_distance_SET((char)30313) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR) ;
        p132.covariance_SET((char)9) ;
        p132.time_boot_ms_SET(997277135L) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.grid_spacing_GET() == (char)40421);
            assert(pack.lat_GET() == 735075575);
            assert(pack.mask_GET() == 6267105474487740726L);
            assert(pack.lon_GET() == 1199065099);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.mask_SET(6267105474487740726L) ;
        p133.lat_SET(735075575) ;
        p133.grid_spacing_SET((char)40421) ;
        p133.lon_SET(1199065099) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.gridbit_GET() == (char)45);
            assert(pack.grid_spacing_GET() == (char)662);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short)16431, (short) -19939, (short)16291, (short)16718, (short)4329, (short)3559, (short)22529, (short)18323, (short)16825, (short)28041, (short) -32602, (short) -2262, (short) -28332, (short) -24994, (short)6267, (short)22153}));
            assert(pack.lat_GET() == 1449492974);
            assert(pack.lon_GET() == -1483236258);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.grid_spacing_SET((char)662) ;
        p134.lon_SET(-1483236258) ;
        p134.gridbit_SET((char)45) ;
        p134.data__SET(new short[] {(short)16431, (short) -19939, (short)16291, (short)16718, (short)4329, (short)3559, (short)22529, (short)18323, (short)16825, (short)28041, (short) -32602, (short) -2262, (short) -28332, (short) -24994, (short)6267, (short)22153}, 0) ;
        p134.lat_SET(1449492974) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 410593533);
            assert(pack.lat_GET() == 1046541957);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(1046541957) ;
        p135.lon_SET(410593533) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.spacing_GET() == (char)42769);
            assert(pack.pending_GET() == (char)49209);
            assert(pack.lon_GET() == 1051794631);
            assert(pack.lat_GET() == -777583354);
            assert(pack.terrain_height_GET() == -4.382652E36F);
            assert(pack.current_height_GET() == 4.81264E37F);
            assert(pack.loaded_GET() == (char)22484);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.loaded_SET((char)22484) ;
        p136.terrain_height_SET(-4.382652E36F) ;
        p136.spacing_SET((char)42769) ;
        p136.lon_SET(1051794631) ;
        p136.lat_SET(-777583354) ;
        p136.pending_SET((char)49209) ;
        p136.current_height_SET(4.81264E37F) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)1261);
            assert(pack.press_abs_GET() == -1.5922247E38F);
            assert(pack.time_boot_ms_GET() == 3786420555L);
            assert(pack.press_diff_GET() == -1.4592311E38F);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.press_diff_SET(-1.4592311E38F) ;
        p137.press_abs_SET(-1.5922247E38F) ;
        p137.time_boot_ms_SET(3786420555L) ;
        p137.temperature_SET((short)1261) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.760312E37F, -1.191208E38F, -2.202734E38F, -9.369341E37F}));
            assert(pack.time_usec_GET() == 5753651542017879269L);
            assert(pack.y_GET() == 2.209038E38F);
            assert(pack.x_GET() == 2.1584859E38F);
            assert(pack.z_GET() == -6.2652465E37F);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.y_SET(2.209038E38F) ;
        p138.time_usec_SET(5753651542017879269L) ;
        p138.z_SET(-6.2652465E37F) ;
        p138.x_SET(2.1584859E38F) ;
        p138.q_SET(new float[] {-3.760312E37F, -1.191208E38F, -2.202734E38F, -9.369341E37F}, 0) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)166);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {9.530802E37F, -9.436303E37F, 2.3465912E38F, 2.0071039E38F, -2.5156901E38F, -2.7738092E38F, 1.3872862E38F, 1.1231596E38F}));
            assert(pack.group_mlx_GET() == (char)109);
            assert(pack.target_system_GET() == (char)97);
            assert(pack.time_usec_GET() == 545893898614966656L);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.group_mlx_SET((char)109) ;
        p139.target_system_SET((char)97) ;
        p139.time_usec_SET(545893898614966656L) ;
        p139.target_component_SET((char)166) ;
        p139.controls_SET(new float[] {9.530802E37F, -9.436303E37F, 2.3465912E38F, 2.0071039E38F, -2.5156901E38F, -2.7738092E38F, 1.3872862E38F, 1.1231596E38F}, 0) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)221);
            assert(pack.time_usec_GET() == 2165341619232853394L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.8853622E38F, 3.8413426E37F, -7.799322E37F, -1.5579942E38F, 2.2463101E38F, 1.2896098E38F, -2.6791268E38F, 1.9440499E38F}));
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.controls_SET(new float[] {2.8853622E38F, 3.8413426E37F, -7.799322E37F, -1.5579942E38F, 2.2463101E38F, 1.2896098E38F, -2.6791268E38F, 1.9440499E38F}, 0) ;
        p140.time_usec_SET(2165341619232853394L) ;
        p140.group_mlx_SET((char)221) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_monotonic_GET() == -8.195999E37F);
            assert(pack.altitude_local_GET() == 7.5701016E37F);
            assert(pack.altitude_amsl_GET() == 1.0952809E38F);
            assert(pack.altitude_terrain_GET() == -3.6797978E37F);
            assert(pack.time_usec_GET() == 3776727383645253863L);
            assert(pack.altitude_relative_GET() == 1.6430249E38F);
            assert(pack.bottom_clearance_GET() == 1.0833833E38F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.time_usec_SET(3776727383645253863L) ;
        p141.altitude_local_SET(7.5701016E37F) ;
        p141.altitude_monotonic_SET(-8.195999E37F) ;
        p141.bottom_clearance_SET(1.0833833E38F) ;
        p141.altitude_amsl_SET(1.0952809E38F) ;
        p141.altitude_relative_SET(1.6430249E38F) ;
        p141.altitude_terrain_SET(-3.6797978E37F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.transfer_type_GET() == (char)178);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)151, (char)6, (char)61, (char)209, (char)69, (char)109, (char)158, (char)58, (char)141, (char)96, (char)162, (char)45, (char)211, (char)108, (char)173, (char)148, (char)81, (char)82, (char)120, (char)215, (char)120, (char)45, (char)37, (char)234, (char)85, (char)177, (char)54, (char)70, (char)65, (char)194, (char)54, (char)37, (char)202, (char)98, (char)66, (char)146, (char)209, (char)189, (char)231, (char)202, (char)59, (char)97, (char)17, (char)242, (char)2, (char)183, (char)209, (char)5, (char)20, (char)75, (char)242, (char)47, (char)149, (char)74, (char)61, (char)239, (char)45, (char)192, (char)212, (char)13, (char)138, (char)187, (char)144, (char)130, (char)42, (char)253, (char)62, (char)154, (char)123, (char)207, (char)72, (char)150, (char)218, (char)73, (char)147, (char)33, (char)158, (char)204, (char)194, (char)16, (char)24, (char)111, (char)115, (char)189, (char)102, (char)86, (char)212, (char)236, (char)41, (char)216, (char)97, (char)2, (char)255, (char)42, (char)198, (char)36, (char)45, (char)211, (char)16, (char)79, (char)197, (char)57, (char)131, (char)63, (char)32, (char)121, (char)36, (char)218, (char)159, (char)150, (char)250, (char)17, (char)121, (char)91, (char)93, (char)48, (char)89, (char)10, (char)0, (char)126}));
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)217, (char)46, (char)29, (char)75, (char)241, (char)25, (char)182, (char)25, (char)29, (char)31, (char)39, (char)116, (char)114, (char)62, (char)164, (char)217, (char)68, (char)183, (char)175, (char)51, (char)134, (char)99, (char)116, (char)132, (char)224, (char)178, (char)27, (char)210, (char)59, (char)146, (char)155, (char)204, (char)35, (char)174, (char)43, (char)140, (char)238, (char)120, (char)113, (char)109, (char)114, (char)242, (char)25, (char)178, (char)92, (char)42, (char)255, (char)30, (char)222, (char)64, (char)23, (char)202, (char)61, (char)211, (char)202, (char)82, (char)74, (char)120, (char)99, (char)188, (char)254, (char)77, (char)225, (char)205, (char)149, (char)53, (char)251, (char)70, (char)8, (char)99, (char)215, (char)107, (char)168, (char)175, (char)65, (char)32, (char)168, (char)40, (char)181, (char)202, (char)129, (char)132, (char)85, (char)235, (char)59, (char)150, (char)129, (char)21, (char)211, (char)220, (char)35, (char)99, (char)158, (char)255, (char)231, (char)72, (char)75, (char)124, (char)182, (char)146, (char)175, (char)22, (char)91, (char)214, (char)153, (char)30, (char)22, (char)85, (char)146, (char)246, (char)246, (char)115, (char)194, (char)37, (char)180, (char)82, (char)126, (char)205, (char)249, (char)130}));
            assert(pack.uri_type_GET() == (char)29);
            assert(pack.request_id_GET() == (char)217);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.uri_SET(new char[] {(char)217, (char)46, (char)29, (char)75, (char)241, (char)25, (char)182, (char)25, (char)29, (char)31, (char)39, (char)116, (char)114, (char)62, (char)164, (char)217, (char)68, (char)183, (char)175, (char)51, (char)134, (char)99, (char)116, (char)132, (char)224, (char)178, (char)27, (char)210, (char)59, (char)146, (char)155, (char)204, (char)35, (char)174, (char)43, (char)140, (char)238, (char)120, (char)113, (char)109, (char)114, (char)242, (char)25, (char)178, (char)92, (char)42, (char)255, (char)30, (char)222, (char)64, (char)23, (char)202, (char)61, (char)211, (char)202, (char)82, (char)74, (char)120, (char)99, (char)188, (char)254, (char)77, (char)225, (char)205, (char)149, (char)53, (char)251, (char)70, (char)8, (char)99, (char)215, (char)107, (char)168, (char)175, (char)65, (char)32, (char)168, (char)40, (char)181, (char)202, (char)129, (char)132, (char)85, (char)235, (char)59, (char)150, (char)129, (char)21, (char)211, (char)220, (char)35, (char)99, (char)158, (char)255, (char)231, (char)72, (char)75, (char)124, (char)182, (char)146, (char)175, (char)22, (char)91, (char)214, (char)153, (char)30, (char)22, (char)85, (char)146, (char)246, (char)246, (char)115, (char)194, (char)37, (char)180, (char)82, (char)126, (char)205, (char)249, (char)130}, 0) ;
        p142.storage_SET(new char[] {(char)151, (char)6, (char)61, (char)209, (char)69, (char)109, (char)158, (char)58, (char)141, (char)96, (char)162, (char)45, (char)211, (char)108, (char)173, (char)148, (char)81, (char)82, (char)120, (char)215, (char)120, (char)45, (char)37, (char)234, (char)85, (char)177, (char)54, (char)70, (char)65, (char)194, (char)54, (char)37, (char)202, (char)98, (char)66, (char)146, (char)209, (char)189, (char)231, (char)202, (char)59, (char)97, (char)17, (char)242, (char)2, (char)183, (char)209, (char)5, (char)20, (char)75, (char)242, (char)47, (char)149, (char)74, (char)61, (char)239, (char)45, (char)192, (char)212, (char)13, (char)138, (char)187, (char)144, (char)130, (char)42, (char)253, (char)62, (char)154, (char)123, (char)207, (char)72, (char)150, (char)218, (char)73, (char)147, (char)33, (char)158, (char)204, (char)194, (char)16, (char)24, (char)111, (char)115, (char)189, (char)102, (char)86, (char)212, (char)236, (char)41, (char)216, (char)97, (char)2, (char)255, (char)42, (char)198, (char)36, (char)45, (char)211, (char)16, (char)79, (char)197, (char)57, (char)131, (char)63, (char)32, (char)121, (char)36, (char)218, (char)159, (char)150, (char)250, (char)17, (char)121, (char)91, (char)93, (char)48, (char)89, (char)10, (char)0, (char)126}, 0) ;
        p142.transfer_type_SET((char)178) ;
        p142.uri_type_SET((char)29) ;
        p142.request_id_SET((char)217) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -1.7057006E38F);
            assert(pack.time_boot_ms_GET() == 2510162894L);
            assert(pack.press_diff_GET() == -3.2251719E38F);
            assert(pack.temperature_GET() == (short) -1003);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(2510162894L) ;
        p143.temperature_SET((short) -1003) ;
        p143.press_abs_SET(-1.7057006E38F) ;
        p143.press_diff_SET(-3.2251719E38F) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.est_capabilities_GET() == (char)110);
            assert(pack.lat_GET() == 2089028739);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-3.0034328E38F, -2.40869E37F, -8.4850756E37F}));
            assert(Arrays.equals(pack.vel_GET(),  new float[] {-3.0985405E38F, 8.200859E37F, 3.3932544E38F}));
            assert(pack.timestamp_GET() == 7081140451244682621L);
            assert(pack.custom_state_GET() == 8551408587715569687L);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-3.116375E38F, 2.562434E38F, -2.4891705E38F}));
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-1.9433078E38F, 2.3238962E38F, 1.4254388E38F}));
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {6.151201E37F, -8.404876E37F, 9.316432E37F, 1.8165632E38F}));
            assert(pack.alt_GET() == 1.076363E38F);
            assert(pack.lon_GET() == -959469783);
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.timestamp_SET(7081140451244682621L) ;
        p144.lon_SET(-959469783) ;
        p144.position_cov_SET(new float[] {-3.116375E38F, 2.562434E38F, -2.4891705E38F}, 0) ;
        p144.est_capabilities_SET((char)110) ;
        p144.lat_SET(2089028739) ;
        p144.vel_SET(new float[] {-3.0985405E38F, 8.200859E37F, 3.3932544E38F}, 0) ;
        p144.rates_SET(new float[] {-1.9433078E38F, 2.3238962E38F, 1.4254388E38F}, 0) ;
        p144.custom_state_SET(8551408587715569687L) ;
        p144.acc_SET(new float[] {-3.0034328E38F, -2.40869E37F, -8.4850756E37F}, 0) ;
        p144.alt_SET(1.076363E38F) ;
        p144.attitude_q_SET(new float[] {6.151201E37F, -8.404876E37F, 9.316432E37F, 1.8165632E38F}, 0) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.x_vel_GET() == 2.6902562E38F);
            assert(pack.time_usec_GET() == 4870915584870790148L);
            assert(pack.roll_rate_GET() == -2.3597507E38F);
            assert(pack.y_acc_GET() == -1.4934735E38F);
            assert(pack.y_pos_GET() == 1.7883855E37F);
            assert(pack.pitch_rate_GET() == -1.6624721E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {-1.7243986E36F, 2.2926717E38F, 1.8210837E38F}));
            assert(pack.z_vel_GET() == -2.4788989E38F);
            assert(pack.airspeed_GET() == 2.0790015E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {2.7674906E38F, -4.8705276E37F, 2.9129704E38F}));
            assert(pack.x_acc_GET() == 3.3526703E38F);
            assert(pack.y_vel_GET() == -1.7861637E38F);
            assert(pack.yaw_rate_GET() == 3.166141E37F);
            assert(pack.z_acc_GET() == 2.8546881E38F);
            assert(pack.x_pos_GET() == -2.1429512E38F);
            assert(pack.z_pos_GET() == 3.0029947E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.3849644E38F, 8.085753E37F, 2.4488507E37F, -2.531354E38F}));
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.y_vel_SET(-1.7861637E38F) ;
        p146.z_pos_SET(3.0029947E38F) ;
        p146.x_acc_SET(3.3526703E38F) ;
        p146.vel_variance_SET(new float[] {2.7674906E38F, -4.8705276E37F, 2.9129704E38F}, 0) ;
        p146.z_acc_SET(2.8546881E38F) ;
        p146.q_SET(new float[] {-3.3849644E38F, 8.085753E37F, 2.4488507E37F, -2.531354E38F}, 0) ;
        p146.y_pos_SET(1.7883855E37F) ;
        p146.yaw_rate_SET(3.166141E37F) ;
        p146.y_acc_SET(-1.4934735E38F) ;
        p146.airspeed_SET(2.0790015E38F) ;
        p146.roll_rate_SET(-2.3597507E38F) ;
        p146.z_vel_SET(-2.4788989E38F) ;
        p146.x_pos_SET(-2.1429512E38F) ;
        p146.time_usec_SET(4870915584870790148L) ;
        p146.pitch_rate_SET(-1.6624721E38F) ;
        p146.pos_variance_SET(new float[] {-1.7243986E36F, 2.2926717E38F, 1.8210837E38F}, 0) ;
        p146.x_vel_SET(2.6902562E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.current_battery_GET() == (short)7594);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)41570, (char)9179, (char)37204, (char)13856, (char)62686, (char)60870, (char)22164, (char)64429, (char)48906, (char)39886}));
            assert(pack.id_GET() == (char)212);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
            assert(pack.temperature_GET() == (short)25749);
            assert(pack.energy_consumed_GET() == 804984339);
            assert(pack.current_consumed_GET() == -867012184);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
            assert(pack.battery_remaining_GET() == (byte) - 16);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION) ;
        p147.current_consumed_SET(-867012184) ;
        p147.id_SET((char)212) ;
        p147.energy_consumed_SET(804984339) ;
        p147.battery_remaining_SET((byte) - 16) ;
        p147.temperature_SET((short)25749) ;
        p147.voltages_SET(new char[] {(char)41570, (char)9179, (char)37204, (char)13856, (char)62686, (char)60870, (char)22164, (char)64429, (char)48906, (char)39886}, 0) ;
        p147.current_battery_SET((short)7594) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)100, (char)60, (char)74, (char)141, (char)218, (char)56, (char)150, (char)50}));
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)115, (char)159, (char)129, (char)62, (char)158, (char)56, (char)22, (char)80}));
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)71, (char)166, (char)95, (char)74, (char)134, (char)179, (char)15, (char)19}));
            assert(pack.board_version_GET() == 2057464372L);
            assert(pack.vendor_id_GET() == (char)20424);
            assert(pack.uid_GET() == 1588934911821622989L);
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET));
            assert(pack.middleware_sw_version_GET() == 2306891085L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)192, (char)108, (char)9, (char)167, (char)42, (char)114, (char)113, (char)7, (char)3, (char)253, (char)69, (char)216, (char)55, (char)219, (char)150, (char)7, (char)55, (char)100}));
            assert(pack.flight_sw_version_GET() == 4074782717L);
            assert(pack.product_id_GET() == (char)32996);
            assert(pack.os_sw_version_GET() == 1693179690L);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.middleware_custom_version_SET(new char[] {(char)100, (char)60, (char)74, (char)141, (char)218, (char)56, (char)150, (char)50}, 0) ;
        p148.vendor_id_SET((char)20424) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET)) ;
        p148.middleware_sw_version_SET(2306891085L) ;
        p148.os_custom_version_SET(new char[] {(char)115, (char)159, (char)129, (char)62, (char)158, (char)56, (char)22, (char)80}, 0) ;
        p148.flight_sw_version_SET(4074782717L) ;
        p148.os_sw_version_SET(1693179690L) ;
        p148.board_version_SET(2057464372L) ;
        p148.uid_SET(1588934911821622989L) ;
        p148.product_id_SET((char)32996) ;
        p148.flight_custom_version_SET(new char[] {(char)71, (char)166, (char)95, (char)74, (char)134, (char)179, (char)15, (char)19}, 0) ;
        p148.uid2_SET(new char[] {(char)192, (char)108, (char)9, (char)167, (char)42, (char)114, (char)113, (char)7, (char)3, (char)253, (char)69, (char)216, (char)55, (char)219, (char)150, (char)7, (char)55, (char)100}, 0, PH) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.y_TRY(ph) == 1.1282816E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL);
            assert(pack.time_usec_GET() == 693277045222541522L);
            assert(pack.x_TRY(ph) == 5.0765597E36F);
            assert(pack.size_x_GET() == -3.1852234E38F);
            assert(pack.target_num_GET() == (char)130);
            assert(pack.angle_y_GET() == 2.5210191E38F);
            assert(pack.angle_x_GET() == -6.688698E37F);
            assert(pack.size_y_GET() == 2.3947214E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.position_valid_TRY(ph) == (char)10);
            assert(pack.distance_GET() == 2.892621E38F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {-1.3059587E38F, 2.8639036E38F, 3.0941676E38F, 2.5363269E38F}));
            assert(pack.z_TRY(ph) == 2.9733033E38F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.y_SET(1.1282816E38F, PH) ;
        p149.size_x_SET(-3.1852234E38F) ;
        p149.target_num_SET((char)130) ;
        p149.q_SET(new float[] {-1.3059587E38F, 2.8639036E38F, 3.0941676E38F, 2.5363269E38F}, 0, PH) ;
        p149.angle_y_SET(2.5210191E38F) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p149.size_y_SET(2.3947214E38F) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL) ;
        p149.position_valid_SET((char)10, PH) ;
        p149.angle_x_SET(-6.688698E37F) ;
        p149.x_SET(5.0765597E36F, PH) ;
        p149.time_usec_SET(693277045222541522L) ;
        p149.z_SET(2.9733033E38F, PH) ;
        p149.distance_SET(2.892621E38F) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_SET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)126);
            assert(pack.target_component_GET() == (char)164);
        });
        GroundControl.FLEXIFUNCTION_SET p150 = CommunicationChannel.new_FLEXIFUNCTION_SET();
        PH.setPack(p150);
        p150.target_component_SET((char)164) ;
        p150.target_system_SET((char)126) ;
        CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_READ_REQ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)76);
            assert(pack.data_index_GET() == (short)15935);
            assert(pack.read_req_type_GET() == (short) -22295);
            assert(pack.target_system_GET() == (char)91);
        });
        GroundControl.FLEXIFUNCTION_READ_REQ p151 = CommunicationChannel.new_FLEXIFUNCTION_READ_REQ();
        PH.setPack(p151);
        p151.data_index_SET((short)15935) ;
        p151.target_system_SET((char)91) ;
        p151.read_req_type_SET((short) -22295) ;
        p151.target_component_SET((char)76) ;
        CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_BUFFER_FUNCTION.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)7);
            assert(pack.func_index_GET() == (char)7125);
            assert(Arrays.equals(pack.data__GET(),  new byte[] {(byte) - 99, (byte)33, (byte) - 103, (byte) - 33, (byte)45, (byte)80, (byte) - 44, (byte) - 10, (byte)114, (byte)89, (byte)105, (byte)96, (byte)63, (byte)96, (byte) - 99, (byte) - 56, (byte) - 104, (byte)89, (byte) - 109, (byte) - 91, (byte)82, (byte)69, (byte) - 75, (byte)65, (byte) - 59, (byte)102, (byte)74, (byte) - 47, (byte) - 18, (byte)57, (byte) - 13, (byte) - 30, (byte) - 94, (byte)90, (byte)49, (byte)27, (byte)88, (byte)60, (byte)22, (byte)49, (byte) - 114, (byte)40, (byte) - 42, (byte)19, (byte)47, (byte) - 33, (byte) - 9, (byte) - 37}));
            assert(pack.data_size_GET() == (char)28405);
            assert(pack.data_address_GET() == (char)32226);
            assert(pack.func_count_GET() == (char)6551);
            assert(pack.target_component_GET() == (char)78);
        });
        GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION p152 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION();
        PH.setPack(p152);
        p152.func_count_SET((char)6551) ;
        p152.data_address_SET((char)32226) ;
        p152.func_index_SET((char)7125) ;
        p152.data_size_SET((char)28405) ;
        p152.target_system_SET((char)7) ;
        p152.target_component_SET((char)78) ;
        p152.data__SET(new byte[] {(byte) - 99, (byte)33, (byte) - 103, (byte) - 33, (byte)45, (byte)80, (byte) - 44, (byte) - 10, (byte)114, (byte)89, (byte)105, (byte)96, (byte)63, (byte)96, (byte) - 99, (byte) - 56, (byte) - 104, (byte)89, (byte) - 109, (byte) - 91, (byte)82, (byte)69, (byte) - 75, (byte)65, (byte) - 59, (byte)102, (byte)74, (byte) - 47, (byte) - 18, (byte)57, (byte) - 13, (byte) - 30, (byte) - 94, (byte)90, (byte)49, (byte)27, (byte)88, (byte)60, (byte)22, (byte)49, (byte) - 114, (byte)40, (byte) - 42, (byte)19, (byte)47, (byte) - 33, (byte) - 9, (byte) - 37}, 0) ;
        CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_BUFFER_FUNCTION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)213);
            assert(pack.func_index_GET() == (char)52334);
            assert(pack.target_component_GET() == (char)130);
            assert(pack.result_GET() == (char)23368);
        });
        GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION_ACK p153 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
        PH.setPack(p153);
        p153.target_component_SET((char)130) ;
        p153.func_index_SET((char)52334) ;
        p153.target_system_SET((char)213) ;
        p153.result_SET((char)23368) ;
        CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_DIRECTORY.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)27);
            assert(pack.target_system_GET() == (char)187);
            assert(pack.count_GET() == (char)56);
            assert(pack.start_index_GET() == (char)5);
            assert(Arrays.equals(pack.directory_data_GET(),  new byte[] {(byte) - 57, (byte) - 78, (byte)100, (byte) - 124, (byte) - 52, (byte)72, (byte)111, (byte) - 125, (byte) - 103, (byte)125, (byte)88, (byte) - 46, (byte)30, (byte) - 67, (byte)93, (byte)49, (byte)1, (byte) - 106, (byte) - 97, (byte)76, (byte)96, (byte)115, (byte) - 6, (byte)1, (byte)121, (byte) - 114, (byte)111, (byte) - 82, (byte) - 94, (byte)99, (byte) - 43, (byte)108, (byte) - 36, (byte)126, (byte) - 123, (byte)118, (byte)14, (byte)60, (byte) - 72, (byte) - 90, (byte)3, (byte)108, (byte) - 114, (byte) - 122, (byte)126, (byte)89, (byte) - 11, (byte) - 104}));
            assert(pack.directory_type_GET() == (char)168);
        });
        GroundControl.FLEXIFUNCTION_DIRECTORY p155 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY();
        PH.setPack(p155);
        p155.count_SET((char)56) ;
        p155.directory_data_SET(new byte[] {(byte) - 57, (byte) - 78, (byte)100, (byte) - 124, (byte) - 52, (byte)72, (byte)111, (byte) - 125, (byte) - 103, (byte)125, (byte)88, (byte) - 46, (byte)30, (byte) - 67, (byte)93, (byte)49, (byte)1, (byte) - 106, (byte) - 97, (byte)76, (byte)96, (byte)115, (byte) - 6, (byte)1, (byte)121, (byte) - 114, (byte)111, (byte) - 82, (byte) - 94, (byte)99, (byte) - 43, (byte)108, (byte) - 36, (byte)126, (byte) - 123, (byte)118, (byte)14, (byte)60, (byte) - 72, (byte) - 90, (byte)3, (byte)108, (byte) - 114, (byte) - 122, (byte)126, (byte)89, (byte) - 11, (byte) - 104}, 0) ;
        p155.target_system_SET((char)187) ;
        p155.target_component_SET((char)27) ;
        p155.start_index_SET((char)5) ;
        p155.directory_type_SET((char)168) ;
        CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_DIRECTORY_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)115);
            assert(pack.directory_type_GET() == (char)198);
            assert(pack.target_system_GET() == (char)183);
            assert(pack.start_index_GET() == (char)145);
            assert(pack.result_GET() == (char)61202);
            assert(pack.count_GET() == (char)99);
        });
        GroundControl.FLEXIFUNCTION_DIRECTORY_ACK p156 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY_ACK();
        PH.setPack(p156);
        p156.directory_type_SET((char)198) ;
        p156.target_system_SET((char)183) ;
        p156.result_SET((char)61202) ;
        p156.target_component_SET((char)115) ;
        p156.start_index_SET((char)145) ;
        p156.count_SET((char)99) ;
        CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_COMMAND.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)143);
            assert(pack.command_type_GET() == (char)153);
            assert(pack.target_component_GET() == (char)52);
        });
        GroundControl.FLEXIFUNCTION_COMMAND p157 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND();
        PH.setPack(p157);
        p157.target_system_SET((char)143) ;
        p157.command_type_SET((char)153) ;
        p157.target_component_SET((char)52) ;
        CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLEXIFUNCTION_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.command_type_GET() == (char)18955);
            assert(pack.result_GET() == (char)24188);
        });
        GroundControl.FLEXIFUNCTION_COMMAND_ACK p158 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND_ACK();
        PH.setPack(p158);
        p158.command_type_SET((char)18955) ;
        p158.result_SET((char)24188) ;
        CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F2_A.add((src, ph, pack) ->
        {
            assert(pack.sue_rmat5_GET() == (short)22724);
            assert(pack.sue_rmat8_GET() == (short)22547);
            assert(pack.sue_cpu_load_GET() == (char)33696);
            assert(pack.sue_rmat1_GET() == (short) -3580);
            assert(pack.sue_rmat3_GET() == (short) -13744);
            assert(pack.sue_magFieldEarth2_GET() == (short) -22126);
            assert(pack.sue_latitude_GET() == -1393607763);
            assert(pack.sue_svs_GET() == (short) -14559);
            assert(pack.sue_estimated_wind_0_GET() == (short) -1107);
            assert(pack.sue_magFieldEarth1_GET() == (short) -22126);
            assert(pack.sue_altitude_GET() == -1332372173);
            assert(pack.sue_rmat4_GET() == (short)10071);
            assert(pack.sue_estimated_wind_2_GET() == (short) -3906);
            assert(pack.sue_magFieldEarth0_GET() == (short) -31741);
            assert(pack.sue_status_GET() == (char)116);
            assert(pack.sue_sog_GET() == (short)22386);
            assert(pack.sue_longitude_GET() == 1001564691);
            assert(pack.sue_air_speed_3DIMU_GET() == (char)24884);
            assert(pack.sue_estimated_wind_1_GET() == (short)19078);
            assert(pack.sue_rmat2_GET() == (short)15789);
            assert(pack.sue_waypoint_index_GET() == (char)28047);
            assert(pack.sue_cog_GET() == (char)18981);
            assert(pack.sue_time_GET() == 2666619767L);
            assert(pack.sue_hdop_GET() == (short)18651);
            assert(pack.sue_rmat7_GET() == (short) -26999);
            assert(pack.sue_rmat0_GET() == (short) -18044);
            assert(pack.sue_rmat6_GET() == (short)3801);
        });
        GroundControl.SERIAL_UDB_EXTRA_F2_A p170 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_A();
        PH.setPack(p170);
        p170.sue_estimated_wind_0_SET((short) -1107) ;
        p170.sue_status_SET((char)116) ;
        p170.sue_rmat3_SET((short) -13744) ;
        p170.sue_cog_SET((char)18981) ;
        p170.sue_magFieldEarth1_SET((short) -22126) ;
        p170.sue_rmat5_SET((short)22724) ;
        p170.sue_rmat4_SET((short)10071) ;
        p170.sue_hdop_SET((short)18651) ;
        p170.sue_magFieldEarth0_SET((short) -31741) ;
        p170.sue_altitude_SET(-1332372173) ;
        p170.sue_svs_SET((short) -14559) ;
        p170.sue_latitude_SET(-1393607763) ;
        p170.sue_rmat2_SET((short)15789) ;
        p170.sue_rmat1_SET((short) -3580) ;
        p170.sue_estimated_wind_2_SET((short) -3906) ;
        p170.sue_magFieldEarth2_SET((short) -22126) ;
        p170.sue_rmat0_SET((short) -18044) ;
        p170.sue_estimated_wind_1_SET((short)19078) ;
        p170.sue_rmat7_SET((short) -26999) ;
        p170.sue_cpu_load_SET((char)33696) ;
        p170.sue_air_speed_3DIMU_SET((char)24884) ;
        p170.sue_rmat6_SET((short)3801) ;
        p170.sue_time_SET(2666619767L) ;
        p170.sue_rmat8_SET((short)22547) ;
        p170.sue_waypoint_index_SET((char)28047) ;
        p170.sue_sog_SET((short)22386) ;
        p170.sue_longitude_SET(1001564691) ;
        CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F2_B.add((src, ph, pack) ->
        {
            assert(pack.sue_pwm_input_6_GET() == (short)19218);
            assert(pack.sue_pwm_input_7_GET() == (short) -12751);
            assert(pack.sue_pwm_input_5_GET() == (short)27292);
            assert(pack.sue_imu_location_y_GET() == (short) -16888);
            assert(pack.sue_pwm_input_4_GET() == (short)18726);
            assert(pack.sue_location_error_earth_y_GET() == (short) -29748);
            assert(pack.sue_location_error_earth_x_GET() == (short)8654);
            assert(pack.sue_aero_x_GET() == (short) -16961);
            assert(pack.sue_pwm_input_2_GET() == (short) -31747);
            assert(pack.sue_waypoint_goal_y_GET() == (short) -7513);
            assert(pack.sue_imu_velocity_y_GET() == (short) -25643);
            assert(pack.sue_pwm_input_8_GET() == (short) -11494);
            assert(pack.sue_pwm_output_5_GET() == (short) -23770);
            assert(pack.sue_location_error_earth_z_GET() == (short) -19201);
            assert(pack.sue_desired_height_GET() == (short) -20004);
            assert(pack.sue_pwm_output_6_GET() == (short) -31133);
            assert(pack.sue_barom_alt_GET() == -276108563);
            assert(pack.sue_pwm_input_9_GET() == (short) -21498);
            assert(pack.sue_pwm_input_3_GET() == (short) -3950);
            assert(pack.sue_pwm_output_1_GET() == (short)13668);
            assert(pack.sue_pwm_input_12_GET() == (short) -21524);
            assert(pack.sue_imu_velocity_x_GET() == (short) -861);
            assert(pack.sue_pwm_output_12_GET() == (short) -28168);
            assert(pack.sue_pwm_output_3_GET() == (short) -11473);
            assert(pack.sue_pwm_output_4_GET() == (short)18577);
            assert(pack.sue_aero_y_GET() == (short) -7415);
            assert(pack.sue_bat_volt_GET() == (short)6167);
            assert(pack.sue_imu_location_x_GET() == (short) -1719);
            assert(pack.sue_memory_stack_free_GET() == (short)21137);
            assert(pack.sue_bat_amp_hours_GET() == (short) -23885);
            assert(pack.sue_osc_fails_GET() == (short) -7382);
            assert(pack.sue_pwm_input_10_GET() == (short)2447);
            assert(pack.sue_pwm_output_8_GET() == (short) -11570);
            assert(pack.sue_barom_temp_GET() == (short) -25711);
            assert(pack.sue_pwm_input_11_GET() == (short)16194);
            assert(pack.sue_bat_amp_GET() == (short)29962);
            assert(pack.sue_pwm_output_10_GET() == (short) -32523);
            assert(pack.sue_pwm_input_1_GET() == (short) -23782);
            assert(pack.sue_imu_location_z_GET() == (short) -19596);
            assert(pack.sue_imu_velocity_z_GET() == (short)1630);
            assert(pack.sue_time_GET() == 762007409L);
            assert(pack.sue_pwm_output_2_GET() == (short) -28167);
            assert(pack.sue_waypoint_goal_z_GET() == (short) -28382);
            assert(pack.sue_pwm_output_9_GET() == (short) -2583);
            assert(pack.sue_barom_press_GET() == -2047725414);
            assert(pack.sue_pwm_output_11_GET() == (short) -12219);
            assert(pack.sue_waypoint_goal_x_GET() == (short)6856);
            assert(pack.sue_aero_z_GET() == (short) -20808);
            assert(pack.sue_pwm_output_7_GET() == (short) -10348);
            assert(pack.sue_flags_GET() == 698501896L);
        });
        GroundControl.SERIAL_UDB_EXTRA_F2_B p171 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_B();
        PH.setPack(p171);
        p171.sue_location_error_earth_z_SET((short) -19201) ;
        p171.sue_pwm_input_8_SET((short) -11494) ;
        p171.sue_pwm_output_9_SET((short) -2583) ;
        p171.sue_pwm_input_9_SET((short) -21498) ;
        p171.sue_pwm_output_1_SET((short)13668) ;
        p171.sue_memory_stack_free_SET((short)21137) ;
        p171.sue_imu_velocity_x_SET((short) -861) ;
        p171.sue_waypoint_goal_z_SET((short) -28382) ;
        p171.sue_waypoint_goal_x_SET((short)6856) ;
        p171.sue_pwm_input_5_SET((short)27292) ;
        p171.sue_location_error_earth_x_SET((short)8654) ;
        p171.sue_location_error_earth_y_SET((short) -29748) ;
        p171.sue_osc_fails_SET((short) -7382) ;
        p171.sue_pwm_output_7_SET((short) -10348) ;
        p171.sue_barom_press_SET(-2047725414) ;
        p171.sue_pwm_output_8_SET((short) -11570) ;
        p171.sue_pwm_input_7_SET((short) -12751) ;
        p171.sue_aero_z_SET((short) -20808) ;
        p171.sue_pwm_input_3_SET((short) -3950) ;
        p171.sue_pwm_input_12_SET((short) -21524) ;
        p171.sue_bat_amp_SET((short)29962) ;
        p171.sue_pwm_input_1_SET((short) -23782) ;
        p171.sue_pwm_output_11_SET((short) -12219) ;
        p171.sue_bat_amp_hours_SET((short) -23885) ;
        p171.sue_pwm_output_3_SET((short) -11473) ;
        p171.sue_pwm_input_2_SET((short) -31747) ;
        p171.sue_imu_velocity_z_SET((short)1630) ;
        p171.sue_pwm_output_5_SET((short) -23770) ;
        p171.sue_imu_location_x_SET((short) -1719) ;
        p171.sue_aero_y_SET((short) -7415) ;
        p171.sue_flags_SET(698501896L) ;
        p171.sue_imu_velocity_y_SET((short) -25643) ;
        p171.sue_bat_volt_SET((short)6167) ;
        p171.sue_imu_location_y_SET((short) -16888) ;
        p171.sue_pwm_output_10_SET((short) -32523) ;
        p171.sue_pwm_input_11_SET((short)16194) ;
        p171.sue_waypoint_goal_y_SET((short) -7513) ;
        p171.sue_pwm_input_6_SET((short)19218) ;
        p171.sue_pwm_output_12_SET((short) -28168) ;
        p171.sue_imu_location_z_SET((short) -19596) ;
        p171.sue_pwm_output_2_SET((short) -28167) ;
        p171.sue_pwm_input_4_SET((short)18726) ;
        p171.sue_desired_height_SET((short) -20004) ;
        p171.sue_barom_alt_SET(-276108563) ;
        p171.sue_aero_x_SET((short) -16961) ;
        p171.sue_pwm_input_10_SET((short)2447) ;
        p171.sue_pwm_output_4_SET((short)18577) ;
        p171.sue_barom_temp_SET((short) -25711) ;
        p171.sue_time_SET(762007409L) ;
        p171.sue_pwm_output_6_SET((short) -31133) ;
        CommunicationChannel.instance.send(p171);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F4.add((src, ph, pack) ->
        {
            assert(pack.sue_ALTITUDEHOLD_STABILIZED_GET() == (char)250);
            assert(pack.sue_YAW_STABILIZATION_AILERON_GET() == (char)185);
            assert(pack.sue_ROLL_STABILIZATION_RUDDER_GET() == (char)15);
            assert(pack.sue_AILERON_NAVIGATION_GET() == (char)97);
            assert(pack.sue_ALTITUDEHOLD_WAYPOINT_GET() == (char)37);
            assert(pack.sue_RUDDER_NAVIGATION_GET() == (char)99);
            assert(pack.sue_YAW_STABILIZATION_RUDDER_GET() == (char)66);
            assert(pack.sue_ROLL_STABILIZATION_AILERONS_GET() == (char)82);
            assert(pack.sue_RACING_MODE_GET() == (char)13);
            assert(pack.sue_PITCH_STABILIZATION_GET() == (char)237);
        });
        GroundControl.SERIAL_UDB_EXTRA_F4 p172 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F4();
        PH.setPack(p172);
        p172.sue_RACING_MODE_SET((char)13) ;
        p172.sue_PITCH_STABILIZATION_SET((char)237) ;
        p172.sue_RUDDER_NAVIGATION_SET((char)99) ;
        p172.sue_AILERON_NAVIGATION_SET((char)97) ;
        p172.sue_ALTITUDEHOLD_STABILIZED_SET((char)250) ;
        p172.sue_ALTITUDEHOLD_WAYPOINT_SET((char)37) ;
        p172.sue_ROLL_STABILIZATION_RUDDER_SET((char)15) ;
        p172.sue_YAW_STABILIZATION_RUDDER_SET((char)66) ;
        p172.sue_ROLL_STABILIZATION_AILERONS_SET((char)82) ;
        p172.sue_YAW_STABILIZATION_AILERON_SET((char)185) ;
        CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F5.add((src, ph, pack) ->
        {
            assert(pack.sue_ROLLKP_GET() == -2.4538197E38F);
            assert(pack.sue_YAWKD_AILERON_GET() == -3.9043012E37F);
            assert(pack.sue_YAWKP_AILERON_GET() == 2.121642E38F);
            assert(pack.sue_ROLLKD_GET() == -2.0846897E38F);
        });
        GroundControl.SERIAL_UDB_EXTRA_F5 p173 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F5();
        PH.setPack(p173);
        p173.sue_ROLLKP_SET(-2.4538197E38F) ;
        p173.sue_YAWKD_AILERON_SET(-3.9043012E37F) ;
        p173.sue_YAWKP_AILERON_SET(2.121642E38F) ;
        p173.sue_ROLLKD_SET(-2.0846897E38F) ;
        CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F6.add((src, ph, pack) ->
        {
            assert(pack.sue_ROLL_ELEV_MIX_GET() == 1.7430983E38F);
            assert(pack.sue_ELEVATOR_BOOST_GET() == -1.1816951E38F);
            assert(pack.sue_RUDDER_ELEV_MIX_GET() == 2.832215E38F);
            assert(pack.sue_PITCHKD_GET() == 1.5933318E38F);
            assert(pack.sue_PITCHGAIN_GET() == 3.8280008E37F);
        });
        GroundControl.SERIAL_UDB_EXTRA_F6 p174 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F6();
        PH.setPack(p174);
        p174.sue_ROLL_ELEV_MIX_SET(1.7430983E38F) ;
        p174.sue_PITCHGAIN_SET(3.8280008E37F) ;
        p174.sue_ELEVATOR_BOOST_SET(-1.1816951E38F) ;
        p174.sue_RUDDER_ELEV_MIX_SET(2.832215E38F) ;
        p174.sue_PITCHKD_SET(1.5933318E38F) ;
        CommunicationChannel.instance.send(p174);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F7.add((src, ph, pack) ->
        {
            assert(pack.sue_ROLLKD_RUDDER_GET() == -2.0573505E38F);
            assert(pack.sue_RTL_PITCH_DOWN_GET() == 1.1433852E38F);
            assert(pack.sue_YAWKD_RUDDER_GET() == 3.3954632E38F);
            assert(pack.sue_YAWKP_RUDDER_GET() == 3.1574803E38F);
            assert(pack.sue_ROLLKP_RUDDER_GET() == -7.8879366E37F);
            assert(pack.sue_RUDDER_BOOST_GET() == -3.0480529E38F);
        });
        GroundControl.SERIAL_UDB_EXTRA_F7 p175 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F7();
        PH.setPack(p175);
        p175.sue_RUDDER_BOOST_SET(-3.0480529E38F) ;
        p175.sue_YAWKP_RUDDER_SET(3.1574803E38F) ;
        p175.sue_YAWKD_RUDDER_SET(3.3954632E38F) ;
        p175.sue_RTL_PITCH_DOWN_SET(1.1433852E38F) ;
        p175.sue_ROLLKP_RUDDER_SET(-7.8879366E37F) ;
        p175.sue_ROLLKD_RUDDER_SET(-2.0573505E38F) ;
        CommunicationChannel.instance.send(p175);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F8.add((src, ph, pack) ->
        {
            assert(pack.sue_HEIGHT_TARGET_MIN_GET() == 2.006354E38F);
            assert(pack.sue_ALT_HOLD_PITCH_HIGH_GET() == -1.6779039E38F);
            assert(pack.sue_HEIGHT_TARGET_MAX_GET() == 2.9134247E38F);
            assert(pack.sue_ALT_HOLD_PITCH_MAX_GET() == -2.4826438E38F);
            assert(pack.sue_ALT_HOLD_THROTTLE_MIN_GET() == -9.826303E37F);
            assert(pack.sue_ALT_HOLD_THROTTLE_MAX_GET() == 1.9079565E38F);
            assert(pack.sue_ALT_HOLD_PITCH_MIN_GET() == 3.3714756E38F);
        });
        GroundControl.SERIAL_UDB_EXTRA_F8 p176 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F8();
        PH.setPack(p176);
        p176.sue_ALT_HOLD_PITCH_HIGH_SET(-1.6779039E38F) ;
        p176.sue_ALT_HOLD_THROTTLE_MAX_SET(1.9079565E38F) ;
        p176.sue_ALT_HOLD_PITCH_MIN_SET(3.3714756E38F) ;
        p176.sue_ALT_HOLD_THROTTLE_MIN_SET(-9.826303E37F) ;
        p176.sue_HEIGHT_TARGET_MIN_SET(2.006354E38F) ;
        p176.sue_HEIGHT_TARGET_MAX_SET(2.9134247E38F) ;
        p176.sue_ALT_HOLD_PITCH_MAX_SET(-2.4826438E38F) ;
        CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F13.add((src, ph, pack) ->
        {
            assert(pack.sue_lon_origin_GET() == 2032807984);
            assert(pack.sue_alt_origin_GET() == 331267134);
            assert(pack.sue_week_no_GET() == (short)3174);
            assert(pack.sue_lat_origin_GET() == -438562206);
        });
        GroundControl.SERIAL_UDB_EXTRA_F13 p177 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F13();
        PH.setPack(p177);
        p177.sue_alt_origin_SET(331267134) ;
        p177.sue_lon_origin_SET(2032807984) ;
        p177.sue_week_no_SET((short)3174) ;
        p177.sue_lat_origin_SET(-438562206) ;
        CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F14.add((src, ph, pack) ->
        {
            assert(pack.sue_RCON_GET() == (short) -9570);
            assert(pack.sue_AIRFRAME_GET() == (char)37);
            assert(pack.sue_CLOCK_CONFIG_GET() == (char)198);
            assert(pack.sue_DR_GET() == (char)132);
            assert(pack.sue_TRAP_FLAGS_GET() == (short) -24314);
            assert(pack.sue_GPS_TYPE_GET() == (char)11);
            assert(pack.sue_BOARD_TYPE_GET() == (char)238);
            assert(pack.sue_FLIGHT_PLAN_TYPE_GET() == (char)25);
            assert(pack.sue_TRAP_SOURCE_GET() == 217582474L);
            assert(pack.sue_osc_fail_count_GET() == (short) -10686);
            assert(pack.sue_WIND_ESTIMATION_GET() == (char)55);
        });
        GroundControl.SERIAL_UDB_EXTRA_F14 p178 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F14();
        PH.setPack(p178);
        p178.sue_FLIGHT_PLAN_TYPE_SET((char)25) ;
        p178.sue_GPS_TYPE_SET((char)11) ;
        p178.sue_TRAP_SOURCE_SET(217582474L) ;
        p178.sue_BOARD_TYPE_SET((char)238) ;
        p178.sue_RCON_SET((short) -9570) ;
        p178.sue_CLOCK_CONFIG_SET((char)198) ;
        p178.sue_osc_fail_count_SET((short) -10686) ;
        p178.sue_WIND_ESTIMATION_SET((char)55) ;
        p178.sue_TRAP_FLAGS_SET((short) -24314) ;
        p178.sue_DR_SET((char)132) ;
        p178.sue_AIRFRAME_SET((char)37) ;
        CommunicationChannel.instance.send(p178);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F15.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.sue_ID_VEHICLE_MODEL_NAME_GET(),  new char[] {(char)114, (char)69, (char)140, (char)207, (char)216, (char)47, (char)245, (char)21, (char)191, (char)251, (char)227, (char)30, (char)106, (char)99, (char)249, (char)248, (char)80, (char)176, (char)85, (char)185, (char)96, (char)156, (char)144, (char)117, (char)82, (char)229, (char)56, (char)195, (char)20, (char)219, (char)202, (char)193, (char)186, (char)249, (char)230, (char)236, (char)199, (char)117, (char)131, (char)93}));
            assert(Arrays.equals(pack.sue_ID_VEHICLE_REGISTRATION_GET(),  new char[] {(char)154, (char)58, (char)148, (char)17, (char)205, (char)7, (char)105, (char)77, (char)247, (char)185, (char)183, (char)172, (char)18, (char)1, (char)99, (char)28, (char)211, (char)128, (char)197, (char)72}));
        });
        GroundControl.SERIAL_UDB_EXTRA_F15 p179 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F15();
        PH.setPack(p179);
        p179.sue_ID_VEHICLE_MODEL_NAME_SET(new char[] {(char)114, (char)69, (char)140, (char)207, (char)216, (char)47, (char)245, (char)21, (char)191, (char)251, (char)227, (char)30, (char)106, (char)99, (char)249, (char)248, (char)80, (char)176, (char)85, (char)185, (char)96, (char)156, (char)144, (char)117, (char)82, (char)229, (char)56, (char)195, (char)20, (char)219, (char)202, (char)193, (char)186, (char)249, (char)230, (char)236, (char)199, (char)117, (char)131, (char)93}, 0) ;
        p179.sue_ID_VEHICLE_REGISTRATION_SET(new char[] {(char)154, (char)58, (char)148, (char)17, (char)205, (char)7, (char)105, (char)77, (char)247, (char)185, (char)183, (char)172, (char)18, (char)1, (char)99, (char)28, (char)211, (char)128, (char)197, (char)72}, 0) ;
        CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F16.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.sue_ID_LEAD_PILOT_GET(),  new char[] {(char)182, (char)156, (char)75, (char)59, (char)30, (char)51, (char)164, (char)4, (char)78, (char)170, (char)3, (char)223, (char)53, (char)218, (char)60, (char)153, (char)177, (char)149, (char)212, (char)19, (char)197, (char)180, (char)121, (char)176, (char)32, (char)168, (char)165, (char)103, (char)113, (char)240, (char)138, (char)234, (char)101, (char)106, (char)139, (char)101, (char)209, (char)200, (char)237, (char)45}));
            assert(Arrays.equals(pack.sue_ID_DIY_DRONES_URL_GET(),  new char[] {(char)115, (char)83, (char)41, (char)48, (char)104, (char)111, (char)54, (char)111, (char)27, (char)238, (char)234, (char)166, (char)192, (char)250, (char)212, (char)233, (char)100, (char)47, (char)224, (char)32, (char)23, (char)33, (char)84, (char)19, (char)220, (char)86, (char)77, (char)45, (char)235, (char)38, (char)245, (char)187, (char)114, (char)255, (char)246, (char)94, (char)51, (char)181, (char)110, (char)93, (char)189, (char)40, (char)175, (char)51, (char)198, (char)56, (char)73, (char)255, (char)82, (char)132, (char)204, (char)15, (char)215, (char)170, (char)86, (char)139, (char)231, (char)250, (char)237, (char)247, (char)4, (char)152, (char)191, (char)253, (char)250, (char)32, (char)126, (char)196, (char)211, (char)126}));
        });
        GroundControl.SERIAL_UDB_EXTRA_F16 p180 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F16();
        PH.setPack(p180);
        p180.sue_ID_DIY_DRONES_URL_SET(new char[] {(char)115, (char)83, (char)41, (char)48, (char)104, (char)111, (char)54, (char)111, (char)27, (char)238, (char)234, (char)166, (char)192, (char)250, (char)212, (char)233, (char)100, (char)47, (char)224, (char)32, (char)23, (char)33, (char)84, (char)19, (char)220, (char)86, (char)77, (char)45, (char)235, (char)38, (char)245, (char)187, (char)114, (char)255, (char)246, (char)94, (char)51, (char)181, (char)110, (char)93, (char)189, (char)40, (char)175, (char)51, (char)198, (char)56, (char)73, (char)255, (char)82, (char)132, (char)204, (char)15, (char)215, (char)170, (char)86, (char)139, (char)231, (char)250, (char)237, (char)247, (char)4, (char)152, (char)191, (char)253, (char)250, (char)32, (char)126, (char)196, (char)211, (char)126}, 0) ;
        p180.sue_ID_LEAD_PILOT_SET(new char[] {(char)182, (char)156, (char)75, (char)59, (char)30, (char)51, (char)164, (char)4, (char)78, (char)170, (char)3, (char)223, (char)53, (char)218, (char)60, (char)153, (char)177, (char)149, (char)212, (char)19, (char)197, (char)180, (char)121, (char)176, (char)32, (char)168, (char)165, (char)103, (char)113, (char)240, (char)138, (char)234, (char)101, (char)106, (char)139, (char)101, (char)209, (char)200, (char)237, (char)45}, 0) ;
        CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ALTITUDES.add((src, ph, pack) ->
        {
            assert(pack.alt_gps_GET() == 82364094);
            assert(pack.alt_range_finder_GET() == 1261383792);
            assert(pack.alt_extra_GET() == -908284807);
            assert(pack.alt_optical_flow_GET() == 1585995826);
            assert(pack.alt_imu_GET() == -1668843292);
            assert(pack.alt_barometric_GET() == 1432237620);
            assert(pack.time_boot_ms_GET() == 3337547966L);
        });
        GroundControl.ALTITUDES p181 = CommunicationChannel.new_ALTITUDES();
        PH.setPack(p181);
        p181.alt_extra_SET(-908284807) ;
        p181.time_boot_ms_SET(3337547966L) ;
        p181.alt_gps_SET(82364094) ;
        p181.alt_range_finder_SET(1261383792) ;
        p181.alt_imu_SET(-1668843292) ;
        p181.alt_barometric_SET(1432237620) ;
        p181.alt_optical_flow_SET(1585995826) ;
        CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_AIRSPEEDS.add((src, ph, pack) ->
        {
            assert(pack.aoa_GET() == (short)26745);
            assert(pack.airspeed_imu_GET() == (short) -27090);
            assert(pack.time_boot_ms_GET() == 4294648258L);
            assert(pack.airspeed_hot_wire_GET() == (short)30103);
            assert(pack.aoy_GET() == (short)15229);
            assert(pack.airspeed_ultrasonic_GET() == (short) -30622);
            assert(pack.airspeed_pitot_GET() == (short)6065);
        });
        GroundControl.AIRSPEEDS p182 = CommunicationChannel.new_AIRSPEEDS();
        PH.setPack(p182);
        p182.airspeed_pitot_SET((short)6065) ;
        p182.airspeed_ultrasonic_SET((short) -30622) ;
        p182.airspeed_imu_SET((short) -27090) ;
        p182.aoy_SET((short)15229) ;
        p182.time_boot_ms_SET(4294648258L) ;
        p182.airspeed_hot_wire_SET((short)30103) ;
        p182.aoa_SET((short)26745) ;
        CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F17.add((src, ph, pack) ->
        {
            assert(pack.sue_turn_rate_nav_GET() == -2.9770324E38F);
            assert(pack.sue_feed_forward_GET() == 2.9884163E38F);
            assert(pack.sue_turn_rate_fbw_GET() == -5.79305E36F);
        });
        GroundControl.SERIAL_UDB_EXTRA_F17 p183 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F17();
        PH.setPack(p183);
        p183.sue_turn_rate_nav_SET(-2.9770324E38F) ;
        p183.sue_feed_forward_SET(2.9884163E38F) ;
        p183.sue_turn_rate_fbw_SET(-5.79305E36F) ;
        CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F18.add((src, ph, pack) ->
        {
            assert(pack.elevator_trim_normal_GET() == 2.0287016E38F);
            assert(pack.angle_of_attack_normal_GET() == -4.2638078E36F);
            assert(pack.reference_speed_GET() == 2.0504297E38F);
            assert(pack.elevator_trim_inverted_GET() == 1.6357329E38F);
            assert(pack.angle_of_attack_inverted_GET() == 1.0055125E37F);
        });
        GroundControl.SERIAL_UDB_EXTRA_F18 p184 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F18();
        PH.setPack(p184);
        p184.elevator_trim_normal_SET(2.0287016E38F) ;
        p184.angle_of_attack_normal_SET(-4.2638078E36F) ;
        p184.angle_of_attack_inverted_SET(1.0055125E37F) ;
        p184.reference_speed_SET(2.0504297E38F) ;
        p184.elevator_trim_inverted_SET(1.6357329E38F) ;
        CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F19.add((src, ph, pack) ->
        {
            assert(pack.sue_aileron_reversed_GET() == (char)150);
            assert(pack.sue_rudder_output_channel_GET() == (char)4);
            assert(pack.sue_elevator_output_channel_GET() == (char)158);
            assert(pack.sue_elevator_reversed_GET() == (char)155);
            assert(pack.sue_throttle_output_channel_GET() == (char)235);
            assert(pack.sue_aileron_output_channel_GET() == (char)142);
            assert(pack.sue_rudder_reversed_GET() == (char)214);
            assert(pack.sue_throttle_reversed_GET() == (char)114);
        });
        GroundControl.SERIAL_UDB_EXTRA_F19 p185 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F19();
        PH.setPack(p185);
        p185.sue_aileron_output_channel_SET((char)142) ;
        p185.sue_elevator_reversed_SET((char)155) ;
        p185.sue_elevator_output_channel_SET((char)158) ;
        p185.sue_rudder_output_channel_SET((char)4) ;
        p185.sue_throttle_reversed_SET((char)114) ;
        p185.sue_aileron_reversed_SET((char)150) ;
        p185.sue_rudder_reversed_SET((char)214) ;
        p185.sue_throttle_output_channel_SET((char)235) ;
        CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F20.add((src, ph, pack) ->
        {
            assert(pack.sue_trim_value_input_8_GET() == (short) -13978);
            assert(pack.sue_trim_value_input_2_GET() == (short)30662);
            assert(pack.sue_trim_value_input_6_GET() == (short) -18960);
            assert(pack.sue_trim_value_input_4_GET() == (short) -29625);
            assert(pack.sue_trim_value_input_11_GET() == (short) -1451);
            assert(pack.sue_trim_value_input_10_GET() == (short) -2770);
            assert(pack.sue_number_of_inputs_GET() == (char)89);
            assert(pack.sue_trim_value_input_5_GET() == (short) -15487);
            assert(pack.sue_trim_value_input_1_GET() == (short)30393);
            assert(pack.sue_trim_value_input_7_GET() == (short)7055);
            assert(pack.sue_trim_value_input_12_GET() == (short)6407);
            assert(pack.sue_trim_value_input_9_GET() == (short)16517);
            assert(pack.sue_trim_value_input_3_GET() == (short) -24289);
        });
        GroundControl.SERIAL_UDB_EXTRA_F20 p186 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F20();
        PH.setPack(p186);
        p186.sue_trim_value_input_2_SET((short)30662) ;
        p186.sue_trim_value_input_12_SET((short)6407) ;
        p186.sue_trim_value_input_8_SET((short) -13978) ;
        p186.sue_trim_value_input_11_SET((short) -1451) ;
        p186.sue_trim_value_input_9_SET((short)16517) ;
        p186.sue_trim_value_input_4_SET((short) -29625) ;
        p186.sue_trim_value_input_5_SET((short) -15487) ;
        p186.sue_trim_value_input_6_SET((short) -18960) ;
        p186.sue_number_of_inputs_SET((char)89) ;
        p186.sue_trim_value_input_7_SET((short)7055) ;
        p186.sue_trim_value_input_1_SET((short)30393) ;
        p186.sue_trim_value_input_10_SET((short) -2770) ;
        p186.sue_trim_value_input_3_SET((short) -24289) ;
        CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F21.add((src, ph, pack) ->
        {
            assert(pack.sue_accel_z_offset_GET() == (short) -22493);
            assert(pack.sue_gyro_z_offset_GET() == (short)2211);
            assert(pack.sue_gyro_y_offset_GET() == (short) -30290);
            assert(pack.sue_accel_y_offset_GET() == (short) -25222);
            assert(pack.sue_accel_x_offset_GET() == (short)20303);
            assert(pack.sue_gyro_x_offset_GET() == (short) -30992);
        });
        GroundControl.SERIAL_UDB_EXTRA_F21 p187 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F21();
        PH.setPack(p187);
        p187.sue_gyro_z_offset_SET((short)2211) ;
        p187.sue_gyro_y_offset_SET((short) -30290) ;
        p187.sue_accel_x_offset_SET((short)20303) ;
        p187.sue_accel_z_offset_SET((short) -22493) ;
        p187.sue_gyro_x_offset_SET((short) -30992) ;
        p187.sue_accel_y_offset_SET((short) -25222) ;
        CommunicationChannel.instance.send(p187);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SERIAL_UDB_EXTRA_F22.add((src, ph, pack) ->
        {
            assert(pack.sue_gyro_z_at_calibration_GET() == (short) -22912);
            assert(pack.sue_gyro_x_at_calibration_GET() == (short)15938);
            assert(pack.sue_accel_y_at_calibration_GET() == (short) -19196);
            assert(pack.sue_gyro_y_at_calibration_GET() == (short) -651);
            assert(pack.sue_accel_x_at_calibration_GET() == (short) -25908);
            assert(pack.sue_accel_z_at_calibration_GET() == (short) -165);
        });
        GroundControl.SERIAL_UDB_EXTRA_F22 p188 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F22();
        PH.setPack(p188);
        p188.sue_accel_z_at_calibration_SET((short) -165) ;
        p188.sue_accel_x_at_calibration_SET((short) -25908) ;
        p188.sue_gyro_z_at_calibration_SET((short) -22912) ;
        p188.sue_gyro_x_at_calibration_SET((short)15938) ;
        p188.sue_gyro_y_at_calibration_SET((short) -651) ;
        p188.sue_accel_y_at_calibration_SET((short) -19196) ;
        CommunicationChannel.instance.send(p188);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        CommunicationChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.hagl_ratio_GET() == 1.5772409E38F);
            assert(pack.time_usec_GET() == 8703233362132732438L);
            assert(pack.mag_ratio_GET() == 1.0004044E38F);
            assert(pack.vel_ratio_GET() == -2.2307345E38F);
            assert(pack.pos_horiz_ratio_GET() == -2.0012599E38F);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL));
            assert(pack.tas_ratio_GET() == 2.4282732E38F);
            assert(pack.pos_vert_ratio_GET() == 1.0744476E38F);
            assert(pack.pos_horiz_accuracy_GET() == -3.0773608E38F);
            assert(pack.pos_vert_accuracy_GET() == 2.419873E37F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.mag_ratio_SET(1.0004044E38F) ;
        p230.hagl_ratio_SET(1.5772409E38F) ;
        p230.tas_ratio_SET(2.4282732E38F) ;
        p230.pos_horiz_ratio_SET(-2.0012599E38F) ;
        p230.time_usec_SET(8703233362132732438L) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL)) ;
        p230.vel_ratio_SET(-2.2307345E38F) ;
        p230.pos_vert_accuracy_SET(2.419873E37F) ;
        p230.pos_horiz_accuracy_SET(-3.0773608E38F) ;
        p230.pos_vert_ratio_SET(1.0744476E38F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.horiz_accuracy_GET() == 3.1981561E38F);
            assert(pack.var_horiz_GET() == 1.3036065E38F);
            assert(pack.var_vert_GET() == 3.1535598E38F);
            assert(pack.time_usec_GET() == 3635636570870917154L);
            assert(pack.wind_z_GET() == -2.7143056E38F);
            assert(pack.wind_y_GET() == -1.3734615E38F);
            assert(pack.wind_alt_GET() == 5.1838685E37F);
            assert(pack.wind_x_GET() == -2.3757916E38F);
            assert(pack.vert_accuracy_GET() == -2.4206565E38F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_x_SET(-2.3757916E38F) ;
        p231.wind_y_SET(-1.3734615E38F) ;
        p231.horiz_accuracy_SET(3.1981561E38F) ;
        p231.wind_z_SET(-2.7143056E38F) ;
        p231.vert_accuracy_SET(-2.4206565E38F) ;
        p231.var_vert_SET(3.1535598E38F) ;
        p231.time_usec_SET(3635636570870917154L) ;
        p231.wind_alt_SET(5.1838685E37F) ;
        p231.var_horiz_SET(1.3036065E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.vn_GET() == 9.992604E37F);
            assert(pack.vd_GET() == -1.0498042E37F);
            assert(pack.hdop_GET() == -2.2717082E38F);
            assert(pack.time_week_GET() == (char)27580);
            assert(pack.lon_GET() == 1776995149);
            assert(pack.lat_GET() == -2039073518);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY));
            assert(pack.vert_accuracy_GET() == 2.056724E38F);
            assert(pack.fix_type_GET() == (char)252);
            assert(pack.time_week_ms_GET() == 22119777L);
            assert(pack.horiz_accuracy_GET() == 2.7562708E38F);
            assert(pack.gps_id_GET() == (char)53);
            assert(pack.vdop_GET() == -6.4483875E37F);
            assert(pack.alt_GET() == -6.593549E37F);
            assert(pack.ve_GET() == -6.085684E37F);
            assert(pack.satellites_visible_GET() == (char)162);
            assert(pack.speed_accuracy_GET() == -2.2839733E38F);
            assert(pack.time_usec_GET() == 2428141337049467377L);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.fix_type_SET((char)252) ;
        p232.time_week_ms_SET(22119777L) ;
        p232.hdop_SET(-2.2717082E38F) ;
        p232.vdop_SET(-6.4483875E37F) ;
        p232.alt_SET(-6.593549E37F) ;
        p232.ve_SET(-6.085684E37F) ;
        p232.satellites_visible_SET((char)162) ;
        p232.time_week_SET((char)27580) ;
        p232.speed_accuracy_SET(-2.2839733E38F) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY)) ;
        p232.gps_id_SET((char)53) ;
        p232.vert_accuracy_SET(2.056724E38F) ;
        p232.horiz_accuracy_SET(2.7562708E38F) ;
        p232.vn_SET(9.992604E37F) ;
        p232.vd_SET(-1.0498042E37F) ;
        p232.time_usec_SET(2428141337049467377L) ;
        p232.lon_SET(1776995149) ;
        p232.lat_SET(-2039073518) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (char)20);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)2, (char)216, (char)78, (char)231, (char)204, (char)176, (char)170, (char)10, (char)227, (char)52, (char)213, (char)20, (char)150, (char)119, (char)89, (char)180, (char)98, (char)213, (char)187, (char)180, (char)21, (char)246, (char)76, (char)129, (char)205, (char)112, (char)215, (char)7, (char)118, (char)246, (char)209, (char)250, (char)115, (char)126, (char)253, (char)81, (char)225, (char)200, (char)1, (char)227, (char)231, (char)134, (char)144, (char)235, (char)177, (char)83, (char)137, (char)52, (char)67, (char)199, (char)171, (char)43, (char)17, (char)79, (char)179, (char)113, (char)19, (char)51, (char)145, (char)151, (char)181, (char)131, (char)77, (char)177, (char)81, (char)198, (char)13, (char)14, (char)238, (char)17, (char)14, (char)207, (char)177, (char)128, (char)247, (char)185, (char)162, (char)247, (char)37, (char)175, (char)100, (char)45, (char)124, (char)35, (char)78, (char)5, (char)159, (char)9, (char)66, (char)22, (char)1, (char)132, (char)23, (char)115, (char)9, (char)108, (char)125, (char)186, (char)56, (char)232, (char)66, (char)196, (char)80, (char)75, (char)8, (char)243, (char)222, (char)122, (char)10, (char)8, (char)4, (char)145, (char)134, (char)98, (char)230, (char)168, (char)170, (char)73, (char)52, (char)166, (char)77, (char)220, (char)2, (char)174, (char)85, (char)123, (char)103, (char)61, (char)107, (char)168, (char)173, (char)66, (char)177, (char)43, (char)248, (char)191, (char)53, (char)239, (char)62, (char)143, (char)53, (char)23, (char)25, (char)159, (char)110, (char)94, (char)252, (char)163, (char)120, (char)21, (char)168, (char)50, (char)130, (char)250, (char)119, (char)215, (char)196, (char)95, (char)93, (char)91, (char)59, (char)223, (char)58, (char)61, (char)115, (char)186, (char)205, (char)106, (char)253, (char)5, (char)158, (char)103, (char)74, (char)205, (char)182, (char)187, (char)207, (char)14, (char)73, (char)241}));
            assert(pack.len_GET() == (char)91);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)20) ;
        p233.len_SET((char)91) ;
        p233.data__SET(new char[] {(char)2, (char)216, (char)78, (char)231, (char)204, (char)176, (char)170, (char)10, (char)227, (char)52, (char)213, (char)20, (char)150, (char)119, (char)89, (char)180, (char)98, (char)213, (char)187, (char)180, (char)21, (char)246, (char)76, (char)129, (char)205, (char)112, (char)215, (char)7, (char)118, (char)246, (char)209, (char)250, (char)115, (char)126, (char)253, (char)81, (char)225, (char)200, (char)1, (char)227, (char)231, (char)134, (char)144, (char)235, (char)177, (char)83, (char)137, (char)52, (char)67, (char)199, (char)171, (char)43, (char)17, (char)79, (char)179, (char)113, (char)19, (char)51, (char)145, (char)151, (char)181, (char)131, (char)77, (char)177, (char)81, (char)198, (char)13, (char)14, (char)238, (char)17, (char)14, (char)207, (char)177, (char)128, (char)247, (char)185, (char)162, (char)247, (char)37, (char)175, (char)100, (char)45, (char)124, (char)35, (char)78, (char)5, (char)159, (char)9, (char)66, (char)22, (char)1, (char)132, (char)23, (char)115, (char)9, (char)108, (char)125, (char)186, (char)56, (char)232, (char)66, (char)196, (char)80, (char)75, (char)8, (char)243, (char)222, (char)122, (char)10, (char)8, (char)4, (char)145, (char)134, (char)98, (char)230, (char)168, (char)170, (char)73, (char)52, (char)166, (char)77, (char)220, (char)2, (char)174, (char)85, (char)123, (char)103, (char)61, (char)107, (char)168, (char)173, (char)66, (char)177, (char)43, (char)248, (char)191, (char)53, (char)239, (char)62, (char)143, (char)53, (char)23, (char)25, (char)159, (char)110, (char)94, (char)252, (char)163, (char)120, (char)21, (char)168, (char)50, (char)130, (char)250, (char)119, (char)215, (char)196, (char)95, (char)93, (char)91, (char)59, (char)223, (char)58, (char)61, (char)115, (char)186, (char)205, (char)106, (char)253, (char)5, (char)158, (char)103, (char)74, (char)205, (char)182, (char)187, (char)207, (char)14, (char)73, (char)241}, 0) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.heading_GET() == (char)35520);
            assert(pack.heading_sp_GET() == (short) -15869);
            assert(pack.altitude_sp_GET() == (short) -12036);
            assert(pack.wp_distance_GET() == (char)36370);
            assert(pack.groundspeed_GET() == (char)202);
            assert(pack.gps_nsat_GET() == (char)35);
            assert(pack.pitch_GET() == (short)14860);
            assert(pack.latitude_GET() == 384977243);
            assert(pack.temperature_air_GET() == (byte)120);
            assert(pack.throttle_GET() == (byte)104);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED));
            assert(pack.roll_GET() == (short) -23676);
            assert(pack.failsafe_GET() == (char)105);
            assert(pack.longitude_GET() == -1500882415);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
            assert(pack.temperature_GET() == (byte)39);
            assert(pack.climb_rate_GET() == (byte)124);
            assert(pack.battery_remaining_GET() == (char)221);
            assert(pack.airspeed_sp_GET() == (char)236);
            assert(pack.altitude_amsl_GET() == (short) -14816);
            assert(pack.wp_num_GET() == (char)98);
            assert(pack.custom_mode_GET() == 1021260556L);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
            assert(pack.airspeed_GET() == (char)23);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF) ;
        p234.airspeed_SET((char)23) ;
        p234.custom_mode_SET(1021260556L) ;
        p234.latitude_SET(384977243) ;
        p234.battery_remaining_SET((char)221) ;
        p234.groundspeed_SET((char)202) ;
        p234.gps_nsat_SET((char)35) ;
        p234.altitude_amsl_SET((short) -14816) ;
        p234.wp_distance_SET((char)36370) ;
        p234.longitude_SET(-1500882415) ;
        p234.climb_rate_SET((byte)124) ;
        p234.altitude_sp_SET((short) -12036) ;
        p234.temperature_SET((byte)39) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED)) ;
        p234.heading_sp_SET((short) -15869) ;
        p234.pitch_SET((short)14860) ;
        p234.temperature_air_SET((byte)120) ;
        p234.wp_num_SET((char)98) ;
        p234.airspeed_sp_SET((char)236) ;
        p234.roll_SET((short) -23676) ;
        p234.throttle_SET((byte)104) ;
        p234.failsafe_SET((char)105) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS) ;
        p234.heading_SET((char)35520) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.clipping_1_GET() == 3619307823L);
            assert(pack.time_usec_GET() == 4505904491110191047L);
            assert(pack.clipping_0_GET() == 4266217416L);
            assert(pack.clipping_2_GET() == 1634596693L);
            assert(pack.vibration_y_GET() == 1.792802E38F);
            assert(pack.vibration_x_GET() == -3.0517567E38F);
            assert(pack.vibration_z_GET() == 3.230368E38F);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.clipping_2_SET(1634596693L) ;
        p241.clipping_1_SET(3619307823L) ;
        p241.vibration_y_SET(1.792802E38F) ;
        p241.time_usec_SET(4505904491110191047L) ;
        p241.clipping_0_SET(4266217416L) ;
        p241.vibration_x_SET(-3.0517567E38F) ;
        p241.vibration_z_SET(3.230368E38F) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.1692906E38F, 3.509157E37F, 5.2887296E37F, 2.7550473E38F}));
            assert(pack.longitude_GET() == -691878991);
            assert(pack.approach_z_GET() == 2.9599986E38F);
            assert(pack.altitude_GET() == 1210578558);
            assert(pack.y_GET() == -1.178408E38F);
            assert(pack.z_GET() == -1.0138865E38F);
            assert(pack.latitude_GET() == 134412038);
            assert(pack.approach_y_GET() == 1.5017834E38F);
            assert(pack.time_usec_TRY(ph) == 3138122310749263628L);
            assert(pack.approach_x_GET() == -6.7561345E37F);
            assert(pack.x_GET() == -5.492683E36F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.approach_y_SET(1.5017834E38F) ;
        p242.time_usec_SET(3138122310749263628L, PH) ;
        p242.x_SET(-5.492683E36F) ;
        p242.longitude_SET(-691878991) ;
        p242.y_SET(-1.178408E38F) ;
        p242.approach_z_SET(2.9599986E38F) ;
        p242.approach_x_SET(-6.7561345E37F) ;
        p242.latitude_SET(134412038) ;
        p242.q_SET(new float[] {-3.1692906E38F, 3.509157E37F, 5.2887296E37F, 2.7550473E38F}, 0) ;
        p242.z_SET(-1.0138865E38F) ;
        p242.altitude_SET(1210578558) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.approach_y_GET() == -6.918316E37F);
            assert(pack.altitude_GET() == -2145665349);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.5415056E37F, 1.6865121E38F, 3.9178894E37F, -5.815799E37F}));
            assert(pack.approach_x_GET() == -3.3756262E38F);
            assert(pack.z_GET() == -2.058606E38F);
            assert(pack.longitude_GET() == 11168770);
            assert(pack.target_system_GET() == (char)149);
            assert(pack.y_GET() == 1.8354796E38F);
            assert(pack.time_usec_TRY(ph) == 7402804373120702517L);
            assert(pack.approach_z_GET() == 7.110943E37F);
            assert(pack.latitude_GET() == -670627128);
            assert(pack.x_GET() == -9.021126E37F);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.longitude_SET(11168770) ;
        p243.latitude_SET(-670627128) ;
        p243.target_system_SET((char)149) ;
        p243.altitude_SET(-2145665349) ;
        p243.x_SET(-9.021126E37F) ;
        p243.time_usec_SET(7402804373120702517L, PH) ;
        p243.q_SET(new float[] {2.5415056E37F, 1.6865121E38F, 3.9178894E37F, -5.815799E37F}, 0) ;
        p243.approach_x_SET(-3.3756262E38F) ;
        p243.approach_y_SET(-6.918316E37F) ;
        p243.z_SET(-2.058606E38F) ;
        p243.y_SET(1.8354796E38F) ;
        p243.approach_z_SET(7.110943E37F) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)25342);
            assert(pack.interval_us_GET() == -659151214);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(-659151214) ;
        p244.message_id_SET((char)25342) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND) ;
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_MC) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.callsign_LEN(ph) == 7);
            assert(pack.callsign_TRY(ph).equals("vUjEveh"));
            assert(pack.ICAO_address_GET() == 3540376476L);
            assert(pack.altitude_GET() == -2048395358);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED));
            assert(pack.squawk_GET() == (char)16773);
            assert(pack.lat_GET() == 1684260425);
            assert(pack.tslc_GET() == (char)207);
            assert(pack.heading_GET() == (char)3422);
            assert(pack.ver_velocity_GET() == (short)31987);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            assert(pack.hor_velocity_GET() == (char)60158);
            assert(pack.lon_GET() == -798869992);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSGINED3);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.heading_SET((char)3422) ;
        p246.ver_velocity_SET((short)31987) ;
        p246.ICAO_address_SET(3540376476L) ;
        p246.tslc_SET((char)207) ;
        p246.lon_SET(-798869992) ;
        p246.altitude_SET(-2048395358) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.callsign_SET("vUjEveh", PH) ;
        p246.hor_velocity_SET((char)60158) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSGINED3) ;
        p246.lat_SET(1684260425) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED)) ;
        p246.squawk_SET((char)16773) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == 1185934409L);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
            assert(pack.time_to_minimum_delta_GET() == -2.752989E38F);
            assert(pack.altitude_minimum_delta_GET() == -1.8544497E38F);
            assert(pack.horizontal_minimum_delta_GET() == -1.3770256E37F);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH) ;
        p247.horizontal_minimum_delta_SET(-1.3770256E37F) ;
        p247.altitude_minimum_delta_SET(-1.8544497E38F) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY) ;
        p247.time_to_minimum_delta_SET(-2.752989E38F) ;
        p247.id_SET(1185934409L) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.message_type_GET() == (char)11299);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)227, (char)146, (char)141, (char)118, (char)4, (char)64, (char)62, (char)236, (char)187, (char)221, (char)180, (char)29, (char)85, (char)63, (char)200, (char)35, (char)131, (char)238, (char)77, (char)53, (char)6, (char)103, (char)220, (char)75, (char)73, (char)233, (char)11, (char)107, (char)23, (char)226, (char)156, (char)227, (char)133, (char)244, (char)106, (char)1, (char)179, (char)14, (char)92, (char)201, (char)149, (char)199, (char)51, (char)146, (char)110, (char)208, (char)254, (char)221, (char)43, (char)228, (char)158, (char)135, (char)61, (char)233, (char)145, (char)238, (char)80, (char)196, (char)25, (char)54, (char)140, (char)112, (char)162, (char)133, (char)117, (char)177, (char)236, (char)90, (char)176, (char)148, (char)175, (char)170, (char)72, (char)97, (char)25, (char)119, (char)113, (char)76, (char)18, (char)147, (char)43, (char)126, (char)3, (char)222, (char)211, (char)238, (char)93, (char)67, (char)254, (char)174, (char)201, (char)190, (char)159, (char)26, (char)209, (char)169, (char)207, (char)97, (char)61, (char)197, (char)205, (char)89, (char)46, (char)0, (char)183, (char)235, (char)140, (char)224, (char)79, (char)251, (char)172, (char)203, (char)101, (char)222, (char)7, (char)105, (char)109, (char)209, (char)177, (char)22, (char)100, (char)190, (char)176, (char)107, (char)178, (char)231, (char)112, (char)57, (char)1, (char)112, (char)23, (char)28, (char)113, (char)59, (char)80, (char)39, (char)188, (char)105, (char)27, (char)158, (char)188, (char)51, (char)30, (char)29, (char)134, (char)219, (char)111, (char)67, (char)193, (char)35, (char)148, (char)38, (char)17, (char)114, (char)153, (char)173, (char)30, (char)171, (char)103, (char)151, (char)83, (char)12, (char)208, (char)72, (char)26, (char)17, (char)203, (char)191, (char)210, (char)21, (char)243, (char)150, (char)252, (char)217, (char)43, (char)53, (char)4, (char)213, (char)36, (char)219, (char)238, (char)38, (char)186, (char)96, (char)191, (char)113, (char)1, (char)191, (char)248, (char)42, (char)213, (char)47, (char)206, (char)226, (char)40, (char)70, (char)36, (char)169, (char)12, (char)160, (char)0, (char)185, (char)234, (char)253, (char)93, (char)8, (char)152, (char)8, (char)49, (char)110, (char)166, (char)127, (char)131, (char)193, (char)236, (char)183, (char)216, (char)142, (char)80, (char)214, (char)99, (char)215, (char)223, (char)184, (char)154, (char)128, (char)86, (char)252, (char)2, (char)196, (char)211, (char)255, (char)97, (char)168, (char)148, (char)42, (char)129, (char)56, (char)153, (char)105, (char)170, (char)190, (char)169, (char)247, (char)85, (char)142, (char)143, (char)79, (char)64}));
            assert(pack.target_system_GET() == (char)120);
            assert(pack.target_network_GET() == (char)5);
            assert(pack.target_component_GET() == (char)126);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.message_type_SET((char)11299) ;
        p248.target_network_SET((char)5) ;
        p248.target_system_SET((char)120) ;
        p248.payload_SET(new char[] {(char)227, (char)146, (char)141, (char)118, (char)4, (char)64, (char)62, (char)236, (char)187, (char)221, (char)180, (char)29, (char)85, (char)63, (char)200, (char)35, (char)131, (char)238, (char)77, (char)53, (char)6, (char)103, (char)220, (char)75, (char)73, (char)233, (char)11, (char)107, (char)23, (char)226, (char)156, (char)227, (char)133, (char)244, (char)106, (char)1, (char)179, (char)14, (char)92, (char)201, (char)149, (char)199, (char)51, (char)146, (char)110, (char)208, (char)254, (char)221, (char)43, (char)228, (char)158, (char)135, (char)61, (char)233, (char)145, (char)238, (char)80, (char)196, (char)25, (char)54, (char)140, (char)112, (char)162, (char)133, (char)117, (char)177, (char)236, (char)90, (char)176, (char)148, (char)175, (char)170, (char)72, (char)97, (char)25, (char)119, (char)113, (char)76, (char)18, (char)147, (char)43, (char)126, (char)3, (char)222, (char)211, (char)238, (char)93, (char)67, (char)254, (char)174, (char)201, (char)190, (char)159, (char)26, (char)209, (char)169, (char)207, (char)97, (char)61, (char)197, (char)205, (char)89, (char)46, (char)0, (char)183, (char)235, (char)140, (char)224, (char)79, (char)251, (char)172, (char)203, (char)101, (char)222, (char)7, (char)105, (char)109, (char)209, (char)177, (char)22, (char)100, (char)190, (char)176, (char)107, (char)178, (char)231, (char)112, (char)57, (char)1, (char)112, (char)23, (char)28, (char)113, (char)59, (char)80, (char)39, (char)188, (char)105, (char)27, (char)158, (char)188, (char)51, (char)30, (char)29, (char)134, (char)219, (char)111, (char)67, (char)193, (char)35, (char)148, (char)38, (char)17, (char)114, (char)153, (char)173, (char)30, (char)171, (char)103, (char)151, (char)83, (char)12, (char)208, (char)72, (char)26, (char)17, (char)203, (char)191, (char)210, (char)21, (char)243, (char)150, (char)252, (char)217, (char)43, (char)53, (char)4, (char)213, (char)36, (char)219, (char)238, (char)38, (char)186, (char)96, (char)191, (char)113, (char)1, (char)191, (char)248, (char)42, (char)213, (char)47, (char)206, (char)226, (char)40, (char)70, (char)36, (char)169, (char)12, (char)160, (char)0, (char)185, (char)234, (char)253, (char)93, (char)8, (char)152, (char)8, (char)49, (char)110, (char)166, (char)127, (char)131, (char)193, (char)236, (char)183, (char)216, (char)142, (char)80, (char)214, (char)99, (char)215, (char)223, (char)184, (char)154, (char)128, (char)86, (char)252, (char)2, (char)196, (char)211, (char)255, (char)97, (char)168, (char)148, (char)42, (char)129, (char)56, (char)153, (char)105, (char)170, (char)190, (char)169, (char)247, (char)85, (char)142, (char)143, (char)79, (char)64}, 0) ;
        p248.target_component_SET((char)126) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.address_GET() == (char)42412);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte) - 71, (byte) - 81, (byte)31, (byte)21, (byte) - 41, (byte)29, (byte)81, (byte) - 88, (byte) - 4, (byte) - 127, (byte)3, (byte) - 87, (byte) - 122, (byte)30, (byte) - 12, (byte) - 108, (byte) - 113, (byte) - 95, (byte) - 74, (byte)20, (byte)102, (byte) - 111, (byte) - 38, (byte) - 77, (byte)93, (byte)93, (byte)127, (byte) - 74, (byte) - 71, (byte) - 77, (byte)24, (byte)104}));
            assert(pack.ver_GET() == (char)180);
            assert(pack.type_GET() == (char)254);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)42412) ;
        p249.value_SET(new byte[] {(byte) - 71, (byte) - 81, (byte)31, (byte)21, (byte) - 41, (byte)29, (byte)81, (byte) - 88, (byte) - 4, (byte) - 127, (byte)3, (byte) - 87, (byte) - 122, (byte)30, (byte) - 12, (byte) - 108, (byte) - 113, (byte) - 95, (byte) - 74, (byte)20, (byte)102, (byte) - 111, (byte) - 38, (byte) - 77, (byte)93, (byte)93, (byte)127, (byte) - 74, (byte) - 71, (byte) - 77, (byte)24, (byte)104}, 0) ;
        p249.type_SET((char)254) ;
        p249.ver_SET((char)180) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 1.1839166E38F);
            assert(pack.x_GET() == -2.5473234E38F);
            assert(pack.name_LEN(ph) == 2);
            assert(pack.name_TRY(ph).equals("nd"));
            assert(pack.z_GET() == 2.760062E38F);
            assert(pack.time_usec_GET() == 5609420045642262783L);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.y_SET(1.1839166E38F) ;
        p250.z_SET(2.760062E38F) ;
        p250.x_SET(-2.5473234E38F) ;
        p250.name_SET("nd", PH) ;
        p250.time_usec_SET(5609420045642262783L) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 10);
            assert(pack.name_TRY(ph).equals("hbguZwjskV"));
            assert(pack.value_GET() == 1.8384359E37F);
            assert(pack.time_boot_ms_GET() == 2087758359L);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(2087758359L) ;
        p251.value_SET(1.8384359E37F) ;
        p251.name_SET("hbguZwjskV", PH) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == 1397748472);
            assert(pack.name_LEN(ph) == 9);
            assert(pack.name_TRY(ph).equals("gaofWehpC"));
            assert(pack.time_boot_ms_GET() == 1313805907L);
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(1313805907L) ;
        p252.value_SET(1397748472) ;
        p252.name_SET("gaofWehpC", PH) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
            assert(pack.text_LEN(ph) == 34);
            assert(pack.text_TRY(ph).equals("fpseByjverlkoriCmefpBzhpmoXweffhty"));
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("fpseByjverlkoriCmefpBzhpmoXweffhty", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_CRITICAL) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1817053114L);
            assert(pack.value_GET() == -1.739161E38F);
            assert(pack.ind_GET() == (char)8);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.ind_SET((char)8) ;
        p254.time_boot_ms_SET(1817053114L) ;
        p254.value_SET(-1.739161E38F) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)196, (char)190, (char)113, (char)222, (char)1, (char)185, (char)106, (char)132, (char)201, (char)71, (char)47, (char)21, (char)127, (char)91, (char)59, (char)159, (char)153, (char)139, (char)83, (char)238, (char)145, (char)241, (char)31, (char)25, (char)104, (char)58, (char)70, (char)88, (char)54, (char)104, (char)233, (char)215}));
            assert(pack.initial_timestamp_GET() == 6813636449253865708L);
            assert(pack.target_system_GET() == (char)235);
            assert(pack.target_component_GET() == (char)129);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.initial_timestamp_SET(6813636449253865708L) ;
        p256.target_component_SET((char)129) ;
        p256.target_system_SET((char)235) ;
        p256.secret_key_SET(new char[] {(char)196, (char)190, (char)113, (char)222, (char)1, (char)185, (char)106, (char)132, (char)201, (char)71, (char)47, (char)21, (char)127, (char)91, (char)59, (char)159, (char)153, (char)139, (char)83, (char)238, (char)145, (char)241, (char)31, (char)25, (char)104, (char)58, (char)70, (char)88, (char)54, (char)104, (char)233, (char)215}, 0) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.state_GET() == (char)0);
            assert(pack.last_change_ms_GET() == 3137218064L);
            assert(pack.time_boot_ms_GET() == 1177164720L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(1177164720L) ;
        p257.state_SET((char)0) ;
        p257.last_change_ms_SET(3137218064L) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.tune_LEN(ph) == 19);
            assert(pack.tune_TRY(ph).equals("FanaspyhitsgyRtusai"));
            assert(pack.target_system_GET() == (char)188);
            assert(pack.target_component_GET() == (char)111);
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_component_SET((char)111) ;
        p258.target_system_SET((char)188) ;
        p258.tune_SET("FanaspyhitsgyRtusai", PH) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.focal_length_GET() == 1.9154814E36F);
            assert(pack.firmware_version_GET() == 2070424495L);
            assert(pack.resolution_h_GET() == (char)35466);
            assert(pack.cam_definition_uri_LEN(ph) == 29);
            assert(pack.cam_definition_uri_TRY(ph).equals("ctgaflnognebkhytnskXktunojfhe"));
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE));
            assert(pack.sensor_size_v_GET() == -1.3159147E38F);
            assert(pack.cam_definition_version_GET() == (char)32039);
            assert(pack.resolution_v_GET() == (char)58569);
            assert(pack.sensor_size_h_GET() == 7.594232E37F);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)187, (char)179, (char)114, (char)213, (char)150, (char)130, (char)172, (char)126, (char)211, (char)47, (char)71, (char)168, (char)70, (char)88, (char)122, (char)69, (char)131, (char)228, (char)129, (char)215, (char)172, (char)42, (char)10, (char)171, (char)211, (char)149, (char)238, (char)179, (char)206, (char)169, (char)129, (char)111}));
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)129, (char)249, (char)252, (char)54, (char)188, (char)88, (char)166, (char)216, (char)218, (char)115, (char)203, (char)55, (char)240, (char)253, (char)48, (char)63, (char)221, (char)57, (char)168, (char)92, (char)37, (char)150, (char)19, (char)217, (char)110, (char)188, (char)173, (char)15, (char)154, (char)245, (char)184, (char)76}));
            assert(pack.time_boot_ms_GET() == 15111960L);
            assert(pack.lens_id_GET() == (char)241);
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.cam_definition_uri_SET("ctgaflnognebkhytnskXktunojfhe", PH) ;
        p259.sensor_size_h_SET(7.594232E37F) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE)) ;
        p259.lens_id_SET((char)241) ;
        p259.time_boot_ms_SET(15111960L) ;
        p259.resolution_v_SET((char)58569) ;
        p259.model_name_SET(new char[] {(char)187, (char)179, (char)114, (char)213, (char)150, (char)130, (char)172, (char)126, (char)211, (char)47, (char)71, (char)168, (char)70, (char)88, (char)122, (char)69, (char)131, (char)228, (char)129, (char)215, (char)172, (char)42, (char)10, (char)171, (char)211, (char)149, (char)238, (char)179, (char)206, (char)169, (char)129, (char)111}, 0) ;
        p259.resolution_h_SET((char)35466) ;
        p259.firmware_version_SET(2070424495L) ;
        p259.vendor_name_SET(new char[] {(char)129, (char)249, (char)252, (char)54, (char)188, (char)88, (char)166, (char)216, (char)218, (char)115, (char)203, (char)55, (char)240, (char)253, (char)48, (char)63, (char)221, (char)57, (char)168, (char)92, (char)37, (char)150, (char)19, (char)217, (char)110, (char)188, (char)173, (char)15, (char)154, (char)245, (char)184, (char)76}, 0) ;
        p259.cam_definition_version_SET((char)32039) ;
        p259.sensor_size_v_SET(-1.3159147E38F) ;
        p259.focal_length_SET(1.9154814E36F) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 388507903L);
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(388507903L) ;
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.storage_id_GET() == (char)180);
            assert(pack.available_capacity_GET() == -2.737618E38F);
            assert(pack.write_speed_GET() == -2.6591462E38F);
            assert(pack.read_speed_GET() == -2.936127E37F);
            assert(pack.used_capacity_GET() == -1.6161784E38F);
            assert(pack.total_capacity_GET() == 2.1670807E38F);
            assert(pack.storage_count_GET() == (char)156);
            assert(pack.time_boot_ms_GET() == 3538235087L);
            assert(pack.status_GET() == (char)142);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.read_speed_SET(-2.936127E37F) ;
        p261.time_boot_ms_SET(3538235087L) ;
        p261.status_SET((char)142) ;
        p261.storage_count_SET((char)156) ;
        p261.available_capacity_SET(-2.737618E38F) ;
        p261.total_capacity_SET(2.1670807E38F) ;
        p261.used_capacity_SET(-1.6161784E38F) ;
        p261.write_speed_SET(-2.6591462E38F) ;
        p261.storage_id_SET((char)180) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.image_interval_GET() == 2.2326709E36F);
            assert(pack.recording_time_ms_GET() == 1368802668L);
            assert(pack.time_boot_ms_GET() == 4291478990L);
            assert(pack.video_status_GET() == (char)2);
            assert(pack.available_capacity_GET() == 3.2588778E38F);
            assert(pack.image_status_GET() == (char)7);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.available_capacity_SET(3.2588778E38F) ;
        p262.video_status_SET((char)2) ;
        p262.image_status_SET((char)7) ;
        p262.time_boot_ms_SET(4291478990L) ;
        p262.image_interval_SET(2.2326709E36F) ;
        p262.recording_time_ms_SET(1368802668L) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.time_utc_GET() == 8202062925338527057L);
            assert(pack.image_index_GET() == -612487496);
            assert(pack.time_boot_ms_GET() == 3149172538L);
            assert(pack.alt_GET() == -1795483712);
            assert(pack.capture_result_GET() == (byte) - 20);
            assert(pack.file_url_LEN(ph) == 164);
            assert(pack.file_url_TRY(ph).equals("leyZbnvxvlNxdoypVsjazsiGibjqbeKtBovluwgclxaumrqpriybqrvmkgiacMwuGpnrkigiswEntmzhwaDfGpyworwefzeutVfhBvXrjqnjaltoGihuewjblIvyLwvmcsmoIRRkhkcaycctnexuqcrqovgaZhrqdsnw"));
            assert(pack.relative_alt_GET() == -76083405);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.7486636E38F, -1.0730647E38F, 1.720514E38F, 2.1121256E37F}));
            assert(pack.camera_id_GET() == (char)153);
            assert(pack.lat_GET() == 1056890455);
            assert(pack.lon_GET() == 5632243);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.camera_id_SET((char)153) ;
        p263.time_boot_ms_SET(3149172538L) ;
        p263.time_utc_SET(8202062925338527057L) ;
        p263.lon_SET(5632243) ;
        p263.file_url_SET("leyZbnvxvlNxdoypVsjazsiGibjqbeKtBovluwgclxaumrqpriybqrvmkgiacMwuGpnrkigiswEntmzhwaDfGpyworwefzeutVfhBvXrjqnjaltoGihuewjblIvyLwvmcsmoIRRkhkcaycctnexuqcrqovgaZhrqdsnw", PH) ;
        p263.relative_alt_SET(-76083405) ;
        p263.lat_SET(1056890455) ;
        p263.q_SET(new float[] {-2.7486636E38F, -1.0730647E38F, 1.720514E38F, 2.1121256E37F}, 0) ;
        p263.alt_SET(-1795483712) ;
        p263.capture_result_SET((byte) - 20) ;
        p263.image_index_SET(-612487496) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3573907713L);
            assert(pack.arming_time_utc_GET() == 8931801810553600061L);
            assert(pack.takeoff_time_utc_GET() == 6749679907758970082L);
            assert(pack.flight_uuid_GET() == 8926213487238985749L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.flight_uuid_SET(8926213487238985749L) ;
        p264.takeoff_time_utc_SET(6749679907758970082L) ;
        p264.arming_time_utc_SET(8931801810553600061L) ;
        p264.time_boot_ms_SET(3573907713L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 1.808477E38F);
            assert(pack.roll_GET() == -2.3878917E38F);
            assert(pack.time_boot_ms_GET() == 3834267822L);
            assert(pack.yaw_GET() == 2.7349026E38F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.roll_SET(-2.3878917E38F) ;
        p265.yaw_SET(2.7349026E38F) ;
        p265.time_boot_ms_SET(3834267822L) ;
        p265.pitch_SET(1.808477E38F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)203, (char)194, (char)171, (char)252, (char)102, (char)115, (char)2, (char)72, (char)211, (char)251, (char)88, (char)145, (char)21, (char)105, (char)250, (char)91, (char)162, (char)208, (char)100, (char)86, (char)238, (char)28, (char)29, (char)3, (char)228, (char)61, (char)109, (char)115, (char)21, (char)229, (char)51, (char)85, (char)29, (char)92, (char)48, (char)239, (char)187, (char)194, (char)219, (char)72, (char)212, (char)5, (char)3, (char)62, (char)43, (char)15, (char)101, (char)204, (char)130, (char)251, (char)144, (char)23, (char)254, (char)102, (char)35, (char)116, (char)33, (char)77, (char)199, (char)14, (char)132, (char)141, (char)167, (char)175, (char)54, (char)149, (char)243, (char)45, (char)57, (char)209, (char)237, (char)5, (char)33, (char)9, (char)22, (char)250, (char)83, (char)239, (char)123, (char)243, (char)215, (char)183, (char)218, (char)105, (char)22, (char)227, (char)72, (char)216, (char)101, (char)123, (char)29, (char)22, (char)150, (char)179, (char)38, (char)145, (char)89, (char)174, (char)116, (char)143, (char)237, (char)176, (char)112, (char)95, (char)234, (char)88, (char)149, (char)124, (char)136, (char)231, (char)69, (char)253, (char)213, (char)93, (char)21, (char)255, (char)135, (char)64, (char)32, (char)93, (char)189, (char)210, (char)63, (char)67, (char)56, (char)157, (char)69, (char)245, (char)26, (char)237, (char)190, (char)100, (char)123, (char)168, (char)251, (char)79, (char)62, (char)103, (char)128, (char)103, (char)227, (char)231, (char)185, (char)36, (char)169, (char)69, (char)129, (char)46, (char)224, (char)93, (char)232, (char)145, (char)109, (char)74, (char)92, (char)229, (char)77, (char)131, (char)152, (char)56, (char)208, (char)159, (char)149, (char)228, (char)117, (char)201, (char)19, (char)153, (char)124, (char)191, (char)51, (char)113, (char)11, (char)161, (char)149, (char)111, (char)157, (char)208, (char)129, (char)31, (char)85, (char)159, (char)127, (char)74, (char)112, (char)213, (char)124, (char)36, (char)69, (char)21, (char)231, (char)246, (char)153, (char)41, (char)131, (char)141, (char)99, (char)45, (char)144, (char)61, (char)102, (char)76, (char)143, (char)222, (char)242, (char)143, (char)66, (char)214, (char)99, (char)198, (char)248, (char)191, (char)60, (char)109, (char)92, (char)172, (char)222, (char)137, (char)117, (char)232, (char)123, (char)137, (char)20, (char)237, (char)213, (char)164, (char)45, (char)6, (char)219, (char)7, (char)148, (char)105, (char)47, (char)209, (char)30, (char)126, (char)188, (char)60, (char)145, (char)130, (char)151, (char)95, (char)250, (char)243, (char)13, (char)211, (char)219, (char)99, (char)50}));
            assert(pack.sequence_GET() == (char)30858);
            assert(pack.length_GET() == (char)253);
            assert(pack.target_system_GET() == (char)216);
            assert(pack.target_component_GET() == (char)188);
            assert(pack.first_message_offset_GET() == (char)193);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.data__SET(new char[] {(char)203, (char)194, (char)171, (char)252, (char)102, (char)115, (char)2, (char)72, (char)211, (char)251, (char)88, (char)145, (char)21, (char)105, (char)250, (char)91, (char)162, (char)208, (char)100, (char)86, (char)238, (char)28, (char)29, (char)3, (char)228, (char)61, (char)109, (char)115, (char)21, (char)229, (char)51, (char)85, (char)29, (char)92, (char)48, (char)239, (char)187, (char)194, (char)219, (char)72, (char)212, (char)5, (char)3, (char)62, (char)43, (char)15, (char)101, (char)204, (char)130, (char)251, (char)144, (char)23, (char)254, (char)102, (char)35, (char)116, (char)33, (char)77, (char)199, (char)14, (char)132, (char)141, (char)167, (char)175, (char)54, (char)149, (char)243, (char)45, (char)57, (char)209, (char)237, (char)5, (char)33, (char)9, (char)22, (char)250, (char)83, (char)239, (char)123, (char)243, (char)215, (char)183, (char)218, (char)105, (char)22, (char)227, (char)72, (char)216, (char)101, (char)123, (char)29, (char)22, (char)150, (char)179, (char)38, (char)145, (char)89, (char)174, (char)116, (char)143, (char)237, (char)176, (char)112, (char)95, (char)234, (char)88, (char)149, (char)124, (char)136, (char)231, (char)69, (char)253, (char)213, (char)93, (char)21, (char)255, (char)135, (char)64, (char)32, (char)93, (char)189, (char)210, (char)63, (char)67, (char)56, (char)157, (char)69, (char)245, (char)26, (char)237, (char)190, (char)100, (char)123, (char)168, (char)251, (char)79, (char)62, (char)103, (char)128, (char)103, (char)227, (char)231, (char)185, (char)36, (char)169, (char)69, (char)129, (char)46, (char)224, (char)93, (char)232, (char)145, (char)109, (char)74, (char)92, (char)229, (char)77, (char)131, (char)152, (char)56, (char)208, (char)159, (char)149, (char)228, (char)117, (char)201, (char)19, (char)153, (char)124, (char)191, (char)51, (char)113, (char)11, (char)161, (char)149, (char)111, (char)157, (char)208, (char)129, (char)31, (char)85, (char)159, (char)127, (char)74, (char)112, (char)213, (char)124, (char)36, (char)69, (char)21, (char)231, (char)246, (char)153, (char)41, (char)131, (char)141, (char)99, (char)45, (char)144, (char)61, (char)102, (char)76, (char)143, (char)222, (char)242, (char)143, (char)66, (char)214, (char)99, (char)198, (char)248, (char)191, (char)60, (char)109, (char)92, (char)172, (char)222, (char)137, (char)117, (char)232, (char)123, (char)137, (char)20, (char)237, (char)213, (char)164, (char)45, (char)6, (char)219, (char)7, (char)148, (char)105, (char)47, (char)209, (char)30, (char)126, (char)188, (char)60, (char)145, (char)130, (char)151, (char)95, (char)250, (char)243, (char)13, (char)211, (char)219, (char)99, (char)50}, 0) ;
        p266.length_SET((char)253) ;
        p266.target_component_SET((char)188) ;
        p266.sequence_SET((char)30858) ;
        p266.target_system_SET((char)216) ;
        p266.first_message_offset_SET((char)193) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)182);
            assert(pack.sequence_GET() == (char)12866);
            assert(pack.first_message_offset_GET() == (char)232);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)158, (char)233, (char)107, (char)248, (char)51, (char)117, (char)253, (char)203, (char)101, (char)202, (char)245, (char)177, (char)37, (char)214, (char)123, (char)132, (char)185, (char)220, (char)193, (char)217, (char)136, (char)72, (char)123, (char)187, (char)78, (char)204, (char)246, (char)29, (char)32, (char)26, (char)233, (char)45, (char)158, (char)75, (char)66, (char)144, (char)126, (char)236, (char)228, (char)92, (char)23, (char)41, (char)103, (char)172, (char)10, (char)151, (char)149, (char)30, (char)205, (char)161, (char)80, (char)116, (char)204, (char)134, (char)215, (char)113, (char)90, (char)61, (char)154, (char)179, (char)142, (char)204, (char)38, (char)21, (char)60, (char)122, (char)3, (char)45, (char)201, (char)214, (char)180, (char)137, (char)49, (char)37, (char)58, (char)50, (char)103, (char)30, (char)38, (char)46, (char)120, (char)6, (char)236, (char)4, (char)240, (char)222, (char)165, (char)181, (char)198, (char)198, (char)64, (char)129, (char)148, (char)10, (char)136, (char)240, (char)218, (char)75, (char)79, (char)112, (char)174, (char)121, (char)203, (char)163, (char)215, (char)33, (char)41, (char)27, (char)58, (char)169, (char)203, (char)32, (char)135, (char)8, (char)153, (char)216, (char)117, (char)22, (char)141, (char)155, (char)106, (char)76, (char)114, (char)62, (char)33, (char)57, (char)66, (char)131, (char)41, (char)101, (char)112, (char)131, (char)234, (char)195, (char)208, (char)98, (char)48, (char)181, (char)153, (char)39, (char)65, (char)82, (char)83, (char)127, (char)231, (char)239, (char)58, (char)137, (char)45, (char)172, (char)178, (char)78, (char)81, (char)100, (char)159, (char)19, (char)88, (char)185, (char)54, (char)222, (char)118, (char)144, (char)242, (char)69, (char)162, (char)71, (char)248, (char)202, (char)20, (char)52, (char)69, (char)31, (char)28, (char)176, (char)66, (char)199, (char)38, (char)145, (char)161, (char)161, (char)32, (char)122, (char)109, (char)36, (char)29, (char)86, (char)161, (char)178, (char)5, (char)155, (char)150, (char)208, (char)231, (char)94, (char)216, (char)171, (char)30, (char)102, (char)26, (char)150, (char)185, (char)245, (char)225, (char)219, (char)139, (char)35, (char)85, (char)215, (char)162, (char)198, (char)96, (char)197, (char)114, (char)243, (char)33, (char)19, (char)223, (char)153, (char)16, (char)91, (char)193, (char)7, (char)97, (char)119, (char)107, (char)163, (char)148, (char)18, (char)28, (char)226, (char)188, (char)142, (char)69, (char)93, (char)76, (char)69, (char)226, (char)103, (char)225, (char)26, (char)164, (char)63, (char)136, (char)223, (char)224, (char)254, (char)199, (char)45, (char)254}));
            assert(pack.length_GET() == (char)235);
            assert(pack.target_component_GET() == (char)34);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.first_message_offset_SET((char)232) ;
        p267.target_component_SET((char)34) ;
        p267.length_SET((char)235) ;
        p267.data__SET(new char[] {(char)158, (char)233, (char)107, (char)248, (char)51, (char)117, (char)253, (char)203, (char)101, (char)202, (char)245, (char)177, (char)37, (char)214, (char)123, (char)132, (char)185, (char)220, (char)193, (char)217, (char)136, (char)72, (char)123, (char)187, (char)78, (char)204, (char)246, (char)29, (char)32, (char)26, (char)233, (char)45, (char)158, (char)75, (char)66, (char)144, (char)126, (char)236, (char)228, (char)92, (char)23, (char)41, (char)103, (char)172, (char)10, (char)151, (char)149, (char)30, (char)205, (char)161, (char)80, (char)116, (char)204, (char)134, (char)215, (char)113, (char)90, (char)61, (char)154, (char)179, (char)142, (char)204, (char)38, (char)21, (char)60, (char)122, (char)3, (char)45, (char)201, (char)214, (char)180, (char)137, (char)49, (char)37, (char)58, (char)50, (char)103, (char)30, (char)38, (char)46, (char)120, (char)6, (char)236, (char)4, (char)240, (char)222, (char)165, (char)181, (char)198, (char)198, (char)64, (char)129, (char)148, (char)10, (char)136, (char)240, (char)218, (char)75, (char)79, (char)112, (char)174, (char)121, (char)203, (char)163, (char)215, (char)33, (char)41, (char)27, (char)58, (char)169, (char)203, (char)32, (char)135, (char)8, (char)153, (char)216, (char)117, (char)22, (char)141, (char)155, (char)106, (char)76, (char)114, (char)62, (char)33, (char)57, (char)66, (char)131, (char)41, (char)101, (char)112, (char)131, (char)234, (char)195, (char)208, (char)98, (char)48, (char)181, (char)153, (char)39, (char)65, (char)82, (char)83, (char)127, (char)231, (char)239, (char)58, (char)137, (char)45, (char)172, (char)178, (char)78, (char)81, (char)100, (char)159, (char)19, (char)88, (char)185, (char)54, (char)222, (char)118, (char)144, (char)242, (char)69, (char)162, (char)71, (char)248, (char)202, (char)20, (char)52, (char)69, (char)31, (char)28, (char)176, (char)66, (char)199, (char)38, (char)145, (char)161, (char)161, (char)32, (char)122, (char)109, (char)36, (char)29, (char)86, (char)161, (char)178, (char)5, (char)155, (char)150, (char)208, (char)231, (char)94, (char)216, (char)171, (char)30, (char)102, (char)26, (char)150, (char)185, (char)245, (char)225, (char)219, (char)139, (char)35, (char)85, (char)215, (char)162, (char)198, (char)96, (char)197, (char)114, (char)243, (char)33, (char)19, (char)223, (char)153, (char)16, (char)91, (char)193, (char)7, (char)97, (char)119, (char)107, (char)163, (char)148, (char)18, (char)28, (char)226, (char)188, (char)142, (char)69, (char)93, (char)76, (char)69, (char)226, (char)103, (char)225, (char)26, (char)164, (char)63, (char)136, (char)223, (char)224, (char)254, (char)199, (char)45, (char)254}, 0) ;
        p267.target_system_SET((char)182) ;
        p267.sequence_SET((char)12866) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)168);
            assert(pack.sequence_GET() == (char)63215);
            assert(pack.target_component_GET() == (char)24);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)168) ;
        p268.target_component_SET((char)24) ;
        p268.sequence_SET((char)63215) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.resolution_h_GET() == (char)11173);
            assert(pack.uri_LEN(ph) == 5);
            assert(pack.uri_TRY(ph).equals("unfhy"));
            assert(pack.framerate_GET() == -3.3884974E38F);
            assert(pack.resolution_v_GET() == (char)55577);
            assert(pack.camera_id_GET() == (char)15);
            assert(pack.rotation_GET() == (char)63747);
            assert(pack.status_GET() == (char)71);
            assert(pack.bitrate_GET() == 4195007180L);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.resolution_v_SET((char)55577) ;
        p269.camera_id_SET((char)15) ;
        p269.framerate_SET(-3.3884974E38F) ;
        p269.uri_SET("unfhy", PH) ;
        p269.status_SET((char)71) ;
        p269.resolution_h_SET((char)11173) ;
        p269.rotation_SET((char)63747) ;
        p269.bitrate_SET(4195007180L) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.rotation_GET() == (char)46807);
            assert(pack.target_component_GET() == (char)107);
            assert(pack.camera_id_GET() == (char)136);
            assert(pack.uri_LEN(ph) == 190);
            assert(pack.uri_TRY(ph).equals("fyfrZzTgyijDkfuzonxdynxKeSbzpmqhuiyDjwawpdovtPoatbchnmDgzrZvsodJkwQsutujjpyzitpBkvevrocoskiWgoqvnyskyKdqftHxLodmwfCtnrpdGuamqMbzmivckqsoabKuxtsdumctjetbaghohnkmidnohrqwmbebdkeqbAcCtfqwecocre"));
            assert(pack.target_system_GET() == (char)219);
            assert(pack.bitrate_GET() == 2406705519L);
            assert(pack.resolution_v_GET() == (char)49766);
            assert(pack.resolution_h_GET() == (char)60955);
            assert(pack.framerate_GET() == 2.5752E38F);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.framerate_SET(2.5752E38F) ;
        p270.rotation_SET((char)46807) ;
        p270.uri_SET("fyfrZzTgyijDkfuzonxdynxKeSbzpmqhuiyDjwawpdovtPoatbchnmDgzrZvsodJkwQsutujjpyzitpBkvevrocoskiWgoqvnyskyKdqftHxLodmwfCtnrpdGuamqMbzmivckqsoabKuxtsdumctjetbaghohnkmidnohrqwmbebdkeqbAcCtfqwecocre", PH) ;
        p270.resolution_h_SET((char)60955) ;
        p270.target_system_SET((char)219) ;
        p270.camera_id_SET((char)136) ;
        p270.resolution_v_SET((char)49766) ;
        p270.bitrate_SET(2406705519L) ;
        p270.target_component_SET((char)107) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 11);
            assert(pack.ssid_TRY(ph).equals("ypyfapwejyc"));
            assert(pack.password_LEN(ph) == 10);
            assert(pack.password_TRY(ph).equals("rbsvtqzkya"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("rbsvtqzkya", PH) ;
        p299.ssid_SET("ypyfapwejyc", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)97, (char)166, (char)123, (char)38, (char)229, (char)238, (char)4, (char)249}));
            assert(pack.min_version_GET() == (char)35872);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)46, (char)226, (char)21, (char)206, (char)224, (char)122, (char)186, (char)94}));
            assert(pack.max_version_GET() == (char)55796);
            assert(pack.version_GET() == (char)62748);
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.min_version_SET((char)35872) ;
        p300.spec_version_hash_SET(new char[] {(char)46, (char)226, (char)21, (char)206, (char)224, (char)122, (char)186, (char)94}, 0) ;
        p300.library_version_hash_SET(new char[] {(char)97, (char)166, (char)123, (char)38, (char)229, (char)238, (char)4, (char)249}, 0) ;
        p300.version_SET((char)62748) ;
        p300.max_version_SET((char)55796) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.uptime_sec_GET() == 2546501774L);
            assert(pack.sub_mode_GET() == (char)75);
            assert(pack.time_usec_GET() == 761910025287389183L);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
            assert(pack.vendor_specific_status_code_GET() == (char)21236);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.sub_mode_SET((char)75) ;
        p310.vendor_specific_status_code_SET((char)21236) ;
        p310.time_usec_SET(761910025287389183L) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE) ;
        p310.uptime_sec_SET(2546501774L) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.hw_version_major_GET() == (char)74);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)189, (char)49, (char)58, (char)227, (char)202, (char)91, (char)245, (char)221, (char)82, (char)204, (char)206, (char)166, (char)216, (char)235, (char)97, (char)150}));
            assert(pack.time_usec_GET() == 5887739648260537395L);
            assert(pack.uptime_sec_GET() == 3363371394L);
            assert(pack.sw_vcs_commit_GET() == 3799713561L);
            assert(pack.hw_version_minor_GET() == (char)55);
            assert(pack.sw_version_major_GET() == (char)237);
            assert(pack.name_LEN(ph) == 68);
            assert(pack.name_TRY(ph).equals("hhFFpkobyFnrclqexoaghEXqdkUveqjomcmqbocixVrvbfqfbblhkdnlmgabtvfcspgj"));
            assert(pack.sw_version_minor_GET() == (char)102);
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.uptime_sec_SET(3363371394L) ;
        p311.hw_version_major_SET((char)74) ;
        p311.time_usec_SET(5887739648260537395L) ;
        p311.sw_version_minor_SET((char)102) ;
        p311.sw_version_major_SET((char)237) ;
        p311.hw_unique_id_SET(new char[] {(char)189, (char)49, (char)58, (char)227, (char)202, (char)91, (char)245, (char)221, (char)82, (char)204, (char)206, (char)166, (char)216, (char)235, (char)97, (char)150}, 0) ;
        p311.sw_vcs_commit_SET(3799713561L) ;
        p311.name_SET("hhFFpkobyFnrclqexoaghEXqdkUveqjomcmqbocixVrvbfqfbblhkdnlmgabtvfcspgj", PH) ;
        p311.hw_version_minor_SET((char)55) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)21);
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("vdFo"));
            assert(pack.target_system_GET() == (char)29);
            assert(pack.param_index_GET() == (short) -8042);
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_component_SET((char)21) ;
        p320.param_index_SET((short) -8042) ;
        p320.target_system_SET((char)29) ;
        p320.param_id_SET("vdFo", PH) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)15);
            assert(pack.target_component_GET() == (char)151);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)151) ;
        p321.target_system_SET((char)15) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("g"));
            assert(pack.param_value_LEN(ph) == 108);
            assert(pack.param_value_TRY(ph).equals("wvmolipzbmfhvstnFuggqvpoijkefaeyfizqubnljplbvuXivazyjxdjipKFyskpcocfpbiqoehogfwzjllfuzzixkuzersMdlrUwgbaocmk"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
            assert(pack.param_index_GET() == (char)32992);
            assert(pack.param_count_GET() == (char)29461);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_count_SET((char)29461) ;
        p322.param_value_SET("wvmolipzbmfhvstnFuggqvpoijkefaeyfizqubnljplbvuXivazyjxdjipKFyskpcocfpbiqoehogfwzjllfuzzixkuzersMdlrUwgbaocmk", PH) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM) ;
        p322.param_index_SET((char)32992) ;
        p322.param_id_SET("g", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 108);
            assert(pack.param_value_TRY(ph).equals("gxatzxsmqwukeLswOaoxnztpgoXgwoyjqufqmwsgydeklktbnlcoiVvvzaugnbwwcNefymxbdxgbiecJpoNkxowazyxeiuhmatjujpovcWxl"));
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("Vk"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
            assert(pack.target_component_GET() == (char)246);
            assert(pack.target_system_GET() == (char)232);
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)232) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64) ;
        p323.target_component_SET((char)246) ;
        p323.param_value_SET("gxatzxsmqwukeLswOaoxnztpgoXgwoyjqufqmwsgydeklktbnlcoiVvvzaugnbwwcNefymxbdxgbiecJpoNkxowazyxeiuhmatjujpovcWxl", PH) ;
        p323.param_id_SET("Vk", PH) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 28);
            assert(pack.param_value_TRY(ph).equals("bwwawbhjemrtiuixmhgbajeqnran"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_ACCEPTED);
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("rl"));
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_ACCEPTED) ;
        p324.param_value_SET("bwwawbhjemrtiuixmhgbajeqnran", PH) ;
        p324.param_id_SET("rl", PH) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.increment_GET() == (char)154);
            assert(pack.max_distance_GET() == (char)56235);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)42223, (char)28883, (char)30417, (char)12622, (char)15401, (char)18028, (char)56308, (char)22444, (char)35142, (char)64792, (char)35872, (char)31496, (char)19801, (char)64122, (char)37114, (char)35915, (char)58679, (char)23234, (char)6626, (char)18147, (char)26753, (char)43570, (char)46697, (char)30146, (char)28902, (char)33639, (char)4394, (char)60459, (char)18556, (char)52695, (char)13423, (char)56673, (char)16208, (char)22679, (char)34098, (char)64649, (char)55947, (char)22110, (char)23864, (char)39841, (char)57799, (char)11361, (char)40494, (char)18674, (char)50479, (char)50671, (char)7214, (char)23334, (char)38745, (char)48568, (char)21612, (char)58267, (char)14905, (char)20160, (char)14786, (char)45320, (char)22564, (char)22770, (char)23402, (char)178, (char)31421, (char)62274, (char)38681, (char)28410, (char)64023, (char)23366, (char)13025, (char)11189, (char)24458, (char)15352, (char)11739, (char)35004}));
            assert(pack.min_distance_GET() == (char)52129);
            assert(pack.time_usec_GET() == 4586751313280808337L);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.max_distance_SET((char)56235) ;
        p330.distances_SET(new char[] {(char)42223, (char)28883, (char)30417, (char)12622, (char)15401, (char)18028, (char)56308, (char)22444, (char)35142, (char)64792, (char)35872, (char)31496, (char)19801, (char)64122, (char)37114, (char)35915, (char)58679, (char)23234, (char)6626, (char)18147, (char)26753, (char)43570, (char)46697, (char)30146, (char)28902, (char)33639, (char)4394, (char)60459, (char)18556, (char)52695, (char)13423, (char)56673, (char)16208, (char)22679, (char)34098, (char)64649, (char)55947, (char)22110, (char)23864, (char)39841, (char)57799, (char)11361, (char)40494, (char)18674, (char)50479, (char)50671, (char)7214, (char)23334, (char)38745, (char)48568, (char)21612, (char)58267, (char)14905, (char)20160, (char)14786, (char)45320, (char)22564, (char)22770, (char)23402, (char)178, (char)31421, (char)62274, (char)38681, (char)28410, (char)64023, (char)23366, (char)13025, (char)11189, (char)24458, (char)15352, (char)11739, (char)35004}, 0) ;
        p330.increment_SET((char)154) ;
        p330.min_distance_SET((char)52129) ;
        p330.time_usec_SET(4586751313280808337L) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
    }

}
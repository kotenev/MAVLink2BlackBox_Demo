
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
            long id = id__O(src);
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
            long id = id__w(src);
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
            long id = id__w(src);
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
    public static class ACTUATOR_CONTROL_TARGET extends GroundControl.ACTUATOR_CONTROL_TARGET
    {
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        /**
        *Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
        *	 this field to difference between instances*/
        public char group_mlx_GET()
        {  return (char)((char) get_bytes(data,  8, 1)); }
        /**
        *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
        *	 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
        *	 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
        *	 mixer to repurpose them as generic outputs*/
        public float[] controls_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 9, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        /**
        *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
        *	 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
        *	 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
        *	 mixer to repurpose them as generic outputs*/
        public float[] controls_GET()
        {return controls_GET(new float[8], 0);}
    }
    public static class ALTITUDE extends GroundControl.ALTITUDE
    {
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        /**
        *This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the
        *	 local altitude change). The only guarantee on this field is that it will never be reset and is consistent
        *	 within a flight. The recommended value for this field is the uncorrected barometric altitude at boot
        *	 time. This altitude will also drift and vary between flights*/
        public float altitude_monotonic_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        /**
        *This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events
        *	 like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints
        *	 are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL
        *	 by default and not the WGS84 altitude*/
        public float altitude_amsl_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        /**
        *This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
        *	 to the coordinate origin (0, 0, 0). It is up-positive*/
        public float altitude_local_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float altitude_relative_GET()//This is the altitude above the home position. It resets on each change of the current home position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        /**
        *This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller
        *	 than -1000 should be interpreted as unknown*/
        public float altitude_terrain_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        /**
        *This is not the altitude, but the clear space below the system according to the fused clearance estimate.
        *	 It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving
        *	 target. A negative value indicates no measurement available*/
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
        *	 on the URI type enum*/
        public char[] uri_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 2, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
        *	 on the URI type enum*/
        public char[] uri_GET()
        {return uri_GET(new char[120], 0);} public char transfer_type_GET()//The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
        {  return (char)((char) get_bytes(data,  122, 1)); }
        /**
        *The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
        *	 has a storage associated (e.g. MAVLink FTP)*/
        public char[] storage_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 123, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
        *	 has a storage associated (e.g. MAVLink FTP)*/
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
        *	 should have the UINT16_MAX value*/
        public char[] voltages_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 0, dst_max = pos + 10; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        /**
        *Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
        *	 should have the UINT16_MAX value*/
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
        *	 energy consumption estimat*/
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
        *	 should allow to identify the commit using the main version number even for very large code bases*/
        public char[] flight_custom_version_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 28, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
        *	 should allow to identify the commit using the main version number even for very large code bases*/
        public char[] flight_custom_version_GET()
        {return flight_custom_version_GET(new char[8], 0);}/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	 should allow to identify the commit using the main version number even for very large code bases*/
        public char[] middleware_custom_version_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 36, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
        *	 should allow to identify the commit using the main version number even for very large code bases*/
        public char[] middleware_custom_version_GET()
        {return middleware_custom_version_GET(new char[8], 0);}/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	 should allow to identify the commit using the main version number even for very large code bases*/
        public char[] os_custom_version_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 44, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
        *	 should allow to identify the commit using the main version number even for very large code bases*/
        public char[] os_custom_version_GET()
        {return os_custom_version_GET(new char[8], 0);} public @MAV_PROTOCOL_CAPABILITY int capabilities_GET()//bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
        {  return  1 + (int)get_bits(data, 416, 17); }
        /**
        *UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
        *	 use uid*/
        public char[]  uid2_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  433 && !try_visit_field(ph, 433)) return null;
            return uid2_GET(ph, new char[ph.items], 0);
        }
        /**
        *UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
        *	 use uid*/
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
        {  return  0 + (int)get_bits(data, 236, 2); }
        public float  x_TRY(Bounds.Inside ph)//X Position of the landing target on MAV_FRAME
        {
            if(ph.field_bit !=  238 && !try_visit_field(ph, 238)) return 0;
            return (float)(Float.intBitsToFloat((int) get_bytes(data,  ph.BYTE, 4)));
        }
        public float  y_TRY(Bounds.Inside ph)//Y Position of the landing target on MAV_FRAME
        {
            if(ph.field_bit !=  239 && !try_visit_field(ph, 239)) return 0;
            return (float)(Float.intBitsToFloat((int) get_bytes(data,  ph.BYTE, 4)));
        }
        public float  z_TRY(Bounds.Inside ph)//Z Position of the landing target on MAV_FRAME
        {
            if(ph.field_bit !=  240 && !try_visit_field(ph, 240)) return 0;
            return (float)(Float.intBitsToFloat((int) get_bytes(data,  ph.BYTE, 4)));
        }
        public float[]  q_TRY(Bounds.Inside ph)//Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        {
            if(ph.field_bit !=  241 && !try_visit_field(ph, 241)) return null;
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
*	 the landing targe*/
        public char  position_valid_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  242 && !try_visit_field(ph, 242)) return 0;
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
        {  return  1 + (int)get_bits(data, 320, 11); }
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
        {  return  1 + (int)get_bits(data, 488, 8); }
    }
    public static class GPS_RTCM_DATA extends GroundControl.GPS_RTCM_DATA
    {
        /**
        *LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for
        *	 the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed
        *	 on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer,
        *	 while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered
        *	 fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment
        *	 with a non full payload is received. This management is used to ensure that normal GPS operation doesn't
        *	 corrupt RTCM data, and to recover from a unreliable transport delivery order*/
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
        *	 bit3:GCS, bit4:fence*/
        public char failsafe_GET()
        {  return (char)((char) get_bytes(data,  35, 1)); }
        public char wp_num_GET()//current waypoint number
        {  return (char)((char) get_bytes(data,  36, 1)); }
        public @MAV_MODE_FLAG int base_mode_GET()//System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
        {  return  1 + (int)get_bits(data, 296, 8); }
        public @MAV_LANDED_STATE int landed_state_GET()//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
        {  return  0 + (int)get_bits(data, 304, 3); }
        public @GPS_FIX_TYPE int gps_fix_type_GET()//See the GPS_FIX_TYPE enum.
        {  return  0 + (int)get_bits(data, 307, 4); }
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
        *	 and slope of the groun*/
        public float[] q_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 24, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        /**
        *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
        *	 and slope of the groun*/
        public float[] q_GET()
        {return q_GET(new float[4], 0);}/**
*Local X position of the end of the approach vector. Multicopters should set this position based on their
*	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	 from the threshold / touchdown zone*/
        public float approach_x_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        /**
        *Local Y position of the end of the approach vector. Multicopters should set this position based on their
        *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	 from the threshold / touchdown zone*/
        public float approach_y_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        /**
        *Local Z position of the end of the approach vector. Multicopters should set this position based on their
        *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	 from the threshold / touchdown zone*/
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
        *	 and slope of the groun*/
        public float[] q_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 25, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        /**
        *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
        *	 and slope of the groun*/
        public float[] q_GET()
        {return q_GET(new float[4], 0);}/**
*Local X position of the end of the approach vector. Multicopters should set this position based on their
*	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	 from the threshold / touchdown zone*/
        public float approach_x_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  41, 4))); }
        /**
        *Local Y position of the end of the approach vector. Multicopters should set this position based on their
        *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	 from the threshold / touchdown zone*/
        public float approach_y_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  45, 4))); }
        /**
        *Local Z position of the end of the approach vector. Multicopters should set this position based on their
        *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	 from the threshold / touchdown zone*/
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
                case 74:
                    if(pack == null) return new VFR_HUD();
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
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED));
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_AEROB);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_COAXIAL);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_STANDBY);
            assert(pack.custom_mode_GET() == 384637211L);
            assert(pack.mavlink_version_GET() == (char)90);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_AEROB) ;
        p0.custom_mode_SET(384637211L) ;
        p0.mavlink_version_SET((char)90) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_STANDBY) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED)) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_COAXIAL) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW));
            assert(pack.errors_count2_GET() == (char)13917);
            assert(pack.battery_remaining_GET() == (byte)83);
            assert(pack.errors_count3_GET() == (char)6737);
            assert(pack.errors_comm_GET() == (char)6510);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
            assert(pack.load_GET() == (char)34338);
            assert(pack.drop_rate_comm_GET() == (char)37653);
            assert(pack.errors_count4_GET() == (char)5254);
            assert(pack.voltage_battery_GET() == (char)2863);
            assert(pack.errors_count1_GET() == (char)14896);
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY));
            assert(pack.current_battery_GET() == (short)2282);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW)) ;
        p1.battery_remaining_SET((byte)83) ;
        p1.current_battery_SET((short)2282) ;
        p1.errors_count4_SET((char)5254) ;
        p1.errors_count1_SET((char)14896) ;
        p1.errors_count3_SET((char)6737) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL)) ;
        p1.voltage_battery_SET((char)2863) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY)) ;
        p1.load_SET((char)34338) ;
        p1.errors_comm_SET((char)6510) ;
        p1.drop_rate_comm_SET((char)37653) ;
        p1.errors_count2_SET((char)13917) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 3707836039748098042L);
            assert(pack.time_boot_ms_GET() == 85713055L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(3707836039748098042L) ;
        p2.time_boot_ms_SET(85713055L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)2045);
            assert(pack.yaw_GET() == 2.644705E37F);
            assert(pack.afx_GET() == -2.750588E38F);
            assert(pack.afy_GET() == 1.451785E38F);
            assert(pack.yaw_rate_GET() == -2.4960586E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.x_GET() == -2.6902043E37F);
            assert(pack.y_GET() == 1.5688472E38F);
            assert(pack.vx_GET() == 2.460571E38F);
            assert(pack.vy_GET() == -1.4326759E38F);
            assert(pack.z_GET() == 8.3607097E36F);
            assert(pack.afz_GET() == -2.7071453E38F);
            assert(pack.vz_GET() == 9.648247E37F);
            assert(pack.time_boot_ms_GET() == 1097325750L);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.yaw_rate_SET(-2.4960586E38F) ;
        p3.time_boot_ms_SET(1097325750L) ;
        p3.x_SET(-2.6902043E37F) ;
        p3.vz_SET(9.648247E37F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p3.vy_SET(-1.4326759E38F) ;
        p3.y_SET(1.5688472E38F) ;
        p3.yaw_SET(2.644705E37F) ;
        p3.afx_SET(-2.750588E38F) ;
        p3.afz_SET(-2.7071453E38F) ;
        p3.vx_SET(2.460571E38F) ;
        p3.type_mask_SET((char)2045) ;
        p3.z_SET(8.3607097E36F) ;
        p3.afy_SET(1.451785E38F) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 9063236117319415647L);
            assert(pack.seq_GET() == 3434762094L);
            assert(pack.target_system_GET() == (char)19);
            assert(pack.target_component_GET() == (char)37);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.seq_SET(3434762094L) ;
        p4.target_component_SET((char)37) ;
        p4.target_system_SET((char)19) ;
        p4.time_usec_SET(9063236117319415647L) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)166);
            assert(pack.control_request_GET() == (char)226);
            assert(pack.passkey_LEN(ph) == 10);
            assert(pack.passkey_TRY(ph).equals("uVhgWkblSj"));
            assert(pack.version_GET() == (char)151);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.passkey_SET("uVhgWkblSj", PH) ;
        p5.control_request_SET((char)226) ;
        p5.version_SET((char)151) ;
        p5.target_system_SET((char)166) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.gcs_system_id_GET() == (char)22);
            assert(pack.control_request_GET() == (char)63);
            assert(pack.ack_GET() == (char)190);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.control_request_SET((char)63) ;
        p6.ack_SET((char)190) ;
        p6.gcs_system_id_SET((char)22) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 30);
            assert(pack.key_TRY(ph).equals("tpuvewhlhwunstDjizypttppmzaoad"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("tpuvewhlhwunstDjizypttppmzaoad", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_MANUAL_ARMED);
            assert(pack.custom_mode_GET() == 2014230469L);
            assert(pack.target_system_GET() == (char)158);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.custom_mode_SET(2014230469L) ;
        p11.target_system_SET((char)158) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_MANUAL_ARMED) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (short) -9143);
            assert(pack.target_component_GET() == (char)6);
            assert(pack.target_system_GET() == (char)119);
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("nlyepgmvjbuq"));
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.param_id_SET("nlyepgmvjbuq", PH) ;
        p20.param_index_SET((short) -9143) ;
        p20.target_component_SET((char)6) ;
        p20.target_system_SET((char)119) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)4);
            assert(pack.target_component_GET() == (char)141);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)141) ;
        p21.target_system_SET((char)4) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
            assert(pack.param_value_GET() == -3.9934125E37F);
            assert(pack.param_index_GET() == (char)34569);
            assert(pack.param_id_LEN(ph) == 7);
            assert(pack.param_id_TRY(ph).equals("bwtkzyi"));
            assert(pack.param_count_GET() == (char)3074);
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_value_SET(-3.9934125E37F) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16) ;
        p22.param_id_SET("bwtkzyi", PH) ;
        p22.param_index_SET((char)34569) ;
        p22.param_count_SET((char)3074) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_value_GET() == 2.3022383E37F);
            assert(pack.target_system_GET() == (char)235);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
            assert(pack.param_id_LEN(ph) == 14);
            assert(pack.param_id_TRY(ph).equals("ltsLadscwmyxwk"));
            assert(pack.target_component_GET() == (char)65);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.param_id_SET("ltsLadscwmyxwk", PH) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16) ;
        p23.target_system_SET((char)235) ;
        p23.param_value_SET(2.3022383E37F) ;
        p23.target_component_SET((char)65) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.epv_GET() == (char)53870);
            assert(pack.lon_GET() == 640272003);
            assert(pack.h_acc_TRY(ph) == 252351012L);
            assert(pack.vel_acc_TRY(ph) == 1148889523L);
            assert(pack.eph_GET() == (char)14462);
            assert(pack.time_usec_GET() == 2607858806501970982L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
            assert(pack.alt_ellipsoid_TRY(ph) == 1001159118);
            assert(pack.alt_GET() == -396345655);
            assert(pack.hdg_acc_TRY(ph) == 489146724L);
            assert(pack.cog_GET() == (char)6599);
            assert(pack.v_acc_TRY(ph) == 3690955839L);
            assert(pack.vel_GET() == (char)39835);
            assert(pack.satellites_visible_GET() == (char)13);
            assert(pack.lat_GET() == 370048390);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.lon_SET(640272003) ;
        p24.satellites_visible_SET((char)13) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS) ;
        p24.alt_ellipsoid_SET(1001159118, PH) ;
        p24.h_acc_SET(252351012L, PH) ;
        p24.lat_SET(370048390) ;
        p24.vel_acc_SET(1148889523L, PH) ;
        p24.alt_SET(-396345655) ;
        p24.eph_SET((char)14462) ;
        p24.hdg_acc_SET(489146724L, PH) ;
        p24.epv_SET((char)53870) ;
        p24.cog_SET((char)6599) ;
        p24.v_acc_SET(3690955839L, PH) ;
        p24.vel_SET((char)39835) ;
        p24.time_usec_SET(2607858806501970982L) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)220, (char)215, (char)67, (char)246, (char)16, (char)69, (char)86, (char)207, (char)236, (char)219, (char)158, (char)170, (char)161, (char)26, (char)82, (char)80, (char)106, (char)205, (char)16, (char)104}));
            assert(pack.satellites_visible_GET() == (char)216);
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)168, (char)230, (char)216, (char)127, (char)9, (char)32, (char)68, (char)132, (char)189, (char)178, (char)151, (char)213, (char)226, (char)228, (char)0, (char)181, (char)84, (char)104, (char)129, (char)85}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)71, (char)161, (char)15, (char)234, (char)251, (char)174, (char)7, (char)190, (char)122, (char)191, (char)30, (char)148, (char)253, (char)10, (char)111, (char)191, (char)160, (char)100, (char)79, (char)127}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)32, (char)106, (char)246, (char)49, (char)75, (char)133, (char)101, (char)136, (char)167, (char)205, (char)220, (char)126, (char)136, (char)42, (char)215, (char)190, (char)209, (char)46, (char)79, (char)246}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)199, (char)72, (char)126, (char)161, (char)198, (char)45, (char)207, (char)5, (char)116, (char)70, (char)96, (char)65, (char)221, (char)38, (char)141, (char)19, (char)216, (char)93, (char)190, (char)120}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_used_SET(new char[] {(char)199, (char)72, (char)126, (char)161, (char)198, (char)45, (char)207, (char)5, (char)116, (char)70, (char)96, (char)65, (char)221, (char)38, (char)141, (char)19, (char)216, (char)93, (char)190, (char)120}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)168, (char)230, (char)216, (char)127, (char)9, (char)32, (char)68, (char)132, (char)189, (char)178, (char)151, (char)213, (char)226, (char)228, (char)0, (char)181, (char)84, (char)104, (char)129, (char)85}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)220, (char)215, (char)67, (char)246, (char)16, (char)69, (char)86, (char)207, (char)236, (char)219, (char)158, (char)170, (char)161, (char)26, (char)82, (char)80, (char)106, (char)205, (char)16, (char)104}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)71, (char)161, (char)15, (char)234, (char)251, (char)174, (char)7, (char)190, (char)122, (char)191, (char)30, (char)148, (char)253, (char)10, (char)111, (char)191, (char)160, (char)100, (char)79, (char)127}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)32, (char)106, (char)246, (char)49, (char)75, (char)133, (char)101, (char)136, (char)167, (char)205, (char)220, (char)126, (char)136, (char)42, (char)215, (char)190, (char)209, (char)46, (char)79, (char)246}, 0) ;
        p25.satellites_visible_SET((char)216) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short)14758);
            assert(pack.xacc_GET() == (short) -28261);
            assert(pack.ymag_GET() == (short) -15683);
            assert(pack.xmag_GET() == (short) -15258);
            assert(pack.zmag_GET() == (short) -19704);
            assert(pack.yacc_GET() == (short) -6495);
            assert(pack.time_boot_ms_GET() == 438531244L);
            assert(pack.xgyro_GET() == (short) -12261);
            assert(pack.zacc_GET() == (short) -14450);
            assert(pack.zgyro_GET() == (short)1141);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.xacc_SET((short) -28261) ;
        p26.yacc_SET((short) -6495) ;
        p26.xgyro_SET((short) -12261) ;
        p26.ygyro_SET((short)14758) ;
        p26.ymag_SET((short) -15683) ;
        p26.zgyro_SET((short)1141) ;
        p26.zmag_SET((short) -19704) ;
        p26.zacc_SET((short) -14450) ;
        p26.xmag_SET((short) -15258) ;
        p26.time_boot_ms_SET(438531244L) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.xmag_GET() == (short) -9561);
            assert(pack.time_usec_GET() == 7549120680915995959L);
            assert(pack.zmag_GET() == (short) -4640);
            assert(pack.yacc_GET() == (short)3362);
            assert(pack.ygyro_GET() == (short) -11233);
            assert(pack.xgyro_GET() == (short) -23111);
            assert(pack.zgyro_GET() == (short)4789);
            assert(pack.ymag_GET() == (short) -20255);
            assert(pack.xacc_GET() == (short)42);
            assert(pack.zacc_GET() == (short) -8098);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.ygyro_SET((short) -11233) ;
        p27.xacc_SET((short)42) ;
        p27.zgyro_SET((short)4789) ;
        p27.ymag_SET((short) -20255) ;
        p27.xmag_SET((short) -9561) ;
        p27.time_usec_SET(7549120680915995959L) ;
        p27.zacc_SET((short) -8098) ;
        p27.zmag_SET((short) -4640) ;
        p27.xgyro_SET((short) -23111) ;
        p27.yacc_SET((short)3362) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -4899);
            assert(pack.press_diff1_GET() == (short)4955);
            assert(pack.press_abs_GET() == (short) -7495);
            assert(pack.press_diff2_GET() == (short)25287);
            assert(pack.time_usec_GET() == 4465350460213569427L);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.time_usec_SET(4465350460213569427L) ;
        p28.press_diff1_SET((short)4955) ;
        p28.press_diff2_SET((short)25287) ;
        p28.press_abs_SET((short) -7495) ;
        p28.temperature_SET((short) -4899) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 530510635L);
            assert(pack.temperature_GET() == (short) -731);
            assert(pack.press_abs_GET() == -3.25064E38F);
            assert(pack.press_diff_GET() == -1.217427E38F);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_diff_SET(-1.217427E38F) ;
        p29.time_boot_ms_SET(530510635L) ;
        p29.temperature_SET((short) -731) ;
        p29.press_abs_SET(-3.25064E38F) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 4.0184625E37F);
            assert(pack.pitchspeed_GET() == 6.6800785E37F);
            assert(pack.time_boot_ms_GET() == 1031025624L);
            assert(pack.pitch_GET() == -2.7755328E38F);
            assert(pack.roll_GET() == 2.4077764E38F);
            assert(pack.yawspeed_GET() == -7.4283494E37F);
            assert(pack.rollspeed_GET() == -6.5542475E37F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.yaw_SET(4.0184625E37F) ;
        p30.roll_SET(2.4077764E38F) ;
        p30.pitch_SET(-2.7755328E38F) ;
        p30.pitchspeed_SET(6.6800785E37F) ;
        p30.yawspeed_SET(-7.4283494E37F) ;
        p30.time_boot_ms_SET(1031025624L) ;
        p30.rollspeed_SET(-6.5542475E37F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == -1.8608888E38F);
            assert(pack.pitchspeed_GET() == -2.0207348E38F);
            assert(pack.q4_GET() == 1.2590755E38F);
            assert(pack.q1_GET() == -2.3600216E38F);
            assert(pack.q3_GET() == 3.0359175E37F);
            assert(pack.rollspeed_GET() == 7.234303E36F);
            assert(pack.time_boot_ms_GET() == 2742370674L);
            assert(pack.q2_GET() == -3.633983E37F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.yawspeed_SET(-1.8608888E38F) ;
        p31.q1_SET(-2.3600216E38F) ;
        p31.q3_SET(3.0359175E37F) ;
        p31.q4_SET(1.2590755E38F) ;
        p31.q2_SET(-3.633983E37F) ;
        p31.pitchspeed_SET(-2.0207348E38F) ;
        p31.rollspeed_SET(7.234303E36F) ;
        p31.time_boot_ms_SET(2742370674L) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == -3.8455218E37F);
            assert(pack.time_boot_ms_GET() == 4027761441L);
            assert(pack.z_GET() == -4.513903E37F);
            assert(pack.vz_GET() == 1.9205513E38F);
            assert(pack.x_GET() == 2.2030145E38F);
            assert(pack.y_GET() == 3.2829735E38F);
            assert(pack.vx_GET() == -2.1468726E38F);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.y_SET(3.2829735E38F) ;
        p32.vy_SET(-3.8455218E37F) ;
        p32.vz_SET(1.9205513E38F) ;
        p32.time_boot_ms_SET(4027761441L) ;
        p32.z_SET(-4.513903E37F) ;
        p32.x_SET(2.2030145E38F) ;
        p32.vx_SET(-2.1468726E38F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.hdg_GET() == (char)51938);
            assert(pack.vx_GET() == (short)21526);
            assert(pack.relative_alt_GET() == 684963116);
            assert(pack.lon_GET() == 735172273);
            assert(pack.vy_GET() == (short) -28505);
            assert(pack.vz_GET() == (short)22553);
            assert(pack.time_boot_ms_GET() == 532750018L);
            assert(pack.lat_GET() == 1963935210);
            assert(pack.alt_GET() == -821682230);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.alt_SET(-821682230) ;
        p33.time_boot_ms_SET(532750018L) ;
        p33.relative_alt_SET(684963116) ;
        p33.lat_SET(1963935210) ;
        p33.lon_SET(735172273) ;
        p33.vx_SET((short)21526) ;
        p33.vy_SET((short) -28505) ;
        p33.vz_SET((short)22553) ;
        p33.hdg_SET((char)51938) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan1_scaled_GET() == (short) -8172);
            assert(pack.rssi_GET() == (char)29);
            assert(pack.time_boot_ms_GET() == 3563349307L);
            assert(pack.chan3_scaled_GET() == (short)24105);
            assert(pack.chan5_scaled_GET() == (short) -5043);
            assert(pack.port_GET() == (char)31);
            assert(pack.chan6_scaled_GET() == (short)15365);
            assert(pack.chan2_scaled_GET() == (short) -27936);
            assert(pack.chan4_scaled_GET() == (short) -30199);
            assert(pack.chan7_scaled_GET() == (short)30592);
            assert(pack.chan8_scaled_GET() == (short)1378);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan8_scaled_SET((short)1378) ;
        p34.chan5_scaled_SET((short) -5043) ;
        p34.chan4_scaled_SET((short) -30199) ;
        p34.chan6_scaled_SET((short)15365) ;
        p34.chan1_scaled_SET((short) -8172) ;
        p34.chan3_scaled_SET((short)24105) ;
        p34.rssi_SET((char)29) ;
        p34.port_SET((char)31) ;
        p34.time_boot_ms_SET(3563349307L) ;
        p34.chan7_scaled_SET((short)30592) ;
        p34.chan2_scaled_SET((short) -27936) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.port_GET() == (char)217);
            assert(pack.rssi_GET() == (char)208);
            assert(pack.chan1_raw_GET() == (char)33308);
            assert(pack.chan7_raw_GET() == (char)19626);
            assert(pack.chan6_raw_GET() == (char)31059);
            assert(pack.chan8_raw_GET() == (char)5395);
            assert(pack.chan5_raw_GET() == (char)29251);
            assert(pack.chan3_raw_GET() == (char)9625);
            assert(pack.chan2_raw_GET() == (char)32429);
            assert(pack.time_boot_ms_GET() == 785216262L);
            assert(pack.chan4_raw_GET() == (char)46967);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan1_raw_SET((char)33308) ;
        p35.chan3_raw_SET((char)9625) ;
        p35.chan8_raw_SET((char)5395) ;
        p35.port_SET((char)217) ;
        p35.rssi_SET((char)208) ;
        p35.chan2_raw_SET((char)32429) ;
        p35.chan4_raw_SET((char)46967) ;
        p35.chan5_raw_SET((char)29251) ;
        p35.chan7_raw_SET((char)19626) ;
        p35.time_boot_ms_SET(785216262L) ;
        p35.chan6_raw_SET((char)31059) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo13_raw_TRY(ph) == (char)41912);
            assert(pack.servo1_raw_GET() == (char)44873);
            assert(pack.servo9_raw_TRY(ph) == (char)9628);
            assert(pack.servo14_raw_TRY(ph) == (char)33431);
            assert(pack.servo8_raw_GET() == (char)16178);
            assert(pack.time_usec_GET() == 2920134635L);
            assert(pack.servo2_raw_GET() == (char)12973);
            assert(pack.servo15_raw_TRY(ph) == (char)3303);
            assert(pack.servo6_raw_GET() == (char)2680);
            assert(pack.servo12_raw_TRY(ph) == (char)4988);
            assert(pack.port_GET() == (char)25);
            assert(pack.servo10_raw_TRY(ph) == (char)24325);
            assert(pack.servo4_raw_GET() == (char)47335);
            assert(pack.servo16_raw_TRY(ph) == (char)55111);
            assert(pack.servo5_raw_GET() == (char)4933);
            assert(pack.servo7_raw_GET() == (char)5569);
            assert(pack.servo3_raw_GET() == (char)11063);
            assert(pack.servo11_raw_TRY(ph) == (char)9457);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo1_raw_SET((char)44873) ;
        p36.servo13_raw_SET((char)41912, PH) ;
        p36.servo11_raw_SET((char)9457, PH) ;
        p36.servo2_raw_SET((char)12973) ;
        p36.servo14_raw_SET((char)33431, PH) ;
        p36.servo5_raw_SET((char)4933) ;
        p36.servo10_raw_SET((char)24325, PH) ;
        p36.servo6_raw_SET((char)2680) ;
        p36.servo12_raw_SET((char)4988, PH) ;
        p36.servo4_raw_SET((char)47335) ;
        p36.servo16_raw_SET((char)55111, PH) ;
        p36.servo8_raw_SET((char)16178) ;
        p36.time_usec_SET(2920134635L) ;
        p36.servo7_raw_SET((char)5569) ;
        p36.port_SET((char)25) ;
        p36.servo9_raw_SET((char)9628, PH) ;
        p36.servo3_raw_SET((char)11063) ;
        p36.servo15_raw_SET((char)3303, PH) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)133);
            assert(pack.end_index_GET() == (short)6443);
            assert(pack.start_index_GET() == (short)29696);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)110);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.end_index_SET((short)6443) ;
        p37.target_system_SET((char)133) ;
        p37.target_component_SET((char)110) ;
        p37.start_index_SET((short)29696) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_index_GET() == (short)28475);
            assert(pack.target_system_GET() == (char)246);
            assert(pack.target_component_GET() == (char)165);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.end_index_GET() == (short) -28993);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_component_SET((char)165) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p38.target_system_SET((char)246) ;
        p38.start_index_SET((short)28475) ;
        p38.end_index_SET((short) -28993) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.param1_GET() == -1.6509869E38F);
            assert(pack.seq_GET() == (char)28979);
            assert(pack.param3_GET() == 1.3564255E38F);
            assert(pack.autocontinue_GET() == (char)194);
            assert(pack.z_GET() == -1.9456337E38F);
            assert(pack.target_component_GET() == (char)64);
            assert(pack.y_GET() == 2.3731145E38F);
            assert(pack.current_GET() == (char)135);
            assert(pack.param4_GET() == 2.2259526E37F);
            assert(pack.param2_GET() == 2.4619174E38F);
            assert(pack.x_GET() == 2.7114422E38F);
            assert(pack.target_system_GET() == (char)66);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_SET_PARAMETER);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.target_system_SET((char)66) ;
        p39.command_SET(MAV_CMD.MAV_CMD_DO_SET_PARAMETER) ;
        p39.current_SET((char)135) ;
        p39.z_SET(-1.9456337E38F) ;
        p39.target_component_SET((char)64) ;
        p39.x_SET(2.7114422E38F) ;
        p39.param2_SET(2.4619174E38F) ;
        p39.autocontinue_SET((char)194) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p39.param1_SET(-1.6509869E38F) ;
        p39.param3_SET(1.3564255E38F) ;
        p39.seq_SET((char)28979) ;
        p39.param4_SET(2.2259526E37F) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p39.y_SET(2.3731145E38F) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)33244);
            assert(pack.target_component_GET() == (char)140);
            assert(pack.target_system_GET() == (char)9);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p40.seq_SET((char)33244) ;
        p40.target_component_SET((char)140) ;
        p40.target_system_SET((char)9) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)213);
            assert(pack.target_component_GET() == (char)239);
            assert(pack.seq_GET() == (char)45882);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.seq_SET((char)45882) ;
        p41.target_component_SET((char)239) ;
        p41.target_system_SET((char)213) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)62410);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)62410) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)38);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)6);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_component_SET((char)6) ;
        p43.target_system_SET((char)38) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)94);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)226);
            assert(pack.count_GET() == (char)51038);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.target_system_SET((char)226) ;
        p44.target_component_SET((char)94) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p44.count_SET((char)51038) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)125);
            assert(pack.target_component_GET() == (char)60);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_component_SET((char)60) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p45.target_system_SET((char)125) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)30031);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)30031) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)168);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED_FRAME);
            assert(pack.target_system_GET() == (char)231);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p47.target_system_SET((char)231) ;
        p47.target_component_SET((char)168) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED_FRAME) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 2780221391228827936L);
            assert(pack.latitude_GET() == -1905215263);
            assert(pack.target_system_GET() == (char)22);
            assert(pack.longitude_GET() == 1011544238);
            assert(pack.altitude_GET() == -494887162);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.latitude_SET(-1905215263) ;
        p48.time_usec_SET(2780221391228827936L, PH) ;
        p48.altitude_SET(-494887162) ;
        p48.longitude_SET(1011544238) ;
        p48.target_system_SET((char)22) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 7203306558601167172L);
            assert(pack.latitude_GET() == -381559845);
            assert(pack.altitude_GET() == -454963084);
            assert(pack.longitude_GET() == 1275252266);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.time_usec_SET(7203306558601167172L, PH) ;
        p49.latitude_SET(-381559845) ;
        p49.altitude_SET(-454963084) ;
        p49.longitude_SET(1275252266) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.scale_GET() == -2.4285889E37F);
            assert(pack.param_index_GET() == (short) -31135);
            assert(pack.param_value_max_GET() == -2.9031164E38F);
            assert(pack.param_value_min_GET() == -2.1031885E38F);
            assert(pack.target_component_GET() == (char)244);
            assert(pack.param_value0_GET() == 1.816676E38F);
            assert(pack.target_system_GET() == (char)174);
            assert(pack.param_id_LEN(ph) == 3);
            assert(pack.param_id_TRY(ph).equals("ugj"));
            assert(pack.parameter_rc_channel_index_GET() == (char)10);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.target_system_SET((char)174) ;
        p50.param_value_min_SET(-2.1031885E38F) ;
        p50.param_value0_SET(1.816676E38F) ;
        p50.scale_SET(-2.4285889E37F) ;
        p50.param_id_SET("ugj", PH) ;
        p50.parameter_rc_channel_index_SET((char)10) ;
        p50.param_index_SET((short) -31135) ;
        p50.target_component_SET((char)244) ;
        p50.param_value_max_SET(-2.9031164E38F) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)52616);
            assert(pack.target_system_GET() == (char)141);
            assert(pack.target_component_GET() == (char)118);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.seq_SET((char)52616) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p51.target_system_SET((char)141) ;
        p51.target_component_SET((char)118) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2x_GET() == -4.010268E37F);
            assert(pack.target_system_GET() == (char)101);
            assert(pack.p2z_GET() == -7.2005373E37F);
            assert(pack.p1z_GET() == 3.1795608E38F);
            assert(pack.p2y_GET() == -5.1227357E35F);
            assert(pack.p1y_GET() == 5.490837E36F);
            assert(pack.p1x_GET() == -5.1545214E37F);
            assert(pack.target_component_GET() == (char)123);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p2y_SET(-5.1227357E35F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p54.target_component_SET((char)123) ;
        p54.p1x_SET(-5.1545214E37F) ;
        p54.p1y_SET(5.490837E36F) ;
        p54.p2x_SET(-4.010268E37F) ;
        p54.target_system_SET((char)101) ;
        p54.p2z_SET(-7.2005373E37F) ;
        p54.p1z_SET(3.1795608E38F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2x_GET() == -2.4812539E38F);
            assert(pack.p1y_GET() == 2.5977726E38F);
            assert(pack.p1x_GET() == 2.5336496E38F);
            assert(pack.p1z_GET() == -2.3464807E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.p2y_GET() == -1.4839049E38F);
            assert(pack.p2z_GET() == 1.716607E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p1z_SET(-2.3464807E38F) ;
        p55.p2x_SET(-2.4812539E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p55.p2y_SET(-1.4839049E38F) ;
        p55.p1y_SET(2.5977726E38F) ;
        p55.p2z_SET(1.716607E38F) ;
        p55.p1x_SET(2.5336496E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5585901905647819120L);
            assert(pack.yawspeed_GET() == -2.6619131E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.6779435E38F, 3.4006249E38F, -7.928777E37F, -2.9342655E38F}));
            assert(pack.pitchspeed_GET() == -3.727353E37F);
            assert(pack.rollspeed_GET() == 2.7853977E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {1.5070254E37F, -1.2753118E38F, 1.950586E38F, 1.2928457E38F, 2.3055982E38F, 2.629641E38F, -1.9044717E37F, 3.187062E38F, -1.6229494E35F}));
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.time_usec_SET(5585901905647819120L) ;
        p61.rollspeed_SET(2.7853977E38F) ;
        p61.q_SET(new float[] {2.6779435E38F, 3.4006249E38F, -7.928777E37F, -2.9342655E38F}, 0) ;
        p61.pitchspeed_SET(-3.727353E37F) ;
        p61.covariance_SET(new float[] {1.5070254E37F, -1.2753118E38F, 1.950586E38F, 1.2928457E38F, 2.3055982E38F, 2.629641E38F, -1.9044717E37F, 3.187062E38F, -1.6229494E35F}, 0) ;
        p61.yawspeed_SET(-2.6619131E37F) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.nav_pitch_GET() == -2.9194318E38F);
            assert(pack.wp_dist_GET() == (char)12986);
            assert(pack.aspd_error_GET() == 3.2984666E38F);
            assert(pack.target_bearing_GET() == (short) -29883);
            assert(pack.nav_roll_GET() == -1.4208628E38F);
            assert(pack.nav_bearing_GET() == (short) -15429);
            assert(pack.xtrack_error_GET() == 1.2207311E38F);
            assert(pack.alt_error_GET() == 2.153238E38F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.aspd_error_SET(3.2984666E38F) ;
        p62.nav_bearing_SET((short) -15429) ;
        p62.target_bearing_SET((short) -29883) ;
        p62.nav_roll_SET(-1.4208628E38F) ;
        p62.xtrack_error_SET(1.2207311E38F) ;
        p62.alt_error_SET(2.153238E38F) ;
        p62.nav_pitch_SET(-2.9194318E38F) ;
        p62.wp_dist_SET((char)12986) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 496190867327411512L);
            assert(pack.lat_GET() == -908041910);
            assert(pack.alt_GET() == 99005841);
            assert(pack.vz_GET() == 1.0512356E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-3.1477176E38F, -1.1665835E37F, 1.702531E38F, 2.2785435E38F, -2.1667416E38F, -1.54995E38F, 2.9387997E38F, 3.2205787E38F, 1.844497E38F, -2.3085368E38F, -1.4838916E38F, -2.8629913E38F, 1.0092987E38F, 3.5865953E37F, -1.2841253E38F, 1.5827617E38F, -2.7063598E38F, 2.1304696E38F, -1.0980512E38F, 2.12033E38F, 5.0222076E37F, -1.601603E38F, 2.816617E38F, -2.838375E38F, 2.0758914E38F, -8.1532407E37F, -1.1454354E38F, -3.104673E38F, 2.6823637E38F, -1.7046132E38F, -1.1707237E38F, 1.3250505E38F, -1.2695017E38F, -2.2772045E38F, 1.39605E38F, -1.7991175E38F}));
            assert(pack.vx_GET() == -7.452276E37F);
            assert(pack.vy_GET() == -9.04881E37F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
            assert(pack.relative_alt_GET() == -1981514386);
            assert(pack.lon_GET() == -1578447390);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.lat_SET(-908041910) ;
        p63.lon_SET(-1578447390) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE) ;
        p63.relative_alt_SET(-1981514386) ;
        p63.vz_SET(1.0512356E38F) ;
        p63.alt_SET(99005841) ;
        p63.vy_SET(-9.04881E37F) ;
        p63.time_usec_SET(496190867327411512L) ;
        p63.covariance_SET(new float[] {-3.1477176E38F, -1.1665835E37F, 1.702531E38F, 2.2785435E38F, -2.1667416E38F, -1.54995E38F, 2.9387997E38F, 3.2205787E38F, 1.844497E38F, -2.3085368E38F, -1.4838916E38F, -2.8629913E38F, 1.0092987E38F, 3.5865953E37F, -1.2841253E38F, 1.5827617E38F, -2.7063598E38F, 2.1304696E38F, -1.0980512E38F, 2.12033E38F, 5.0222076E37F, -1.601603E38F, 2.816617E38F, -2.838375E38F, 2.0758914E38F, -8.1532407E37F, -1.1454354E38F, -3.104673E38F, 2.6823637E38F, -1.7046132E38F, -1.1707237E38F, 1.3250505E38F, -1.2695017E38F, -2.2772045E38F, 1.39605E38F, -1.7991175E38F}, 0) ;
        p63.vx_SET(-7.452276E37F) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 1.2433575E38F);
            assert(pack.time_usec_GET() == 1797221746800308849L);
            assert(pack.y_GET() == -2.0171071E38F);
            assert(pack.vz_GET() == 1.8716689E38F);
            assert(pack.ax_GET() == 1.1690218E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
            assert(pack.z_GET() == -3.3980368E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.1512018E37F, 1.9545243E38F, -2.229566E38F, 2.097179E38F, 2.527988E38F, -1.1336908E38F, -3.170163E38F, -8.777617E37F, -1.4309971E38F, -1.8030553E38F, -1.9594017E38F, -2.9764462E38F, -2.5460509E38F, 1.0067673E37F, 5.541788E37F, 1.3154326E38F, 2.2792434E37F, -2.8663894E38F, -2.5893948E38F, -3.061742E38F, 2.9357924E38F, -9.18009E37F, 1.46174E38F, 1.7295731E38F, 2.0608656E38F, -2.343232E38F, 3.3847466E38F, 1.1512509E38F, 9.309515E37F, 3.0983178E38F, 1.1751314E38F, -2.9833214E37F, 2.1196747E38F, 1.497766E38F, -3.125856E38F, -9.935702E37F, 2.4304342E38F, 1.3103157E38F, 6.6304713E37F, -3.2415465E38F, 3.1378447E38F, 1.2208112E38F, -1.6941885E38F, 4.3956256E37F, 8.6097E37F}));
            assert(pack.az_GET() == 5.6433807E37F);
            assert(pack.ay_GET() == -1.6981554E37F);
            assert(pack.vx_GET() == 2.8374097E38F);
            assert(pack.vy_GET() == 2.941475E38F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.vx_SET(2.8374097E38F) ;
        p64.z_SET(-3.3980368E38F) ;
        p64.time_usec_SET(1797221746800308849L) ;
        p64.covariance_SET(new float[] {-2.1512018E37F, 1.9545243E38F, -2.229566E38F, 2.097179E38F, 2.527988E38F, -1.1336908E38F, -3.170163E38F, -8.777617E37F, -1.4309971E38F, -1.8030553E38F, -1.9594017E38F, -2.9764462E38F, -2.5460509E38F, 1.0067673E37F, 5.541788E37F, 1.3154326E38F, 2.2792434E37F, -2.8663894E38F, -2.5893948E38F, -3.061742E38F, 2.9357924E38F, -9.18009E37F, 1.46174E38F, 1.7295731E38F, 2.0608656E38F, -2.343232E38F, 3.3847466E38F, 1.1512509E38F, 9.309515E37F, 3.0983178E38F, 1.1751314E38F, -2.9833214E37F, 2.1196747E38F, 1.497766E38F, -3.125856E38F, -9.935702E37F, 2.4304342E38F, 1.3103157E38F, 6.6304713E37F, -3.2415465E38F, 3.1378447E38F, 1.2208112E38F, -1.6941885E38F, 4.3956256E37F, 8.6097E37F}, 0) ;
        p64.ax_SET(1.1690218E38F) ;
        p64.y_SET(-2.0171071E38F) ;
        p64.az_SET(5.6433807E37F) ;
        p64.vy_SET(2.941475E38F) ;
        p64.x_SET(1.2433575E38F) ;
        p64.ay_SET(-1.6981554E37F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS) ;
        p64.vz_SET(1.8716689E38F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan10_raw_GET() == (char)49260);
            assert(pack.chan11_raw_GET() == (char)27864);
            assert(pack.chan15_raw_GET() == (char)10172);
            assert(pack.chancount_GET() == (char)124);
            assert(pack.chan16_raw_GET() == (char)19651);
            assert(pack.chan1_raw_GET() == (char)22423);
            assert(pack.chan13_raw_GET() == (char)27620);
            assert(pack.chan6_raw_GET() == (char)12473);
            assert(pack.chan9_raw_GET() == (char)725);
            assert(pack.chan2_raw_GET() == (char)61924);
            assert(pack.chan14_raw_GET() == (char)57748);
            assert(pack.chan5_raw_GET() == (char)23701);
            assert(pack.chan12_raw_GET() == (char)17406);
            assert(pack.time_boot_ms_GET() == 281299780L);
            assert(pack.rssi_GET() == (char)163);
            assert(pack.chan3_raw_GET() == (char)64153);
            assert(pack.chan8_raw_GET() == (char)17170);
            assert(pack.chan7_raw_GET() == (char)15316);
            assert(pack.chan17_raw_GET() == (char)55380);
            assert(pack.chan4_raw_GET() == (char)2975);
            assert(pack.chan18_raw_GET() == (char)37209);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan3_raw_SET((char)64153) ;
        p65.chan16_raw_SET((char)19651) ;
        p65.chan9_raw_SET((char)725) ;
        p65.chan5_raw_SET((char)23701) ;
        p65.chan18_raw_SET((char)37209) ;
        p65.chan6_raw_SET((char)12473) ;
        p65.chan7_raw_SET((char)15316) ;
        p65.chan12_raw_SET((char)17406) ;
        p65.time_boot_ms_SET(281299780L) ;
        p65.chan1_raw_SET((char)22423) ;
        p65.chan2_raw_SET((char)61924) ;
        p65.chan10_raw_SET((char)49260) ;
        p65.chan4_raw_SET((char)2975) ;
        p65.chan8_raw_SET((char)17170) ;
        p65.chan15_raw_SET((char)10172) ;
        p65.chan17_raw_SET((char)55380) ;
        p65.rssi_SET((char)163) ;
        p65.chan11_raw_SET((char)27864) ;
        p65.chancount_SET((char)124) ;
        p65.chan13_raw_SET((char)27620) ;
        p65.chan14_raw_SET((char)57748) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.req_stream_id_GET() == (char)252);
            assert(pack.target_system_GET() == (char)0);
            assert(pack.target_component_GET() == (char)1);
            assert(pack.start_stop_GET() == (char)34);
            assert(pack.req_message_rate_GET() == (char)63201);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.req_stream_id_SET((char)252) ;
        p66.start_stop_SET((char)34) ;
        p66.req_message_rate_SET((char)63201) ;
        p66.target_system_SET((char)0) ;
        p66.target_component_SET((char)1) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.on_off_GET() == (char)35);
            assert(pack.message_rate_GET() == (char)21965);
            assert(pack.stream_id_GET() == (char)222);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.stream_id_SET((char)222) ;
        p67.message_rate_SET((char)21965) ;
        p67.on_off_SET((char)35) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == (short)6028);
            assert(pack.buttons_GET() == (char)41975);
            assert(pack.r_GET() == (short) -16738);
            assert(pack.target_GET() == (char)157);
            assert(pack.z_GET() == (short)5799);
            assert(pack.y_GET() == (short) -15140);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.z_SET((short)5799) ;
        p69.buttons_SET((char)41975) ;
        p69.x_SET((short)6028) ;
        p69.y_SET((short) -15140) ;
        p69.target_SET((char)157) ;
        p69.r_SET((short) -16738) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan8_raw_GET() == (char)12952);
            assert(pack.chan3_raw_GET() == (char)50041);
            assert(pack.chan2_raw_GET() == (char)29354);
            assert(pack.chan7_raw_GET() == (char)23341);
            assert(pack.target_component_GET() == (char)27);
            assert(pack.chan5_raw_GET() == (char)4621);
            assert(pack.target_system_GET() == (char)84);
            assert(pack.chan1_raw_GET() == (char)26015);
            assert(pack.chan4_raw_GET() == (char)42778);
            assert(pack.chan6_raw_GET() == (char)31939);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan1_raw_SET((char)26015) ;
        p70.chan3_raw_SET((char)50041) ;
        p70.chan5_raw_SET((char)4621) ;
        p70.target_system_SET((char)84) ;
        p70.target_component_SET((char)27) ;
        p70.chan6_raw_SET((char)31939) ;
        p70.chan2_raw_SET((char)29354) ;
        p70.chan7_raw_SET((char)23341) ;
        p70.chan8_raw_SET((char)12952) ;
        p70.chan4_raw_SET((char)42778) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)51716);
            assert(pack.current_GET() == (char)214);
            assert(pack.target_component_GET() == (char)205);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_PATHPLANNING);
            assert(pack.param1_GET() == 3.33311E38F);
            assert(pack.param2_GET() == 3.2546187E38F);
            assert(pack.z_GET() == -6.10709E37F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.param3_GET() == 5.0671797E37F);
            assert(pack.autocontinue_GET() == (char)65);
            assert(pack.x_GET() == -1326096956);
            assert(pack.param4_GET() == -1.0585564E38F);
            assert(pack.y_GET() == 224511022);
            assert(pack.target_system_GET() == (char)50);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.y_SET(224511022) ;
        p73.autocontinue_SET((char)65) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p73.target_system_SET((char)50) ;
        p73.param1_SET(3.33311E38F) ;
        p73.z_SET(-6.10709E37F) ;
        p73.seq_SET((char)51716) ;
        p73.current_SET((char)214) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p73.target_component_SET((char)205) ;
        p73.command_SET(MAV_CMD.MAV_CMD_NAV_PATHPLANNING) ;
        p73.param3_SET(5.0671797E37F) ;
        p73.x_SET(-1326096956) ;
        p73.param4_SET(-1.0585564E38F) ;
        p73.param2_SET(3.2546187E38F) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.heading_GET() == (short)32623);
            assert(pack.climb_GET() == 2.304422E38F);
            assert(pack.throttle_GET() == (char)24710);
            assert(pack.groundspeed_GET() == -5.7443977E37F);
            assert(pack.airspeed_GET() == 4.473565E37F);
            assert(pack.alt_GET() == -1.2852424E37F);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.climb_SET(2.304422E38F) ;
        p74.alt_SET(-1.2852424E37F) ;
        p74.heading_SET((short)32623) ;
        p74.airspeed_SET(4.473565E37F) ;
        p74.groundspeed_SET(-5.7443977E37F) ;
        p74.throttle_SET((char)24710) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 1294217570);
            assert(pack.target_system_GET() == (char)36);
            assert(pack.param3_GET() == -3.0596995E38F);
            assert(pack.param1_GET() == 1.4474588E38F);
            assert(pack.z_GET() == -1.2600157E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.y_GET() == -2000153181);
            assert(pack.param4_GET() == -4.4562376E37F);
            assert(pack.target_component_GET() == (char)148);
            assert(pack.current_GET() == (char)16);
            assert(pack.autocontinue_GET() == (char)125);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION);
            assert(pack.param2_GET() == -1.3235541E38F);
        });
        GroundControl.COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
        PH.setPack(p75);
        p75.z_SET(-1.2600157E38F) ;
        p75.command_SET(MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION) ;
        p75.param2_SET(-1.3235541E38F) ;
        p75.param4_SET(-4.4562376E37F) ;
        p75.target_system_SET((char)36) ;
        p75.y_SET(-2000153181) ;
        p75.param3_SET(-3.0596995E38F) ;
        p75.current_SET((char)16) ;
        p75.target_component_SET((char)148) ;
        p75.param1_SET(1.4474588E38F) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p75.autocontinue_SET((char)125) ;
        p75.x_SET(1294217570) ;
        CommunicationChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param2_GET() == 1.0449558E38F);
            assert(pack.param5_GET() == 4.979877E37F);
            assert(pack.confirmation_GET() == (char)185);
            assert(pack.target_system_GET() == (char)180);
            assert(pack.param6_GET() == -1.0560252E38F);
            assert(pack.param7_GET() == 1.9321877E38F);
            assert(pack.target_component_GET() == (char)107);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_JUMP);
            assert(pack.param4_GET() == 2.9808976E37F);
            assert(pack.param1_GET() == -3.3937791E38F);
            assert(pack.param3_GET() == 1.925081E37F);
        });
        GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.target_component_SET((char)107) ;
        p76.confirmation_SET((char)185) ;
        p76.param5_SET(4.979877E37F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_DO_JUMP) ;
        p76.param3_SET(1.925081E37F) ;
        p76.param7_SET(1.9321877E38F) ;
        p76.param1_SET(-3.3937791E38F) ;
        p76.param4_SET(2.9808976E37F) ;
        p76.param2_SET(1.0449558E38F) ;
        p76.target_system_SET((char)180) ;
        p76.param6_SET(-1.0560252E38F) ;
        CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED);
            assert(pack.target_component_TRY(ph) == (char)164);
            assert(pack.target_system_TRY(ph) == (char)185);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_GO_AROUND);
            assert(pack.progress_TRY(ph) == (char)139);
            assert(pack.result_param2_TRY(ph) == -23549342);
        });
        GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.progress_SET((char)139, PH) ;
        p77.target_system_SET((char)185, PH) ;
        p77.result_param2_SET(-23549342, PH) ;
        p77.target_component_SET((char)164, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_DO_GO_AROUND) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED) ;
        CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -9.267411E37F);
            assert(pack.thrust_GET() == -2.5416502E38F);
            assert(pack.roll_GET() == 2.6382363E38F);
            assert(pack.time_boot_ms_GET() == 2191062499L);
            assert(pack.pitch_GET() == 3.0230777E38F);
            assert(pack.mode_switch_GET() == (char)250);
            assert(pack.manual_override_switch_GET() == (char)28);
        });
        GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.time_boot_ms_SET(2191062499L) ;
        p81.thrust_SET(-2.5416502E38F) ;
        p81.manual_override_switch_SET((char)28) ;
        p81.roll_SET(2.6382363E38F) ;
        p81.pitch_SET(3.0230777E38F) ;
        p81.yaw_SET(-9.267411E37F) ;
        p81.mode_switch_SET((char)250) ;
        CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_yaw_rate_GET() == -3.2570193E38F);
            assert(pack.target_component_GET() == (char)104);
            assert(pack.body_pitch_rate_GET() == -6.4965785E37F);
            assert(pack.body_roll_rate_GET() == 6.960278E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {4.7200975E37F, -2.8455286E38F, -2.2576E38F, -3.101193E38F}));
            assert(pack.target_system_GET() == (char)243);
            assert(pack.thrust_GET() == 3.0577671E38F);
            assert(pack.type_mask_GET() == (char)121);
            assert(pack.time_boot_ms_GET() == 2008400634L);
        });
        GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.body_roll_rate_SET(6.960278E37F) ;
        p82.body_yaw_rate_SET(-3.2570193E38F) ;
        p82.q_SET(new float[] {4.7200975E37F, -2.8455286E38F, -2.2576E38F, -3.101193E38F}, 0) ;
        p82.thrust_SET(3.0577671E38F) ;
        p82.time_boot_ms_SET(2008400634L) ;
        p82.body_pitch_rate_SET(-6.4965785E37F) ;
        p82.target_system_SET((char)243) ;
        p82.type_mask_SET((char)121) ;
        p82.target_component_SET((char)104) ;
        CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_roll_rate_GET() == 2.9209941E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.5543313E38F, 6.5024477E37F, -3.124946E38F, -3.137371E38F}));
            assert(pack.time_boot_ms_GET() == 4177979719L);
            assert(pack.body_pitch_rate_GET() == 2.5144738E38F);
            assert(pack.body_yaw_rate_GET() == -8.269196E37F);
            assert(pack.thrust_GET() == 1.2509073E38F);
            assert(pack.type_mask_GET() == (char)154);
        });
        GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.body_pitch_rate_SET(2.5144738E38F) ;
        p83.time_boot_ms_SET(4177979719L) ;
        p83.body_roll_rate_SET(2.9209941E38F) ;
        p83.q_SET(new float[] {-1.5543313E38F, 6.5024477E37F, -3.124946E38F, -3.137371E38F}, 0) ;
        p83.body_yaw_rate_SET(-8.269196E37F) ;
        p83.type_mask_SET((char)154) ;
        p83.thrust_SET(1.2509073E38F) ;
        CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.afz_GET() == -1.9654489E38F);
            assert(pack.vy_GET() == -1.3117005E38F);
            assert(pack.target_system_GET() == (char)65);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.yaw_GET() == 1.8936238E38F);
            assert(pack.target_component_GET() == (char)242);
            assert(pack.vx_GET() == 3.541191E37F);
            assert(pack.x_GET() == -2.0466363E38F);
            assert(pack.y_GET() == 6.8090787E37F);
            assert(pack.vz_GET() == -8.703832E37F);
            assert(pack.afx_GET() == 3.0120697E38F);
            assert(pack.afy_GET() == 2.6279182E38F);
            assert(pack.type_mask_GET() == (char)50840);
            assert(pack.yaw_rate_GET() == 1.901654E38F);
            assert(pack.time_boot_ms_GET() == 4064326251L);
            assert(pack.z_GET() == -5.4801534E37F);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.afx_SET(3.0120697E38F) ;
        p84.yaw_SET(1.8936238E38F) ;
        p84.x_SET(-2.0466363E38F) ;
        p84.yaw_rate_SET(1.901654E38F) ;
        p84.afz_SET(-1.9654489E38F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p84.target_system_SET((char)65) ;
        p84.y_SET(6.8090787E37F) ;
        p84.afy_SET(2.6279182E38F) ;
        p84.target_component_SET((char)242) ;
        p84.type_mask_SET((char)50840) ;
        p84.vz_SET(-8.703832E37F) ;
        p84.vx_SET(3.541191E37F) ;
        p84.vy_SET(-1.3117005E38F) ;
        p84.z_SET(-5.4801534E37F) ;
        p84.time_boot_ms_SET(4064326251L) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.lat_int_GET() == -389261108);
            assert(pack.type_mask_GET() == (char)65345);
            assert(pack.afy_GET() == -2.7537718E38F);
            assert(pack.yaw_rate_GET() == 3.0570354E38F);
            assert(pack.alt_GET() == -1.0935647E38F);
            assert(pack.time_boot_ms_GET() == 202286094L);
            assert(pack.vx_GET() == -1.1864169E38F);
            assert(pack.afz_GET() == 4.803436E37F);
            assert(pack.afx_GET() == -2.8481229E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.lon_int_GET() == -1345776021);
            assert(pack.vz_GET() == 6.7218157E37F);
            assert(pack.yaw_GET() == 1.86805E38F);
            assert(pack.target_system_GET() == (char)159);
            assert(pack.vy_GET() == -1.6938384E38F);
            assert(pack.target_component_GET() == (char)199);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.vz_SET(6.7218157E37F) ;
        p86.vx_SET(-1.1864169E38F) ;
        p86.yaw_rate_SET(3.0570354E38F) ;
        p86.target_component_SET((char)199) ;
        p86.type_mask_SET((char)65345) ;
        p86.lon_int_SET(-1345776021) ;
        p86.time_boot_ms_SET(202286094L) ;
        p86.vy_SET(-1.6938384E38F) ;
        p86.lat_int_SET(-389261108) ;
        p86.afy_SET(-2.7537718E38F) ;
        p86.target_system_SET((char)159) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p86.afx_SET(-2.8481229E38F) ;
        p86.yaw_SET(1.86805E38F) ;
        p86.afz_SET(4.803436E37F) ;
        p86.alt_SET(-1.0935647E38F) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -2.0643114E38F);
            assert(pack.lat_int_GET() == 1218715389);
            assert(pack.yaw_rate_GET() == 3.1592461E38F);
            assert(pack.afz_GET() == 2.8450968E38F);
            assert(pack.vx_GET() == -2.6704048E38F);
            assert(pack.time_boot_ms_GET() == 83199061L);
            assert(pack.lon_int_GET() == 812793814);
            assert(pack.type_mask_GET() == (char)33372);
            assert(pack.afy_GET() == -1.6197764E38F);
            assert(pack.alt_GET() == -1.5558199E38F);
            assert(pack.afx_GET() == -2.3778636E37F);
            assert(pack.vy_GET() == 2.4158776E38F);
            assert(pack.vz_GET() == 2.044126E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.vy_SET(2.4158776E38F) ;
        p87.lon_int_SET(812793814) ;
        p87.vz_SET(2.044126E38F) ;
        p87.time_boot_ms_SET(83199061L) ;
        p87.afx_SET(-2.3778636E37F) ;
        p87.vx_SET(-2.6704048E38F) ;
        p87.yaw_SET(-2.0643114E38F) ;
        p87.afz_SET(2.8450968E38F) ;
        p87.alt_SET(-1.5558199E38F) ;
        p87.yaw_rate_SET(3.1592461E38F) ;
        p87.lat_int_SET(1218715389) ;
        p87.afy_SET(-1.6197764E38F) ;
        p87.type_mask_SET((char)33372) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -5.176562E37F);
            assert(pack.z_GET() == 1.1435718E38F);
            assert(pack.yaw_GET() == 1.4898698E38F);
            assert(pack.y_GET() == -8.3459433E37F);
            assert(pack.time_boot_ms_GET() == 3656239461L);
            assert(pack.pitch_GET() == -1.9635997E38F);
            assert(pack.x_GET() == -9.076625E37F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.z_SET(1.1435718E38F) ;
        p89.yaw_SET(1.4898698E38F) ;
        p89.roll_SET(-5.176562E37F) ;
        p89.pitch_SET(-1.9635997E38F) ;
        p89.time_boot_ms_SET(3656239461L) ;
        p89.x_SET(-9.076625E37F) ;
        p89.y_SET(-8.3459433E37F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4030796174218363186L);
            assert(pack.yacc_GET() == (short)21646);
            assert(pack.rollspeed_GET() == 2.1062763E38F);
            assert(pack.pitch_GET() == -2.0184677E38F);
            assert(pack.vz_GET() == (short)26841);
            assert(pack.alt_GET() == -624291587);
            assert(pack.vy_GET() == (short) -26967);
            assert(pack.lat_GET() == 1128718516);
            assert(pack.vx_GET() == (short) -3853);
            assert(pack.yaw_GET() == -1.6541627E38F);
            assert(pack.pitchspeed_GET() == -3.2544633E38F);
            assert(pack.xacc_GET() == (short)7983);
            assert(pack.roll_GET() == -9.636548E37F);
            assert(pack.lon_GET() == -1272145420);
            assert(pack.yawspeed_GET() == -3.1744239E38F);
            assert(pack.zacc_GET() == (short) -17137);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.yacc_SET((short)21646) ;
        p90.lat_SET(1128718516) ;
        p90.vx_SET((short) -3853) ;
        p90.vz_SET((short)26841) ;
        p90.rollspeed_SET(2.1062763E38F) ;
        p90.roll_SET(-9.636548E37F) ;
        p90.time_usec_SET(4030796174218363186L) ;
        p90.pitch_SET(-2.0184677E38F) ;
        p90.yawspeed_SET(-3.1744239E38F) ;
        p90.zacc_SET((short) -17137) ;
        p90.yaw_SET(-1.6541627E38F) ;
        p90.lon_SET(-1272145420) ;
        p90.alt_SET(-624291587) ;
        p90.vy_SET((short) -26967) ;
        p90.xacc_SET((short)7983) ;
        p90.pitchspeed_SET(-3.2544633E38F) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.throttle_GET() == 2.6986168E38F);
            assert(pack.aux3_GET() == -1.9156975E38F);
            assert(pack.aux4_GET() == -6.355954E37F);
            assert(pack.aux2_GET() == -2.581557E38F);
            assert(pack.pitch_elevator_GET() == 1.7731448E38F);
            assert(pack.roll_ailerons_GET() == -4.844878E37F);
            assert(pack.aux1_GET() == 1.4592096E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_MANUAL_ARMED);
            assert(pack.yaw_rudder_GET() == 1.3122797E38F);
            assert(pack.nav_mode_GET() == (char)196);
            assert(pack.time_usec_GET() == 3226653202180243180L);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.mode_SET(MAV_MODE.MAV_MODE_MANUAL_ARMED) ;
        p91.aux3_SET(-1.9156975E38F) ;
        p91.pitch_elevator_SET(1.7731448E38F) ;
        p91.aux4_SET(-6.355954E37F) ;
        p91.roll_ailerons_SET(-4.844878E37F) ;
        p91.aux1_SET(1.4592096E38F) ;
        p91.time_usec_SET(3226653202180243180L) ;
        p91.nav_mode_SET((char)196) ;
        p91.yaw_rudder_SET(1.3122797E38F) ;
        p91.aux2_SET(-2.581557E38F) ;
        p91.throttle_SET(2.6986168E38F) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan3_raw_GET() == (char)59246);
            assert(pack.chan6_raw_GET() == (char)31208);
            assert(pack.chan7_raw_GET() == (char)30756);
            assert(pack.chan4_raw_GET() == (char)45264);
            assert(pack.chan9_raw_GET() == (char)63478);
            assert(pack.time_usec_GET() == 6716680566133842296L);
            assert(pack.chan12_raw_GET() == (char)37902);
            assert(pack.chan10_raw_GET() == (char)36497);
            assert(pack.chan8_raw_GET() == (char)10144);
            assert(pack.rssi_GET() == (char)226);
            assert(pack.chan11_raw_GET() == (char)2091);
            assert(pack.chan5_raw_GET() == (char)31829);
            assert(pack.chan1_raw_GET() == (char)24512);
            assert(pack.chan2_raw_GET() == (char)18810);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan11_raw_SET((char)2091) ;
        p92.chan6_raw_SET((char)31208) ;
        p92.chan1_raw_SET((char)24512) ;
        p92.time_usec_SET(6716680566133842296L) ;
        p92.chan12_raw_SET((char)37902) ;
        p92.chan8_raw_SET((char)10144) ;
        p92.chan7_raw_SET((char)30756) ;
        p92.chan2_raw_SET((char)18810) ;
        p92.chan4_raw_SET((char)45264) ;
        p92.rssi_SET((char)226) ;
        p92.chan3_raw_SET((char)59246) ;
        p92.chan10_raw_SET((char)36497) ;
        p92.chan5_raw_SET((char)31829) ;
        p92.chan9_raw_SET((char)63478) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-1.3643531E38F, -2.0947036E38F, 8.044739E37F, 2.8890104E38F, -1.9370514E38F, 2.5634264E38F, -5.989418E37F, -2.1629162E37F, 1.9002087E38F, 3.3572515E38F, 1.3108934E38F, 5.57105E37F, -1.5343203E38F, -1.8519353E38F, 3.2984587E38F, 3.4022193E38F}));
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_ARMED);
            assert(pack.time_usec_GET() == 4844094948465398541L);
            assert(pack.flags_GET() == 1675893622041598268L);
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.mode_SET(MAV_MODE.MAV_MODE_GUIDED_ARMED) ;
        p93.flags_SET(1675893622041598268L) ;
        p93.time_usec_SET(4844094948465398541L) ;
        p93.controls_SET(new float[] {-1.3643531E38F, -2.0947036E38F, 8.044739E37F, 2.8890104E38F, -1.9370514E38F, 2.5634264E38F, -5.989418E37F, -2.1629162E37F, 1.9002087E38F, 3.3572515E38F, 1.3108934E38F, 5.57105E37F, -1.5343203E38F, -1.8519353E38F, 3.2984587E38F, 3.4022193E38F}, 0) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.ground_distance_GET() == -1.3726292E38F);
            assert(pack.time_usec_GET() == 4750302622083960205L);
            assert(pack.flow_comp_m_x_GET() == -1.1739951E38F);
            assert(pack.flow_x_GET() == (short)23297);
            assert(pack.flow_rate_y_TRY(ph) == -5.720519E37F);
            assert(pack.flow_rate_x_TRY(ph) == 9.684613E37F);
            assert(pack.flow_comp_m_y_GET() == -5.714843E37F);
            assert(pack.sensor_id_GET() == (char)22);
            assert(pack.flow_y_GET() == (short)23874);
            assert(pack.quality_GET() == (char)31);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.time_usec_SET(4750302622083960205L) ;
        p100.flow_rate_y_SET(-5.720519E37F, PH) ;
        p100.flow_rate_x_SET(9.684613E37F, PH) ;
        p100.flow_comp_m_x_SET(-1.1739951E38F) ;
        p100.flow_y_SET((short)23874) ;
        p100.flow_x_SET((short)23297) ;
        p100.ground_distance_SET(-1.3726292E38F) ;
        p100.flow_comp_m_y_SET(-5.714843E37F) ;
        p100.quality_SET((char)31) ;
        p100.sensor_id_SET((char)22) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -4.866577E37F);
            assert(pack.y_GET() == -2.3726714E38F);
            assert(pack.roll_GET() == 7.419956E36F);
            assert(pack.usec_GET() == 1013673002424203570L);
            assert(pack.yaw_GET() == 3.5656347E37F);
            assert(pack.x_GET() == 3.0776086E38F);
            assert(pack.pitch_GET() == -9.833349E37F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.z_SET(-4.866577E37F) ;
        p101.usec_SET(1013673002424203570L) ;
        p101.yaw_SET(3.5656347E37F) ;
        p101.pitch_SET(-9.833349E37F) ;
        p101.x_SET(3.0776086E38F) ;
        p101.y_SET(-2.3726714E38F) ;
        p101.roll_SET(7.419956E36F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -4.5751244E37F);
            assert(pack.usec_GET() == 6113475726845256358L);
            assert(pack.z_GET() == 1.577808E38F);
            assert(pack.y_GET() == -2.7685844E38F);
            assert(pack.roll_GET() == 3.3419172E38F);
            assert(pack.yaw_GET() == 2.5447611E38F);
            assert(pack.pitch_GET() == 1.9047801E38F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.z_SET(1.577808E38F) ;
        p102.yaw_SET(2.5447611E38F) ;
        p102.roll_SET(3.3419172E38F) ;
        p102.y_SET(-2.7685844E38F) ;
        p102.usec_SET(6113475726845256358L) ;
        p102.pitch_SET(1.9047801E38F) ;
        p102.x_SET(-4.5751244E37F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -3.2592315E38F);
            assert(pack.z_GET() == 1.6128563E36F);
            assert(pack.x_GET() == -2.8530329E38F);
            assert(pack.usec_GET() == 3661577802867930868L);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.y_SET(-3.2592315E38F) ;
        p103.x_SET(-2.8530329E38F) ;
        p103.usec_SET(3661577802867930868L) ;
        p103.z_SET(1.6128563E36F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 7482204219862068974L);
            assert(pack.y_GET() == 1.1347646E38F);
            assert(pack.roll_GET() == 2.4278815E37F);
            assert(pack.pitch_GET() == 4.9325436E37F);
            assert(pack.yaw_GET() == 2.379421E38F);
            assert(pack.x_GET() == -2.6662276E38F);
            assert(pack.z_GET() == -1.4257327E37F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(7482204219862068974L) ;
        p104.roll_SET(2.4278815E37F) ;
        p104.y_SET(1.1347646E38F) ;
        p104.pitch_SET(4.9325436E37F) ;
        p104.z_SET(-1.4257327E37F) ;
        p104.x_SET(-2.6662276E38F) ;
        p104.yaw_SET(2.379421E38F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == 3.137236E38F);
            assert(pack.xacc_GET() == 1.9145382E38F);
            assert(pack.diff_pressure_GET() == 2.2306237E38F);
            assert(pack.zacc_GET() == -2.1851838E38F);
            assert(pack.xmag_GET() == -2.2042838E38F);
            assert(pack.ygyro_GET() == -1.8629675E38F);
            assert(pack.xgyro_GET() == 2.412825E38F);
            assert(pack.fields_updated_GET() == (char)64044);
            assert(pack.abs_pressure_GET() == -3.2994022E38F);
            assert(pack.time_usec_GET() == 2732123218135632908L);
            assert(pack.zmag_GET() == -2.7349878E38F);
            assert(pack.yacc_GET() == -1.6663512E38F);
            assert(pack.temperature_GET() == -8.788715E37F);
            assert(pack.ymag_GET() == 1.9706318E38F);
            assert(pack.pressure_alt_GET() == -1.0518023E38F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.zacc_SET(-2.1851838E38F) ;
        p105.fields_updated_SET((char)64044) ;
        p105.time_usec_SET(2732123218135632908L) ;
        p105.temperature_SET(-8.788715E37F) ;
        p105.xmag_SET(-2.2042838E38F) ;
        p105.diff_pressure_SET(2.2306237E38F) ;
        p105.abs_pressure_SET(-3.2994022E38F) ;
        p105.pressure_alt_SET(-1.0518023E38F) ;
        p105.xgyro_SET(2.412825E38F) ;
        p105.ymag_SET(1.9706318E38F) ;
        p105.ygyro_SET(-1.8629675E38F) ;
        p105.yacc_SET(-1.6663512E38F) ;
        p105.zmag_SET(-2.7349878E38F) ;
        p105.xacc_SET(1.9145382E38F) ;
        p105.zgyro_SET(3.137236E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.sensor_id_GET() == (char)239);
            assert(pack.quality_GET() == (char)132);
            assert(pack.temperature_GET() == (short) -3238);
            assert(pack.time_usec_GET() == 4629552071168324810L);
            assert(pack.distance_GET() == -8.997741E34F);
            assert(pack.integrated_xgyro_GET() == -9.393211E37F);
            assert(pack.integration_time_us_GET() == 1246955618L);
            assert(pack.integrated_x_GET() == -2.9831451E38F);
            assert(pack.integrated_y_GET() == -4.1254104E37F);
            assert(pack.integrated_ygyro_GET() == -1.2334109E38F);
            assert(pack.integrated_zgyro_GET() == 3.3336759E38F);
            assert(pack.time_delta_distance_us_GET() == 2892030401L);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integrated_y_SET(-4.1254104E37F) ;
        p106.sensor_id_SET((char)239) ;
        p106.integration_time_us_SET(1246955618L) ;
        p106.distance_SET(-8.997741E34F) ;
        p106.temperature_SET((short) -3238) ;
        p106.integrated_xgyro_SET(-9.393211E37F) ;
        p106.integrated_x_SET(-2.9831451E38F) ;
        p106.time_usec_SET(4629552071168324810L) ;
        p106.integrated_zgyro_SET(3.3336759E38F) ;
        p106.integrated_ygyro_SET(-1.2334109E38F) ;
        p106.time_delta_distance_us_SET(2892030401L) ;
        p106.quality_SET((char)132) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3885240880531004819L);
            assert(pack.zacc_GET() == 2.0949784E38F);
            assert(pack.zgyro_GET() == 7.6554606E37F);
            assert(pack.abs_pressure_GET() == 7.2435436E37F);
            assert(pack.temperature_GET() == 2.6456545E38F);
            assert(pack.zmag_GET() == -2.8951562E38F);
            assert(pack.xacc_GET() == -1.4985302E38F);
            assert(pack.ymag_GET() == 7.8910317E37F);
            assert(pack.yacc_GET() == -2.7939524E38F);
            assert(pack.pressure_alt_GET() == -1.6239361E38F);
            assert(pack.xgyro_GET() == -7.2999014E37F);
            assert(pack.ygyro_GET() == -8.07171E37F);
            assert(pack.fields_updated_GET() == 3197057410L);
            assert(pack.xmag_GET() == -7.4721203E37F);
            assert(pack.diff_pressure_GET() == 2.3716512E38F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.zacc_SET(2.0949784E38F) ;
        p107.xmag_SET(-7.4721203E37F) ;
        p107.ygyro_SET(-8.07171E37F) ;
        p107.yacc_SET(-2.7939524E38F) ;
        p107.ymag_SET(7.8910317E37F) ;
        p107.fields_updated_SET(3197057410L) ;
        p107.diff_pressure_SET(2.3716512E38F) ;
        p107.zmag_SET(-2.8951562E38F) ;
        p107.pressure_alt_SET(-1.6239361E38F) ;
        p107.temperature_SET(2.6456545E38F) ;
        p107.zgyro_SET(7.6554606E37F) ;
        p107.xacc_SET(-1.4985302E38F) ;
        p107.abs_pressure_SET(7.2435436E37F) ;
        p107.time_usec_SET(3885240880531004819L) ;
        p107.xgyro_SET(-7.2999014E37F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == 2.1284245E38F);
            assert(pack.vn_GET() == -1.1939381E38F);
            assert(pack.zacc_GET() == -2.8533213E38F);
            assert(pack.q1_GET() == 2.2068595E37F);
            assert(pack.q2_GET() == 1.0856171E38F);
            assert(pack.std_dev_vert_GET() == -6.877594E37F);
            assert(pack.pitch_GET() == -1.7043656E38F);
            assert(pack.xgyro_GET() == -2.779046E38F);
            assert(pack.xacc_GET() == 2.2160656E38F);
            assert(pack.roll_GET() == -1.3443068E38F);
            assert(pack.lat_GET() == 9.282487E37F);
            assert(pack.q3_GET() == -3.0458614E38F);
            assert(pack.std_dev_horz_GET() == 2.0948563E38F);
            assert(pack.alt_GET() == -6.816475E37F);
            assert(pack.ygyro_GET() == -3.2048945E38F);
            assert(pack.q4_GET() == -2.0918851E38F);
            assert(pack.zgyro_GET() == -2.9171434E37F);
            assert(pack.vd_GET() == 2.8716385E38F);
            assert(pack.ve_GET() == -1.7870525E38F);
            assert(pack.lon_GET() == 3.2331994E38F);
            assert(pack.yaw_GET() == -3.1823344E37F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.ve_SET(-1.7870525E38F) ;
        p108.q2_SET(1.0856171E38F) ;
        p108.std_dev_vert_SET(-6.877594E37F) ;
        p108.lon_SET(3.2331994E38F) ;
        p108.q1_SET(2.2068595E37F) ;
        p108.std_dev_horz_SET(2.0948563E38F) ;
        p108.pitch_SET(-1.7043656E38F) ;
        p108.q4_SET(-2.0918851E38F) ;
        p108.alt_SET(-6.816475E37F) ;
        p108.zgyro_SET(-2.9171434E37F) ;
        p108.zacc_SET(-2.8533213E38F) ;
        p108.xacc_SET(2.2160656E38F) ;
        p108.ygyro_SET(-3.2048945E38F) ;
        p108.q3_SET(-3.0458614E38F) ;
        p108.vd_SET(2.8716385E38F) ;
        p108.vn_SET(-1.1939381E38F) ;
        p108.yacc_SET(2.1284245E38F) ;
        p108.yaw_SET(-3.1823344E37F) ;
        p108.roll_SET(-1.3443068E38F) ;
        p108.xgyro_SET(-2.779046E38F) ;
        p108.lat_SET(9.282487E37F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.noise_GET() == (char)10);
            assert(pack.txbuf_GET() == (char)89);
            assert(pack.remrssi_GET() == (char)174);
            assert(pack.remnoise_GET() == (char)77);
            assert(pack.rxerrors_GET() == (char)33207);
            assert(pack.rssi_GET() == (char)216);
            assert(pack.fixed__GET() == (char)44318);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.noise_SET((char)10) ;
        p109.remnoise_SET((char)77) ;
        p109.remrssi_SET((char)174) ;
        p109.fixed__SET((char)44318) ;
        p109.rxerrors_SET((char)33207) ;
        p109.rssi_SET((char)216) ;
        p109.txbuf_SET((char)89) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)223);
            assert(pack.target_component_GET() == (char)188);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)214, (char)253, (char)93, (char)2, (char)89, (char)94, (char)255, (char)54, (char)21, (char)132, (char)176, (char)238, (char)50, (char)174, (char)234, (char)157, (char)27, (char)198, (char)6, (char)139, (char)46, (char)239, (char)182, (char)227, (char)24, (char)7, (char)176, (char)3, (char)208, (char)228, (char)148, (char)102, (char)171, (char)33, (char)250, (char)110, (char)59, (char)196, (char)170, (char)14, (char)211, (char)38, (char)37, (char)13, (char)33, (char)240, (char)148, (char)239, (char)57, (char)92, (char)192, (char)85, (char)199, (char)76, (char)221, (char)214, (char)149, (char)126, (char)62, (char)220, (char)194, (char)195, (char)42, (char)181, (char)166, (char)50, (char)251, (char)44, (char)238, (char)7, (char)3, (char)33, (char)169, (char)85, (char)161, (char)132, (char)117, (char)70, (char)168, (char)231, (char)224, (char)135, (char)43, (char)42, (char)54, (char)219, (char)87, (char)60, (char)238, (char)1, (char)16, (char)247, (char)222, (char)40, (char)147, (char)39, (char)152, (char)84, (char)222, (char)87, (char)117, (char)221, (char)216, (char)93, (char)95, (char)25, (char)253, (char)237, (char)134, (char)58, (char)141, (char)177, (char)160, (char)219, (char)179, (char)157, (char)226, (char)191, (char)185, (char)101, (char)200, (char)191, (char)138, (char)191, (char)134, (char)17, (char)113, (char)137, (char)176, (char)33, (char)100, (char)132, (char)49, (char)39, (char)27, (char)56, (char)113, (char)249, (char)230, (char)248, (char)67, (char)104, (char)168, (char)68, (char)95, (char)54, (char)102, (char)146, (char)159, (char)134, (char)191, (char)38, (char)57, (char)75, (char)233, (char)194, (char)15, (char)244, (char)205, (char)222, (char)191, (char)186, (char)92, (char)17, (char)73, (char)148, (char)138, (char)80, (char)41, (char)132, (char)102, (char)190, (char)232, (char)106, (char)143, (char)36, (char)134, (char)214, (char)19, (char)146, (char)253, (char)0, (char)74, (char)120, (char)182, (char)133, (char)100, (char)8, (char)51, (char)109, (char)230, (char)38, (char)199, (char)172, (char)92, (char)201, (char)44, (char)130, (char)255, (char)201, (char)135, (char)46, (char)9, (char)211, (char)105, (char)2, (char)136, (char)90, (char)254, (char)229, (char)232, (char)162, (char)176, (char)197, (char)237, (char)172, (char)43, (char)183, (char)163, (char)220, (char)74, (char)36, (char)57, (char)116, (char)200, (char)177, (char)255, (char)253, (char)164, (char)62, (char)171, (char)140, (char)203, (char)33, (char)139, (char)35, (char)105, (char)51, (char)54, (char)42, (char)48, (char)153, (char)31, (char)87, (char)114, (char)241, (char)35, (char)62, (char)23, (char)135, (char)99}));
            assert(pack.target_network_GET() == (char)75);
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)75) ;
        p110.target_system_SET((char)223) ;
        p110.target_component_SET((char)188) ;
        p110.payload_SET(new char[] {(char)214, (char)253, (char)93, (char)2, (char)89, (char)94, (char)255, (char)54, (char)21, (char)132, (char)176, (char)238, (char)50, (char)174, (char)234, (char)157, (char)27, (char)198, (char)6, (char)139, (char)46, (char)239, (char)182, (char)227, (char)24, (char)7, (char)176, (char)3, (char)208, (char)228, (char)148, (char)102, (char)171, (char)33, (char)250, (char)110, (char)59, (char)196, (char)170, (char)14, (char)211, (char)38, (char)37, (char)13, (char)33, (char)240, (char)148, (char)239, (char)57, (char)92, (char)192, (char)85, (char)199, (char)76, (char)221, (char)214, (char)149, (char)126, (char)62, (char)220, (char)194, (char)195, (char)42, (char)181, (char)166, (char)50, (char)251, (char)44, (char)238, (char)7, (char)3, (char)33, (char)169, (char)85, (char)161, (char)132, (char)117, (char)70, (char)168, (char)231, (char)224, (char)135, (char)43, (char)42, (char)54, (char)219, (char)87, (char)60, (char)238, (char)1, (char)16, (char)247, (char)222, (char)40, (char)147, (char)39, (char)152, (char)84, (char)222, (char)87, (char)117, (char)221, (char)216, (char)93, (char)95, (char)25, (char)253, (char)237, (char)134, (char)58, (char)141, (char)177, (char)160, (char)219, (char)179, (char)157, (char)226, (char)191, (char)185, (char)101, (char)200, (char)191, (char)138, (char)191, (char)134, (char)17, (char)113, (char)137, (char)176, (char)33, (char)100, (char)132, (char)49, (char)39, (char)27, (char)56, (char)113, (char)249, (char)230, (char)248, (char)67, (char)104, (char)168, (char)68, (char)95, (char)54, (char)102, (char)146, (char)159, (char)134, (char)191, (char)38, (char)57, (char)75, (char)233, (char)194, (char)15, (char)244, (char)205, (char)222, (char)191, (char)186, (char)92, (char)17, (char)73, (char)148, (char)138, (char)80, (char)41, (char)132, (char)102, (char)190, (char)232, (char)106, (char)143, (char)36, (char)134, (char)214, (char)19, (char)146, (char)253, (char)0, (char)74, (char)120, (char)182, (char)133, (char)100, (char)8, (char)51, (char)109, (char)230, (char)38, (char)199, (char)172, (char)92, (char)201, (char)44, (char)130, (char)255, (char)201, (char)135, (char)46, (char)9, (char)211, (char)105, (char)2, (char)136, (char)90, (char)254, (char)229, (char)232, (char)162, (char)176, (char)197, (char)237, (char)172, (char)43, (char)183, (char)163, (char)220, (char)74, (char)36, (char)57, (char)116, (char)200, (char)177, (char)255, (char)253, (char)164, (char)62, (char)171, (char)140, (char)203, (char)33, (char)139, (char)35, (char)105, (char)51, (char)54, (char)42, (char)48, (char)153, (char)31, (char)87, (char)114, (char)241, (char)35, (char)62, (char)23, (char)135, (char)99}, 0) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == -1064901934851512091L);
            assert(pack.ts1_GET() == 2569911201196506124L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(-1064901934851512091L) ;
        p111.ts1_SET(2569911201196506124L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5342307379902351331L);
            assert(pack.seq_GET() == 2376283012L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(5342307379902351331L) ;
        p112.seq_SET(2376283012L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.eph_GET() == (char)2200);
            assert(pack.time_usec_GET() == 3760797539728767587L);
            assert(pack.alt_GET() == -394684143);
            assert(pack.epv_GET() == (char)27396);
            assert(pack.lon_GET() == -874463060);
            assert(pack.satellites_visible_GET() == (char)75);
            assert(pack.cog_GET() == (char)64324);
            assert(pack.vel_GET() == (char)61980);
            assert(pack.fix_type_GET() == (char)169);
            assert(pack.vd_GET() == (short) -14127);
            assert(pack.vn_GET() == (short) -20317);
            assert(pack.lat_GET() == 890210784);
            assert(pack.ve_GET() == (short) -3708);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.lon_SET(-874463060) ;
        p113.time_usec_SET(3760797539728767587L) ;
        p113.lat_SET(890210784) ;
        p113.eph_SET((char)2200) ;
        p113.vd_SET((short) -14127) ;
        p113.vel_SET((char)61980) ;
        p113.satellites_visible_SET((char)75) ;
        p113.fix_type_SET((char)169) ;
        p113.ve_SET((short) -3708) ;
        p113.epv_SET((char)27396) ;
        p113.alt_SET(-394684143) ;
        p113.vn_SET((short) -20317) ;
        p113.cog_SET((char)64324) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integrated_ygyro_GET() == 2.0334197E38F);
            assert(pack.time_delta_distance_us_GET() == 1054837587L);
            assert(pack.quality_GET() == (char)48);
            assert(pack.time_usec_GET() == 9146050872832977834L);
            assert(pack.integrated_x_GET() == -3.3513988E38F);
            assert(pack.sensor_id_GET() == (char)108);
            assert(pack.temperature_GET() == (short)30982);
            assert(pack.integrated_y_GET() == -3.1251757E38F);
            assert(pack.integrated_zgyro_GET() == -2.8942453E38F);
            assert(pack.integration_time_us_GET() == 2687150507L);
            assert(pack.distance_GET() == -3.0376085E38F);
            assert(pack.integrated_xgyro_GET() == 1.5942773E38F);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_usec_SET(9146050872832977834L) ;
        p114.quality_SET((char)48) ;
        p114.temperature_SET((short)30982) ;
        p114.sensor_id_SET((char)108) ;
        p114.time_delta_distance_us_SET(1054837587L) ;
        p114.integrated_y_SET(-3.1251757E38F) ;
        p114.distance_SET(-3.0376085E38F) ;
        p114.integrated_ygyro_SET(2.0334197E38F) ;
        p114.integration_time_us_SET(2687150507L) ;
        p114.integrated_zgyro_SET(-2.8942453E38F) ;
        p114.integrated_xgyro_SET(1.5942773E38F) ;
        p114.integrated_x_SET(-3.3513988E38F) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == (short) -13870);
            assert(pack.yawspeed_GET() == 9.631476E37F);
            assert(pack.ind_airspeed_GET() == (char)48560);
            assert(pack.lat_GET() == -904253048);
            assert(pack.true_airspeed_GET() == (char)52689);
            assert(pack.vz_GET() == (short) -15252);
            assert(pack.zacc_GET() == (short)20322);
            assert(pack.lon_GET() == -1038192213);
            assert(pack.xacc_GET() == (short) -17071);
            assert(pack.time_usec_GET() == 2829627627606462052L);
            assert(pack.rollspeed_GET() == -1.4195313E38F);
            assert(pack.pitchspeed_GET() == 6.5789327E37F);
            assert(pack.alt_GET() == -833047918);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {1.9930838E38F, 1.6690658E38F, -2.239531E38F, -2.3801872E38F}));
            assert(pack.vy_GET() == (short) -31843);
            assert(pack.yacc_GET() == (short) -25948);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.ind_airspeed_SET((char)48560) ;
        p115.zacc_SET((short)20322) ;
        p115.rollspeed_SET(-1.4195313E38F) ;
        p115.lat_SET(-904253048) ;
        p115.vz_SET((short) -15252) ;
        p115.lon_SET(-1038192213) ;
        p115.alt_SET(-833047918) ;
        p115.pitchspeed_SET(6.5789327E37F) ;
        p115.yacc_SET((short) -25948) ;
        p115.attitude_quaternion_SET(new float[] {1.9930838E38F, 1.6690658E38F, -2.239531E38F, -2.3801872E38F}, 0) ;
        p115.yawspeed_SET(9.631476E37F) ;
        p115.true_airspeed_SET((char)52689) ;
        p115.time_usec_SET(2829627627606462052L) ;
        p115.xacc_SET((short) -17071) ;
        p115.vy_SET((short) -31843) ;
        p115.vx_SET((short) -13870) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2435327510L);
            assert(pack.zacc_GET() == (short)21734);
            assert(pack.xacc_GET() == (short)11205);
            assert(pack.zmag_GET() == (short)12591);
            assert(pack.ygyro_GET() == (short) -24433);
            assert(pack.zgyro_GET() == (short)18615);
            assert(pack.xmag_GET() == (short) -4710);
            assert(pack.yacc_GET() == (short)5593);
            assert(pack.xgyro_GET() == (short)18868);
            assert(pack.ymag_GET() == (short)12384);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.time_boot_ms_SET(2435327510L) ;
        p116.xgyro_SET((short)18868) ;
        p116.zacc_SET((short)21734) ;
        p116.xacc_SET((short)11205) ;
        p116.yacc_SET((short)5593) ;
        p116.zmag_SET((short)12591) ;
        p116.xmag_SET((short) -4710) ;
        p116.ygyro_SET((short) -24433) ;
        p116.ymag_SET((short)12384) ;
        p116.zgyro_SET((short)18615) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)82);
            assert(pack.start_GET() == (char)31613);
            assert(pack.end_GET() == (char)59676);
            assert(pack.target_system_GET() == (char)226);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.end_SET((char)59676) ;
        p117.target_component_SET((char)82) ;
        p117.start_SET((char)31613) ;
        p117.target_system_SET((char)226) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.time_utc_GET() == 2770853094L);
            assert(pack.last_log_num_GET() == (char)59153);
            assert(pack.size_GET() == 622501484L);
            assert(pack.id_GET() == (char)56353);
            assert(pack.num_logs_GET() == (char)63124);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)56353) ;
        p118.num_logs_SET((char)63124) ;
        p118.last_log_num_SET((char)59153) ;
        p118.size_SET(622501484L) ;
        p118.time_utc_SET(2770853094L) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)20);
            assert(pack.target_system_GET() == (char)216);
            assert(pack.id_GET() == (char)61080);
            assert(pack.count_GET() == 3556961401L);
            assert(pack.ofs_GET() == 1892586832L);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.ofs_SET(1892586832L) ;
        p119.target_component_SET((char)20) ;
        p119.count_SET(3556961401L) ;
        p119.target_system_SET((char)216) ;
        p119.id_SET((char)61080) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)39, (char)180, (char)159, (char)216, (char)89, (char)31, (char)43, (char)206, (char)1, (char)6, (char)229, (char)162, (char)212, (char)170, (char)4, (char)195, (char)237, (char)54, (char)92, (char)217, (char)59, (char)209, (char)234, (char)213, (char)61, (char)74, (char)86, (char)186, (char)131, (char)173, (char)171, (char)198, (char)219, (char)6, (char)151, (char)225, (char)198, (char)188, (char)217, (char)202, (char)72, (char)201, (char)187, (char)164, (char)159, (char)28, (char)238, (char)176, (char)10, (char)31, (char)19, (char)41, (char)222, (char)13, (char)84, (char)122, (char)172, (char)46, (char)51, (char)46, (char)202, (char)228, (char)85, (char)7, (char)86, (char)66, (char)70, (char)250, (char)160, (char)10, (char)121, (char)130, (char)70, (char)105, (char)134, (char)98, (char)172, (char)239, (char)215, (char)167, (char)125, (char)236, (char)45, (char)196, (char)201, (char)222, (char)124, (char)255, (char)112, (char)105}));
            assert(pack.id_GET() == (char)64400);
            assert(pack.count_GET() == (char)115);
            assert(pack.ofs_GET() == 3408816276L);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.count_SET((char)115) ;
        p120.id_SET((char)64400) ;
        p120.data__SET(new char[] {(char)39, (char)180, (char)159, (char)216, (char)89, (char)31, (char)43, (char)206, (char)1, (char)6, (char)229, (char)162, (char)212, (char)170, (char)4, (char)195, (char)237, (char)54, (char)92, (char)217, (char)59, (char)209, (char)234, (char)213, (char)61, (char)74, (char)86, (char)186, (char)131, (char)173, (char)171, (char)198, (char)219, (char)6, (char)151, (char)225, (char)198, (char)188, (char)217, (char)202, (char)72, (char)201, (char)187, (char)164, (char)159, (char)28, (char)238, (char)176, (char)10, (char)31, (char)19, (char)41, (char)222, (char)13, (char)84, (char)122, (char)172, (char)46, (char)51, (char)46, (char)202, (char)228, (char)85, (char)7, (char)86, (char)66, (char)70, (char)250, (char)160, (char)10, (char)121, (char)130, (char)70, (char)105, (char)134, (char)98, (char)172, (char)239, (char)215, (char)167, (char)125, (char)236, (char)45, (char)196, (char)201, (char)222, (char)124, (char)255, (char)112, (char)105}, 0) ;
        p120.ofs_SET(3408816276L) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)18);
            assert(pack.target_system_GET() == (char)246);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_component_SET((char)18) ;
        p121.target_system_SET((char)246) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)67);
            assert(pack.target_system_GET() == (char)82);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)82) ;
        p122.target_component_SET((char)67) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)227);
            assert(pack.target_system_GET() == (char)71);
            assert(pack.target_component_GET() == (char)36);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)40, (char)219, (char)48, (char)120, (char)128, (char)92, (char)42, (char)111, (char)112, (char)112, (char)113, (char)179, (char)163, (char)255, (char)54, (char)162, (char)206, (char)24, (char)203, (char)183, (char)77, (char)70, (char)182, (char)37, (char)146, (char)207, (char)207, (char)142, (char)43, (char)118, (char)74, (char)53, (char)139, (char)155, (char)209, (char)178, (char)23, (char)186, (char)51, (char)85, (char)32, (char)133, (char)231, (char)163, (char)230, (char)166, (char)47, (char)241, (char)231, (char)117, (char)230, (char)69, (char)155, (char)183, (char)154, (char)2, (char)77, (char)65, (char)214, (char)234, (char)107, (char)12, (char)20, (char)182, (char)97, (char)172, (char)16, (char)243, (char)211, (char)13, (char)136, (char)13, (char)18, (char)96, (char)84, (char)90, (char)191, (char)25, (char)165, (char)91, (char)206, (char)184, (char)125, (char)6, (char)30, (char)6, (char)28, (char)96, (char)29, (char)140, (char)137, (char)232, (char)139, (char)210, (char)242, (char)28, (char)172, (char)146, (char)77, (char)47, (char)123, (char)121, (char)224, (char)246, (char)197, (char)60, (char)233, (char)183, (char)141, (char)76}));
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.len_SET((char)227) ;
        p123.target_system_SET((char)71) ;
        p123.data__SET(new char[] {(char)40, (char)219, (char)48, (char)120, (char)128, (char)92, (char)42, (char)111, (char)112, (char)112, (char)113, (char)179, (char)163, (char)255, (char)54, (char)162, (char)206, (char)24, (char)203, (char)183, (char)77, (char)70, (char)182, (char)37, (char)146, (char)207, (char)207, (char)142, (char)43, (char)118, (char)74, (char)53, (char)139, (char)155, (char)209, (char)178, (char)23, (char)186, (char)51, (char)85, (char)32, (char)133, (char)231, (char)163, (char)230, (char)166, (char)47, (char)241, (char)231, (char)117, (char)230, (char)69, (char)155, (char)183, (char)154, (char)2, (char)77, (char)65, (char)214, (char)234, (char)107, (char)12, (char)20, (char)182, (char)97, (char)172, (char)16, (char)243, (char)211, (char)13, (char)136, (char)13, (char)18, (char)96, (char)84, (char)90, (char)191, (char)25, (char)165, (char)91, (char)206, (char)184, (char)125, (char)6, (char)30, (char)6, (char)28, (char)96, (char)29, (char)140, (char)137, (char)232, (char)139, (char)210, (char)242, (char)28, (char)172, (char)146, (char)77, (char)47, (char)123, (char)121, (char)224, (char)246, (char)197, (char)60, (char)233, (char)183, (char)141, (char)76}, 0) ;
        p123.target_component_SET((char)36) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.dgps_age_GET() == 2701289477L);
            assert(pack.time_usec_GET() == 3870938616313261857L);
            assert(pack.lon_GET() == -585241857);
            assert(pack.eph_GET() == (char)64842);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
            assert(pack.dgps_numch_GET() == (char)255);
            assert(pack.alt_GET() == 1795513539);
            assert(pack.cog_GET() == (char)46996);
            assert(pack.vel_GET() == (char)4587);
            assert(pack.lat_GET() == -2070449996);
            assert(pack.epv_GET() == (char)45989);
            assert(pack.satellites_visible_GET() == (char)244);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.lon_SET(-585241857) ;
        p124.satellites_visible_SET((char)244) ;
        p124.time_usec_SET(3870938616313261857L) ;
        p124.alt_SET(1795513539) ;
        p124.dgps_numch_SET((char)255) ;
        p124.eph_SET((char)64842) ;
        p124.epv_SET((char)45989) ;
        p124.cog_SET((char)46996) ;
        p124.vel_SET((char)4587) ;
        p124.lat_SET(-2070449996) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS) ;
        p124.dgps_age_SET(2701289477L) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vcc_GET() == (char)21281);
            assert(pack.Vservo_GET() == (char)63856);
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID));
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vservo_SET((char)63856) ;
        p125.Vcc_SET((char)21281) ;
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID)) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.timeout_GET() == (char)43196);
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI));
            assert(pack.count_GET() == (char)220);
            assert(pack.baudrate_GET() == 3807791602L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)240, (char)179, (char)187, (char)11, (char)169, (char)36, (char)206, (char)141, (char)177, (char)103, (char)83, (char)89, (char)10, (char)77, (char)226, (char)249, (char)137, (char)11, (char)207, (char)0, (char)253, (char)58, (char)202, (char)200, (char)150, (char)224, (char)49, (char)0, (char)205, (char)20, (char)108, (char)113, (char)24, (char)98, (char)10, (char)145, (char)86, (char)245, (char)63, (char)21, (char)206, (char)93, (char)19, (char)59, (char)252, (char)41, (char)90, (char)164, (char)173, (char)46, (char)133, (char)105, (char)80, (char)186, (char)185, (char)68, (char)31, (char)4, (char)128, (char)207, (char)14, (char)162, (char)146, (char)144, (char)71, (char)10, (char)211, (char)3, (char)197, (char)251}));
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.timeout_SET((char)43196) ;
        p126.data__SET(new char[] {(char)240, (char)179, (char)187, (char)11, (char)169, (char)36, (char)206, (char)141, (char)177, (char)103, (char)83, (char)89, (char)10, (char)77, (char)226, (char)249, (char)137, (char)11, (char)207, (char)0, (char)253, (char)58, (char)202, (char)200, (char)150, (char)224, (char)49, (char)0, (char)205, (char)20, (char)108, (char)113, (char)24, (char)98, (char)10, (char)145, (char)86, (char)245, (char)63, (char)21, (char)206, (char)93, (char)19, (char)59, (char)252, (char)41, (char)90, (char)164, (char)173, (char)46, (char)133, (char)105, (char)80, (char)186, (char)185, (char)68, (char)31, (char)4, (char)128, (char)207, (char)14, (char)162, (char)146, (char)144, (char)71, (char)10, (char)211, (char)3, (char)197, (char)251}, 0) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2) ;
        p126.count_SET((char)220) ;
        p126.baudrate_SET(3807791602L) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI)) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.iar_num_hypotheses_GET() == 631035171);
            assert(pack.nsats_GET() == (char)181);
            assert(pack.tow_GET() == 1311660962L);
            assert(pack.baseline_b_mm_GET() == -707632312);
            assert(pack.time_last_baseline_ms_GET() == 2201619757L);
            assert(pack.rtk_rate_GET() == (char)226);
            assert(pack.baseline_c_mm_GET() == -1764322495);
            assert(pack.wn_GET() == (char)41342);
            assert(pack.baseline_coords_type_GET() == (char)131);
            assert(pack.rtk_receiver_id_GET() == (char)13);
            assert(pack.accuracy_GET() == 2424985444L);
            assert(pack.rtk_health_GET() == (char)89);
            assert(pack.baseline_a_mm_GET() == -620114819);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.nsats_SET((char)181) ;
        p127.rtk_receiver_id_SET((char)13) ;
        p127.accuracy_SET(2424985444L) ;
        p127.rtk_health_SET((char)89) ;
        p127.rtk_rate_SET((char)226) ;
        p127.baseline_coords_type_SET((char)131) ;
        p127.baseline_a_mm_SET(-620114819) ;
        p127.tow_SET(1311660962L) ;
        p127.baseline_b_mm_SET(-707632312) ;
        p127.baseline_c_mm_SET(-1764322495) ;
        p127.time_last_baseline_ms_SET(2201619757L) ;
        p127.wn_SET((char)41342) ;
        p127.iar_num_hypotheses_SET(631035171) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.iar_num_hypotheses_GET() == -1214768582);
            assert(pack.baseline_a_mm_GET() == 2088428109);
            assert(pack.baseline_c_mm_GET() == -1184896342);
            assert(pack.wn_GET() == (char)34757);
            assert(pack.rtk_health_GET() == (char)3);
            assert(pack.rtk_rate_GET() == (char)192);
            assert(pack.baseline_coords_type_GET() == (char)40);
            assert(pack.rtk_receiver_id_GET() == (char)209);
            assert(pack.tow_GET() == 241941726L);
            assert(pack.nsats_GET() == (char)214);
            assert(pack.accuracy_GET() == 1998765512L);
            assert(pack.time_last_baseline_ms_GET() == 2393845501L);
            assert(pack.baseline_b_mm_GET() == -561436031);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.rtk_health_SET((char)3) ;
        p128.iar_num_hypotheses_SET(-1214768582) ;
        p128.tow_SET(241941726L) ;
        p128.baseline_coords_type_SET((char)40) ;
        p128.baseline_c_mm_SET(-1184896342) ;
        p128.nsats_SET((char)214) ;
        p128.rtk_rate_SET((char)192) ;
        p128.rtk_receiver_id_SET((char)209) ;
        p128.accuracy_SET(1998765512L) ;
        p128.baseline_b_mm_SET(-561436031) ;
        p128.wn_SET((char)34757) ;
        p128.baseline_a_mm_SET(2088428109) ;
        p128.time_last_baseline_ms_SET(2393845501L) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == (short)8259);
            assert(pack.xgyro_GET() == (short)17024);
            assert(pack.yacc_GET() == (short) -16297);
            assert(pack.xacc_GET() == (short)29455);
            assert(pack.time_boot_ms_GET() == 3190443753L);
            assert(pack.zgyro_GET() == (short) -22758);
            assert(pack.ymag_GET() == (short) -19251);
            assert(pack.zmag_GET() == (short) -5401);
            assert(pack.ygyro_GET() == (short) -32111);
            assert(pack.xmag_GET() == (short) -10890);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.time_boot_ms_SET(3190443753L) ;
        p129.xacc_SET((short)29455) ;
        p129.ymag_SET((short) -19251) ;
        p129.xgyro_SET((short)17024) ;
        p129.zacc_SET((short)8259) ;
        p129.yacc_SET((short) -16297) ;
        p129.xmag_SET((short) -10890) ;
        p129.zgyro_SET((short) -22758) ;
        p129.ygyro_SET((short) -32111) ;
        p129.zmag_SET((short) -5401) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.packets_GET() == (char)30037);
            assert(pack.type_GET() == (char)223);
            assert(pack.size_GET() == 3235010823L);
            assert(pack.height_GET() == (char)33694);
            assert(pack.jpg_quality_GET() == (char)233);
            assert(pack.width_GET() == (char)52210);
            assert(pack.payload_GET() == (char)27);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.size_SET(3235010823L) ;
        p130.payload_SET((char)27) ;
        p130.height_SET((char)33694) ;
        p130.jpg_quality_SET((char)233) ;
        p130.packets_SET((char)30037) ;
        p130.type_SET((char)223) ;
        p130.width_SET((char)52210) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)25454);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)217, (char)168, (char)131, (char)208, (char)50, (char)9, (char)147, (char)253, (char)145, (char)172, (char)36, (char)214, (char)68, (char)71, (char)14, (char)125, (char)160, (char)4, (char)41, (char)71, (char)106, (char)105, (char)174, (char)36, (char)25, (char)92, (char)88, (char)137, (char)176, (char)119, (char)222, (char)2, (char)217, (char)235, (char)86, (char)13, (char)149, (char)77, (char)78, (char)122, (char)230, (char)164, (char)33, (char)187, (char)124, (char)24, (char)151, (char)74, (char)94, (char)255, (char)153, (char)232, (char)145, (char)69, (char)142, (char)83, (char)187, (char)203, (char)96, (char)195, (char)243, (char)64, (char)251, (char)122, (char)55, (char)199, (char)93, (char)104, (char)115, (char)236, (char)252, (char)99, (char)103, (char)33, (char)186, (char)182, (char)46, (char)134, (char)174, (char)114, (char)245, (char)224, (char)56, (char)235, (char)127, (char)21, (char)26, (char)144, (char)101, (char)248, (char)35, (char)63, (char)226, (char)194, (char)127, (char)154, (char)11, (char)108, (char)43, (char)8, (char)243, (char)205, (char)67, (char)175, (char)214, (char)167, (char)106, (char)135, (char)75, (char)2, (char)220, (char)125, (char)77, (char)150, (char)19, (char)252, (char)146, (char)115, (char)74, (char)40, (char)104, (char)60, (char)205, (char)157, (char)69, (char)22, (char)121, (char)75, (char)213, (char)202, (char)171, (char)55, (char)14, (char)96, (char)243, (char)51, (char)94, (char)228, (char)68, (char)177, (char)190, (char)202, (char)182, (char)84, (char)66, (char)110, (char)98, (char)180, (char)247, (char)18, (char)203, (char)201, (char)9, (char)87, (char)232, (char)250, (char)210, (char)212, (char)111, (char)123, (char)11, (char)13, (char)227, (char)248, (char)221, (char)8, (char)2, (char)91, (char)17, (char)154, (char)124, (char)13, (char)245, (char)160, (char)170, (char)21, (char)179, (char)251, (char)205, (char)55, (char)74, (char)166, (char)149, (char)180, (char)180, (char)239, (char)157, (char)157, (char)163, (char)195, (char)6, (char)73, (char)2, (char)31, (char)137, (char)226, (char)255, (char)186, (char)223, (char)39, (char)236, (char)196, (char)239, (char)215, (char)251, (char)202, (char)19, (char)6, (char)189, (char)35, (char)78, (char)214, (char)73, (char)87, (char)177, (char)162, (char)218, (char)168, (char)154, (char)114, (char)200, (char)152, (char)150, (char)107, (char)119, (char)138, (char)87, (char)20, (char)142, (char)50, (char)65, (char)79, (char)133, (char)87, (char)107, (char)160, (char)221, (char)107, (char)184, (char)75, (char)32, (char)73, (char)223, (char)178, (char)86, (char)115, (char)166, (char)57, (char)52, (char)44, (char)161, (char)34, (char)200}));
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)25454) ;
        p131.data__SET(new char[] {(char)217, (char)168, (char)131, (char)208, (char)50, (char)9, (char)147, (char)253, (char)145, (char)172, (char)36, (char)214, (char)68, (char)71, (char)14, (char)125, (char)160, (char)4, (char)41, (char)71, (char)106, (char)105, (char)174, (char)36, (char)25, (char)92, (char)88, (char)137, (char)176, (char)119, (char)222, (char)2, (char)217, (char)235, (char)86, (char)13, (char)149, (char)77, (char)78, (char)122, (char)230, (char)164, (char)33, (char)187, (char)124, (char)24, (char)151, (char)74, (char)94, (char)255, (char)153, (char)232, (char)145, (char)69, (char)142, (char)83, (char)187, (char)203, (char)96, (char)195, (char)243, (char)64, (char)251, (char)122, (char)55, (char)199, (char)93, (char)104, (char)115, (char)236, (char)252, (char)99, (char)103, (char)33, (char)186, (char)182, (char)46, (char)134, (char)174, (char)114, (char)245, (char)224, (char)56, (char)235, (char)127, (char)21, (char)26, (char)144, (char)101, (char)248, (char)35, (char)63, (char)226, (char)194, (char)127, (char)154, (char)11, (char)108, (char)43, (char)8, (char)243, (char)205, (char)67, (char)175, (char)214, (char)167, (char)106, (char)135, (char)75, (char)2, (char)220, (char)125, (char)77, (char)150, (char)19, (char)252, (char)146, (char)115, (char)74, (char)40, (char)104, (char)60, (char)205, (char)157, (char)69, (char)22, (char)121, (char)75, (char)213, (char)202, (char)171, (char)55, (char)14, (char)96, (char)243, (char)51, (char)94, (char)228, (char)68, (char)177, (char)190, (char)202, (char)182, (char)84, (char)66, (char)110, (char)98, (char)180, (char)247, (char)18, (char)203, (char)201, (char)9, (char)87, (char)232, (char)250, (char)210, (char)212, (char)111, (char)123, (char)11, (char)13, (char)227, (char)248, (char)221, (char)8, (char)2, (char)91, (char)17, (char)154, (char)124, (char)13, (char)245, (char)160, (char)170, (char)21, (char)179, (char)251, (char)205, (char)55, (char)74, (char)166, (char)149, (char)180, (char)180, (char)239, (char)157, (char)157, (char)163, (char)195, (char)6, (char)73, (char)2, (char)31, (char)137, (char)226, (char)255, (char)186, (char)223, (char)39, (char)236, (char)196, (char)239, (char)215, (char)251, (char)202, (char)19, (char)6, (char)189, (char)35, (char)78, (char)214, (char)73, (char)87, (char)177, (char)162, (char)218, (char)168, (char)154, (char)114, (char)200, (char)152, (char)150, (char)107, (char)119, (char)138, (char)87, (char)20, (char)142, (char)50, (char)65, (char)79, (char)133, (char)87, (char)107, (char)160, (char)221, (char)107, (char)184, (char)75, (char)32, (char)73, (char)223, (char)178, (char)86, (char)115, (char)166, (char)57, (char)52, (char)44, (char)161, (char)34, (char)200}, 0) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.min_distance_GET() == (char)3410);
            assert(pack.current_distance_GET() == (char)31696);
            assert(pack.max_distance_GET() == (char)3095);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_270);
            assert(pack.time_boot_ms_GET() == 4247133673L);
            assert(pack.covariance_GET() == (char)106);
            assert(pack.id_GET() == (char)240);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_270) ;
        p132.min_distance_SET((char)3410) ;
        p132.id_SET((char)240) ;
        p132.time_boot_ms_SET(4247133673L) ;
        p132.max_distance_SET((char)3095) ;
        p132.current_distance_SET((char)31696) ;
        p132.covariance_SET((char)106) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 2085351745);
            assert(pack.lon_GET() == -725451465);
            assert(pack.mask_GET() == 7813107765691002678L);
            assert(pack.grid_spacing_GET() == (char)8482);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.mask_SET(7813107765691002678L) ;
        p133.lon_SET(-725451465) ;
        p133.lat_SET(2085351745) ;
        p133.grid_spacing_SET((char)8482) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.grid_spacing_GET() == (char)40196);
            assert(pack.lat_GET() == -598270223);
            assert(pack.gridbit_GET() == (char)213);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short)18255, (short)11629, (short)18892, (short)24428, (short)23595, (short)1515, (short)30338, (short) -29114, (short)8794, (short)9111, (short) -32482, (short)5339, (short) -6248, (short)5348, (short)6097, (short) -25780}));
            assert(pack.lon_GET() == -1900908092);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.gridbit_SET((char)213) ;
        p134.lon_SET(-1900908092) ;
        p134.lat_SET(-598270223) ;
        p134.data__SET(new short[] {(short)18255, (short)11629, (short)18892, (short)24428, (short)23595, (short)1515, (short)30338, (short) -29114, (short)8794, (short)9111, (short) -32482, (short)5339, (short) -6248, (short)5348, (short)6097, (short) -25780}, 0) ;
        p134.grid_spacing_SET((char)40196) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -1844431407);
            assert(pack.lat_GET() == 1298984982);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(1298984982) ;
        p135.lon_SET(-1844431407) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 909904968);
            assert(pack.current_height_GET() == -1.4297362E38F);
            assert(pack.terrain_height_GET() == -1.9068694E38F);
            assert(pack.spacing_GET() == (char)41995);
            assert(pack.pending_GET() == (char)25977);
            assert(pack.lon_GET() == -1362514047);
            assert(pack.loaded_GET() == (char)13307);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.pending_SET((char)25977) ;
        p136.lat_SET(909904968) ;
        p136.loaded_SET((char)13307) ;
        p136.lon_SET(-1362514047) ;
        p136.spacing_SET((char)41995) ;
        p136.terrain_height_SET(-1.9068694E38F) ;
        p136.current_height_SET(-1.4297362E38F) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == -1.1959159E38F);
            assert(pack.time_boot_ms_GET() == 1791503083L);
            assert(pack.temperature_GET() == (short)26669);
            assert(pack.press_abs_GET() == 2.643405E38F);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.press_abs_SET(2.643405E38F) ;
        p137.press_diff_SET(-1.1959159E38F) ;
        p137.time_boot_ms_SET(1791503083L) ;
        p137.temperature_SET((short)26669) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.9799848E37F, -1.1059016E38F, 9.890816E37F, -2.0696767E38F}));
            assert(pack.time_usec_GET() == 110918610222242943L);
            assert(pack.z_GET() == 2.0842752E37F);
            assert(pack.y_GET() == 1.2918618E38F);
            assert(pack.x_GET() == -1.0473371E38F);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.q_SET(new float[] {-3.9799848E37F, -1.1059016E38F, 9.890816E37F, -2.0696767E38F}, 0) ;
        p138.z_SET(2.0842752E37F) ;
        p138.x_SET(-1.0473371E38F) ;
        p138.time_usec_SET(110918610222242943L) ;
        p138.y_SET(1.2918618E38F) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 1717566268692531435L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.0642236E38F, 1.4793493E38F, -6.812403E37F, -2.664931E38F, 2.8500562E38F, -1.8898452E38F, -2.383574E38F, 1.1353033E38F}));
            assert(pack.target_component_GET() == (char)158);
            assert(pack.target_system_GET() == (char)108);
            assert(pack.group_mlx_GET() == (char)80);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(1717566268692531435L) ;
        p139.group_mlx_SET((char)80) ;
        p139.controls_SET(new float[] {1.0642236E38F, 1.4793493E38F, -6.812403E37F, -2.664931E38F, 2.8500562E38F, -1.8898452E38F, -2.383574E38F, 1.1353033E38F}, 0) ;
        p139.target_system_SET((char)108) ;
        p139.target_component_SET((char)158) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6757446518220409768L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.6711717E38F, 2.6256086E38F, -2.8106568E38F, 2.6250174E38F, 2.2297065E38F, -2.9567587E38F, 2.1782393E38F, 3.1549367E38F}));
            assert(pack.group_mlx_GET() == (char)6);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.controls_SET(new float[] {1.6711717E38F, 2.6256086E38F, -2.8106568E38F, 2.6250174E38F, 2.2297065E38F, -2.9567587E38F, 2.1782393E38F, 3.1549367E38F}, 0) ;
        p140.group_mlx_SET((char)6) ;
        p140.time_usec_SET(6757446518220409768L) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_terrain_GET() == -6.862109E37F);
            assert(pack.altitude_relative_GET() == -8.593575E37F);
            assert(pack.altitude_local_GET() == -2.0997675E38F);
            assert(pack.altitude_amsl_GET() == -2.3811427E37F);
            assert(pack.altitude_monotonic_GET() == -2.2526904E38F);
            assert(pack.bottom_clearance_GET() == -6.7379093E37F);
            assert(pack.time_usec_GET() == 5120317391491783857L);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_monotonic_SET(-2.2526904E38F) ;
        p141.altitude_relative_SET(-8.593575E37F) ;
        p141.altitude_local_SET(-2.0997675E38F) ;
        p141.time_usec_SET(5120317391491783857L) ;
        p141.altitude_amsl_SET(-2.3811427E37F) ;
        p141.bottom_clearance_SET(-6.7379093E37F) ;
        p141.altitude_terrain_SET(-6.862109E37F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)45, (char)36, (char)88, (char)31, (char)202, (char)6, (char)150, (char)20, (char)176, (char)22, (char)219, (char)236, (char)154, (char)148, (char)37, (char)154, (char)47, (char)58, (char)73, (char)36, (char)225, (char)209, (char)184, (char)179, (char)216, (char)109, (char)67, (char)206, (char)167, (char)138, (char)83, (char)210, (char)92, (char)85, (char)149, (char)179, (char)174, (char)230, (char)169, (char)143, (char)157, (char)8, (char)168, (char)131, (char)1, (char)40, (char)199, (char)83, (char)213, (char)90, (char)33, (char)198, (char)241, (char)20, (char)119, (char)182, (char)87, (char)126, (char)65, (char)4, (char)250, (char)236, (char)30, (char)28, (char)24, (char)164, (char)241, (char)164, (char)226, (char)23, (char)104, (char)148, (char)117, (char)22, (char)247, (char)189, (char)219, (char)150, (char)125, (char)65, (char)49, (char)43, (char)19, (char)179, (char)150, (char)97, (char)109, (char)140, (char)134, (char)93, (char)42, (char)32, (char)75, (char)124, (char)162, (char)47, (char)12, (char)120, (char)91, (char)66, (char)105, (char)63, (char)93, (char)180, (char)22, (char)16, (char)146, (char)52, (char)245, (char)41, (char)138, (char)246, (char)101, (char)4, (char)133, (char)20, (char)70, (char)98, (char)130, (char)109}));
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)95, (char)151, (char)91, (char)166, (char)34, (char)64, (char)42, (char)178, (char)2, (char)56, (char)117, (char)156, (char)193, (char)148, (char)144, (char)156, (char)166, (char)232, (char)109, (char)74, (char)173, (char)59, (char)66, (char)120, (char)55, (char)151, (char)130, (char)151, (char)172, (char)126, (char)27, (char)170, (char)197, (char)58, (char)16, (char)242, (char)111, (char)227, (char)58, (char)225, (char)37, (char)104, (char)210, (char)48, (char)208, (char)175, (char)65, (char)63, (char)249, (char)153, (char)206, (char)239, (char)82, (char)16, (char)229, (char)35, (char)153, (char)13, (char)133, (char)91, (char)199, (char)162, (char)0, (char)111, (char)182, (char)200, (char)250, (char)39, (char)47, (char)168, (char)210, (char)49, (char)161, (char)238, (char)12, (char)162, (char)74, (char)157, (char)98, (char)157, (char)59, (char)255, (char)167, (char)57, (char)28, (char)127, (char)226, (char)223, (char)53, (char)230, (char)205, (char)231, (char)1, (char)8, (char)110, (char)199, (char)58, (char)158, (char)211, (char)116, (char)100, (char)132, (char)248, (char)34, (char)217, (char)159, (char)109, (char)13, (char)220, (char)133, (char)62, (char)123, (char)7, (char)192, (char)78, (char)77, (char)197, (char)18, (char)102, (char)217}));
            assert(pack.transfer_type_GET() == (char)62);
            assert(pack.uri_type_GET() == (char)177);
            assert(pack.request_id_GET() == (char)16);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.uri_type_SET((char)177) ;
        p142.request_id_SET((char)16) ;
        p142.storage_SET(new char[] {(char)95, (char)151, (char)91, (char)166, (char)34, (char)64, (char)42, (char)178, (char)2, (char)56, (char)117, (char)156, (char)193, (char)148, (char)144, (char)156, (char)166, (char)232, (char)109, (char)74, (char)173, (char)59, (char)66, (char)120, (char)55, (char)151, (char)130, (char)151, (char)172, (char)126, (char)27, (char)170, (char)197, (char)58, (char)16, (char)242, (char)111, (char)227, (char)58, (char)225, (char)37, (char)104, (char)210, (char)48, (char)208, (char)175, (char)65, (char)63, (char)249, (char)153, (char)206, (char)239, (char)82, (char)16, (char)229, (char)35, (char)153, (char)13, (char)133, (char)91, (char)199, (char)162, (char)0, (char)111, (char)182, (char)200, (char)250, (char)39, (char)47, (char)168, (char)210, (char)49, (char)161, (char)238, (char)12, (char)162, (char)74, (char)157, (char)98, (char)157, (char)59, (char)255, (char)167, (char)57, (char)28, (char)127, (char)226, (char)223, (char)53, (char)230, (char)205, (char)231, (char)1, (char)8, (char)110, (char)199, (char)58, (char)158, (char)211, (char)116, (char)100, (char)132, (char)248, (char)34, (char)217, (char)159, (char)109, (char)13, (char)220, (char)133, (char)62, (char)123, (char)7, (char)192, (char)78, (char)77, (char)197, (char)18, (char)102, (char)217}, 0) ;
        p142.transfer_type_SET((char)62) ;
        p142.uri_SET(new char[] {(char)45, (char)36, (char)88, (char)31, (char)202, (char)6, (char)150, (char)20, (char)176, (char)22, (char)219, (char)236, (char)154, (char)148, (char)37, (char)154, (char)47, (char)58, (char)73, (char)36, (char)225, (char)209, (char)184, (char)179, (char)216, (char)109, (char)67, (char)206, (char)167, (char)138, (char)83, (char)210, (char)92, (char)85, (char)149, (char)179, (char)174, (char)230, (char)169, (char)143, (char)157, (char)8, (char)168, (char)131, (char)1, (char)40, (char)199, (char)83, (char)213, (char)90, (char)33, (char)198, (char)241, (char)20, (char)119, (char)182, (char)87, (char)126, (char)65, (char)4, (char)250, (char)236, (char)30, (char)28, (char)24, (char)164, (char)241, (char)164, (char)226, (char)23, (char)104, (char)148, (char)117, (char)22, (char)247, (char)189, (char)219, (char)150, (char)125, (char)65, (char)49, (char)43, (char)19, (char)179, (char)150, (char)97, (char)109, (char)140, (char)134, (char)93, (char)42, (char)32, (char)75, (char)124, (char)162, (char)47, (char)12, (char)120, (char)91, (char)66, (char)105, (char)63, (char)93, (char)180, (char)22, (char)16, (char)146, (char)52, (char)245, (char)41, (char)138, (char)246, (char)101, (char)4, (char)133, (char)20, (char)70, (char)98, (char)130, (char)109}, 0) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -1.0660612E38F);
            assert(pack.time_boot_ms_GET() == 3778413951L);
            assert(pack.press_diff_GET() == 3.1967252E38F);
            assert(pack.temperature_GET() == (short)12804);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(3778413951L) ;
        p143.press_abs_SET(-1.0660612E38F) ;
        p143.press_diff_SET(3.1967252E38F) ;
        p143.temperature_SET((short)12804) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.est_capabilities_GET() == (char)67);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-1.8507123E38F, 2.2652227E38F, -3.0035756E38F}));
            assert(pack.lat_GET() == 1229556358);
            assert(Arrays.equals(pack.rates_GET(),  new float[] {2.80632E38F, 1.6178436E38F, -2.2558901E38F}));
            assert(Arrays.equals(pack.vel_GET(),  new float[] {-7.1199715E37F, 1.6363047E38F, 2.9557939E38F}));
            assert(pack.timestamp_GET() == 4109744472709710176L);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {1.5985302E38F, 5.0209703E37F, -1.7886493E38F}));
            assert(pack.lon_GET() == -1985611849);
            assert(pack.custom_state_GET() == 6826146570393306891L);
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-3.9576194E37F, 2.1858836E38F, 2.1749838E38F, 7.525395E37F}));
            assert(pack.alt_GET() == -1.2887258E38F);
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.est_capabilities_SET((char)67) ;
        p144.attitude_q_SET(new float[] {-3.9576194E37F, 2.1858836E38F, 2.1749838E38F, 7.525395E37F}, 0) ;
        p144.timestamp_SET(4109744472709710176L) ;
        p144.rates_SET(new float[] {2.80632E38F, 1.6178436E38F, -2.2558901E38F}, 0) ;
        p144.custom_state_SET(6826146570393306891L) ;
        p144.alt_SET(-1.2887258E38F) ;
        p144.position_cov_SET(new float[] {1.5985302E38F, 5.0209703E37F, -1.7886493E38F}, 0) ;
        p144.acc_SET(new float[] {-1.8507123E38F, 2.2652227E38F, -3.0035756E38F}, 0) ;
        p144.vel_SET(new float[] {-7.1199715E37F, 1.6363047E38F, 2.9557939E38F}, 0) ;
        p144.lon_SET(-1985611849) ;
        p144.lat_SET(1229556358) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.z_pos_GET() == 1.3970804E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {-3.3096753E38F, 1.8130639E38F, 5.8009005E37F}));
            assert(pack.pitch_rate_GET() == 1.0478804E38F);
            assert(pack.z_acc_GET() == 3.3839665E38F);
            assert(pack.x_acc_GET() == 1.4972954E37F);
            assert(pack.time_usec_GET() == 4872828625961692L);
            assert(pack.x_vel_GET() == 4.495852E37F);
            assert(pack.roll_rate_GET() == 1.1546891E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.5533813E38F, 2.9692701E38F, -2.894059E38F, -2.1573334E38F}));
            assert(pack.yaw_rate_GET() == -3.5008283E37F);
            assert(pack.z_vel_GET() == 1.6821304E38F);
            assert(pack.y_vel_GET() == -1.8331351E38F);
            assert(pack.y_acc_GET() == -1.2264698E38F);
            assert(pack.airspeed_GET() == -3.1841769E38F);
            assert(pack.y_pos_GET() == -1.4768039E38F);
            assert(pack.x_pos_GET() == 1.6547288E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {-2.4970587E38F, 1.2720229E38F, -7.109429E37F}));
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.yaw_rate_SET(-3.5008283E37F) ;
        p146.q_SET(new float[] {2.5533813E38F, 2.9692701E38F, -2.894059E38F, -2.1573334E38F}, 0) ;
        p146.pitch_rate_SET(1.0478804E38F) ;
        p146.x_vel_SET(4.495852E37F) ;
        p146.airspeed_SET(-3.1841769E38F) ;
        p146.y_pos_SET(-1.4768039E38F) ;
        p146.vel_variance_SET(new float[] {-2.4970587E38F, 1.2720229E38F, -7.109429E37F}, 0) ;
        p146.y_vel_SET(-1.8331351E38F) ;
        p146.x_acc_SET(1.4972954E37F) ;
        p146.y_acc_SET(-1.2264698E38F) ;
        p146.z_vel_SET(1.6821304E38F) ;
        p146.pos_variance_SET(new float[] {-3.3096753E38F, 1.8130639E38F, 5.8009005E37F}, 0) ;
        p146.z_acc_SET(3.3839665E38F) ;
        p146.time_usec_SET(4872828625961692L) ;
        p146.x_pos_SET(1.6547288E38F) ;
        p146.roll_rate_SET(1.1546891E38F) ;
        p146.z_pos_SET(1.3970804E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.energy_consumed_GET() == -1368279389);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)52928, (char)25513, (char)61397, (char)56005, (char)52333, (char)29300, (char)50833, (char)21880, (char)53166, (char)12402}));
            assert(pack.current_battery_GET() == (short)14031);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN);
            assert(pack.id_GET() == (char)56);
            assert(pack.current_consumed_GET() == 142562736);
            assert(pack.temperature_GET() == (short) -26581);
            assert(pack.battery_remaining_GET() == (byte)88);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.voltages_SET(new char[] {(char)52928, (char)25513, (char)61397, (char)56005, (char)52333, (char)29300, (char)50833, (char)21880, (char)53166, (char)12402}, 0) ;
        p147.current_battery_SET((short)14031) ;
        p147.id_SET((char)56) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH) ;
        p147.energy_consumed_SET(-1368279389) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN) ;
        p147.current_consumed_SET(142562736) ;
        p147.temperature_SET((short) -26581) ;
        p147.battery_remaining_SET((byte)88) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.os_sw_version_GET() == 3852228947L);
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE));
            assert(pack.middleware_sw_version_GET() == 2480751074L);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)191, (char)197, (char)103, (char)241, (char)145, (char)233, (char)116, (char)129}));
            assert(pack.flight_sw_version_GET() == 1325874191L);
            assert(pack.board_version_GET() == 2152696631L);
            assert(pack.uid_GET() == 8200261952329708983L);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)196, (char)229, (char)104, (char)78, (char)114, (char)165, (char)10, (char)188}));
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)244, (char)121, (char)224, (char)187, (char)155, (char)176, (char)252, (char)144}));
            assert(pack.vendor_id_GET() == (char)37394);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)87, (char)159, (char)111, (char)66, (char)251, (char)182, (char)81, (char)246, (char)247, (char)182, (char)188, (char)202, (char)23, (char)139, (char)29, (char)115, (char)206, (char)209}));
            assert(pack.product_id_GET() == (char)25917);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.flight_sw_version_SET(1325874191L) ;
        p148.board_version_SET(2152696631L) ;
        p148.uid2_SET(new char[] {(char)87, (char)159, (char)111, (char)66, (char)251, (char)182, (char)81, (char)246, (char)247, (char)182, (char)188, (char)202, (char)23, (char)139, (char)29, (char)115, (char)206, (char)209}, 0, PH) ;
        p148.os_sw_version_SET(3852228947L) ;
        p148.middleware_sw_version_SET(2480751074L) ;
        p148.middleware_custom_version_SET(new char[] {(char)244, (char)121, (char)224, (char)187, (char)155, (char)176, (char)252, (char)144}, 0) ;
        p148.uid_SET(8200261952329708983L) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE)) ;
        p148.flight_custom_version_SET(new char[] {(char)196, (char)229, (char)104, (char)78, (char)114, (char)165, (char)10, (char)188}, 0) ;
        p148.os_custom_version_SET(new char[] {(char)191, (char)197, (char)103, (char)241, (char)145, (char)233, (char)116, (char)129}, 0) ;
        p148.vendor_id_SET((char)37394) ;
        p148.product_id_SET((char)25917) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.angle_x_GET() == -8.2200337E37F);
            assert(pack.size_y_GET() == -2.2444097E38F);
            assert(pack.position_valid_TRY(ph) == (char)177);
            assert(pack.distance_GET() == 5.9958834E36F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
            assert(pack.angle_y_GET() == -2.4826124E38F);
            assert(pack.target_num_GET() == (char)162);
            assert(pack.y_TRY(ph) == -4.854156E37F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {2.933663E38F, -2.5573762E38F, 1.1108939E38F, 2.1395675E38F}));
            assert(pack.z_TRY(ph) == 2.4375102E38F);
            assert(pack.x_TRY(ph) == 3.18795E38F);
            assert(pack.time_usec_GET() == 3214802211730145439L);
            assert(pack.size_x_GET() == -1.0970931E38F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.q_SET(new float[] {2.933663E38F, -2.5573762E38F, 1.1108939E38F, 2.1395675E38F}, 0, PH) ;
        p149.position_valid_SET((char)177, PH) ;
        p149.size_x_SET(-1.0970931E38F) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p149.time_usec_SET(3214802211730145439L) ;
        p149.target_num_SET((char)162) ;
        p149.angle_y_SET(-2.4826124E38F) ;
        p149.x_SET(3.18795E38F, PH) ;
        p149.size_y_SET(-2.2444097E38F) ;
        p149.z_SET(2.4375102E38F, PH) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON) ;
        p149.distance_SET(5.9958834E36F) ;
        p149.angle_x_SET(-8.2200337E37F) ;
        p149.y_SET(-4.854156E37F, PH) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AQ_TELEMETRY_F.add((src, ph, pack) ->
        {
            assert(pack.value5_GET() == -1.8464098E38F);
            assert(pack.value6_GET() == 2.972922E38F);
            assert(pack.value14_GET() == 2.895604E38F);
            assert(pack.value3_GET() == 5.024432E37F);
            assert(pack.value18_GET() == 7.987428E37F);
            assert(pack.value9_GET() == -6.9927745E37F);
            assert(pack.value10_GET() == 2.6023733E38F);
            assert(pack.value7_GET() == 1.2798137E38F);
            assert(pack.value16_GET() == -1.1260738E38F);
            assert(pack.value20_GET() == -2.908616E38F);
            assert(pack.value4_GET() == 3.0377105E38F);
            assert(pack.value2_GET() == 4.6604464E37F);
            assert(pack.value15_GET() == 8.662438E37F);
            assert(pack.value8_GET() == 7.556438E37F);
            assert(pack.value12_GET() == 2.1634978E37F);
            assert(pack.value19_GET() == -1.8612326E38F);
            assert(pack.value13_GET() == 3.337498E38F);
            assert(pack.Index_GET() == (char)52553);
            assert(pack.value17_GET() == 2.1239038E38F);
            assert(pack.value11_GET() == 2.1892932E38F);
            assert(pack.value1_GET() == -2.3121975E38F);
        });
        GroundControl.AQ_TELEMETRY_F p150 = CommunicationChannel.new_AQ_TELEMETRY_F();
        PH.setPack(p150);
        p150.value16_SET(-1.1260738E38F) ;
        p150.value7_SET(1.2798137E38F) ;
        p150.value13_SET(3.337498E38F) ;
        p150.value8_SET(7.556438E37F) ;
        p150.value11_SET(2.1892932E38F) ;
        p150.value10_SET(2.6023733E38F) ;
        p150.value3_SET(5.024432E37F) ;
        p150.value12_SET(2.1634978E37F) ;
        p150.value6_SET(2.972922E38F) ;
        p150.value2_SET(4.6604464E37F) ;
        p150.value20_SET(-2.908616E38F) ;
        p150.value1_SET(-2.3121975E38F) ;
        p150.value15_SET(8.662438E37F) ;
        p150.value19_SET(-1.8612326E38F) ;
        p150.Index_SET((char)52553) ;
        p150.value14_SET(2.895604E38F) ;
        p150.value9_SET(-6.9927745E37F) ;
        p150.value18_SET(7.987428E37F) ;
        p150.value4_SET(3.0377105E38F) ;
        p150.value17_SET(2.1239038E38F) ;
        p150.value5_SET(-1.8464098E38F) ;
        CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AQ_ESC_TELEMETRY.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2068527977L);
            assert(Arrays.equals(pack.data_version_GET(),  new char[] {(char)79, (char)67, (char)144, (char)68}));
            assert(Arrays.equals(pack.data0_GET(),  new long[] {3209356584L, 3045678770L, 3074066237L, 2944910188L}));
            assert(pack.num_in_seq_GET() == (char)195);
            assert(Arrays.equals(pack.escid_GET(),  new char[] {(char)30, (char)121, (char)84, (char)112}));
            assert(Arrays.equals(pack.data1_GET(),  new long[] {1675298583L, 3425211318L, 1617659248L, 1804116456L}));
            assert(pack.seq_GET() == (char)26);
            assert(pack.num_motors_GET() == (char)114);
            assert(Arrays.equals(pack.status_age_GET(),  new char[] {(char)4440, (char)20971, (char)5093, (char)20027}));
        });
        GroundControl.AQ_ESC_TELEMETRY p152 = CommunicationChannel.new_AQ_ESC_TELEMETRY();
        PH.setPack(p152);
        p152.status_age_SET(new char[] {(char)4440, (char)20971, (char)5093, (char)20027}, 0) ;
        p152.time_boot_ms_SET(2068527977L) ;
        p152.data_version_SET(new char[] {(char)79, (char)67, (char)144, (char)68}, 0) ;
        p152.num_motors_SET((char)114) ;
        p152.escid_SET(new char[] {(char)30, (char)121, (char)84, (char)112}, 0) ;
        p152.num_in_seq_SET((char)195) ;
        p152.data0_SET(new long[] {3209356584L, 3045678770L, 3074066237L, 2944910188L}, 0) ;
        p152.data1_SET(new long[] {1675298583L, 3425211318L, 1617659248L, 1804116456L}, 0) ;
        p152.seq_SET((char)26) ;
        CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pos_vert_ratio_GET() == 3.057471E38F);
            assert(pack.pos_horiz_accuracy_GET() == 2.9731084E37F);
            assert(pack.hagl_ratio_GET() == -2.2452384E38F);
            assert(pack.time_usec_GET() == 8282103889446928947L);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS));
            assert(pack.tas_ratio_GET() == 1.930192E38F);
            assert(pack.vel_ratio_GET() == -1.6546762E38F);
            assert(pack.pos_horiz_ratio_GET() == -8.911011E37F);
            assert(pack.mag_ratio_GET() == -9.440564E37F);
            assert(pack.pos_vert_accuracy_GET() == 1.9231844E38F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.tas_ratio_SET(1.930192E38F) ;
        p230.pos_horiz_accuracy_SET(2.9731084E37F) ;
        p230.pos_horiz_ratio_SET(-8.911011E37F) ;
        p230.time_usec_SET(8282103889446928947L) ;
        p230.pos_vert_ratio_SET(3.057471E38F) ;
        p230.hagl_ratio_SET(-2.2452384E38F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS)) ;
        p230.vel_ratio_SET(-1.6546762E38F) ;
        p230.mag_ratio_SET(-9.440564E37F) ;
        p230.pos_vert_accuracy_SET(1.9231844E38F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.var_vert_GET() == 1.7657087E38F);
            assert(pack.time_usec_GET() == 5959325907904093506L);
            assert(pack.vert_accuracy_GET() == 2.0981702E38F);
            assert(pack.wind_z_GET() == 6.6655964E37F);
            assert(pack.wind_x_GET() == 3.3105955E38F);
            assert(pack.horiz_accuracy_GET() == -6.240802E36F);
            assert(pack.wind_y_GET() == -1.6249653E38F);
            assert(pack.wind_alt_GET() == 3.1930155E38F);
            assert(pack.var_horiz_GET() == 2.8214067E38F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_alt_SET(3.1930155E38F) ;
        p231.horiz_accuracy_SET(-6.240802E36F) ;
        p231.var_horiz_SET(2.8214067E38F) ;
        p231.wind_x_SET(3.3105955E38F) ;
        p231.wind_z_SET(6.6655964E37F) ;
        p231.var_vert_SET(1.7657087E38F) ;
        p231.time_usec_SET(5959325907904093506L) ;
        p231.wind_y_SET(-1.6249653E38F) ;
        p231.vert_accuracy_SET(2.0981702E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.vert_accuracy_GET() == -1.2190876E38F);
            assert(pack.fix_type_GET() == (char)222);
            assert(pack.speed_accuracy_GET() == -6.061632E37F);
            assert(pack.vn_GET() == 8.1559595E37F);
            assert(pack.time_usec_GET() == 3130760154218957520L);
            assert(pack.lon_GET() == 1495736562);
            assert(pack.vd_GET() == -1.7386148E38F);
            assert(pack.hdop_GET() == -1.3611531E38F);
            assert(pack.ve_GET() == 1.1990715E37F);
            assert(pack.time_week_GET() == (char)21060);
            assert(pack.lat_GET() == -1434598016);
            assert(pack.horiz_accuracy_GET() == 2.3102336E38F);
            assert(pack.vdop_GET() == 1.9345882E38F);
            assert(pack.satellites_visible_GET() == (char)59);
            assert(pack.time_week_ms_GET() == 390530880L);
            assert(pack.alt_GET() == 1.6965542E38F);
            assert(pack.gps_id_GET() == (char)192);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP));
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.time_week_SET((char)21060) ;
        p232.vert_accuracy_SET(-1.2190876E38F) ;
        p232.fix_type_SET((char)222) ;
        p232.gps_id_SET((char)192) ;
        p232.ve_SET(1.1990715E37F) ;
        p232.speed_accuracy_SET(-6.061632E37F) ;
        p232.time_week_ms_SET(390530880L) ;
        p232.lon_SET(1495736562) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP)) ;
        p232.alt_SET(1.6965542E38F) ;
        p232.hdop_SET(-1.3611531E38F) ;
        p232.vd_SET(-1.7386148E38F) ;
        p232.satellites_visible_SET((char)59) ;
        p232.horiz_accuracy_SET(2.3102336E38F) ;
        p232.time_usec_SET(3130760154218957520L) ;
        p232.lat_SET(-1434598016) ;
        p232.vn_SET(8.1559595E37F) ;
        p232.vdop_SET(1.9345882E38F) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)138, (char)155, (char)207, (char)101, (char)61, (char)131, (char)184, (char)250, (char)229, (char)28, (char)43, (char)222, (char)198, (char)206, (char)127, (char)182, (char)143, (char)182, (char)153, (char)71, (char)116, (char)67, (char)220, (char)73, (char)173, (char)16, (char)222, (char)23, (char)215, (char)245, (char)161, (char)201, (char)197, (char)132, (char)52, (char)5, (char)123, (char)89, (char)245, (char)15, (char)93, (char)234, (char)117, (char)166, (char)86, (char)135, (char)96, (char)241, (char)219, (char)157, (char)189, (char)69, (char)103, (char)60, (char)207, (char)135, (char)100, (char)197, (char)255, (char)157, (char)215, (char)235, (char)197, (char)6, (char)169, (char)148, (char)123, (char)231, (char)2, (char)98, (char)216, (char)27, (char)144, (char)247, (char)128, (char)56, (char)72, (char)27, (char)220, (char)119, (char)44, (char)181, (char)63, (char)172, (char)209, (char)30, (char)9, (char)179, (char)167, (char)216, (char)194, (char)107, (char)155, (char)194, (char)223, (char)203, (char)217, (char)114, (char)223, (char)114, (char)18, (char)36, (char)76, (char)13, (char)228, (char)81, (char)41, (char)89, (char)54, (char)8, (char)199, (char)46, (char)140, (char)165, (char)178, (char)249, (char)117, (char)193, (char)249, (char)60, (char)194, (char)169, (char)2, (char)115, (char)107, (char)209, (char)139, (char)72, (char)104, (char)159, (char)13, (char)231, (char)88, (char)116, (char)108, (char)186, (char)104, (char)8, (char)48, (char)191, (char)44, (char)28, (char)8, (char)54, (char)136, (char)84, (char)249, (char)92, (char)66, (char)45, (char)52, (char)192, (char)240, (char)240, (char)161, (char)63, (char)83, (char)100, (char)37, (char)28, (char)168, (char)132, (char)230, (char)217, (char)143, (char)55, (char)113, (char)18, (char)224, (char)147, (char)252, (char)15, (char)41, (char)99, (char)245, (char)41, (char)71, (char)219, (char)133, (char)177}));
            assert(pack.len_GET() == (char)112);
            assert(pack.flags_GET() == (char)203);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)203) ;
        p233.data__SET(new char[] {(char)138, (char)155, (char)207, (char)101, (char)61, (char)131, (char)184, (char)250, (char)229, (char)28, (char)43, (char)222, (char)198, (char)206, (char)127, (char)182, (char)143, (char)182, (char)153, (char)71, (char)116, (char)67, (char)220, (char)73, (char)173, (char)16, (char)222, (char)23, (char)215, (char)245, (char)161, (char)201, (char)197, (char)132, (char)52, (char)5, (char)123, (char)89, (char)245, (char)15, (char)93, (char)234, (char)117, (char)166, (char)86, (char)135, (char)96, (char)241, (char)219, (char)157, (char)189, (char)69, (char)103, (char)60, (char)207, (char)135, (char)100, (char)197, (char)255, (char)157, (char)215, (char)235, (char)197, (char)6, (char)169, (char)148, (char)123, (char)231, (char)2, (char)98, (char)216, (char)27, (char)144, (char)247, (char)128, (char)56, (char)72, (char)27, (char)220, (char)119, (char)44, (char)181, (char)63, (char)172, (char)209, (char)30, (char)9, (char)179, (char)167, (char)216, (char)194, (char)107, (char)155, (char)194, (char)223, (char)203, (char)217, (char)114, (char)223, (char)114, (char)18, (char)36, (char)76, (char)13, (char)228, (char)81, (char)41, (char)89, (char)54, (char)8, (char)199, (char)46, (char)140, (char)165, (char)178, (char)249, (char)117, (char)193, (char)249, (char)60, (char)194, (char)169, (char)2, (char)115, (char)107, (char)209, (char)139, (char)72, (char)104, (char)159, (char)13, (char)231, (char)88, (char)116, (char)108, (char)186, (char)104, (char)8, (char)48, (char)191, (char)44, (char)28, (char)8, (char)54, (char)136, (char)84, (char)249, (char)92, (char)66, (char)45, (char)52, (char)192, (char)240, (char)240, (char)161, (char)63, (char)83, (char)100, (char)37, (char)28, (char)168, (char)132, (char)230, (char)217, (char)143, (char)55, (char)113, (char)18, (char)224, (char)147, (char)252, (char)15, (char)41, (char)99, (char)245, (char)41, (char)71, (char)219, (char)133, (char)177}, 0) ;
        p233.len_SET((char)112) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.failsafe_GET() == (char)25);
            assert(pack.temperature_GET() == (byte) - 16);
            assert(pack.wp_distance_GET() == (char)54820);
            assert(pack.pitch_GET() == (short) -18859);
            assert(pack.longitude_GET() == -1450479165);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
            assert(pack.custom_mode_GET() == 641309440L);
            assert(pack.roll_GET() == (short) -31814);
            assert(pack.temperature_air_GET() == (byte) - 103);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
            assert(pack.groundspeed_GET() == (char)7);
            assert(pack.airspeed_GET() == (char)109);
            assert(pack.wp_num_GET() == (char)171);
            assert(pack.battery_remaining_GET() == (char)90);
            assert(pack.altitude_sp_GET() == (short) -5614);
            assert(pack.airspeed_sp_GET() == (char)214);
            assert(pack.heading_sp_GET() == (short)5352);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
            assert(pack.heading_GET() == (char)20780);
            assert(pack.climb_rate_GET() == (byte)0);
            assert(pack.throttle_GET() == (byte)101);
            assert(pack.altitude_amsl_GET() == (short)31711);
            assert(pack.gps_nsat_GET() == (char)8);
            assert(pack.latitude_GET() == 2120504822);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.altitude_sp_SET((short) -5614) ;
        p234.latitude_SET(2120504822) ;
        p234.heading_sp_SET((short)5352) ;
        p234.airspeed_SET((char)109) ;
        p234.heading_SET((char)20780) ;
        p234.wp_num_SET((char)171) ;
        p234.groundspeed_SET((char)7) ;
        p234.altitude_amsl_SET((short)31711) ;
        p234.roll_SET((short) -31814) ;
        p234.gps_nsat_SET((char)8) ;
        p234.custom_mode_SET(641309440L) ;
        p234.throttle_SET((byte)101) ;
        p234.pitch_SET((short) -18859) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF) ;
        p234.longitude_SET(-1450479165) ;
        p234.failsafe_SET((char)25) ;
        p234.temperature_SET((byte) - 16) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) ;
        p234.climb_rate_SET((byte)0) ;
        p234.temperature_air_SET((byte) - 103) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS) ;
        p234.airspeed_sp_SET((char)214) ;
        p234.battery_remaining_SET((char)90) ;
        p234.wp_distance_SET((char)54820) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.clipping_1_GET() == 2016430918L);
            assert(pack.clipping_2_GET() == 2800139611L);
            assert(pack.vibration_x_GET() == -6.1992637E37F);
            assert(pack.vibration_y_GET() == -4.626324E37F);
            assert(pack.vibration_z_GET() == -2.8980719E37F);
            assert(pack.clipping_0_GET() == 2798504861L);
            assert(pack.time_usec_GET() == 2685594018291419478L);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.clipping_2_SET(2800139611L) ;
        p241.clipping_1_SET(2016430918L) ;
        p241.time_usec_SET(2685594018291419478L) ;
        p241.clipping_0_SET(2798504861L) ;
        p241.vibration_x_SET(-6.1992637E37F) ;
        p241.vibration_y_SET(-4.626324E37F) ;
        p241.vibration_z_SET(-2.8980719E37F) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.3363878E38F, 4.1016896E37F, -1.2306311E38F, 1.1026649E38F}));
            assert(pack.altitude_GET() == -1093030502);
            assert(pack.time_usec_TRY(ph) == 5513815581660215470L);
            assert(pack.y_GET() == 1.6534787E38F);
            assert(pack.x_GET() == -2.318675E38F);
            assert(pack.z_GET() == 3.2390349E38F);
            assert(pack.approach_y_GET() == -2.8662702E38F);
            assert(pack.approach_z_GET() == 3.6004657E37F);
            assert(pack.longitude_GET() == -2137987252);
            assert(pack.approach_x_GET() == 2.1751333E38F);
            assert(pack.latitude_GET() == -368380386);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.longitude_SET(-2137987252) ;
        p242.approach_x_SET(2.1751333E38F) ;
        p242.y_SET(1.6534787E38F) ;
        p242.x_SET(-2.318675E38F) ;
        p242.approach_z_SET(3.6004657E37F) ;
        p242.time_usec_SET(5513815581660215470L, PH) ;
        p242.approach_y_SET(-2.8662702E38F) ;
        p242.altitude_SET(-1093030502) ;
        p242.z_SET(3.2390349E38F) ;
        p242.latitude_SET(-368380386) ;
        p242.q_SET(new float[] {2.3363878E38F, 4.1016896E37F, -1.2306311E38F, 1.1026649E38F}, 0) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 1.519182E38F);
            assert(pack.altitude_GET() == -143950061);
            assert(pack.z_GET() == 3.797239E37F);
            assert(pack.time_usec_TRY(ph) == 563675646661719214L);
            assert(pack.target_system_GET() == (char)113);
            assert(pack.approach_x_GET() == 1.6419061E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.6655693E37F, 9.840063E37F, 2.3676484E38F, 4.5203406E37F}));
            assert(pack.approach_z_GET() == 1.0554373E38F);
            assert(pack.longitude_GET() == -949535616);
            assert(pack.approach_y_GET() == -2.9662292E38F);
            assert(pack.x_GET() == 1.5951702E38F);
            assert(pack.latitude_GET() == 534956198);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.approach_x_SET(1.6419061E38F) ;
        p243.latitude_SET(534956198) ;
        p243.approach_z_SET(1.0554373E38F) ;
        p243.altitude_SET(-143950061) ;
        p243.longitude_SET(-949535616) ;
        p243.approach_y_SET(-2.9662292E38F) ;
        p243.time_usec_SET(563675646661719214L, PH) ;
        p243.q_SET(new float[] {-1.6655693E37F, 9.840063E37F, 2.3676484E38F, 4.5203406E37F}, 0) ;
        p243.y_SET(1.519182E38F) ;
        p243.z_SET(3.797239E37F) ;
        p243.x_SET(1.5951702E38F) ;
        p243.target_system_SET((char)113) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.interval_us_GET() == -911899318);
            assert(pack.message_id_GET() == (char)21438);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(-911899318) ;
        p244.message_id_SET((char)21438) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_FW) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.callsign_LEN(ph) == 3);
            assert(pack.callsign_TRY(ph).equals("jus"));
            assert(pack.ICAO_address_GET() == 3649902423L);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UAV);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            assert(pack.heading_GET() == (char)39585);
            assert(pack.altitude_GET() == -764906413);
            assert(pack.squawk_GET() == (char)13147);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY));
            assert(pack.tslc_GET() == (char)215);
            assert(pack.ver_velocity_GET() == (short)16832);
            assert(pack.hor_velocity_GET() == (char)6456);
            assert(pack.lat_GET() == 1106660046);
            assert(pack.lon_GET() == -1069471604);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.tslc_SET((char)215) ;
        p246.callsign_SET("jus", PH) ;
        p246.hor_velocity_SET((char)6456) ;
        p246.squawk_SET((char)13147) ;
        p246.lon_SET(-1069471604) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UAV) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY)) ;
        p246.heading_SET((char)39585) ;
        p246.altitude_SET(-764906413) ;
        p246.ICAO_address_SET(3649902423L) ;
        p246.lat_SET(1106660046) ;
        p246.ver_velocity_SET((short)16832) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.threat_level_GET() == (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH));
            assert(pack.altitude_minimum_delta_GET() == -1.2828228E38F);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
            assert(pack.id_GET() == 1628083092L);
            assert(pack.horizontal_minimum_delta_GET() == -7.3333625E36F);
            assert(pack.time_to_minimum_delta_GET() == 1.273064E38F);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND) ;
        p247.id_SET(1628083092L) ;
        p247.time_to_minimum_delta_SET(1.273064E38F) ;
        p247.altitude_minimum_delta_SET(-1.2828228E38F) ;
        p247.threat_level_SET((MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH)) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        p247.horizontal_minimum_delta_SET(-7.3333625E36F) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)63);
            assert(pack.target_system_GET() == (char)196);
            assert(pack.message_type_GET() == (char)45460);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)203, (char)202, (char)171, (char)156, (char)205, (char)205, (char)182, (char)240, (char)78, (char)214, (char)43, (char)175, (char)26, (char)237, (char)143, (char)30, (char)231, (char)147, (char)9, (char)195, (char)100, (char)194, (char)20, (char)241, (char)14, (char)36, (char)151, (char)130, (char)70, (char)35, (char)54, (char)246, (char)97, (char)121, (char)34, (char)137, (char)135, (char)47, (char)200, (char)244, (char)228, (char)79, (char)222, (char)148, (char)58, (char)23, (char)78, (char)197, (char)130, (char)67, (char)92, (char)221, (char)110, (char)33, (char)174, (char)132, (char)41, (char)39, (char)20, (char)156, (char)83, (char)182, (char)214, (char)232, (char)169, (char)42, (char)74, (char)236, (char)97, (char)122, (char)109, (char)47, (char)115, (char)0, (char)20, (char)172, (char)7, (char)60, (char)113, (char)186, (char)26, (char)205, (char)64, (char)59, (char)231, (char)192, (char)213, (char)84, (char)104, (char)188, (char)16, (char)139, (char)49, (char)175, (char)67, (char)117, (char)41, (char)126, (char)111, (char)197, (char)118, (char)137, (char)170, (char)180, (char)78, (char)97, (char)1, (char)201, (char)111, (char)142, (char)240, (char)21, (char)49, (char)108, (char)73, (char)128, (char)64, (char)181, (char)208, (char)97, (char)182, (char)49, (char)103, (char)82, (char)15, (char)189, (char)85, (char)218, (char)112, (char)101, (char)191, (char)64, (char)212, (char)107, (char)229, (char)175, (char)53, (char)62, (char)5, (char)66, (char)49, (char)42, (char)81, (char)108, (char)145, (char)106, (char)114, (char)181, (char)4, (char)156, (char)198, (char)63, (char)140, (char)252, (char)212, (char)161, (char)245, (char)94, (char)190, (char)181, (char)183, (char)230, (char)61, (char)45, (char)10, (char)91, (char)133, (char)214, (char)214, (char)106, (char)230, (char)17, (char)123, (char)45, (char)227, (char)211, (char)204, (char)136, (char)108, (char)86, (char)41, (char)201, (char)247, (char)255, (char)30, (char)86, (char)2, (char)198, (char)87, (char)19, (char)150, (char)26, (char)5, (char)237, (char)122, (char)228, (char)188, (char)77, (char)51, (char)109, (char)249, (char)201, (char)94, (char)91, (char)138, (char)14, (char)199, (char)125, (char)7, (char)224, (char)243, (char)118, (char)245, (char)126, (char)150, (char)131, (char)187, (char)176, (char)161, (char)48, (char)75, (char)118, (char)141, (char)71, (char)200, (char)90, (char)216, (char)19, (char)218, (char)187, (char)117, (char)124, (char)192, (char)215, (char)119, (char)31, (char)188, (char)190, (char)11, (char)151, (char)25, (char)91, (char)181, (char)190, (char)255, (char)76, (char)198, (char)108, (char)42}));
            assert(pack.target_component_GET() == (char)75);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_system_SET((char)196) ;
        p248.target_component_SET((char)75) ;
        p248.message_type_SET((char)45460) ;
        p248.target_network_SET((char)63) ;
        p248.payload_SET(new char[] {(char)203, (char)202, (char)171, (char)156, (char)205, (char)205, (char)182, (char)240, (char)78, (char)214, (char)43, (char)175, (char)26, (char)237, (char)143, (char)30, (char)231, (char)147, (char)9, (char)195, (char)100, (char)194, (char)20, (char)241, (char)14, (char)36, (char)151, (char)130, (char)70, (char)35, (char)54, (char)246, (char)97, (char)121, (char)34, (char)137, (char)135, (char)47, (char)200, (char)244, (char)228, (char)79, (char)222, (char)148, (char)58, (char)23, (char)78, (char)197, (char)130, (char)67, (char)92, (char)221, (char)110, (char)33, (char)174, (char)132, (char)41, (char)39, (char)20, (char)156, (char)83, (char)182, (char)214, (char)232, (char)169, (char)42, (char)74, (char)236, (char)97, (char)122, (char)109, (char)47, (char)115, (char)0, (char)20, (char)172, (char)7, (char)60, (char)113, (char)186, (char)26, (char)205, (char)64, (char)59, (char)231, (char)192, (char)213, (char)84, (char)104, (char)188, (char)16, (char)139, (char)49, (char)175, (char)67, (char)117, (char)41, (char)126, (char)111, (char)197, (char)118, (char)137, (char)170, (char)180, (char)78, (char)97, (char)1, (char)201, (char)111, (char)142, (char)240, (char)21, (char)49, (char)108, (char)73, (char)128, (char)64, (char)181, (char)208, (char)97, (char)182, (char)49, (char)103, (char)82, (char)15, (char)189, (char)85, (char)218, (char)112, (char)101, (char)191, (char)64, (char)212, (char)107, (char)229, (char)175, (char)53, (char)62, (char)5, (char)66, (char)49, (char)42, (char)81, (char)108, (char)145, (char)106, (char)114, (char)181, (char)4, (char)156, (char)198, (char)63, (char)140, (char)252, (char)212, (char)161, (char)245, (char)94, (char)190, (char)181, (char)183, (char)230, (char)61, (char)45, (char)10, (char)91, (char)133, (char)214, (char)214, (char)106, (char)230, (char)17, (char)123, (char)45, (char)227, (char)211, (char)204, (char)136, (char)108, (char)86, (char)41, (char)201, (char)247, (char)255, (char)30, (char)86, (char)2, (char)198, (char)87, (char)19, (char)150, (char)26, (char)5, (char)237, (char)122, (char)228, (char)188, (char)77, (char)51, (char)109, (char)249, (char)201, (char)94, (char)91, (char)138, (char)14, (char)199, (char)125, (char)7, (char)224, (char)243, (char)118, (char)245, (char)126, (char)150, (char)131, (char)187, (char)176, (char)161, (char)48, (char)75, (char)118, (char)141, (char)71, (char)200, (char)90, (char)216, (char)19, (char)218, (char)187, (char)117, (char)124, (char)192, (char)215, (char)119, (char)31, (char)188, (char)190, (char)11, (char)151, (char)25, (char)91, (char)181, (char)190, (char)255, (char)76, (char)198, (char)108, (char)42}, 0) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte) - 102, (byte)20, (byte) - 126, (byte)42, (byte) - 25, (byte) - 117, (byte)27, (byte)13, (byte)22, (byte) - 91, (byte) - 5, (byte) - 36, (byte)85, (byte) - 117, (byte) - 36, (byte) - 5, (byte)91, (byte)33, (byte)65, (byte) - 72, (byte)116, (byte)76, (byte) - 71, (byte) - 123, (byte) - 52, (byte)43, (byte) - 85, (byte) - 50, (byte) - 115, (byte) - 38, (byte) - 107, (byte)93}));
            assert(pack.ver_GET() == (char)41);
            assert(pack.address_GET() == (char)10144);
            assert(pack.type_GET() == (char)212);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.ver_SET((char)41) ;
        p249.type_SET((char)212) ;
        p249.address_SET((char)10144) ;
        p249.value_SET(new byte[] {(byte) - 102, (byte)20, (byte) - 126, (byte)42, (byte) - 25, (byte) - 117, (byte)27, (byte)13, (byte)22, (byte) - 91, (byte) - 5, (byte) - 36, (byte)85, (byte) - 117, (byte) - 36, (byte) - 5, (byte)91, (byte)33, (byte)65, (byte) - 72, (byte)116, (byte)76, (byte) - 71, (byte) - 123, (byte) - 52, (byte)43, (byte) - 85, (byte) - 50, (byte) - 115, (byte) - 38, (byte) - 107, (byte)93}, 0) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5252915647078034987L);
            assert(pack.name_LEN(ph) == 7);
            assert(pack.name_TRY(ph).equals("mhoMhir"));
            assert(pack.x_GET() == -6.148726E37F);
            assert(pack.z_GET() == 2.7440778E38F);
            assert(pack.y_GET() == 4.321603E37F);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.z_SET(2.7440778E38F) ;
        p250.y_SET(4.321603E37F) ;
        p250.x_SET(-6.148726E37F) ;
        p250.time_usec_SET(5252915647078034987L) ;
        p250.name_SET("mhoMhir", PH) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 7);
            assert(pack.name_TRY(ph).equals("ysphdzj"));
            assert(pack.value_GET() == 1.0444303E38F);
            assert(pack.time_boot_ms_GET() == 1763139253L);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.value_SET(1.0444303E38F) ;
        p251.time_boot_ms_SET(1763139253L) ;
        p251.name_SET("ysphdzj", PH) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == 1530790955);
            assert(pack.name_LEN(ph) == 10);
            assert(pack.name_TRY(ph).equals("ycdgrrkfcx"));
            assert(pack.time_boot_ms_GET() == 932405908L);
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.name_SET("ycdgrrkfcx", PH) ;
        p252.value_SET(1530790955) ;
        p252.time_boot_ms_SET(932405908L) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 10);
            assert(pack.text_TRY(ph).equals("vvtahzqvvb"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_DEBUG);
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("vvtahzqvvb", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_DEBUG) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2916630077L);
            assert(pack.ind_GET() == (char)253);
            assert(pack.value_GET() == -1.2374578E38F);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.ind_SET((char)253) ;
        p254.value_SET(-1.2374578E38F) ;
        p254.time_boot_ms_SET(2916630077L) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.initial_timestamp_GET() == 5157336123854078024L);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)0, (char)64, (char)194, (char)244, (char)31, (char)234, (char)244, (char)249, (char)2, (char)82, (char)61, (char)237, (char)214, (char)111, (char)154, (char)235, (char)85, (char)183, (char)23, (char)45, (char)179, (char)79, (char)163, (char)6, (char)48, (char)183, (char)215, (char)31, (char)114, (char)185, (char)176, (char)180}));
            assert(pack.target_component_GET() == (char)60);
            assert(pack.target_system_GET() == (char)40);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_component_SET((char)60) ;
        p256.target_system_SET((char)40) ;
        p256.initial_timestamp_SET(5157336123854078024L) ;
        p256.secret_key_SET(new char[] {(char)0, (char)64, (char)194, (char)244, (char)31, (char)234, (char)244, (char)249, (char)2, (char)82, (char)61, (char)237, (char)214, (char)111, (char)154, (char)235, (char)85, (char)183, (char)23, (char)45, (char)179, (char)79, (char)163, (char)6, (char)48, (char)183, (char)215, (char)31, (char)114, (char)185, (char)176, (char)180}, 0) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.state_GET() == (char)175);
            assert(pack.last_change_ms_GET() == 2956460465L);
            assert(pack.time_boot_ms_GET() == 1218630694L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(1218630694L) ;
        p257.last_change_ms_SET(2956460465L) ;
        p257.state_SET((char)175) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)71);
            assert(pack.target_component_GET() == (char)198);
            assert(pack.tune_LEN(ph) == 2);
            assert(pack.tune_TRY(ph).equals("me"));
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)71) ;
        p258.target_component_SET((char)198) ;
        p258.tune_SET("me", PH) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.cam_definition_uri_LEN(ph) == 64);
            assert(pack.cam_definition_uri_TRY(ph).equals("qwapysfqdwtlkbluycfdrqtmheaxzppzzgqcbqatolekfwgaqwaAwptiwuesiaky"));
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)78, (char)251, (char)11, (char)138, (char)192, (char)54, (char)58, (char)0, (char)92, (char)47, (char)187, (char)161, (char)167, (char)89, (char)133, (char)45, (char)155, (char)182, (char)94, (char)21, (char)63, (char)115, (char)50, (char)170, (char)117, (char)255, (char)171, (char)122, (char)119, (char)146, (char)85, (char)133}));
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)92, (char)56, (char)79, (char)60, (char)7, (char)71, (char)89, (char)27, (char)134, (char)85, (char)227, (char)94, (char)110, (char)238, (char)53, (char)21, (char)120, (char)18, (char)175, (char)91, (char)15, (char)58, (char)164, (char)50, (char)223, (char)76, (char)106, (char)94, (char)59, (char)68, (char)24, (char)50}));
            assert(pack.cam_definition_version_GET() == (char)9236);
            assert(pack.firmware_version_GET() == 4047974346L);
            assert(pack.lens_id_GET() == (char)59);
            assert(pack.focal_length_GET() == 1.5593516E38F);
            assert(pack.time_boot_ms_GET() == 3141551026L);
            assert(pack.sensor_size_h_GET() == -1.9896228E38F);
            assert(pack.resolution_v_GET() == (char)39763);
            assert(pack.resolution_h_GET() == (char)61764);
            assert(pack.sensor_size_v_GET() == 1.1003502E38F);
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO));
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.focal_length_SET(1.5593516E38F) ;
        p259.sensor_size_v_SET(1.1003502E38F) ;
        p259.lens_id_SET((char)59) ;
        p259.model_name_SET(new char[] {(char)92, (char)56, (char)79, (char)60, (char)7, (char)71, (char)89, (char)27, (char)134, (char)85, (char)227, (char)94, (char)110, (char)238, (char)53, (char)21, (char)120, (char)18, (char)175, (char)91, (char)15, (char)58, (char)164, (char)50, (char)223, (char)76, (char)106, (char)94, (char)59, (char)68, (char)24, (char)50}, 0) ;
        p259.resolution_h_SET((char)61764) ;
        p259.vendor_name_SET(new char[] {(char)78, (char)251, (char)11, (char)138, (char)192, (char)54, (char)58, (char)0, (char)92, (char)47, (char)187, (char)161, (char)167, (char)89, (char)133, (char)45, (char)155, (char)182, (char)94, (char)21, (char)63, (char)115, (char)50, (char)170, (char)117, (char)255, (char)171, (char)122, (char)119, (char)146, (char)85, (char)133}, 0) ;
        p259.firmware_version_SET(4047974346L) ;
        p259.time_boot_ms_SET(3141551026L) ;
        p259.resolution_v_SET((char)39763) ;
        p259.cam_definition_version_SET((char)9236) ;
        p259.cam_definition_uri_SET("qwapysfqdwtlkbluycfdrqtmheaxzppzzgqcbqatolekfwgaqwaAwptiwuesiaky", PH) ;
        p259.sensor_size_h_SET(-1.9896228E38F) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO)) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.mode_id_GET() == (CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY));
            assert(pack.time_boot_ms_GET() == 3506956473L);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(3506956473L) ;
        p260.mode_id_SET((CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY)) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.total_capacity_GET() == -2.8603536E38F);
            assert(pack.read_speed_GET() == 1.2102385E38F);
            assert(pack.write_speed_GET() == 1.7217711E38F);
            assert(pack.storage_count_GET() == (char)75);
            assert(pack.used_capacity_GET() == 2.3228251E38F);
            assert(pack.status_GET() == (char)26);
            assert(pack.storage_id_GET() == (char)227);
            assert(pack.time_boot_ms_GET() == 582918660L);
            assert(pack.available_capacity_GET() == 1.5310358E38F);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.used_capacity_SET(2.3228251E38F) ;
        p261.write_speed_SET(1.7217711E38F) ;
        p261.read_speed_SET(1.2102385E38F) ;
        p261.storage_count_SET((char)75) ;
        p261.time_boot_ms_SET(582918660L) ;
        p261.storage_id_SET((char)227) ;
        p261.available_capacity_SET(1.5310358E38F) ;
        p261.total_capacity_SET(-2.8603536E38F) ;
        p261.status_SET((char)26) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.image_status_GET() == (char)83);
            assert(pack.recording_time_ms_GET() == 4046257154L);
            assert(pack.available_capacity_GET() == -1.5572034E37F);
            assert(pack.video_status_GET() == (char)133);
            assert(pack.time_boot_ms_GET() == 1356086893L);
            assert(pack.image_interval_GET() == 2.7948449E38F);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.available_capacity_SET(-1.5572034E37F) ;
        p262.image_status_SET((char)83) ;
        p262.recording_time_ms_SET(4046257154L) ;
        p262.time_boot_ms_SET(1356086893L) ;
        p262.image_interval_SET(2.7948449E38F) ;
        p262.video_status_SET((char)133) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.file_url_LEN(ph) == 25);
            assert(pack.file_url_TRY(ph).equals("kqkltyyqwkfgZswkroaBfhzqC"));
            assert(pack.time_boot_ms_GET() == 3143091904L);
            assert(pack.lon_GET() == 1504045202);
            assert(pack.camera_id_GET() == (char)241);
            assert(pack.lat_GET() == 74933920);
            assert(pack.capture_result_GET() == (byte)68);
            assert(pack.relative_alt_GET() == -154009771);
            assert(pack.alt_GET() == 57918369);
            assert(pack.image_index_GET() == 1019464202);
            assert(pack.time_utc_GET() == 621103231748171733L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.740023E38F, -2.231418E38F, 2.5386482E38F, -1.8313742E38F}));
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.camera_id_SET((char)241) ;
        p263.time_boot_ms_SET(3143091904L) ;
        p263.alt_SET(57918369) ;
        p263.time_utc_SET(621103231748171733L) ;
        p263.q_SET(new float[] {2.740023E38F, -2.231418E38F, 2.5386482E38F, -1.8313742E38F}, 0) ;
        p263.file_url_SET("kqkltyyqwkfgZswkroaBfhzqC", PH) ;
        p263.image_index_SET(1019464202) ;
        p263.lat_SET(74933920) ;
        p263.lon_SET(1504045202) ;
        p263.relative_alt_SET(-154009771) ;
        p263.capture_result_SET((byte)68) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.flight_uuid_GET() == 1966650102307132254L);
            assert(pack.time_boot_ms_GET() == 2389376526L);
            assert(pack.arming_time_utc_GET() == 4730235497790288814L);
            assert(pack.takeoff_time_utc_GET() == 412948851480806437L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.flight_uuid_SET(1966650102307132254L) ;
        p264.arming_time_utc_SET(4730235497790288814L) ;
        p264.takeoff_time_utc_SET(412948851480806437L) ;
        p264.time_boot_ms_SET(2389376526L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2048865460L);
            assert(pack.yaw_GET() == -2.8807478E38F);
            assert(pack.roll_GET() == 2.3195385E38F);
            assert(pack.pitch_GET() == 2.6725519E38F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.roll_SET(2.3195385E38F) ;
        p265.time_boot_ms_SET(2048865460L) ;
        p265.pitch_SET(2.6725519E38F) ;
        p265.yaw_SET(-2.8807478E38F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.length_GET() == (char)122);
            assert(pack.sequence_GET() == (char)60953);
            assert(pack.target_component_GET() == (char)172);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)21, (char)21, (char)119, (char)224, (char)132, (char)86, (char)124, (char)207, (char)110, (char)157, (char)180, (char)80, (char)10, (char)120, (char)124, (char)20, (char)165, (char)211, (char)190, (char)221, (char)139, (char)171, (char)254, (char)192, (char)54, (char)176, (char)24, (char)103, (char)225, (char)178, (char)131, (char)68, (char)96, (char)11, (char)161, (char)101, (char)87, (char)190, (char)89, (char)239, (char)239, (char)108, (char)245, (char)111, (char)8, (char)81, (char)47, (char)167, (char)65, (char)149, (char)128, (char)56, (char)97, (char)235, (char)140, (char)121, (char)232, (char)77, (char)65, (char)9, (char)51, (char)61, (char)144, (char)12, (char)82, (char)210, (char)200, (char)214, (char)0, (char)2, (char)139, (char)28, (char)13, (char)175, (char)174, (char)2, (char)46, (char)16, (char)169, (char)27, (char)192, (char)214, (char)227, (char)138, (char)33, (char)38, (char)152, (char)56, (char)43, (char)121, (char)10, (char)141, (char)8, (char)195, (char)6, (char)246, (char)166, (char)52, (char)250, (char)192, (char)165, (char)71, (char)131, (char)23, (char)69, (char)181, (char)11, (char)156, (char)239, (char)27, (char)224, (char)111, (char)66, (char)4, (char)66, (char)108, (char)219, (char)41, (char)118, (char)153, (char)97, (char)57, (char)82, (char)71, (char)190, (char)48, (char)9, (char)60, (char)134, (char)93, (char)139, (char)248, (char)242, (char)51, (char)196, (char)73, (char)179, (char)162, (char)137, (char)120, (char)169, (char)239, (char)111, (char)225, (char)47, (char)41, (char)150, (char)47, (char)70, (char)146, (char)114, (char)115, (char)177, (char)177, (char)158, (char)249, (char)142, (char)134, (char)39, (char)38, (char)227, (char)167, (char)142, (char)254, (char)57, (char)191, (char)95, (char)134, (char)103, (char)153, (char)21, (char)103, (char)225, (char)98, (char)103, (char)205, (char)235, (char)124, (char)225, (char)58, (char)62, (char)81, (char)93, (char)142, (char)178, (char)112, (char)223, (char)186, (char)202, (char)238, (char)16, (char)244, (char)217, (char)91, (char)15, (char)56, (char)153, (char)20, (char)101, (char)134, (char)255, (char)176, (char)99, (char)223, (char)206, (char)200, (char)52, (char)153, (char)61, (char)0, (char)19, (char)92, (char)44, (char)147, (char)214, (char)126, (char)199, (char)35, (char)34, (char)189, (char)180, (char)226, (char)124, (char)17, (char)103, (char)250, (char)139, (char)184, (char)83, (char)44, (char)254, (char)140, (char)146, (char)204, (char)202, (char)237, (char)245, (char)128, (char)170, (char)168, (char)204, (char)12, (char)148, (char)166, (char)37, (char)68, (char)236, (char)215, (char)67}));
            assert(pack.first_message_offset_GET() == (char)155);
            assert(pack.target_system_GET() == (char)138);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.data__SET(new char[] {(char)21, (char)21, (char)119, (char)224, (char)132, (char)86, (char)124, (char)207, (char)110, (char)157, (char)180, (char)80, (char)10, (char)120, (char)124, (char)20, (char)165, (char)211, (char)190, (char)221, (char)139, (char)171, (char)254, (char)192, (char)54, (char)176, (char)24, (char)103, (char)225, (char)178, (char)131, (char)68, (char)96, (char)11, (char)161, (char)101, (char)87, (char)190, (char)89, (char)239, (char)239, (char)108, (char)245, (char)111, (char)8, (char)81, (char)47, (char)167, (char)65, (char)149, (char)128, (char)56, (char)97, (char)235, (char)140, (char)121, (char)232, (char)77, (char)65, (char)9, (char)51, (char)61, (char)144, (char)12, (char)82, (char)210, (char)200, (char)214, (char)0, (char)2, (char)139, (char)28, (char)13, (char)175, (char)174, (char)2, (char)46, (char)16, (char)169, (char)27, (char)192, (char)214, (char)227, (char)138, (char)33, (char)38, (char)152, (char)56, (char)43, (char)121, (char)10, (char)141, (char)8, (char)195, (char)6, (char)246, (char)166, (char)52, (char)250, (char)192, (char)165, (char)71, (char)131, (char)23, (char)69, (char)181, (char)11, (char)156, (char)239, (char)27, (char)224, (char)111, (char)66, (char)4, (char)66, (char)108, (char)219, (char)41, (char)118, (char)153, (char)97, (char)57, (char)82, (char)71, (char)190, (char)48, (char)9, (char)60, (char)134, (char)93, (char)139, (char)248, (char)242, (char)51, (char)196, (char)73, (char)179, (char)162, (char)137, (char)120, (char)169, (char)239, (char)111, (char)225, (char)47, (char)41, (char)150, (char)47, (char)70, (char)146, (char)114, (char)115, (char)177, (char)177, (char)158, (char)249, (char)142, (char)134, (char)39, (char)38, (char)227, (char)167, (char)142, (char)254, (char)57, (char)191, (char)95, (char)134, (char)103, (char)153, (char)21, (char)103, (char)225, (char)98, (char)103, (char)205, (char)235, (char)124, (char)225, (char)58, (char)62, (char)81, (char)93, (char)142, (char)178, (char)112, (char)223, (char)186, (char)202, (char)238, (char)16, (char)244, (char)217, (char)91, (char)15, (char)56, (char)153, (char)20, (char)101, (char)134, (char)255, (char)176, (char)99, (char)223, (char)206, (char)200, (char)52, (char)153, (char)61, (char)0, (char)19, (char)92, (char)44, (char)147, (char)214, (char)126, (char)199, (char)35, (char)34, (char)189, (char)180, (char)226, (char)124, (char)17, (char)103, (char)250, (char)139, (char)184, (char)83, (char)44, (char)254, (char)140, (char)146, (char)204, (char)202, (char)237, (char)245, (char)128, (char)170, (char)168, (char)204, (char)12, (char)148, (char)166, (char)37, (char)68, (char)236, (char)215, (char)67}, 0) ;
        p266.length_SET((char)122) ;
        p266.target_system_SET((char)138) ;
        p266.target_component_SET((char)172) ;
        p266.first_message_offset_SET((char)155) ;
        p266.sequence_SET((char)60953) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)139, (char)82, (char)223, (char)190, (char)147, (char)162, (char)34, (char)224, (char)101, (char)167, (char)119, (char)56, (char)68, (char)228, (char)198, (char)13, (char)202, (char)156, (char)255, (char)19, (char)170, (char)8, (char)79, (char)71, (char)101, (char)74, (char)156, (char)104, (char)14, (char)172, (char)139, (char)52, (char)96, (char)192, (char)63, (char)30, (char)22, (char)24, (char)47, (char)140, (char)121, (char)19, (char)180, (char)251, (char)64, (char)26, (char)255, (char)202, (char)126, (char)198, (char)89, (char)0, (char)158, (char)206, (char)68, (char)63, (char)205, (char)42, (char)16, (char)16, (char)214, (char)25, (char)118, (char)9, (char)217, (char)154, (char)77, (char)253, (char)141, (char)42, (char)205, (char)33, (char)127, (char)106, (char)254, (char)153, (char)137, (char)198, (char)146, (char)3, (char)25, (char)42, (char)169, (char)25, (char)167, (char)233, (char)180, (char)50, (char)161, (char)16, (char)55, (char)75, (char)7, (char)251, (char)25, (char)71, (char)31, (char)165, (char)182, (char)193, (char)198, (char)172, (char)8, (char)219, (char)125, (char)126, (char)243, (char)176, (char)24, (char)162, (char)114, (char)205, (char)73, (char)112, (char)42, (char)4, (char)82, (char)25, (char)181, (char)77, (char)28, (char)13, (char)37, (char)7, (char)216, (char)141, (char)121, (char)46, (char)54, (char)119, (char)102, (char)100, (char)0, (char)251, (char)135, (char)66, (char)93, (char)40, (char)161, (char)35, (char)166, (char)90, (char)230, (char)122, (char)124, (char)39, (char)56, (char)1, (char)20, (char)34, (char)48, (char)216, (char)94, (char)179, (char)207, (char)39, (char)43, (char)83, (char)75, (char)147, (char)228, (char)187, (char)104, (char)252, (char)218, (char)185, (char)239, (char)237, (char)209, (char)102, (char)75, (char)232, (char)247, (char)164, (char)113, (char)172, (char)172, (char)83, (char)80, (char)245, (char)207, (char)86, (char)134, (char)170, (char)5, (char)210, (char)51, (char)38, (char)207, (char)145, (char)251, (char)149, (char)202, (char)115, (char)67, (char)65, (char)170, (char)168, (char)26, (char)138, (char)216, (char)134, (char)186, (char)50, (char)199, (char)84, (char)58, (char)171, (char)21, (char)201, (char)134, (char)71, (char)62, (char)59, (char)170, (char)91, (char)240, (char)190, (char)102, (char)4, (char)31, (char)131, (char)88, (char)18, (char)180, (char)53, (char)50, (char)73, (char)23, (char)88, (char)26, (char)227, (char)152, (char)52, (char)206, (char)180, (char)181, (char)217, (char)105, (char)115, (char)101, (char)170, (char)98, (char)194, (char)37, (char)137, (char)69, (char)94, (char)73}));
            assert(pack.length_GET() == (char)100);
            assert(pack.target_system_GET() == (char)70);
            assert(pack.first_message_offset_GET() == (char)208);
            assert(pack.sequence_GET() == (char)40496);
            assert(pack.target_component_GET() == (char)91);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.sequence_SET((char)40496) ;
        p267.target_component_SET((char)91) ;
        p267.data__SET(new char[] {(char)139, (char)82, (char)223, (char)190, (char)147, (char)162, (char)34, (char)224, (char)101, (char)167, (char)119, (char)56, (char)68, (char)228, (char)198, (char)13, (char)202, (char)156, (char)255, (char)19, (char)170, (char)8, (char)79, (char)71, (char)101, (char)74, (char)156, (char)104, (char)14, (char)172, (char)139, (char)52, (char)96, (char)192, (char)63, (char)30, (char)22, (char)24, (char)47, (char)140, (char)121, (char)19, (char)180, (char)251, (char)64, (char)26, (char)255, (char)202, (char)126, (char)198, (char)89, (char)0, (char)158, (char)206, (char)68, (char)63, (char)205, (char)42, (char)16, (char)16, (char)214, (char)25, (char)118, (char)9, (char)217, (char)154, (char)77, (char)253, (char)141, (char)42, (char)205, (char)33, (char)127, (char)106, (char)254, (char)153, (char)137, (char)198, (char)146, (char)3, (char)25, (char)42, (char)169, (char)25, (char)167, (char)233, (char)180, (char)50, (char)161, (char)16, (char)55, (char)75, (char)7, (char)251, (char)25, (char)71, (char)31, (char)165, (char)182, (char)193, (char)198, (char)172, (char)8, (char)219, (char)125, (char)126, (char)243, (char)176, (char)24, (char)162, (char)114, (char)205, (char)73, (char)112, (char)42, (char)4, (char)82, (char)25, (char)181, (char)77, (char)28, (char)13, (char)37, (char)7, (char)216, (char)141, (char)121, (char)46, (char)54, (char)119, (char)102, (char)100, (char)0, (char)251, (char)135, (char)66, (char)93, (char)40, (char)161, (char)35, (char)166, (char)90, (char)230, (char)122, (char)124, (char)39, (char)56, (char)1, (char)20, (char)34, (char)48, (char)216, (char)94, (char)179, (char)207, (char)39, (char)43, (char)83, (char)75, (char)147, (char)228, (char)187, (char)104, (char)252, (char)218, (char)185, (char)239, (char)237, (char)209, (char)102, (char)75, (char)232, (char)247, (char)164, (char)113, (char)172, (char)172, (char)83, (char)80, (char)245, (char)207, (char)86, (char)134, (char)170, (char)5, (char)210, (char)51, (char)38, (char)207, (char)145, (char)251, (char)149, (char)202, (char)115, (char)67, (char)65, (char)170, (char)168, (char)26, (char)138, (char)216, (char)134, (char)186, (char)50, (char)199, (char)84, (char)58, (char)171, (char)21, (char)201, (char)134, (char)71, (char)62, (char)59, (char)170, (char)91, (char)240, (char)190, (char)102, (char)4, (char)31, (char)131, (char)88, (char)18, (char)180, (char)53, (char)50, (char)73, (char)23, (char)88, (char)26, (char)227, (char)152, (char)52, (char)206, (char)180, (char)181, (char)217, (char)105, (char)115, (char)101, (char)170, (char)98, (char)194, (char)37, (char)137, (char)69, (char)94, (char)73}, 0) ;
        p267.length_SET((char)100) ;
        p267.target_system_SET((char)70) ;
        p267.first_message_offset_SET((char)208) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)43338);
            assert(pack.target_component_GET() == (char)249);
            assert(pack.target_system_GET() == (char)73);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_component_SET((char)249) ;
        p268.sequence_SET((char)43338) ;
        p268.target_system_SET((char)73) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.camera_id_GET() == (char)138);
            assert(pack.framerate_GET() == -7.412047E37F);
            assert(pack.resolution_h_GET() == (char)59492);
            assert(pack.rotation_GET() == (char)31418);
            assert(pack.status_GET() == (char)52);
            assert(pack.uri_LEN(ph) == 230);
            assert(pack.uri_TRY(ph).equals("iwzuyhgiyuwvlykbzGplBbfpyxmliaxxvsnhhKvqyeooqRwwdLctnqtjhQupdepoxCfSssfrobVffgrqumscbqamUjkbmutShpwguyibgqlmcbuqfejurfRqgxCvuaoduwchtchvjceirdiviwlgnglfnxtjsbpqyeiaoadbbrzfrbcgbXeauyokmvultbHpEhejizbutkhrRwazeaRdgshkNzPvmkVallkvfe"));
            assert(pack.bitrate_GET() == 1236686875L);
            assert(pack.resolution_v_GET() == (char)5512);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.framerate_SET(-7.412047E37F) ;
        p269.status_SET((char)52) ;
        p269.resolution_v_SET((char)5512) ;
        p269.camera_id_SET((char)138) ;
        p269.resolution_h_SET((char)59492) ;
        p269.rotation_SET((char)31418) ;
        p269.bitrate_SET(1236686875L) ;
        p269.uri_SET("iwzuyhgiyuwvlykbzGplBbfpyxmliaxxvsnhhKvqyeooqRwwdLctnqtjhQupdepoxCfSssfrobVffgrqumscbqamUjkbmutShpwguyibgqlmcbuqfejurfRqgxCvuaoduwchtchvjceirdiviwlgnglfnxtjsbpqyeiaoadbbrzfrbcgbXeauyokmvultbHpEhejizbutkhrRwazeaRdgshkNzPvmkVallkvfe", PH) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.uri_LEN(ph) == 53);
            assert(pack.uri_TRY(ph).equals("naHLinlpsvpzxuvrwKwmloxxsrfUitmrpiswzjidwbiarbgjsqset"));
            assert(pack.framerate_GET() == -9.150709E37F);
            assert(pack.target_component_GET() == (char)84);
            assert(pack.camera_id_GET() == (char)27);
            assert(pack.target_system_GET() == (char)81);
            assert(pack.bitrate_GET() == 1829685949L);
            assert(pack.resolution_h_GET() == (char)9474);
            assert(pack.resolution_v_GET() == (char)51339);
            assert(pack.rotation_GET() == (char)36809);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.camera_id_SET((char)27) ;
        p270.rotation_SET((char)36809) ;
        p270.target_component_SET((char)84) ;
        p270.uri_SET("naHLinlpsvpzxuvrwKwmloxxsrfUitmrpiswzjidwbiarbgjsqset", PH) ;
        p270.resolution_h_SET((char)9474) ;
        p270.target_system_SET((char)81) ;
        p270.resolution_v_SET((char)51339) ;
        p270.bitrate_SET(1829685949L) ;
        p270.framerate_SET(-9.150709E37F) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.password_LEN(ph) == 20);
            assert(pack.password_TRY(ph).equals("mmfXhyjcqzjrwfprawpt"));
            assert(pack.ssid_LEN(ph) == 19);
            assert(pack.ssid_TRY(ph).equals("wvUqhFnpfvdndfpqhpO"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("mmfXhyjcqzjrwfprawpt", PH) ;
        p299.ssid_SET("wvUqhFnpfvdndfpqhpO", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)125, (char)150, (char)15, (char)207, (char)114, (char)60, (char)52, (char)249}));
            assert(pack.max_version_GET() == (char)56924);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)139, (char)237, (char)52, (char)50, (char)160, (char)217, (char)24, (char)2}));
            assert(pack.version_GET() == (char)49686);
            assert(pack.min_version_GET() == (char)64853);
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.spec_version_hash_SET(new char[] {(char)139, (char)237, (char)52, (char)50, (char)160, (char)217, (char)24, (char)2}, 0) ;
        p300.version_SET((char)49686) ;
        p300.min_version_SET((char)64853) ;
        p300.max_version_SET((char)56924) ;
        p300.library_version_hash_SET(new char[] {(char)125, (char)150, (char)15, (char)207, (char)114, (char)60, (char)52, (char)249}, 0) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 9129072571435839484L);
            assert(pack.uptime_sec_GET() == 2510356083L);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
            assert(pack.sub_mode_GET() == (char)49);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
            assert(pack.vendor_specific_status_code_GET() == (char)32074);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.uptime_sec_SET(2510356083L) ;
        p310.sub_mode_SET((char)49) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE) ;
        p310.vendor_specific_status_code_SET((char)32074) ;
        p310.time_usec_SET(9129072571435839484L) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.sw_version_minor_GET() == (char)131);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)107, (char)32, (char)156, (char)35, (char)202, (char)59, (char)62, (char)70, (char)55, (char)91, (char)12, (char)224, (char)188, (char)92, (char)0, (char)152}));
            assert(pack.sw_version_major_GET() == (char)200);
            assert(pack.hw_version_major_GET() == (char)131);
            assert(pack.time_usec_GET() == 552163127708682002L);
            assert(pack.sw_vcs_commit_GET() == 921795001L);
            assert(pack.hw_version_minor_GET() == (char)75);
            assert(pack.uptime_sec_GET() == 554866053L);
            assert(pack.name_LEN(ph) == 57);
            assert(pack.name_TRY(ph).equals("dergsgpcqOcmncqzJvzaMMlncogwibfrsjGnrqFdvohpfvgGgzTljFtCa"));
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.hw_version_major_SET((char)131) ;
        p311.sw_version_minor_SET((char)131) ;
        p311.uptime_sec_SET(554866053L) ;
        p311.sw_vcs_commit_SET(921795001L) ;
        p311.name_SET("dergsgpcqOcmncqzJvzaMMlncogwibfrsjGnrqFdvohpfvgGgzTljFtCa", PH) ;
        p311.sw_version_major_SET((char)200) ;
        p311.time_usec_SET(552163127708682002L) ;
        p311.hw_unique_id_SET(new char[] {(char)107, (char)32, (char)156, (char)35, (char)202, (char)59, (char)62, (char)70, (char)55, (char)91, (char)12, (char)224, (char)188, (char)92, (char)0, (char)152}, 0) ;
        p311.hw_version_minor_SET((char)75) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)156);
            assert(pack.target_system_GET() == (char)166);
            assert(pack.param_id_LEN(ph) == 3);
            assert(pack.param_id_TRY(ph).equals("rea"));
            assert(pack.param_index_GET() == (short)21960);
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)166) ;
        p320.param_id_SET("rea", PH) ;
        p320.param_index_SET((short)21960) ;
        p320.target_component_SET((char)156) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)54);
            assert(pack.target_component_GET() == (char)67);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)54) ;
        p321.target_component_SET((char)67) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("bowJtalhs"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
            assert(pack.param_count_GET() == (char)28703);
            assert(pack.param_index_GET() == (char)44184);
            assert(pack.param_value_LEN(ph) == 44);
            assert(pack.param_value_TRY(ph).equals("tzcubdstfveTiubtTbrDzadrgkiayWmembxrkmwdscvs"));
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_count_SET((char)28703) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32) ;
        p322.param_value_SET("tzcubdstfveTiubtTbrDzadrgkiayWmembxrkmwdscvs", PH) ;
        p322.param_index_SET((char)44184) ;
        p322.param_id_SET("bowJtalhs", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)18);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("c"));
            assert(pack.param_value_LEN(ph) == 44);
            assert(pack.param_value_TRY(ph).equals("fcopbjrgvkvbbknnuljqrsygmzywgndsBQPbrfpsemiS"));
            assert(pack.target_system_GET() == (char)212);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_id_SET("c", PH) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8) ;
        p323.target_system_SET((char)212) ;
        p323.param_value_SET("fcopbjrgvkvbbknnuljqrsygmzywgndsBQPbrfpsemiS", PH) ;
        p323.target_component_SET((char)18) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("rvkr"));
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_IN_PROGRESS);
            assert(pack.param_value_LEN(ph) == 106);
            assert(pack.param_value_TRY(ph).equals("lEvtrJxrvajupqhvoyrndvmqeioqwyjdwkLfnsnuwoakjuyfcngvytufsobykQrnxlyqsozVmwablbqbgqffJuajKqbiuxuqyonxoRiouD"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_IN_PROGRESS) ;
        p324.param_value_SET("lEvtrJxrvajupqhvoyrndvmqeioqwyjdwkLfnsnuwoakjuyfcngvytufsobykQrnxlyqsozVmwablbqbgqffJuajKqbiuxuqyonxoRiouD", PH) ;
        p324.param_id_SET("rvkr", PH) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.increment_GET() == (char)198);
            assert(pack.time_usec_GET() == 2860945053756408352L);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            assert(pack.min_distance_GET() == (char)31178);
            assert(pack.max_distance_GET() == (char)17776);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)37275, (char)32636, (char)40938, (char)20062, (char)27863, (char)41752, (char)25444, (char)35610, (char)19503, (char)43002, (char)63987, (char)26297, (char)37219, (char)33239, (char)64033, (char)19173, (char)32005, (char)50704, (char)18678, (char)31096, (char)22915, (char)25678, (char)17085, (char)8907, (char)29727, (char)47313, (char)19201, (char)45870, (char)54395, (char)14051, (char)38145, (char)29359, (char)35374, (char)60011, (char)22598, (char)51322, (char)3050, (char)19647, (char)32931, (char)34432, (char)1552, (char)40743, (char)27352, (char)24613, (char)61252, (char)53919, (char)48972, (char)5920, (char)23945, (char)29515, (char)41711, (char)36073, (char)48969, (char)64648, (char)47848, (char)61628, (char)25606, (char)29959, (char)39003, (char)46224, (char)55103, (char)43578, (char)58868, (char)20289, (char)26569, (char)44370, (char)29786, (char)15839, (char)29154, (char)9752, (char)1608, (char)46601}));
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        p330.min_distance_SET((char)31178) ;
        p330.increment_SET((char)198) ;
        p330.max_distance_SET((char)17776) ;
        p330.distances_SET(new char[] {(char)37275, (char)32636, (char)40938, (char)20062, (char)27863, (char)41752, (char)25444, (char)35610, (char)19503, (char)43002, (char)63987, (char)26297, (char)37219, (char)33239, (char)64033, (char)19173, (char)32005, (char)50704, (char)18678, (char)31096, (char)22915, (char)25678, (char)17085, (char)8907, (char)29727, (char)47313, (char)19201, (char)45870, (char)54395, (char)14051, (char)38145, (char)29359, (char)35374, (char)60011, (char)22598, (char)51322, (char)3050, (char)19647, (char)32931, (char)34432, (char)1552, (char)40743, (char)27352, (char)24613, (char)61252, (char)53919, (char)48972, (char)5920, (char)23945, (char)29515, (char)41711, (char)36073, (char)48969, (char)64648, (char)47848, (char)61628, (char)25606, (char)29959, (char)39003, (char)46224, (char)55103, (char)43578, (char)58868, (char)20289, (char)26569, (char)44370, (char)29786, (char)15839, (char)29154, (char)9752, (char)1608, (char)46601}, 0) ;
        p330.time_usec_SET(2860945053756408352L) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
    }

}
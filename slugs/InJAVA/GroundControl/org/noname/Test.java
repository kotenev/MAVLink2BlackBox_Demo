
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
        *	 0 ignore*/
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
        *	 (2) Distance in cm (3) Absolute valu*/
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
        *	 mode, 7=Manual input mode (fixed position), 8=Simulator mode, 9= WAAS*/
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
        *	 (W) adds to True cours*/
        public byte magDir_GET()
        {  return (byte)((byte) get_bytes(data,  9, 1)); }
        /**
        *Positioning system mode indicator. A - Autonomous;D-Differential; E-Estimated (dead reckoning) mode;M-Manual
        *	 input; N-Data not vali*/
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
                case 83:
                    if(pack == null) return new ATTITUDE_TARGET();
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
            assert(pack.custom_mode_GET() == 1750339552L);
            assert(pack.mavlink_version_GET() == (char)41);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_CALIBRATING);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_FREE_BALLOON);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_INVALID);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED));
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.mavlink_version_SET((char)41) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED)) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_CALIBRATING) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_FREE_BALLOON) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_INVALID) ;
        p0.custom_mode_SET(1750339552L) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.errors_comm_GET() == (char)49919);
            assert(pack.voltage_battery_GET() == (char)15517);
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW));
            assert(pack.load_GET() == (char)13763);
            assert(pack.errors_count4_GET() == (char)49841);
            assert(pack.errors_count2_GET() == (char)26532);
            assert(pack.drop_rate_comm_GET() == (char)44989);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.errors_count3_GET() == (char)21457);
            assert(pack.errors_count1_GET() == (char)60927);
            assert(pack.current_battery_GET() == (short)25570);
            assert(pack.battery_remaining_GET() == (byte) - 45);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.errors_comm_SET((char)49919) ;
        p1.current_battery_SET((short)25570) ;
        p1.drop_rate_comm_SET((char)44989) ;
        p1.errors_count3_SET((char)21457) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL)) ;
        p1.errors_count4_SET((char)49841) ;
        p1.errors_count2_SET((char)26532) ;
        p1.errors_count1_SET((char)60927) ;
        p1.voltage_battery_SET((char)15517) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        p1.load_SET((char)13763) ;
        p1.battery_remaining_SET((byte) - 45) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW)) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2725398862L);
            assert(pack.time_unix_usec_GET() == 9034200107758639726L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(9034200107758639726L) ;
        p2.time_boot_ms_SET(2725398862L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == -1.4836737E37F);
            assert(pack.afy_GET() == -2.4553457E38F);
            assert(pack.x_GET() == -3.2432218E38F);
            assert(pack.yaw_rate_GET() == 3.1488035E38F);
            assert(pack.afz_GET() == 2.0174811E38F);
            assert(pack.y_GET() == -3.0260958E38F);
            assert(pack.z_GET() == 2.0925346E38F);
            assert(pack.yaw_GET() == -8.459764E37F);
            assert(pack.vy_GET() == 2.7984797E38F);
            assert(pack.vz_GET() == -2.4774176E38F);
            assert(pack.time_boot_ms_GET() == 3271016599L);
            assert(pack.type_mask_GET() == (char)51230);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.afx_GET() == 3.206279E37F);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p3.vx_SET(-1.4836737E37F) ;
        p3.yaw_SET(-8.459764E37F) ;
        p3.afy_SET(-2.4553457E38F) ;
        p3.time_boot_ms_SET(3271016599L) ;
        p3.vz_SET(-2.4774176E38F) ;
        p3.y_SET(-3.0260958E38F) ;
        p3.yaw_rate_SET(3.1488035E38F) ;
        p3.z_SET(2.0925346E38F) ;
        p3.afz_SET(2.0174811E38F) ;
        p3.type_mask_SET((char)51230) ;
        p3.afx_SET(3.206279E37F) ;
        p3.vy_SET(2.7984797E38F) ;
        p3.x_SET(-3.2432218E38F) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 8158270827484237073L);
            assert(pack.target_component_GET() == (char)41);
            assert(pack.target_system_GET() == (char)16);
            assert(pack.seq_GET() == 3369167088L);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.target_component_SET((char)41) ;
        p4.seq_SET(3369167088L) ;
        p4.time_usec_SET(8158270827484237073L) ;
        p4.target_system_SET((char)16) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.control_request_GET() == (char)124);
            assert(pack.version_GET() == (char)46);
            assert(pack.target_system_GET() == (char)110);
            assert(pack.passkey_LEN(ph) == 21);
            assert(pack.passkey_TRY(ph).equals("yhhlGifqjYzeqsbizZTtn"));
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.passkey_SET("yhhlGifqjYzeqsbizZTtn", PH) ;
        p5.version_SET((char)46) ;
        p5.control_request_SET((char)124) ;
        p5.target_system_SET((char)110) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.ack_GET() == (char)170);
            assert(pack.control_request_GET() == (char)197);
            assert(pack.gcs_system_id_GET() == (char)219);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.ack_SET((char)170) ;
        p6.control_request_SET((char)197) ;
        p6.gcs_system_id_SET((char)219) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 21);
            assert(pack.key_TRY(ph).equals("upaUdxhpzyAqvgzdoeoRw"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("upaUdxhpzyAqvgzdoeoRw", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_AUTO_DISARMED);
            assert(pack.custom_mode_GET() == 3645698972L);
            assert(pack.target_system_GET() == (char)134);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.target_system_SET((char)134) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_AUTO_DISARMED) ;
        p11.custom_mode_SET(3645698972L) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)0);
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("FfcrTkahuuYw"));
            assert(pack.param_index_GET() == (short)28015);
            assert(pack.target_system_GET() == (char)187);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_component_SET((char)0) ;
        p20.param_index_SET((short)28015) ;
        p20.param_id_SET("FfcrTkahuuYw", PH) ;
        p20.target_system_SET((char)187) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)121);
            assert(pack.target_system_GET() == (char)239);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)239) ;
        p21.target_component_SET((char)121) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_value_GET() == -2.1362655E38F);
            assert(pack.param_id_LEN(ph) == 3);
            assert(pack.param_id_TRY(ph).equals("egn"));
            assert(pack.param_count_GET() == (char)9792);
            assert(pack.param_index_GET() == (char)5368);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_id_SET("egn", PH) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16) ;
        p22.param_index_SET((char)5368) ;
        p22.param_count_SET((char)9792) ;
        p22.param_value_SET(-2.1362655E38F) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("xpwyg"));
            assert(pack.target_component_GET() == (char)111);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8);
            assert(pack.target_system_GET() == (char)163);
            assert(pack.param_value_GET() == -2.6836441E38F);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.target_system_SET((char)163) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8) ;
        p23.target_component_SET((char)111) ;
        p23.param_value_SET(-2.6836441E38F) ;
        p23.param_id_SET("xpwyg", PH) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.satellites_visible_GET() == (char)66);
            assert(pack.alt_ellipsoid_TRY(ph) == -1873245650);
            assert(pack.eph_GET() == (char)56677);
            assert(pack.h_acc_TRY(ph) == 3972572659L);
            assert(pack.epv_GET() == (char)33588);
            assert(pack.cog_GET() == (char)3136);
            assert(pack.vel_GET() == (char)24232);
            assert(pack.v_acc_TRY(ph) == 3500690078L);
            assert(pack.lat_GET() == -1843527460);
            assert(pack.lon_GET() == 242941984);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
            assert(pack.time_usec_GET() == 546196103842218964L);
            assert(pack.alt_GET() == -1327373905);
            assert(pack.hdg_acc_TRY(ph) == 878790520L);
            assert(pack.vel_acc_TRY(ph) == 3819122714L);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.alt_ellipsoid_SET(-1873245650, PH) ;
        p24.cog_SET((char)3136) ;
        p24.vel_SET((char)24232) ;
        p24.lat_SET(-1843527460) ;
        p24.satellites_visible_SET((char)66) ;
        p24.alt_SET(-1327373905) ;
        p24.lon_SET(242941984) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT) ;
        p24.h_acc_SET(3972572659L, PH) ;
        p24.eph_SET((char)56677) ;
        p24.vel_acc_SET(3819122714L, PH) ;
        p24.hdg_acc_SET(878790520L, PH) ;
        p24.time_usec_SET(546196103842218964L) ;
        p24.v_acc_SET(3500690078L, PH) ;
        p24.epv_SET((char)33588) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)211, (char)216, (char)43, (char)225, (char)32, (char)222, (char)254, (char)99, (char)26, (char)239, (char)1, (char)83, (char)62, (char)183, (char)72, (char)149, (char)73, (char)155, (char)152, (char)70}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)31, (char)204, (char)123, (char)146, (char)12, (char)14, (char)6, (char)139, (char)209, (char)240, (char)195, (char)25, (char)98, (char)50, (char)96, (char)102, (char)54, (char)79, (char)66, (char)219}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)178, (char)136, (char)146, (char)57, (char)175, (char)236, (char)149, (char)38, (char)249, (char)6, (char)68, (char)131, (char)194, (char)210, (char)171, (char)85, (char)170, (char)173, (char)119, (char)92}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)166, (char)247, (char)59, (char)227, (char)185, (char)45, (char)214, (char)74, (char)65, (char)28, (char)240, (char)72, (char)68, (char)135, (char)95, (char)72, (char)114, (char)194, (char)228, (char)47}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)152, (char)128, (char)94, (char)116, (char)102, (char)137, (char)158, (char)125, (char)122, (char)85, (char)14, (char)97, (char)63, (char)108, (char)21, (char)227, (char)231, (char)89, (char)159, (char)14}));
            assert(pack.satellites_visible_GET() == (char)238);
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_elevation_SET(new char[] {(char)31, (char)204, (char)123, (char)146, (char)12, (char)14, (char)6, (char)139, (char)209, (char)240, (char)195, (char)25, (char)98, (char)50, (char)96, (char)102, (char)54, (char)79, (char)66, (char)219}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)211, (char)216, (char)43, (char)225, (char)32, (char)222, (char)254, (char)99, (char)26, (char)239, (char)1, (char)83, (char)62, (char)183, (char)72, (char)149, (char)73, (char)155, (char)152, (char)70}, 0) ;
        p25.satellite_used_SET(new char[] {(char)178, (char)136, (char)146, (char)57, (char)175, (char)236, (char)149, (char)38, (char)249, (char)6, (char)68, (char)131, (char)194, (char)210, (char)171, (char)85, (char)170, (char)173, (char)119, (char)92}, 0) ;
        p25.satellites_visible_SET((char)238) ;
        p25.satellite_snr_SET(new char[] {(char)166, (char)247, (char)59, (char)227, (char)185, (char)45, (char)214, (char)74, (char)65, (char)28, (char)240, (char)72, (char)68, (char)135, (char)95, (char)72, (char)114, (char)194, (char)228, (char)47}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)152, (char)128, (char)94, (char)116, (char)102, (char)137, (char)158, (char)125, (char)122, (char)85, (char)14, (char)97, (char)63, (char)108, (char)21, (char)227, (char)231, (char)89, (char)159, (char)14}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.xmag_GET() == (short)4610);
            assert(pack.xgyro_GET() == (short)27124);
            assert(pack.zmag_GET() == (short)470);
            assert(pack.time_boot_ms_GET() == 280303480L);
            assert(pack.yacc_GET() == (short) -23449);
            assert(pack.zacc_GET() == (short) -18089);
            assert(pack.ymag_GET() == (short)3828);
            assert(pack.ygyro_GET() == (short)13281);
            assert(pack.xacc_GET() == (short)10252);
            assert(pack.zgyro_GET() == (short) -20510);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.zmag_SET((short)470) ;
        p26.xacc_SET((short)10252) ;
        p26.zgyro_SET((short) -20510) ;
        p26.time_boot_ms_SET(280303480L) ;
        p26.ygyro_SET((short)13281) ;
        p26.ymag_SET((short)3828) ;
        p26.xgyro_SET((short)27124) ;
        p26.zacc_SET((short) -18089) ;
        p26.yacc_SET((short) -23449) ;
        p26.xmag_SET((short)4610) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6362049182521145961L);
            assert(pack.zmag_GET() == (short) -18367);
            assert(pack.xgyro_GET() == (short)19853);
            assert(pack.zgyro_GET() == (short)30006);
            assert(pack.xmag_GET() == (short) -18937);
            assert(pack.ymag_GET() == (short) -6294);
            assert(pack.yacc_GET() == (short)6902);
            assert(pack.zacc_GET() == (short)19484);
            assert(pack.ygyro_GET() == (short) -32327);
            assert(pack.xacc_GET() == (short) -2622);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.ymag_SET((short) -6294) ;
        p27.yacc_SET((short)6902) ;
        p27.xgyro_SET((short)19853) ;
        p27.zacc_SET((short)19484) ;
        p27.zmag_SET((short) -18367) ;
        p27.ygyro_SET((short) -32327) ;
        p27.xmag_SET((short) -18937) ;
        p27.time_usec_SET(6362049182521145961L) ;
        p27.xacc_SET((short) -2622) ;
        p27.zgyro_SET((short)30006) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff2_GET() == (short)23041);
            assert(pack.temperature_GET() == (short)7319);
            assert(pack.press_abs_GET() == (short) -4660);
            assert(pack.time_usec_GET() == 2589315840578971917L);
            assert(pack.press_diff1_GET() == (short) -15103);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_diff1_SET((short) -15103) ;
        p28.time_usec_SET(2589315840578971917L) ;
        p28.temperature_SET((short)7319) ;
        p28.press_diff2_SET((short)23041) ;
        p28.press_abs_SET((short) -4660) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -14109);
            assert(pack.press_diff_GET() == -2.4081463E38F);
            assert(pack.time_boot_ms_GET() == 3505863267L);
            assert(pack.press_abs_GET() == -2.9780025E38F);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.time_boot_ms_SET(3505863267L) ;
        p29.press_diff_SET(-2.4081463E38F) ;
        p29.temperature_SET((short) -14109) ;
        p29.press_abs_SET(-2.9780025E38F) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.pitchspeed_GET() == -6.4210823E37F);
            assert(pack.time_boot_ms_GET() == 3406468436L);
            assert(pack.yaw_GET() == 4.536286E37F);
            assert(pack.yawspeed_GET() == -1.5405647E38F);
            assert(pack.roll_GET() == -1.7585646E38F);
            assert(pack.pitch_GET() == -1.3872099E38F);
            assert(pack.rollspeed_GET() == 2.8691695E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.yawspeed_SET(-1.5405647E38F) ;
        p30.pitchspeed_SET(-6.4210823E37F) ;
        p30.roll_SET(-1.7585646E38F) ;
        p30.rollspeed_SET(2.8691695E38F) ;
        p30.time_boot_ms_SET(3406468436L) ;
        p30.yaw_SET(4.536286E37F) ;
        p30.pitch_SET(-1.3872099E38F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1029043461L);
            assert(pack.q4_GET() == 1.7489974E38F);
            assert(pack.q2_GET() == 3.1499734E38F);
            assert(pack.q3_GET() == -2.2043783E38F);
            assert(pack.yawspeed_GET() == 2.6970993E38F);
            assert(pack.pitchspeed_GET() == 1.9404972E38F);
            assert(pack.rollspeed_GET() == 1.6156468E38F);
            assert(pack.q1_GET() == -2.805465E38F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q3_SET(-2.2043783E38F) ;
        p31.pitchspeed_SET(1.9404972E38F) ;
        p31.time_boot_ms_SET(1029043461L) ;
        p31.q4_SET(1.7489974E38F) ;
        p31.q1_SET(-2.805465E38F) ;
        p31.q2_SET(3.1499734E38F) ;
        p31.rollspeed_SET(1.6156468E38F) ;
        p31.yawspeed_SET(2.6970993E38F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -3.7762264E37F);
            assert(pack.time_boot_ms_GET() == 612503342L);
            assert(pack.vx_GET() == 2.9116318E38F);
            assert(pack.y_GET() == 2.7392978E38F);
            assert(pack.z_GET() == -8.272085E37F);
            assert(pack.vz_GET() == 2.7063182E38F);
            assert(pack.vy_GET() == -1.2193699E38F);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.y_SET(2.7392978E38F) ;
        p32.vy_SET(-1.2193699E38F) ;
        p32.x_SET(-3.7762264E37F) ;
        p32.vx_SET(2.9116318E38F) ;
        p32.vz_SET(2.7063182E38F) ;
        p32.z_SET(-8.272085E37F) ;
        p32.time_boot_ms_SET(612503342L) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2116093130L);
            assert(pack.vx_GET() == (short)6547);
            assert(pack.lon_GET() == -188279660);
            assert(pack.vy_GET() == (short)27364);
            assert(pack.lat_GET() == -443721407);
            assert(pack.hdg_GET() == (char)38943);
            assert(pack.relative_alt_GET() == 629452005);
            assert(pack.vz_GET() == (short) -7568);
            assert(pack.alt_GET() == -1835519683);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.vz_SET((short) -7568) ;
        p33.time_boot_ms_SET(2116093130L) ;
        p33.lon_SET(-188279660) ;
        p33.relative_alt_SET(629452005) ;
        p33.alt_SET(-1835519683) ;
        p33.vx_SET((short)6547) ;
        p33.lat_SET(-443721407) ;
        p33.vy_SET((short)27364) ;
        p33.hdg_SET((char)38943) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan3_scaled_GET() == (short) -18094);
            assert(pack.chan5_scaled_GET() == (short)30210);
            assert(pack.port_GET() == (char)64);
            assert(pack.chan2_scaled_GET() == (short) -7157);
            assert(pack.chan8_scaled_GET() == (short)23908);
            assert(pack.chan4_scaled_GET() == (short)31323);
            assert(pack.rssi_GET() == (char)77);
            assert(pack.chan1_scaled_GET() == (short) -10807);
            assert(pack.chan6_scaled_GET() == (short) -7819);
            assert(pack.chan7_scaled_GET() == (short) -16840);
            assert(pack.time_boot_ms_GET() == 1301600453L);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan4_scaled_SET((short)31323) ;
        p34.chan1_scaled_SET((short) -10807) ;
        p34.chan7_scaled_SET((short) -16840) ;
        p34.chan2_scaled_SET((short) -7157) ;
        p34.chan3_scaled_SET((short) -18094) ;
        p34.chan8_scaled_SET((short)23908) ;
        p34.port_SET((char)64) ;
        p34.chan5_scaled_SET((short)30210) ;
        p34.chan6_scaled_SET((short) -7819) ;
        p34.rssi_SET((char)77) ;
        p34.time_boot_ms_SET(1301600453L) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan8_raw_GET() == (char)45545);
            assert(pack.chan2_raw_GET() == (char)65208);
            assert(pack.chan3_raw_GET() == (char)31927);
            assert(pack.chan1_raw_GET() == (char)35201);
            assert(pack.port_GET() == (char)172);
            assert(pack.time_boot_ms_GET() == 3223892864L);
            assert(pack.chan4_raw_GET() == (char)52061);
            assert(pack.rssi_GET() == (char)227);
            assert(pack.chan6_raw_GET() == (char)8323);
            assert(pack.chan5_raw_GET() == (char)20418);
            assert(pack.chan7_raw_GET() == (char)39463);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan3_raw_SET((char)31927) ;
        p35.chan7_raw_SET((char)39463) ;
        p35.time_boot_ms_SET(3223892864L) ;
        p35.chan6_raw_SET((char)8323) ;
        p35.chan5_raw_SET((char)20418) ;
        p35.port_SET((char)172) ;
        p35.chan1_raw_SET((char)35201) ;
        p35.rssi_SET((char)227) ;
        p35.chan2_raw_SET((char)65208) ;
        p35.chan8_raw_SET((char)45545) ;
        p35.chan4_raw_SET((char)52061) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo9_raw_TRY(ph) == (char)36522);
            assert(pack.servo5_raw_GET() == (char)62867);
            assert(pack.servo14_raw_TRY(ph) == (char)11025);
            assert(pack.servo3_raw_GET() == (char)37762);
            assert(pack.time_usec_GET() == 1313535990L);
            assert(pack.servo10_raw_TRY(ph) == (char)30568);
            assert(pack.port_GET() == (char)16);
            assert(pack.servo12_raw_TRY(ph) == (char)39639);
            assert(pack.servo15_raw_TRY(ph) == (char)41626);
            assert(pack.servo6_raw_GET() == (char)40679);
            assert(pack.servo16_raw_TRY(ph) == (char)20805);
            assert(pack.servo11_raw_TRY(ph) == (char)7335);
            assert(pack.servo4_raw_GET() == (char)60001);
            assert(pack.servo8_raw_GET() == (char)32809);
            assert(pack.servo2_raw_GET() == (char)34971);
            assert(pack.servo1_raw_GET() == (char)50589);
            assert(pack.servo7_raw_GET() == (char)3259);
            assert(pack.servo13_raw_TRY(ph) == (char)58356);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo10_raw_SET((char)30568, PH) ;
        p36.servo12_raw_SET((char)39639, PH) ;
        p36.servo16_raw_SET((char)20805, PH) ;
        p36.servo7_raw_SET((char)3259) ;
        p36.servo15_raw_SET((char)41626, PH) ;
        p36.time_usec_SET(1313535990L) ;
        p36.servo14_raw_SET((char)11025, PH) ;
        p36.servo3_raw_SET((char)37762) ;
        p36.servo8_raw_SET((char)32809) ;
        p36.servo9_raw_SET((char)36522, PH) ;
        p36.port_SET((char)16) ;
        p36.servo1_raw_SET((char)50589) ;
        p36.servo13_raw_SET((char)58356, PH) ;
        p36.servo2_raw_SET((char)34971) ;
        p36.servo11_raw_SET((char)7335, PH) ;
        p36.servo4_raw_SET((char)60001) ;
        p36.servo6_raw_SET((char)40679) ;
        p36.servo5_raw_SET((char)62867) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)242);
            assert(pack.target_component_GET() == (char)139);
            assert(pack.start_index_GET() == (short) -11592);
            assert(pack.end_index_GET() == (short)19777);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_component_SET((char)139) ;
        p37.target_system_SET((char)242) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p37.start_index_SET((short) -11592) ;
        p37.end_index_SET((short)19777) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.end_index_GET() == (short)13374);
            assert(pack.target_system_GET() == (char)221);
            assert(pack.start_index_GET() == (short)21548);
            assert(pack.target_component_GET() == (char)213);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_component_SET((char)213) ;
        p38.end_index_SET((short)13374) ;
        p38.start_index_SET((short)21548) ;
        p38.target_system_SET((char)221) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)244);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.seq_GET() == (char)34768);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL);
            assert(pack.x_GET() == -2.7922325E38F);
            assert(pack.z_GET() == 2.824727E38F);
            assert(pack.param4_GET() == -1.8320265E38F);
            assert(pack.autocontinue_GET() == (char)220);
            assert(pack.target_system_GET() == (char)192);
            assert(pack.param1_GET() == -1.0081931E38F);
            assert(pack.y_GET() == -1.6884109E38F);
            assert(pack.current_GET() == (char)228);
            assert(pack.param2_GET() == -2.5162252E38F);
            assert(pack.param3_GET() == -1.6966688E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p39.seq_SET((char)34768) ;
        p39.param3_SET(-1.6966688E38F) ;
        p39.param4_SET(-1.8320265E38F) ;
        p39.command_SET(MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL) ;
        p39.param1_SET(-1.0081931E38F) ;
        p39.target_component_SET((char)244) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p39.autocontinue_SET((char)220) ;
        p39.y_SET(-1.6884109E38F) ;
        p39.z_SET(2.824727E38F) ;
        p39.param2_SET(-2.5162252E38F) ;
        p39.target_system_SET((char)192) ;
        p39.current_SET((char)228) ;
        p39.x_SET(-2.7922325E38F) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.seq_GET() == (char)48739);
            assert(pack.target_system_GET() == (char)39);
            assert(pack.target_component_GET() == (char)216);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p40.target_component_SET((char)216) ;
        p40.seq_SET((char)48739) ;
        p40.target_system_SET((char)39) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)202);
            assert(pack.target_system_GET() == (char)146);
            assert(pack.seq_GET() == (char)46028);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)146) ;
        p41.target_component_SET((char)202) ;
        p41.seq_SET((char)46028) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)11755);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)11755) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)217);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)40);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)40) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p43.target_component_SET((char)217) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.count_GET() == (char)3891);
            assert(pack.target_system_GET() == (char)66);
            assert(pack.target_component_GET() == (char)148);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.target_system_SET((char)66) ;
        p44.target_component_SET((char)148) ;
        p44.count_SET((char)3891) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)77);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)31);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p45.target_system_SET((char)31) ;
        p45.target_component_SET((char)77) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)10414);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)10414) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)85);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM5_X);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_component_GET() == (char)1);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.target_system_SET((char)85) ;
        p47.target_component_SET((char)1) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM5_X) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)233);
            assert(pack.latitude_GET() == -1115193681);
            assert(pack.time_usec_TRY(ph) == 2202238936819289959L);
            assert(pack.longitude_GET() == 1762943761);
            assert(pack.altitude_GET() == 433223434);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.time_usec_SET(2202238936819289959L, PH) ;
        p48.target_system_SET((char)233) ;
        p48.latitude_SET(-1115193681) ;
        p48.longitude_SET(1762943761) ;
        p48.altitude_SET(433223434) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == 31340009);
            assert(pack.latitude_GET() == -668739842);
            assert(pack.longitude_GET() == -2112314257);
            assert(pack.time_usec_TRY(ph) == 6366155506427542194L);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.time_usec_SET(6366155506427542194L, PH) ;
        p49.longitude_SET(-2112314257) ;
        p49.altitude_SET(31340009) ;
        p49.latitude_SET(-668739842) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)79);
            assert(pack.target_component_GET() == (char)222);
            assert(pack.param_value0_GET() == 2.3667686E38F);
            assert(pack.scale_GET() == -4.4965215E37F);
            assert(pack.param_value_min_GET() == -1.5581541E38F);
            assert(pack.parameter_rc_channel_index_GET() == (char)74);
            assert(pack.param_index_GET() == (short) -12971);
            assert(pack.param_id_LEN(ph) == 15);
            assert(pack.param_id_TRY(ph).equals("cecalzmcfqybyaz"));
            assert(pack.param_value_max_GET() == -2.044306E37F);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.param_value_min_SET(-1.5581541E38F) ;
        p50.param_value_max_SET(-2.044306E37F) ;
        p50.parameter_rc_channel_index_SET((char)74) ;
        p50.param_value0_SET(2.3667686E38F) ;
        p50.target_system_SET((char)79) ;
        p50.param_id_SET("cecalzmcfqybyaz", PH) ;
        p50.scale_SET(-4.4965215E37F) ;
        p50.param_index_SET((short) -12971) ;
        p50.target_component_SET((char)222) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)62);
            assert(pack.seq_GET() == (char)5476);
            assert(pack.target_system_GET() == (char)63);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_system_SET((char)63) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p51.target_component_SET((char)62) ;
        p51.seq_SET((char)5476) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1z_GET() == -2.5591047E36F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.target_component_GET() == (char)255);
            assert(pack.p2x_GET() == -1.3693613E38F);
            assert(pack.p2z_GET() == -1.0261999E38F);
            assert(pack.p1x_GET() == 6.0389703E37F);
            assert(pack.target_system_GET() == (char)47);
            assert(pack.p1y_GET() == -3.1940436E38F);
            assert(pack.p2y_GET() == 2.5656972E38F);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p1y_SET(-3.1940436E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p54.target_system_SET((char)47) ;
        p54.p2y_SET(2.5656972E38F) ;
        p54.p2z_SET(-1.0261999E38F) ;
        p54.p1z_SET(-2.5591047E36F) ;
        p54.p2x_SET(-1.3693613E38F) ;
        p54.p1x_SET(6.0389703E37F) ;
        p54.target_component_SET((char)255) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1x_GET() == -5.439289E37F);
            assert(pack.p1z_GET() == -2.2095395E38F);
            assert(pack.p2y_GET() == 1.6478593E38F);
            assert(pack.p2x_GET() == -3.842565E37F);
            assert(pack.p1y_GET() == -2.1390945E38F);
            assert(pack.p2z_GET() == 1.4200075E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p1x_SET(-5.439289E37F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p55.p1y_SET(-2.1390945E38F) ;
        p55.p2y_SET(1.6478593E38F) ;
        p55.p2x_SET(-3.842565E37F) ;
        p55.p1z_SET(-2.2095395E38F) ;
        p55.p2z_SET(1.4200075E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == -1.7878595E38F);
            assert(pack.time_usec_GET() == 431722678798479735L);
            assert(pack.pitchspeed_GET() == 1.0730896E37F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.9549714E37F, 7.6351584E37F, 9.301709E37F, -7.497397E37F, -2.4459902E38F, 2.7483146E38F, -6.0496394E37F, -2.9972347E38F, 1.3084424E38F}));
            assert(pack.yawspeed_GET() == 1.0927233E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.0468952E38F, 4.365603E37F, 2.8913175E38F, 1.9491663E38F}));
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.rollspeed_SET(-1.7878595E38F) ;
        p61.pitchspeed_SET(1.0730896E37F) ;
        p61.yawspeed_SET(1.0927233E38F) ;
        p61.time_usec_SET(431722678798479735L) ;
        p61.covariance_SET(new float[] {2.9549714E37F, 7.6351584E37F, 9.301709E37F, -7.497397E37F, -2.4459902E38F, 2.7483146E38F, -6.0496394E37F, -2.9972347E38F, 1.3084424E38F}, 0) ;
        p61.q_SET(new float[] {3.0468952E38F, 4.365603E37F, 2.8913175E38F, 1.9491663E38F}, 0) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.alt_error_GET() == 4.7767336E37F);
            assert(pack.nav_pitch_GET() == -3.102632E38F);
            assert(pack.nav_bearing_GET() == (short) -11394);
            assert(pack.nav_roll_GET() == -9.030526E37F);
            assert(pack.aspd_error_GET() == 2.1183496E38F);
            assert(pack.wp_dist_GET() == (char)12828);
            assert(pack.xtrack_error_GET() == -2.0441397E38F);
            assert(pack.target_bearing_GET() == (short)31810);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.nav_roll_SET(-9.030526E37F) ;
        p62.aspd_error_SET(2.1183496E38F) ;
        p62.target_bearing_SET((short)31810) ;
        p62.wp_dist_SET((char)12828) ;
        p62.nav_bearing_SET((short) -11394) ;
        p62.alt_error_SET(4.7767336E37F) ;
        p62.nav_pitch_SET(-3.102632E38F) ;
        p62.xtrack_error_SET(-2.0441397E38F) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.relative_alt_GET() == -994039337);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {7.0721167E37F, 2.6035197E38F, 2.0869178E36F, 2.3063495E38F, 2.0758647E38F, -7.716741E37F, 1.0197201E38F, 8.947486E37F, 1.969177E38F, -2.2692526E38F, 3.3195709E38F, -1.05323695E37F, 2.3425755E38F, -8.662503E37F, -1.5588655E38F, -2.3300337E38F, -2.2338478E38F, -1.1494774E38F, -2.1993805E38F, -3.2905938E38F, 2.2465523E38F, 3.2944448E38F, -2.0452784E38F, 2.9974446E38F, 3.3216715E38F, -2.5759279E38F, 1.0114982E38F, 1.3974603E38F, 2.5692763E38F, -7.080537E37F, -1.2176538E38F, 1.5310009E38F, 1.4822297E37F, -5.552494E37F, 3.3951115E38F, -1.780883E38F}));
            assert(pack.lon_GET() == -921631477);
            assert(pack.vz_GET() == 1.9364639E38F);
            assert(pack.lat_GET() == -1600261491);
            assert(pack.alt_GET() == 385757007);
            assert(pack.time_usec_GET() == 5863689806323446576L);
            assert(pack.vy_GET() == -2.6754236E38F);
            assert(pack.vx_GET() == 2.5286384E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.lat_SET(-1600261491) ;
        p63.time_usec_SET(5863689806323446576L) ;
        p63.alt_SET(385757007) ;
        p63.vx_SET(2.5286384E38F) ;
        p63.vz_SET(1.9364639E38F) ;
        p63.covariance_SET(new float[] {7.0721167E37F, 2.6035197E38F, 2.0869178E36F, 2.3063495E38F, 2.0758647E38F, -7.716741E37F, 1.0197201E38F, 8.947486E37F, 1.969177E38F, -2.2692526E38F, 3.3195709E38F, -1.05323695E37F, 2.3425755E38F, -8.662503E37F, -1.5588655E38F, -2.3300337E38F, -2.2338478E38F, -1.1494774E38F, -2.1993805E38F, -3.2905938E38F, 2.2465523E38F, 3.2944448E38F, -2.0452784E38F, 2.9974446E38F, 3.3216715E38F, -2.5759279E38F, 1.0114982E38F, 1.3974603E38F, 2.5692763E38F, -7.080537E37F, -1.2176538E38F, 1.5310009E38F, 1.4822297E37F, -5.552494E37F, 3.3951115E38F, -1.780883E38F}, 0) ;
        p63.lon_SET(-921631477) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS) ;
        p63.vy_SET(-2.6754236E38F) ;
        p63.relative_alt_SET(-994039337) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == 2.0481246E37F);
            assert(pack.vx_GET() == 2.4059879E38F);
            assert(pack.z_GET() == 8.4504545E37F);
            assert(pack.y_GET() == -2.8322833E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {1.5463198E38F, 7.2996565E37F, 3.222863E37F, 1.3994487E38F, 2.8407598E38F, -2.0689423E38F, 2.8686297E37F, -2.0944876E37F, 1.9791829E38F, -2.4749434E38F, -6.155908E37F, -2.8089967E38F, -2.1712723E38F, 1.9806876E37F, 3.115534E38F, 2.2408782E37F, 2.1267715E38F, 3.3042842E38F, 3.0902208E38F, -3.390314E38F, 2.7284354E38F, 2.174838E38F, -1.322012E38F, 8.26672E37F, -1.3794239E38F, -2.7194698E37F, 2.5766944E38F, -1.6680535E38F, -2.7499078E38F, -6.223144E37F, 6.7373586E37F, 1.7361479E38F, 2.7519922E38F, -5.5327777E37F, -1.018439E38F, -1.4993662E38F, 1.26529E38F, 3.1668784E38F, 2.8585462E38F, 1.8426758E38F, -1.5161777E38F, -1.837364E37F, -2.6888707E38F, -1.85192E38F, 1.1391887E38F}));
            assert(pack.ay_GET() == -1.182583E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
            assert(pack.ax_GET() == -3.2317527E38F);
            assert(pack.time_usec_GET() == 433997657554492993L);
            assert(pack.x_GET() == 2.3217345E38F);
            assert(pack.az_GET() == 3.3771328E38F);
            assert(pack.vy_GET() == -1.7100953E38F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.y_SET(-2.8322833E38F) ;
        p64.x_SET(2.3217345E38F) ;
        p64.vx_SET(2.4059879E38F) ;
        p64.ay_SET(-1.182583E38F) ;
        p64.vy_SET(-1.7100953E38F) ;
        p64.time_usec_SET(433997657554492993L) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS) ;
        p64.covariance_SET(new float[] {1.5463198E38F, 7.2996565E37F, 3.222863E37F, 1.3994487E38F, 2.8407598E38F, -2.0689423E38F, 2.8686297E37F, -2.0944876E37F, 1.9791829E38F, -2.4749434E38F, -6.155908E37F, -2.8089967E38F, -2.1712723E38F, 1.9806876E37F, 3.115534E38F, 2.2408782E37F, 2.1267715E38F, 3.3042842E38F, 3.0902208E38F, -3.390314E38F, 2.7284354E38F, 2.174838E38F, -1.322012E38F, 8.26672E37F, -1.3794239E38F, -2.7194698E37F, 2.5766944E38F, -1.6680535E38F, -2.7499078E38F, -6.223144E37F, 6.7373586E37F, 1.7361479E38F, 2.7519922E38F, -5.5327777E37F, -1.018439E38F, -1.4993662E38F, 1.26529E38F, 3.1668784E38F, 2.8585462E38F, 1.8426758E38F, -1.5161777E38F, -1.837364E37F, -2.6888707E38F, -1.85192E38F, 1.1391887E38F}, 0) ;
        p64.ax_SET(-3.2317527E38F) ;
        p64.az_SET(3.3771328E38F) ;
        p64.z_SET(8.4504545E37F) ;
        p64.vz_SET(2.0481246E37F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan16_raw_GET() == (char)30863);
            assert(pack.chan12_raw_GET() == (char)34051);
            assert(pack.chan11_raw_GET() == (char)23709);
            assert(pack.chan1_raw_GET() == (char)40010);
            assert(pack.chan14_raw_GET() == (char)42173);
            assert(pack.chan3_raw_GET() == (char)100);
            assert(pack.chan2_raw_GET() == (char)28604);
            assert(pack.chan9_raw_GET() == (char)2526);
            assert(pack.chan6_raw_GET() == (char)49743);
            assert(pack.rssi_GET() == (char)148);
            assert(pack.chan17_raw_GET() == (char)44455);
            assert(pack.time_boot_ms_GET() == 3253551290L);
            assert(pack.chancount_GET() == (char)22);
            assert(pack.chan15_raw_GET() == (char)57853);
            assert(pack.chan8_raw_GET() == (char)85);
            assert(pack.chan13_raw_GET() == (char)8309);
            assert(pack.chan10_raw_GET() == (char)32264);
            assert(pack.chan18_raw_GET() == (char)6147);
            assert(pack.chan7_raw_GET() == (char)46273);
            assert(pack.chan4_raw_GET() == (char)63976);
            assert(pack.chan5_raw_GET() == (char)33970);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan10_raw_SET((char)32264) ;
        p65.chan18_raw_SET((char)6147) ;
        p65.rssi_SET((char)148) ;
        p65.chan13_raw_SET((char)8309) ;
        p65.chan3_raw_SET((char)100) ;
        p65.chan9_raw_SET((char)2526) ;
        p65.chan5_raw_SET((char)33970) ;
        p65.chan8_raw_SET((char)85) ;
        p65.chan1_raw_SET((char)40010) ;
        p65.chan2_raw_SET((char)28604) ;
        p65.chancount_SET((char)22) ;
        p65.chan16_raw_SET((char)30863) ;
        p65.chan14_raw_SET((char)42173) ;
        p65.chan15_raw_SET((char)57853) ;
        p65.chan6_raw_SET((char)49743) ;
        p65.chan4_raw_SET((char)63976) ;
        p65.time_boot_ms_SET(3253551290L) ;
        p65.chan7_raw_SET((char)46273) ;
        p65.chan12_raw_SET((char)34051) ;
        p65.chan11_raw_SET((char)23709) ;
        p65.chan17_raw_SET((char)44455) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.start_stop_GET() == (char)4);
            assert(pack.target_component_GET() == (char)22);
            assert(pack.req_stream_id_GET() == (char)177);
            assert(pack.target_system_GET() == (char)218);
            assert(pack.req_message_rate_GET() == (char)31714);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.req_stream_id_SET((char)177) ;
        p66.target_component_SET((char)22) ;
        p66.start_stop_SET((char)4) ;
        p66.target_system_SET((char)218) ;
        p66.req_message_rate_SET((char)31714) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.stream_id_GET() == (char)143);
            assert(pack.message_rate_GET() == (char)30029);
            assert(pack.on_off_GET() == (char)38);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.message_rate_SET((char)30029) ;
        p67.on_off_SET((char)38) ;
        p67.stream_id_SET((char)143) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == (short)6654);
            assert(pack.buttons_GET() == (char)62215);
            assert(pack.z_GET() == (short) -12650);
            assert(pack.x_GET() == (short)9085);
            assert(pack.target_GET() == (char)186);
            assert(pack.r_GET() == (short) -18870);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.r_SET((short) -18870) ;
        p69.z_SET((short) -12650) ;
        p69.y_SET((short)6654) ;
        p69.target_SET((char)186) ;
        p69.x_SET((short)9085) ;
        p69.buttons_SET((char)62215) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)82);
            assert(pack.target_component_GET() == (char)85);
            assert(pack.chan8_raw_GET() == (char)2350);
            assert(pack.chan4_raw_GET() == (char)3508);
            assert(pack.chan3_raw_GET() == (char)56264);
            assert(pack.chan7_raw_GET() == (char)12293);
            assert(pack.chan5_raw_GET() == (char)9614);
            assert(pack.chan6_raw_GET() == (char)4637);
            assert(pack.chan2_raw_GET() == (char)44177);
            assert(pack.chan1_raw_GET() == (char)273);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan3_raw_SET((char)56264) ;
        p70.target_component_SET((char)85) ;
        p70.chan8_raw_SET((char)2350) ;
        p70.target_system_SET((char)82) ;
        p70.chan1_raw_SET((char)273) ;
        p70.chan5_raw_SET((char)9614) ;
        p70.chan2_raw_SET((char)44177) ;
        p70.chan4_raw_SET((char)3508) ;
        p70.chan7_raw_SET((char)12293) ;
        p70.chan6_raw_SET((char)4637) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_SPATIAL_USER_4);
            assert(pack.param1_GET() == 1.8567352E37F);
            assert(pack.target_component_GET() == (char)237);
            assert(pack.param4_GET() == -2.7722774E38F);
            assert(pack.x_GET() == -493909421);
            assert(pack.target_system_GET() == (char)135);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.current_GET() == (char)143);
            assert(pack.z_GET() == -1.2851181E38F);
            assert(pack.seq_GET() == (char)10852);
            assert(pack.param2_GET() == -1.1010821E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.autocontinue_GET() == (char)127);
            assert(pack.param3_GET() == -2.6714491E38F);
            assert(pack.y_GET() == -1122392536);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.z_SET(-1.2851181E38F) ;
        p73.param1_SET(1.8567352E37F) ;
        p73.command_SET(MAV_CMD.MAV_CMD_SPATIAL_USER_4) ;
        p73.autocontinue_SET((char)127) ;
        p73.param3_SET(-2.6714491E38F) ;
        p73.current_SET((char)143) ;
        p73.target_system_SET((char)135) ;
        p73.target_component_SET((char)237) ;
        p73.x_SET(-493909421) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p73.y_SET(-1122392536) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p73.seq_SET((char)10852) ;
        p73.param4_SET(-2.7722774E38F) ;
        p73.param2_SET(-1.1010821E38F) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.groundspeed_GET() == -1.2344763E38F);
            assert(pack.throttle_GET() == (char)43482);
            assert(pack.climb_GET() == 2.6489003E38F);
            assert(pack.heading_GET() == (short) -26782);
            assert(pack.alt_GET() == 2.3427595E38F);
            assert(pack.airspeed_GET() == -1.350814E38F);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.airspeed_SET(-1.350814E38F) ;
        p74.groundspeed_SET(-1.2344763E38F) ;
        p74.throttle_SET((char)43482) ;
        p74.alt_SET(2.3427595E38F) ;
        p74.climb_SET(2.6489003E38F) ;
        p74.heading_SET((short) -26782) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 3.1697895E38F);
            assert(pack.param4_GET() == 2.592457E38F);
            assert(pack.current_GET() == (char)1);
            assert(pack.param3_GET() == -4.669501E37F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION);
            assert(pack.autocontinue_GET() == (char)148);
            assert(pack.y_GET() == -601276527);
            assert(pack.param2_GET() == 2.7053662E38F);
            assert(pack.target_component_GET() == (char)12);
            assert(pack.param1_GET() == -1.2469881E38F);
            assert(pack.x_GET() == 894120871);
            assert(pack.target_system_GET() == (char)106);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p75.param2_SET(2.7053662E38F) ;
        p75.autocontinue_SET((char)148) ;
        p75.command_SET(MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION) ;
        p75.param1_SET(-1.2469881E38F) ;
        p75.param3_SET(-4.669501E37F) ;
        p75.param4_SET(2.592457E38F) ;
        p75.x_SET(894120871) ;
        p75.y_SET(-601276527) ;
        p75.current_SET((char)1) ;
        p75.z_SET(3.1697895E38F) ;
        p75.target_component_SET((char)12) ;
        p75.target_system_SET((char)106) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN);
            assert(pack.param1_GET() == 1.4753687E38F);
            assert(pack.target_system_GET() == (char)48);
            assert(pack.param4_GET() == -2.070145E38F);
            assert(pack.param7_GET() == 1.0728671E38F);
            assert(pack.param5_GET() == 8.673353E37F);
            assert(pack.param2_GET() == 1.7927547E38F);
            assert(pack.confirmation_GET() == (char)58);
            assert(pack.param3_GET() == 2.6779754E38F);
            assert(pack.param6_GET() == 1.7528752E38F);
            assert(pack.target_component_GET() == (char)215);
        });
        COMMAND_LONG p76 = new COMMAND_LONG();
        PH.setPack(p76);
        p76.target_system_SET((char)48) ;
        p76.param4_SET(-2.070145E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN) ;
        p76.param6_SET(1.7528752E38F) ;
        p76.param1_SET(1.4753687E38F) ;
        p76.param3_SET(2.6779754E38F) ;
        p76.param7_SET(1.0728671E38F) ;
        p76.confirmation_SET((char)58) ;
        p76.target_component_SET((char)215) ;
        p76.param2_SET(1.7927547E38F) ;
        p76.param5_SET(8.673353E37F) ;
        TestChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.progress_TRY(ph) == (char)241);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_DENIED);
            assert(pack.target_component_TRY(ph) == (char)48);
            assert(pack.target_system_TRY(ph) == (char)31);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST);
            assert(pack.result_param2_TRY(ph) == -1677959899);
        });
        COMMAND_ACK p77 = new COMMAND_ACK();
        PH.setPack(p77);
        p77.result_param2_SET(-1677959899, PH) ;
        p77.target_system_SET((char)31, PH) ;
        p77.progress_SET((char)241, PH) ;
        p77.target_component_SET((char)48, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_DENIED) ;
        TestChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3972717262L);
            assert(pack.thrust_GET() == 1.6340604E38F);
            assert(pack.roll_GET() == -2.1475497E37F);
            assert(pack.mode_switch_GET() == (char)122);
            assert(pack.pitch_GET() == 2.6090673E37F);
            assert(pack.manual_override_switch_GET() == (char)142);
            assert(pack.yaw_GET() == -1.3167866E38F);
        });
        MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.mode_switch_SET((char)122) ;
        p81.time_boot_ms_SET(3972717262L) ;
        p81.manual_override_switch_SET((char)142) ;
        p81.pitch_SET(2.6090673E37F) ;
        p81.thrust_SET(1.6340604E38F) ;
        p81.roll_SET(-2.1475497E37F) ;
        p81.yaw_SET(-1.3167866E38F) ;
        TestChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)58);
            assert(pack.body_yaw_rate_GET() == -1.5593518E37F);
            assert(pack.type_mask_GET() == (char)129);
            assert(pack.body_roll_rate_GET() == -1.453225E37F);
            assert(pack.thrust_GET() == -3.1939307E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.2670092E38F, 9.34463E37F, 2.6547177E38F, 1.9271451E38F}));
            assert(pack.body_pitch_rate_GET() == 2.629374E38F);
            assert(pack.time_boot_ms_GET() == 1527140951L);
            assert(pack.target_system_GET() == (char)151);
        });
        SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.type_mask_SET((char)129) ;
        p82.time_boot_ms_SET(1527140951L) ;
        p82.body_roll_rate_SET(-1.453225E37F) ;
        p82.target_system_SET((char)151) ;
        p82.q_SET(new float[] {3.2670092E38F, 9.34463E37F, 2.6547177E38F, 1.9271451E38F}, 0) ;
        p82.body_yaw_rate_SET(-1.5593518E37F) ;
        p82.thrust_SET(-3.1939307E38F) ;
        p82.body_pitch_rate_SET(2.629374E38F) ;
        p82.target_component_SET((char)58) ;
        TestChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_yaw_rate_GET() == 2.0521101E38F);
            assert(pack.body_pitch_rate_GET() == -2.7372899E38F);
            assert(pack.body_roll_rate_GET() == 1.692337E36F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.855775E38F, -1.5636107E38F, 7.329515E37F, 3.0433352E38F}));
            assert(pack.type_mask_GET() == (char)252);
            assert(pack.time_boot_ms_GET() == 3432804749L);
            assert(pack.thrust_GET() == 2.153259E38F);
        });
        ATTITUDE_TARGET p83 = new ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.body_roll_rate_SET(1.692337E36F) ;
        p83.time_boot_ms_SET(3432804749L) ;
        p83.body_pitch_rate_SET(-2.7372899E38F) ;
        p83.body_yaw_rate_SET(2.0521101E38F) ;
        p83.type_mask_SET((char)252) ;
        p83.q_SET(new float[] {1.855775E38F, -1.5636107E38F, 7.329515E37F, 3.0433352E38F}, 0) ;
        p83.thrust_SET(2.153259E38F) ;
        TestChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)62720);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.time_boot_ms_GET() == 3012940370L);
            assert(pack.y_GET() == 2.5658615E38F);
            assert(pack.z_GET() == -1.354467E38F);
            assert(pack.vy_GET() == -2.5170564E38F);
            assert(pack.yaw_GET() == 1.4446049E38F);
            assert(pack.afy_GET() == 3.1981987E38F);
            assert(pack.vz_GET() == -2.8460746E38F);
            assert(pack.yaw_rate_GET() == -1.06407645E36F);
            assert(pack.x_GET() == 3.991703E37F);
            assert(pack.target_system_GET() == (char)43);
            assert(pack.afz_GET() == -2.298037E38F);
            assert(pack.target_component_GET() == (char)198);
            assert(pack.vx_GET() == -2.8647756E38F);
            assert(pack.afx_GET() == -7.736307E37F);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.time_boot_ms_SET(3012940370L) ;
        p84.vx_SET(-2.8647756E38F) ;
        p84.target_component_SET((char)198) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p84.yaw_SET(1.4446049E38F) ;
        p84.vy_SET(-2.5170564E38F) ;
        p84.afx_SET(-7.736307E37F) ;
        p84.x_SET(3.991703E37F) ;
        p84.vz_SET(-2.8460746E38F) ;
        p84.afy_SET(3.1981987E38F) ;
        p84.y_SET(2.5658615E38F) ;
        p84.target_system_SET((char)43) ;
        p84.yaw_rate_SET(-1.06407645E36F) ;
        p84.type_mask_SET((char)62720) ;
        p84.afz_SET(-2.298037E38F) ;
        p84.z_SET(-1.354467E38F) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == 3.1250376E38F);
            assert(pack.time_boot_ms_GET() == 2901837005L);
            assert(pack.vz_GET() == 1.0397202E38F);
            assert(pack.lat_int_GET() == -1846712974);
            assert(pack.type_mask_GET() == (char)52065);
            assert(pack.afy_GET() == -5.606777E37F);
            assert(pack.afx_GET() == 9.667763E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.yaw_GET() == 1.8333708E38F);
            assert(pack.alt_GET() == -5.651661E37F);
            assert(pack.vy_GET() == 8.696831E37F);
            assert(pack.afz_GET() == 2.2798682E38F);
            assert(pack.target_system_GET() == (char)175);
            assert(pack.target_component_GET() == (char)97);
            assert(pack.lon_int_GET() == -1036859581);
            assert(pack.yaw_rate_GET() == 2.8147794E37F);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.afx_SET(9.667763E37F) ;
        p86.lat_int_SET(-1846712974) ;
        p86.vx_SET(3.1250376E38F) ;
        p86.target_component_SET((char)97) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p86.target_system_SET((char)175) ;
        p86.vz_SET(1.0397202E38F) ;
        p86.afz_SET(2.2798682E38F) ;
        p86.lon_int_SET(-1036859581) ;
        p86.alt_SET(-5.651661E37F) ;
        p86.time_boot_ms_SET(2901837005L) ;
        p86.afy_SET(-5.606777E37F) ;
        p86.yaw_SET(1.8333708E38F) ;
        p86.vy_SET(8.696831E37F) ;
        p86.yaw_rate_SET(2.8147794E37F) ;
        p86.type_mask_SET((char)52065) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == 1.5978572E38F);
            assert(pack.yaw_GET() == -3.844195E37F);
            assert(pack.lon_int_GET() == 550717138);
            assert(pack.alt_GET() == -1.2096594E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.time_boot_ms_GET() == 743635722L);
            assert(pack.afy_GET() == -2.725686E38F);
            assert(pack.afx_GET() == -1.3772107E38F);
            assert(pack.vx_GET() == -1.1937399E38F);
            assert(pack.afz_GET() == 2.7977937E38F);
            assert(pack.lat_int_GET() == -2078446199);
            assert(pack.vz_GET() == -3.3870922E38F);
            assert(pack.type_mask_GET() == (char)36259);
            assert(pack.vy_GET() == 2.220832E38F);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.vy_SET(2.220832E38F) ;
        p87.afy_SET(-2.725686E38F) ;
        p87.vx_SET(-1.1937399E38F) ;
        p87.time_boot_ms_SET(743635722L) ;
        p87.type_mask_SET((char)36259) ;
        p87.lat_int_SET(-2078446199) ;
        p87.afx_SET(-1.3772107E38F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p87.alt_SET(-1.2096594E38F) ;
        p87.yaw_rate_SET(1.5978572E38F) ;
        p87.vz_SET(-3.3870922E38F) ;
        p87.afz_SET(2.7977937E38F) ;
        p87.yaw_SET(-3.844195E37F) ;
        p87.lon_int_SET(550717138) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -5.336639E37F);
            assert(pack.z_GET() == -2.892132E38F);
            assert(pack.pitch_GET() == 1.1194384E38F);
            assert(pack.yaw_GET() == -3.3305818E38F);
            assert(pack.x_GET() == -1.7155505E38F);
            assert(pack.time_boot_ms_GET() == 2111439687L);
            assert(pack.roll_GET() == -2.1896978E36F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.roll_SET(-2.1896978E36F) ;
        p89.z_SET(-2.892132E38F) ;
        p89.y_SET(-5.336639E37F) ;
        p89.x_SET(-1.7155505E38F) ;
        p89.time_boot_ms_SET(2111439687L) ;
        p89.pitch_SET(1.1194384E38F) ;
        p89.yaw_SET(-3.3305818E38F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == (short) -17034);
            assert(pack.time_usec_GET() == 1127388695575099037L);
            assert(pack.pitchspeed_GET() == 2.4949564E38F);
            assert(pack.zacc_GET() == (short) -32671);
            assert(pack.yacc_GET() == (short) -23140);
            assert(pack.lat_GET() == -1863887582);
            assert(pack.yawspeed_GET() == -1.7435867E38F);
            assert(pack.pitch_GET() == 1.4842236E38F);
            assert(pack.roll_GET() == 3.3706225E38F);
            assert(pack.vy_GET() == (short) -19945);
            assert(pack.xacc_GET() == (short)29835);
            assert(pack.lon_GET() == 2021366660);
            assert(pack.vx_GET() == (short)28512);
            assert(pack.alt_GET() == -863640580);
            assert(pack.rollspeed_GET() == 2.8605284E37F);
            assert(pack.yaw_GET() == 2.9918544E38F);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.zacc_SET((short) -32671) ;
        p90.pitchspeed_SET(2.4949564E38F) ;
        p90.yawspeed_SET(-1.7435867E38F) ;
        p90.yacc_SET((short) -23140) ;
        p90.time_usec_SET(1127388695575099037L) ;
        p90.lon_SET(2021366660) ;
        p90.alt_SET(-863640580) ;
        p90.vz_SET((short) -17034) ;
        p90.lat_SET(-1863887582) ;
        p90.xacc_SET((short)29835) ;
        p90.pitch_SET(1.4842236E38F) ;
        p90.vx_SET((short)28512) ;
        p90.vy_SET((short) -19945) ;
        p90.rollspeed_SET(2.8605284E37F) ;
        p90.roll_SET(3.3706225E38F) ;
        p90.yaw_SET(2.9918544E38F) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.aux3_GET() == -2.9046327E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
            assert(pack.pitch_elevator_GET() == 7.7071706E37F);
            assert(pack.aux1_GET() == 4.933058E37F);
            assert(pack.roll_ailerons_GET() == -2.164096E38F);
            assert(pack.aux2_GET() == 9.877843E37F);
            assert(pack.time_usec_GET() == 1057224189518607635L);
            assert(pack.throttle_GET() == 1.1180218E38F);
            assert(pack.yaw_rudder_GET() == -1.1421935E38F);
            assert(pack.aux4_GET() == -3.3436796E38F);
            assert(pack.nav_mode_GET() == (char)200);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.throttle_SET(1.1180218E38F) ;
        p91.yaw_rudder_SET(-1.1421935E38F) ;
        p91.aux4_SET(-3.3436796E38F) ;
        p91.pitch_elevator_SET(7.7071706E37F) ;
        p91.time_usec_SET(1057224189518607635L) ;
        p91.aux3_SET(-2.9046327E38F) ;
        p91.roll_ailerons_SET(-2.164096E38F) ;
        p91.aux2_SET(9.877843E37F) ;
        p91.aux1_SET(4.933058E37F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_GUIDED_DISARMED) ;
        p91.nav_mode_SET((char)200) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan2_raw_GET() == (char)47946);
            assert(pack.rssi_GET() == (char)246);
            assert(pack.chan4_raw_GET() == (char)28139);
            assert(pack.chan3_raw_GET() == (char)53640);
            assert(pack.chan1_raw_GET() == (char)8680);
            assert(pack.chan12_raw_GET() == (char)42894);
            assert(pack.chan11_raw_GET() == (char)63781);
            assert(pack.time_usec_GET() == 7105260604177013680L);
            assert(pack.chan6_raw_GET() == (char)29549);
            assert(pack.chan8_raw_GET() == (char)33235);
            assert(pack.chan5_raw_GET() == (char)16593);
            assert(pack.chan7_raw_GET() == (char)27964);
            assert(pack.chan9_raw_GET() == (char)52725);
            assert(pack.chan10_raw_GET() == (char)20424);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan9_raw_SET((char)52725) ;
        p92.chan8_raw_SET((char)33235) ;
        p92.chan1_raw_SET((char)8680) ;
        p92.chan6_raw_SET((char)29549) ;
        p92.chan4_raw_SET((char)28139) ;
        p92.chan12_raw_SET((char)42894) ;
        p92.chan7_raw_SET((char)27964) ;
        p92.chan2_raw_SET((char)47946) ;
        p92.chan3_raw_SET((char)53640) ;
        p92.rssi_SET((char)246) ;
        p92.chan10_raw_SET((char)20424) ;
        p92.time_usec_SET(7105260604177013680L) ;
        p92.chan11_raw_SET((char)63781) ;
        p92.chan5_raw_SET((char)16593) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.7653227E38F, -1.1049681E38F, 1.7075714E38F, 2.1263813E38F, 3.277123E38F, -9.392683E37F, 1.5081795E38F, 6.4545224E37F, 2.859473E38F, 4.821909E37F, -1.716214E38F, -2.755475E36F, -4.327317E37F, -3.2933445E38F, -1.7941765E38F, -1.6494837E38F}));
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_MANUAL_ARMED);
            assert(pack.flags_GET() == 4224233026816321585L);
            assert(pack.time_usec_GET() == 444437634732809311L);
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.flags_SET(4224233026816321585L) ;
        p93.time_usec_SET(444437634732809311L) ;
        p93.controls_SET(new float[] {1.7653227E38F, -1.1049681E38F, 1.7075714E38F, 2.1263813E38F, 3.277123E38F, -9.392683E37F, 1.5081795E38F, 6.4545224E37F, 2.859473E38F, 4.821909E37F, -1.716214E38F, -2.755475E36F, -4.327317E37F, -3.2933445E38F, -1.7941765E38F, -1.6494837E38F}, 0) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_MANUAL_ARMED) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_y_GET() == (short)2502);
            assert(pack.flow_comp_m_x_GET() == -3.2073061E38F);
            assert(pack.flow_rate_y_TRY(ph) == -2.3451185E38F);
            assert(pack.flow_comp_m_y_GET() == -2.7997443E38F);
            assert(pack.sensor_id_GET() == (char)17);
            assert(pack.flow_rate_x_TRY(ph) == 3.164824E38F);
            assert(pack.time_usec_GET() == 8423674921357214717L);
            assert(pack.quality_GET() == (char)10);
            assert(pack.flow_x_GET() == (short) -8392);
            assert(pack.ground_distance_GET() == -2.976553E38F);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.flow_y_SET((short)2502) ;
        p100.time_usec_SET(8423674921357214717L) ;
        p100.flow_rate_y_SET(-2.3451185E38F, PH) ;
        p100.ground_distance_SET(-2.976553E38F) ;
        p100.quality_SET((char)10) ;
        p100.flow_rate_x_SET(3.164824E38F, PH) ;
        p100.flow_comp_m_y_SET(-2.7997443E38F) ;
        p100.flow_x_SET((short) -8392) ;
        p100.flow_comp_m_x_SET(-3.2073061E38F) ;
        p100.sensor_id_SET((char)17) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -3.3609143E38F);
            assert(pack.pitch_GET() == 2.539235E38F);
            assert(pack.z_GET() == 3.2702777E38F);
            assert(pack.yaw_GET() == -1.1498223E38F);
            assert(pack.usec_GET() == 1934764743867036864L);
            assert(pack.roll_GET() == -1.9279558E38F);
            assert(pack.x_GET() == -2.8729896E37F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.roll_SET(-1.9279558E38F) ;
        p101.pitch_SET(2.539235E38F) ;
        p101.yaw_SET(-1.1498223E38F) ;
        p101.z_SET(3.2702777E38F) ;
        p101.x_SET(-2.8729896E37F) ;
        p101.y_SET(-3.3609143E38F) ;
        p101.usec_SET(1934764743867036864L) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -1.1342042E38F);
            assert(pack.roll_GET() == -2.9748942E38F);
            assert(pack.usec_GET() == 4300214872486558082L);
            assert(pack.yaw_GET() == -5.357867E37F);
            assert(pack.x_GET() == 2.315712E38F);
            assert(pack.pitch_GET() == -2.7053345E38F);
            assert(pack.z_GET() == -1.4443498E38F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.pitch_SET(-2.7053345E38F) ;
        p102.y_SET(-1.1342042E38F) ;
        p102.x_SET(2.315712E38F) ;
        p102.yaw_SET(-5.357867E37F) ;
        p102.roll_SET(-2.9748942E38F) ;
        p102.z_SET(-1.4443498E38F) ;
        p102.usec_SET(4300214872486558082L) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 8719022496135008390L);
            assert(pack.z_GET() == -5.659312E37F);
            assert(pack.y_GET() == -2.0569278E38F);
            assert(pack.x_GET() == 2.0863807E38F);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.x_SET(2.0863807E38F) ;
        p103.y_SET(-2.0569278E38F) ;
        p103.usec_SET(8719022496135008390L) ;
        p103.z_SET(-5.659312E37F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 3.075865E38F);
            assert(pack.y_GET() == 3.8175016E37F);
            assert(pack.z_GET() == 7.445218E37F);
            assert(pack.roll_GET() == 5.389306E37F);
            assert(pack.usec_GET() == 908882795200282885L);
            assert(pack.x_GET() == -3.2070775E38F);
            assert(pack.pitch_GET() == -5.667857E36F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.pitch_SET(-5.667857E36F) ;
        p104.usec_SET(908882795200282885L) ;
        p104.y_SET(3.8175016E37F) ;
        p104.z_SET(7.445218E37F) ;
        p104.yaw_SET(3.075865E38F) ;
        p104.x_SET(-3.2070775E38F) ;
        p104.roll_SET(5.389306E37F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.xgyro_GET() == -1.1695734E38F);
            assert(pack.zacc_GET() == 3.8886468E37F);
            assert(pack.xacc_GET() == -4.2393646E37F);
            assert(pack.fields_updated_GET() == (char)43240);
            assert(pack.zmag_GET() == -2.594625E38F);
            assert(pack.ymag_GET() == -3.5143184E37F);
            assert(pack.diff_pressure_GET() == -7.370057E37F);
            assert(pack.pressure_alt_GET() == 8.3823E37F);
            assert(pack.ygyro_GET() == 2.57082E38F);
            assert(pack.time_usec_GET() == 7027031387313666280L);
            assert(pack.xmag_GET() == 1.7593972E38F);
            assert(pack.zgyro_GET() == -2.6983148E38F);
            assert(pack.temperature_GET() == 1.6346399E38F);
            assert(pack.yacc_GET() == -1.065945E38F);
            assert(pack.abs_pressure_GET() == 3.0623794E38F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.xmag_SET(1.7593972E38F) ;
        p105.diff_pressure_SET(-7.370057E37F) ;
        p105.zgyro_SET(-2.6983148E38F) ;
        p105.temperature_SET(1.6346399E38F) ;
        p105.ymag_SET(-3.5143184E37F) ;
        p105.xgyro_SET(-1.1695734E38F) ;
        p105.fields_updated_SET((char)43240) ;
        p105.pressure_alt_SET(8.3823E37F) ;
        p105.zmag_SET(-2.594625E38F) ;
        p105.abs_pressure_SET(3.0623794E38F) ;
        p105.ygyro_SET(2.57082E38F) ;
        p105.zacc_SET(3.8886468E37F) ;
        p105.yacc_SET(-1.065945E38F) ;
        p105.time_usec_SET(7027031387313666280L) ;
        p105.xacc_SET(-4.2393646E37F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integrated_xgyro_GET() == 1.882103E38F);
            assert(pack.time_delta_distance_us_GET() == 2024392077L);
            assert(pack.temperature_GET() == (short)13884);
            assert(pack.distance_GET() == -1.8467237E38F);
            assert(pack.integrated_ygyro_GET() == -2.5038687E38F);
            assert(pack.integrated_zgyro_GET() == -2.703555E38F);
            assert(pack.quality_GET() == (char)12);
            assert(pack.integrated_y_GET() == -1.4114445E38F);
            assert(pack.time_usec_GET() == 3857192331589462695L);
            assert(pack.integrated_x_GET() == 2.5856924E38F);
            assert(pack.integration_time_us_GET() == 1882453246L);
            assert(pack.sensor_id_GET() == (char)151);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integrated_zgyro_SET(-2.703555E38F) ;
        p106.distance_SET(-1.8467237E38F) ;
        p106.integrated_xgyro_SET(1.882103E38F) ;
        p106.integrated_ygyro_SET(-2.5038687E38F) ;
        p106.sensor_id_SET((char)151) ;
        p106.integrated_y_SET(-1.4114445E38F) ;
        p106.temperature_SET((short)13884) ;
        p106.time_usec_SET(3857192331589462695L) ;
        p106.integration_time_us_SET(1882453246L) ;
        p106.time_delta_distance_us_SET(2024392077L) ;
        p106.quality_SET((char)12) ;
        p106.integrated_x_SET(2.5856924E38F) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == -1.0291787E37F);
            assert(pack.temperature_GET() == 4.2164146E37F);
            assert(pack.zgyro_GET() == 1.7294711E38F);
            assert(pack.pressure_alt_GET() == -1.593299E38F);
            assert(pack.abs_pressure_GET() == -2.7469341E38F);
            assert(pack.time_usec_GET() == 4090779867394040745L);
            assert(pack.ygyro_GET() == 2.1709E38F);
            assert(pack.diff_pressure_GET() == -1.8762456E38F);
            assert(pack.fields_updated_GET() == 3679790489L);
            assert(pack.xacc_GET() == -2.0243478E38F);
            assert(pack.zacc_GET() == 1.3747845E38F);
            assert(pack.xgyro_GET() == -2.9654787E38F);
            assert(pack.ymag_GET() == -1.1896662E38F);
            assert(pack.xmag_GET() == -5.183363E37F);
            assert(pack.zmag_GET() == -2.1584047E38F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.ymag_SET(-1.1896662E38F) ;
        p107.ygyro_SET(2.1709E38F) ;
        p107.xgyro_SET(-2.9654787E38F) ;
        p107.zmag_SET(-2.1584047E38F) ;
        p107.pressure_alt_SET(-1.593299E38F) ;
        p107.xacc_SET(-2.0243478E38F) ;
        p107.temperature_SET(4.2164146E37F) ;
        p107.fields_updated_SET(3679790489L) ;
        p107.diff_pressure_SET(-1.8762456E38F) ;
        p107.yacc_SET(-1.0291787E37F) ;
        p107.abs_pressure_SET(-2.7469341E38F) ;
        p107.zgyro_SET(1.7294711E38F) ;
        p107.zacc_SET(1.3747845E38F) ;
        p107.xmag_SET(-5.183363E37F) ;
        p107.time_usec_SET(4090779867394040745L) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 2.370668E38F);
            assert(pack.lat_GET() == 3.0225555E38F);
            assert(pack.vd_GET() == -1.0875628E38F);
            assert(pack.ve_GET() == 1.6641912E38F);
            assert(pack.alt_GET() == 6.2875896E37F);
            assert(pack.lon_GET() == 2.589214E38F);
            assert(pack.xacc_GET() == -3.1878571E38F);
            assert(pack.q1_GET() == 3.083165E38F);
            assert(pack.pitch_GET() == -2.104936E38F);
            assert(pack.std_dev_horz_GET() == 9.649806E37F);
            assert(pack.vn_GET() == 3.3802297E38F);
            assert(pack.xgyro_GET() == -2.1183407E38F);
            assert(pack.std_dev_vert_GET() == -9.121276E37F);
            assert(pack.q4_GET() == 7.598107E37F);
            assert(pack.zacc_GET() == -9.208303E37F);
            assert(pack.zgyro_GET() == -2.2389089E38F);
            assert(pack.q3_GET() == -3.2958751E38F);
            assert(pack.roll_GET() == 2.1552778E38F);
            assert(pack.yacc_GET() == 5.710011E37F);
            assert(pack.ygyro_GET() == 2.9427022E38F);
            assert(pack.q2_GET() == 1.1788708E38F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.xgyro_SET(-2.1183407E38F) ;
        p108.vn_SET(3.3802297E38F) ;
        p108.lon_SET(2.589214E38F) ;
        p108.vd_SET(-1.0875628E38F) ;
        p108.zacc_SET(-9.208303E37F) ;
        p108.std_dev_vert_SET(-9.121276E37F) ;
        p108.q2_SET(1.1788708E38F) ;
        p108.lat_SET(3.0225555E38F) ;
        p108.zgyro_SET(-2.2389089E38F) ;
        p108.pitch_SET(-2.104936E38F) ;
        p108.yaw_SET(2.370668E38F) ;
        p108.ve_SET(1.6641912E38F) ;
        p108.q4_SET(7.598107E37F) ;
        p108.ygyro_SET(2.9427022E38F) ;
        p108.xacc_SET(-3.1878571E38F) ;
        p108.std_dev_horz_SET(9.649806E37F) ;
        p108.q1_SET(3.083165E38F) ;
        p108.q3_SET(-3.2958751E38F) ;
        p108.roll_SET(2.1552778E38F) ;
        p108.yacc_SET(5.710011E37F) ;
        p108.alt_SET(6.2875896E37F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.fixed__GET() == (char)54011);
            assert(pack.rssi_GET() == (char)21);
            assert(pack.remnoise_GET() == (char)120);
            assert(pack.rxerrors_GET() == (char)24216);
            assert(pack.txbuf_GET() == (char)94);
            assert(pack.remrssi_GET() == (char)116);
            assert(pack.noise_GET() == (char)100);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.remrssi_SET((char)116) ;
        p109.txbuf_SET((char)94) ;
        p109.rssi_SET((char)21) ;
        p109.remnoise_SET((char)120) ;
        p109.rxerrors_SET((char)24216) ;
        p109.fixed__SET((char)54011) ;
        p109.noise_SET((char)100) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)254, (char)61, (char)175, (char)251, (char)137, (char)203, (char)208, (char)23, (char)199, (char)165, (char)236, (char)101, (char)77, (char)55, (char)7, (char)188, (char)144, (char)192, (char)15, (char)86, (char)174, (char)70, (char)192, (char)0, (char)64, (char)38, (char)26, (char)54, (char)166, (char)106, (char)114, (char)107, (char)228, (char)226, (char)168, (char)77, (char)37, (char)240, (char)98, (char)67, (char)50, (char)8, (char)241, (char)254, (char)223, (char)102, (char)143, (char)37, (char)86, (char)255, (char)147, (char)251, (char)32, (char)177, (char)62, (char)191, (char)7, (char)211, (char)128, (char)44, (char)247, (char)251, (char)113, (char)168, (char)165, (char)22, (char)110, (char)8, (char)186, (char)230, (char)199, (char)32, (char)195, (char)38, (char)208, (char)176, (char)84, (char)211, (char)97, (char)116, (char)194, (char)153, (char)94, (char)122, (char)37, (char)80, (char)180, (char)140, (char)128, (char)62, (char)149, (char)195, (char)116, (char)202, (char)233, (char)70, (char)195, (char)247, (char)191, (char)204, (char)189, (char)152, (char)120, (char)48, (char)27, (char)255, (char)161, (char)68, (char)75, (char)222, (char)163, (char)187, (char)201, (char)148, (char)117, (char)101, (char)209, (char)239, (char)237, (char)86, (char)41, (char)156, (char)220, (char)10, (char)207, (char)241, (char)169, (char)114, (char)129, (char)58, (char)230, (char)166, (char)75, (char)154, (char)29, (char)140, (char)76, (char)80, (char)67, (char)249, (char)240, (char)180, (char)94, (char)195, (char)206, (char)91, (char)168, (char)183, (char)36, (char)143, (char)134, (char)167, (char)23, (char)9, (char)81, (char)165, (char)128, (char)159, (char)233, (char)234, (char)27, (char)171, (char)76, (char)166, (char)209, (char)123, (char)144, (char)170, (char)115, (char)20, (char)63, (char)246, (char)142, (char)4, (char)54, (char)231, (char)147, (char)71, (char)106, (char)28, (char)141, (char)28, (char)27, (char)248, (char)120, (char)225, (char)145, (char)101, (char)228, (char)223, (char)62, (char)30, (char)53, (char)17, (char)163, (char)173, (char)97, (char)193, (char)127, (char)21, (char)165, (char)178, (char)87, (char)35, (char)162, (char)210, (char)148, (char)65, (char)227, (char)207, (char)87, (char)231, (char)102, (char)176, (char)206, (char)6, (char)66, (char)145, (char)70, (char)55, (char)209, (char)68, (char)162, (char)227, (char)165, (char)110, (char)173, (char)250, (char)49, (char)29, (char)237, (char)59, (char)12, (char)168, (char)66, (char)249, (char)239, (char)99, (char)227, (char)154, (char)217, (char)242, (char)192, (char)204, (char)3, (char)115, (char)101, (char)54, (char)37, (char)228, (char)235}));
            assert(pack.target_component_GET() == (char)247);
            assert(pack.target_system_GET() == (char)199);
            assert(pack.target_network_GET() == (char)136);
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)136) ;
        p110.payload_SET(new char[] {(char)254, (char)61, (char)175, (char)251, (char)137, (char)203, (char)208, (char)23, (char)199, (char)165, (char)236, (char)101, (char)77, (char)55, (char)7, (char)188, (char)144, (char)192, (char)15, (char)86, (char)174, (char)70, (char)192, (char)0, (char)64, (char)38, (char)26, (char)54, (char)166, (char)106, (char)114, (char)107, (char)228, (char)226, (char)168, (char)77, (char)37, (char)240, (char)98, (char)67, (char)50, (char)8, (char)241, (char)254, (char)223, (char)102, (char)143, (char)37, (char)86, (char)255, (char)147, (char)251, (char)32, (char)177, (char)62, (char)191, (char)7, (char)211, (char)128, (char)44, (char)247, (char)251, (char)113, (char)168, (char)165, (char)22, (char)110, (char)8, (char)186, (char)230, (char)199, (char)32, (char)195, (char)38, (char)208, (char)176, (char)84, (char)211, (char)97, (char)116, (char)194, (char)153, (char)94, (char)122, (char)37, (char)80, (char)180, (char)140, (char)128, (char)62, (char)149, (char)195, (char)116, (char)202, (char)233, (char)70, (char)195, (char)247, (char)191, (char)204, (char)189, (char)152, (char)120, (char)48, (char)27, (char)255, (char)161, (char)68, (char)75, (char)222, (char)163, (char)187, (char)201, (char)148, (char)117, (char)101, (char)209, (char)239, (char)237, (char)86, (char)41, (char)156, (char)220, (char)10, (char)207, (char)241, (char)169, (char)114, (char)129, (char)58, (char)230, (char)166, (char)75, (char)154, (char)29, (char)140, (char)76, (char)80, (char)67, (char)249, (char)240, (char)180, (char)94, (char)195, (char)206, (char)91, (char)168, (char)183, (char)36, (char)143, (char)134, (char)167, (char)23, (char)9, (char)81, (char)165, (char)128, (char)159, (char)233, (char)234, (char)27, (char)171, (char)76, (char)166, (char)209, (char)123, (char)144, (char)170, (char)115, (char)20, (char)63, (char)246, (char)142, (char)4, (char)54, (char)231, (char)147, (char)71, (char)106, (char)28, (char)141, (char)28, (char)27, (char)248, (char)120, (char)225, (char)145, (char)101, (char)228, (char)223, (char)62, (char)30, (char)53, (char)17, (char)163, (char)173, (char)97, (char)193, (char)127, (char)21, (char)165, (char)178, (char)87, (char)35, (char)162, (char)210, (char)148, (char)65, (char)227, (char)207, (char)87, (char)231, (char)102, (char)176, (char)206, (char)6, (char)66, (char)145, (char)70, (char)55, (char)209, (char)68, (char)162, (char)227, (char)165, (char)110, (char)173, (char)250, (char)49, (char)29, (char)237, (char)59, (char)12, (char)168, (char)66, (char)249, (char)239, (char)99, (char)227, (char)154, (char)217, (char)242, (char)192, (char)204, (char)3, (char)115, (char)101, (char)54, (char)37, (char)228, (char)235}, 0) ;
        p110.target_system_SET((char)199) ;
        p110.target_component_SET((char)247) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == -4899713994294222262L);
            assert(pack.ts1_GET() == 3878847302645449390L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(3878847302645449390L) ;
        p111.tc1_SET(-4899713994294222262L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6196867274232982125L);
            assert(pack.seq_GET() == 2153172961L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(2153172961L) ;
        p112.time_usec_SET(6196867274232982125L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -362310837);
            assert(pack.fix_type_GET() == (char)33);
            assert(pack.vel_GET() == (char)9453);
            assert(pack.eph_GET() == (char)63357);
            assert(pack.ve_GET() == (short) -31150);
            assert(pack.lon_GET() == 1532248015);
            assert(pack.alt_GET() == -1940697859);
            assert(pack.cog_GET() == (char)63805);
            assert(pack.epv_GET() == (char)49315);
            assert(pack.vd_GET() == (short) -5179);
            assert(pack.satellites_visible_GET() == (char)171);
            assert(pack.vn_GET() == (short)8997);
            assert(pack.time_usec_GET() == 2504720171180196220L);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.fix_type_SET((char)33) ;
        p113.epv_SET((char)49315) ;
        p113.lat_SET(-362310837) ;
        p113.satellites_visible_SET((char)171) ;
        p113.lon_SET(1532248015) ;
        p113.eph_SET((char)63357) ;
        p113.vel_SET((char)9453) ;
        p113.time_usec_SET(2504720171180196220L) ;
        p113.vn_SET((short)8997) ;
        p113.cog_SET((char)63805) ;
        p113.alt_SET(-1940697859) ;
        p113.vd_SET((short) -5179) ;
        p113.ve_SET((short) -31150) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integrated_zgyro_GET() == 4.664902E37F);
            assert(pack.quality_GET() == (char)111);
            assert(pack.integrated_y_GET() == -2.9431746E38F);
            assert(pack.time_usec_GET() == 5503103871825413163L);
            assert(pack.distance_GET() == 2.244763E38F);
            assert(pack.integrated_xgyro_GET() == -1.4437286E38F);
            assert(pack.integration_time_us_GET() == 539104404L);
            assert(pack.integrated_ygyro_GET() == 1.2689392E38F);
            assert(pack.sensor_id_GET() == (char)176);
            assert(pack.integrated_x_GET() == 1.4976909E38F);
            assert(pack.temperature_GET() == (short)30994);
            assert(pack.time_delta_distance_us_GET() == 598997739L);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.temperature_SET((short)30994) ;
        p114.time_delta_distance_us_SET(598997739L) ;
        p114.integrated_x_SET(1.4976909E38F) ;
        p114.integrated_ygyro_SET(1.2689392E38F) ;
        p114.integrated_zgyro_SET(4.664902E37F) ;
        p114.integrated_y_SET(-2.9431746E38F) ;
        p114.quality_SET((char)111) ;
        p114.integration_time_us_SET(539104404L) ;
        p114.integrated_xgyro_SET(-1.4437286E38F) ;
        p114.time_usec_SET(5503103871825413163L) ;
        p114.sensor_id_SET((char)176) ;
        p114.distance_SET(2.244763E38F) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 958778057);
            assert(pack.rollspeed_GET() == -1.6678399E38F);
            assert(pack.alt_GET() == 1754418957);
            assert(pack.ind_airspeed_GET() == (char)3437);
            assert(pack.zacc_GET() == (short)24601);
            assert(pack.yacc_GET() == (short) -3976);
            assert(pack.yawspeed_GET() == 1.2388865E38F);
            assert(pack.lon_GET() == 1594890379);
            assert(pack.vy_GET() == (short)18582);
            assert(pack.xacc_GET() == (short) -15419);
            assert(pack.vz_GET() == (short)14184);
            assert(pack.time_usec_GET() == 4060767666938041612L);
            assert(pack.true_airspeed_GET() == (char)26351);
            assert(pack.pitchspeed_GET() == -5.7394793E37F);
            assert(pack.vx_GET() == (short)16666);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {-2.647818E38F, 1.7844393E38F, 7.9849504E37F, -1.1061244E38F}));
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.alt_SET(1754418957) ;
        p115.yawspeed_SET(1.2388865E38F) ;
        p115.lat_SET(958778057) ;
        p115.attitude_quaternion_SET(new float[] {-2.647818E38F, 1.7844393E38F, 7.9849504E37F, -1.1061244E38F}, 0) ;
        p115.lon_SET(1594890379) ;
        p115.true_airspeed_SET((char)26351) ;
        p115.vy_SET((short)18582) ;
        p115.xacc_SET((short) -15419) ;
        p115.rollspeed_SET(-1.6678399E38F) ;
        p115.zacc_SET((short)24601) ;
        p115.pitchspeed_SET(-5.7394793E37F) ;
        p115.vz_SET((short)14184) ;
        p115.vx_SET((short)16666) ;
        p115.yacc_SET((short) -3976) ;
        p115.time_usec_SET(4060767666938041612L) ;
        p115.ind_airspeed_SET((char)3437) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == (short) -21724);
            assert(pack.ygyro_GET() == (short)5357);
            assert(pack.yacc_GET() == (short)23851);
            assert(pack.xacc_GET() == (short)29227);
            assert(pack.xgyro_GET() == (short) -12248);
            assert(pack.zacc_GET() == (short)19040);
            assert(pack.zmag_GET() == (short) -22148);
            assert(pack.ymag_GET() == (short) -24838);
            assert(pack.xmag_GET() == (short)2968);
            assert(pack.time_boot_ms_GET() == 3460826347L);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.xgyro_SET((short) -12248) ;
        p116.zgyro_SET((short) -21724) ;
        p116.xacc_SET((short)29227) ;
        p116.ygyro_SET((short)5357) ;
        p116.yacc_SET((short)23851) ;
        p116.time_boot_ms_SET(3460826347L) ;
        p116.zmag_SET((short) -22148) ;
        p116.zacc_SET((short)19040) ;
        p116.ymag_SET((short) -24838) ;
        p116.xmag_SET((short)2968) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)50);
            assert(pack.target_system_GET() == (char)2);
            assert(pack.start_GET() == (char)39713);
            assert(pack.end_GET() == (char)38587);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.end_SET((char)38587) ;
        p117.start_SET((char)39713) ;
        p117.target_component_SET((char)50) ;
        p117.target_system_SET((char)2) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)6291);
            assert(pack.time_utc_GET() == 3386841611L);
            assert(pack.size_GET() == 1869275498L);
            assert(pack.num_logs_GET() == (char)41769);
            assert(pack.last_log_num_GET() == (char)58792);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.time_utc_SET(3386841611L) ;
        p118.id_SET((char)6291) ;
        p118.size_SET(1869275498L) ;
        p118.last_log_num_SET((char)58792) ;
        p118.num_logs_SET((char)41769) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.ofs_GET() == 2047872812L);
            assert(pack.target_component_GET() == (char)11);
            assert(pack.id_GET() == (char)46637);
            assert(pack.count_GET() == 3547114213L);
            assert(pack.target_system_GET() == (char)250);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.ofs_SET(2047872812L) ;
        p119.target_component_SET((char)11) ;
        p119.count_SET(3547114213L) ;
        p119.id_SET((char)46637) ;
        p119.target_system_SET((char)250) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)189);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)11, (char)128, (char)6, (char)71, (char)18, (char)25, (char)179, (char)24, (char)237, (char)128, (char)118, (char)58, (char)74, (char)0, (char)161, (char)54, (char)123, (char)228, (char)124, (char)20, (char)175, (char)168, (char)33, (char)45, (char)165, (char)126, (char)74, (char)129, (char)49, (char)174, (char)172, (char)122, (char)12, (char)109, (char)8, (char)188, (char)20, (char)81, (char)34, (char)117, (char)241, (char)45, (char)107, (char)191, (char)127, (char)53, (char)0, (char)219, (char)186, (char)79, (char)69, (char)117, (char)202, (char)220, (char)58, (char)43, (char)137, (char)77, (char)44, (char)166, (char)33, (char)129, (char)186, (char)82, (char)227, (char)227, (char)249, (char)199, (char)138, (char)66, (char)3, (char)213, (char)208, (char)192, (char)75, (char)5, (char)156, (char)180, (char)175, (char)156, (char)143, (char)62, (char)118, (char)14, (char)127, (char)228, (char)88, (char)148, (char)158, (char)97}));
            assert(pack.id_GET() == (char)52426);
            assert(pack.ofs_GET() == 3031579179L);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.data__SET(new char[] {(char)11, (char)128, (char)6, (char)71, (char)18, (char)25, (char)179, (char)24, (char)237, (char)128, (char)118, (char)58, (char)74, (char)0, (char)161, (char)54, (char)123, (char)228, (char)124, (char)20, (char)175, (char)168, (char)33, (char)45, (char)165, (char)126, (char)74, (char)129, (char)49, (char)174, (char)172, (char)122, (char)12, (char)109, (char)8, (char)188, (char)20, (char)81, (char)34, (char)117, (char)241, (char)45, (char)107, (char)191, (char)127, (char)53, (char)0, (char)219, (char)186, (char)79, (char)69, (char)117, (char)202, (char)220, (char)58, (char)43, (char)137, (char)77, (char)44, (char)166, (char)33, (char)129, (char)186, (char)82, (char)227, (char)227, (char)249, (char)199, (char)138, (char)66, (char)3, (char)213, (char)208, (char)192, (char)75, (char)5, (char)156, (char)180, (char)175, (char)156, (char)143, (char)62, (char)118, (char)14, (char)127, (char)228, (char)88, (char)148, (char)158, (char)97}, 0) ;
        p120.count_SET((char)189) ;
        p120.id_SET((char)52426) ;
        p120.ofs_SET(3031579179L) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)224);
            assert(pack.target_component_GET() == (char)147);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_component_SET((char)147) ;
        p121.target_system_SET((char)224) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)86);
            assert(pack.target_component_GET() == (char)254);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_component_SET((char)254) ;
        p122.target_system_SET((char)86) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)37);
            assert(pack.target_system_GET() == (char)12);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)150, (char)90, (char)134, (char)13, (char)225, (char)193, (char)57, (char)222, (char)232, (char)80, (char)115, (char)246, (char)46, (char)93, (char)44, (char)215, (char)154, (char)206, (char)216, (char)118, (char)12, (char)174, (char)90, (char)199, (char)212, (char)205, (char)36, (char)250, (char)236, (char)206, (char)163, (char)48, (char)51, (char)107, (char)54, (char)175, (char)228, (char)217, (char)57, (char)235, (char)182, (char)16, (char)143, (char)190, (char)120, (char)121, (char)77, (char)85, (char)9, (char)155, (char)45, (char)34, (char)213, (char)238, (char)136, (char)66, (char)44, (char)93, (char)76, (char)8, (char)63, (char)192, (char)83, (char)93, (char)96, (char)71, (char)245, (char)77, (char)174, (char)224, (char)131, (char)197, (char)68, (char)50, (char)142, (char)66, (char)153, (char)173, (char)61, (char)87, (char)86, (char)7, (char)32, (char)131, (char)64, (char)123, (char)119, (char)85, (char)60, (char)252, (char)130, (char)68, (char)17, (char)145, (char)171, (char)205, (char)37, (char)124, (char)128, (char)241, (char)136, (char)48, (char)101, (char)46, (char)97, (char)79, (char)197, (char)135, (char)156, (char)121}));
            assert(pack.target_component_GET() == (char)114);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.data__SET(new char[] {(char)150, (char)90, (char)134, (char)13, (char)225, (char)193, (char)57, (char)222, (char)232, (char)80, (char)115, (char)246, (char)46, (char)93, (char)44, (char)215, (char)154, (char)206, (char)216, (char)118, (char)12, (char)174, (char)90, (char)199, (char)212, (char)205, (char)36, (char)250, (char)236, (char)206, (char)163, (char)48, (char)51, (char)107, (char)54, (char)175, (char)228, (char)217, (char)57, (char)235, (char)182, (char)16, (char)143, (char)190, (char)120, (char)121, (char)77, (char)85, (char)9, (char)155, (char)45, (char)34, (char)213, (char)238, (char)136, (char)66, (char)44, (char)93, (char)76, (char)8, (char)63, (char)192, (char)83, (char)93, (char)96, (char)71, (char)245, (char)77, (char)174, (char)224, (char)131, (char)197, (char)68, (char)50, (char)142, (char)66, (char)153, (char)173, (char)61, (char)87, (char)86, (char)7, (char)32, (char)131, (char)64, (char)123, (char)119, (char)85, (char)60, (char)252, (char)130, (char)68, (char)17, (char)145, (char)171, (char)205, (char)37, (char)124, (char)128, (char)241, (char)136, (char)48, (char)101, (char)46, (char)97, (char)79, (char)197, (char)135, (char)156, (char)121}, 0) ;
        p123.target_system_SET((char)12) ;
        p123.len_SET((char)37) ;
        p123.target_component_SET((char)114) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -942419385);
            assert(pack.cog_GET() == (char)42310);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
            assert(pack.dgps_numch_GET() == (char)252);
            assert(pack.lat_GET() == -874844007);
            assert(pack.epv_GET() == (char)52030);
            assert(pack.satellites_visible_GET() == (char)86);
            assert(pack.alt_GET() == 393131366);
            assert(pack.time_usec_GET() == 5494474254880586226L);
            assert(pack.vel_GET() == (char)31941);
            assert(pack.eph_GET() == (char)60456);
            assert(pack.dgps_age_GET() == 23516045L);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.epv_SET((char)52030) ;
        p124.dgps_numch_SET((char)252) ;
        p124.satellites_visible_SET((char)86) ;
        p124.dgps_age_SET(23516045L) ;
        p124.alt_SET(393131366) ;
        p124.vel_SET((char)31941) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP) ;
        p124.eph_SET((char)60456) ;
        p124.cog_SET((char)42310) ;
        p124.lon_SET(-942419385) ;
        p124.time_usec_SET(5494474254880586226L) ;
        p124.lat_SET(-874844007) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vcc_GET() == (char)10311);
            assert(pack.Vservo_GET() == (char)61526);
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED));
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vservo_SET((char)61526) ;
        p125.Vcc_SET((char)10311) ;
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED)) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.baudrate_GET() == 4028263711L);
            assert(pack.timeout_GET() == (char)23251);
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE));
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)153, (char)254, (char)42, (char)175, (char)44, (char)59, (char)85, (char)222, (char)150, (char)160, (char)176, (char)199, (char)107, (char)169, (char)102, (char)96, (char)141, (char)110, (char)84, (char)218, (char)86, (char)146, (char)160, (char)229, (char)57, (char)132, (char)106, (char)177, (char)88, (char)106, (char)9, (char)128, (char)147, (char)124, (char)64, (char)80, (char)30, (char)86, (char)12, (char)224, (char)7, (char)210, (char)116, (char)182, (char)115, (char)27, (char)24, (char)120, (char)148, (char)128, (char)106, (char)121, (char)124, (char)85, (char)33, (char)154, (char)189, (char)93, (char)93, (char)240, (char)116, (char)205, (char)66, (char)124, (char)8, (char)27, (char)252, (char)146, (char)79, (char)222}));
            assert(pack.count_GET() == (char)179);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.baudrate_SET(4028263711L) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1) ;
        p126.timeout_SET((char)23251) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE)) ;
        p126.count_SET((char)179) ;
        p126.data__SET(new char[] {(char)153, (char)254, (char)42, (char)175, (char)44, (char)59, (char)85, (char)222, (char)150, (char)160, (char)176, (char)199, (char)107, (char)169, (char)102, (char)96, (char)141, (char)110, (char)84, (char)218, (char)86, (char)146, (char)160, (char)229, (char)57, (char)132, (char)106, (char)177, (char)88, (char)106, (char)9, (char)128, (char)147, (char)124, (char)64, (char)80, (char)30, (char)86, (char)12, (char)224, (char)7, (char)210, (char)116, (char)182, (char)115, (char)27, (char)24, (char)120, (char)148, (char)128, (char)106, (char)121, (char)124, (char)85, (char)33, (char)154, (char)189, (char)93, (char)93, (char)240, (char)116, (char)205, (char)66, (char)124, (char)8, (char)27, (char)252, (char)146, (char)79, (char)222}, 0) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.time_last_baseline_ms_GET() == 326317298L);
            assert(pack.baseline_a_mm_GET() == 1317243632);
            assert(pack.baseline_b_mm_GET() == -1488640540);
            assert(pack.rtk_receiver_id_GET() == (char)205);
            assert(pack.accuracy_GET() == 390066673L);
            assert(pack.iar_num_hypotheses_GET() == 1275364505);
            assert(pack.rtk_health_GET() == (char)199);
            assert(pack.baseline_coords_type_GET() == (char)103);
            assert(pack.tow_GET() == 3138546557L);
            assert(pack.baseline_c_mm_GET() == 1603496192);
            assert(pack.nsats_GET() == (char)33);
            assert(pack.wn_GET() == (char)49098);
            assert(pack.rtk_rate_GET() == (char)244);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.nsats_SET((char)33) ;
        p127.wn_SET((char)49098) ;
        p127.iar_num_hypotheses_SET(1275364505) ;
        p127.rtk_health_SET((char)199) ;
        p127.baseline_b_mm_SET(-1488640540) ;
        p127.time_last_baseline_ms_SET(326317298L) ;
        p127.accuracy_SET(390066673L) ;
        p127.baseline_coords_type_SET((char)103) ;
        p127.tow_SET(3138546557L) ;
        p127.baseline_a_mm_SET(1317243632) ;
        p127.baseline_c_mm_SET(1603496192) ;
        p127.rtk_rate_SET((char)244) ;
        p127.rtk_receiver_id_SET((char)205) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.tow_GET() == 3262486901L);
            assert(pack.wn_GET() == (char)42031);
            assert(pack.rtk_health_GET() == (char)21);
            assert(pack.time_last_baseline_ms_GET() == 609532095L);
            assert(pack.baseline_b_mm_GET() == -74117433);
            assert(pack.baseline_c_mm_GET() == -1442117460);
            assert(pack.rtk_receiver_id_GET() == (char)183);
            assert(pack.nsats_GET() == (char)120);
            assert(pack.rtk_rate_GET() == (char)106);
            assert(pack.accuracy_GET() == 403857952L);
            assert(pack.iar_num_hypotheses_GET() == -1458312573);
            assert(pack.baseline_coords_type_GET() == (char)193);
            assert(pack.baseline_a_mm_GET() == 919643971);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.baseline_a_mm_SET(919643971) ;
        p128.wn_SET((char)42031) ;
        p128.tow_SET(3262486901L) ;
        p128.iar_num_hypotheses_SET(-1458312573) ;
        p128.baseline_c_mm_SET(-1442117460) ;
        p128.time_last_baseline_ms_SET(609532095L) ;
        p128.accuracy_SET(403857952L) ;
        p128.rtk_rate_SET((char)106) ;
        p128.baseline_b_mm_SET(-74117433) ;
        p128.rtk_health_SET((char)21) ;
        p128.baseline_coords_type_SET((char)193) ;
        p128.rtk_receiver_id_SET((char)183) ;
        p128.nsats_SET((char)120) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short) -9698);
            assert(pack.ymag_GET() == (short) -26706);
            assert(pack.zgyro_GET() == (short)18436);
            assert(pack.zacc_GET() == (short)27215);
            assert(pack.ygyro_GET() == (short) -31246);
            assert(pack.zmag_GET() == (short)28802);
            assert(pack.yacc_GET() == (short)14587);
            assert(pack.xgyro_GET() == (short)4067);
            assert(pack.time_boot_ms_GET() == 3835594752L);
            assert(pack.xmag_GET() == (short)23210);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.yacc_SET((short)14587) ;
        p129.zacc_SET((short)27215) ;
        p129.zmag_SET((short)28802) ;
        p129.ygyro_SET((short) -31246) ;
        p129.ymag_SET((short) -26706) ;
        p129.zgyro_SET((short)18436) ;
        p129.time_boot_ms_SET(3835594752L) ;
        p129.xgyro_SET((short)4067) ;
        p129.xacc_SET((short) -9698) ;
        p129.xmag_SET((short)23210) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.height_GET() == (char)4744);
            assert(pack.width_GET() == (char)32949);
            assert(pack.type_GET() == (char)212);
            assert(pack.payload_GET() == (char)215);
            assert(pack.packets_GET() == (char)29176);
            assert(pack.size_GET() == 2124440224L);
            assert(pack.jpg_quality_GET() == (char)250);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.packets_SET((char)29176) ;
        p130.width_SET((char)32949) ;
        p130.height_SET((char)4744) ;
        p130.payload_SET((char)215) ;
        p130.type_SET((char)212) ;
        p130.jpg_quality_SET((char)250) ;
        p130.size_SET(2124440224L) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)165, (char)157, (char)108, (char)131, (char)181, (char)24, (char)103, (char)169, (char)111, (char)184, (char)201, (char)204, (char)62, (char)34, (char)69, (char)241, (char)135, (char)108, (char)162, (char)150, (char)138, (char)23, (char)34, (char)101, (char)100, (char)96, (char)91, (char)233, (char)248, (char)174, (char)0, (char)234, (char)75, (char)124, (char)207, (char)241, (char)110, (char)10, (char)33, (char)248, (char)240, (char)125, (char)31, (char)150, (char)158, (char)236, (char)11, (char)89, (char)136, (char)62, (char)93, (char)207, (char)158, (char)252, (char)73, (char)75, (char)156, (char)125, (char)183, (char)213, (char)147, (char)173, (char)236, (char)199, (char)77, (char)188, (char)246, (char)204, (char)229, (char)180, (char)201, (char)126, (char)205, (char)206, (char)121, (char)18, (char)15, (char)10, (char)29, (char)62, (char)144, (char)98, (char)66, (char)177, (char)174, (char)186, (char)8, (char)210, (char)242, (char)170, (char)201, (char)29, (char)66, (char)132, (char)0, (char)91, (char)251, (char)45, (char)69, (char)144, (char)191, (char)213, (char)168, (char)109, (char)148, (char)206, (char)250, (char)57, (char)209, (char)99, (char)47, (char)192, (char)115, (char)251, (char)136, (char)24, (char)195, (char)129, (char)218, (char)186, (char)167, (char)34, (char)212, (char)23, (char)145, (char)235, (char)54, (char)53, (char)55, (char)118, (char)243, (char)110, (char)45, (char)244, (char)126, (char)107, (char)20, (char)148, (char)156, (char)192, (char)237, (char)165, (char)122, (char)6, (char)120, (char)61, (char)106, (char)3, (char)5, (char)22, (char)80, (char)103, (char)149, (char)50, (char)68, (char)173, (char)82, (char)171, (char)154, (char)86, (char)224, (char)2, (char)153, (char)185, (char)208, (char)140, (char)237, (char)74, (char)234, (char)220, (char)69, (char)9, (char)198, (char)131, (char)197, (char)79, (char)162, (char)38, (char)91, (char)179, (char)201, (char)77, (char)191, (char)134, (char)242, (char)142, (char)224, (char)132, (char)224, (char)29, (char)177, (char)46, (char)220, (char)83, (char)194, (char)63, (char)226, (char)104, (char)105, (char)140, (char)177, (char)178, (char)10, (char)245, (char)174, (char)71, (char)185, (char)24, (char)56, (char)229, (char)211, (char)87, (char)171, (char)49, (char)201, (char)8, (char)221, (char)222, (char)221, (char)71, (char)175, (char)135, (char)143, (char)41, (char)72, (char)108, (char)140, (char)46, (char)181, (char)202, (char)85, (char)148, (char)157, (char)62, (char)10, (char)118, (char)123, (char)87, (char)38, (char)26, (char)78, (char)77, (char)166, (char)30, (char)107, (char)130, (char)139, (char)235, (char)117, (char)208, (char)198, (char)32, (char)67}));
            assert(pack.seqnr_GET() == (char)54449);
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)54449) ;
        p131.data__SET(new char[] {(char)165, (char)157, (char)108, (char)131, (char)181, (char)24, (char)103, (char)169, (char)111, (char)184, (char)201, (char)204, (char)62, (char)34, (char)69, (char)241, (char)135, (char)108, (char)162, (char)150, (char)138, (char)23, (char)34, (char)101, (char)100, (char)96, (char)91, (char)233, (char)248, (char)174, (char)0, (char)234, (char)75, (char)124, (char)207, (char)241, (char)110, (char)10, (char)33, (char)248, (char)240, (char)125, (char)31, (char)150, (char)158, (char)236, (char)11, (char)89, (char)136, (char)62, (char)93, (char)207, (char)158, (char)252, (char)73, (char)75, (char)156, (char)125, (char)183, (char)213, (char)147, (char)173, (char)236, (char)199, (char)77, (char)188, (char)246, (char)204, (char)229, (char)180, (char)201, (char)126, (char)205, (char)206, (char)121, (char)18, (char)15, (char)10, (char)29, (char)62, (char)144, (char)98, (char)66, (char)177, (char)174, (char)186, (char)8, (char)210, (char)242, (char)170, (char)201, (char)29, (char)66, (char)132, (char)0, (char)91, (char)251, (char)45, (char)69, (char)144, (char)191, (char)213, (char)168, (char)109, (char)148, (char)206, (char)250, (char)57, (char)209, (char)99, (char)47, (char)192, (char)115, (char)251, (char)136, (char)24, (char)195, (char)129, (char)218, (char)186, (char)167, (char)34, (char)212, (char)23, (char)145, (char)235, (char)54, (char)53, (char)55, (char)118, (char)243, (char)110, (char)45, (char)244, (char)126, (char)107, (char)20, (char)148, (char)156, (char)192, (char)237, (char)165, (char)122, (char)6, (char)120, (char)61, (char)106, (char)3, (char)5, (char)22, (char)80, (char)103, (char)149, (char)50, (char)68, (char)173, (char)82, (char)171, (char)154, (char)86, (char)224, (char)2, (char)153, (char)185, (char)208, (char)140, (char)237, (char)74, (char)234, (char)220, (char)69, (char)9, (char)198, (char)131, (char)197, (char)79, (char)162, (char)38, (char)91, (char)179, (char)201, (char)77, (char)191, (char)134, (char)242, (char)142, (char)224, (char)132, (char)224, (char)29, (char)177, (char)46, (char)220, (char)83, (char)194, (char)63, (char)226, (char)104, (char)105, (char)140, (char)177, (char)178, (char)10, (char)245, (char)174, (char)71, (char)185, (char)24, (char)56, (char)229, (char)211, (char)87, (char)171, (char)49, (char)201, (char)8, (char)221, (char)222, (char)221, (char)71, (char)175, (char)135, (char)143, (char)41, (char)72, (char)108, (char)140, (char)46, (char)181, (char)202, (char)85, (char)148, (char)157, (char)62, (char)10, (char)118, (char)123, (char)87, (char)38, (char)26, (char)78, (char)77, (char)166, (char)30, (char)107, (char)130, (char)139, (char)235, (char)117, (char)208, (char)198, (char)32, (char)67}, 0) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.min_distance_GET() == (char)4187);
            assert(pack.max_distance_GET() == (char)31512);
            assert(pack.current_distance_GET() == (char)13428);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            assert(pack.id_GET() == (char)243);
            assert(pack.covariance_GET() == (char)128);
            assert(pack.time_boot_ms_GET() == 4104598853L);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_PITCH_180_YAW_90);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.max_distance_SET((char)31512) ;
        p132.current_distance_SET((char)13428) ;
        p132.min_distance_SET((char)4187) ;
        p132.time_boot_ms_SET(4104598853L) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_PITCH_180_YAW_90) ;
        p132.id_SET((char)243) ;
        p132.covariance_SET((char)128) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mask_GET() == 4514932068719099520L);
            assert(pack.lat_GET() == -504191991);
            assert(pack.lon_GET() == -539136784);
            assert(pack.grid_spacing_GET() == (char)39535);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(-504191991) ;
        p133.mask_SET(4514932068719099520L) ;
        p133.lon_SET(-539136784) ;
        p133.grid_spacing_SET((char)39535) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -632713902);
            assert(pack.gridbit_GET() == (char)160);
            assert(pack.grid_spacing_GET() == (char)55904);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -14462, (short)8364, (short) -22069, (short)32753, (short)31884, (short)7419, (short) -25685, (short)175, (short) -17048, (short)19123, (short)24169, (short)32422, (short) -16500, (short) -11970, (short) -17579, (short) -25615}));
            assert(pack.lon_GET() == -1617150543);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.gridbit_SET((char)160) ;
        p134.data__SET(new short[] {(short) -14462, (short)8364, (short) -22069, (short)32753, (short)31884, (short)7419, (short) -25685, (short)175, (short) -17048, (short)19123, (short)24169, (short)32422, (short) -16500, (short) -11970, (short) -17579, (short) -25615}, 0) ;
        p134.lon_SET(-1617150543) ;
        p134.lat_SET(-632713902) ;
        p134.grid_spacing_SET((char)55904) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -550263867);
            assert(pack.lon_GET() == -1096627039);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(-550263867) ;
        p135.lon_SET(-1096627039) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.spacing_GET() == (char)47575);
            assert(pack.pending_GET() == (char)8317);
            assert(pack.loaded_GET() == (char)41124);
            assert(pack.lon_GET() == -1896343115);
            assert(pack.terrain_height_GET() == -2.0710433E38F);
            assert(pack.lat_GET() == 812670059);
            assert(pack.current_height_GET() == -7.7003126E37F);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.pending_SET((char)8317) ;
        p136.current_height_SET(-7.7003126E37F) ;
        p136.spacing_SET((char)47575) ;
        p136.lon_SET(-1896343115) ;
        p136.terrain_height_SET(-2.0710433E38F) ;
        p136.loaded_SET((char)41124) ;
        p136.lat_SET(812670059) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1080236950L);
            assert(pack.temperature_GET() == (short) -24039);
            assert(pack.press_abs_GET() == 2.0875295E38F);
            assert(pack.press_diff_GET() == 1.8287915E38F);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.press_abs_SET(2.0875295E38F) ;
        p137.time_boot_ms_SET(1080236950L) ;
        p137.temperature_SET((short) -24039) ;
        p137.press_diff_SET(1.8287915E38F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 3.1778053E37F);
            assert(pack.z_GET() == -3.0488891E38F);
            assert(pack.time_usec_GET() == 2220919145712252165L);
            assert(pack.x_GET() == -2.3348337E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.7807128E38F, 2.8017567E38F, 1.3725326E38F, 2.699487E38F}));
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.y_SET(3.1778053E37F) ;
        p138.x_SET(-2.3348337E38F) ;
        p138.z_SET(-3.0488891E38F) ;
        p138.time_usec_SET(2220919145712252165L) ;
        p138.q_SET(new float[] {-1.7807128E38F, 2.8017567E38F, 1.3725326E38F, 2.699487E38F}, 0) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 9150269674092777247L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {3.3452677E38F, 3.1763359E38F, 9.942171E37F, 9.367713E37F, -3.152823E38F, -2.3302166E38F, -5.6267446E37F, 2.1955832E38F}));
            assert(pack.group_mlx_GET() == (char)252);
            assert(pack.target_component_GET() == (char)44);
            assert(pack.target_system_GET() == (char)20);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.target_system_SET((char)20) ;
        p139.target_component_SET((char)44) ;
        p139.time_usec_SET(9150269674092777247L) ;
        p139.controls_SET(new float[] {3.3452677E38F, 3.1763359E38F, 9.942171E37F, 9.367713E37F, -3.152823E38F, -2.3302166E38F, -5.6267446E37F, 2.1955832E38F}, 0) ;
        p139.group_mlx_SET((char)252) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)13);
            assert(pack.time_usec_GET() == 919477739213172184L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-3.3238734E38F, 1.4205042E38F, 2.7924757E38F, -1.6095588E38F, 2.7040042E38F, -3.1275433E38F, 2.33423E38F, -3.1663679E38F}));
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(919477739213172184L) ;
        p140.group_mlx_SET((char)13) ;
        p140.controls_SET(new float[] {-3.3238734E38F, 1.4205042E38F, 2.7924757E38F, -1.6095588E38F, 2.7040042E38F, -3.1275433E38F, 2.33423E38F, -3.1663679E38F}, 0) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_local_GET() == -2.1810164E37F);
            assert(pack.altitude_amsl_GET() == 9.266859E37F);
            assert(pack.bottom_clearance_GET() == 1.9079876E38F);
            assert(pack.altitude_relative_GET() == -5.2948346E37F);
            assert(pack.altitude_terrain_GET() == 2.4913571E38F);
            assert(pack.time_usec_GET() == 3317806796942592283L);
            assert(pack.altitude_monotonic_GET() == 2.6776756E38F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.time_usec_SET(3317806796942592283L) ;
        p141.altitude_monotonic_SET(2.6776756E38F) ;
        p141.bottom_clearance_SET(1.9079876E38F) ;
        p141.altitude_relative_SET(-5.2948346E37F) ;
        p141.altitude_amsl_SET(9.266859E37F) ;
        p141.altitude_terrain_SET(2.4913571E38F) ;
        p141.altitude_local_SET(-2.1810164E37F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.request_id_GET() == (char)150);
            assert(pack.uri_type_GET() == (char)136);
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)34, (char)187, (char)100, (char)66, (char)241, (char)71, (char)126, (char)147, (char)22, (char)214, (char)174, (char)180, (char)120, (char)33, (char)41, (char)151, (char)92, (char)153, (char)96, (char)114, (char)225, (char)81, (char)212, (char)200, (char)8, (char)105, (char)146, (char)247, (char)32, (char)101, (char)239, (char)241, (char)135, (char)207, (char)101, (char)144, (char)9, (char)44, (char)238, (char)0, (char)212, (char)110, (char)255, (char)153, (char)69, (char)45, (char)208, (char)197, (char)74, (char)219, (char)214, (char)44, (char)168, (char)218, (char)68, (char)244, (char)21, (char)97, (char)248, (char)55, (char)202, (char)1, (char)157, (char)49, (char)144, (char)202, (char)223, (char)179, (char)131, (char)123, (char)143, (char)76, (char)205, (char)143, (char)121, (char)114, (char)77, (char)40, (char)10, (char)58, (char)74, (char)197, (char)115, (char)219, (char)136, (char)147, (char)1, (char)1, (char)183, (char)220, (char)112, (char)216, (char)40, (char)190, (char)105, (char)86, (char)111, (char)223, (char)198, (char)70, (char)58, (char)127, (char)42, (char)157, (char)70, (char)84, (char)250, (char)184, (char)213, (char)43, (char)238, (char)17, (char)73, (char)178, (char)95, (char)208, (char)156, (char)191, (char)86, (char)7}));
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)110, (char)113, (char)144, (char)114, (char)117, (char)3, (char)153, (char)246, (char)138, (char)83, (char)16, (char)178, (char)9, (char)20, (char)53, (char)109, (char)10, (char)77, (char)134, (char)153, (char)251, (char)202, (char)116, (char)202, (char)250, (char)214, (char)178, (char)241, (char)249, (char)100, (char)91, (char)28, (char)194, (char)183, (char)208, (char)28, (char)161, (char)231, (char)236, (char)15, (char)172, (char)61, (char)199, (char)230, (char)193, (char)32, (char)57, (char)251, (char)42, (char)198, (char)97, (char)120, (char)150, (char)119, (char)18, (char)237, (char)173, (char)65, (char)248, (char)140, (char)174, (char)23, (char)191, (char)22, (char)221, (char)164, (char)251, (char)254, (char)125, (char)81, (char)217, (char)22, (char)121, (char)196, (char)170, (char)29, (char)123, (char)146, (char)184, (char)155, (char)252, (char)129, (char)65, (char)167, (char)147, (char)135, (char)184, (char)39, (char)203, (char)11, (char)224, (char)237, (char)250, (char)62, (char)161, (char)30, (char)74, (char)37, (char)224, (char)178, (char)98, (char)28, (char)30, (char)120, (char)164, (char)202, (char)208, (char)188, (char)17, (char)1, (char)116, (char)226, (char)162, (char)180, (char)184, (char)144, (char)70, (char)190, (char)33, (char)88}));
            assert(pack.transfer_type_GET() == (char)234);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.uri_type_SET((char)136) ;
        p142.transfer_type_SET((char)234) ;
        p142.storage_SET(new char[] {(char)110, (char)113, (char)144, (char)114, (char)117, (char)3, (char)153, (char)246, (char)138, (char)83, (char)16, (char)178, (char)9, (char)20, (char)53, (char)109, (char)10, (char)77, (char)134, (char)153, (char)251, (char)202, (char)116, (char)202, (char)250, (char)214, (char)178, (char)241, (char)249, (char)100, (char)91, (char)28, (char)194, (char)183, (char)208, (char)28, (char)161, (char)231, (char)236, (char)15, (char)172, (char)61, (char)199, (char)230, (char)193, (char)32, (char)57, (char)251, (char)42, (char)198, (char)97, (char)120, (char)150, (char)119, (char)18, (char)237, (char)173, (char)65, (char)248, (char)140, (char)174, (char)23, (char)191, (char)22, (char)221, (char)164, (char)251, (char)254, (char)125, (char)81, (char)217, (char)22, (char)121, (char)196, (char)170, (char)29, (char)123, (char)146, (char)184, (char)155, (char)252, (char)129, (char)65, (char)167, (char)147, (char)135, (char)184, (char)39, (char)203, (char)11, (char)224, (char)237, (char)250, (char)62, (char)161, (char)30, (char)74, (char)37, (char)224, (char)178, (char)98, (char)28, (char)30, (char)120, (char)164, (char)202, (char)208, (char)188, (char)17, (char)1, (char)116, (char)226, (char)162, (char)180, (char)184, (char)144, (char)70, (char)190, (char)33, (char)88}, 0) ;
        p142.uri_SET(new char[] {(char)34, (char)187, (char)100, (char)66, (char)241, (char)71, (char)126, (char)147, (char)22, (char)214, (char)174, (char)180, (char)120, (char)33, (char)41, (char)151, (char)92, (char)153, (char)96, (char)114, (char)225, (char)81, (char)212, (char)200, (char)8, (char)105, (char)146, (char)247, (char)32, (char)101, (char)239, (char)241, (char)135, (char)207, (char)101, (char)144, (char)9, (char)44, (char)238, (char)0, (char)212, (char)110, (char)255, (char)153, (char)69, (char)45, (char)208, (char)197, (char)74, (char)219, (char)214, (char)44, (char)168, (char)218, (char)68, (char)244, (char)21, (char)97, (char)248, (char)55, (char)202, (char)1, (char)157, (char)49, (char)144, (char)202, (char)223, (char)179, (char)131, (char)123, (char)143, (char)76, (char)205, (char)143, (char)121, (char)114, (char)77, (char)40, (char)10, (char)58, (char)74, (char)197, (char)115, (char)219, (char)136, (char)147, (char)1, (char)1, (char)183, (char)220, (char)112, (char)216, (char)40, (char)190, (char)105, (char)86, (char)111, (char)223, (char)198, (char)70, (char)58, (char)127, (char)42, (char)157, (char)70, (char)84, (char)250, (char)184, (char)213, (char)43, (char)238, (char)17, (char)73, (char)178, (char)95, (char)208, (char)156, (char)191, (char)86, (char)7}, 0) ;
        p142.request_id_SET((char)150) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -27325);
            assert(pack.press_abs_GET() == -2.644128E38F);
            assert(pack.time_boot_ms_GET() == 3098868946L);
            assert(pack.press_diff_GET() == 2.4393074E38F);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.temperature_SET((short) -27325) ;
        p143.press_diff_SET(2.4393074E38F) ;
        p143.press_abs_SET(-2.644128E38F) ;
        p143.time_boot_ms_SET(3098868946L) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-1.6061725E38F, 1.5288799E38F, 9.212202E37F}));
            assert(Arrays.equals(pack.rates_GET(),  new float[] {2.6691653E38F, 8.5552154E36F, -3.097216E38F}));
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-1.7978646E38F, -2.2348753E38F, -2.0786773E36F, -1.8436075E38F}));
            assert(pack.lat_GET() == 792154953);
            assert(pack.custom_state_GET() == 9150923443619018047L);
            assert(pack.timestamp_GET() == 4655351506187739685L);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-8.1083557E37F, -2.8788685E37F, -9.14953E37F}));
            assert(pack.lon_GET() == 1871838538);
            assert(pack.est_capabilities_GET() == (char)165);
            assert(Arrays.equals(pack.vel_GET(),  new float[] {2.1470415E38F, -6.5434055E37F, -2.8376543E38F}));
            assert(pack.alt_GET() == -7.4783896E37F);
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.alt_SET(-7.4783896E37F) ;
        p144.position_cov_SET(new float[] {-1.6061725E38F, 1.5288799E38F, 9.212202E37F}, 0) ;
        p144.lon_SET(1871838538) ;
        p144.attitude_q_SET(new float[] {-1.7978646E38F, -2.2348753E38F, -2.0786773E36F, -1.8436075E38F}, 0) ;
        p144.rates_SET(new float[] {2.6691653E38F, 8.5552154E36F, -3.097216E38F}, 0) ;
        p144.lat_SET(792154953) ;
        p144.est_capabilities_SET((char)165) ;
        p144.timestamp_SET(4655351506187739685L) ;
        p144.acc_SET(new float[] {-8.1083557E37F, -2.8788685E37F, -9.14953E37F}, 0) ;
        p144.vel_SET(new float[] {2.1470415E38F, -6.5434055E37F, -2.8376543E38F}, 0) ;
        p144.custom_state_SET(9150923443619018047L) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.x_vel_GET() == 1.8267237E36F);
            assert(pack.z_pos_GET() == 2.7744128E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {4.0706215E37F, -1.5468931E37F, -5.191089E37F, -2.324062E37F}));
            assert(pack.airspeed_GET() == -1.3488517E38F);
            assert(pack.pitch_rate_GET() == 1.4323338E38F);
            assert(pack.x_pos_GET() == 1.6373139E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {2.6893623E38F, 3.1288588E38F, -1.0131027E38F}));
            assert(pack.yaw_rate_GET() == -2.150362E38F);
            assert(pack.x_acc_GET() == -2.8065965E38F);
            assert(pack.z_vel_GET() == -6.368787E37F);
            assert(pack.time_usec_GET() == 5682880528613503210L);
            assert(pack.y_vel_GET() == -2.1616307E38F);
            assert(pack.z_acc_GET() == 1.0723514E38F);
            assert(pack.y_acc_GET() == 1.7605288E38F);
            assert(pack.roll_rate_GET() == -2.7912494E38F);
            assert(pack.y_pos_GET() == 1.4760466E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {-1.819086E38F, -2.5442997E38F, 3.0238083E38F}));
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.roll_rate_SET(-2.7912494E38F) ;
        p146.z_vel_SET(-6.368787E37F) ;
        p146.y_pos_SET(1.4760466E38F) ;
        p146.x_vel_SET(1.8267237E36F) ;
        p146.vel_variance_SET(new float[] {-1.819086E38F, -2.5442997E38F, 3.0238083E38F}, 0) ;
        p146.time_usec_SET(5682880528613503210L) ;
        p146.y_acc_SET(1.7605288E38F) ;
        p146.airspeed_SET(-1.3488517E38F) ;
        p146.y_vel_SET(-2.1616307E38F) ;
        p146.x_pos_SET(1.6373139E38F) ;
        p146.pos_variance_SET(new float[] {2.6893623E38F, 3.1288588E38F, -1.0131027E38F}, 0) ;
        p146.pitch_rate_SET(1.4323338E38F) ;
        p146.yaw_rate_SET(-2.150362E38F) ;
        p146.x_acc_SET(-2.8065965E38F) ;
        p146.q_SET(new float[] {4.0706215E37F, -1.5468931E37F, -5.191089E37F, -2.324062E37F}, 0) ;
        p146.z_acc_SET(1.0723514E38F) ;
        p146.z_pos_SET(2.7744128E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)117);
            assert(pack.current_battery_GET() == (short) -18006);
            assert(pack.temperature_GET() == (short)19526);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
            assert(pack.current_consumed_GET() == -915688223);
            assert(pack.energy_consumed_GET() == -815759302);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)28903, (char)55777, (char)61955, (char)63299, (char)15662, (char)44681, (char)34185, (char)18570, (char)26326, (char)52478}));
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN);
            assert(pack.battery_remaining_GET() == (byte) - 28);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.energy_consumed_SET(-815759302) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN) ;
        p147.current_battery_SET((short) -18006) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH) ;
        p147.battery_remaining_SET((byte) - 28) ;
        p147.temperature_SET((short)19526) ;
        p147.id_SET((char)117) ;
        p147.voltages_SET(new char[] {(char)28903, (char)55777, (char)61955, (char)63299, (char)15662, (char)44681, (char)34185, (char)18570, (char)26326, (char)52478}, 0) ;
        p147.current_consumed_SET(-915688223) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.product_id_GET() == (char)24703);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)130, (char)171, (char)111, (char)166, (char)146, (char)166, (char)48, (char)118}));
            assert(pack.middleware_sw_version_GET() == 718089438L);
            assert(pack.uid_GET() == 8625540134418989460L);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)167, (char)112, (char)69, (char)239, (char)180, (char)51, (char)53, (char)233}));
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE));
            assert(pack.vendor_id_GET() == (char)58244);
            assert(pack.board_version_GET() == 3229484890L);
            assert(pack.os_sw_version_GET() == 2971790823L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)203, (char)121, (char)158, (char)206, (char)118, (char)47, (char)43, (char)241, (char)175, (char)174, (char)119, (char)75, (char)151, (char)54, (char)174, (char)55, (char)29, (char)97}));
            assert(pack.flight_sw_version_GET() == 275418722L);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)90, (char)80, (char)252, (char)176, (char)143, (char)79, (char)131, (char)165}));
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.os_sw_version_SET(2971790823L) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE)) ;
        p148.flight_sw_version_SET(275418722L) ;
        p148.vendor_id_SET((char)58244) ;
        p148.product_id_SET((char)24703) ;
        p148.uid_SET(8625540134418989460L) ;
        p148.board_version_SET(3229484890L) ;
        p148.middleware_sw_version_SET(718089438L) ;
        p148.uid2_SET(new char[] {(char)203, (char)121, (char)158, (char)206, (char)118, (char)47, (char)43, (char)241, (char)175, (char)174, (char)119, (char)75, (char)151, (char)54, (char)174, (char)55, (char)29, (char)97}, 0, PH) ;
        p148.middleware_custom_version_SET(new char[] {(char)90, (char)80, (char)252, (char)176, (char)143, (char)79, (char)131, (char)165}, 0) ;
        p148.os_custom_version_SET(new char[] {(char)130, (char)171, (char)111, (char)166, (char)146, (char)166, (char)48, (char)118}, 0) ;
        p148.flight_custom_version_SET(new char[] {(char)167, (char)112, (char)69, (char)239, (char)180, (char)51, (char)53, (char)233}, 0) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.size_x_GET() == -2.3141218E37F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {2.501898E38F, 2.7728909E37F, 2.8556613E38F, 3.4011984E38F}));
            assert(pack.x_TRY(ph) == 2.9807621E38F);
            assert(pack.position_valid_TRY(ph) == (char)59);
            assert(pack.distance_GET() == -1.5570249E37F);
            assert(pack.target_num_GET() == (char)119);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.size_y_GET() == 2.3800215E38F);
            assert(pack.y_TRY(ph) == -1.5868159E38F);
            assert(pack.angle_y_GET() == 2.6994461E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL);
            assert(pack.angle_x_GET() == -1.9888769E38F);
            assert(pack.time_usec_GET() == 2115521995305182852L);
            assert(pack.z_TRY(ph) == -2.9334346E38F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.angle_y_SET(2.6994461E38F) ;
        p149.distance_SET(-1.5570249E37F) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL) ;
        p149.q_SET(new float[] {2.501898E38F, 2.7728909E37F, 2.8556613E38F, 3.4011984E38F}, 0, PH) ;
        p149.size_y_SET(2.3800215E38F) ;
        p149.z_SET(-2.9334346E38F, PH) ;
        p149.time_usec_SET(2115521995305182852L) ;
        p149.position_valid_SET((char)59, PH) ;
        p149.size_x_SET(-2.3141218E37F) ;
        p149.angle_x_SET(-1.9888769E38F) ;
        p149.y_SET(-1.5868159E38F, PH) ;
        p149.x_SET(2.9807621E38F, PH) ;
        p149.target_num_SET((char)119) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_CPU_LOAD.add((src, ph, pack) ->
        {
            assert(pack.batVolt_GET() == (char)56717);
            assert(pack.sensLoad_GET() == (char)229);
            assert(pack.ctrlLoad_GET() == (char)228);
        });
        GroundControl.CPU_LOAD p170 = CommunicationChannel.new_CPU_LOAD();
        PH.setPack(p170);
        p170.sensLoad_SET((char)229) ;
        p170.batVolt_SET((char)56717) ;
        p170.ctrlLoad_SET((char)228) ;
        CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SENSOR_BIAS.add((src, ph, pack) ->
        {
            assert(pack.ayBias_GET() == -2.43554E37F);
            assert(pack.axBias_GET() == 8.3207516E37F);
            assert(pack.azBias_GET() == -1.5450684E38F);
            assert(pack.gzBias_GET() == 2.9616701E38F);
            assert(pack.gxBias_GET() == 3.89782E36F);
            assert(pack.gyBias_GET() == 2.6994974E38F);
        });
        GroundControl.SENSOR_BIAS p172 = CommunicationChannel.new_SENSOR_BIAS();
        PH.setPack(p172);
        p172.azBias_SET(-1.5450684E38F) ;
        p172.gzBias_SET(2.9616701E38F) ;
        p172.gyBias_SET(2.6994974E38F) ;
        p172.gxBias_SET(3.89782E36F) ;
        p172.ayBias_SET(-2.43554E37F) ;
        p172.axBias_SET(8.3207516E37F) ;
        CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DIAGNOSTIC.add((src, ph, pack) ->
        {
            assert(pack.diagFl3_GET() == -7.877843E37F);
            assert(pack.diagSh3_GET() == (short)2004);
            assert(pack.diagSh1_GET() == (short)9935);
            assert(pack.diagFl2_GET() == -2.9339303E38F);
            assert(pack.diagSh2_GET() == (short)26508);
            assert(pack.diagFl1_GET() == -2.9118263E38F);
        });
        GroundControl.DIAGNOSTIC p173 = CommunicationChannel.new_DIAGNOSTIC();
        PH.setPack(p173);
        p173.diagSh3_SET((short)2004) ;
        p173.diagFl1_SET(-2.9118263E38F) ;
        p173.diagSh2_SET((short)26508) ;
        p173.diagSh1_SET((short)9935) ;
        p173.diagFl3_SET(-7.877843E37F) ;
        p173.diagFl2_SET(-2.9339303E38F) ;
        CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SLUGS_NAVIGATION.add((src, ph, pack) ->
        {
            assert(pack.totalDist_GET() == -2.8736513E38F);
            assert(pack.phi_c_GET() == -3.36123E38F);
            assert(pack.u_m_GET() == -1.6619864E38F);
            assert(pack.toWP_GET() == (char)39);
            assert(pack.ay_body_GET() == 2.4740309E38F);
            assert(pack.psiDot_c_GET() == 3.0271892E38F);
            assert(pack.fromWP_GET() == (char)66);
            assert(pack.theta_c_GET() == -1.3372459E38F);
            assert(pack.h_c_GET() == (char)6318);
            assert(pack.dist2Go_GET() == 8.813869E37F);
        });
        GroundControl.SLUGS_NAVIGATION p176 = CommunicationChannel.new_SLUGS_NAVIGATION();
        PH.setPack(p176);
        p176.psiDot_c_SET(3.0271892E38F) ;
        p176.totalDist_SET(-2.8736513E38F) ;
        p176.toWP_SET((char)39) ;
        p176.fromWP_SET((char)66) ;
        p176.dist2Go_SET(8.813869E37F) ;
        p176.ay_body_SET(2.4740309E38F) ;
        p176.theta_c_SET(-1.3372459E38F) ;
        p176.h_c_SET((char)6318) ;
        p176.u_m_SET(-1.6619864E38F) ;
        p176.phi_c_SET(-3.36123E38F) ;
        CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DATA_LOG.add((src, ph, pack) ->
        {
            assert(pack.fl_1_GET() == 2.8214777E38F);
            assert(pack.fl_5_GET() == -1.9477898E38F);
            assert(pack.fl_3_GET() == -1.7638403E38F);
            assert(pack.fl_6_GET() == -2.26885E38F);
            assert(pack.fl_2_GET() == 3.1424787E37F);
            assert(pack.fl_4_GET() == 1.5339433E38F);
        });
        GroundControl.DATA_LOG p177 = CommunicationChannel.new_DATA_LOG();
        PH.setPack(p177);
        p177.fl_3_SET(-1.7638403E38F) ;
        p177.fl_6_SET(-2.26885E38F) ;
        p177.fl_1_SET(2.8214777E38F) ;
        p177.fl_4_SET(1.5339433E38F) ;
        p177.fl_5_SET(-1.9477898E38F) ;
        p177.fl_2_SET(3.1424787E37F) ;
        CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_DATE_TIME.add((src, ph, pack) ->
        {
            assert(pack.min_GET() == (char)161);
            assert(pack.visSat_GET() == (char)230);
            assert(pack.useSat_GET() == (char)70);
            assert(pack.day_GET() == (char)62);
            assert(pack.percentUsed_GET() == (char)248);
            assert(pack.sec_GET() == (char)182);
            assert(pack.sigUsedMask_GET() == (char)176);
            assert(pack.year_GET() == (char)109);
            assert(pack.month_GET() == (char)157);
            assert(pack.hour_GET() == (char)194);
            assert(pack.clockStat_GET() == (char)144);
            assert(pack.GppGl_GET() == (char)76);
        });
        GroundControl.GPS_DATE_TIME p179 = CommunicationChannel.new_GPS_DATE_TIME();
        PH.setPack(p179);
        p179.day_SET((char)62) ;
        p179.GppGl_SET((char)76) ;
        p179.visSat_SET((char)230) ;
        p179.year_SET((char)109) ;
        p179.useSat_SET((char)70) ;
        p179.clockStat_SET((char)144) ;
        p179.month_SET((char)157) ;
        p179.hour_SET((char)194) ;
        p179.sigUsedMask_SET((char)176) ;
        p179.sec_SET((char)182) ;
        p179.percentUsed_SET((char)248) ;
        p179.min_SET((char)161) ;
        CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MID_LVL_CMDS.add((src, ph, pack) ->
        {
            assert(pack.rCommand_GET() == -2.8050804E38F);
            assert(pack.target_GET() == (char)99);
            assert(pack.hCommand_GET() == 2.287806E36F);
            assert(pack.uCommand_GET() == -3.0794945E37F);
        });
        GroundControl.MID_LVL_CMDS p180 = CommunicationChannel.new_MID_LVL_CMDS();
        PH.setPack(p180);
        p180.hCommand_SET(2.287806E36F) ;
        p180.rCommand_SET(-2.8050804E38F) ;
        p180.uCommand_SET(-3.0794945E37F) ;
        p180.target_SET((char)99) ;
        CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CTRL_SRFC_PT.add((src, ph, pack) ->
        {
            assert(pack.bitfieldPt_GET() == (char)13987);
            assert(pack.target_GET() == (char)127);
        });
        GroundControl.CTRL_SRFC_PT p181 = CommunicationChannel.new_CTRL_SRFC_PT();
        PH.setPack(p181);
        p181.bitfieldPt_SET((char)13987) ;
        p181.target_SET((char)127) ;
        CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SLUGS_CAMERA_ORDER.add((src, ph, pack) ->
        {
            assert(pack.tilt_GET() == (byte) - 62);
            assert(pack.target_GET() == (char)86);
            assert(pack.zoom_GET() == (byte)94);
            assert(pack.moveHome_GET() == (byte) - 44);
            assert(pack.pan_GET() == (byte) - 124);
        });
        GroundControl.SLUGS_CAMERA_ORDER p184 = CommunicationChannel.new_SLUGS_CAMERA_ORDER();
        PH.setPack(p184);
        p184.tilt_SET((byte) - 62) ;
        p184.pan_SET((byte) - 124) ;
        p184.target_SET((char)86) ;
        p184.zoom_SET((byte)94) ;
        p184.moveHome_SET((byte) - 44) ;
        CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CONTROL_SURFACE.add((src, ph, pack) ->
        {
            assert(pack.bControl_GET() == -1.3727719E38F);
            assert(pack.idSurface_GET() == (char)115);
            assert(pack.mControl_GET() == -9.1711504E36F);
            assert(pack.target_GET() == (char)243);
        });
        GroundControl.CONTROL_SURFACE p185 = CommunicationChannel.new_CONTROL_SURFACE();
        PH.setPack(p185);
        p185.bControl_SET(-1.3727719E38F) ;
        p185.target_SET((char)243) ;
        p185.idSurface_SET((char)115) ;
        p185.mControl_SET(-9.1711504E36F) ;
        CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SLUGS_MOBILE_LOCATION.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == -4.2482435E37F);
            assert(pack.target_GET() == (char)57);
            assert(pack.latitude_GET() == 2.8980054E38F);
        });
        GroundControl.SLUGS_MOBILE_LOCATION p186 = CommunicationChannel.new_SLUGS_MOBILE_LOCATION();
        PH.setPack(p186);
        p186.longitude_SET(-4.2482435E37F) ;
        p186.latitude_SET(2.8980054E38F) ;
        p186.target_SET((char)57) ;
        CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SLUGS_CONFIGURATION_CAMERA.add((src, ph, pack) ->
        {
            assert(pack.idOrder_GET() == (char)211);
            assert(pack.target_GET() == (char)244);
            assert(pack.order_GET() == (char)51);
        });
        GroundControl.SLUGS_CONFIGURATION_CAMERA p188 = CommunicationChannel.new_SLUGS_CONFIGURATION_CAMERA();
        PH.setPack(p188);
        p188.idOrder_SET((char)211) ;
        p188.order_SET((char)51) ;
        p188.target_SET((char)244) ;
        CommunicationChannel.instance.send(p188);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ISR_LOCATION.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == -3.2822086E38F);
            assert(pack.option2_GET() == (char)47);
            assert(pack.target_GET() == (char)19);
            assert(pack.option3_GET() == (char)248);
            assert(pack.height_GET() == -2.360882E38F);
            assert(pack.option1_GET() == (char)39);
            assert(pack.latitude_GET() == -2.9174948E38F);
        });
        GroundControl.ISR_LOCATION p189 = CommunicationChannel.new_ISR_LOCATION();
        PH.setPack(p189);
        p189.option3_SET((char)248) ;
        p189.option2_SET((char)47) ;
        p189.target_SET((char)19) ;
        p189.option1_SET((char)39) ;
        p189.height_SET(-2.360882E38F) ;
        p189.longitude_SET(-3.2822086E38F) ;
        p189.latitude_SET(-2.9174948E38F) ;
        CommunicationChannel.instance.send(p189);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VOLT_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.reading2_GET() == (char)47133);
            assert(pack.voltage_GET() == (char)14970);
            assert(pack.r2Type_GET() == (char)95);
        });
        GroundControl.VOLT_SENSOR p191 = CommunicationChannel.new_VOLT_SENSOR();
        PH.setPack(p191);
        p191.reading2_SET((char)47133) ;
        p191.r2Type_SET((char)95) ;
        p191.voltage_SET((char)14970) ;
        CommunicationChannel.instance.send(p191);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PTZ_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pan_GET() == (short)21492);
            assert(pack.tilt_GET() == (short) -29933);
            assert(pack.zoom_GET() == (char)69);
        });
        GroundControl.PTZ_STATUS p192 = CommunicationChannel.new_PTZ_STATUS();
        PH.setPack(p192);
        p192.pan_SET((short)21492) ;
        p192.zoom_SET((char)69) ;
        p192.tilt_SET((short) -29933) ;
        CommunicationChannel.instance.send(p192);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAV_STATUS.add((src, ph, pack) ->
        {
            assert(pack.target_GET() == (char)181);
            assert(pack.course_GET() == 2.1603188E38F);
            assert(pack.latitude_GET() == -4.0816098E37F);
            assert(pack.longitude_GET() == -2.876804E38F);
            assert(pack.speed_GET() == -2.3847915E38F);
            assert(pack.altitude_GET() == 1.9679527E38F);
        });
        GroundControl.UAV_STATUS p193 = CommunicationChannel.new_UAV_STATUS();
        PH.setPack(p193);
        p193.latitude_SET(-4.0816098E37F) ;
        p193.speed_SET(-2.3847915E38F) ;
        p193.altitude_SET(1.9679527E38F) ;
        p193.longitude_SET(-2.876804E38F) ;
        p193.target_SET((char)181) ;
        p193.course_SET(2.1603188E38F) ;
        CommunicationChannel.instance.send(p193);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STATUS_GPS.add((src, ph, pack) ->
        {
            assert(pack.gpsQuality_GET() == (char)224);
            assert(pack.modeInd_GET() == (char)0);
            assert(pack.posStatus_GET() == (char)227);
            assert(pack.magDir_GET() == (byte)84);
            assert(pack.msgsType_GET() == (char)240);
            assert(pack.magVar_GET() == -1.8773231E37F);
            assert(pack.csFails_GET() == (char)46867);
        });
        GroundControl.STATUS_GPS p194 = CommunicationChannel.new_STATUS_GPS();
        PH.setPack(p194);
        p194.gpsQuality_SET((char)224) ;
        p194.magVar_SET(-1.8773231E37F) ;
        p194.magDir_SET((byte)84) ;
        p194.csFails_SET((char)46867) ;
        p194.posStatus_SET((char)227) ;
        p194.modeInd_SET((char)0) ;
        p194.msgsType_SET((char)240) ;
        CommunicationChannel.instance.send(p194);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NOVATEL_DIAG.add((src, ph, pack) ->
        {
            assert(pack.receiverStatus_GET() == 97700456L);
            assert(pack.timeStatus_GET() == (char)233);
            assert(pack.posType_GET() == (char)80);
            assert(pack.velType_GET() == (char)105);
            assert(pack.posSolAge_GET() == -6.430693E37F);
            assert(pack.solStatus_GET() == (char)88);
            assert(pack.csFails_GET() == (char)61733);
        });
        GroundControl.NOVATEL_DIAG p195 = CommunicationChannel.new_NOVATEL_DIAG();
        PH.setPack(p195);
        p195.solStatus_SET((char)88) ;
        p195.posType_SET((char)80) ;
        p195.csFails_SET((char)61733) ;
        p195.velType_SET((char)105) ;
        p195.timeStatus_SET((char)233) ;
        p195.receiverStatus_SET(97700456L) ;
        p195.posSolAge_SET(-6.430693E37F) ;
        CommunicationChannel.instance.send(p195);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SENSOR_DIAG.add((src, ph, pack) ->
        {
            assert(pack.char1_GET() == (byte) - 125);
            assert(pack.float1_GET() == -1.8196082E38F);
            assert(pack.int1_GET() == (short) -1178);
            assert(pack.float2_GET() == -1.1633247E38F);
        });
        GroundControl.SENSOR_DIAG p196 = CommunicationChannel.new_SENSOR_DIAG();
        PH.setPack(p196);
        p196.int1_SET((short) -1178) ;
        p196.char1_SET((byte) - 125) ;
        p196.float1_SET(-1.8196082E38F) ;
        p196.float2_SET(-1.1633247E38F) ;
        CommunicationChannel.instance.send(p196);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BOOT.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == 1727317095L);
        });
        GroundControl.BOOT p197 = CommunicationChannel.new_BOOT();
        PH.setPack(p197);
        p197.version_SET(1727317095L) ;
        CommunicationChannel.instance.send(p197);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        CommunicationChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pos_horiz_ratio_GET() == 3.3688512E38F);
            assert(pack.mag_ratio_GET() == -2.7000418E38F);
            assert(pack.pos_vert_accuracy_GET() == -9.631621E37F);
            assert(pack.tas_ratio_GET() == 2.0923993E38F);
            assert(pack.hagl_ratio_GET() == -2.0553078E38F);
            assert(pack.pos_horiz_accuracy_GET() == -9.673157E37F);
            assert(pack.pos_vert_ratio_GET() == -3.2886335E38F);
            assert(pack.time_usec_GET() == 3428603003195403508L);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ));
            assert(pack.vel_ratio_GET() == 1.5636721E38F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.hagl_ratio_SET(-2.0553078E38F) ;
        p230.vel_ratio_SET(1.5636721E38F) ;
        p230.mag_ratio_SET(-2.7000418E38F) ;
        p230.pos_vert_accuracy_SET(-9.631621E37F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ)) ;
        p230.time_usec_SET(3428603003195403508L) ;
        p230.pos_vert_ratio_SET(-3.2886335E38F) ;
        p230.pos_horiz_accuracy_SET(-9.673157E37F) ;
        p230.tas_ratio_SET(2.0923993E38F) ;
        p230.pos_horiz_ratio_SET(3.3688512E38F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_alt_GET() == 1.2185533E38F);
            assert(pack.var_horiz_GET() == -8.8937466E36F);
            assert(pack.var_vert_GET() == 2.0956321E38F);
            assert(pack.horiz_accuracy_GET() == 3.3317715E38F);
            assert(pack.wind_y_GET() == -3.8195846E37F);
            assert(pack.wind_x_GET() == -2.2591787E38F);
            assert(pack.wind_z_GET() == 3.0203901E38F);
            assert(pack.vert_accuracy_GET() == 2.3929012E38F);
            assert(pack.time_usec_GET() == 3930462070828788411L);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_y_SET(-3.8195846E37F) ;
        p231.vert_accuracy_SET(2.3929012E38F) ;
        p231.wind_alt_SET(1.2185533E38F) ;
        p231.var_vert_SET(2.0956321E38F) ;
        p231.time_usec_SET(3930462070828788411L) ;
        p231.horiz_accuracy_SET(3.3317715E38F) ;
        p231.wind_x_SET(-2.2591787E38F) ;
        p231.var_horiz_SET(-8.8937466E36F) ;
        p231.wind_z_SET(3.0203901E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4966213143787849611L);
            assert(pack.vert_accuracy_GET() == 5.234376E37F);
            assert(pack.satellites_visible_GET() == (char)249);
            assert(pack.alt_GET() == -3.0002595E37F);
            assert(pack.time_week_ms_GET() == 1099735583L);
            assert(pack.gps_id_GET() == (char)235);
            assert(pack.time_week_GET() == (char)50663);
            assert(pack.lat_GET() == -849812805);
            assert(pack.vdop_GET() == -1.2929863E38F);
            assert(pack.speed_accuracy_GET() == -2.474084E38F);
            assert(pack.horiz_accuracy_GET() == -9.890755E37F);
            assert(pack.ve_GET() == -1.7966838E38F);
            assert(pack.vd_GET() == -2.355044E38F);
            assert(pack.vn_GET() == -1.66745E38F);
            assert(pack.lon_GET() == 1034177678);
            assert(pack.hdop_GET() == 5.2503674E37F);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT));
            assert(pack.fix_type_GET() == (char)48);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.vn_SET(-1.66745E38F) ;
        p232.horiz_accuracy_SET(-9.890755E37F) ;
        p232.time_week_SET((char)50663) ;
        p232.speed_accuracy_SET(-2.474084E38F) ;
        p232.alt_SET(-3.0002595E37F) ;
        p232.hdop_SET(5.2503674E37F) ;
        p232.vert_accuracy_SET(5.234376E37F) ;
        p232.lat_SET(-849812805) ;
        p232.ve_SET(-1.7966838E38F) ;
        p232.satellites_visible_SET((char)249) ;
        p232.time_usec_SET(4966213143787849611L) ;
        p232.vdop_SET(-1.2929863E38F) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT)) ;
        p232.time_week_ms_SET(1099735583L) ;
        p232.vd_SET(-2.355044E38F) ;
        p232.gps_id_SET((char)235) ;
        p232.fix_type_SET((char)48) ;
        p232.lon_SET(1034177678) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)239, (char)148, (char)156, (char)232, (char)23, (char)248, (char)107, (char)158, (char)16, (char)255, (char)1, (char)237, (char)229, (char)245, (char)34, (char)37, (char)10, (char)209, (char)85, (char)71, (char)147, (char)236, (char)161, (char)57, (char)106, (char)225, (char)185, (char)47, (char)154, (char)141, (char)38, (char)108, (char)228, (char)53, (char)140, (char)153, (char)29, (char)241, (char)213, (char)124, (char)228, (char)47, (char)122, (char)190, (char)38, (char)255, (char)239, (char)113, (char)107, (char)59, (char)0, (char)115, (char)60, (char)230, (char)92, (char)220, (char)230, (char)242, (char)134, (char)168, (char)92, (char)8, (char)107, (char)159, (char)142, (char)69, (char)193, (char)185, (char)115, (char)203, (char)254, (char)225, (char)136, (char)104, (char)102, (char)75, (char)153, (char)193, (char)104, (char)126, (char)229, (char)169, (char)44, (char)30, (char)226, (char)135, (char)45, (char)64, (char)112, (char)90, (char)120, (char)239, (char)173, (char)203, (char)182, (char)217, (char)132, (char)97, (char)95, (char)177, (char)85, (char)111, (char)17, (char)212, (char)81, (char)124, (char)200, (char)26, (char)156, (char)4, (char)116, (char)51, (char)205, (char)65, (char)190, (char)183, (char)178, (char)43, (char)201, (char)209, (char)31, (char)128, (char)66, (char)245, (char)243, (char)107, (char)254, (char)116, (char)72, (char)134, (char)20, (char)49, (char)201, (char)55, (char)59, (char)249, (char)96, (char)217, (char)38, (char)15, (char)222, (char)8, (char)8, (char)167, (char)197, (char)27, (char)110, (char)216, (char)175, (char)140, (char)150, (char)54, (char)222, (char)12, (char)95, (char)242, (char)98, (char)41, (char)245, (char)11, (char)0, (char)13, (char)45, (char)230, (char)7, (char)3, (char)157, (char)59, (char)223, (char)196, (char)134, (char)58, (char)137, (char)27, (char)56, (char)1, (char)55, (char)38, (char)118, (char)253}));
            assert(pack.flags_GET() == (char)183);
            assert(pack.len_GET() == (char)100);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)183) ;
        p233.data__SET(new char[] {(char)239, (char)148, (char)156, (char)232, (char)23, (char)248, (char)107, (char)158, (char)16, (char)255, (char)1, (char)237, (char)229, (char)245, (char)34, (char)37, (char)10, (char)209, (char)85, (char)71, (char)147, (char)236, (char)161, (char)57, (char)106, (char)225, (char)185, (char)47, (char)154, (char)141, (char)38, (char)108, (char)228, (char)53, (char)140, (char)153, (char)29, (char)241, (char)213, (char)124, (char)228, (char)47, (char)122, (char)190, (char)38, (char)255, (char)239, (char)113, (char)107, (char)59, (char)0, (char)115, (char)60, (char)230, (char)92, (char)220, (char)230, (char)242, (char)134, (char)168, (char)92, (char)8, (char)107, (char)159, (char)142, (char)69, (char)193, (char)185, (char)115, (char)203, (char)254, (char)225, (char)136, (char)104, (char)102, (char)75, (char)153, (char)193, (char)104, (char)126, (char)229, (char)169, (char)44, (char)30, (char)226, (char)135, (char)45, (char)64, (char)112, (char)90, (char)120, (char)239, (char)173, (char)203, (char)182, (char)217, (char)132, (char)97, (char)95, (char)177, (char)85, (char)111, (char)17, (char)212, (char)81, (char)124, (char)200, (char)26, (char)156, (char)4, (char)116, (char)51, (char)205, (char)65, (char)190, (char)183, (char)178, (char)43, (char)201, (char)209, (char)31, (char)128, (char)66, (char)245, (char)243, (char)107, (char)254, (char)116, (char)72, (char)134, (char)20, (char)49, (char)201, (char)55, (char)59, (char)249, (char)96, (char)217, (char)38, (char)15, (char)222, (char)8, (char)8, (char)167, (char)197, (char)27, (char)110, (char)216, (char)175, (char)140, (char)150, (char)54, (char)222, (char)12, (char)95, (char)242, (char)98, (char)41, (char)245, (char)11, (char)0, (char)13, (char)45, (char)230, (char)7, (char)3, (char)157, (char)59, (char)223, (char)196, (char)134, (char)58, (char)137, (char)27, (char)56, (char)1, (char)55, (char)38, (char)118, (char)253}, 0) ;
        p233.len_SET((char)100) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.wp_num_GET() == (char)160);
            assert(pack.altitude_sp_GET() == (short)21751);
            assert(pack.latitude_GET() == -86212171);
            assert(pack.altitude_amsl_GET() == (short) -6207);
            assert(pack.heading_sp_GET() == (short) -17071);
            assert(pack.custom_mode_GET() == 3467685156L);
            assert(pack.longitude_GET() == 728572806);
            assert(pack.temperature_GET() == (byte) - 2);
            assert(pack.battery_remaining_GET() == (char)29);
            assert(pack.pitch_GET() == (short)4119);
            assert(pack.airspeed_GET() == (char)198);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
            assert(pack.groundspeed_GET() == (char)232);
            assert(pack.climb_rate_GET() == (byte)56);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
            assert(pack.airspeed_sp_GET() == (char)139);
            assert(pack.roll_GET() == (short) -23258);
            assert(pack.heading_GET() == (char)32358);
            assert(pack.temperature_air_GET() == (byte)83);
            assert(pack.failsafe_GET() == (char)135);
            assert(pack.throttle_GET() == (byte) - 6);
            assert(pack.wp_distance_GET() == (char)36571);
            assert(pack.gps_nsat_GET() == (char)18);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT) ;
        p234.airspeed_SET((char)198) ;
        p234.airspeed_sp_SET((char)139) ;
        p234.groundspeed_SET((char)232) ;
        p234.pitch_SET((short)4119) ;
        p234.throttle_SET((byte) - 6) ;
        p234.altitude_amsl_SET((short) -6207) ;
        p234.gps_nsat_SET((char)18) ;
        p234.wp_distance_SET((char)36571) ;
        p234.battery_remaining_SET((char)29) ;
        p234.heading_SET((char)32358) ;
        p234.latitude_SET(-86212171) ;
        p234.wp_num_SET((char)160) ;
        p234.climb_rate_SET((byte)56) ;
        p234.temperature_air_SET((byte)83) ;
        p234.temperature_SET((byte) - 2) ;
        p234.failsafe_SET((char)135) ;
        p234.longitude_SET(728572806) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED)) ;
        p234.roll_SET((short) -23258) ;
        p234.altitude_sp_SET((short)21751) ;
        p234.heading_sp_SET((short) -17071) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED) ;
        p234.custom_mode_SET(3467685156L) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.clipping_2_GET() == 3527741782L);
            assert(pack.vibration_y_GET() == -1.3144132E38F);
            assert(pack.vibration_x_GET() == 1.2493721E38F);
            assert(pack.time_usec_GET() == 1566007776008762621L);
            assert(pack.clipping_1_GET() == 2508436110L);
            assert(pack.vibration_z_GET() == 1.2724849E38F);
            assert(pack.clipping_0_GET() == 2920755607L);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(1566007776008762621L) ;
        p241.vibration_y_SET(-1.3144132E38F) ;
        p241.vibration_z_SET(1.2724849E38F) ;
        p241.vibration_x_SET(1.2493721E38F) ;
        p241.clipping_2_SET(3527741782L) ;
        p241.clipping_1_SET(2508436110L) ;
        p241.clipping_0_SET(2920755607L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 4.793879E37F);
            assert(pack.approach_x_GET() == -3.146864E38F);
            assert(pack.time_usec_TRY(ph) == 3133540926350245024L);
            assert(pack.latitude_GET() == 1928306629);
            assert(pack.z_GET() == -1.695848E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.2502079E38F, -1.2300424E38F, 1.743458E38F, 1.939881E38F}));
            assert(pack.approach_z_GET() == 3.1152777E38F);
            assert(pack.longitude_GET() == 381102094);
            assert(pack.altitude_GET() == -88130036);
            assert(pack.y_GET() == -3.4111273E37F);
            assert(pack.approach_y_GET() == -2.935385E38F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.approach_x_SET(-3.146864E38F) ;
        p242.latitude_SET(1928306629) ;
        p242.longitude_SET(381102094) ;
        p242.q_SET(new float[] {-3.2502079E38F, -1.2300424E38F, 1.743458E38F, 1.939881E38F}, 0) ;
        p242.y_SET(-3.4111273E37F) ;
        p242.approach_y_SET(-2.935385E38F) ;
        p242.altitude_SET(-88130036) ;
        p242.time_usec_SET(3133540926350245024L, PH) ;
        p242.x_SET(4.793879E37F) ;
        p242.approach_z_SET(3.1152777E38F) ;
        p242.z_SET(-1.695848E38F) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == 1053892844);
            assert(pack.approach_y_GET() == 8.966948E37F);
            assert(pack.y_GET() == -1.2599012E37F);
            assert(pack.approach_x_GET() == 9.7654E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {4.268194E37F, -2.8837644E38F, 9.696974E37F, 2.1166566E36F}));
            assert(pack.latitude_GET() == -342588593);
            assert(pack.approach_z_GET() == -1.3854611E38F);
            assert(pack.z_GET() == -2.0584812E38F);
            assert(pack.altitude_GET() == 18003258);
            assert(pack.target_system_GET() == (char)140);
            assert(pack.time_usec_TRY(ph) == 189433212525830217L);
            assert(pack.x_GET() == 8.685513E37F);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.altitude_SET(18003258) ;
        p243.q_SET(new float[] {4.268194E37F, -2.8837644E38F, 9.696974E37F, 2.1166566E36F}, 0) ;
        p243.approach_z_SET(-1.3854611E38F) ;
        p243.target_system_SET((char)140) ;
        p243.time_usec_SET(189433212525830217L, PH) ;
        p243.y_SET(-1.2599012E37F) ;
        p243.approach_x_SET(9.7654E37F) ;
        p243.latitude_SET(-342588593) ;
        p243.longitude_SET(1053892844) ;
        p243.approach_y_SET(8.966948E37F) ;
        p243.z_SET(-2.0584812E38F) ;
        p243.x_SET(8.685513E37F) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)42642);
            assert(pack.interval_us_GET() == 1175613096);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)42642) ;
        p244.interval_us_SET(1175613096) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
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
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.hor_velocity_GET() == (char)23285);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS));
            assert(pack.ver_velocity_GET() == (short)12847);
            assert(pack.heading_GET() == (char)1296);
            assert(pack.ICAO_address_GET() == 3289711273L);
            assert(pack.squawk_GET() == (char)4678);
            assert(pack.callsign_LEN(ph) == 8);
            assert(pack.callsign_TRY(ph).equals("kkplfGkC"));
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHTER_AIR);
            assert(pack.tslc_GET() == (char)109);
            assert(pack.lon_GET() == 658867911);
            assert(pack.lat_GET() == 64103381);
            assert(pack.altitude_GET() == -397900205);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.altitude_SET(-397900205) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH) ;
        p246.hor_velocity_SET((char)23285) ;
        p246.lon_SET(658867911) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHTER_AIR) ;
        p246.ver_velocity_SET((short)12847) ;
        p246.ICAO_address_SET(3289711273L) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS)) ;
        p246.lat_SET(64103381) ;
        p246.tslc_SET((char)109) ;
        p246.squawk_SET((char)4678) ;
        p246.callsign_SET("kkplfGkC", PH) ;
        p246.heading_SET((char)1296) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == 3631521808L);
            assert(pack.threat_level_GET() == (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW));
            assert(pack.time_to_minimum_delta_GET() == -1.9400139E38F);
            assert(pack.altitude_minimum_delta_GET() == 8.254149E37F);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
            assert(pack.horizontal_minimum_delta_GET() == 1.1830716E38F);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.time_to_minimum_delta_SET(-1.9400139E38F) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT) ;
        p247.id_SET(3631521808L) ;
        p247.altitude_minimum_delta_SET(8.254149E37F) ;
        p247.horizontal_minimum_delta_SET(1.1830716E38F) ;
        p247.threat_level_SET((MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW)) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)204);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)145, (char)121, (char)50, (char)74, (char)32, (char)90, (char)60, (char)52, (char)245, (char)35, (char)208, (char)89, (char)207, (char)72, (char)51, (char)144, (char)105, (char)42, (char)95, (char)195, (char)225, (char)31, (char)244, (char)7, (char)31, (char)241, (char)54, (char)83, (char)232, (char)124, (char)35, (char)116, (char)238, (char)156, (char)126, (char)116, (char)240, (char)183, (char)175, (char)60, (char)140, (char)58, (char)196, (char)57, (char)47, (char)33, (char)23, (char)94, (char)66, (char)35, (char)224, (char)133, (char)189, (char)35, (char)139, (char)29, (char)133, (char)145, (char)177, (char)115, (char)23, (char)154, (char)72, (char)61, (char)143, (char)151, (char)184, (char)5, (char)153, (char)211, (char)4, (char)86, (char)118, (char)16, (char)172, (char)171, (char)134, (char)245, (char)49, (char)204, (char)56, (char)86, (char)71, (char)200, (char)1, (char)12, (char)214, (char)147, (char)90, (char)128, (char)214, (char)13, (char)169, (char)77, (char)28, (char)210, (char)202, (char)246, (char)121, (char)112, (char)212, (char)244, (char)110, (char)50, (char)31, (char)158, (char)74, (char)40, (char)19, (char)166, (char)139, (char)27, (char)75, (char)234, (char)204, (char)238, (char)59, (char)224, (char)197, (char)84, (char)240, (char)196, (char)25, (char)143, (char)143, (char)49, (char)24, (char)124, (char)214, (char)120, (char)163, (char)209, (char)230, (char)208, (char)211, (char)117, (char)141, (char)20, (char)175, (char)185, (char)52, (char)85, (char)46, (char)107, (char)161, (char)123, (char)197, (char)58, (char)220, (char)143, (char)246, (char)43, (char)231, (char)171, (char)220, (char)15, (char)147, (char)57, (char)205, (char)189, (char)61, (char)204, (char)158, (char)124, (char)69, (char)59, (char)6, (char)63, (char)122, (char)234, (char)221, (char)152, (char)33, (char)65, (char)151, (char)147, (char)173, (char)152, (char)167, (char)228, (char)152, (char)12, (char)171, (char)210, (char)28, (char)244, (char)13, (char)135, (char)108, (char)206, (char)8, (char)40, (char)28, (char)191, (char)185, (char)58, (char)80, (char)124, (char)99, (char)36, (char)176, (char)212, (char)135, (char)0, (char)205, (char)181, (char)72, (char)190, (char)194, (char)195, (char)118, (char)139, (char)46, (char)83, (char)39, (char)218, (char)152, (char)202, (char)118, (char)192, (char)53, (char)78, (char)110, (char)87, (char)103, (char)197, (char)89, (char)249, (char)102, (char)105, (char)184, (char)184, (char)132, (char)58, (char)28, (char)20, (char)129, (char)114, (char)178, (char)218, (char)213, (char)34, (char)39, (char)76, (char)74, (char)168, (char)96, (char)122, (char)75}));
            assert(pack.target_network_GET() == (char)153);
            assert(pack.target_system_GET() == (char)233);
            assert(pack.message_type_GET() == (char)42401);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.payload_SET(new char[] {(char)145, (char)121, (char)50, (char)74, (char)32, (char)90, (char)60, (char)52, (char)245, (char)35, (char)208, (char)89, (char)207, (char)72, (char)51, (char)144, (char)105, (char)42, (char)95, (char)195, (char)225, (char)31, (char)244, (char)7, (char)31, (char)241, (char)54, (char)83, (char)232, (char)124, (char)35, (char)116, (char)238, (char)156, (char)126, (char)116, (char)240, (char)183, (char)175, (char)60, (char)140, (char)58, (char)196, (char)57, (char)47, (char)33, (char)23, (char)94, (char)66, (char)35, (char)224, (char)133, (char)189, (char)35, (char)139, (char)29, (char)133, (char)145, (char)177, (char)115, (char)23, (char)154, (char)72, (char)61, (char)143, (char)151, (char)184, (char)5, (char)153, (char)211, (char)4, (char)86, (char)118, (char)16, (char)172, (char)171, (char)134, (char)245, (char)49, (char)204, (char)56, (char)86, (char)71, (char)200, (char)1, (char)12, (char)214, (char)147, (char)90, (char)128, (char)214, (char)13, (char)169, (char)77, (char)28, (char)210, (char)202, (char)246, (char)121, (char)112, (char)212, (char)244, (char)110, (char)50, (char)31, (char)158, (char)74, (char)40, (char)19, (char)166, (char)139, (char)27, (char)75, (char)234, (char)204, (char)238, (char)59, (char)224, (char)197, (char)84, (char)240, (char)196, (char)25, (char)143, (char)143, (char)49, (char)24, (char)124, (char)214, (char)120, (char)163, (char)209, (char)230, (char)208, (char)211, (char)117, (char)141, (char)20, (char)175, (char)185, (char)52, (char)85, (char)46, (char)107, (char)161, (char)123, (char)197, (char)58, (char)220, (char)143, (char)246, (char)43, (char)231, (char)171, (char)220, (char)15, (char)147, (char)57, (char)205, (char)189, (char)61, (char)204, (char)158, (char)124, (char)69, (char)59, (char)6, (char)63, (char)122, (char)234, (char)221, (char)152, (char)33, (char)65, (char)151, (char)147, (char)173, (char)152, (char)167, (char)228, (char)152, (char)12, (char)171, (char)210, (char)28, (char)244, (char)13, (char)135, (char)108, (char)206, (char)8, (char)40, (char)28, (char)191, (char)185, (char)58, (char)80, (char)124, (char)99, (char)36, (char)176, (char)212, (char)135, (char)0, (char)205, (char)181, (char)72, (char)190, (char)194, (char)195, (char)118, (char)139, (char)46, (char)83, (char)39, (char)218, (char)152, (char)202, (char)118, (char)192, (char)53, (char)78, (char)110, (char)87, (char)103, (char)197, (char)89, (char)249, (char)102, (char)105, (char)184, (char)184, (char)132, (char)58, (char)28, (char)20, (char)129, (char)114, (char)178, (char)218, (char)213, (char)34, (char)39, (char)76, (char)74, (char)168, (char)96, (char)122, (char)75}, 0) ;
        p248.message_type_SET((char)42401) ;
        p248.target_component_SET((char)204) ;
        p248.target_network_SET((char)153) ;
        p248.target_system_SET((char)233) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte) - 27, (byte)39, (byte) - 27, (byte)7, (byte) - 22, (byte) - 54, (byte) - 36, (byte) - 30, (byte)28, (byte) - 53, (byte) - 90, (byte) - 21, (byte) - 100, (byte)37, (byte)53, (byte)44, (byte) - 26, (byte) - 71, (byte)36, (byte) - 37, (byte) - 51, (byte) - 48, (byte)72, (byte) - 88, (byte) - 63, (byte) - 46, (byte) - 28, (byte) - 101, (byte)89, (byte) - 11, (byte) - 51, (byte) - 75}));
            assert(pack.ver_GET() == (char)75);
            assert(pack.address_GET() == (char)33516);
            assert(pack.type_GET() == (char)126);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.value_SET(new byte[] {(byte) - 27, (byte)39, (byte) - 27, (byte)7, (byte) - 22, (byte) - 54, (byte) - 36, (byte) - 30, (byte)28, (byte) - 53, (byte) - 90, (byte) - 21, (byte) - 100, (byte)37, (byte)53, (byte)44, (byte) - 26, (byte) - 71, (byte)36, (byte) - 37, (byte) - 51, (byte) - 48, (byte)72, (byte) - 88, (byte) - 63, (byte) - 46, (byte) - 28, (byte) - 101, (byte)89, (byte) - 11, (byte) - 51, (byte) - 75}, 0) ;
        p249.ver_SET((char)75) ;
        p249.address_SET((char)33516) ;
        p249.type_SET((char)126) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 10);
            assert(pack.name_TRY(ph).equals("xacimGCvxc"));
            assert(pack.y_GET() == 2.4014265E38F);
            assert(pack.time_usec_GET() == 789162513263106676L);
            assert(pack.z_GET() == -5.474106E37F);
            assert(pack.x_GET() == -1.3619201E38F);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.z_SET(-5.474106E37F) ;
        p250.y_SET(2.4014265E38F) ;
        p250.name_SET("xacimGCvxc", PH) ;
        p250.time_usec_SET(789162513263106676L) ;
        p250.x_SET(-1.3619201E38F) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == -1.7559985E38F);
            assert(pack.name_LEN(ph) == 10);
            assert(pack.name_TRY(ph).equals("ngsuhjNnhw"));
            assert(pack.time_boot_ms_GET() == 4248359735L);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(4248359735L) ;
        p251.value_SET(-1.7559985E38F) ;
        p251.name_SET("ngsuhjNnhw", PH) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == -301810707);
            assert(pack.name_LEN(ph) == 3);
            assert(pack.name_TRY(ph).equals("lrm"));
            assert(pack.time_boot_ms_GET() == 2741483575L);
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.value_SET(-301810707) ;
        p252.name_SET("lrm", PH) ;
        p252.time_boot_ms_SET(2741483575L) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_WARNING);
            assert(pack.text_LEN(ph) == 35);
            assert(pack.text_TRY(ph).equals("hmfhkcpxKkboylhwhBslflwKQqynigoeslf"));
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_WARNING) ;
        p253.text_SET("hmfhkcpxKkboylhwhBslflwKQqynigoeslf", PH) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == 1.4987773E37F);
            assert(pack.ind_GET() == (char)96);
            assert(pack.time_boot_ms_GET() == 366565446L);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.value_SET(1.4987773E37F) ;
        p254.ind_SET((char)96) ;
        p254.time_boot_ms_SET(366565446L) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)167);
            assert(pack.initial_timestamp_GET() == 7068219266955917097L);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)180, (char)18, (char)160, (char)181, (char)194, (char)15, (char)156, (char)200, (char)181, (char)20, (char)76, (char)79, (char)85, (char)249, (char)148, (char)99, (char)162, (char)194, (char)194, (char)195, (char)75, (char)144, (char)58, (char)97, (char)60, (char)175, (char)49, (char)12, (char)206, (char)159, (char)236, (char)159}));
            assert(pack.target_component_GET() == (char)157);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.initial_timestamp_SET(7068219266955917097L) ;
        p256.target_system_SET((char)167) ;
        p256.target_component_SET((char)157) ;
        p256.secret_key_SET(new char[] {(char)180, (char)18, (char)160, (char)181, (char)194, (char)15, (char)156, (char)200, (char)181, (char)20, (char)76, (char)79, (char)85, (char)249, (char)148, (char)99, (char)162, (char)194, (char)194, (char)195, (char)75, (char)144, (char)58, (char)97, (char)60, (char)175, (char)49, (char)12, (char)206, (char)159, (char)236, (char)159}, 0) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.last_change_ms_GET() == 3214790519L);
            assert(pack.state_GET() == (char)162);
            assert(pack.time_boot_ms_GET() == 1936326871L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(1936326871L) ;
        p257.state_SET((char)162) ;
        p257.last_change_ms_SET(3214790519L) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)37);
            assert(pack.target_component_GET() == (char)193);
            assert(pack.tune_LEN(ph) == 1);
            assert(pack.tune_TRY(ph).equals("f"));
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)37) ;
        p258.target_component_SET((char)193) ;
        p258.tune_SET("f", PH) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.cam_definition_version_GET() == (char)30812);
            assert(pack.sensor_size_h_GET() == -2.7619778E38F);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)147, (char)150, (char)118, (char)129, (char)153, (char)236, (char)70, (char)104, (char)90, (char)145, (char)216, (char)218, (char)63, (char)198, (char)224, (char)108, (char)111, (char)4, (char)193, (char)9, (char)201, (char)193, (char)241, (char)205, (char)28, (char)117, (char)161, (char)148, (char)200, (char)23, (char)132, (char)70}));
            assert(pack.cam_definition_uri_LEN(ph) == 74);
            assert(pack.cam_definition_uri_TRY(ph).equals("dkzrixduipXfNSwUsrmwmuugglbhysexknryxjwwslrzywsdpevgtesjplMfMxbpgcjdracjku"));
            assert(pack.time_boot_ms_GET() == 2492735093L);
            assert(pack.resolution_v_GET() == (char)14823);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)156, (char)72, (char)66, (char)71, (char)142, (char)113, (char)109, (char)134, (char)219, (char)113, (char)232, (char)245, (char)79, (char)125, (char)38, (char)54, (char)144, (char)27, (char)44, (char)149, (char)31, (char)33, (char)253, (char)60, (char)205, (char)37, (char)107, (char)165, (char)81, (char)131, (char)25, (char)198}));
            assert(pack.resolution_h_GET() == (char)21951);
            assert(pack.firmware_version_GET() == 4287913846L);
            assert(pack.lens_id_GET() == (char)29);
            assert(pack.sensor_size_v_GET() == 2.8903991E38F);
            assert(pack.focal_length_GET() == -3.232872E38F);
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE));
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.model_name_SET(new char[] {(char)156, (char)72, (char)66, (char)71, (char)142, (char)113, (char)109, (char)134, (char)219, (char)113, (char)232, (char)245, (char)79, (char)125, (char)38, (char)54, (char)144, (char)27, (char)44, (char)149, (char)31, (char)33, (char)253, (char)60, (char)205, (char)37, (char)107, (char)165, (char)81, (char)131, (char)25, (char)198}, 0) ;
        p259.vendor_name_SET(new char[] {(char)147, (char)150, (char)118, (char)129, (char)153, (char)236, (char)70, (char)104, (char)90, (char)145, (char)216, (char)218, (char)63, (char)198, (char)224, (char)108, (char)111, (char)4, (char)193, (char)9, (char)201, (char)193, (char)241, (char)205, (char)28, (char)117, (char)161, (char)148, (char)200, (char)23, (char)132, (char)70}, 0) ;
        p259.cam_definition_version_SET((char)30812) ;
        p259.time_boot_ms_SET(2492735093L) ;
        p259.cam_definition_uri_SET("dkzrixduipXfNSwUsrmwmuugglbhysexknryxjwwslrzywsdpevgtesjplMfMxbpgcjdracjku", PH) ;
        p259.sensor_size_v_SET(2.8903991E38F) ;
        p259.lens_id_SET((char)29) ;
        p259.firmware_version_SET(4287913846L) ;
        p259.resolution_v_SET((char)14823) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE)) ;
        p259.sensor_size_h_SET(-2.7619778E38F) ;
        p259.focal_length_SET(-3.232872E38F) ;
        p259.resolution_h_SET((char)21951) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2099411591L);
            assert(pack.mode_id_GET() == (CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY |
                                          CAMERA_MODE.CAMERA_MODE_VIDEO));
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(2099411591L) ;
        p260.mode_id_SET((CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY |
                          CAMERA_MODE.CAMERA_MODE_VIDEO)) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.write_speed_GET() == 1.2544229E38F);
            assert(pack.available_capacity_GET() == -2.7461675E38F);
            assert(pack.read_speed_GET() == 1.3486943E38F);
            assert(pack.storage_count_GET() == (char)176);
            assert(pack.status_GET() == (char)116);
            assert(pack.used_capacity_GET() == -1.983578E38F);
            assert(pack.total_capacity_GET() == -4.5041583E37F);
            assert(pack.time_boot_ms_GET() == 1199668179L);
            assert(pack.storage_id_GET() == (char)31);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.write_speed_SET(1.2544229E38F) ;
        p261.time_boot_ms_SET(1199668179L) ;
        p261.total_capacity_SET(-4.5041583E37F) ;
        p261.status_SET((char)116) ;
        p261.available_capacity_SET(-2.7461675E38F) ;
        p261.storage_count_SET((char)176) ;
        p261.storage_id_SET((char)31) ;
        p261.read_speed_SET(1.3486943E38F) ;
        p261.used_capacity_SET(-1.983578E38F) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2707547415L);
            assert(pack.available_capacity_GET() == -2.1651886E38F);
            assert(pack.image_status_GET() == (char)75);
            assert(pack.image_interval_GET() == 1.5429272E38F);
            assert(pack.recording_time_ms_GET() == 1026697066L);
            assert(pack.video_status_GET() == (char)154);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.video_status_SET((char)154) ;
        p262.image_interval_SET(1.5429272E38F) ;
        p262.recording_time_ms_SET(1026697066L) ;
        p262.available_capacity_SET(-2.1651886E38F) ;
        p262.time_boot_ms_SET(2707547415L) ;
        p262.image_status_SET((char)75) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.file_url_LEN(ph) == 7);
            assert(pack.file_url_TRY(ph).equals("kxhdizf"));
            assert(pack.time_boot_ms_GET() == 2366211354L);
            assert(pack.image_index_GET() == -847622219);
            assert(pack.capture_result_GET() == (byte) - 80);
            assert(pack.relative_alt_GET() == 2021269495);
            assert(pack.alt_GET() == -1952002471);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.6019625E38F, -3.2195208E38F, -2.1733348E38F, 6.69259E37F}));
            assert(pack.lat_GET() == 1084730427);
            assert(pack.camera_id_GET() == (char)148);
            assert(pack.time_utc_GET() == 4465448959944521840L);
            assert(pack.lon_GET() == 425165989);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.time_boot_ms_SET(2366211354L) ;
        p263.camera_id_SET((char)148) ;
        p263.image_index_SET(-847622219) ;
        p263.relative_alt_SET(2021269495) ;
        p263.alt_SET(-1952002471) ;
        p263.q_SET(new float[] {-1.6019625E38F, -3.2195208E38F, -2.1733348E38F, 6.69259E37F}, 0) ;
        p263.lat_SET(1084730427) ;
        p263.file_url_SET("kxhdizf", PH) ;
        p263.lon_SET(425165989) ;
        p263.capture_result_SET((byte) - 80) ;
        p263.time_utc_SET(4465448959944521840L) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.flight_uuid_GET() == 8339952715493398415L);
            assert(pack.arming_time_utc_GET() == 1699369231647338985L);
            assert(pack.time_boot_ms_GET() == 3483692701L);
            assert(pack.takeoff_time_utc_GET() == 7515107748121684035L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.arming_time_utc_SET(1699369231647338985L) ;
        p264.takeoff_time_utc_SET(7515107748121684035L) ;
        p264.flight_uuid_SET(8339952715493398415L) ;
        p264.time_boot_ms_SET(3483692701L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3556939485L);
            assert(pack.yaw_GET() == 2.7802255E38F);
            assert(pack.pitch_GET() == 4.6120186E37F);
            assert(pack.roll_GET() == -1.6108506E38F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(3556939485L) ;
        p265.roll_SET(-1.6108506E38F) ;
        p265.pitch_SET(4.6120186E37F) ;
        p265.yaw_SET(2.7802255E38F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)102);
            assert(pack.length_GET() == (char)190);
            assert(pack.target_component_GET() == (char)215);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)208, (char)38, (char)88, (char)128, (char)208, (char)6, (char)14, (char)90, (char)104, (char)175, (char)122, (char)128, (char)34, (char)47, (char)53, (char)81, (char)135, (char)47, (char)174, (char)156, (char)199, (char)228, (char)250, (char)172, (char)113, (char)1, (char)9, (char)114, (char)217, (char)121, (char)3, (char)16, (char)180, (char)163, (char)100, (char)32, (char)129, (char)185, (char)39, (char)151, (char)41, (char)86, (char)211, (char)179, (char)136, (char)245, (char)4, (char)41, (char)173, (char)208, (char)175, (char)91, (char)55, (char)32, (char)229, (char)242, (char)184, (char)150, (char)119, (char)13, (char)158, (char)123, (char)52, (char)70, (char)157, (char)135, (char)245, (char)231, (char)55, (char)2, (char)217, (char)71, (char)148, (char)216, (char)255, (char)69, (char)25, (char)44, (char)22, (char)206, (char)73, (char)215, (char)183, (char)14, (char)177, (char)149, (char)251, (char)211, (char)48, (char)222, (char)197, (char)226, (char)111, (char)137, (char)86, (char)53, (char)246, (char)17, (char)26, (char)205, (char)174, (char)30, (char)252, (char)222, (char)196, (char)6, (char)75, (char)142, (char)215, (char)127, (char)116, (char)60, (char)170, (char)63, (char)117, (char)244, (char)24, (char)176, (char)157, (char)154, (char)75, (char)146, (char)234, (char)169, (char)141, (char)113, (char)237, (char)164, (char)116, (char)89, (char)107, (char)78, (char)182, (char)75, (char)162, (char)89, (char)10, (char)104, (char)36, (char)157, (char)123, (char)237, (char)10, (char)182, (char)20, (char)200, (char)200, (char)245, (char)143, (char)215, (char)171, (char)16, (char)75, (char)83, (char)40, (char)105, (char)68, (char)156, (char)94, (char)13, (char)92, (char)50, (char)0, (char)185, (char)62, (char)117, (char)215, (char)236, (char)171, (char)117, (char)226, (char)107, (char)138, (char)208, (char)236, (char)208, (char)200, (char)125, (char)3, (char)119, (char)157, (char)170, (char)254, (char)142, (char)44, (char)149, (char)248, (char)97, (char)148, (char)53, (char)188, (char)79, (char)75, (char)119, (char)255, (char)243, (char)151, (char)195, (char)45, (char)181, (char)20, (char)101, (char)209, (char)93, (char)49, (char)182, (char)74, (char)108, (char)23, (char)113, (char)218, (char)232, (char)191, (char)77, (char)179, (char)202, (char)79, (char)36, (char)36, (char)49, (char)53, (char)147, (char)17, (char)81, (char)140, (char)79, (char)8, (char)205, (char)169, (char)171, (char)8, (char)178, (char)35, (char)176, (char)150, (char)217, (char)93, (char)156, (char)255, (char)134, (char)230, (char)190, (char)64, (char)5, (char)244, (char)144, (char)205, (char)56, (char)151}));
            assert(pack.sequence_GET() == (char)43856);
            assert(pack.first_message_offset_GET() == (char)48);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_component_SET((char)215) ;
        p266.data__SET(new char[] {(char)208, (char)38, (char)88, (char)128, (char)208, (char)6, (char)14, (char)90, (char)104, (char)175, (char)122, (char)128, (char)34, (char)47, (char)53, (char)81, (char)135, (char)47, (char)174, (char)156, (char)199, (char)228, (char)250, (char)172, (char)113, (char)1, (char)9, (char)114, (char)217, (char)121, (char)3, (char)16, (char)180, (char)163, (char)100, (char)32, (char)129, (char)185, (char)39, (char)151, (char)41, (char)86, (char)211, (char)179, (char)136, (char)245, (char)4, (char)41, (char)173, (char)208, (char)175, (char)91, (char)55, (char)32, (char)229, (char)242, (char)184, (char)150, (char)119, (char)13, (char)158, (char)123, (char)52, (char)70, (char)157, (char)135, (char)245, (char)231, (char)55, (char)2, (char)217, (char)71, (char)148, (char)216, (char)255, (char)69, (char)25, (char)44, (char)22, (char)206, (char)73, (char)215, (char)183, (char)14, (char)177, (char)149, (char)251, (char)211, (char)48, (char)222, (char)197, (char)226, (char)111, (char)137, (char)86, (char)53, (char)246, (char)17, (char)26, (char)205, (char)174, (char)30, (char)252, (char)222, (char)196, (char)6, (char)75, (char)142, (char)215, (char)127, (char)116, (char)60, (char)170, (char)63, (char)117, (char)244, (char)24, (char)176, (char)157, (char)154, (char)75, (char)146, (char)234, (char)169, (char)141, (char)113, (char)237, (char)164, (char)116, (char)89, (char)107, (char)78, (char)182, (char)75, (char)162, (char)89, (char)10, (char)104, (char)36, (char)157, (char)123, (char)237, (char)10, (char)182, (char)20, (char)200, (char)200, (char)245, (char)143, (char)215, (char)171, (char)16, (char)75, (char)83, (char)40, (char)105, (char)68, (char)156, (char)94, (char)13, (char)92, (char)50, (char)0, (char)185, (char)62, (char)117, (char)215, (char)236, (char)171, (char)117, (char)226, (char)107, (char)138, (char)208, (char)236, (char)208, (char)200, (char)125, (char)3, (char)119, (char)157, (char)170, (char)254, (char)142, (char)44, (char)149, (char)248, (char)97, (char)148, (char)53, (char)188, (char)79, (char)75, (char)119, (char)255, (char)243, (char)151, (char)195, (char)45, (char)181, (char)20, (char)101, (char)209, (char)93, (char)49, (char)182, (char)74, (char)108, (char)23, (char)113, (char)218, (char)232, (char)191, (char)77, (char)179, (char)202, (char)79, (char)36, (char)36, (char)49, (char)53, (char)147, (char)17, (char)81, (char)140, (char)79, (char)8, (char)205, (char)169, (char)171, (char)8, (char)178, (char)35, (char)176, (char)150, (char)217, (char)93, (char)156, (char)255, (char)134, (char)230, (char)190, (char)64, (char)5, (char)244, (char)144, (char)205, (char)56, (char)151}, 0) ;
        p266.target_system_SET((char)102) ;
        p266.first_message_offset_SET((char)48) ;
        p266.sequence_SET((char)43856) ;
        p266.length_SET((char)190) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)120);
            assert(pack.sequence_GET() == (char)9460);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)180, (char)127, (char)22, (char)74, (char)212, (char)92, (char)165, (char)112, (char)203, (char)155, (char)119, (char)25, (char)114, (char)185, (char)167, (char)144, (char)190, (char)169, (char)28, (char)69, (char)200, (char)1, (char)29, (char)196, (char)237, (char)34, (char)194, (char)184, (char)183, (char)104, (char)247, (char)34, (char)241, (char)79, (char)75, (char)251, (char)129, (char)170, (char)65, (char)7, (char)199, (char)203, (char)9, (char)76, (char)19, (char)167, (char)45, (char)192, (char)254, (char)100, (char)142, (char)72, (char)252, (char)201, (char)39, (char)13, (char)195, (char)207, (char)179, (char)179, (char)0, (char)41, (char)127, (char)188, (char)155, (char)226, (char)209, (char)23, (char)217, (char)55, (char)38, (char)5, (char)70, (char)142, (char)216, (char)173, (char)147, (char)230, (char)69, (char)179, (char)108, (char)212, (char)212, (char)100, (char)198, (char)127, (char)186, (char)46, (char)173, (char)37, (char)246, (char)252, (char)100, (char)30, (char)97, (char)31, (char)112, (char)141, (char)190, (char)148, (char)221, (char)157, (char)34, (char)69, (char)200, (char)43, (char)248, (char)86, (char)70, (char)231, (char)149, (char)147, (char)76, (char)5, (char)45, (char)64, (char)52, (char)127, (char)158, (char)106, (char)162, (char)48, (char)216, (char)16, (char)99, (char)212, (char)0, (char)163, (char)220, (char)183, (char)25, (char)176, (char)151, (char)25, (char)171, (char)220, (char)240, (char)202, (char)226, (char)228, (char)15, (char)227, (char)165, (char)119, (char)126, (char)61, (char)148, (char)216, (char)0, (char)196, (char)33, (char)102, (char)74, (char)78, (char)93, (char)181, (char)192, (char)33, (char)44, (char)254, (char)81, (char)152, (char)197, (char)60, (char)54, (char)16, (char)108, (char)75, (char)7, (char)122, (char)23, (char)239, (char)237, (char)167, (char)137, (char)99, (char)14, (char)40, (char)105, (char)197, (char)67, (char)159, (char)8, (char)176, (char)118, (char)180, (char)85, (char)103, (char)110, (char)153, (char)88, (char)180, (char)189, (char)93, (char)129, (char)21, (char)64, (char)223, (char)203, (char)187, (char)43, (char)105, (char)163, (char)152, (char)120, (char)94, (char)81, (char)47, (char)235, (char)93, (char)67, (char)213, (char)78, (char)1, (char)117, (char)159, (char)53, (char)153, (char)219, (char)179, (char)155, (char)136, (char)79, (char)19, (char)229, (char)109, (char)115, (char)238, (char)25, (char)150, (char)21, (char)180, (char)178, (char)156, (char)23, (char)75, (char)93, (char)121, (char)189, (char)26, (char)147, (char)250, (char)52, (char)104, (char)57, (char)90, (char)59, (char)14, (char)3}));
            assert(pack.first_message_offset_GET() == (char)204);
            assert(pack.target_component_GET() == (char)41);
            assert(pack.length_GET() == (char)97);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_component_SET((char)41) ;
        p267.length_SET((char)97) ;
        p267.first_message_offset_SET((char)204) ;
        p267.target_system_SET((char)120) ;
        p267.sequence_SET((char)9460) ;
        p267.data__SET(new char[] {(char)180, (char)127, (char)22, (char)74, (char)212, (char)92, (char)165, (char)112, (char)203, (char)155, (char)119, (char)25, (char)114, (char)185, (char)167, (char)144, (char)190, (char)169, (char)28, (char)69, (char)200, (char)1, (char)29, (char)196, (char)237, (char)34, (char)194, (char)184, (char)183, (char)104, (char)247, (char)34, (char)241, (char)79, (char)75, (char)251, (char)129, (char)170, (char)65, (char)7, (char)199, (char)203, (char)9, (char)76, (char)19, (char)167, (char)45, (char)192, (char)254, (char)100, (char)142, (char)72, (char)252, (char)201, (char)39, (char)13, (char)195, (char)207, (char)179, (char)179, (char)0, (char)41, (char)127, (char)188, (char)155, (char)226, (char)209, (char)23, (char)217, (char)55, (char)38, (char)5, (char)70, (char)142, (char)216, (char)173, (char)147, (char)230, (char)69, (char)179, (char)108, (char)212, (char)212, (char)100, (char)198, (char)127, (char)186, (char)46, (char)173, (char)37, (char)246, (char)252, (char)100, (char)30, (char)97, (char)31, (char)112, (char)141, (char)190, (char)148, (char)221, (char)157, (char)34, (char)69, (char)200, (char)43, (char)248, (char)86, (char)70, (char)231, (char)149, (char)147, (char)76, (char)5, (char)45, (char)64, (char)52, (char)127, (char)158, (char)106, (char)162, (char)48, (char)216, (char)16, (char)99, (char)212, (char)0, (char)163, (char)220, (char)183, (char)25, (char)176, (char)151, (char)25, (char)171, (char)220, (char)240, (char)202, (char)226, (char)228, (char)15, (char)227, (char)165, (char)119, (char)126, (char)61, (char)148, (char)216, (char)0, (char)196, (char)33, (char)102, (char)74, (char)78, (char)93, (char)181, (char)192, (char)33, (char)44, (char)254, (char)81, (char)152, (char)197, (char)60, (char)54, (char)16, (char)108, (char)75, (char)7, (char)122, (char)23, (char)239, (char)237, (char)167, (char)137, (char)99, (char)14, (char)40, (char)105, (char)197, (char)67, (char)159, (char)8, (char)176, (char)118, (char)180, (char)85, (char)103, (char)110, (char)153, (char)88, (char)180, (char)189, (char)93, (char)129, (char)21, (char)64, (char)223, (char)203, (char)187, (char)43, (char)105, (char)163, (char)152, (char)120, (char)94, (char)81, (char)47, (char)235, (char)93, (char)67, (char)213, (char)78, (char)1, (char)117, (char)159, (char)53, (char)153, (char)219, (char)179, (char)155, (char)136, (char)79, (char)19, (char)229, (char)109, (char)115, (char)238, (char)25, (char)150, (char)21, (char)180, (char)178, (char)156, (char)23, (char)75, (char)93, (char)121, (char)189, (char)26, (char)147, (char)250, (char)52, (char)104, (char)57, (char)90, (char)59, (char)14, (char)3}, 0) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)44);
            assert(pack.target_component_GET() == (char)222);
            assert(pack.sequence_GET() == (char)32164);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.sequence_SET((char)32164) ;
        p268.target_system_SET((char)44) ;
        p268.target_component_SET((char)222) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.bitrate_GET() == 2494231866L);
            assert(pack.uri_LEN(ph) == 68);
            assert(pack.uri_TRY(ph).equals("lwMqrbzvPnjrjsNmvlpsRdsmvojgeyTvZaRjtayqhrysuiqpfzkwpjzYkewzjhhufkyr"));
            assert(pack.resolution_v_GET() == (char)7797);
            assert(pack.status_GET() == (char)196);
            assert(pack.rotation_GET() == (char)38020);
            assert(pack.camera_id_GET() == (char)187);
            assert(pack.resolution_h_GET() == (char)27037);
            assert(pack.framerate_GET() == -3.0702029E37F);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.bitrate_SET(2494231866L) ;
        p269.resolution_v_SET((char)7797) ;
        p269.framerate_SET(-3.0702029E37F) ;
        p269.resolution_h_SET((char)27037) ;
        p269.status_SET((char)196) ;
        p269.rotation_SET((char)38020) ;
        p269.uri_SET("lwMqrbzvPnjrjsNmvlpsRdsmvojgeyTvZaRjtayqhrysuiqpfzkwpjzYkewzjhhufkyr", PH) ;
        p269.camera_id_SET((char)187) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.rotation_GET() == (char)19534);
            assert(pack.target_component_GET() == (char)225);
            assert(pack.resolution_h_GET() == (char)24173);
            assert(pack.camera_id_GET() == (char)130);
            assert(pack.framerate_GET() == -3.2841036E38F);
            assert(pack.uri_LEN(ph) == 139);
            assert(pack.uri_TRY(ph).equals("nZqhvcfnhvwyujzkrtytzdndifzrvujcsksjjzbcVscdbcDubFrqdssEajzgajppjxogyiosfeYfxywouDtfsxqpmraojgmplzlxokvIxbvudosxupbxLkzzhsopTnrpqjbaylouaja"));
            assert(pack.bitrate_GET() == 4175094110L);
            assert(pack.resolution_v_GET() == (char)16076);
            assert(pack.target_system_GET() == (char)216);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)216) ;
        p270.resolution_h_SET((char)24173) ;
        p270.rotation_SET((char)19534) ;
        p270.framerate_SET(-3.2841036E38F) ;
        p270.bitrate_SET(4175094110L) ;
        p270.uri_SET("nZqhvcfnhvwyujzkrtytzdndifzrvujcsksjjzbcVscdbcDubFrqdssEajzgajppjxogyiosfeYfxywouDtfsxqpmraojgmplzlxokvIxbvudosxupbxLkzzhsopTnrpqjbaylouaja", PH) ;
        p270.camera_id_SET((char)130) ;
        p270.resolution_v_SET((char)16076) ;
        p270.target_component_SET((char)225) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.password_LEN(ph) == 34);
            assert(pack.password_TRY(ph).equals("wqxtdypqgatpisgsesqjlfmtiftcjbzhgf"));
            assert(pack.ssid_LEN(ph) == 23);
            assert(pack.ssid_TRY(ph).equals("scKGkeNhhlgwktrvHrnOcpB"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("scKGkeNhhlgwktrvHrnOcpB", PH) ;
        p299.password_SET("wqxtdypqgatpisgsesqjlfmtiftcjbzhgf", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)255, (char)117, (char)150, (char)172, (char)181, (char)99, (char)203, (char)248}));
            assert(pack.version_GET() == (char)47590);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)129, (char)253, (char)204, (char)19, (char)89, (char)219, (char)200, (char)199}));
            assert(pack.min_version_GET() == (char)47822);
            assert(pack.max_version_GET() == (char)33843);
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)47590) ;
        p300.min_version_SET((char)47822) ;
        p300.library_version_hash_SET(new char[] {(char)255, (char)117, (char)150, (char)172, (char)181, (char)99, (char)203, (char)248}, 0) ;
        p300.spec_version_hash_SET(new char[] {(char)129, (char)253, (char)204, (char)19, (char)89, (char)219, (char)200, (char)199}, 0) ;
        p300.max_version_SET((char)33843) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.sub_mode_GET() == (char)194);
            assert(pack.vendor_specific_status_code_GET() == (char)59319);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
            assert(pack.time_usec_GET() == 6987881515711333535L);
            assert(pack.uptime_sec_GET() == 2558087654L);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.uptime_sec_SET(2558087654L) ;
        p310.time_usec_SET(6987881515711333535L) ;
        p310.vendor_specific_status_code_SET((char)59319) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL) ;
        p310.sub_mode_SET((char)194) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.sw_version_minor_GET() == (char)126);
            assert(pack.time_usec_GET() == 1058634558775003184L);
            assert(pack.hw_version_minor_GET() == (char)15);
            assert(pack.hw_version_major_GET() == (char)111);
            assert(pack.uptime_sec_GET() == 680471021L);
            assert(pack.sw_version_major_GET() == (char)211);
            assert(pack.sw_vcs_commit_GET() == 2958812634L);
            assert(pack.name_LEN(ph) == 2);
            assert(pack.name_TRY(ph).equals("lv"));
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)13, (char)172, (char)201, (char)137, (char)73, (char)196, (char)19, (char)184, (char)65, (char)41, (char)9, (char)155, (char)128, (char)236, (char)86, (char)73}));
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.hw_unique_id_SET(new char[] {(char)13, (char)172, (char)201, (char)137, (char)73, (char)196, (char)19, (char)184, (char)65, (char)41, (char)9, (char)155, (char)128, (char)236, (char)86, (char)73}, 0) ;
        p311.sw_version_major_SET((char)211) ;
        p311.hw_version_major_SET((char)111) ;
        p311.time_usec_SET(1058634558775003184L) ;
        p311.name_SET("lv", PH) ;
        p311.uptime_sec_SET(680471021L) ;
        p311.sw_version_minor_SET((char)126) ;
        p311.sw_vcs_commit_SET(2958812634L) ;
        p311.hw_version_minor_SET((char)15) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)105);
            assert(pack.target_component_GET() == (char)57);
            assert(pack.param_index_GET() == (short)27108);
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("fehmuKvbwjdf"));
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)105) ;
        p320.param_index_SET((short)27108) ;
        p320.param_id_SET("fehmuKvbwjdf", PH) ;
        p320.target_component_SET((char)57) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)37);
            assert(pack.target_component_GET() == (char)225);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)225) ;
        p321.target_system_SET((char)37) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (char)57978);
            assert(pack.param_id_LEN(ph) == 13);
            assert(pack.param_id_TRY(ph).equals("dLujgdhiqneyy"));
            assert(pack.param_count_GET() == (char)20181);
            assert(pack.param_value_LEN(ph) == 86);
            assert(pack.param_value_TRY(ph).equals("awapZkkuesshwYipgweuncvhmpwmkcdqendqxhnlSTtgikbyyuzbiswoeivpnPzkowOylwhbircdpvkbtbgsXb"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("dLujgdhiqneyy", PH) ;
        p322.param_value_SET("awapZkkuesshwYipgweuncvhmpwmkcdqendqxhnlSTtgikbyyuzbiswoeivpnPzkowOylwhbircdpvkbtbgsXb", PH) ;
        p322.param_index_SET((char)57978) ;
        p322.param_count_SET((char)20181) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)83);
            assert(pack.target_system_GET() == (char)91);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("wn"));
            assert(pack.param_value_LEN(ph) == 105);
            assert(pack.param_value_TRY(ph).equals("rqTkilnxptTcutNisiqlckwxgltXflbhulknlimemoZinoVvbvtgkHcdfDtlgyqmxsjUsUrZraqdpuctpimapsnImpulzblhbtsjIjzkr"));
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64) ;
        p323.param_value_SET("rqTkilnxptTcutNisiqlckwxgltXflbhulknlimemoZinoVvbvtgkHcdfDtlgyqmxsjUsUrZraqdpuctpimapsnImpulzblhbtsjIjzkr", PH) ;
        p323.param_id_SET("wn", PH) ;
        p323.target_system_SET((char)91) ;
        p323.target_component_SET((char)83) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
            assert(pack.param_id_LEN(ph) == 7);
            assert(pack.param_id_TRY(ph).equals("gjdpepf"));
            assert(pack.param_value_LEN(ph) == 56);
            assert(pack.param_value_TRY(ph).equals("dakaerwixueDerkqiogDTyPhumlcszZfVzCxkFclfEbcmAAwwpsxzXzb"));
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64) ;
        p324.param_value_SET("dakaerwixueDerkqiogDTyPhumlcszZfVzCxkFclfEbcmAAwwpsxzXzb", PH) ;
        p324.param_id_SET("gjdpepf", PH) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.max_distance_GET() == (char)14194);
            assert(pack.time_usec_GET() == 8210045589391199645L);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)65010, (char)49805, (char)46629, (char)25731, (char)6317, (char)52077, (char)45252, (char)29961, (char)10070, (char)16879, (char)1738, (char)35027, (char)7462, (char)59543, (char)20595, (char)59512, (char)38242, (char)26542, (char)51828, (char)51034, (char)5128, (char)57752, (char)28872, (char)10375, (char)64805, (char)1700, (char)5084, (char)15135, (char)57576, (char)3289, (char)53133, (char)606, (char)63802, (char)28276, (char)36415, (char)54270, (char)19839, (char)25268, (char)40100, (char)44456, (char)21986, (char)62634, (char)10790, (char)57270, (char)41683, (char)28792, (char)6018, (char)29816, (char)3, (char)47707, (char)48692, (char)44158, (char)12483, (char)24656, (char)48284, (char)54115, (char)25381, (char)5855, (char)5209, (char)48228, (char)17731, (char)56770, (char)16623, (char)38550, (char)6477, (char)23623, (char)21775, (char)53899, (char)43576, (char)23874, (char)35653, (char)2142}));
            assert(pack.increment_GET() == (char)34);
            assert(pack.min_distance_GET() == (char)13129);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.time_usec_SET(8210045589391199645L) ;
        p330.max_distance_SET((char)14194) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR) ;
        p330.min_distance_SET((char)13129) ;
        p330.distances_SET(new char[] {(char)65010, (char)49805, (char)46629, (char)25731, (char)6317, (char)52077, (char)45252, (char)29961, (char)10070, (char)16879, (char)1738, (char)35027, (char)7462, (char)59543, (char)20595, (char)59512, (char)38242, (char)26542, (char)51828, (char)51034, (char)5128, (char)57752, (char)28872, (char)10375, (char)64805, (char)1700, (char)5084, (char)15135, (char)57576, (char)3289, (char)53133, (char)606, (char)63802, (char)28276, (char)36415, (char)54270, (char)19839, (char)25268, (char)40100, (char)44456, (char)21986, (char)62634, (char)10790, (char)57270, (char)41683, (char)28792, (char)6018, (char)29816, (char)3, (char)47707, (char)48692, (char)44158, (char)12483, (char)24656, (char)48284, (char)54115, (char)25381, (char)5855, (char)5209, (char)48228, (char)17731, (char)56770, (char)16623, (char)38550, (char)6477, (char)23623, (char)21775, (char)53899, (char)43576, (char)23874, (char)35653, (char)2142}, 0) ;
        p330.increment_SET((char)34) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
    }

}